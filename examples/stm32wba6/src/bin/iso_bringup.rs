#![no_std]
#![no_main]

//! Peer-free ISO bring-up probe for the STM32WBA Full link layer.
//!
//! Build and run with:
//!
//! ```text
//! cargo run --release --no-default-features --features stm32wba65ri-iso --bin iso_bringup
//! ```
//!
//! `--no-default-features` is required because the Basic (default) and Full link
//! layer images cannot be linked into the same binary.
//!
//! The Full link layer is what ST's own BLE Audio applications use
//! (`LinkLayer_BLE_Full_V3_0.a`). Before writing any LE Audio code we need to know
//! whether that image, built for the link-layer-only (`llo`) configuration, actually
//! exposes isochronous channels to a Rust host. This probe answers the two questions
//! that gate everything else:
//!
//!   1. Does the controller advertise the LE isochronous-channel feature bits?
//!   2. Does it report dedicated ISO data buffers via `LE Read Buffer Size v2`?
//!
//! The next step is to extend this with `LE Set CIG Parameters`, which allocates a
//! CIG/CIS in the controller and needs no peer device.

use bt_hci::cmd::le::{LeReadBufferSizeV2, LeReadLocalSupportedFeatures, LeRemoveCig};
use bt_hci::controller::ControllerCmdSync;
use bt_hci::param::CigId;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::aes::{self, Aes};
use embassy_stm32::peripherals::{AES, PKA, RNG};
use embassy_stm32::pka::{self, Pka};
use embassy_stm32::rcc;
use embassy_stm32::rng::{self, Rng};
use embassy_stm32::{Config, bind_interrupts};
use embassy_stm32_wpan::controller::ControllerAdapter;
use embassy_stm32_wpan::{Controller as WbaController, HighInterruptHandler, LowInterruptHandler, Platform, new_platform};
use embassy_time::Timer;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    AES => aes::InterruptHandler<AES>;
    PKA => pka::InterruptHandler<PKA>;
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
});

/// BLE runner task - drives the BLE stack sequencer
#[embassy_executor::task]
async fn ble_runner_task(platform: &'static Platform) {
    platform.run_ble().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = rcc::Config::new_wpan();

    let p = embassy_stm32::init(config);
    info!("STM32WBA6 ISO bring-up probe (Full link layer)");

    // Initialize hardware peripherals required by BLE stack
    let (platform, runtime) = new_platform!(
        Rng::new(p.RNG, Irqs),
        Pka::new(p.PKA, Irqs),
        Aes::new_blocking(p.AES, Irqs),
        8
    );

    // Spawn the BLE runner task (required for proper BLE operation)
    spawner.spawn(ble_runner_task(platform).expect("Failed to spawn BLE runner"));

    // Initialize BLE stack
    let ble = WbaController::new(platform, runtime, Irqs)
        .await
        .expect("BLE initialization failed");

    info!("BLE stack initialized");

    let controller = ControllerAdapter::new(ble);

    probe(controller).await;
}

async fn probe<C>(controller: C)
where
    C: Controller
        + ControllerCmdSync<LeReadLocalSupportedFeatures>
        + ControllerCmdSync<LeReadBufferSizeV2>
        + ControllerCmdSync<LeRemoveCig>,
{
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();

    let mut runner = stack.runner();
    let iso = stack.iso();

    let task = async {
        Timer::after_secs(1).await;

        // 1. Isochronous channel capability bits.
        match iso.command(LeReadLocalSupportedFeatures::new()).await {
            Ok(f) => info!(
                "LE features: cis_central={} cis_peripheral={} iso_broadcaster={} sync_receiver={} cis={}",
                f.supports_connected_isochronous_stream_central(),
                f.supports_connected_isochronous_stream_peripheral(),
                f.supports_isochronous_broadcaster(),
                f.supports_synchronized_receiver(),
                f.supports_connected_isochronous_stream(),
            ),
            Err(e) => {
                let e = Debug2Format(&e);
                error!("LE Read Local Supported Features failed: {:?}", e);
            }
        }

        // 2. Dedicated ISO data buffers.
        match iso.command(LeReadBufferSizeV2::new()).await {
            Ok(b) => {
                // `LeReadBufferSizeV2Return` is a packed struct, so copy the fields out
                // before handing them to defmt (which would otherwise take a reference).
                let acl_len = b.le_acl_data_packet_length;
                let acl_num = b.total_num_le_acl_data_packets;
                let iso_len = b.iso_data_packet_length;
                let iso_num = b.total_num_iso_data_packets;
                info!(
                    "buffers: acl_len={} acl_num={} iso_len={} iso_num={}",
                    acl_len, acl_num, iso_len, iso_num
                );
            }
            Err(e) => {
                let e = Debug2Format(&e);
                error!("LE Read Buffer Size v2 failed: {:?}", e);
            }
        }

        // 3. Is the ISO command surface implemented at all?
        //
        // `LE Remove CIG` for a CIG that was never allocated is a cheap probe: a
        // controller implementing the ISO command set answers "Unknown Connection
        // Identifier" (0x02), while one that does not know the opcode answers
        // "Unknown HCI Command" (0x01).
        match iso.command(LeRemoveCig::new(CigId::new(0))).await {
            Ok(r) => {
                let cig_id = r.cig_id.into_inner();
                info!("LE Remove CIG(0): accepted, cig_id={}", cig_id);
            }
            Err(e) => {
                let e = Debug2Format(&e);
                info!("LE Remove CIG(0): rejected: {:?}", e);
            }
        }

        loop {
            Timer::after_secs(5).await;
            info!("probe alive");
        }
    };

    join(runner.run(), task).await;
}
