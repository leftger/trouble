//! Minimal ISO sink: accept a CIS, set up the data path, log what arrives.
//!
//! This is deliberately not BAP. It exists to answer one question — whether the
//! STM32WBA Full link layer carries ISO data at all — with as little between the
//! controller and the answer as possible. No GATT, no ASCS, no codec: just a
//! connection, a CIS, and whatever the controller hands up.
//!
//! Build with `--no-default-features --features stm32wba65ri-iso`.
//!
//! The peer is `iso_central`, which creates the CIG and the CIS and sends data.

#![no_std]
#![no_main]

use bt_hci::cmd::le::{LeAcceptCisRequest, LeSetupIsoDataPath};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use bt_hci::param::{CodecId as HciCodecId, ConnHandle, DataPathDirection, DataPathId, ExtDuration};
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
// The WBA stack type, aliased: `Controller` is the trouble-host trait this file
// bounds its tasks on, and the two would collide.
use embassy_stm32_wpan::{
    Controller as WbaController, HighInterruptHandler, LowInterruptHandler, Platform, new_platform,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use {defmt_rtt as _, panic_probe as _};

use trouble_host::iso::IsoEvents;
use trouble_host::prelude::*;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    AES => aes::InterruptHandler<AES>;
    PKA => pka::InterruptHandler<PKA>;
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
});

/// The sink's address. `iso_central` connects to exactly this, so neither side
/// has to implement discovery.
const SINK_ADDRESS: [u8; 6] = [0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff];

const SDU_MAX: usize = 120;
const SDU_QUEUE: usize = 8;

/// The controller capabilities the sink path needs.
trait SinkController:
    Controller + ControllerCmdAsync<LeAcceptCisRequest> + ControllerCmdSync<LeSetupIsoDataPath<'static>>
{
}

impl<T> SinkController for T where
    T: Controller + ControllerCmdAsync<LeAcceptCisRequest> + ControllerCmdSync<LeSetupIsoDataPath<'static>>
{
}

/// LC3, transparent coding format.
fn lc3() -> HciCodecId {
    HciCodecId {
        coding_format: 0x06,
        company_id: 0x0000,
        vendor_specific_codec_id: 0x0000,
    }
}

#[embassy_executor::task]
async fn ble_runner_task(platform: &'static Platform) {
    platform.run_ble().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = rcc::Config::new_wpan();

    let p = embassy_stm32::init(config);
    info!("STM32WBA6 ISO sink");

    let (platform, runtime) = new_platform!(
        Rng::new(p.RNG, Irqs),
        Pka::new(p.PKA, Irqs),
        Aes::new_blocking(p.AES, Irqs),
        8
    );

    spawner.spawn(ble_runner_task(platform).expect("Failed to spawn BLE runner"));

    let ble = WbaController::new(platform, runtime, Irqs)
        .await
        .expect("BLE initialization failed");
    info!("BLE stack initialized");

    run(ControllerAdapter::new(ble)).await;
}

async fn run<C: SinkController>(controller: C) {
    let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(SINK_ADDRESS))
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();
    let iso = stack.iso();

    let events: IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX> = IsoEvents::new();

    let app = async {
        let mut adv = [0u8; 31];
        let len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::CompleteLocalName(b"iso-sink"),
            ],
            &mut adv[..],
        )
        .expect("advertising data fits");

        loop {
            let advertiser = peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv[..len],
                        scan_data: &[],
                    },
                )
                .await
                .expect("advertising starts");
            info!("[sink] advertising as iso-sink");

            // Held for as long as the connection should live: the CIS is created
            // on top of this ACL connection.
            let conn = match advertiser.accept().await {
                Ok(conn) => conn,
                Err(e) => {
                    warn!("[sink] accept failed: {:?}", Debug2Format(&e));
                    continue;
                }
            };
            info!("[sink] connected, waiting for a CIS request");

            let request = events.request().await;
            info!(
                "[sink] CIS request: cis=0x{:04x} acl=0x{:04x} cig={} cis_id={}",
                request.cis_handle.0, request.acl_handle.0, request.cig_id, request.cis_id
            );

            if let Err(e) = iso.command_async(LeAcceptCisRequest::new(request.cis_handle)).await {
                warn!("[sink] accepting failed: {:?}", Debug2Format(&e));
                continue;
            }

            let established = events.established().await;
            info!(
                "[sink] CIS established: handle=0x{:04x} interval={}us nse={} max_pdu_c2p={} max_pdu_p2c={}",
                established.handle.0,
                established.iso_interval.as_micros(),
                established.nse,
                established.max_pdu_c_to_p,
                established.max_pdu_p_to_c
            );

            // Audio arrives from the controller, so the direction is
            // controller-to-host. Getting this backwards fails at the controller.
            match iso
                .command(LeSetupIsoDataPath::new(
                    established.handle,
                    DataPathDirection::Output,
                    DataPathId::HCI,
                    lc3(),
                    ExtDuration::from_micros(0),
                    &[],
                ))
                .await
            {
                Ok(_) => info!("[sink] data path ready, waiting for audio"),
                Err(e) => {
                    warn!("[sink] setting up the data path failed: {:?}", Debug2Format(&e));
                    continue;
                }
            }

            let mut count: u32 = 0;
            loop {
                let sdu = events.received().await;
                count = count.wrapping_add(1);
                if count <= 5 || count % 100 == 0 {
                    info!(
                        "[sink] SDU {} of {} octets on 0x{:04x}: {:02x} {:02x} {:02x} {:02x}",
                        count,
                        sdu.len,
                        sdu.handle.0,
                        sdu.as_slice().first().copied().unwrap_or(0),
                        sdu.as_slice().get(1).copied().unwrap_or(0),
                        sdu.as_slice().get(2).copied().unwrap_or(0),
                        sdu.as_slice().get(3).copied().unwrap_or(0),
                    );
                }
            }
        }
    };

    let _ = join(runner.run_with_handler(&events), app).await;
}

/// Keeps the `ConnHandle` import meaningful if the codec-id shape changes.
#[allow(dead_code)]
fn _conn_handle_is_used(h: ConnHandle) -> u16 {
    h.0
}
