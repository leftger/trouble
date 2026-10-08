//! Does the STM32WBA stack accept an ISO data packet?
//!
//! `write_iso_data` was `todo!()` in `embassy-stm32-wpan` until we implemented it
//! to hand the packet to `BleStack_Request` with its HCI type indicator, the same
//! way ACL data already goes. That compiles; whether the stack *accepts* packet
//! type `0x05` is the open question, and it is the last one standing between this
//! port and real audio.
//!
//! This asks it directly, and with controls. Calling `BleStack_Request` by hand
//! lets us see the status it returns, which the real `write_iso_data` path throws
//! away because the ACL path it mirrors throws it away too:
//!
//! - a well-formed ACL packet, which is known to work, as the baseline;
//! - the same packet with a byte corrupted, so we can see what a *rejected* packet
//!   looks like and tell it apart from an accepted one;
//! - a well-formed ISO data packet, which is the question;
//! - a packet with a type nothing should recognise, as the negative control.
//!
//! If the ISO packet returns the same status as the ACL one rather than the
//! nonsense type's, the stack dispatches it. If it returns something else
//! entirely, the dispatch is where the problem is — and we will know that before
//! building a central to drive a CIS.
//!
//! Build with `--no-default-features --features stm32wba65ri-iso`.

#![no_std]
#![no_main]

use bt_hci::param::ConnHandle;
use bt_hci::data::IsoPacket;
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
use {defmt_rtt as _, panic_probe as _};

use trouble_host::prelude::*;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    AES => aes::InterruptHandler<AES>;
    PKA => pka::InterruptHandler<PKA>;
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
});

/// The stack's single host-to-controller entry point, which every command and
/// every data packet goes through.
///
/// # Safety
///
/// The caller must pass a buffer holding one complete, well-formed HCI packet
/// including its type indicator, and must not call this concurrently with the
/// stack's own use of it.
unsafe extern "C" {
    fn BleStack_Request(buffer: *mut u8) -> u8;
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
    info!("STM32WBA6 ISO TX probe");

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

/// Send one hand-built packet through the stack and report what it says.
///
/// # Safety
///
/// `packet` must be a complete HCI packet with its type indicator, and this must
/// not run while the stack is using its own buffer.
unsafe fn send_raw(label: &str, packet: &mut [u8]) -> u8 {
    let status = unsafe { BleStack_Request(packet.as_mut_ptr()) };
    info!("[probe] {} -> status 0x{:02x}", label, status);
    status
}

async fn run<C: Controller>(controller: C) {
    let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random([0xff, 0x8f, 0x1c, 0x05, 0xe4, 0xfe]))
        .build();
    let mut runner = stack.runner();
    let iso = stack.iso();

    let app = async {
        info!("--- packet-type dispatch probe ---");

        // 1. Baseline: a well-formed ACL data packet. The stack handles these, so
        //    whatever it returns here is what "accepted" looks like.
        let mut acl = [0u8; 9];
        acl[0] = 0x02; // HCI ACL data
        acl[1] = 0x00; // handle 0, low
        acl[2] = 0x00; // handle 0, high
        acl[3] = 0x04; // length
        acl[4] = 0x00;
        acl[5..9].copy_from_slice(&[0x01, 0x00, 0x04, 0x00]);
        let acl_status = unsafe { send_raw("ACL data, well-formed", &mut acl) };

        // 2. The same packet with a corrupted length, so a rejection has a shape.
        let mut acl_bad = acl;
        acl_bad[3] = 0xff;
        let acl_bad_status = unsafe { send_raw("ACL data, corrupted length", &mut acl_bad) };

        // 3. The question: a well-formed ISO data packet for CIS handle 0.
        //    Type 0x05, handle with PB = 0b10 (complete SDU), then the data load
        //    header (sequence number, SDU length) and eight octets of payload.
        let mut iso_pkt = [0u8; 17];
        iso_pkt[0] = 0x05; // HCI ISO data
        iso_pkt[1] = 0x00; // handle 0, low
        iso_pkt[2] = 0x20; // PB = 0b10 in bits 12..14
        iso_pkt[3] = 0x0c; // data load length = 4 + 8
        iso_pkt[4] = 0x00;
        iso_pkt[5] = 0x00; // sequence number
        iso_pkt[6] = 0x00;
        iso_pkt[7] = 0x08; // ISO SDU length
        iso_pkt[8] = 0x00;
        for (i, b) in iso_pkt[9..17].iter_mut().enumerate() {
            *b = 0xa0 + i as u8;
        }
        let iso_status = unsafe { send_raw("ISO data, well-formed", &mut iso_pkt) };

        // 4. Negative control: a type the stack has no business recognising.
        let mut nonsense = [0u8; 9];
        nonsense.copy_from_slice(&acl);
        nonsense[0] = 0x09;
        let nonsense_status = unsafe { send_raw("type 0x09, nonsense", &mut nonsense) };

        info!("--- summary ---");
        info!("acl=0x{:02x} acl_corrupt=0x{:02x} iso=0x{:02x} nonsense=0x{:02x}",
            acl_status, acl_bad_status, iso_status, nonsense_status);
        if iso_status == acl_status {
            info!("[probe] the ISO packet returned the same status as a good ACL packet");
        }
        if iso_status == nonsense_status {
            info!("[probe] the ISO packet returned the same status as a nonsense type");
        }

        // 5. And through the path the host would actually use, which mirrors ACL
        //    and discards the status — if this ever panics we are still on the
        //    `todo!()`, and if it returns Ok the stack did not fault.
        let sdu = [0xa0u8, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7];
        match iso.send(&IsoPacket::new(ConnHandle::new(0), &sdu)).await {
            Ok(()) => info!("[probe] iso.send() returned Ok"),
            Err(e) => warn!("[probe] iso.send() failed: {:?}", Debug2Format(&e)),
        }

        info!("--- probe done ---");
        core::future::pending::<()>().await
    };

    let _ = join(runner.run(), app).await;
}
