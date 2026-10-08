//! Minimal ISO central: create the CIG and CIS, then send audio.
//!
//! The counterpart to `iso_sink`. Between them they answer the one question the
//! host-side work cannot: whether the STM32WBA carries ISO data at all.
//!
//! # Why this talks to the link layer instead of over HCI
//!
//! It first tried the obvious route — `iso.command(LeSetCigParameters{..})`, the
//! HCI command the specification names. Both it and `LeSetCigParametersTest`
//! (0x063, correctly encoded by stock bt-hci) came back `0x12`, *Invalid HCI
//! Command Parameters*, even though the ACL connection was up and the parameters
//! were well formed.
//!
//! That is not a parameter problem. In the link-layer-only configuration ST ships
//! a *link layer* and expects the host to drive it: the legacy HCI commands work,
//! and ISO/CIG management lives behind the internal `ll_intf_*` API instead. We
//! had already called `ll_intf_le_set_cig_params` by hand and seen
//! `ble_stat_t = 0` from it, so the link layer can do this — it is only the HCI
//! front door that is missing.
//!
//! So this calls the link layer directly for the three things that matter:
//! `ll_intf_le_set_cig_params`, `ll_intf_le_create_cis`, and
//! `ll_intf_setup_iso_data_path`. The structures mirror `ll_intf.h`. Once this is
//! proven, the same code belongs in `embassy-stm32-wpan`'s WBA controller rather
//! than in an example.
//!
//! Build with `--no-default-features --features stm32wba65ri-iso`.

#![no_std]
#![no_main]

use bt_hci::data::IsoPacket;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
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
    Controller as WbaController, HighInterruptHandler, LowInterruptHandler, Platform, Runtime, new_platform,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Timer};
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

/// The sink's address, matching `iso_sink`. Neither side implements discovery.
const SINK_ADDRESS: [u8; 6] = [0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff];
const CENTRAL_ADDRESS: [u8; 6] = [0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xfd];

const SDU_MAX: usize = 120;
const SDU_QUEUE: usize = 8;
/// 48 kHz/10 ms LC3 frames arrive every 10 ms.
const SDU_INTERVAL_US: u32 = 10_000;
/// LC3 at 48 kHz/10 ms mono, as advertised by the sink.
const SDU_LEN: usize = 40;

/// The link layer's own ISO interface, as declared in `ll_intf.h`.
///
/// These are the calls the HCI commands are supposed to reach, and on this stack
/// they only exist here.
mod ll_intf {
    /// `ble_intf_set_cig_params_common_cmd_st`.
    #[repr(C)]
    pub struct CigHostParam {
        pub sdu_intrv_m_to_s: u32,
        pub sdu_intrv_s_to_m: u32,
        pub iso_interval: u16,
        pub max_trnsprt_ltncy_m_to_s: u16,
        pub max_trnsprt_ltncy_s_to_m: u16,
        pub cig_id: u8,
        pub sca: u8,
        pub pack: u8,
        pub framing: u8,
        pub cis_cnt: u8,
        pub ft_m_to_s: u8,
        pub ft_s_to_m: u8,
    }

    /// One CIS's parameters inside the command.
    #[repr(C)]
    pub struct CisHostParam {
        pub cis_id: u8,
        pub max_sdu_m_to_s_least: u8,
        pub max_sdu_m_to_s_most: u8,
        pub max_sdu_s_to_m_least: u8,
        pub max_sdu_s_to_m_most: u8,
        pub phy_m_to_s: u8,
        pub phy_s_to_m: u8,
        pub rtn_m_to_s: u8,
        pub rtn_s_to_m: u8,
    }

    /// `ble_intf_set_cig_params_comman_cmd_st`.
    #[repr(C)]
    pub struct SetCigParamsCmd {
        pub cis_host_params: *const CisHostParam,
        pub cis_host_params_test: *const u8,
        pub cig_host_params: CigHostParam,
        pub slv_cis_id: u8,
    }

    /// `ble_intf_create_cis_cmd_hndl_st`: a CIS mapped to the ACL it rides on.
    #[repr(C)]
    pub struct CreateCisHndl {
        pub cis_conn_hndl: u16,
        pub acl_conn_hndl: u16,
    }

    /// `ble_intf_create_cis_cmd_st`.
    #[repr(C)]
    pub struct CreateCisCmd {
        pub create_cis_hndls: *const CreateCisHndl,
        pub cis_cnt: u8,
    }

    /// `ble_intf_setup_iso_data_path`.
    #[repr(C)]
    pub struct SetupIsoDataPath {
        pub codec_config: *mut u8,
        pub controller_delay: u32,
        pub codec_id: [u8; 5],
        pub data_path_dir: u8,
        pub data_path_id: u8,
        pub codec_config_length: u8,
    }

    // The layout of these has to match the C structs exactly, because the link
    // layer reads them directly. A mismatch here corrupts memory rather than
    // failing to compile, so it is checked.
    const _: () = assert!(core::mem::size_of::<CigHostParam>() == 24);
    const _: () = assert!(core::mem::size_of::<CisHostParam>() == 9);
    const _: () = assert!(core::mem::size_of::<SetCigParamsCmd>() == 36);
    const _: () = assert!(core::mem::size_of::<SetupIsoDataPath>() == 16);

    /// The `ble_stat_t` value meaning success.
    pub const BLE_STATUS_SUCCESS: u32 = 0x00;

    unsafe extern "C" {
        pub fn ll_intf_le_set_cig_params(cmd: *mut SetCigParamsCmd, conn_hndl: *mut u8) -> u32;
        pub fn ll_intf_le_create_cis(cmd: *mut CreateCisCmd) -> u32;
        pub fn ll_intf_setup_iso_data_path(conn_hndl: u16, params: *mut SetupIsoDataPath) -> u32;
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
    info!("STM32WBA6 ISO central");

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

async fn run<T: Runtime>(controller: ControllerAdapter<'_, T>) {
    let mut resources: HostResources<DefaultPacketPool, 1, 3> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(CENTRAL_ADDRESS))
        .build();
    let mut runner = stack.runner();
    let mut central = stack.central();
    let iso = stack.iso();

    let events: IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX> = IsoEvents::new();

    let app = async {
        let target = Address::random(SINK_ADDRESS);
        // Connection parameters are deliberately left at the default.
        //
        // They were briefly forced to a 10 ms interval with a 10 ms event length, on
        // the theory that a 10 ms ISO interval cannot be scheduled inside a slow ACL
        // connection — which is still the leading suspect for the `0x1f` below. It
        // did not work: `LE Create Connection` went out and no connection complete
        // ever came back, so the link layer rejects that combination and the host
        // waits forever. Whatever interval this turns out to need, it is not that
        // one, and it should be tried with the connection outcome logged rather than
        // assumed.
        let config = ConnectConfig {
            connect_params: Default::default(),
            scan_config: ScanConfig {
                filter_accept_list: &[target],
                ..Default::default()
            },
        };

        // Connecting is racy on this stack.
        //
        // The command is accepted (`LE Create Connection` returns success) and the
        // link layer does connect — the sink logs the connection every time — but
        // the connection-complete event sometimes never reaches the host, and then
        // `connect()` waits forever while the sequencer spins. When it works the
        // event turns up 100–500 ms after the command.
        //
        // So bound each attempt and retry: a lost event then costs a few seconds
        // instead of the whole run.
        info!("[central] connecting to the sink");
        let mut connected = None;
        for attempt in 1..=5u32 {
            match select(central.connect(&config), Timer::after_millis(1500)).await {
                Either::First(Ok(conn)) => {
                    connected = Some(conn);
                    break;
                }
                Either::First(Err(e)) => {
                    warn!("[central] attempt {} failed: {:?}", attempt, Debug2Format(&e));
                }
                Either::Second(()) => {
                    warn!("[central] attempt {} timed out: the connection event never arrived", attempt);
                }
            }
        }

        let Some(conn) = connected else {
            warn!("[central] could never connect");
            return;
        };
        let acl = conn.handle();
        info!("[central] connected, acl=0x{:04x}", acl.0);

        // 1. Configure the CIG, through the link layer.
        let cis_params = [ll_intf::CisHostParam {
            cis_id: 0,
            max_sdu_m_to_s_least: SDU_LEN as u8,
            max_sdu_m_to_s_most: 0,
            max_sdu_s_to_m_least: SDU_LEN as u8,
            max_sdu_s_to_m_most: 0,
            phy_m_to_s: 0x02, // LE 2M
            phy_s_to_m: 0x02,
            rtn_m_to_s: 2,
            rtn_s_to_m: 2,
        }];
        let mut set_cig = ll_intf::SetCigParamsCmd {
            cis_host_params: cis_params.as_ptr(),
            cis_host_params_test: core::ptr::null(),
            cig_host_params: ll_intf::CigHostParam {
                sdu_intrv_m_to_s: SDU_INTERVAL_US,
                sdu_intrv_s_to_m: SDU_INTERVAL_US,
                // 1.25 ms units: 10 ms, matching the SDU interval. An ISO interval
                // shorter than the SDU interval is not a legal configuration and is
                // the first thing to suspect when CIG setup succeeds and CIS
                // creation then fails.
                iso_interval: 8,
                max_trnsprt_ltncy_m_to_s: 60,
                max_trnsprt_ltncy_s_to_m: 60,
                cig_id: 0,
                sca: 0,
                pack: 0,    // sequential
                framing: 0, // unframed
                cis_cnt: 1,
                ft_m_to_s: 2,
                ft_s_to_m: 2,
            },
            slv_cis_id: 0,
        };

        let mut cis_handle: u8 = 0;
        // SAFETY: both pointers are valid for the duration of the call, and the
        // structures match the layouts `ll_intf.h` declares.
        let status = unsafe { ll_intf::ll_intf_le_set_cig_params(&mut set_cig, &mut cis_handle) };
        info!(
            "[central] ll_intf_le_set_cig_params -> ble_stat_t=0x{:02x}, cis handle {}",
            status, cis_handle
        );
        if status != ll_intf::BLE_STATUS_SUCCESS {
            warn!("[central] the link layer refused the CIG; nothing more to try");
            return;
        }

        // 2. Create the CIS on the ACL connection we already have.
        //
        // The contract in `ll_intf.h` says `conn_hndl` is "array of connection
        // handles in the CIG" and that this call takes ISO connection handles and
        // ACL connection handles — which is what we pass. It still answers `0x1f`,
        // *Unspecified Error*, and the implementation is inside a prebuilt archive,
        // so the parameters cannot be reasoned about any further. Sweep instead.
        //
        // A settle delay first: this used to fire about a millisecond after the
        // connection came up, and a CIS is meant to follow a settled ACL link.
        Timer::after(Duration::from_millis(500)).await;

        let mut created = false;
        for (cis, acl_h) in [
            (cis_handle, acl.0), // exactly what the documentation implies
            (cis_handle, 1),
            (cis_handle, 2),
            (0, acl.0),
            (1, acl.0),
            (cis_handle, cis_handle as u16),
        ] {
            let hndls = [ll_intf::CreateCisHndl {
                cis_conn_hndl: cis as u16,
                acl_conn_hndl: acl_h,
            }];
            let mut create_cis = ll_intf::CreateCisCmd {
                create_cis_hndls: hndls.as_ptr(),
                cis_cnt: 1,
            };
            // SAFETY: both pointers are valid for the duration of the call, and
            // the structures match the layouts `ll_intf.h` declares.
            let status = unsafe { ll_intf::ll_intf_le_create_cis(&mut create_cis) };
            info!(
                "[central] create_cis(cis={}, acl={}) -> ble_stat_t=0x{:02x}",
                cis, acl_h, status
            );
            if status == ll_intf::BLE_STATUS_SUCCESS {
                created = true;
                info!("[central] the link layer accepted cis={} acl={}", cis, acl_h);
                break;
            }
        }

        if !created {
            warn!("[central] no handle combination was accepted");
            return;
        }

        // The CIS coming up is reported asynchronously, as an HCI event.
        info!("[central] waiting for the CIS to come up");
        let established = events.established().await;
        info!(
            "[central] CIS established: handle=0x{:04x} interval={}us",
            established.handle.0,
            established.iso_interval.as_micros()
        );

        // 3. The data path. We send, so the direction is input to the controller.
        let mut data_path = ll_intf::SetupIsoDataPath {
            codec_config: core::ptr::null_mut(),
            controller_delay: 0,
            codec_id: [0x06, 0x00, 0x00, 0x00, 0x00], // LC3
            data_path_dir: 0,                          // input: host to controller
            data_path_id: 0,                           // HCI
            codec_config_length: 0,
        };
        // SAFETY: as above.
        let status = unsafe { ll_intf::ll_intf_setup_iso_data_path(established.handle.0, &mut data_path) };
        info!("[central] ll_intf_setup_iso_data_path -> ble_stat_t=0x{:02x}", status);
        if status != ll_intf::BLE_STATUS_SUCCESS {
            warn!("[central] the link layer refused the data path");
            return;
        }

        info!("[central] sending");
        let mut count: u32 = 0;
        let mut sdu = [0u8; SDU_LEN];
        loop {
            // A recognisable pattern: every SDU is filled with one byte, counting
            // up, so the sink can tell them apart and see gaps.
            sdu.fill(count as u8);
            match iso.send(&IsoPacket::new(established.handle, &sdu)).await {
                Ok(()) => {
                    if count < 5 || count % 100 == 0 {
                        info!(
                            "[central] sent SDU {} ({} octets of 0x{:02x})",
                            count,
                            sdu.len(),
                            count as u8
                        );
                    }
                }
                Err(e) => {
                    warn!("[central] sending failed at SDU {}: {:?}", count, Debug2Format(&e));
                    break;
                }
            }
            count = count.wrapping_add(1);
            Timer::after(Duration::from_micros(SDU_INTERVAL_US as u64)).await;
        }

        info!("[central] stopped");
        core::future::pending::<()>().await
    };

    let _ = join(runner.run_with_handler(&events), app).await;
}
