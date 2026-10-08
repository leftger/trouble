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

use bt_hci::cmd::le::{LeReadBufferSizeV2, LeReadLocalSupportedFeatures, LeRemoveCig, LeSetCigParameters};
use bt_hci::controller::ControllerCmdSync;
use bt_hci::param::{CigId, CisConfig, CisId, ExtDuration, Framing, Packing, PhyMask};
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
use embassy_stm32_wpan::{
    Controller as WbaController, HighInterruptHandler, LowInterruptHandler, Platform, new_platform,
};
use embassy_time::Timer;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

/// Direct bindings to the ST link layer's `ll_intf_*` API (`ll_intf.h`).
///
/// The HCI path through the link-layer-only stack reports a wrong status for
/// `LE Set CIG Parameters`, so this calls the link layer directly instead. The
/// implementations are linked and real: `ll_intf_le_set_cig_params` is 0xb4
/// bytes in the linker map, not one of the 4-byte link-layer-only stubs.
#[allow(dead_code)]
mod ll_intf {
    /// Mirrors `ble_intf_cig_host_param_st`. Compiled with
    /// `SUPPORT_ISO_UNSEG_MODE == 0`, so there is no `seg_mode` field.
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

    /// Mirrors `ble_intf_cis_host_param_st`.
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

    /// Mirrors `ble_intf_set_cig_params_comman_cmd_st`.
    #[repr(C)]
    pub struct SetCigParamsCmd {
        pub cis_host_params: *const CisHostParam,
        pub cis_host_params_test: *const u8,
        pub cig_host_params: CigHostParam,
        pub slv_cis_id: u8,
    }

    // A layout mismatch would silently corrupt the call, so pin the C ABI sizes.
    const _: () = assert!(core::mem::size_of::<CigHostParam>() == 24);
    const _: () = assert!(core::mem::size_of::<CisHostParam>() == 9);
    const _: () = assert!(core::mem::size_of::<SetCigParamsCmd>() == 36);

    unsafe extern "C" {
        /// `ble_stat_t` is a `uint32_t`.
        pub fn ll_intf_le_set_cig_params(cmd: *mut SetCigParamsCmd, conn_hndl: *mut u8) -> u32;

        /// Reads ACL and ISO data-buffer sizing straight from the link layer.
        pub fn ll_intf_le_read_buffer_size_v2(
            le_acl_data_pkt_length: *mut u16,
            total_num_le_acl_data_pkts: *mut u8,
            iso_data_pkt_length: *mut u16,
            total_num_iso_data_pkts: *mut u8,
        ) -> u32;
    }
}

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
        + ControllerCmdSync<LeRemoveCig>
        + for<'a> ControllerCmdSync<LeSetCigParameters<'a>>,
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

        // 3. Baseline: is CIG 0 in use on a freshly reset controller? It must not
        //    be, otherwise the check after step 4 cannot distinguish "the CIG was
        //    allocated" from "Remove CIG succeeds regardless".
        match iso.command(LeRemoveCig::new(CigId::new(0))).await {
            Ok(r) => info!("LE Remove CIG(0) BEFORE: accepted, cig_id={}", r.cig_id.into_inner()),
            Err(e) => {
                let e = Debug2Format(&e);
                info!("LE Remove CIG(0) BEFORE: rejected: {:?}", e);
            }
        }

        // 4. Allocate a CIG. This is entirely local: no peer and no ACL connection
        //    are required, so it exercises the ISO resource machinery by itself.
        //
        //    One CIS carrying 48 kHz / 10 ms mono, 40-octet SDUs, unframed, 2M PHY.
        //
        //    REQUIRES A PATCHED bt-hci. Stock bt-hci 0.10.1 encodes
        //    `LeSetCigParameters` with the parameter layout of `LE Set CIG
        //    Parameters Test` (OCF 0x063) while sending the real opcode 0x062, so
        //    a spec-conforming controller rejects it with "Invalid HCI Command
        //    Parameters" (0x12).
        //
        //    Spec / ST `ble_hci_le.h` order for 0x062:
        //        CIG_ID, SDU_C_To_P, SDU_P_To_C, Worst_Case_SCA, Packing, Framing,
        //        Max_Transport_Latency_C_To_P, Max_Transport_Latency_P_To_C,
        //        CIS_Count, per-CIS...
        //    Stock bt-hci 0.10.1 emits:
        //        CIG_ID, SDU_C_To_P, SDU_P_To_C, FT_C_To_P, FT_P_To_C, ISO_Interval,
        //        Worst_Case_SCA, Packing, Framing, CIS_Count, per-CIS...
        //
        //    Both layouts total 24 octets, so `param_len` is correct and the
        //    mismatch is invisible in the length field; the controller reads
        //    offset 11 as `Packing`, finds 0x02, and rejects the command.
        //
        //    With the layout corrected, the encoding is spec-valid -- every field
        //    decodes correctly and param_len is 24 -- but this ST firmware still
        //    answers 0x12 while *actually allocating the CIG*. That is proven by
        //    steps 3 and 5: Remove CIG(0) reports "Unknown Connection
        //    Identifier" before this command and succeeds after it, with nothing
        //    else in between. So the controller's status contradicts its own
        //    behaviour, and because bt-hci surfaces the status as an error the
        //    host never learns the CIS connection handles it would need for
        //    LE Create CIS.
        //
        //    NOTE: this call uses the corrected field order, so it only compiles
        //    against a bt-hci with the layout fixed. The example's
        //    `[patch.crates-io]` line points at such a copy, and both that line
        //    and this call are deliberately left uncommitted; see the patch
        //    section in Cargo.toml.
        // 4. Parameter sweep. The controller allocates the CIG but reports an
        //    error, so vary one parameter at a time looking for a combination it
        //    accepts with status 0x00.
        //
        //    Each variant uses its own CIG ID, so a variant that allocates
        //    despite reporting an error cannot collide with the next one, and
        //    each is released afterwards. The cleanup result doubles as evidence
        //    of whether the controller really allocated the CIG.
        struct Variant {
            name: &'static str,
            phy_1m: bool,
            phy_2m: bool,
            sdu_us: u32,
            sca: u8,
            rtn: u8,
            mtl_ms: u16,
            max_sdu_c2p: u16,
            max_sdu_p2c: u16,
        }

        const VARIANTS: &[Variant] = &[
            Variant {
                name: "baseline 2M 10ms 40B",
                phy_1m: false,
                phy_2m: true,
                sdu_us: 10_000,
                sca: 0,
                rtn: 2,
                mtl_ms: 20,
                max_sdu_c2p: 40,
                max_sdu_p2c: 40,
            },
            Variant {
                name: "LE 1M phy",
                phy_1m: true,
                phy_2m: false,
                sdu_us: 10_000,
                sca: 0,
                rtn: 2,
                mtl_ms: 20,
                max_sdu_c2p: 40,
                max_sdu_p2c: 40,
            },
            Variant {
                name: "mtl 100ms",
                phy_1m: false,
                phy_2m: true,
                sdu_us: 10_000,
                sca: 0,
                rtn: 2,
                mtl_ms: 100,
                max_sdu_c2p: 40,
                max_sdu_p2c: 40,
            },
            Variant {
                name: "sca 7",
                phy_1m: false,
                phy_2m: true,
                sdu_us: 10_000,
                sca: 7,
                rtn: 2,
                mtl_ms: 20,
                max_sdu_c2p: 40,
                max_sdu_p2c: 40,
            },
            Variant {
                name: "rtn 0",
                phy_1m: false,
                phy_2m: true,
                sdu_us: 10_000,
                sca: 0,
                rtn: 0,
                mtl_ms: 20,
                max_sdu_c2p: 40,
                max_sdu_p2c: 40,
            },
            Variant {
                name: "sdu 7.5ms",
                phy_1m: false,
                phy_2m: true,
                sdu_us: 7_500,
                sca: 0,
                rtn: 2,
                mtl_ms: 20,
                max_sdu_c2p: 40,
                max_sdu_p2c: 40,
            },
            Variant {
                name: "max_sdu 120",
                phy_1m: false,
                phy_2m: true,
                sdu_us: 10_000,
                sca: 0,
                rtn: 2,
                mtl_ms: 20,
                max_sdu_c2p: 120,
                max_sdu_p2c: 120,
            },
            Variant {
                name: "sink-only c2p 0",
                phy_1m: false,
                phy_2m: true,
                sdu_us: 10_000,
                sca: 0,
                rtn: 2,
                mtl_ms: 20,
                max_sdu_c2p: 0,
                max_sdu_p2c: 40,
            },
            Variant {
                name: "both phys",
                phy_1m: true,
                phy_2m: true,
                sdu_us: 10_000,
                sca: 0,
                rtn: 2,
                mtl_ms: 20,
                max_sdu_c2p: 40,
                max_sdu_p2c: 40,
            },
        ];

        info!("--- LE Set CIG Parameters sweep, {} variants ---", VARIANTS.len());

        for (i, v) in VARIANTS.iter().enumerate() {
            let phy = PhyMask::new().set_le_1m_phy(v.phy_1m).set_le_2m_phy(v.phy_2m);
            let cis_configs = [CisConfig {
                cis_id: CisId::new(0),
                max_sdu_c_to_p: v.max_sdu_c2p,
                max_sdu_p_to_c: v.max_sdu_p2c,
                phy_c_to_p: phy,
                phy_p_to_c: phy,
                rtn_c_to_p: v.rtn,
                rtn_p_to_c: v.rtn,
            }];
            let cig_id = CigId::new(i as u8);

            let result = iso
                .command(LeSetCigParameters::new(
                    cig_id,
                    ExtDuration::<1>::from_micros(v.sdu_us as u64),
                    ExtDuration::<1>::from_micros(v.sdu_us as u64),
                    v.sca,
                    Packing::Sequential,
                    Framing::Unframed,
                    v.mtl_ms,
                    v.mtl_ms,
                    &cis_configs,
                ))
                .await;

            match result {
                // `LeSetCigParametersReturn` is packed, so copy the fields out.
                Ok(r) => {
                    // `LeSetCigParametersReturn` is packed, so copy the fields out
                    // before handing them to defmt, which would take a reference.
                    let num_cis = r.num_cis;
                    let cis_handle = r.cis_handles[0].0;
                    info!(
                        "variant {} [{}]: OK num_cis={} cis_handle={}",
                        i, v.name, num_cis, cis_handle
                    );
                }
                Err(e) => {
                    let e = Debug2Format(&e);
                    info!("variant {} [{}]: {:?}", i, v.name, e);
                }
            }

            match iso.command(LeRemoveCig::new(cig_id)).await {
                Ok(_) => info!("    cig {} WAS allocated", i),
                Err(e) => {
                    let e = Debug2Format(&e);
                    info!("    cig {} not allocated: {:?}", i, e);
                }
            }
        }

        info!("--- sweep done ---");

        // 6. Call the link layer directly, bypassing the HCI translation layer.
        //
        //    The link layer's real `ll_intf_*` implementations are linked and used
        //    (`ll_intf_le_set_cig_params` is 0xb4 bytes in the linker map), while
        //    the HCI path through the link-layer-only stack reports a wrong status
        //    even though it allocates. The link layer's own API hands back the CIS
        //    connection handle through an out-parameter, which is the value the HCI
        //    path never delivers.
        //
        //    One CIS: 48 kHz / 10 ms mono, 40-octet SDUs, 2M PHY, unframed.
        //    `iso_interval` and the flush timeouts have no equivalent in the
        //    non-test HCI command, so these values are a first guess.
        let cis_params = [ll_intf::CisHostParam {
            cis_id: 0,
            max_sdu_m_to_s_least: 40,
            max_sdu_m_to_s_most: 0,
            max_sdu_s_to_m_least: 40,
            max_sdu_s_to_m_most: 0,
            phy_m_to_s: 0x02, // LE 2M
            phy_s_to_m: 0x02,
            rtn_m_to_s: 2,
            rtn_s_to_m: 2,
        }];

        let mut cmd = ll_intf::SetCigParamsCmd {
            cis_host_params: cis_params.as_ptr(),
            cis_host_params_test: core::ptr::null(),
            cig_host_params: ll_intf::CigHostParam {
                sdu_intrv_m_to_s: 10_000,
                sdu_intrv_s_to_m: 10_000,
                iso_interval: 8, // 8 * 1.25 ms = 10 ms
                max_trnsprt_ltncy_m_to_s: 20,
                max_trnsprt_ltncy_s_to_m: 20,
                cig_id: 1,
                sca: 0,
                pack: 0,
                framing: 0,
                cis_cnt: 1,
                ft_m_to_s: 2,
                ft_s_to_m: 2,
            },
            slv_cis_id: 0,
        };

        let mut conn_hndl: u8 = 0xff;
        // SAFETY: `cis_params` and `cmd` outlive the call, and the struct layouts
        // are asserted against the C ABI above.
        let stat = unsafe { ll_intf::ll_intf_le_set_cig_params(&mut cmd, &mut conn_hndl) };
        info!(
            "direct ll_intf_le_set_cig_params: ble_stat_t={} conn_hndl={}",
            stat, conn_hndl
        );

        // Clean up over HCI, which does work for Remove CIG.
        match iso.command(LeRemoveCig::new(CigId::new(1))).await {
            Ok(_) => info!("HCI LE Remove CIG(1) after direct call: accepted"),
            Err(e) => {
                let e = Debug2Format(&e);
                info!("HCI LE Remove CIG(1) after direct call: {:?}", e);
            }
        }

        // 7. Read the ISO buffer sizing from the link layer directly. HCI opcode
        //    0x2060 returns 0x12 on this part, so this is the only way to get it.
        let (mut acl_len, mut acl_num) = (0u16, 0u8);
        let (mut iso_len, mut iso_num) = (0u16, 0u8);
        // SAFETY: all four pointers are valid for the duration of the call.
        let stat =
            unsafe { ll_intf::ll_intf_le_read_buffer_size_v2(&mut acl_len, &mut acl_num, &mut iso_len, &mut iso_num) };
        info!(
            "direct ll_intf_le_read_buffer_size_v2: ble_stat_t={} acl_len={} acl_num={} iso_len={} iso_num={}",
            stat, acl_len, acl_num, iso_len, iso_num
        );

        loop {
            Timer::after_secs(5).await;
            info!("probe alive");
        }
    };

    join(runner.run(), task).await;
}
