//! LE Audio unicast sink bring-up for the STM32WBA6.
//!
//! A peer can discover what this device can play (PACS), configure it (ASCS), and
//! then send audio over a connected isochronous stream. This first cut takes the
//! stream as far as receiving SDUs and logging them: there is no LC3 decode and no
//! audio output yet, because the point of this firmware is to find out whether the
//! Full link layer carries ISO data at all.
//!
//! Build with `--no-default-features --features stm32wba65ri-audio`; the Basic and
//! Full link-layer images cannot be linked together.
//!
//! ## Known gaps
//!
//! The ASE characteristic keeps reporting `Idle`. Reporting the negotiated
//! parameters requires the server to retain them — the codec configuration from
//! `Config Codec`, the transport parameters from `Config QoS` — which is real
//! application state and deliberately left out of this cut. A client that tracks
//! progress through the control point responses is unaffected; one that reads the
//! ASE characteristic would see a stale value.

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
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use trouble_audio::{Ase, AseDirection, AseState, CodecCapabilities, CodecId, PacRecord};
use trouble_host::audio::{ascs, pacs};
use trouble_host::iso::IsoEvents;
use trouble_host::prelude::*;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    AES => aes::InterruptHandler<AES>;
    PKA => pka::InterruptHandler<PKA>;
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
});

/// Max number of connections.
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels: signal + att.
const L2CAP_CHANNELS_MAX: usize = 2;
/// Attributes in the table: GAP, PACS, one ASE and the control point.
const ATT_MAX: usize = 64;
/// The single sink ASE this device exposes.
const ASE_ID: u8 = 0;
/// The largest SDU the queue will hold. An LC3 frame at 48 kHz/10 ms mono is far
/// below this; the ceiling exists so a larger one is dropped rather than spliced.
const SDU_MAX: usize = 120;
/// How many received SDUs to hold before dropping.
const SDU_QUEUE: usize = 8;

/// The codec-specific capability this device advertises for a sink: LC3 at
/// 48 kHz, 10 ms frames, mono, up to 40 octets per frame.
const LC3_CAPABILITIES: &[u8] = &[
    0x05, 0x01, 0x03, 0x08, 0x00, 0x00, // Sampling frequencies: 48 kHz
    0x03, 0x02, 0x02, 0x00, // Frame durations: 10 ms
    0x05, 0x03, 0x01, 0x00, 0x00, 0x00, // Channel counts: 1
    0x04, 0x04, 40, 40, // Octets per codec frame: 40..=40
];

/// Backing storage for the ASE characteristic values. The attribute table borrows
/// it for as long as the server lives, and the PAC records are `const`, so that
/// borrow resolves to `'static` and the storage has to live there too.
static ASCS_STORAGE: StaticCell<ascs::AscsStorage<1>> = StaticCell::new();

/// What this server bounds its tasks on: a controller that can run the three ISO
/// commands the sink path needs.
trait SinkController:
    Controller + ControllerCmdAsync<LeAcceptCisRequest> + ControllerCmdSync<LeSetupIsoDataPath<'static>>
{
}

impl<T> SinkController for T where
    T: Controller + ControllerCmdAsync<LeAcceptCisRequest> + ControllerCmdSync<LeSetupIsoDataPath<'static>>
{
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
    info!("Embassy STM32WBA6 LE Audio unicast sink");

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
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xfe]))
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();
    let iso = stack.iso();

    // The ASE state machine. Its mirror in the attribute table is what a client
    // reads; the two have to move together.
    let mut ases = [Ase::new(ASE_ID, AseDirection::Sink)];

    // What this server is prepared to accept, checked against every `Config
    // Codec` a peer sends.
    let sink_records = [PacRecord {
        codec_id: CodecId::LC3,
        codec_specific_capabilities: LC3_CAPABILITIES,
        metadata: &[],
    }];
    let capabilities = CodecCapabilities {
        sink: &sink_records,
        source: &[],
    };

    let mut table = AttributeTable::<NoopRawMutex, ATT_MAX>::new();
    GapConfig::default("TrouBLE Audio")
        .build(&mut table)
        .expect("GAP service");

    // PACS: what we can be configured to do.
    pacs::PacsConfig {
        sink_pac: pacs::LC3_SINK_48K_10MS_MONO,
        sink_locations: 0x0000_0001, // Front left, which is also mono.
        source_pac: &[],
        source_locations: 0,
        sink_available_contexts: 0x0002, // Conversational
        sink_supported_contexts: 0x0002, // Conversational
        source_available_contexts: 0,
        source_supported_contexts: 0,
    }
    .build(&mut table);

    // ASCS: the ASE a peer configures, and the control point it drives.
    let ascs_storage = ASCS_STORAGE.init(ascs::AscsStorage::<1>::new());
    let idle = [ASE_ID, 0x00];
    let ascs_handles = ascs::add_service(
        &mut table,
        &[ascs::AseDesc {
            id: ASE_ID,
            direction: AseDirection::Sink,
        }],
        &[&idle],
        ascs_storage,
    );

    let server: AttributeServer<'_, NoopRawMutex, DefaultPacketPool, ATT_MAX, CONNECTIONS_MAX> =
        AttributeServer::new(table);

    // ISO events arrive through the runner's event handler, so this has to exist
    // before the runner starts.
    let iso_events: IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX> = IsoEvents::new();

    let app = async {
        let mut advertiser_data = [0; 31];
        // Advertise both audio services so a peer knows what it is connecting to.
        let len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::IncompleteServiceUuids16(&[[0x4e, 0x18], [0x50, 0x18]]), // ASCS, PACS
                AdStructure::CompleteLocalName(b"TrouBLE Audio"),
            ],
            &mut advertiser_data[..],
        )
        .expect("advertising data fits");

        loop {
            let advertiser = peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &advertiser_data[..len],
                        scan_data: &[],
                    },
                )
                .await
                .expect("advertising starts");

            info!("[adv] advertising");
            let conn = match advertiser.accept().await {
                Ok(conn) => conn,
                Err(e) => {
                    warn!("[adv] accept failed: {:?}", Debug2Format(&e));
                    continue;
                }
            };

            info!("[adv] connected");
            let conn = match conn.with_attribute_server(&server) {
                Ok(conn) => conn,
                Err(e) => {
                    warn!("[adv] attribute server: {:?}", Debug2Format(&e));
                    continue;
                }
            };

            // One connection is enough: this device serves a single peer.
            serve(
                &conn,
                &server,
                &stack,
                &iso,
                &iso_events,
                &mut ases,
                &capabilities,
                &ascs_handles,
            )
            .await;
            info!("[adv] disconnected");
        }
    };

    let _ = join(runner.run_with_handler(&iso_events), app).await;
}

/// Serve GATT and audio for one connection.
#[allow(clippy::too_many_arguments)]
async fn serve<C: SinkController, P: PacketPool>(
    conn: &GattConnection<'_, '_, P>,
    server: &AttributeServer<'_, NoopRawMutex, P, ATT_MAX, CONNECTIONS_MAX>,
    stack: &Stack<'_, C, P>,
    iso: &trouble_host::iso::Iso<'_, C, P>,
    iso_events: &IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX>,
    ases: &mut [Ase],
    capabilities: &CodecCapabilities<'_>,
    handles: &ascs::AscsHandles<1>,
) {
    let mut response = [0u8; ascs::MAX_RESPONSE_LEN];

    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {:?}", reason);
                return;
            }
            GattConnectionEvent::Gatt { event } => {
                let handled: Result<(), Error> = match event {
                    GattEvent::Write(event) if event.handle() == handles.control_point => {
                        let outcome = event
                            .with_data(|_, data| ascs::handle_control_point(ases, capabilities, data, &mut response));

                        match outcome {
                            ascs::ControlPointOutcome::Respond(len) => {
                                info!("[ascs] control point answered with {} octets", len);
                                match event.accept() {
                                    Ok(reply) => {
                                        // The response is an indication, so it has to
                                        // reach the peer before the next operation.
                                        if let Err(e) =
                                            server.notify(stack, handles.control_point, &response[..len]).await
                                        {
                                            warn!("[ascs] indicating the response failed: {:?}", Debug2Format(&e));
                                        }
                                        // `send` has nothing to report: a failure to put
                                        // the reply on the link shows up as a
                                        // disconnect, not as an error here.
                                        reply.send().await;
                                        Ok(())
                                    }
                                    Err(e) => Err(e),
                                }
                            }
                            ascs::ControlPointOutcome::Reject(err) => {
                                warn!("[ascs] malformed control point, rejecting");
                                match event.reject(err) {
                                    Ok(reply) => {
                                        reply.send().await;
                                        Ok(())
                                    }
                                    Err(e) => Err(e),
                                }
                            }
                        }
                    }
                    GattEvent::Write(event) => {
                        info!("[gatt] write to handle {}", event.handle());
                        match event.accept() {
                            Ok(reply) => {
                                reply.send().await;
                                Ok(())
                            }
                            Err(e) => Err(e),
                        }
                    }
                    GattEvent::Read(event) => match event.accept() {
                        Ok(reply) => {
                            reply.send().await;
                            Ok(())
                        }
                        Err(e) => Err(e),
                    },
                    other => match other.accept() {
                        Ok(reply) => {
                            reply.send().await;
                            Ok(())
                        }
                        Err(e) => Err(e),
                    },
                };

                if let Err(e) = handled {
                    warn!("[gatt] handling the request failed: {:?}", Debug2Format(&e));
                    return;
                }

                // Once a stream is enabled, bring up the CIS and take audio off it.
                if ases[0].state() == AseState::Enabling {
                    stream(iso, iso_events).await;
                }
            }
            _ => {}
        }
    }
}

/// Bring the stream up and receive audio until it stops.
async fn stream<C: SinkController, P: PacketPool>(
    iso: &trouble_host::iso::Iso<'_, C, P>,
    iso_events: &IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX>,
) {
    // The central creates the CIS; this side is asked whether it wants it.
    info!("[iso] waiting for a CIS request");
    let request = iso_events.request().await;
    let cis = request.cis_handle;
    info!("[iso] CIS request for cis 0x{:04x}", cis.0);

    if let Err(e) = iso.command_async(LeAcceptCisRequest::new(cis)).await {
        warn!("[iso] accepting the CIS failed: {:?}", Debug2Format(&e));
        return;
    }

    let established = iso_events.established().await;
    info!(
        "[iso] CIS established: handle 0x{:04x}, interval {} us",
        established.handle.0,
        established.iso_interval.as_micros()
    );

    // Audio flows from the controller to us, so the data path direction is
    // controller-to-host. Getting this backwards fails at the controller.
    let lc3 = HciCodecId {
        coding_format: 0x06,
        company_id: 0x0000,
        vendor_specific_codec_id: 0x0000,
    };
    if let Err(e) = iso
        .command(LeSetupIsoDataPath::new(
            established.handle,
            DataPathDirection::Output,
            DataPathId::HCI,
            lc3,
            ExtDuration::from_micros(0),
            &[],
        ))
        .await
    {
        warn!("[iso] setting up the data path failed: {:?}", Debug2Format(&e));
        return;
    }

    info!("[iso] data path ready, receiving");

    let mut count: u32 = 0;
    loop {
        let sdu = iso_events.received().await;
        count = count.wrapping_add(1);

        // Log sparsely: every 10 ms frame would flood RTT.
        if count % 100 == 1 {
            info!("[iso] SDU {} from 0x{:04x}: {} octets", count, sdu.handle.0, sdu.len);
        }
    }
}

/// Keeps the `ConnHandle` import meaningful if the codec-id shape changes; it is
/// part of the public surface this file builds on.
#[allow(dead_code)]
fn _conn_handle_is_used(h: ConnHandle) -> u16 {
    h.0
}
