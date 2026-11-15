//! Minimal BLE LE Audio unicast sink on the STM32WBA65.
//!
//! Advertises over LE Extended Advertising (`BAP General Announcement` with
//! ASCS Service Data `0x184E`), exposes `PACS` (LC3 mono sink at 16/24/48 kHz,
//! 7.5/10 ms) and `ASCS` (1 Sink ASE), pairs with JustWorks + bonding,
//! requests LE 2M PHY, drives the ASCS ASE characteristic notifications across
//! state transitions, accepts CIS requests, sets up the controller-to-host ISO
//! data path, and decodes incoming LC3 SDUs (falling back to LC3 PLC on bad,
//! lost, or timed-out packets).

#![no_std]
#![no_main]

use bt_hci::cmd::le::{
    LeAcceptCisRequest, LeClearAdvSets, LeReadNumberOfSupportedAdvSets, LeSetAdvSetRandomAddr, LeSetExtAdvData,
    LeSetExtAdvEnable, LeSetExtAdvParams, LeSetExtScanResponseData, LeSetPhy, LeSetupIsoDataPath,
};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use bt_hci::param::{CodecId as HciCodecId, ConnHandle, DataPathDirection, DataPathId, ExtDuration, PhyKind};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{Either, select};
use embassy_stm32::rcc;
use embassy_stm32::{Config, bind_interrupts};
use embassy_stm32_wpan::controller::ControllerAdapter;
// The WBA stack type, aliased: `Controller` is the trouble-host trait this file
// imports via `trouble_host::prelude::*`.
use embassy_stm32_wpan::{
    Controller as WpanController, HighInterruptHandler, LowInterruptHandler, Platform, new_platform,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, with_timeout};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use trouble_audio::codec::AudioDecoder;
use trouble_audio::codec::lc3::{Complex, Lc3Decoder, Scaler, decoder_buffer_lengths};
use trouble_audio::types::FrameDuration;
use trouble_audio::{
    Ase, AseDirection, AseOperation, AseParams, AseState, AudioLocation, CodecCapabilities, CodecConfig,
    CodecConfiguredParams, CodecId, CodecSpecificConfig, ControlPointRequest, Framing, MetadataParams, PacRecord, Phy,
    QosConfiguredParams, SampleRate,
};
use trouble_host::audio::{ascs, pacs};
use trouble_host::iso::{IsoEvents, IsoSdu};
use trouble_host::prelude::*;

bind_interrupts!(struct Irqs {
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
});

/// Max number of connections.
const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels (signal + ATT + SMP + spare).
const L2CAP_CHANNELS_MAX: usize = 4;
/// Attributes in the table: GAP, PACS, one ASE and the control point.
const ATT_MAX: usize = 64;
/// ASCS Table 3.1: ASE_ID 0x00 is RFU; valid IDs start at 1.
const ASE_ID: u8 = 1;
/// Largest LC3 SDU we accept (max advertised `Octets_Per_Codec_Frame` = 155).
const SDU_MAX: usize = 160;
/// Queued SDUs between the runner ISR/rx path and the LC3 decoder loop.
const SDU_QUEUE: usize = 8;

/// LC3 Sink capabilities: 16 / 24 / 48 kHz, 7.5 / 10 ms, 1 channel, 30..=155 octets, 1 frame/SDU.
const LC3_CAPABILITIES: &[u8] = &[
    // Supported_Sampling_Frequencies: 16 kHz (0x0004) | 24 kHz (0x0010) | 48 kHz (0x0080) = 0x0094 LE.
    0x03, 0x01, 0x94, 0x00, // Supported_Frame_Durations: 7.5 ms (bit 0) | 10 ms (bit 1) = 0x03.
    0x02, 0x02, 0x03, // Supported_Audio_Channel_Counts: 1 channel (bit 0).
    0x02, 0x03, 0x01, // Supported_Octets_Per_Codec_Frame: 30..=155 octets LE.
    0x05, 0x04, 30, 0x00, 155, 0x00, // Supported_Max_Codec_Frames_Per_SDU: 1.
    0x02, 0x05, 0x01,
];

/// Available & Supported Sink Audio Contexts: Unspecified (0x0001) | Conversational (0x0002) | Media (0x0004).
const SINK_CONTEXTS: u16 = 0x0007;

/// Largest LC3 configuration supported, used to size static decoder scratch buffers.
const LC3_MAX_CFG: CodecConfig = CodecConfig::new(SampleRate::Hz48000, FrameDuration::Ms10, 1, 155);
const LC3_DEC_LENS: (usize, usize) = match decoder_buffer_lengths(&LC3_MAX_CFG) {
    Some(lens) => lens,
    None => core::panic!("invalid LC3 decoder config"),
};

static ASCS_STORAGE: StaticCell<ascs::AscsStorage<1>> = StaticCell::new();

trait SinkController:
    Controller
    + ControllerCmdAsync<LeAcceptCisRequest>
    + ControllerCmdAsync<LeSetPhy>
    + ControllerCmdSync<LeSetupIsoDataPath<'static>>
    + for<'t> ControllerCmdSync<LeSetExtAdvData<'t>>
    + ControllerCmdSync<LeClearAdvSets>
    + ControllerCmdSync<LeSetExtAdvParams>
    + ControllerCmdSync<LeSetAdvSetRandomAddr>
    + ControllerCmdSync<LeReadNumberOfSupportedAdvSets>
    + for<'t> ControllerCmdSync<LeSetExtAdvEnable<'t>>
    + for<'t> ControllerCmdSync<LeSetExtScanResponseData<'t>>
{
}

impl<T> SinkController for T where
    T: Controller
        + ControllerCmdAsync<LeAcceptCisRequest>
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeSetupIsoDataPath<'static>>
        + for<'t> ControllerCmdSync<LeSetExtAdvData<'t>>
        + ControllerCmdSync<LeClearAdvSets>
        + ControllerCmdSync<LeSetExtAdvParams>
        + ControllerCmdSync<LeSetAdvSetRandomAddr>
        + ControllerCmdSync<LeReadNumberOfSupportedAdvSets>
        + for<'t> ControllerCmdSync<LeSetExtAdvEnable<'t>>
        + for<'t> ControllerCmdSync<LeSetExtScanResponseData<'t>>
{
}

/// Signal payload sent from the ASCS GATT handler to the ISO/decoder loop.
#[derive(Clone, Copy)]
enum StreamCommand {
    /// Codec + QoS parameters for an incoming/enabled CIS.
    Configure(CodecConfig),
    /// ASE was disabled or released.
    Stop,
}

/// Tracks the negotiated ASCS parameters needed to encode ASE characteristic values.
struct AseSession {
    ase: Ase,
    codec_id: CodecId,
    codec_config: [u8; 32],
    codec_config_len: usize,
    codec_cfg: CodecConfig,
    framing: Framing,
    phy: Phy,
    cig_id: u8,
    cis_id: u8,
    sdu_interval_us: u32,
    max_sdu: u16,
    rtn: u8,
    max_transport_latency_ms: u16,
    presentation_delay_us: u32,
    metadata: [u8; 32],
    metadata_len: usize,
}

impl AseSession {
    fn new(ase_id: u8) -> Self {
        Self {
            ase: Ase::new(ase_id, AseDirection::Sink),
            codec_id: CodecId::LC3,
            codec_config: [0; 32],
            codec_config_len: 0,
            codec_cfg: CodecConfig::new(SampleRate::Hz48000, FrameDuration::Ms10, 1, 100),
            framing: Framing::Unframed,
            phy: Phy::LE_2M,
            cig_id: 0,
            cis_id: 0,
            sdu_interval_us: 10_000,
            max_sdu: 100,
            rtn: 2,
            max_transport_latency_ms: 20,
            presentation_delay_us: 40_000,
            metadata: [0; 32],
            metadata_len: 0,
        }
    }

    fn record_request(&mut self, req: &ControlPointRequest<'_>) {
        let id = self.ase.id();
        match req.op() {
            AseOperation::ConfigCodec => {
                if let Some(block) = req.config_codec_blocks().flatten().find(|b| b.ase_id == id) {
                    self.codec_id = block.codec_id;
                    let n = block.config.len().min(self.codec_config.len());
                    self.codec_config[..n].copy_from_slice(&block.config[..n]);
                    self.codec_config_len = n;
                    self.phy = if block.target_phy == Phy::LE_1M.bits() {
                        Phy::LE_1M
                    } else {
                        Phy::LE_2M
                    };
                    if let Ok(cfg) = CodecSpecificConfig::new(block.config) {
                        let sr = cfg.sampling_rate().unwrap_or(SampleRate::Hz48000);
                        let fd = cfg.frame_duration().unwrap_or(FrameDuration::Ms10);
                        let octets = cfg.octets_per_frame().unwrap_or(100);
                        self.codec_cfg = CodecConfig::new(sr, fd, 1, octets);
                    }
                }
            }
            AseOperation::ConfigQos => {
                if let Some(block) = req.config_qos_blocks().flatten().find(|b| b.ase_id == id) {
                    self.cig_id = block.cig_id;
                    self.cis_id = block.cis_id;
                    self.sdu_interval_us = block.sdu_interval_us;
                    self.framing = Framing::from_u8(block.framing).unwrap_or(Framing::Unframed);
                    self.phy = if block.phy == Phy::LE_1M.bits() {
                        Phy::LE_1M
                    } else {
                        Phy::LE_2M
                    };
                    self.max_sdu = block.max_sdu;
                    self.rtn = block.rtn;
                    self.max_transport_latency_ms = block.max_transport_latency_ms;
                    self.presentation_delay_us = block.presentation_delay_us;
                }
            }
            AseOperation::Enable | AseOperation::UpdateMetadata => {
                if let Some(block) = req.metadata_blocks().flatten().find(|b| b.ase_id == id) {
                    let n = block.metadata.len().min(self.metadata.len());
                    self.metadata[..n].copy_from_slice(&block.metadata[..n]);
                    self.metadata_len = n;
                }
            }
            _ => {}
        }
    }

    fn encode_value(&self, out: &mut [u8]) -> Result<usize, trouble_audio::AseStatusError> {
        let params = match self.ase.state() {
            AseState::Idle | AseState::Releasing => AseParams::None,
            AseState::CodecConfigured => AseParams::CodecConfigured(CodecConfiguredParams {
                framing: self.framing,
                phy: self.phy,
                rtn: 2,
                max_transport_latency_ms: 20,
                presentation_delay_min_us: 10_000,
                presentation_delay_max_us: 40_000,
                preferred_presentation_delay_min_us: 10_000,
                preferred_presentation_delay_max_us: 40_000,
                codec_id: self.codec_id,
                config: &self.codec_config[..self.codec_config_len],
            }),
            AseState::QosConfigured => AseParams::QosConfigured(QosConfiguredParams {
                cig_id: self.cig_id,
                cis_id: self.cis_id,
                sdu_interval_us: self.sdu_interval_us,
                framing: self.framing,
                phy: self.phy,
                max_sdu: self.max_sdu,
                rtn: self.rtn,
                max_transport_latency_ms: self.max_transport_latency_ms,
                presentation_delay_us: self.presentation_delay_us,
            }),
            AseState::Enabling | AseState::Streaming | AseState::Disabling => AseParams::Metadata(MetadataParams {
                cig_id: self.cig_id,
                cis_id: self.cis_id,
                metadata: &self.metadata[..self.metadata_len],
            }),
        };
        ascs::encode_ase_value(out, self.ase.id(), self.ase.state(), params)
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

    let _p = embassy_stm32::init(config);
    info!("Embassy STM32WBA6 LE Audio unicast sink");

    let (platform, runtime) = new_platform!(8);
    spawner.spawn(ble_runner_task(platform).expect("Failed to spawn BLE runner"));

    let ble = WpanController::new(platform, runtime, Irqs)
        .await
        .expect("BLE initialization failed");
    let controller = ControllerAdapter::new(ble);

    run(controller).await;
}

async fn run<C: SinkController>(controller: C) {
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xfe]))
        .set_io_capabilities(IoCapabilities::NoInputNoOutput)
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();
    let iso = stack.iso();

    let mut session = AseSession::new(ASE_ID);
    let sink_records = [PacRecord {
        codec_id: CodecId::LC3,
        codec_specific_capabilities: LC3_CAPABILITIES,
        metadata: &[],
    }];
    let capabilities = CodecCapabilities {
        sink: &sink_records,
        source: &[],
    };

    let mut pac_buf = [0u8; 32];
    let pac_len = sink_records[0].encode(&mut pac_buf).expect("PAC record fits");

    let mut table: AttributeTable<'_, NoopRawMutex, ATT_MAX> = AttributeTable::new();
    table.add_service(Service::new(0x1800u16));

    pacs::PacsConfig {
        sink_pac: &pac_buf[..pac_len],
        sink_locations: AudioLocation::MONO.bits(),
        source_pac: &[],
        source_locations: 0,
        sink_available_contexts: SINK_CONTEXTS,
        sink_supported_contexts: SINK_CONTEXTS,
        source_available_contexts: 0,
        source_supported_contexts: 0,
    }
    .build(&mut table);

    let ascs_storage = ASCS_STORAGE.init(ascs::AscsStorage::<1>::new());
    let mut idle_buf = [0u8; 8];
    let idle_len = session.encode_value(&mut idle_buf).expect("idle ASE fits");
    let ases_desc = [ascs::AseDesc {
        id: ASE_ID,
        direction: AseDirection::Sink,
    }];
    let initial: [&[u8]; 1] = [&idle_buf[..idle_len]];
    let ascs_handles = ascs::add_service(&mut table, &ases_desc, &initial, ascs_storage);

    let server: AttributeServer<'_, NoopRawMutex, DefaultPacketPool, ATT_MAX, CONNECTIONS_MAX> =
        AttributeServer::new(table);

    let iso_events: IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX> = IsoEvents::new();
    let stream_cmd: Signal<NoopRawMutex, StreamCommand> = Signal::new();

    let _ = join(runner.run_with_handler(&iso_events), async {
        loop {
            match advertise("WBA6_Sink", &mut peripheral, &server).await {
                Ok(conn) => {
                    // Enable bonding (JustWorks) and request LE 2M PHY required for BAP audio.
                    let _ = conn.raw().set_bondable(true);
                    if let Err(e) = conn.raw().set_phy(&stack, PhyKind::Le2M).await {
                        warn!("[gap] LE 2M PHY request failed: {:?}", Debug2Format(&e));
                    }

                    session.ase = Ase::new(ASE_ID, AseDirection::Sink);
                    stream_cmd.reset();

                    let gatt = serve(
                        &conn,
                        &server,
                        &stack,
                        &stream_cmd,
                        &mut session,
                        &capabilities,
                        &ascs_handles,
                    );
                    let audio = iso_audio_task(&iso, &iso_events, &stream_cmd);

                    // `serve` exits on disconnect; `audio` loops until `serve` finishes.
                    select(gatt, audio).await;
                }
                Err(e) => {
                    warn!("[adv] error: {:?}", Debug2Format(&e));
                }
            }
        }
    })
    .await;
}

/// Encode extended advertising data with BAP Unicast Server General Announcement
/// (`ServiceData16` for ASCS `0x184E`) and accept the next connection.
async fn advertise<'a, 'b, C: SinkController, P: PacketPool>(
    name: &'a str,
    peripheral: &mut Peripheral<'a, C, P>,
    server: &'b AttributeServer<'a, NoopRawMutex, P, ATT_MAX, CONNECTIONS_MAX>,
) -> Result<GattConnection<'a, 'b, P>, BleHostError<C::Error>> {
    // ASCS General Announcement (BAP §3.7.2.2): Announcement_Type(0x00), Sink/Source
    // Available Audio Contexts (4 bytes LE), Metadata_Length(0x00).
    let sink_le = SINK_CONTEXTS.to_le_bytes();
    let ascs_svc_data = [0x00, sink_le[0], sink_le[1], 0x00, 0x00, 0x00];

    let mut adv_data = [0u8; 64];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(name.as_bytes()),
            AdStructure::IncompleteServiceUuids16(&[[0x4e, 0x18], [0x50, 0x18]]),
            AdStructure::Unknown {
                ty: 0x19, // Appearance: Loudspeaker (0x0841 LE)
                data: &[0x41, 0x08],
            },
            AdStructure::ServiceData16 {
                uuid: [0x4e, 0x18],
                data: &ascs_svc_data,
            },
        ],
        &mut adv_data[..],
    )?;

    let sets = [AdvertisementSet {
        params: AdvertisementParameters {
            primary_phy: PhyKind::Le1M,
            secondary_phy: PhyKind::Le2M,
            ..Default::default()
        },
        data: Advertisement::ExtConnectableNonscannableUndirected {
            adv_data: &adv_data[..len],
        },
        address: None,
    }];
    let mut handles = AdvertisementSet::handles(&sets);

    info!("[adv] extended advertising as {} ({} bytes)", name, len);
    let advertiser = peripheral.advertise_ext(&sets, &mut handles).await?;
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connected");
    Ok(conn)
}

async fn notify_ase<C: SinkController, P: PacketPool>(
    server: &AttributeServer<'_, NoopRawMutex, P, ATT_MAX, CONNECTIONS_MAX>,
    stack: &Stack<'_, C, P>,
    ase_handle: u16,
    session: &AseSession,
) {
    let mut buf = [0u8; 96];
    match session.encode_value(&mut buf) {
        Ok(n) => {
            if let Err(e) = server.write_and_notify(stack, ase_handle, &buf[..n]).await {
                warn!("[ascs] ASE notify failed: {:?}", e);
            }
        }
        Err(e) => {
            warn!("[ascs] ASE encode failed: {:?}", Debug2Format(&e));
        }
    }
}

async fn serve<C: SinkController, P: PacketPool>(
    conn: &GattConnection<'_, '_, P>,
    server: &AttributeServer<'_, NoopRawMutex, P, ATT_MAX, CONNECTIONS_MAX>,
    stack: &Stack<'_, C, P>,
    stream_cmd: &Signal<NoopRawMutex, StreamCommand>,
    session: &mut AseSession,
    capabilities: &CodecCapabilities<'_>,
    handles: &ascs::AscsHandles<1>,
) {
    let mut response = [0u8; ascs::MAX_RESPONSE_LEN];
    let mut _bonded: Option<BondInformation> = None;

    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {:?}", reason);
                stream_cmd.signal(StreamCommand::Stop);
                return;
            }
            GattConnectionEvent::PairingComplete { security_level, bond } => {
                info!("[sec] paired: level={:?}, bonded={}", security_level, bond.is_some());
                _bonded = bond;
            }
            GattConnectionEvent::PairingFailed(err) => {
                warn!("[sec] pairing failed: {:?}", err);
            }
            GattConnectionEvent::PhyUpdated { tx_phy, rx_phy } => {
                info!("[gap] PHY updated: tx={:?}, rx={:?}", tx_phy, rx_phy);
            }
            GattConnectionEvent::Gatt { event } => match event {
                GattEvent::Write(event) if event.handle() == handles.control_point => {
                    let mut parsed_op = None;
                    let outcome = event.with_data(|_, data| {
                        let outcome = ascs::handle_control_point(
                            core::slice::from_mut(&mut session.ase),
                            capabilities,
                            data,
                            &mut response,
                        );
                        if let Ok(req) = ControlPointRequest::parse(data) {
                            parsed_op = Some(req.op());
                            if matches!(outcome, ascs::ControlPointOutcome::Respond(_)) {
                                session.record_request(&req);
                            }
                        }
                        outcome
                    });

                    match outcome {
                        ascs::ControlPointOutcome::Respond(len) => {
                            // Send the ATT Write Response before CP indication and ASE notifications
                            // so the peer never sees a CP/ASE PDU arrive ahead of the Write Response.
                            match event.accept() {
                                Ok(reply) => reply.send().await,
                                Err(e) => {
                                    warn!("[gatt] CP accept failed: {:?}", e);
                                    continue;
                                }
                            }
                            if let Err(e) = server.notify(stack, handles.control_point, &response[..len]).await {
                                warn!("[ascs] CP notify failed: {:?}", e);
                            }
                            notify_ase(server, stack, handles.ases[0], session).await;

                            match parsed_op {
                                Some(AseOperation::ConfigQos | AseOperation::Enable) => {
                                    stream_cmd.signal(StreamCommand::Configure(session.codec_cfg));
                                }
                                Some(AseOperation::Disable) => {
                                    stream_cmd.signal(StreamCommand::Stop);
                                }
                                Some(AseOperation::Release) => {
                                    stream_cmd.signal(StreamCommand::Stop);
                                    session.ase.complete_release();
                                    notify_ase(server, stack, handles.ases[0], session).await;
                                }
                                _ => {}
                            }
                        }
                        ascs::ControlPointOutcome::Reject(code) => {
                            if let Ok(reply) = event.reject(code) {
                                reply.send().await;
                            }
                        }
                    }
                }
                GattEvent::Read(event) => match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] read reply error: {:?}", e),
                },
                GattEvent::Write(event) => match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] write reply error: {:?}", e),
                },
                GattEvent::Other(event) => match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] event reply error: {:?}", e),
                },
                GattEvent::NotAllowed(event) => match event.reject(AttErrorCode::READ_NOT_PERMITTED) {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] reject reply error: {:?}", e),
                },
            },
            _ => {}
        }
    }
}

/// Handles controller ISO setup events and decodes incoming LC3 SDUs (with PLC
/// on lost/corrupt/timed-out packets) without blocking the GATT server loop.
async fn iso_audio_task<C: SinkController, P: PacketPool>(
    iso: &trouble_host::iso::Iso<'_, C, P>,
    iso_events: &IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX>,
    stream_cmd: &Signal<NoopRawMutex, StreamCommand>,
) {
    let mut codec_cfg = CodecConfig::new(SampleRate::Hz48000, FrameDuration::Ms10, 1, 100);
    let mut dec_scaler = [0 as Scaler; LC3_DEC_LENS.0];
    let mut dec_complex = [Complex::default(); LC3_DEC_LENS.1];
    let mut pcm = [0i16; LC3_MAX_CFG.frame_samples()];

    loop {
        match select(
            stream_cmd.wait(),
            select(iso_events.request(), iso_events.established()),
        )
        .await
        {
            Either::First(StreamCommand::Configure(cfg)) => {
                codec_cfg = cfg;
            }
            Either::First(StreamCommand::Stop) => {}
            Either::Second(Either::First(req)) => {
                let cis = req.cis_handle;
                info!("[iso] CIS request for 0x{:04x}", cis.0);
                if let Err(e) = iso.command_async(LeAcceptCisRequest::new(cis)).await {
                    warn!("[iso] LeAcceptCisRequest failed: {:?}", Debug2Format(&e));
                }
            }
            Either::Second(Either::Second(established)) => {
                info!(
                    "[iso] CIS established: handle 0x{:04x}, interval {} us",
                    established.handle.0,
                    established.iso_interval.as_micros()
                );
                if setup_hci_data_path(iso, established.handle).await {
                    codec_cfg = run_stream(
                        codec_cfg,
                        &mut dec_scaler,
                        &mut dec_complex,
                        &mut pcm,
                        iso_events,
                        stream_cmd,
                    )
                    .await;
                }
            }
        }
    }
}

/// Decodes incoming LC3 SDUs for one established CIS until stopped.
///
/// Scoping the [`Lc3Decoder`] borrow inside this function ensures the scratch
/// slices are cleanly released when the CIS stops before any reconfiguration.
async fn run_stream(
    mut codec_cfg: CodecConfig,
    dec_scaler: &mut [Scaler],
    dec_complex: &mut [Complex],
    pcm: &mut [i16],
    iso_events: &IsoEvents<NoopRawMutex, 2, 2, SDU_QUEUE, SDU_MAX>,
    stream_cmd: &Signal<NoopRawMutex, StreamCommand>,
) -> CodecConfig {
    let Ok(mut dec) = Lc3Decoder::new(codec_cfg, dec_scaler, dec_complex) else {
        warn!("[audio] LC3 decoder init failed");
        return codec_cfg;
    };

    let mut ok_frames: u32 = 0;
    let mut plc_frames: u32 = 0;

    loop {
        let sdu_timeout = Duration::from_micros(u64::from(codec_cfg.frame_duration.micros()) + 4_000);
        match select(stream_cmd.wait(), with_timeout(sdu_timeout, iso_events.received())).await {
            Either::First(StreamCommand::Stop) => {
                info!("[iso] stream stopped (ok={}, plc={})", ok_frames, plc_frames);
                return codec_cfg;
            }
            Either::First(StreamCommand::Configure(cfg)) => {
                codec_cfg = cfg;
            }
            Either::Second(recv) => {
                if decode_or_conceal(&mut dec, recv.ok().as_ref(), pcm) {
                    ok_frames += 1;
                } else {
                    plc_frames += 1;
                }
                if (ok_frames + plc_frames).is_multiple_of(100) {
                    info!("[audio] decoded={}, plc={}", ok_frames, plc_frames);
                }
            }
        }
    }
}

async fn setup_hci_data_path<C: SinkController, P: PacketPool>(
    iso: &trouble_host::iso::Iso<'_, C, P>,
    cis_handle: ConnHandle,
) -> bool {
    let lc3 = HciCodecId {
        coding_format: 0x06,
        company_id: 0x0000,
        vendor_specific_codec_id: 0x0000,
    };
    if let Err(e) = iso
        .command(LeSetupIsoDataPath::new(
            cis_handle,
            DataPathDirection::Output,
            DataPathId::HCI,
            lc3,
            ExtDuration::from_micros(0),
            &[],
        ))
        .await
    {
        warn!("[iso] LeSetupIsoDataPath failed: {:?}", Debug2Format(&e));
        return false;
    }
    true
}

/// Decode a valid complete SDU, or invoke LC3 PLC on lost/corrupt/timed-out frames.
///
/// Returns `true` when decoded from a valid controller SDU, `false` when PLC ran.
fn decode_or_conceal(dec: &mut Lc3Decoder<'_>, sdu: Option<&IsoSdu<SDU_MAX>>, pcm: &mut [i16]) -> bool {
    match sdu {
        Some(pkt) if pkt.is_valid() && pkt.len > 0 => {
            if dec.decode(pkt.as_slice(), pcm).is_ok() {
                return true;
            }
            let _ = dec.decode_lost(pcm);
            false
        }
        _ => {
            let _ = dec.decode_lost(pcm);
            false
        }
    }
}
