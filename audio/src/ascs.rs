//! ASE Control Point (ASCS) framing and multi-ASE orchestration.
//!
//! The peer drives the Audio Stream Endpoints with operations written to the ASE
//! Control Point characteristic, and the server answers each one with an
//! indication. The framing is fixed, with any operation parameters appended
//! after the ASE IDs:
//!
//! ```text
//! Request:  Opcode(1) | Number_of_ASEs(1) | ASE_ID[n](1 each) | parameters...
//! Response: Opcode(1) | Number_of_ASEs(1) | (ASE_ID, Response_Code, Reason)[n]
//! ```
//!
//! A response carries [`RESPONSE_OPCODE`] in the opcode field, which is what
//! distinguishes it from a request. `Reason` carries a parameter-specific reason
//! when the code is one of the rejected or invalid codec, QoS or metadata codes,
//! and is zero otherwise.
//!
//! # Multi-ASE operations
//!
//! One operation may address several ASEs, and BAP requires it to be atomic: if
//! any addressed ASE cannot accept it, none of them may change state. Every
//! target is therefore checked with [`Ase::can_handle`] before any state change,
//! and on failure *every* ASE in the operation carries the failing code — the
//! resulting response is per-ASE, but with a single code for the operation, as
//! the specification describes for an atomic operation.
//!
//! # Framing
//!
//! Four operations carry a plain list of ASE IDs — `Receiver Start Ready`,
//! `Disable`, `Receiver Stop Ready` and `Release`:
//!
//! ```text
//! Opcode | Number_of_ASEs | ASE_ID[n]
//! ```
//!
//! The other four — `Config Codec`, `Config QoS`, `Enable` and `Update Metadata`
//! — interleave each ASE ID with that ASE's own parameters, because those
//! parameters can differ from one ASE to the next. See [`ControlPointRequest`]
//! for the layouts.
//!
//! Treating the two the same would mis-read a parameter octet as a second
//! `ASE_ID`, so the framing is opcode-aware: [`ControlPointRequest::ase_ids`]
//! knows which layout to walk. The layouts follow Zephyr's `ascs_internal.h`.
//!
//! # What is validated
//!
//! The framing, the ASE states, and for `Config Codec` the codec itself, the
//! well-formedness of its configuration, and whether every parameter it carries
//! is within the advertised capabilities. See [`validate_codec_config`] for the
//! order those checks run in.
//!
//! Still missing: which parameters a configuration is *required* to carry (a
//! sampling frequency and a frame duration are mandatory, but an incomplete
//! configuration is currently accepted), and any judgement of the QoS and
//! metadata parameters. Those are parsed into [`QosConfigBlock`] and
//! [`MetadataBlock`] but not checked against anything.

use crate::ase::{Ase, AseDirection, AseOperation, AseResponse};
use crate::bap::{rate_from_index, PacRecord};
use crate::ltv::{self, LtvIter};
use crate::types::{AudioLocation, CodecId};

/// Number of ASEs a single operation may address.
pub const MAX_ASES_PER_OPERATION: usize = 31;

/// The opcode carried by a response, which is never a valid request opcode.
pub const RESPONSE_OPCODE: u8 = 0x00;

/// Bytes a response uses per addressed ASE.
pub const RESPONSE_ENTRY_LEN: usize = 3;

/// Bytes a response uses before the per-ASE entries.
pub const RESPONSE_HEADER_LEN: usize = 2;

/// Error raised while handling a request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AscsError {
    /// The request could not be parsed. The carried code belongs in the response
    /// sent back to the peer.
    Response(AseResponse),
    /// The supplied buffer is too small to hold the response.
    BufferTooSmall,
}

impl From<AseResponse> for AscsError {
    fn from(code: AseResponse) -> Self {
        AscsError::Response(code)
    }
}

/// The parameter a rejected or invalid configuration was refused over.
///
/// This is the `Reason` octet of a response. The values are ST's, from
/// `audio_types.h`. The specification defines a reason for the rejected and
/// invalid codec, QoS and metadata codes, so every other code carries
/// [`Reason::None`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    /// No reason, or a code that does not carry one.
    None,
    /// The codec itself.
    CodecId,
    /// A codec-specific configuration parameter.
    CodecSpecificConfiguration,
    /// The SDU interval.
    SduInterval,
    /// The ISOAL framing mode.
    Framing,
    /// The PHY.
    Phy,
    /// The maximum SDU size.
    MaxSdu,
    /// The retransmission number.
    RetransmissionNumber,
    /// The maximum transport latency.
    MaxTransportLatency,
    /// The presentation delay.
    PresentationDelay,
}

impl Reason {
    /// The wire value.
    pub const fn to_u8(self) -> u8 {
        match self {
            Reason::None => 0x00,
            Reason::CodecId => 0x01,
            Reason::CodecSpecificConfiguration => 0x02,
            Reason::SduInterval => 0x03,
            Reason::Framing => 0x04,
            Reason::Phy => 0x05,
            Reason::MaxSdu => 0x06,
            Reason::RetransmissionNumber => 0x07,
            Reason::MaxTransportLatency => 0x08,
            Reason::PresentationDelay => 0x09,
        }
    }

    /// The reason to report alongside `code`.
    ///
    /// A malformed or out-of-range codec configuration is what this module
    /// reports today, so it names the codec-specific configuration. The QoS and
    /// metadata codes will need their own reason once those parameters are
    /// validated — and that reason has to name the parameter actually at fault,
    /// which is why one is not guessed at here.
    pub const fn for_code(code: AseResponse) -> Self {
        match code {
            AseResponse::RejectedCodecConfiguration | AseResponse::InvalidCodecConfiguration => {
                Reason::CodecSpecificConfiguration
            }
            _ => Reason::None,
        }
    }
}

/// A parsed ASE Control Point request.
///
/// The operations do not share one framing. Four of them — `Receiver Start
/// Ready`, `Disable`, `Receiver Stop Ready` and `Release` — are a plain list of
/// ASE IDs:
///
/// ```text
/// Opcode | Number_of_ASEs | ASE_ID[n]
/// ```
///
/// The other four interleave each ASE ID with that ASE's own parameters, because
/// those parameters can differ from one ASE to the next:
///
/// ```text
/// Config Codec    | Opcode | N | ASE_ID | Target_Latency | Target_PHY | Codec_ID(5) | Len | Config
/// Config QoS      | Opcode | N | ASE_ID | CIG_ID | CIS_ID | SDU_Interval(3) | Framing | PHY
///                              | Max_SDU(2) | RTN | Max_Transport_Latency(2) | Presentation_Delay(3)
/// Enable          | Opcode | N | ASE_ID | Len | Metadata
/// Update Metadata | Opcode | N | ASE_ID | Len | Metadata
/// ```
///
/// Treating an interleaved operation as a plain list mis-reads a parameter octet
/// as a second ASE ID, so the framing is walked per opcode. The layouts follow
/// Zephyr's `ascs_internal.h`, which is a mature implementation of the same
/// specification and states them as packed structs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControlPointRequest<'a> {
    op: AseOperation,
    count: u8,
    body: &'a [u8],
}

impl<'a> ControlPointRequest<'a> {
    /// Parse and validate a request written to the ASE Control Point.
    ///
    /// This checks the opcode, the ASE count and the framing of the operation
    /// body, so the accessors below cannot fail. The *contents* — the codec, QoS
    /// and metadata parameters — are validated separately, because only the
    /// operation that carries them can judge them.
    pub fn parse(data: &'a [u8]) -> Result<Self, AseResponse> {
        let (&raw_op, rest) = data.split_first().ok_or(AseResponse::InvalidLength)?;
        let op = AseOperation::from_u8(raw_op).ok_or(AseResponse::UnsupportedOpcode)?;

        let (&count, body) = rest.split_first().ok_or(AseResponse::InvalidLength)?;
        if count == 0 || count as usize > MAX_ASES_PER_OPERATION {
            return Err(AseResponse::InvalidLength);
        }

        let req = Self { op, count, body };

        // Walk the body now, so that every later access is total.
        match op {
            AseOperation::ConfigCodec => {
                for block in req.config_codec_blocks() {
                    block?;
                }
            }
            AseOperation::ConfigQos => {
                for block in req.config_qos_blocks() {
                    block?;
                }
            }
            AseOperation::Enable | AseOperation::UpdateMetadata => {
                for block in req.metadata_blocks() {
                    block?;
                }
            }
            _ => {
                if req.body.len() < req.count as usize {
                    return Err(AseResponse::InvalidLength);
                }
            }
        }

        Ok(req)
    }

    /// The requested operation.
    pub const fn op(&self) -> AseOperation {
        self.op
    }

    /// How many ASEs the operation addresses.
    pub const fn count(&self) -> u8 {
        self.count
    }

    /// The ASE IDs the operation addresses, in the order given.
    ///
    /// Walks whichever framing the opcode uses, so the IDs of an interleaved
    /// operation come from its per-ASE blocks.
    pub fn ase_ids(&self) -> AseIdList {
        let mut ids = AseIdList::new();

        match self.op {
            AseOperation::ConfigCodec => {
                for block in self.config_codec_blocks().flatten() {
                    ids.push(block.ase_id);
                }
            }
            AseOperation::ConfigQos => {
                for block in self.config_qos_blocks().flatten() {
                    ids.push(block.ase_id);
                }
            }
            AseOperation::Enable | AseOperation::UpdateMetadata => {
                for block in self.metadata_blocks().flatten() {
                    ids.push(block.ase_id);
                }
            }
            _ => {
                for &id in &self.body[..self.count as usize] {
                    ids.push(id);
                }
            }
        }

        ids
    }

    /// The per-ASE entries of a `Config Codec` operation.
    pub fn config_codec_blocks(&self) -> CodecConfigBlocks<'a> {
        if self.op != AseOperation::ConfigCodec {
            return CodecConfigBlocks {
                rest: &[],
                remaining: 0,
            };
        }
        CodecConfigBlocks {
            rest: self.body,
            remaining: self.count as usize,
        }
    }

    /// The per-ASE entries of a `Config QoS` operation.
    pub fn config_qos_blocks(&self) -> QosConfigBlocks<'a> {
        if self.op != AseOperation::ConfigQos {
            return QosConfigBlocks {
                rest: &[],
                remaining: 0,
            };
        }
        QosConfigBlocks {
            rest: self.body,
            remaining: self.count as usize,
        }
    }

    /// The per-ASE metadata of an `Enable` or `Update Metadata` operation.
    pub fn metadata_blocks(&self) -> MetadataBlocks<'a> {
        if !matches!(self.op, AseOperation::Enable | AseOperation::UpdateMetadata) {
            return MetadataBlocks {
                rest: &[],
                remaining: 0,
            };
        }
        MetadataBlocks {
            rest: self.body,
            remaining: self.count as usize,
        }
    }

    /// The operation parameters that follow a plain ASE ID list.
    ///
    /// The four operations that use that framing carry none, so this is empty —
    /// including for the interleaved operations, whose parameters are reached
    /// through their block iterators instead.
    pub fn params(&self) -> &'a [u8] {
        match self.op {
            AseOperation::ConfigCodec
            | AseOperation::ConfigQos
            | AseOperation::Enable
            | AseOperation::UpdateMetadata => &[],
            _ => {
                let n = self.count as usize;
                if self.body.len() < n {
                    &[]
                } else {
                    &self.body[n..]
                }
            }
        }
    }
}

/// The ASE IDs an operation addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AseIdList {
    ids: [u8; MAX_ASES_PER_OPERATION],
    len: usize,
}

impl AseIdList {
    const fn new() -> Self {
        Self {
            ids: [0; MAX_ASES_PER_OPERATION],
            len: 0,
        }
    }

    fn push(&mut self, id: u8) {
        if self.len < MAX_ASES_PER_OPERATION {
            self.ids[self.len] = id;
            self.len += 1;
        }
    }

    /// The IDs, in order.
    pub fn as_slice(&self) -> &[u8] {
        &self.ids[..self.len]
    }

    /// How many IDs there are.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether the operation addresses no ASE.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// One ASE's entry in a `Config Codec` operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CodecConfigBlock<'a> {
    /// The ASE being configured.
    pub ase_id: u8,
    /// The peer's target latency, in milliseconds.
    pub target_latency_ms: u8,
    /// The peer's target PHY.
    pub target_phy: u8,
    /// The codec the peer asks for.
    pub codec_id: CodecId,
    /// The codec-specific configuration, as an LTV block.
    pub config: &'a [u8],
}

/// Iterator over the per-ASE entries of a `Config Codec` operation.
#[derive(Debug, Clone, Copy)]
pub struct CodecConfigBlocks<'a> {
    rest: &'a [u8],
    remaining: usize,
}

impl<'a> Iterator for CodecConfigBlocks<'a> {
    type Item = Result<CodecConfigBlock<'a>, AseResponse>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        self.remaining -= 1;

        // ASE_ID (1), target latency (1), target PHY (1), Codec_ID (5),
        // configuration length (1), configuration (L).
        const HEADER: usize = 1 + 1 + 1 + CodecId::SIZE + 1;
        if self.rest.len() < HEADER {
            return Some(Err(AseResponse::InvalidLength));
        }

        let ase_id = self.rest[0];
        let target_latency_ms = self.rest[1];
        let target_phy = self.rest[2];

        let mut codec_bytes = [0u8; CodecId::SIZE];
        codec_bytes.copy_from_slice(&self.rest[3..3 + CodecId::SIZE]);
        let codec_id = CodecId::decode(&codec_bytes);

        let len = self.rest[3 + CodecId::SIZE] as usize;
        let after = &self.rest[HEADER..];
        if after.len() < len {
            return Some(Err(AseResponse::InvalidLength));
        }

        let (config, tail) = after.split_at(len);
        self.rest = tail;

        Some(Ok(CodecConfigBlock {
            ase_id,
            target_latency_ms,
            target_phy,
            codec_id,
            config,
        }))
    }
}

/// One ASE's entry in a `Config QoS` operation.
///
/// `framing` and `phy` are the raw wire octets; decoding them belongs with the
/// QoS validation, which is not written yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QosConfigBlock {
    /// The ASE being configured.
    pub ase_id: u8,
    /// The CIG the peer asks to place the CIS in.
    pub cig_id: u8,
    /// The CIS the peer asks for.
    pub cis_id: u8,
    /// The SDU interval, in microseconds. 24-bit on the wire.
    pub sdu_interval_us: u32,
    /// The ISOAL framing mode.
    pub framing: u8,
    /// The PHY bitfield.
    pub phy: u8,
    /// The maximum SDU size, in octets.
    pub max_sdu: u16,
    /// The retransmission number.
    pub rtn: u8,
    /// The maximum transport latency, in milliseconds.
    pub max_transport_latency_ms: u16,
    /// The presentation delay, in microseconds. 24-bit on the wire.
    pub presentation_delay_us: u32,
}

/// Iterator over the per-ASE entries of a `Config QoS` operation.
#[derive(Debug, Clone, Copy)]
pub struct QosConfigBlocks<'a> {
    rest: &'a [u8],
    remaining: usize,
}

impl Iterator for QosConfigBlocks<'_> {
    type Item = Result<QosConfigBlock, AseResponse>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        self.remaining -= 1;

        // ASE_ID (1), CIG_ID (1), CIS_ID (1), SDU interval (3), framing (1),
        // PHY (1), max SDU (2), RTN (1), max transport latency (2),
        // presentation delay (3).
        const BLOCK: usize = 1 + 1 + 1 + 3 + 1 + 1 + 2 + 1 + 2 + 3;
        if self.rest.len() < BLOCK {
            return Some(Err(AseResponse::InvalidLength));
        }

        let b = &self.rest[..BLOCK];
        let block = QosConfigBlock {
            ase_id: b[0],
            cig_id: b[1],
            cis_id: b[2],
            sdu_interval_us: u32::from_le_bytes([b[3], b[4], b[5], 0]),
            framing: b[6],
            phy: b[7],
            max_sdu: u16::from_le_bytes([b[8], b[9]]),
            rtn: b[10],
            max_transport_latency_ms: u16::from_le_bytes([b[11], b[12]]),
            presentation_delay_us: u32::from_le_bytes([b[13], b[14], b[15], 0]),
        };

        self.rest = &self.rest[BLOCK..];
        Some(Ok(block))
    }
}

/// One ASE's metadata, in an `Enable` or `Update Metadata` operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MetadataBlock<'a> {
    /// The ASE the metadata applies to.
    pub ase_id: u8,
    /// The metadata, as an LTV block. May be empty.
    pub metadata: &'a [u8],
}

/// Iterator over the per-ASE metadata of an `Enable` or `Update Metadata`
/// operation.
#[derive(Debug, Clone, Copy)]
pub struct MetadataBlocks<'a> {
    rest: &'a [u8],
    remaining: usize,
}

impl<'a> Iterator for MetadataBlocks<'a> {
    type Item = Result<MetadataBlock<'a>, AseResponse>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        self.remaining -= 1;

        // ASE_ID (1), metadata length (1), metadata (L).
        if self.rest.len() < 2 {
            return Some(Err(AseResponse::InvalidLength));
        }

        let ase_id = self.rest[0];
        let len = self.rest[1] as usize;
        let after = &self.rest[2..];
        if after.len() < len {
            return Some(Err(AseResponse::InvalidLength));
        }

        let (metadata, tail) = after.split_at(len);
        self.rest = tail;

        Some(Ok(MetadataBlock { ase_id, metadata }))
    }
}

/// One ASE's outcome within a response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResponseEntry {
    /// The ASE this entry refers to.
    pub ase_id: u8,
    /// The response code for the operation on that ASE.
    pub code: AseResponse,
    /// Parameter-specific reason, or zero.
    pub reason: u8,
}

/// An ASE Control Point response, ready to be indicated to the peer.
#[derive(Debug, Clone)]
pub struct ControlPointResponse {
    entries: [ResponseEntry; MAX_ASES_PER_OPERATION],
    len: usize,
}

impl Default for ControlPointResponse {
    fn default() -> Self {
        Self::new()
    }
}

impl ControlPointResponse {
    /// Create an empty response.
    pub const fn new() -> Self {
        Self {
            entries: [ResponseEntry {
                ase_id: 0,
                code: AseResponse::Success,
                reason: 0,
            }; MAX_ASES_PER_OPERATION],
            len: 0,
        }
    }

    /// Append an entry. Returns `false` if the response is already full.
    pub fn push(&mut self, ase_id: u8, code: AseResponse, reason: u8) -> bool {
        if self.len == MAX_ASES_PER_OPERATION {
            return false;
        }
        self.entries[self.len] = ResponseEntry { ase_id, code, reason };
        self.len += 1;
        true
    }

    /// Number of entries.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether there are no entries.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The entries.
    pub fn entries(&self) -> &[ResponseEntry] {
        &self.entries[..self.len]
    }

    /// Bytes this response occupies on the wire.
    pub const fn wire_len(&self) -> usize {
        RESPONSE_HEADER_LEN + self.len * RESPONSE_ENTRY_LEN
    }

    /// Serialise into `out`, returning the number of bytes written.
    pub fn write(&self, out: &mut [u8]) -> Result<usize, AscsError> {
        let needed = self.wire_len();
        if out.len() < needed {
            return Err(AscsError::BufferTooSmall);
        }

        out[0] = RESPONSE_OPCODE;
        out[1] = self.len as u8;
        for (i, entry) in self.entries[..self.len].iter().enumerate() {
            let at = RESPONSE_HEADER_LEN + i * RESPONSE_ENTRY_LEN;
            out[at] = entry.ase_id;
            out[at + 1] = entry.code.to_u8();
            out[at + 2] = entry.reason;
        }

        Ok(needed)
    }
}

/// The codec capabilities a server advertises, by ASE direction.
#[derive(Debug, Clone, Copy)]
pub struct CodecCapabilities<'a> {
    /// PAC records for Sink ASEs — audio the server receives.
    pub sink: &'a [PacRecord<'a>],
    /// PAC records for Source ASEs — audio the server sends.
    pub source: &'a [PacRecord<'a>],
}

impl<'a> CodecCapabilities<'a> {
    /// A server advertising nothing, which therefore accepts no codec.
    pub const NONE: Self = Self { sink: &[], source: &[] };

    fn records(&self, direction: AseDirection) -> &'a [PacRecord<'a>] {
        match direction {
            AseDirection::Sink => self.sink,
            AseDirection::Source => self.source,
        }
    }
}

/// Validate one `Config Codec` entry against the advertised capabilities.
///
/// The order of the checks matters, because more than one can apply to the same
/// entry:
///
/// 1. The codec itself must be advertised for the addressed ASE's direction,
///    otherwise [`AseResponse::UnsupportedAudioCapability`]. This comes first:
///    there is no point judging a configuration for a codec the server does not
///    have.
/// 2. The codec-specific configuration must be well-formed LTV, otherwise
///    [`AseResponse::InvalidCodecConfiguration`]. This is judged against the
///    codec, not against what the server supports, so it takes precedence over
///    the check below.
/// 3. Each entry must be within the advertised codec-specific capabilities,
///    otherwise [`AseResponse::UnsupportedCodecConfiguration`] — see
///    [`config_within_capabilities`] for the rule each parameter uses.
///
/// What is *not* checked yet is which parameters a configuration has to carry. A
/// codec configuration must specify at least a sampling frequency and a frame
/// duration, but an incomplete one is currently accepted as long as everything it
/// does carry is supported.
pub fn validate_codec_config(
    block: &CodecConfigBlock<'_>,
    direction: AseDirection,
    caps: &CodecCapabilities<'_>,
) -> AseResponse {
    let Some(record) = caps.records(direction).iter().find(|r| r.codec_id == block.codec_id) else {
        return AseResponse::UnsupportedAudioCapability;
    };

    // Well-formedness is judged against the codec, not against the server, so it
    // precedes the capability comparison below.
    if block.config.is_empty() || LtvIter::new(block.config).is_err() {
        return AseResponse::InvalidCodecConfiguration;
    }

    config_within_capabilities(block.config, record.codec_specific_capabilities)
}

/// Compare a well-formed codec configuration against the advertised capabilities.
///
/// Each parameter needs its own rule, because the two sides are not encoded the
/// same way:
///
/// * a configured *sampling frequency* is an ordinal (`0x08` = 48 kHz) while the
///   capability is a bitfield (`0x0080`), so the ordinal is translated to a bit
///   before the capability is tested;
/// * a configured *frame duration* is a bitfield on both sides, so the requested
///   bit has to be present in the capability;
/// * a configured *channel allocation* names physical loudspeaker locations while
///   the capability only advertises how many channels are supported, so this
///   compares the number of allocated channels rather than the locations;
/// * *octets per codec frame* is a single value against a minimum and a maximum;
/// * *codec frames per SDU* is a maximum, so any value up to it is acceptable.
///
/// A parameter this implementation does not know, or one the capability does not
/// advertise, is [`AseResponse::UnsupportedCodecConfiguration`].
///
/// `Supported_Audio_Channel_Counts` is read as `bit (n - 1)` meaning `n`
/// channels: there is no such thing as a stream with zero channels, so bit 0 has
/// to mean one channel. That convention should be confirmed against the
/// specification, since getting it backwards would reject valid configurations.
fn config_within_capabilities(config: &[u8], caps: &[u8]) -> AseResponse {
    use crate::ltv::{cap, cfg};

    let Ok(entries) = LtvIter::new(config) else {
        return AseResponse::InvalidCodecConfiguration;
    };

    for entry in entries {
        match entry.ty {
            cfg::SAMPLING_FREQUENCY => {
                let Some(ordinal) = entry.value.first().copied() else {
                    return AseResponse::InvalidCodecConfiguration;
                };
                let Some(rate) = rate_from_index(ordinal) else {
                    return AseResponse::InvalidCodecConfiguration;
                };
                match ltv::find(caps, cap::SUPPORTED_SAMPLING_FREQUENCIES).and_then(|e| e.as_u16()) {
                    Some(supported) if supported & rate.bit() != 0 => {}
                    _ => return AseResponse::UnsupportedCodecConfiguration,
                }
            }

            cfg::FRAME_DURATION => {
                let Some(requested) = entry.value.first().copied() else {
                    return AseResponse::InvalidCodecConfiguration;
                };
                let supported = ltv::find(caps, cap::SUPPORTED_FRAME_DURATIONS).and_then(|e| e.value.first().copied());
                match supported {
                    Some(bits) if bits & requested != 0 => {}
                    _ => return AseResponse::UnsupportedCodecConfiguration,
                }
            }

            cfg::AUDIO_CHANNEL_ALLOCATION => {
                let Some(value) = entry.as_u32() else {
                    return AseResponse::InvalidCodecConfiguration;
                };
                let channels = AudioLocation(value).count();
                let supported = ltv::find(caps, cap::SUPPORTED_AUDIO_CHANNEL_COUNTS).and_then(|e| e.as_u32());
                match supported {
                    Some(bits) if (1..=32).contains(&channels) => {
                        if bits & (1u32 << (channels - 1)) == 0 {
                            return AseResponse::UnsupportedCodecConfiguration;
                        }
                    }
                    _ => return AseResponse::UnsupportedCodecConfiguration,
                }
            }

            cfg::OCTETS_PER_CODEC_FRAME => {
                let Some(value) = entry.as_u16() else {
                    return AseResponse::InvalidCodecConfiguration;
                };
                let range = ltv::find(caps, cap::SUPPORTED_OCTETS_PER_CODEC_FRAME).and_then(|e| {
                    if e.value.len() >= 4 {
                        Some((
                            u16::from_le_bytes([e.value[0], e.value[1]]),
                            u16::from_le_bytes([e.value[2], e.value[3]]),
                        ))
                    } else {
                        None
                    }
                });
                match range {
                    Some((min, max)) if value >= min && value <= max => {}
                    _ => return AseResponse::UnsupportedCodecConfiguration,
                }
            }

            cfg::CODEC_FRAMES_PER_SDU => {
                let Some(value) = entry.value.first().copied() else {
                    return AseResponse::InvalidCodecConfiguration;
                };
                let supported =
                    ltv::find(caps, cap::SUPPORTED_MAX_CODEC_FRAMES_PER_SDU).and_then(|e| e.value.first().copied());
                match supported {
                    Some(max) if value >= 1 && value <= max => {}
                    _ => return AseResponse::UnsupportedCodecConfiguration,
                }
            }

            // A parameter this implementation does not know cannot be honoured.
            _ => return AseResponse::UnsupportedCodecConfiguration,
        }
    }

    AseResponse::Success
}

/// The code that prevents the whole operation from being applied, if any.
///
/// Several things can reject an operation, and the order they are checked in
/// decides which code the peer sees when more than one applies:
///
/// 1. an ASE ID that does not exist, so an unknown ID is never reported as a
///    state or parameter error;
/// 2. an ASE that cannot accept the operation in its current state;
/// 3. a `Config Codec` entry that the server cannot accept.
///
/// Nothing is changed here; this only decides.
fn rejection(ases: &[Ase], caps: &CodecCapabilities<'_>, req: &ControlPointRequest<'_>) -> Option<AseResponse> {
    let ids = req.ase_ids();

    if ids.as_slice().iter().any(|id| !ases.iter().any(|a| a.id() == *id)) {
        return Some(AseResponse::InvalidAseId);
    }

    for &id in ids.as_slice() {
        // Resolved by the check above.
        let Some(ase) = ases.iter().find(|a| a.id() == id) else {
            continue;
        };

        let code = ase.can_handle(req.op());
        if !code.is_success() {
            return Some(code);
        }
    }

    if req.op() == AseOperation::ConfigCodec {
        for block in req.config_codec_blocks() {
            // Framing was validated by `parse`, and the ASE ID by the first loop.
            let Ok(block) = block else {
                continue;
            };
            let Some(ase) = ases.iter().find(|a| a.id() == block.ase_id) else {
                continue;
            };

            let code = validate_codec_config(&block, ase.direction(), caps);
            if !code.is_success() {
                return Some(code);
            }
        }
    }

    None
}

/// Apply a parsed request to `ases` and build the response to indicate back.
///
/// The operation is atomic across the ASEs it addresses: either every one of
/// them accepts it and changes state, or none of them changes and every entry
/// carries the failing code.
///
/// `caps` are the codec capabilities this server advertises, used to validate a
/// `Config Codec` operation.
///
/// A `Release` leaves the ASEs in [`AseState::Releasing`](crate::AseState); the
/// application is responsible for finishing it with
/// [`Ase::complete_release`] once any associated CIS has gone, and likewise for
/// [`Ase::complete_disable`] on a Source ASE.
pub fn apply(ases: &mut [Ase], caps: &CodecCapabilities<'_>, req: &ControlPointRequest<'_>) -> ControlPointResponse {
    let mut resp = ControlPointResponse::new();
    let rejected = rejection(ases, caps, req);

    for &id in req.ase_ids().as_slice() {
        let code = match rejected {
            Some(code) => code,
            None => {
                // Resolved by `rejection` above, so this cannot fail.
                if let Some(ase) = ases.iter_mut().find(|a| a.id() == id) {
                    let _ = ase.handle(req.op());
                }
                AseResponse::Success
            }
        };
        resp.push(id, code, Reason::for_code(code).to_u8());
    }

    resp
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ase::AseDirection;
    use crate::types::CodecId;

    extern crate std;
    use std::vec::Vec;

    /// A sink PAC record advertising LC3.
    fn lc3_caps() -> [PacRecord<'static>; 1] {
        [PacRecord {
            codec_id: CodecId::LC3,
            // Supported_Sampling_Frequencies, bit for 48 kHz.
            codec_specific_capabilities: &[0x03, 0x01, 0x80, 0x00],
            metadata: &[],
        }]
    }

    fn with_sink<'a>(records: &'a [PacRecord<'a>]) -> CodecCapabilities<'a> {
        CodecCapabilities {
            sink: records,
            source: &[],
        }
    }

    /// The LC3 codec id: coding format 0x06, no company or vendor code.
    const LC3: [u8; 5] = [0x06, 0x00, 0x00, 0x00, 0x00];

    /// A codec-specific configuration: 48 kHz.
    const CC_48K: &[u8] = &[0x02, 0x01, 0x08];

    /// Build a `Config Codec` request, one entry per `(ase_id, codec, config)`.
    ///
    /// Each entry is `ASE_ID | Target_Latency | Target_PHY | Codec_ID(5) | Len |
    /// Config`, interleaved rather than listed.
    fn config_codec(entries: &[(u8, [u8; 5], &[u8])]) -> Vec<u8> {
        let mut v = std::vec![0x01, entries.len() as u8];
        for (ase_id, codec, cc) in entries {
            v.push(*ase_id);
            v.push(0x01); // target latency, milliseconds
            v.push(0x02); // target PHY: LE 2M
            v.extend_from_slice(codec);
            v.push(cc.len() as u8);
            v.extend_from_slice(cc);
        }
        v
    }

    /// Build a `Config QoS` request, one entry per `(ase_id, cig_id, cis_id)`.
    ///
    /// Each entry is `ASE_ID | CIG_ID | CIS_ID | SDU_Interval(3) | Framing | PHY
    /// | Max_SDU(2) | RTN | Max_Transport_Latency(2) | Presentation_Delay(3)`,
    /// interleaved rather than shared.
    fn config_qos(entries: &[(u8, u8, u8)]) -> Vec<u8> {
        let mut v = std::vec![0x02, entries.len() as u8];
        for (ase_id, cig, cis) in entries {
            v.push(*ase_id);
            v.push(*cig);
            v.push(*cis);
            v.extend_from_slice(&[0x10, 0x27, 0x00]); // SDU interval: 10 000 us
            v.push(0x00); // unframed
            v.push(0x02); // LE 2M
            v.extend_from_slice(&40u16.to_le_bytes()); // max SDU
            v.push(2); // RTN
            v.extend_from_slice(&20u16.to_le_bytes()); // max transport latency, ms
            v.extend_from_slice(&[0x40, 0x9C, 0x00]); // presentation delay: 40 000 us
        }
        v
    }

    #[test]
    fn parses_a_single_ase_request() {
        // Release ASE 0.
        let req = ControlPointRequest::parse(&[0x08, 0x01, 0x00]).unwrap();
        assert_eq!(req.op(), AseOperation::Release);
        assert_eq!(req.count(), 1);
        assert_eq!(req.ase_ids().as_slice(), &[0x00]);
        assert!(req.params().is_empty());
    }

    #[test]
    fn config_qos_interleaves_each_ase_with_its_parameters() {
        let data = config_qos(&[(0x03, 0x00, 0x00), (0x04, 0x00, 0x01)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        assert_eq!(req.op(), AseOperation::ConfigQos);
        assert_eq!(req.ase_ids().as_slice(), &[0x03, 0x04]);
        // The QoS parameters are per ASE, so there is no shared block.
        assert!(req.params().is_empty());

        let mut blocks = req.config_qos_blocks();
        let first = blocks.next().unwrap().unwrap();
        assert_eq!(first.ase_id, 0x03);
        assert_eq!(first.cig_id, 0x00);
        assert_eq!(first.cis_id, 0x00);
        assert_eq!(first.sdu_interval_us, 10_000);
        assert_eq!(first.framing, 0x00);
        assert_eq!(first.phy, 0x02);
        assert_eq!(first.max_sdu, 40);
        assert_eq!(first.rtn, 2);
        assert_eq!(first.max_transport_latency_ms, 20);
        assert_eq!(first.presentation_delay_us, 40_000);

        let second = blocks.next().unwrap().unwrap();
        assert_eq!(second.ase_id, 0x04);
        assert_eq!(second.cis_id, 0x01);
        assert!(blocks.next().is_none());
    }

    #[test]
    fn config_codec_interleaves_each_ase_with_its_configuration() {
        let data = config_codec(&[(0x00, LC3, CC_48K), (0x01, LC3, CC_48K)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        assert_eq!(req.op(), AseOperation::ConfigCodec);
        assert_eq!(req.ase_ids().as_slice(), &[0x00, 0x01]);
        // Config Codec has no shared parameters either.
        assert!(req.params().is_empty());

        let mut blocks = req.config_codec_blocks();
        let first = blocks.next().unwrap().unwrap();
        assert_eq!(first.ase_id, 0x00);
        assert_eq!(first.target_latency_ms, 0x01);
        assert_eq!(first.target_phy, 0x02);
        assert_eq!(first.codec_id, CodecId::LC3);
        assert_eq!(first.config, CC_48K);

        let second = blocks.next().unwrap().unwrap();
        assert_eq!(second.ase_id, 0x01);
        assert!(blocks.next().is_none());
    }

    #[test]
    fn rejects_malformed_requests() {
        assert_eq!(ControlPointRequest::parse(&[]), Err(AseResponse::InvalidLength));
        // Response opcode 0x00 is not a valid request opcode.
        assert_eq!(
            ControlPointRequest::parse(&[0x00, 0x01, 0x00]),
            Err(AseResponse::UnsupportedOpcode)
        );
        assert_eq!(
            ControlPointRequest::parse(&[0x09, 0x01, 0x00]),
            Err(AseResponse::UnsupportedOpcode)
        );
        // No ASEs.
        assert_eq!(
            ControlPointRequest::parse(&[0x08, 0x00]),
            Err(AseResponse::InvalidLength)
        );
        // Truncated ASE ID list.
        assert_eq!(
            ControlPointRequest::parse(&[0x08, 0x02, 0x00]),
            Err(AseResponse::InvalidLength)
        );
    }

    #[test]
    fn rejects_a_truncated_config_codec_block() {
        // No room for even the fixed part of a block.
        assert_eq!(
            ControlPointRequest::parse(&[0x01, 0x01, 0x00]),
            Err(AseResponse::InvalidLength)
        );

        // A block that claims four octets of configuration and supplies two.
        let mut data = config_codec(&[(0x00, LC3, &[])]);
        let len_at = data.len() - 1;
        data[len_at] = 0x04;
        data.extend_from_slice(&[0xAA, 0xBB]);
        assert_eq!(ControlPointRequest::parse(&data), Err(AseResponse::InvalidLength));

        // Two ASEs announced, only one block present.
        let mut data = config_codec(&[(0x00, LC3, &[])]);
        data[1] = 0x02;
        assert_eq!(ControlPointRequest::parse(&data), Err(AseResponse::InvalidLength));
    }

    #[test]
    fn response_wire_format() {
        let mut resp = ControlPointResponse::new();
        assert!(resp.push(0x00, AseResponse::Success, 0x00));
        assert!(resp.push(0x01, AseResponse::InvalidAseState, 0x00));
        assert_eq!(resp.len(), 2);
        assert_eq!(resp.wire_len(), 2 + 2 * 3);

        let mut out = [0u8; 8];
        let n = resp.write(&mut out).unwrap();
        assert_eq!(n, 8);
        assert_eq!(out, [0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00]);
    }

    #[test]
    fn response_write_checks_the_buffer() {
        let mut resp = ControlPointResponse::new();
        resp.push(0x00, AseResponse::Success, 0x00);
        let mut too_small = [0u8; 4];
        assert_eq!(resp.write(&mut too_small), Err(AscsError::BufferTooSmall));
    }

    #[test]
    fn applies_to_every_addressed_ase() {
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink), Ase::new(1, AseDirection::Sink)];

        let data = config_codec(&[(0x00, LC3, CC_48K), (0x01, LC3, CC_48K)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);

        assert_eq!(resp.len(), 2);
        assert!(resp.entries().iter().all(|e| e.code.is_success()));
        assert_eq!(ases[0].state(), crate::AseState::CodecConfigured);
        assert_eq!(ases[1].state(), crate::AseState::CodecConfigured);
    }

    #[test]
    fn a_failing_ase_makes_the_whole_operation_atomic() {
        let mut ases = [Ase::new(0, AseDirection::Sink), Ase::new(1, AseDirection::Sink)];

        // Put ASE 0 into Codec Configured so it is ahead of ASE 1.
        ases[0].handle(AseOperation::ConfigCodec);

        // Config QoS on both: valid for ASE 0, invalid for ASE 1 (still Idle).
        let data = config_qos(&[(0x00, 0x00, 0x00), (0x01, 0x00, 0x01)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &CodecCapabilities::NONE, &req);

        // Both carry the failure, and neither ASE moved.
        assert_eq!(resp.len(), 2);
        assert!(resp.entries().iter().all(|e| e.code == AseResponse::InvalidAseState));
        assert_eq!(ases[0].state(), crate::AseState::CodecConfigured);
        assert_eq!(ases[1].state(), crate::AseState::Idle);
    }

    #[test]
    fn unknown_ase_id_is_reported_per_ase() {
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        let req = ControlPointRequest::parse(&[0x08, 0x02, 0x00, 0x07]).unwrap();
        let resp = apply(&mut ases, &CodecCapabilities::NONE, &req);

        assert_eq!(resp.entries()[0].ase_id, 0x00);
        assert_eq!(resp.entries()[1].ase_id, 0x07);
        assert!(resp.entries().iter().all(|e| e.code == AseResponse::InvalidAseId));
        // Nothing was released.
        assert_eq!(ases[0].state(), crate::AseState::Idle);
    }

    #[test]
    fn source_ase_rejects_sink_only_operations() {
        let mut ases = [Ase::new(0, AseDirection::Source)];
        // Enable is only valid once QoS is configured, so drive it there first.
        ases[0].handle(AseOperation::ConfigCodec);
        ases[0].handle(AseOperation::ConfigQos);
        ases[0].handle(AseOperation::Enable);

        let req = ControlPointRequest::parse(&[0x04, 0x01, 0x00]).unwrap();
        let resp = apply(&mut ases, &CodecCapabilities::NONE, &req);
        assert_eq!(resp.entries()[0].code, AseResponse::InvalidAseDirection);
        assert_eq!(ases[0].state(), crate::AseState::Enabling);
    }

    #[test]
    fn config_codec_rejects_an_unadvertised_codec() {
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];

        // Coding format 0x07 rather than LC3's 0x06.
        let not_lc3 = [0x07, 0x00, 0x00, 0x00, 0x00];
        let data = config_codec(&[(0x00, not_lc3, CC_48K)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);

        assert_eq!(resp.entries()[0].code, AseResponse::UnsupportedAudioCapability);
        assert_eq!(ases[0].state(), crate::AseState::Idle);
    }

    #[test]
    fn config_codec_rejects_a_malformed_configuration() {
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];

        // A configuration whose first LTV entry claims a length of five.
        let data = config_codec(&[(0x00, LC3, &[0x05, 0x01, 0x08])]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::InvalidCodecConfiguration);

        // An empty configuration is not usable for LC3 either.
        let data = config_codec(&[(0x00, LC3, &[])]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::InvalidCodecConfiguration);
        assert_eq!(ases[0].state(), crate::AseState::Idle);
    }

    #[test]
    fn a_well_formed_configuration_passes() {
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        let data = config_codec(&[(0x00, LC3, CC_48K)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert!(resp.entries()[0].code.is_success());
        assert_eq!(ases[0].state(), crate::AseState::CodecConfigured);
    }

    #[test]
    fn config_codec_rejects_an_unsupported_parameter_value() {
        // The record advertises 48 kHz only, so 16 kHz is a well-formed
        // configuration the server cannot support: Unsupported, not Invalid.
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        // Sampling_Frequency ordinal 0x03 is 16 kHz.
        let data = config_codec(&[(0x00, LC3, &[0x02, 0x01, 0x03])]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);

        assert_eq!(resp.entries()[0].code, AseResponse::UnsupportedCodecConfiguration);
        assert_eq!(ases[0].state(), crate::AseState::Idle);
    }

    #[test]
    fn config_codec_treats_an_unknown_ordinal_as_invalid() {
        // 0x02 is not a defined sampling frequency ordinal, so the configuration
        // is wrong whatever the server supports.
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        let data = config_codec(&[(0x00, LC3, &[0x02, 0x01, 0x02])]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);

        assert_eq!(resp.entries()[0].code, AseResponse::InvalidCodecConfiguration);
    }

    #[test]
    fn config_codec_checks_octets_per_frame_against_the_advertised_range() {
        // 48 kHz, 7.5 or 10 ms, and 40..=120 octets per codec frame.
        const CAPS: &[u8] = &[
            0x03, 0x01, 0x80, 0x00, // supported sampling frequencies: 48 kHz
            0x02, 0x02, 0x03, // supported frame durations: 7.5 ms and 10 ms
            0x05, 0x04, 40, 0, 120, 0, // octets per codec frame: 40..=120
        ];
        let records = [PacRecord {
            codec_id: CodecId::LC3,
            codec_specific_capabilities: CAPS,
            metadata: &[],
        }];
        let mut ases = [Ase::new(0, AseDirection::Sink)];

        // 48 kHz, 10 ms, 40 octets: inside the advertised range.
        let inside = [0x02, 0x01, 0x08, 0x02, 0x02, 0x02, 0x03, 0x04, 40, 0];
        let data = config_codec(&[(0x00, LC3, &inside)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert!(resp.entries()[0].code.is_success());

        // The same, but 200 octets, above the advertised maximum.
        let outside = [0x02, 0x01, 0x08, 0x02, 0x02, 0x02, 0x03, 0x04, 200, 0];
        let data = config_codec(&[(0x00, LC3, &outside)]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::UnsupportedCodecConfiguration);
    }

    #[test]
    fn reason_codes_match_the_vendor_table() {
        assert_eq!(Reason::None.to_u8(), 0x00);
        assert_eq!(Reason::CodecId.to_u8(), 0x01);
        assert_eq!(Reason::CodecSpecificConfiguration.to_u8(), 0x02);
        assert_eq!(Reason::SduInterval.to_u8(), 0x03);
        assert_eq!(Reason::Framing.to_u8(), 0x04);
        assert_eq!(Reason::Phy.to_u8(), 0x05);
        assert_eq!(Reason::MaxSdu.to_u8(), 0x06);
        assert_eq!(Reason::RetransmissionNumber.to_u8(), 0x07);
        assert_eq!(Reason::MaxTransportLatency.to_u8(), 0x08);
        assert_eq!(Reason::PresentationDelay.to_u8(), 0x09);
    }

    #[test]
    fn only_invalid_and_rejected_codes_carry_a_reason() {
        // An invalid codec configuration names the codec-specific configuration.
        assert_eq!(
            Reason::for_code(AseResponse::InvalidCodecConfiguration),
            Reason::CodecSpecificConfiguration
        );
        // Codes that do not carry a reason must not invent one.
        for code in [
            AseResponse::Success,
            AseResponse::UnsupportedOpcode,
            AseResponse::InvalidLength,
            AseResponse::InvalidAseId,
            AseResponse::InvalidAseState,
            AseResponse::UnsupportedAudioCapability,
            AseResponse::UnsupportedCodecConfiguration,
            AseResponse::InsufficientResources,
        ] {
            assert_eq!(Reason::for_code(code), Reason::None);
        }
    }

    #[test]
    fn a_response_carries_the_reason_for_an_invalid_configuration() {
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];

        // A configuration whose first LTV entry claims a length of five.
        let data = config_codec(&[(0x00, LC3, &[0x05, 0x01, 0x08])]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::InvalidCodecConfiguration);
        assert_eq!(resp.entries()[0].reason, 0x02);

        // And an unsupported value carries no reason.
        let data = config_codec(&[(0x00, LC3, &[0x02, 0x01, 0x03])]);
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::UnsupportedCodecConfiguration);
        assert_eq!(resp.entries()[0].reason, 0x00);
    }
}
