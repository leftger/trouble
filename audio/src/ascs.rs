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
//! Most operations list their ASE IDs first and then a single parameter block
//! shared by all of them, because the addressed ASEs form one CIS:
//!
//! ```text
//! 0x02 | 0x02 | ASE_ID | ASE_ID | SDU_Interval_C_To_P | ... | Presentation_Delay_P_To_C
//! ```
//!
//! `Config Codec` is different: it interleaves each ASE ID with that ASE's own
//! codec configuration, so the operation can configure each ASE separately:
//!
//! ```text
//! 0x01 | 0x02 | ASE_ID | Codec_ID(5) | Len | Config | ASE_ID | Codec_ID(5) | Len | Config
//! ```
//!
//! Treating the two the same would mis-read a `Codec_ID` octet as a second
//! `ASE_ID`, so the framing is opcode-aware: [`ControlPointRequest::ase_ids`]
//! knows which layout to walk.
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
//! configuration is currently accepted), and any interpretation of a request's
//! QoS and metadata parameters. For that reason [`apply`] always writes a zero
//! `Reason`.

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

/// A parsed ASE Control Point request.
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
    /// body, so the accessors below cannot fail. The *contents* of an operation —
    /// the codec, QoS and metadata parameters — are validated separately, against
    /// the server's advertised capabilities, because only the operation that
    /// carries them can judge them.
    pub fn parse(data: &'a [u8]) -> Result<Self, AseResponse> {
        let (&raw_op, rest) = data.split_first().ok_or(AseResponse::InvalidLength)?;
        let op = AseOperation::from_u8(raw_op).ok_or(AseResponse::UnsupportedOpcode)?;

        let (&count, body) = rest.split_first().ok_or(AseResponse::InvalidLength)?;
        if count == 0 || count as usize > MAX_ASES_PER_OPERATION {
            return Err(AseResponse::InvalidLength);
        }

        let req = Self { op, count, body };

        // Walk the framing now, so that every later access is total.
        if op == AseOperation::ConfigCodec {
            for block in req.config_codec_blocks() {
                block?;
            }
        } else if req.body.len() < req.count as usize {
            return Err(AseResponse::InvalidLength);
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
    /// Every operation except `Config Codec` lists its ASE IDs first. `Config
    /// Codec` interleaves each ASE ID with that ASE's own codec configuration,
    /// so its IDs are taken from [`Self::config_codec_blocks`].
    pub fn ase_ids(&self) -> AseIdList {
        let mut ids = AseIdList::new();

        if self.op == AseOperation::ConfigCodec {
            for block in self.config_codec_blocks().flatten() {
                ids.push(block.ase_id);
            }
        } else {
            for &id in &self.body[..self.count as usize] {
                ids.push(id);
            }
        }

        ids
    }

    /// The per-ASE entries of a `Config Codec` operation.
    ///
    /// Yields one item per addressed ASE, and is empty for any other operation.
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

    /// The operation parameters that follow the ASE ID list.
    ///
    /// Empty for `Config Codec`, whose parameters are per-ASE and reached through
    /// [`Self::config_codec_blocks`].
    pub fn params(&self) -> &'a [u8] {
        if self.op == AseOperation::ConfigCodec {
            return &[];
        }
        &self.body[self.count as usize..]
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

        // ASE_ID (1), Codec_ID (5), configuration length (1), configuration (L).
        let Some((&ase_id, rest)) = self.rest.split_first() else {
            return Some(Err(AseResponse::InvalidLength));
        };
        if rest.len() < CodecId::SIZE + 1 {
            return Some(Err(AseResponse::InvalidLength));
        }

        let mut codec_bytes = [0u8; CodecId::SIZE];
        codec_bytes.copy_from_slice(&rest[..CodecId::SIZE]);
        let codec_id = CodecId::decode(&codec_bytes);

        let len = rest[CodecId::SIZE] as usize;
        let after_len = &rest[CodecId::SIZE + 1..];
        if after_len.len() < len {
            return Some(Err(AseResponse::InvalidLength));
        }

        let (config, tail) = after_len.split_at(len);
        self.rest = tail;

        Some(Ok(CodecConfigBlock {
            ase_id,
            codec_id,
            config,
        }))
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
        resp.push(id, code, 0);
    }

    resp
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ase::AseDirection;
    use crate::types::CodecId;

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

    /// `Config Codec` for one ASE: LC3 with a 48 kHz sampling frequency.
    const CONFIG_CODEC_LC3_48K: &[u8] = &[0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x08];

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
    fn parses_two_ases_and_keeps_params_opaque() {
        // Config QoS lists the ASE IDs first, then one shared parameter block.
        let data = [0x02, 0x02, 0x03, 0x04, 0xAA, 0xBB];
        let req = ControlPointRequest::parse(&data).unwrap();
        assert_eq!(req.op(), AseOperation::ConfigQos);
        assert_eq!(req.ase_ids().as_slice(), &[0x03, 0x04]);
        assert_eq!(req.params(), &[0xAA, 0xBB]);
    }

    #[test]
    fn config_codec_interleaves_each_ase_with_its_configuration() {
        // Config Codec does not list the ASE IDs first: each ASE ID is followed
        // by that ASE's own Codec_ID, length and configuration.
        let data = [
            0x01, 0x02, // Config Codec, 2 ASEs
            0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x08, // ASE 0
            0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x08, // ASE 1
        ];
        let req = ControlPointRequest::parse(&data).unwrap();
        assert_eq!(req.op(), AseOperation::ConfigCodec);
        assert_eq!(req.ase_ids().as_slice(), &[0x00, 0x01]);
        // Config Codec has no shared parameters.
        assert!(req.params().is_empty());

        let mut blocks = req.config_codec_blocks();
        let first = blocks.next().unwrap().unwrap();
        assert_eq!(first.ase_id, 0x00);
        assert_eq!(first.codec_id, CodecId::LC3);
        assert_eq!(first.config, &[0x02, 0x01, 0x08]);

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
        // No room for Codec_ID and the length octet.
        assert_eq!(
            ControlPointRequest::parse(&[0x01, 0x01, 0x00]),
            Err(AseResponse::InvalidLength)
        );
        // Claims a 4-octet configuration but supplies two.
        assert_eq!(
            ControlPointRequest::parse(&[0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x04, 0xAA, 0xBB]),
            Err(AseResponse::InvalidLength)
        );
        // Two ASEs announced, only one block present.
        assert_eq!(
            ControlPointRequest::parse(&[0x01, 0x02, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00]),
            Err(AseResponse::InvalidLength)
        );
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

        let data = [
            0x01, 0x02, // Config Codec, 2 ASEs
            0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x08, // ASE 0
            0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x08, // ASE 1
        ];
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
        let req = ControlPointRequest::parse(&[0x02, 0x02, 0x00, 0x01]).unwrap();
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

        // Same shape, but coding format 0x07 rather than LC3's 0x06.
        let data = [0x01, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x08];
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);

        assert_eq!(resp.entries()[0].code, AseResponse::UnsupportedAudioCapability);
        assert_eq!(ases[0].state(), crate::AseState::Idle);
    }

    #[test]
    fn config_codec_rejects_a_malformed_configuration() {
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];

        // A three-octet configuration whose first LTV claims a length of five.
        let data = [0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x05, 0x01, 0x08];
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::InvalidCodecConfiguration);

        // An empty configuration is not usable for LC3 either.
        let data = [0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00];
        let req = ControlPointRequest::parse(&data).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::InvalidCodecConfiguration);
        assert_eq!(ases[0].state(), crate::AseState::Idle);
    }

    #[test]
    fn a_well_formed_configuration_passes() {
        let records = lc3_caps();
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        let req = ControlPointRequest::parse(CONFIG_CODEC_LC3_48K).unwrap();
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
        let data = [0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x03];
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
        let data = [0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x02];
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

        // A ten-octet configuration: 48 kHz, 10 ms, 40 octets.
        let inside = [
            0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x0A, // Config Codec, ASE 0, len 10
            0x02, 0x01, 0x08, // sampling frequency 48 kHz
            0x02, 0x02, 0x02, // frame duration 10 ms
            0x03, 0x04, 40, 0, // 40 octets per codec frame
        ];
        let req = ControlPointRequest::parse(&inside).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert!(resp.entries()[0].code.is_success());

        // The same, but 200 octets, above the advertised maximum.
        let outside = [
            0x01, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x02, 0x01, 0x08, 0x02, 0x02, 0x02, 0x03, 0x04, 200,
            0,
        ];
        let req = ControlPointRequest::parse(&outside).unwrap();
        let resp = apply(&mut ases, &with_sink(&records), &req);
        assert_eq!(resp.entries()[0].code, AseResponse::UnsupportedCodecConfiguration);
    }
}
