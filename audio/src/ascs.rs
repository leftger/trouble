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
//! # Not yet modelled
//!
//! The operation parameters are exposed by [`ControlPointRequest::params`] but
//! not yet interpreted: this layer validates the framing and the ASE states, not
//! the codec or QoS contents. Two things are needed before that can be added,
//! and they should be settled against the specification first:
//!
//! * Whether `Config Codec` interleaves each `ASE_ID` with that ASE's codec
//!   configuration, or lists every `ASE_ID` first and then one shared codec
//!   configuration. (`Config QoS` lists the ASE IDs first and then shares one
//!   parameter block, because the addressed ASEs form a single CIS.)
//! * How much of a codec configuration must be checked against the advertised
//!   PAC records before responding [`AseResponse::UnsupportedCodecConfiguration`]
//!   rather than [`AseResponse::InvalidCodecConfiguration`].
//!
//! For that reason [`apply`] always writes a zero `Reason`.

use crate::ase::{Ase, AseOperation, AseResponse};

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
    ase_ids: &'a [u8],
    params: &'a [u8],
}

impl<'a> ControlPointRequest<'a> {
    /// Parse a request written to the ASE Control Point.
    ///
    /// This validates the opcode, the ASE count and that the ASE IDs are
    /// present. The operation parameters are returned unvalidated, because
    /// their layout depends on the opcode and is not yet modelled.
    pub fn parse(data: &'a [u8]) -> Result<Self, AseResponse> {
        let (&raw_op, rest) = data.split_first().ok_or(AseResponse::InvalidLength)?;
        let op = AseOperation::from_u8(raw_op).ok_or(AseResponse::UnsupportedOpcode)?;

        let (&count, rest) = rest.split_first().ok_or(AseResponse::InvalidLength)?;
        if count == 0 || count as usize > MAX_ASES_PER_OPERATION {
            return Err(AseResponse::InvalidLength);
        }

        let count = count as usize;
        if rest.len() < count {
            return Err(AseResponse::InvalidLength);
        }
        let (ase_ids, params) = rest.split_at(count);

        Ok(Self { op, ase_ids, params })
    }

    /// The requested operation.
    pub const fn op(&self) -> AseOperation {
        self.op
    }

    /// The ASE IDs the operation addresses, in the order they were given.
    pub const fn ase_ids(&self) -> &'a [u8] {
        self.ase_ids
    }

    /// The operation parameters, with the header and ASE IDs removed.
    pub const fn params(&self) -> &'a [u8] {
        self.params
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

/// The code that prevents the whole operation from being applied, if any.
///
/// An unknown ASE ID and an ASE that cannot accept the operation both reject the
/// entire operation, and are reported before anything is changed. The ASE IDs are
/// resolved first, so an unknown ID is reported ahead of any state error.
fn rejection(ases: &[Ase], req: &ControlPointRequest<'_>) -> Option<AseResponse> {
    if req.ase_ids().iter().any(|id| !ases.iter().any(|a| a.id() == *id)) {
        return Some(AseResponse::InvalidAseId);
    }

    for &id in req.ase_ids() {
        // Resolved by the check above.
        let Some(ase) = ases.iter().find(|a| a.id() == id) else {
            continue;
        };

        let code = ase.can_handle(req.op());
        if !code.is_success() {
            return Some(code);
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
/// A `Release` leaves the ASEs in [`AseState::Releasing`](crate::AseState); the
/// application is responsible for finishing it with
/// [`Ase::complete_release`] once any associated CIS has gone, and likewise for
/// [`Ase::complete_disable`] on a Source ASE.
pub fn apply(ases: &mut [Ase], req: &ControlPointRequest<'_>) -> ControlPointResponse {
    let mut resp = ControlPointResponse::new();
    let rejected = rejection(ases, req);

    for &id in req.ase_ids() {
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

    #[test]
    fn parses_a_single_ase_request() {
        // Release ASE 0.
        let req = ControlPointRequest::parse(&[0x08, 0x01, 0x00]).unwrap();
        assert_eq!(req.op(), AseOperation::Release);
        assert_eq!(req.ase_ids(), &[0x00]);
        assert!(req.params().is_empty());
    }

    #[test]
    fn parses_two_ases_and_keeps_params_opaque() {
        // Config QoS for ASE 3 and 4, followed by some parameter bytes.
        let data = [0x02, 0x02, 0x03, 0x04, 0xAA, 0xBB];
        let req = ControlPointRequest::parse(&data).unwrap();
        assert_eq!(req.op(), AseOperation::ConfigQos);
        assert_eq!(req.ase_ids(), &[0x03, 0x04]);
        assert_eq!(req.params(), &[0xAA, 0xBB]);
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
        let mut ases = [Ase::new(0, AseDirection::Sink), Ase::new(1, AseDirection::Sink)];

        // Config Codec on both.
        let req = ControlPointRequest::parse(&[0x01, 0x02, 0x00, 0x01]).unwrap();
        let resp = apply(&mut ases, &req);
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
        let resp = apply(&mut ases, &req);

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
        let resp = apply(&mut ases, &req);

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
        let resp = apply(&mut ases, &req);
        assert_eq!(resp.entries()[0].code, AseResponse::InvalidAseDirection);
        assert_eq!(ases[0].state(), crate::AseState::Enabling);
    }
}
