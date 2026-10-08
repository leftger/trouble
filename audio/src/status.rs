//! The value of an ASE characteristic.
//!
//! Each Audio Stream Endpoint is a characteristic, and its value is the pair
//! `ASE_ID | ASE_State` followed by the parameters that state carries. The
//! parameters are not the same shape from state to state: a configured ASE
//! reports its codec, a QoS-configured one its transport parameters, and an
//! enabling or streaming one the metadata being applied.
//!
//! ```text
//! Idle               | 0x06, 0x00                            (2 octets)
//! Codec Configured   | 0x06, 0x01, Framing, PHY, RTN, Max_Transport_Latency,
//!                      PD_Min, PD_Max, Pref_PD_Min, Pref_PD_Max, Codec_ID,
//!                      Config_Length, Config
//! QoS Configured     | 0x06, 0x02, CIG_ID, CIS_ID, SDU_Interval, Framing, PHY,
//!                      Max_SDU, RTN, Max_Transport_Latency, Presentation_Delay
//! Enabling           | 0x06, 0x03, CIG_ID, CIS_ID, Metadata_Length, Metadata
//! Streaming          | 0x06, 0x04, CIG_ID, CIS_ID, Metadata_Length, Metadata
//! Disabling          | 0x06, 0x05, CIG_ID, CIS_ID, Metadata_Length, Metadata
//! Releasing          | 0x06, 0x06                            (2 octets)
//! ```
//!
//! The layouts follow Zephyr's `bt_ascs_ase_status` and the per-state structs
//! beside it, which build these values in `ascs_ep_get_status_*`.
//!
//! Only encoding is implemented. Decoding a peer's ASE value is not something a
//! server needs to do — the client reads these, it does not write them.

use crate::ase::{AseDirection, AseState};
use crate::types::{CodecId, Framing, Phy};

/// Why an ASE status value could not be produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AseStatusError {
    /// The destination buffer is too small.
    InsufficientSpace,
    /// A length field would not fit in its single octet.
    InvalidLength,
    /// The parameters do not belong to the given state.
    WrongParams,
}

/// The longest an ASE characteristic value can be.
///
/// The longest state is Codec Configured, whose codec-specific configuration
/// alone occupies an octet-prefixed field.
pub const MAX_VALUE_LEN: usize = 2 + CODEC_CONFIGURED_PARAMS_LEN + u8::MAX as usize;

/// Octets of the `Codec Configured` parameters, excluding the configuration.
const CODEC_CONFIGURED_PARAMS_LEN: usize = 1 + 1 + 1 + 2 + 3 + 3 + 3 + 3 + CodecId::SIZE + 1;

/// The parameters carried by a `Codec Configured` ASE.
///
/// The server describes what it would accept for the stream: the preferred
/// framing, PHY, retransmission number and transport latency, and the range of
/// presentation delays it supports. The reachable delay range and the preferred
/// range are separate, and either may be [`PD_NO_PREFERENCE`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CodecConfiguredParams<'a> {
    /// The framing the server prefers.
    pub framing: Framing,
    /// The PHY the server prefers, as a bitfield.
    pub phy: Phy,
    /// The retransmission number the server prefers.
    pub rtn: u8,
    /// The maximum transport latency the server accepts, in milliseconds.
    pub max_transport_latency_ms: u16,
    /// The lowest presentation delay the server accepts, in microseconds.
    pub presentation_delay_min_us: u32,
    /// The highest presentation delay the server accepts, in microseconds.
    pub presentation_delay_max_us: u32,
    /// The lowest presentation delay the server prefers, in microseconds.
    pub preferred_presentation_delay_min_us: u32,
    /// The highest presentation delay the server prefers, in microseconds.
    pub preferred_presentation_delay_max_us: u32,
    /// The codec that was configured.
    pub codec_id: CodecId,
    /// The codec-specific configuration, as an LTV block.
    pub config: &'a [u8],
}

/// A presentation delay of zero means the server has no preference.
pub const PD_NO_PREFERENCE: u32 = 0x0000_0000;

/// The parameters carried by a `QoS Configured` ASE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QosConfiguredParams {
    /// The CIG the server placed the CIS in.
    pub cig_id: u8,
    /// The CIS identifier.
    pub cis_id: u8,
    /// The SDU interval, in microseconds. 24-bit on the wire.
    pub sdu_interval_us: u32,
    /// The ISOAL framing mode.
    pub framing: Framing,
    /// The PHY, as a bitfield.
    pub phy: Phy,
    /// The maximum SDU size, in octets.
    pub max_sdu: u16,
    /// The retransmission number.
    pub rtn: u8,
    /// The maximum transport latency, in milliseconds.
    pub max_transport_latency_ms: u16,
    /// The presentation delay, in microseconds. 24-bit on the wire.
    pub presentation_delay_us: u32,
}

/// The parameters carried by an `Enabling`, `Streaming` or `Disabling` ASE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MetadataParams<'a> {
    /// The CIG the stream belongs to.
    pub cig_id: u8,
    /// The CIS the stream belongs to.
    pub cis_id: u8,
    /// The metadata in effect, as an LTV block. May be empty.
    pub metadata: &'a [u8],
}

/// The parameters an ASE characteristic carries, which depend on its state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AseParams<'a> {
    /// `Idle` and `Releasing` carry no parameters.
    None,
    /// `Codec Configured`.
    CodecConfigured(CodecConfiguredParams<'a>),
    /// `QoS Configured`.
    QosConfigured(QosConfiguredParams),
    /// `Enabling`, `Streaming` and `Disabling`.
    Metadata(MetadataParams<'a>),
}

/// Encode an ASE characteristic value into `out`.
///
/// Returns the number of octets written, which is what the characteristic's
/// length should be reported as. The state and the parameters have to agree:
/// passing `Idle` with configured parameters is a programming error rather than
/// something a peer can cause, so it is reported rather than encoded.
pub fn encode(out: &mut [u8], ase_id: u8, state: AseState, params: AseParams<'_>) -> Result<usize, AseStatusError> {
    let mut w = Writer { out, at: 0 };

    w.u8(ase_id)?;
    w.u8(state.to_u8())?;

    match (state, params) {
        (AseState::Idle | AseState::Releasing, AseParams::None) => {}

        (AseState::CodecConfigured, AseParams::CodecConfigured(p)) => {
            if p.config.len() > u8::MAX as usize {
                return Err(AseStatusError::InvalidLength);
            }
            w.u8(p.framing.to_u8())?;
            w.u8(p.phy.bits())?;
            w.u8(p.rtn)?;
            w.u16(p.max_transport_latency_ms)?;
            w.u24(p.presentation_delay_min_us)?;
            w.u24(p.presentation_delay_max_us)?;
            w.u24(p.preferred_presentation_delay_min_us)?;
            w.u24(p.preferred_presentation_delay_max_us)?;
            w.codec_id(&p.codec_id)?;
            w.u8(p.config.len() as u8)?;
            w.slice(p.config)?;
        }

        (AseState::QosConfigured, AseParams::QosConfigured(p)) => {
            w.u8(p.cig_id)?;
            w.u8(p.cis_id)?;
            w.u24(p.sdu_interval_us)?;
            w.u8(p.framing.to_u8())?;
            w.u8(p.phy.bits())?;
            w.u16(p.max_sdu)?;
            w.u8(p.rtn)?;
            w.u16(p.max_transport_latency_ms)?;
            w.u24(p.presentation_delay_us)?;
        }

        (AseState::Enabling | AseState::Streaming | AseState::Disabling, AseParams::Metadata(p)) => {
            if p.metadata.len() > u8::MAX as usize {
                return Err(AseStatusError::InvalidLength);
            }
            w.u8(p.cig_id)?;
            w.u8(p.cis_id)?;
            w.u8(p.metadata.len() as u8)?;
            w.slice(p.metadata)?;
        }

        _ => return Err(AseStatusError::WrongParams),
    }

    Ok(w.at)
}

/// The direction an ASE description refers to, used when naming a characteristic.
///
/// This only exists so that a service can pick the right characteristic UUID
/// without matching on the direction itself.
pub const fn characteristic_is_sink(direction: AseDirection) -> bool {
    matches!(direction, AseDirection::Sink)
}

struct Writer<'a> {
    out: &'a mut [u8],
    at: usize,
}

impl Writer<'_> {
    fn u8(&mut self, v: u8) -> Result<(), AseStatusError> {
        if self.out.len() < self.at + 1 {
            return Err(AseStatusError::InsufficientSpace);
        }
        self.out[self.at] = v;
        self.at += 1;
        Ok(())
    }

    fn u16(&mut self, v: u16) -> Result<(), AseStatusError> {
        self.slice(&v.to_le_bytes())
    }

    /// A 24-bit little-endian field, which is how the specification writes SDU
    /// intervals and presentation delays.
    fn u24(&mut self, v: u32) -> Result<(), AseStatusError> {
        if v > 0x00FF_FFFF {
            return Err(AseStatusError::InvalidLength);
        }
        let b = v.to_le_bytes();
        self.slice(&b[..3])
    }

    fn codec_id(&mut self, id: &CodecId) -> Result<(), AseStatusError> {
        let mut b = [0u8; CodecId::SIZE];
        id.encode(&mut b);
        self.slice(&b)
    }

    fn slice(&mut self, v: &[u8]) -> Result<(), AseStatusError> {
        if self.out.len() < self.at + v.len() {
            return Err(AseStatusError::InsufficientSpace);
        }
        self.out[self.at..self.at + v.len()].copy_from_slice(v);
        self.at += v.len();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{AudioLocation, FrameDuration, SampleRate};

    /// A codec-specific configuration for 48 kHz, 10 ms, mono, 40 octets.
    fn cc_48k() -> [u8; 19] {
        let mut buf = [0u8; 32];
        let block = crate::bap::CodecSpecificConfig::build(
            &mut buf,
            SampleRate::Hz48000,
            FrameDuration::Ms10,
            AudioLocation::MONO,
            40,
        )
        .unwrap();
        let mut out = [0u8; 19];
        out.copy_from_slice(block);
        out
    }

    #[test]
    fn idle_and_releasing_are_two_octets() {
        let mut out = [0u8; 8];

        let n = encode(&mut out, 0x06, AseState::Idle, AseParams::None).unwrap();
        assert_eq!(n, 2);
        assert_eq!(&out[..2], &[0x06, 0x00]);

        let n = encode(&mut out, 0x06, AseState::Releasing, AseParams::None).unwrap();
        assert_eq!(n, 2);
        assert_eq!(&out[..2], &[0x06, 0x06]);
    }

    #[test]
    fn codec_configured_layout() {
        let cc = cc_48k();
        let mut out = [0u8; MAX_VALUE_LEN];

        let params = CodecConfiguredParams {
            framing: Framing::Unframed,
            phy: Phy::LE_2M,
            rtn: 2,
            max_transport_latency_ms: 20,
            presentation_delay_min_us: 10_000,
            presentation_delay_max_us: 40_000,
            preferred_presentation_delay_min_us: PD_NO_PREFERENCE,
            preferred_presentation_delay_max_us: PD_NO_PREFERENCE,
            codec_id: CodecId::LC3,
            config: &cc,
        };
        let n = encode(
            &mut out,
            0x03,
            AseState::CodecConfigured,
            AseParams::CodecConfigured(params),
        )
        .unwrap();

        // ASE_ID, state, framing, PHY, RTN
        assert_eq!(&out[..5], &[0x03, 0x01, 0x00, 0x02, 0x02]);
        // Max_Transport_Latency 20 = 0x0014, little-endian.
        assert_eq!(&out[5..7], &[0x14, 0x00]);
        // PD min 10000 = 0x002710.
        assert_eq!(&out[7..10], &[0x10, 0x27, 0x00]);
        // PD max 40000 = 0x009c40.
        assert_eq!(&out[10..13], &[0x40, 0x9c, 0x00]);
        // Both preferred delays carry the "no preference" value.
        assert_eq!(&out[13..19], &[0, 0, 0, 0, 0, 0]);
        // Codec_ID: LC3.
        assert_eq!(&out[19..24], &[0x06, 0x00, 0x00, 0x00, 0x00]);
        // Configuration length, then the configuration itself.
        assert_eq!(out[24], cc.len() as u8);
        assert_eq!(&out[25..25 + cc.len()], &cc);
        assert_eq!(n, 25 + cc.len());
    }

    #[test]
    fn qos_configured_layout() {
        let mut out = [0u8; MAX_VALUE_LEN];
        let params = QosConfiguredParams {
            cig_id: 0x01,
            cis_id: 0x02,
            sdu_interval_us: 10_000,
            framing: Framing::Unframed,
            phy: Phy::LE_2M,
            max_sdu: 40,
            rtn: 2,
            max_transport_latency_ms: 20,
            presentation_delay_us: 40_000,
        };
        let n = encode(
            &mut out,
            0x03,
            AseState::QosConfigured,
            AseParams::QosConfigured(params),
        )
        .unwrap();

        assert_eq!(n, 17);
        assert_eq!(&out[..2], &[0x03, 0x02]);
        assert_eq!(&out[2..4], &[0x01, 0x02]); // CIG_ID, CIS_ID
        assert_eq!(&out[4..7], &[0x10, 0x27, 0x00]); // SDU interval
        assert_eq!(out[7], 0x00); // framing
        assert_eq!(out[8], 0x02); // PHY
        assert_eq!(&out[9..11], &[40, 0]); // max SDU
        assert_eq!(out[11], 2); // RTN
        assert_eq!(&out[12..14], &[0x14, 0x00]); // max transport latency
        assert_eq!(&out[14..17], &[0x40, 0x9c, 0x00]); // presentation delay
    }

    #[test]
    fn enabling_carries_the_metadata() {
        let meta = [0x03u8, 0x02, 0x04, 0x00];
        let mut out = [0u8; MAX_VALUE_LEN];
        let params = MetadataParams {
            cig_id: 0x01,
            cis_id: 0x02,
            metadata: &meta,
        };
        let n = encode(&mut out, 0x03, AseState::Enabling, AseParams::Metadata(params)).unwrap();

        assert_eq!(n, 5 + meta.len());
        assert_eq!(&out[..2], &[0x03, 0x03]);
        assert_eq!(&out[2..4], &[0x01, 0x02]);
        assert_eq!(out[4], meta.len() as u8);
        assert_eq!(&out[5..5 + meta.len()], &meta);

        // Streaming and Disabling use the same layout with a different state.
        let n = encode(&mut out, 0x03, AseState::Streaming, AseParams::Metadata(params)).unwrap();
        assert_eq!(&out[..2], &[0x03, 0x04]);
        assert_eq!(n, 5 + meta.len());

        let n = encode(&mut out, 0x03, AseState::Disabling, AseParams::Metadata(params)).unwrap();
        assert_eq!(&out[..2], &[0x03, 0x05]);
        assert_eq!(n, 5 + meta.len());
    }

    #[test]
    fn empty_metadata_is_still_a_full_header() {
        let mut out = [0u8; MAX_VALUE_LEN];
        let params = MetadataParams {
            cig_id: 1,
            cis_id: 2,
            metadata: &[],
        };
        let n = encode(&mut out, 0x00, AseState::Streaming, AseParams::Metadata(params)).unwrap();
        assert_eq!(n, 5);
        assert_eq!(&out[..5], &[0x00, 0x04, 0x01, 0x02, 0x00]);
    }

    #[test]
    fn mismatched_state_and_params_are_refused() {
        let mut out = [0u8; MAX_VALUE_LEN];
        let params = MetadataParams {
            cig_id: 1,
            cis_id: 2,
            metadata: &[],
        };

        // An Idle ASE carries nothing, so metadata is a programming error.
        assert_eq!(
            encode(&mut out, 0, AseState::Idle, AseParams::Metadata(params)),
            Err(AseStatusError::WrongParams)
        );
        // And a configured-ASE layout cannot stand in for a streaming one.
        assert_eq!(
            encode(&mut out, 0, AseState::Streaming, AseParams::None),
            Err(AseStatusError::WrongParams)
        );
    }

    #[test]
    fn a_short_buffer_is_reported_rather_than_truncated() {
        let cc = cc_48k();
        let params = CodecConfiguredParams {
            framing: Framing::Unframed,
            phy: Phy::LE_2M,
            rtn: 0,
            max_transport_latency_ms: 0,
            presentation_delay_min_us: 0,
            presentation_delay_max_us: 0,
            preferred_presentation_delay_min_us: 0,
            preferred_presentation_delay_max_us: 0,
            codec_id: CodecId::LC3,
            config: &cc,
        };

        // Two octets short of what the state needs.
        let needed = 25 + cc.len();
        let mut out = [0u8; 64];
        assert_eq!(
            encode(
                &mut out[..needed - 2],
                0,
                AseState::CodecConfigured,
                AseParams::CodecConfigured(params)
            ),
            Err(AseStatusError::InsufficientSpace)
        );
    }

    #[test]
    fn twenty_four_bit_fields_are_range_checked() {
        let mut out = [0u8; MAX_VALUE_LEN];
        let params = QosConfiguredParams {
            cig_id: 0,
            cis_id: 0,
            sdu_interval_us: 0x0100_0000,
            framing: Framing::Unframed,
            phy: Phy::LE_1M,
            max_sdu: 0,
            rtn: 0,
            max_transport_latency_ms: 0,
            presentation_delay_us: 0,
        };
        assert_eq!(
            encode(&mut out, 0, AseState::QosConfigured, AseParams::QosConfigured(params)),
            Err(AseStatusError::InvalidLength)
        );
    }

    #[test]
    fn the_sink_helper_matches_the_direction() {
        assert!(characteristic_is_sink(AseDirection::Sink));
        assert!(!characteristic_is_sink(AseDirection::Source));
    }
}
