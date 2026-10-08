//! The BAP data model: PAC records, codec-specific configuration, and QoS.
//!
//! # Wire formats
//!
//! A Published Audio Capabilities (PAC) record is
//!
//! ```text
//! +-----------+----------------+---------------+--------------+----------+
//! | Codec_ID  | Caps_Length    | Caps          | Meta_Length  | Metadata |
//! | 5 octets  | 1 octet        | Caps_Length   | 1 octet      | Meta_Len |
//! +-----------+----------------+---------------+--------------+----------+
//! ```
//!
//! [`QosConfig`] is a *single direction* view of a QoS configuration: one SDU
//! interval, one maximum SDU, and the framing, PHY, retransmission number,
//! transport latency and presentation delay. It mirrors ST's `BAP_ASEQoSConf_t`,
//! which is likewise per-ASE, and it borrows ST's field order rather than any
//! wire layout.
//!
//! It is **not** the `Config QoS` operation payload. That operation interleaves a
//! 16-octet block per ASE — `ASE_ID | CIG_ID | CIS_ID | SDU_Interval(3) | Framing
//! | PHY | Max_SDU(2) | RTN | Max_Transport_Latency(2) | Presentation_Delay(3)` —
//! which Zephyr's `bt_ascs_qos` states directly. [crate::ascs::QosConfigBlock]
//! models it; this type does not.
//!
//! The ASE Control Point framing that carries this payload (opcode,
//! number-of-ASEs, ASE ID) is not modelled here; it belongs with the ASCS service
//! implementation.

use crate::ltv::{self, LtvError};
use crate::types::{AudioLocation, CodecId, FrameDuration, Framing, Phy, SampleRate};

/// Errors from encoding or decoding BAP structures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BapError {
    /// The buffer ended before the structure did.
    Truncated,
    /// A field carried a value that is not defined by the specification.
    InvalidValue,
    /// The destination buffer was too small.
    InsufficientSpace,
    /// An LTV block was malformed.
    Ltv(LtvError),
}

impl From<LtvError> for BapError {
    fn from(e: LtvError) -> Self {
        BapError::Ltv(e)
    }
}

impl core::fmt::Display for BapError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BapError::Truncated => f.write_str("truncated BAP structure"),
            BapError::InvalidValue => f.write_str("invalid value in BAP structure"),
            BapError::InsufficientSpace => f.write_str("insufficient space"),
            BapError::Ltv(e) => write!(f, "{}", e),
        }
    }
}

/// BAP encodes the *configured* sampling frequency as an ordinal
/// (`0x01` = 8 kHz), unlike the *capability* bitfield. This converts an ordinal.
pub(crate) const fn rate_from_index(index: u8) -> Option<SampleRate> {
    match index {
        0x01 => Some(SampleRate::Hz8000),
        0x03 => Some(SampleRate::Hz16000),
        0x05 => Some(SampleRate::Hz24000),
        0x06 => Some(SampleRate::Hz32000),
        0x07 => Some(SampleRate::Hz44100),
        0x08 => Some(SampleRate::Hz48000),
        _ => None,
    }
}

/// Inverse of [`rate_from_index`].
const fn rate_to_index(rate: SampleRate) -> u8 {
    match rate {
        SampleRate::Hz8000 => 0x01,
        SampleRate::Hz16000 => 0x03,
        SampleRate::Hz24000 => 0x05,
        SampleRate::Hz32000 => 0x06,
        SampleRate::Hz44100 => 0x07,
        SampleRate::Hz48000 => 0x08,
    }
}

/// A decoded codec-specific configuration (the LTV block written in Config Codec).
#[derive(Debug, Clone, Copy)]
pub struct CodecSpecificConfig<'a> {
    raw: &'a [u8],
}

impl<'a> CodecSpecificConfig<'a> {
    /// Wrap a raw LTV block, validating it.
    pub fn new(raw: &'a [u8]) -> Result<Self, BapError> {
        ltv::LtvIter::new(raw)?;
        Ok(Self { raw })
    }

    /// The raw LTV block.
    pub const fn as_slice(&self) -> &'a [u8] {
        self.raw
    }

    /// The configured sampling frequency.
    pub fn sampling_rate(&self) -> Option<SampleRate> {
        let e = ltv::find(self.raw, ltv::cfg::SAMPLING_FREQUENCY)?;
        rate_from_index(*e.value.first()?)
    }

    /// The configured frame duration.
    pub fn frame_duration(&self) -> Option<FrameDuration> {
        let e = ltv::find(self.raw, ltv::cfg::FRAME_DURATION)?;
        let bits = *e.value.first()?;
        if bits & FrameDuration::Ms10.bit() != 0 {
            Some(FrameDuration::Ms10)
        } else if bits & FrameDuration::Ms7_5.bit() != 0 {
            Some(FrameDuration::Ms7_5)
        } else {
            None
        }
    }

    /// The configured audio channel allocation.
    pub fn channel_allocation(&self) -> Option<AudioLocation> {
        let e = ltv::find(self.raw, ltv::cfg::AUDIO_CHANNEL_ALLOCATION)?;
        Some(AudioLocation(e.as_u32()?))
    }

    /// The configured number of octets per codec frame.
    pub fn octets_per_frame(&self) -> Option<u16> {
        ltv::find(self.raw, ltv::cfg::OCTETS_PER_CODEC_FRAME)?.as_u16()
    }

    /// The configured number of codec frames per SDU.
    pub fn codec_frames_per_sdu(&self) -> Option<u8> {
        Some(*ltv::find(self.raw, ltv::cfg::CODEC_FRAMES_PER_SDU)?.value.first()?)
    }

    /// Build the LTV block for a configuration, writing into `out`.
    ///
    /// Returns the block written.
    pub fn build(
        out: &mut [u8],
        rate: SampleRate,
        frame_duration: FrameDuration,
        allocation: AudioLocation,
        octets_per_frame: u16,
    ) -> Result<&[u8], BapError> {
        let len = {
            let mut w = ltv::LtvWriter::new(out);
            w.push_u8(ltv::cfg::SAMPLING_FREQUENCY, rate_to_index(rate))?;
            w.push_u8(ltv::cfg::FRAME_DURATION, frame_duration.bit())?;
            w.push_u32(ltv::cfg::AUDIO_CHANNEL_ALLOCATION, allocation.bits())?;
            w.push_u16(ltv::cfg::OCTETS_PER_CODEC_FRAME, octets_per_frame)?;
            w.push_u8(ltv::cfg::CODEC_FRAMES_PER_SDU, 1)?;
            w.len()
        };
        Ok(&out[..len])
    }
}

/// QoS configuration for one ASE (the Config QoS parameter block).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct QosConfig {
    /// SDU interval in microseconds. 24-bit on the wire.
    pub sdu_interval_us: u32,
    /// ISOAL framing mode.
    pub framing: Framing,
    /// PHY bitfield.
    pub phy: Phy,
    /// Maximum SDU size in octets.
    pub max_sdu: u16,
    /// Retransmission number.
    pub rtn: u8,
    /// Maximum transport latency in milliseconds.
    pub max_transport_latency_ms: u16,
    /// Presentation delay in microseconds. 24-bit on the wire.
    pub presentation_delay_us: u32,
}

impl QosConfig {
    /// Size of the Config QoS parameter block.
    pub const WIRE_SIZE: usize = 13;

    /// Serialise into `out`.
    pub fn encode(&self, out: &mut [u8; Self::WIRE_SIZE]) -> Result<(), BapError> {
        if self.sdu_interval_us > 0x00FF_FFFF || self.presentation_delay_us > 0x00FF_FFFF {
            return Err(BapError::InvalidValue);
        }
        let si = self.sdu_interval_us.to_le_bytes();
        out[0..3].copy_from_slice(&si[..3]);
        out[3] = self.framing.to_u8();
        out[4] = self.phy.bits();
        out[5..7].copy_from_slice(&self.max_sdu.to_le_bytes());
        out[7] = self.rtn;
        out[8..10].copy_from_slice(&self.max_transport_latency_ms.to_le_bytes());
        let pd = self.presentation_delay_us.to_le_bytes();
        out[10..13].copy_from_slice(&pd[..3]);
        Ok(())
    }

    /// Parse from the 13-octet Config QoS parameter block.
    pub fn decode(data: &[u8; Self::WIRE_SIZE]) -> Result<Self, BapError> {
        Ok(Self {
            sdu_interval_us: u32::from_le_bytes([data[0], data[1], data[2], 0]),
            framing: Framing::from_u8(data[3]).ok_or(BapError::InvalidValue)?,
            phy: Phy(data[4]),
            max_sdu: u16::from_le_bytes([data[5], data[6]]),
            rtn: data[7],
            max_transport_latency_ms: u16::from_le_bytes([data[8], data[9]]),
            presentation_delay_us: u32::from_le_bytes([data[10], data[11], data[12], 0]),
        })
    }
}

/// A Published Audio Capabilities (PAC) record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacRecord<'a> {
    /// The codec this record describes.
    pub codec_id: CodecId,
    /// Codec-specific capabilities, as an LTV block.
    pub codec_specific_capabilities: &'a [u8],
    /// Additional metadata, as an LTV block. May be empty.
    pub metadata: &'a [u8],
}

impl<'a> PacRecord<'a> {
    /// Total encoded size in octets.
    pub fn encoded_len(&self) -> usize {
        CodecId::SIZE + 1 + self.codec_specific_capabilities.len() + 1 + self.metadata.len()
    }

    /// Serialise into `out`, returning the number of octets written.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, BapError> {
        let caps_len = self.codec_specific_capabilities.len();
        let meta_len = self.metadata.len();
        if caps_len > u8::MAX as usize || meta_len > u8::MAX as usize {
            return Err(BapError::InvalidValue);
        }
        if out.len() < self.encoded_len() {
            return Err(BapError::InsufficientSpace);
        }
        let mut pos = 0;
        let mut id = [0u8; CodecId::SIZE];
        self.codec_id.encode(&mut id);
        out[pos..pos + CodecId::SIZE].copy_from_slice(&id);
        pos += CodecId::SIZE;
        out[pos] = caps_len as u8;
        pos += 1;
        out[pos..pos + caps_len].copy_from_slice(self.codec_specific_capabilities);
        pos += caps_len;
        out[pos] = meta_len as u8;
        pos += 1;
        out[pos..pos + meta_len].copy_from_slice(self.metadata);
        pos += meta_len;
        Ok(pos)
    }

    /// Parse a single PAC record from the start of `data`.
    ///
    /// Returns the record and the number of octets consumed.
    pub fn decode(data: &'a [u8]) -> Result<(Self, usize), BapError> {
        if data.len() < CodecId::SIZE + 1 {
            return Err(BapError::Truncated);
        }
        let mut id = [0u8; CodecId::SIZE];
        id.copy_from_slice(&data[..CodecId::SIZE]);
        let codec_id = CodecId::decode(&id);

        let mut pos = CodecId::SIZE;
        let caps_len = data[pos] as usize;
        pos += 1;
        if data.len() < pos + caps_len + 1 {
            return Err(BapError::Truncated);
        }
        let caps = &data[pos..pos + caps_len];
        pos += caps_len;

        let meta_len = data[pos] as usize;
        pos += 1;
        if data.len() < pos + meta_len {
            return Err(BapError::Truncated);
        }
        let metadata = &data[pos..pos + meta_len];
        pos += meta_len;

        Ok((
            Self {
                codec_id,
                codec_specific_capabilities: caps,
                metadata,
            },
            pos,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ltv::{cap, LtvWriter};

    #[test]
    fn configured_sampling_rate_is_an_ordinal_not_a_bitfield() {
        // 48 kHz is index 0x08 in a configuration, but bit 0x0080 in a capability.
        let mut buf = [0u8; 32];
        let block = CodecSpecificConfig::build(
            &mut buf,
            SampleRate::Hz48000,
            FrameDuration::Ms10,
            AudioLocation::STEREO,
            40,
        )
        .unwrap();
        let cfg = CodecSpecificConfig::new(block).unwrap();
        assert_eq!(cfg.sampling_rate(), Some(SampleRate::Hz48000));
        assert_eq!(cfg.frame_duration(), Some(FrameDuration::Ms10));
        assert_eq!(cfg.channel_allocation(), Some(AudioLocation::STEREO));
        assert_eq!(cfg.octets_per_frame(), Some(40));
        assert_eq!(cfg.codec_frames_per_sdu(), Some(1));
        // The raw LTV must carry the ordinal, not the bitfield.
        assert_eq!(
            crate::ltv::find(block, crate::ltv::cfg::SAMPLING_FREQUENCY)
                .unwrap()
                .value,
            &[0x08]
        );
    }

    #[test]
    fn qos_config_roundtrip() {
        let qos = QosConfig {
            sdu_interval_us: 10_000,
            framing: Framing::Unframed,
            phy: Phy::LE_2M,
            max_sdu: 40,
            rtn: 2,
            max_transport_latency_ms: 20,
            presentation_delay_us: 40_000,
        };
        let mut buf = [0u8; QosConfig::WIRE_SIZE];
        qos.encode(&mut buf).unwrap();
        assert_eq!(buf.len(), 13);
        // SDU interval 10000 = 0x002710 little-endian 3 octets.
        assert_eq!(&buf[0..3], &[0x10, 0x27, 0x00]);
        assert_eq!(buf[3], 0x00); // unframed
        assert_eq!(buf[4], 0x02); // 2M
        assert_eq!(&buf[5..7], &[40, 0]);
        assert_eq!(buf[7], 2);
        assert_eq!(QosConfig::decode(&buf).unwrap(), qos);
    }

    #[test]
    fn qos_rejects_out_of_range_24_bit_fields() {
        let qos = QosConfig {
            sdu_interval_us: 0x0100_0000,
            framing: Framing::Unframed,
            phy: Phy::LE_1M,
            max_sdu: 40,
            rtn: 0,
            max_transport_latency_ms: 10,
            presentation_delay_us: 0,
        };
        let mut buf = [0u8; QosConfig::WIRE_SIZE];
        assert_eq!(qos.encode(&mut buf), Err(BapError::InvalidValue));
    }

    #[test]
    fn qos_rejects_unknown_framing() {
        let mut buf = [0u8; QosConfig::WIRE_SIZE];
        buf[3] = 0x07;
        assert_eq!(QosConfig::decode(&buf), Err(BapError::InvalidValue));
    }

    #[test]
    fn pac_record_roundtrip() {
        // Capabilities for a 48 kHz stereo sink.
        let mut caps_buf = [0u8; 32];
        let mut w = LtvWriter::new(&mut caps_buf);
        w.push_u16(cap::SUPPORTED_SAMPLING_FREQUENCIES, 0x0080).unwrap();
        w.push_u8(cap::SUPPORTED_FRAME_DURATIONS, 0x03).unwrap();
        w.push_u32(cap::SUPPORTED_AUDIO_CHANNEL_COUNTS, 0x03).unwrap();
        w.push(cap::SUPPORTED_OCTETS_PER_CODEC_FRAME, &[40, 0, 120, 0]).unwrap();

        let rec = PacRecord {
            codec_id: CodecId::LC3,
            codec_specific_capabilities: w.as_slice(),
            metadata: &[],
        };

        let mut out = [0u8; 64];
        let n = rec.encode(&mut out).unwrap();
        assert_eq!(n, rec.encoded_len());

        let (decoded, consumed) = PacRecord::decode(&out[..n]).unwrap();
        assert_eq!(consumed, n);
        assert_eq!(decoded.codec_id, CodecId::LC3);
        assert_eq!(decoded.codec_specific_capabilities, rec.codec_specific_capabilities);
        assert_eq!(decoded.metadata, &[]);

        // The embedded capability block must still parse as LTV.
        let freqs = crate::ltv::find(decoded.codec_specific_capabilities, cap::SUPPORTED_SAMPLING_FREQUENCIES)
            .unwrap()
            .as_u16()
            .unwrap();
        assert_eq!(freqs, 0x0080);
    }

    #[test]
    fn pac_record_detects_truncation() {
        let mut out = [0u8; 16];
        // Claims a 200-octet capability block.
        out[0..5].copy_from_slice(&[0x06, 0, 0, 0, 0]);
        out[5] = 200;
        assert_eq!(PacRecord::decode(&out), Err(BapError::Truncated));
        assert_eq!(PacRecord::decode(&[0x06, 0, 0]), Err(BapError::Truncated));
    }

    #[test]
    fn pac_record_length_prefixes_are_respected() {
        let caps = [0x03u8, 0x01, 0x10, 0x27];
        let meta = [0x03u8, 0x02, 0x04, 0x00];
        let rec = PacRecord {
            codec_id: CodecId::LC3,
            codec_specific_capabilities: &caps,
            metadata: &meta,
        };
        let mut out = [0u8; 32];
        let n = rec.encode(&mut out).unwrap();
        assert_eq!(n, 5 + 1 + 4 + 1 + 4);
        let (decoded, _) = PacRecord::decode(&out[..n]).unwrap();
        assert_eq!(decoded.codec_specific_capabilities, &caps);
        assert_eq!(decoded.metadata, &meta);
    }
}
