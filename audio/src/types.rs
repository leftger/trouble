//! Wire types defined by the Bluetooth Basic Audio Profile (BAP).
//!
//! These are small bitfield and enumeration wrappers over the raw integer values
//! that appear on the air. Values are cross-checked against ST's `STM32CubeWBA`
//! BLE Audio headers (`audio_types.h`, `bap_types.h`), which implement the same
//! specification.

/// Audio sample rate.
///
/// This is the set LC3 supports. The BAP `Supported_Sampling_Frequencies`
/// bitfield covers additional rates up to 384 kHz; those are reachable through
/// [`SupportedSamplingFreqs`] but are not produced by [`SampleRate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleRate {
    /// 8 kHz.
    Hz8000,
    /// 16 kHz.
    Hz16000,
    /// 24 kHz.
    Hz24000,
    /// 32 kHz.
    Hz32000,
    /// 44.1 kHz.
    Hz44100,
    /// 48 kHz.
    Hz48000,
}

impl SampleRate {
    /// The BAP bit for this rate, as used in the `Supported_Sampling_Frequencies` LTV.
    pub const fn bit(self) -> u16 {
        match self {
            SampleRate::Hz8000 => 0x0001,
            SampleRate::Hz16000 => 0x0004,
            SampleRate::Hz24000 => 0x0010,
            SampleRate::Hz32000 => 0x0020,
            SampleRate::Hz44100 => 0x0040,
            SampleRate::Hz48000 => 0x0080,
        }
    }

    /// The rate in Hz.
    pub const fn hz(self) -> u32 {
        match self {
            SampleRate::Hz8000 => 8000,
            SampleRate::Hz16000 => 16000,
            SampleRate::Hz24000 => 24000,
            SampleRate::Hz32000 => 32000,
            SampleRate::Hz44100 => 44100,
            SampleRate::Hz48000 => 48000,
        }
    }

    /// Parse a single rate from a BAP capability bitfield, picking the lowest set bit.
    pub const fn from_bits(bits: u16) -> Option<Self> {
        if bits & 0x0001 != 0 {
            Some(SampleRate::Hz8000)
        } else if bits & 0x0004 != 0 {
            Some(SampleRate::Hz16000)
        } else if bits & 0x0010 != 0 {
            Some(SampleRate::Hz24000)
        } else if bits & 0x0020 != 0 {
            Some(SampleRate::Hz32000)
        } else if bits & 0x0040 != 0 {
            Some(SampleRate::Hz44100)
        } else if bits & 0x0080 != 0 {
            Some(SampleRate::Hz48000)
        } else {
            None
        }
    }
}

/// The BAP `Supported_Sampling_Frequencies` bitfield.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SupportedSamplingFreqs(pub u16);

#[allow(missing_docs)]
impl SupportedSamplingFreqs {
    pub const HZ8000: Self = Self(0x0001);
    pub const HZ11025: Self = Self(0x0002);
    pub const HZ16000: Self = Self(0x0004);
    pub const HZ22050: Self = Self(0x0008);
    pub const HZ24000: Self = Self(0x0010);
    pub const HZ32000: Self = Self(0x0020);
    pub const HZ44100: Self = Self(0x0040);
    pub const HZ48000: Self = Self(0x0080);

    /// Whether every bit set in `other` is set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// The raw bitfield.
    pub const fn bits(self) -> u16 {
        self.0
    }
}

impl From<SampleRate> for SupportedSamplingFreqs {
    fn from(r: SampleRate) -> Self {
        Self(r.bit())
    }
}

/// Audio frame duration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FrameDuration {
    /// 7.5 ms.
    Ms7_5,
    /// 10 ms.
    Ms10,
}

impl FrameDuration {
    /// The BAP bit for this duration.
    pub const fn bit(self) -> u8 {
        match self {
            FrameDuration::Ms7_5 => 0x01,
            FrameDuration::Ms10 => 0x02,
        }
    }

    /// Duration in microseconds.
    pub const fn micros(self) -> u32 {
        match self {
            FrameDuration::Ms7_5 => 7_500,
            FrameDuration::Ms10 => 10_000,
        }
    }

    /// Number of PCM samples per frame per channel at `rate`.
    pub const fn samples(self, rate: SampleRate) -> usize {
        match (self, rate) {
            (FrameDuration::Ms7_5, SampleRate::Hz8000) => 60,
            (FrameDuration::Ms7_5, SampleRate::Hz16000) => 120,
            (FrameDuration::Ms7_5, SampleRate::Hz24000) => 180,
            (FrameDuration::Ms7_5, SampleRate::Hz32000) => 240,
            (FrameDuration::Ms7_5, SampleRate::Hz44100) => 330,
            (FrameDuration::Ms7_5, SampleRate::Hz48000) => 360,
            (FrameDuration::Ms10, SampleRate::Hz8000) => 80,
            (FrameDuration::Ms10, SampleRate::Hz16000) => 160,
            (FrameDuration::Ms10, SampleRate::Hz24000) => 240,
            (FrameDuration::Ms10, SampleRate::Hz32000) => 320,
            (FrameDuration::Ms10, SampleRate::Hz44100) => 441,
            (FrameDuration::Ms10, SampleRate::Hz48000) => 480,
        }
    }
}

/// The BAP `Supported_Frame_Durations` bitfield.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SupportedFrameDurations(pub u8);

#[allow(missing_docs)]
impl SupportedFrameDurations {
    pub const MS_7_5: Self = Self(0x01);
    pub const MS_10: Self = Self(0x02);
    pub const PREFERRED_7_5: Self = Self(0x10);
    pub const PREFERRED_10: Self = Self(0x20);

    /// Whether every bit set in `other` is set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// The raw bitfield.
    pub const fn bits(self) -> u8 {
        self.0
    }
}

impl From<FrameDuration> for SupportedFrameDurations {
    fn from(d: FrameDuration) -> Self {
        Self(d.bit())
    }
}

/// The BAP `Supported_Audio_Channel_Counts` bitfield: bit `n` means `n + 1`
/// channels are supported.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SupportedChannelCounts(pub u32);

impl SupportedChannelCounts {
    /// Up to `n` channels.
    pub const fn up_to(n: u8) -> Self {
        if n >= 32 {
            Self(u32::MAX)
        } else {
            Self((1u32 << n) - 1)
        }
    }

    /// Whether `count` channels are supported.
    pub const fn supports(self, count: u8) -> bool {
        count >= 1 && count <= 32 && (self.0 & (1 << (count - 1))) != 0
    }

    /// The raw bitfield.
    pub const fn bits(self) -> u32 {
        self.0
    }
}

/// Audio channel allocation / audio location bitfield.
///
/// The same 32-bit layout is used for `Audio_Channel_Allocation` in a codec
/// configuration and for the `Sink/Source_Audio_Locations` characteristics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AudioLocation(pub u32);

#[allow(missing_docs)]
impl AudioLocation {
    pub const FRONT_LEFT: Self = Self(1 << 0);
    pub const FRONT_RIGHT: Self = Self(1 << 1);
    pub const FRONT_CENTER: Self = Self(1 << 2);
    pub const LOW_FREQUENCY_EFFECTS_1: Self = Self(1 << 3);
    pub const BACK_LEFT: Self = Self(1 << 4);
    pub const BACK_RIGHT: Self = Self(1 << 5);
    pub const FRONT_LEFT_OF_CENTER: Self = Self(1 << 6);
    pub const FRONT_RIGHT_OF_CENTER: Self = Self(1 << 7);
    pub const BACK_CENTER: Self = Self(1 << 8);
    pub const LOW_FREQUENCY_EFFECTS_2: Self = Self(1 << 9);
    pub const SIDE_LEFT: Self = Self(1 << 10);
    pub const SIDE_RIGHT: Self = Self(1 << 11);
    pub const TOP_FRONT_LEFT: Self = Self(1 << 12);
    pub const TOP_FRONT_RIGHT: Self = Self(1 << 13);
    pub const TOP_FRONT_CENTER: Self = Self(1 << 14);
    pub const TOP_CENTER: Self = Self(1 << 15);
    pub const TOP_BACK_LEFT: Self = Self(1 << 16);
    pub const TOP_BACK_RIGHT: Self = Self(1 << 17);
    pub const TOP_SIDE_LEFT: Self = Self(1 << 18);
    pub const TOP_SIDE_RIGHT: Self = Self(1 << 19);
    pub const BOTTOM_FRONT_CENTER: Self = Self(1 << 20);
    pub const BOTTOM_FRONT_LEFT: Self = Self(1 << 21);
    pub const BOTTOM_FRONT_RIGHT: Self = Self(1 << 22);
    pub const BOTTOM_BACK_CENTER: Self = Self(1 << 23);
    pub const BOTTOM_BACK_LEFT: Self = Self(1 << 24);
    pub const BOTTOM_BACK_RIGHT: Self = Self(1 << 25);
    pub const FRONT_LEFT_WIDE: Self = Self(1 << 26);
    pub const FRONT_RIGHT_WIDE: Self = Self(1 << 27);
    pub const LEFT_SURROUND: Self = Self(1 << 28);
    pub const RIGHT_SURROUND: Self = Self(1 << 29);
    pub const TOP_SURROUND_LEFT: Self = Self(1 << 30);
    pub const TOP_SURROUND_RIGHT: Self = Self(1u32 << 31);

    /// A single mono channel.
    pub const MONO: Self = Self::FRONT_CENTER;
    /// A conventional stereo pair.
    pub const STEREO: Self = Self(Self::FRONT_LEFT.0 | Self::FRONT_RIGHT.0);

    /// Combine two locations.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Whether every bit set in `other` is set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Number of channels set in the allocation.
    pub const fn count(self) -> u32 {
        self.0.count_ones()
    }

    /// The raw bitfield.
    pub const fn bits(self) -> u32 {
        self.0
    }
}

/// Audio context bitfield (BAP `Audio_Context`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AudioContext(pub u16);

#[allow(missing_docs)]
impl AudioContext {
    pub const UNSPECIFIED: Self = Self(0x0001);
    pub const CONVERSATIONAL: Self = Self(0x0002);
    pub const MEDIA: Self = Self(0x0004);
    pub const GAME: Self = Self(0x0008);
    pub const INSTRUCTIONAL: Self = Self(0x0010);
    pub const VOICE_ASSISTANTS: Self = Self(0x0020);
    pub const LIVE: Self = Self(0x0040);
    pub const SOUND_EFFECTS: Self = Self(0x0080);
    pub const NOTIFICATIONS: Self = Self(0x0100);
    pub const RINGTONE: Self = Self(0x0200);
    pub const ALERTS: Self = Self(0x0400);
    pub const EMERGENCY_ALARM: Self = Self(0x0800);

    /// Combine two context sets.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Whether every bit set in `other` is set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// The raw bitfield.
    pub const fn bits(self) -> u16 {
        self.0
    }
}

/// A 5-octet `Codec_ID`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CodecId {
    /// Coding format, e.g. `0x06` for LC3.
    pub coding_format: u8,
    /// Company identifier, `0x0000` for the Bluetooth SIG.
    pub company_id: u16,
    /// Vendor-specific codec identifier.
    pub vendor_specific_codec_id: u16,
}

impl CodecId {
    /// The LC3 codec, as defined by BAP.
    pub const LC3: Self = Self {
        coding_format: 0x06,
        company_id: 0x0000,
        vendor_specific_codec_id: 0x0000,
    };

    /// Encoded size in octets.
    pub const SIZE: usize = 5;

    /// Serialise into `out`.
    pub fn encode(&self, out: &mut [u8; Self::SIZE]) {
        out[0] = self.coding_format;
        out[1..3].copy_from_slice(&self.company_id.to_le_bytes());
        out[3..5].copy_from_slice(&self.vendor_specific_codec_id.to_le_bytes());
    }

    /// Parse from a 5-octet buffer.
    pub const fn decode(data: &[u8; Self::SIZE]) -> Self {
        Self {
            coding_format: data[0],
            company_id: u16::from_le_bytes([data[1], data[2]]),
            vendor_specific_codec_id: u16::from_le_bytes([data[3], data[4]]),
        }
    }
}

/// ISOAL framing mode, as negotiated in QoS configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Framing {
    /// Unframed: one SDU per ISO packet, no time-stamp field.
    Unframed,
    /// Framed: SDUs may be fragmented across ISO packets.
    Framed,
}

impl Framing {
    /// The wire value.
    pub const fn to_u8(self) -> u8 {
        match self {
            Framing::Unframed => 0x00,
            Framing::Framed => 0x01,
        }
    }

    /// Parse from the wire value.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Framing::Unframed),
            0x01 => Some(Framing::Framed),
            _ => None,
        }
    }
}

/// CIS/BIS packing mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Packing {
    /// Sequential.
    Unpacked,
    /// Interleaved.
    Packed,
}

impl Packing {
    /// The wire value.
    pub const fn to_u8(self) -> u8 {
        match self {
            Packing::Unpacked => 0x00,
            Packing::Packed => 0x01,
        }
    }

    /// Parse from the wire value.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Packing::Unpacked),
            0x01 => Some(Packing::Packed),
            _ => None,
        }
    }
}

/// PHY bitfield used in QoS configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Phy(pub u8);

#[allow(missing_docs)]
impl Phy {
    pub const LE_1M: Self = Self(0x01);
    pub const LE_2M: Self = Self(0x02);
    pub const LE_CODED: Self = Self(0x04);

    /// The raw bitfield.
    pub const fn bits(self) -> u8 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_rate_bits_roundtrip() {
        for r in [
            SampleRate::Hz8000,
            SampleRate::Hz16000,
            SampleRate::Hz24000,
            SampleRate::Hz32000,
            SampleRate::Hz44100,
            SampleRate::Hz48000,
        ] {
            assert_eq!(SampleRate::from_bits(r.bit()), Some(r));
            assert!(SupportedSamplingFreqs::from(r).contains(SupportedSamplingFreqs::from(r)));
        }
    }

    #[test]
    fn frame_samples_match_lc3_table() {
        // These must agree with the LC3 frame sizes (see the LC3 codec module).
        assert_eq!(FrameDuration::Ms10.samples(SampleRate::Hz48000), 480);
        assert_eq!(FrameDuration::Ms7_5.samples(SampleRate::Hz48000), 360);
        assert_eq!(FrameDuration::Ms10.samples(SampleRate::Hz8000), 80);
    }

    #[test]
    fn channel_counts() {
        let c = SupportedChannelCounts::up_to(2);
        assert!(c.supports(1));
        assert!(c.supports(2));
        assert!(!c.supports(3));
        assert!(!c.supports(0));
    }

    #[test]
    fn audio_location_count() {
        assert_eq!(AudioLocation::MONO.count(), 1);
        assert_eq!(AudioLocation::STEREO.count(), 2);
        assert!(AudioLocation::STEREO.contains(AudioLocation::FRONT_LEFT));
        assert!(!AudioLocation::MONO.contains(AudioLocation::FRONT_LEFT));
    }

    #[test]
    fn codec_id_roundtrip() {
        let mut buf = [0u8; CodecId::SIZE];
        CodecId::LC3.encode(&mut buf);
        assert_eq!(buf, [0x06, 0x00, 0x00, 0x00, 0x00]);
        assert_eq!(CodecId::decode(&buf), CodecId::LC3);
    }

    #[test]
    fn framing_roundtrip() {
        for f in [Framing::Unframed, Framing::Framed] {
            assert_eq!(Framing::from_u8(f.to_u8()), Some(f));
        }
        assert_eq!(Framing::from_u8(0x02), None);
    }
}
