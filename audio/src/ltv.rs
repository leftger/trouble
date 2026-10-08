//! Length-Type-Value (LTV) encoding, used for codec capabilities, codec
//! configuration and metadata.
//!
//! An LTV block is a concatenation of entries, each encoded as
//!
//! ```text
//! +----------+----------+------------------+
//! | Length   | Type     | Value (Length)   |
//! | 1 octet  | 1 octet  | Length octets    |
//! +----------+----------+------------------+
//! ```
//!
//! `Length` counts the value octets plus the type octet, so a `Length` of `1`
//! is a value-less entry. A `Length` of `0` is not valid and is treated as
//! malformed.

use core::fmt;

/// Errors from parsing or building LTV blocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LtvError {
    /// An entry ran past the end of the buffer, or a length field was missing.
    Truncated,
    /// An entry declared a length of zero. `Length` counts the type octet, so
    /// the smallest valid value is 1.
    ZeroLength,
    /// The destination buffer was too small.
    InsufficientSpace,
}

impl fmt::Display for LtvError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LtvError::Truncated => f.write_str("truncated LTV block"),
            LtvError::ZeroLength => f.write_str("zero-length LTV entry"),
            LtvError::InsufficientSpace => f.write_str("insufficient space for LTV entry"),
        }
    }
}

/// A single LTV entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ltv<'a> {
    /// The type octet.
    pub ty: u8,
    /// The value octets. May be empty.
    pub value: &'a [u8],
}

impl Ltv<'_> {
    /// Interpret the value as a little-endian `u16`.
    ///
    /// Returns `None` if the value is not exactly two octets.
    pub const fn as_u16(&self) -> Option<u16> {
        if self.value.len() != 2 {
            return None;
        }
        Some(u16::from_le_bytes([self.value[0], self.value[1]]))
    }

    /// Interpret the value as a little-endian `u24`.
    pub const fn as_u24(&self) -> Option<u32> {
        if self.value.len() != 3 {
            return None;
        }
        Some(self.value[0] as u32 | (self.value[1] as u32) << 8 | (self.value[2] as u32) << 16)
    }

    /// Interpret the value as a little-endian `u32`.
    pub const fn as_u32(&self) -> Option<u32> {
        if self.value.len() != 4 {
            return None;
        }
        Some(u32::from_le_bytes([
            self.value[0],
            self.value[1],
            self.value[2],
            self.value[3],
        ]))
    }
}

/// Iterator over the LTV entries of a block.
///
/// Constructed via [`LtvIter::new`], which validates the block up front so that
/// iteration cannot fail part-way through.
#[derive(Debug, Clone)]
pub struct LtvIter<'a> {
    remaining: &'a [u8],
}

impl<'a> LtvIter<'a> {
    /// Validate `data` as an LTV block and prepare to iterate it.
    pub fn new(data: &'a [u8]) -> Result<Self, LtvError> {
        let mut rest = data;
        while !rest.is_empty() {
            let len = rest[0] as usize;
            if len == 0 {
                return Err(LtvError::ZeroLength);
            }
            // len counts the type octet, which follows the length octet.
            if rest.len() < len + 1 {
                return Err(LtvError::Truncated);
            }
            rest = &rest[len + 1..];
        }
        Ok(Self { remaining: data })
    }

    /// The block being iterated.
    pub const fn as_slice(&self) -> &'a [u8] {
        self.remaining
    }
}

impl<'a> Iterator for LtvIter<'a> {
    type Item = Ltv<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining.is_empty() {
            return None;
        }
        let len = self.remaining[0] as usize;
        debug_assert!(len >= 1 && self.remaining.len() > len);
        let ty = self.remaining[1];
        let value = &self.remaining[2..len + 1];
        self.remaining = &self.remaining[len + 1..];
        Some(Ltv { ty, value })
    }
}

/// Find the first entry of type `ty` in an LTV block.
///
/// Returns `None` if the block is malformed or the type is absent.
pub fn find(data: &[u8], ty: u8) -> Option<Ltv<'_>> {
    LtvIter::new(data).ok()?.find(|e| e.ty == ty)
}

/// Builder for an LTV block backed by a caller-provided buffer.
#[derive(Debug)]
pub struct LtvWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> LtvWriter<'a> {
    /// Create a writer over `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Append an entry.
    pub fn push(&mut self, ty: u8, value: &[u8]) -> Result<(), LtvError> {
        let len = value.len() + 1;
        if len > u8::MAX as usize {
            return Err(LtvError::InsufficientSpace);
        }
        if self.pos + 1 + len > self.buf.len() {
            return Err(LtvError::InsufficientSpace);
        }
        self.buf[self.pos] = len as u8;
        self.buf[self.pos + 1] = ty;
        self.buf[self.pos + 2..self.pos + 1 + len].copy_from_slice(value);
        self.pos += 1 + len;
        Ok(())
    }

    /// Append an entry with a single-octet value.
    pub fn push_u8(&mut self, ty: u8, value: u8) -> Result<(), LtvError> {
        self.push(ty, &[value])
    }

    /// Append an entry with a little-endian two-octet value.
    pub fn push_u16(&mut self, ty: u8, value: u16) -> Result<(), LtvError> {
        self.push(ty, &value.to_le_bytes())
    }

    /// Append an entry with a little-endian three-octet value.
    pub fn push_u24(&mut self, ty: u8, value: u32) -> Result<(), LtvError> {
        let b = value.to_le_bytes();
        self.push(ty, &b[..3])
    }

    /// Append an entry with a little-endian four-octet value.
    pub fn push_u32(&mut self, ty: u8, value: u32) -> Result<(), LtvError> {
        self.push(ty, &value.to_le_bytes())
    }

    /// Number of octets written so far.
    pub const fn len(&self) -> usize {
        self.pos
    }

    /// Whether nothing has been written.
    pub const fn is_empty(&self) -> bool {
        self.pos == 0
    }

    /// The block written so far.
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

/// LTV type values for codec-specific capabilities.
pub mod cap {
    /// `Supported_Sampling_Frequencies`, `u16` bitfield.
    pub const SUPPORTED_SAMPLING_FREQUENCIES: u8 = 0x01;
    /// `Supported_Frame_Durations`, `u8` bitfield.
    pub const SUPPORTED_FRAME_DURATIONS: u8 = 0x02;
    /// `Supported_Audio_Channel_Counts`, `u32` bitfield.
    pub const SUPPORTED_AUDIO_CHANNEL_COUNTS: u8 = 0x03;
    /// `Supported_Octets_Per_Codec_Frame`, `u16` minimum followed by `u16` maximum.
    pub const SUPPORTED_OCTETS_PER_CODEC_FRAME: u8 = 0x04;
    /// `Supported_Max_Codec_Frames_Per_SDU`, `u8`.
    pub const SUPPORTED_MAX_CODEC_FRAMES_PER_SDU: u8 = 0x05;
}

/// LTV type values for codec-specific configuration.
pub mod cfg {
    /// `Sampling_Frequency`, `u8` (the BAP sampling-frequency bit index).
    pub const SAMPLING_FREQUENCY: u8 = 0x01;
    /// `Frame_Duration`, `u8` bitfield.
    pub const FRAME_DURATION: u8 = 0x02;
    /// `Audio_Channel_Allocation`, `u32` bitfield.
    pub const AUDIO_CHANNEL_ALLOCATION: u8 = 0x03;
    /// `Octets_Per_Codec_Frame`, `u16`.
    pub const OCTETS_PER_CODEC_FRAME: u8 = 0x04;
    /// `Codec_Frames_Per_SDU`, `u8`.
    pub const CODEC_FRAMES_PER_SDU: u8 = 0x05;
}

/// LTV type values for metadata.
pub mod meta {
    /// `Preferred_Audio_Contexts`, `u16` bitfield.
    pub const PREFERRED_AUDIO_CONTEXTS: u8 = 0x01;
    /// `Streaming_Audio_Contexts`, `u16` bitfield.
    pub const STREAMING_AUDIO_CONTEXTS: u8 = 0x02;
    /// `Program_Info`, UTF-8.
    pub const PROGRAM_INFO: u8 = 0x03;
    /// `Language`, 2 or 3 octets.
    pub const LANGUAGE: u8 = 0x04;
    /// `CCID_List`, list of `u8`.
    pub const CCID_LIST: u8 = 0x05;
    /// `Parental_Rating`, `u8`.
    pub const PARENTAL_RATING: u8 = 0x06;
    /// `Program_Reference`, list of `u8`.
    pub const PROGRAM_REFERENCE: u8 = 0x07;
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    use crate::types::SupportedSamplingFreqs;
    use crate::{AudioLocation, SampleRate};
    use std::vec::Vec;

    #[test]
    fn empty_block_is_valid() {
        let mut it = LtvIter::new(&[]).unwrap();
        assert_eq!(it.next(), None);
    }

    #[test]
    fn parses_multiple_entries() {
        // Supported sampling frequencies (u16), then supported frame durations (u8).
        let mut buf = [0u8; 16];
        let mut w = LtvWriter::new(&mut buf);
        w.push_u16(cap::SUPPORTED_SAMPLING_FREQUENCIES, 0x0088).unwrap();
        w.push_u8(cap::SUPPORTED_FRAME_DURATIONS, 0x03).unwrap();
        w.push_u32(cap::SUPPORTED_AUDIO_CHANNEL_COUNTS, 0x03).unwrap();
        let block = w.as_slice().to_vec();

        let entries: Vec<_> = LtvIter::new(&block).unwrap().collect();
        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0].ty, cap::SUPPORTED_SAMPLING_FREQUENCIES);
        assert_eq!(entries[0].as_u16(), Some(0x0088));
        assert_eq!(entries[1].as_u16(), None);
        assert_eq!(entries[2].as_u32(), Some(0x03));
    }

    #[test]
    fn values_are_little_endian() {
        let mut buf = [0u8; 8];
        let mut w = LtvWriter::new(&mut buf);
        w.push_u24(cfg::AUDIO_CHANNEL_ALLOCATION, 0x00_00_00_03).unwrap();
        assert_eq!(w.as_slice(), &[4, cfg::AUDIO_CHANNEL_ALLOCATION, 0x03, 0x00, 0x00]);
    }

    #[test]
    fn find_returns_first_match() {
        let mut buf = [0u8; 16];
        let mut w = LtvWriter::new(&mut buf);
        w.push_u8(cfg::FRAME_DURATION, 0x02).unwrap();
        w.push_u8(cfg::SAMPLING_FREQUENCY, 0x07).unwrap();
        let block = w.as_slice();
        assert_eq!(find(block, cfg::SAMPLING_FREQUENCY).unwrap().value, &[0x07]);
        assert!(find(block, meta::PROGRAM_INFO).is_none());
    }

    #[test]
    fn rejects_zero_length() {
        assert_eq!(LtvIter::new(&[0x00, 0x01]).err(), Some(LtvError::ZeroLength));
    }

    #[test]
    fn rejects_truncated() {
        // Declares two value octets but supplies none.
        assert_eq!(LtvIter::new(&[0x03, 0x01]).err(), Some(LtvError::Truncated));
    }

    #[test]
    fn writer_reports_insufficient_space() {
        let mut buf = [0u8; 3];
        let mut w = LtvWriter::new(&mut buf);
        // An entry with a 4-octet value needs 6 octets.
        assert_eq!(
            w.push_u32(cap::SUPPORTED_AUDIO_CHANNEL_COUNTS, 1),
            Err(LtvError::InsufficientSpace)
        );
        assert!(w.is_empty());
    }

    #[test]
    fn capability_block_for_lc3_48k_stereo() {
        // The shape a PAC record capability block has for a 48 kHz stereo sink.
        let mut buf = [0u8; 32];
        let mut w = LtvWriter::new(&mut buf);
        w.push_u16(
            cap::SUPPORTED_SAMPLING_FREQUENCIES,
            SupportedSamplingFreqs::from(SampleRate::Hz48000).bits(),
        )
        .unwrap();
        w.push_u8(cap::SUPPORTED_FRAME_DURATIONS, 0x03).unwrap();
        w.push_u32(cap::SUPPORTED_AUDIO_CHANNEL_COUNTS, 0x03).unwrap();
        w.push(cap::SUPPORTED_OCTETS_PER_CODEC_FRAME, &[40, 0, 120, 0]).unwrap();
        let block = w.as_slice();

        let freqs = SupportedSamplingFreqs(
            find(block, cap::SUPPORTED_SAMPLING_FREQUENCIES)
                .unwrap()
                .as_u16()
                .unwrap(),
        );
        assert!(freqs.contains(SupportedSamplingFreqs::from(SampleRate::Hz48000)));
        assert!(!freqs.contains(SupportedSamplingFreqs::HZ8000));

        let loc = AudioLocation::STEREO;
        assert_eq!(loc.count(), 2);
    }
}
