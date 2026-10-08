//! Codec abstraction.
//!
//! LE Audio decouples the codec from the transport: the codec configuration is
//! negotiated over ASCS, and the ISO packets then carry codec frames. These
//! traits are the seam between the two, so that the transport can be exercised
//! with [`PassthroughCodec`] before a real codec is wired in.
//!
//! A sink only needs [`AudioDecoder`] and a source only needs [`AudioEncoder`];
//! [`AudioCodec`] is the convenience bound for something that does both.

use crate::types::{FrameDuration, SampleRate};

/// Errors produced by a codec.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CodecError {
    /// The requested configuration is not supported by this codec.
    UnsupportedConfiguration,
    /// The output buffer was too small for the decoded or encoded frame.
    BufferTooSmall,
    /// The codec failed to encode the frame.
    EncodeFailed,
    /// The codec failed to decode the frame (malformed or truncated input).
    DecodeFailed,
}

/// The codec configuration negotiated for one ASE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CodecConfig {
    /// Sampling rate.
    pub sampling_rate: SampleRate,
    /// Frame duration.
    pub frame_duration: FrameDuration,
    /// Number of audio channels carried by this ASE.
    ///
    /// A single ASE carries a single channel, so this is normally 1.
    pub channels: u8,
    /// Octets per codec frame, from the configured `Octets_Per_Codec_Frame`.
    pub octets_per_frame: u16,
}

impl CodecConfig {
    /// Create a configuration.
    pub const fn new(
        sampling_rate: SampleRate,
        frame_duration: FrameDuration,
        channels: u8,
        octets_per_frame: u16,
    ) -> Self {
        Self {
            sampling_rate,
            frame_duration,
            channels,
            octets_per_frame,
        }
    }

    /// PCM samples per frame per channel.
    pub const fn frame_samples(&self) -> usize {
        self.frame_duration.samples(self.sampling_rate)
    }

    /// Total PCM samples per frame across all channels.
    pub const fn pcm_frame_len(&self) -> usize {
        self.frame_samples() * self.channels as usize
    }
}

/// Decodes codec frames into PCM samples.
pub trait AudioDecoder {
    /// Error type.
    type Error;

    /// Decode one codec frame into `pcm`.
    ///
    /// `pcm` must have room for [`CodecConfig::pcm_frame_len`] samples; the
    /// excess, if any, is left untouched.
    fn decode(&mut self, frame: &[u8], pcm: &mut [i16]) -> Result<(), Self::Error>;
}

/// Encodes PCM samples into codec frames.
pub trait AudioEncoder {
    /// Error type.
    type Error;

    /// Encode one PCM frame into `frame`.
    ///
    /// `pcm` must hold [`CodecConfig::pcm_frame_len`] samples. Returns the
    /// number of octets written, which may be less than `frame.len()`.
    fn encode(&mut self, pcm: &[i16], frame: &mut [u8]) -> Result<usize, Self::Error>;
}

/// A codec that can both encode and decode with a common error type.
pub trait AudioCodec: AudioDecoder<Error = CodecError> + AudioEncoder<Error = CodecError> {}

impl<T> AudioCodec for T where T: AudioDecoder<Error = CodecError> + AudioEncoder<Error = CodecError> {}

/// A codec that copies PCM through unchanged.
///
/// This is not a real codec: it exists so the ISO transport, the ASE state
/// machine and the stream plumbing can be exercised without a codec, and it
/// keeps the frame format self-describing (little-endian `i16` samples).
#[derive(Debug, Clone, Copy)]
pub struct PassthroughCodec {
    config: CodecConfig,
}

impl PassthroughCodec {
    /// Create a passthrough codec for `config`.
    pub const fn new(config: CodecConfig) -> Self {
        Self { config }
    }

    /// The configuration this codec was created with.
    pub const fn config(&self) -> &CodecConfig {
        &self.config
    }
}

impl AudioDecoder for PassthroughCodec {
    type Error = CodecError;

    fn decode(&mut self, frame: &[u8], pcm: &mut [i16]) -> Result<(), Self::Error> {
        let n = self.config.pcm_frame_len();
        if pcm.len() < n {
            return Err(CodecError::BufferTooSmall);
        }
        if frame.len() < n * 2 {
            return Err(CodecError::DecodeFailed);
        }
        for (i, out) in pcm[..n].iter_mut().enumerate() {
            *out = i16::from_le_bytes([frame[i * 2], frame[i * 2 + 1]]);
        }
        Ok(())
    }
}

impl AudioEncoder for PassthroughCodec {
    type Error = CodecError;

    fn encode(&mut self, pcm: &[i16], frame: &mut [u8]) -> Result<usize, Self::Error> {
        let n = self.config.pcm_frame_len();
        if pcm.len() < n {
            return Err(CodecError::BufferTooSmall);
        }
        if frame.len() < n * 2 {
            return Err(CodecError::BufferTooSmall);
        }
        for (i, s) in pcm[..n].iter().enumerate() {
            frame[i * 2..i * 2 + 2].copy_from_slice(&s.to_le_bytes());
        }
        Ok(n * 2)
    }
}

#[cfg(feature = "codec-lc3")]
pub mod lc3 {
    //! The LC3 codec, backed by the pure-Rust `lc3-codec` crate.
    //!
    //! The `lc3-codec` API splits caller-provided working buffers, so the codec
    //! borrows its memory rather than owning it: the application allocates the
    //! buffers (typically in a `static`) and the codec is constructed over them.
    //! Buffer sizes depend on the configuration, so they are computed with
    //! [`decoder_buffer_lengths`] and [`encoder_buffer_lengths`], which are
    //! `const` and can therefore size a static array directly.
    //!
    //! Sizes for 48 kHz / 10 ms mono, for reference:
    //!
    //! | | integer (`i16`) | scaler (`f32`) | complex |
    //! |---|---|---|---|
    //! | encoder | 1900 | 1106 | 960 |
    //! | decoder | — | 4971 | 960 |
    //!
    //! That is roughly 15.9 KB for the encoder and 27.6 KB for the decoder.

    use lc3_codec::common::config::{FrameDuration as Lc3Duration, SamplingFrequency};
    use lc3_codec::decoder::lc3_decoder::Lc3Decoder as Lc3InnerDecoder;
    use lc3_codec::encoder::lc3_encoder::Lc3Encoder as Lc3InnerEncoder;

    use super::{AudioDecoder, AudioEncoder, CodecConfig, CodecError};
    use crate::types::{FrameDuration, SampleRate};

    pub use lc3_codec::common::complex::{Complex, Scaler};

    /// Map a negotiated configuration onto LC3 parameters.
    ///
    /// LC3 supports 8, 16, 24, 32, 44.1 and 48 kHz, so every [`SampleRate`]
    /// maps. Returns `None` only if a new rate is added that LC3 lacks.
    pub const fn lc3_config(cfg: &CodecConfig) -> Option<(SamplingFrequency, Lc3Duration)> {
        let freq = match cfg.sampling_rate {
            SampleRate::Hz8000 => SamplingFrequency::Hz8000,
            SampleRate::Hz16000 => SamplingFrequency::Hz16000,
            SampleRate::Hz24000 => SamplingFrequency::Hz24000,
            SampleRate::Hz32000 => SamplingFrequency::Hz32000,
            SampleRate::Hz44100 => SamplingFrequency::Hz44100,
            SampleRate::Hz48000 => SamplingFrequency::Hz48000,
        };
        let dur = match cfg.frame_duration {
            FrameDuration::Ms7_5 => Lc3Duration::SevenPointFiveMs,
            FrameDuration::Ms10 => Lc3Duration::TenMs,
        };
        Some((freq, dur))
    }

    /// Working-buffer lengths for a decoder: `(scaler, complex)`.
    pub const fn decoder_buffer_lengths(cfg: &CodecConfig) -> Option<(usize, usize)> {
        match lc3_config(cfg) {
            Some((freq, dur)) => Some(Lc3InnerDecoder::<1>::calc_working_buffer_lengths(dur, freq)),
            None => None,
        }
    }

    /// Working-buffer lengths for an encoder: `(integer, scaler, complex)`.
    pub const fn encoder_buffer_lengths(cfg: &CodecConfig) -> Option<(usize, usize, usize)> {
        match lc3_config(cfg) {
            Some((freq, dur)) => Some(Lc3InnerEncoder::<1>::calc_working_buffer_lengths(dur, freq)),
            None => None,
        }
    }

    /// An LC3 decoder for a single channel, borrowing caller-provided buffers.
    pub struct Lc3Decoder<'a> {
        inner: Lc3InnerDecoder<'a, 1>,
        config: CodecConfig,
    }

    impl<'a> Lc3Decoder<'a> {
        /// Create a decoder over `scaler` and `complex`.
        ///
        /// The buffers must be at least as long as
        /// [`decoder_buffer_lengths`] reports; longer buffers are fine.
        pub fn new(
            config: CodecConfig,
            scaler: &'a mut [Scaler],
            complex: &'a mut [Complex],
        ) -> Result<Self, CodecError> {
            let (freq, dur) = lc3_config(&config).ok_or(CodecError::UnsupportedConfiguration)?;
            Ok(Self {
                inner: Lc3InnerDecoder::<1>::new(dur, freq, scaler, complex),
                config,
            })
        }

        /// The configuration in use.
        pub const fn config(&self) -> &CodecConfig {
            &self.config
        }
    }

    impl AudioDecoder for Lc3Decoder<'_> {
        type Error = CodecError;

        fn decode(&mut self, frame: &[u8], pcm: &mut [i16]) -> Result<(), Self::Error> {
            let n = self.config.frame_samples();
            if pcm.len() < n {
                return Err(CodecError::BufferTooSmall);
            }
            self.inner
                .decode_frame(16, 0, frame, &mut pcm[..n])
                .map_err(|_| CodecError::DecodeFailed)
        }
    }

    /// An LC3 encoder for a single channel, borrowing caller-provided buffers.
    pub struct Lc3Encoder<'a> {
        inner: Lc3InnerEncoder<'a, 1>,
        config: CodecConfig,
    }

    impl<'a> Lc3Encoder<'a> {
        /// Create an encoder over `integer`, `scaler` and `complex`.
        ///
        /// The buffers must be at least as long as
        /// [`encoder_buffer_lengths`] reports; longer buffers are fine.
        pub fn new(
            config: CodecConfig,
            integer: &'a mut [i16],
            scaler: &'a mut [Scaler],
            complex: &'a mut [Complex],
        ) -> Result<Self, CodecError> {
            let (freq, dur) = lc3_config(&config).ok_or(CodecError::UnsupportedConfiguration)?;
            Ok(Self {
                inner: Lc3InnerEncoder::<1>::new(dur, freq, integer, scaler, complex),
                config,
            })
        }

        /// The configuration in use.
        pub const fn config(&self) -> &CodecConfig {
            &self.config
        }
    }

    impl AudioEncoder for Lc3Encoder<'_> {
        type Error = CodecError;

        fn encode(&mut self, pcm: &[i16], frame: &mut [u8]) -> Result<usize, Self::Error> {
            let n = self.config.frame_samples();
            if pcm.len() < n {
                return Err(CodecError::BufferTooSmall);
            }
            let octets = self.config.octets_per_frame as usize;
            if frame.len() < octets {
                return Err(CodecError::BufferTooSmall);
            }
            self.inner
                .encode_frame(0, &pcm[..n], &mut frame[..octets])
                .map_err(|_| CodecError::EncodeFailed)?;
            Ok(octets)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> CodecConfig {
        CodecConfig::new(SampleRate::Hz48000, FrameDuration::Ms10, 1, 40)
    }

    #[test]
    fn frame_geometry() {
        let c = cfg();
        assert_eq!(c.frame_samples(), 480);
        assert_eq!(c.pcm_frame_len(), 480);
        let stereo = CodecConfig::new(SampleRate::Hz48000, FrameDuration::Ms10, 2, 40);
        assert_eq!(stereo.pcm_frame_len(), 960);
    }

    #[test]
    fn passthrough_roundtrips() {
        let mut codec = PassthroughCodec::new(cfg());
        let pcm: [i16; 480] = core::array::from_fn(|i| (i as i16).wrapping_mul(37));
        let mut frame = [0u8; 960];
        let n = codec.encode(&pcm, &mut frame).unwrap();
        assert_eq!(n, 960);

        let mut out = [0i16; 480];
        codec.decode(&frame[..n], &mut out).unwrap();
        assert_eq!(out, pcm);
    }

    #[test]
    fn passthrough_rejects_short_buffers() {
        let mut codec = PassthroughCodec::new(cfg());
        let mut frame = [0u8; 4];
        assert_eq!(codec.encode(&[0i16; 480], &mut frame), Err(CodecError::BufferTooSmall));

        let mut out = [0i16; 4];
        assert_eq!(codec.decode(&[0u8; 960], &mut out), Err(CodecError::BufferTooSmall));
    }

    #[cfg(feature = "codec-lc3")]
    mod lc3_tests {
        use super::*;
        use crate::codec::lc3::{Complex, Scaler};

        const CFG: CodecConfig = CodecConfig::new(SampleRate::Hz48000, FrameDuration::Ms10, 1, 40);
        const DEC: (usize, usize) = match crate::codec::lc3::decoder_buffer_lengths(&CFG) {
            Some(v) => v,
            None => (0, 0),
        };
        const ENC: (usize, usize, usize) = match crate::codec::lc3::encoder_buffer_lengths(&CFG) {
            Some(v) => v,
            None => (0, 0, 0),
        };

        #[test]
        fn buffer_lengths_are_known_and_nonzero() {
            assert!(DEC.0 > 0 && DEC.1 > 0);
            assert!(ENC.0 > 0 && ENC.1 > 0 && ENC.2 > 0);
        }

        #[test]
        fn lc3_roundtrips_a_tone() {
            let mut integer = [0i16; ENC.0];
            let mut enc_scaler = [0 as Scaler; ENC.1];
            let mut enc_complex = [Complex::default(); ENC.2];
            let mut encoder =
                crate::codec::lc3::Lc3Encoder::new(CFG, &mut integer, &mut enc_scaler, &mut enc_complex).unwrap();

            // 1 kHz sine at 48 kHz.
            let pcm: [i16; 480] = core::array::from_fn(|i| {
                let t = i as f32 / 48000.0;
                (20000.0 * (2.0 * core::f32::consts::PI * 1000.0 * t).sin()) as i16
            });
            let mut frame = [0u8; 40];
            let n = encoder.encode(&pcm, &mut frame).unwrap();
            assert_eq!(n, 40);
            assert!(frame.iter().any(|&b| b != 0), "encoder produced an empty frame");

            let mut dec_scaler = [0 as Scaler; DEC.0];
            let mut dec_complex = [Complex::default(); DEC.1];
            let mut decoder = crate::codec::lc3::Lc3Decoder::new(CFG, &mut dec_scaler, &mut dec_complex).unwrap();
            let mut out = [0i16; 480];
            decoder.decode(&frame, &mut out).unwrap();

            let rms = |x: &[i16]| {
                let s: f32 = x.iter().map(|&v| (v as f32) * (v as f32)).sum();
                (s / x.len() as f32).sqrt()
            };
            let (rin, rout) = (rms(&pcm), rms(&out));
            assert!(rout > 1000.0, "decoded output near silent: {}", rout);
            let ratio = rout / rin;
            assert!((0.3..3.0).contains(&ratio), "energy ratio out of range: {}", ratio);
        }

        #[test]
        fn lc3_rejects_short_pcm() {
            let mut integer = [0i16; ENC.0];
            let mut enc_scaler = [0 as Scaler; ENC.1];
            let mut enc_complex = [Complex::default(); ENC.2];
            let mut encoder =
                crate::codec::lc3::Lc3Encoder::new(CFG, &mut integer, &mut enc_scaler, &mut enc_complex).unwrap();
            let mut frame = [0u8; 40];
            assert_eq!(encoder.encode(&[0i16; 10], &mut frame), Err(CodecError::BufferTooSmall));
        }
    }
}
