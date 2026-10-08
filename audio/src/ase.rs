//! Audio Stream Endpoint (ASE) state machine.
//!
//! An ASE is one direction of one audio stream on a Unicast Server. The peer
//! drives it with operations written to the ASE Control Point; each operation
//! moves the ASE between the states defined by BAP.
//!
//! ```text
//!                    Config Codec
//!        Idle ───────────────────────► Codec Configured
//!         ▲                                 │  ▲
//!         │                         Config QoS │  │ Config Codec
//!         │                                 ▼  │
//!         │                          QoS Configured
//!         │                                 │  ▲
//!         │                            Enable│  │ Receiver Stop Ready (sink)
//!         │                                 ▼  │
//!         │                             Enabling
//!         │                                 │  │
//!         │            Receiver Start Ready │  │ Disable
//!         │                                 ▼  │
//!         │                             Streaming ──► Disabling
//!         │                                              │
//!         └──────────── Release ◄────────────────────────┘
//! ```
//!
//! `Release` is available from every state except `Idle`, and always passes
//! through `Releasing` before returning to `Idle` via [`Ase::complete_release`].
//!
//! The exact set of operations that carry a Sink ASE from `Enabling` to
//! `Streaming` (and `Disabling` back to `QoS Configured`) differs by direction:
//! `Receiver Start Ready` and `Receiver Stop Ready` only apply to Sink ASEs, so
//! requesting them on a Source ASE yields
//! [`AseResponse::InvalidAseDirection`]. A Source ASE completes a `Disable` with
//! [`Ase::complete_disable`] instead.

/// State of an ASE, as reported in the ASE characteristic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AseState {
    /// No codec or QoS configuration applied.
    Idle,
    /// Codec configuration applied; QoS not yet applied.
    CodecConfigured,
    /// Codec and QoS configuration applied.
    QosConfigured,
    /// Enabled, and the CIS is being established.
    Enabling,
    /// Streaming; the ASE is coupled to a CIS and audio may flow.
    Streaming,
    /// Being decoupled from the CIS.
    Disabling,
    /// Being released; any CIS is disconnecting.
    Releasing,
}

impl AseState {
    /// The wire value.
    pub const fn to_u8(self) -> u8 {
        match self {
            AseState::Idle => 0x00,
            AseState::CodecConfigured => 0x01,
            AseState::QosConfigured => 0x02,
            AseState::Enabling => 0x03,
            AseState::Streaming => 0x04,
            AseState::Disabling => 0x05,
            AseState::Releasing => 0x06,
        }
    }

    /// Parse a wire value.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(AseState::Idle),
            0x01 => Some(AseState::CodecConfigured),
            0x02 => Some(AseState::QosConfigured),
            0x03 => Some(AseState::Enabling),
            0x04 => Some(AseState::Streaming),
            0x05 => Some(AseState::Disabling),
            0x06 => Some(AseState::Releasing),
            _ => None,
        }
    }

    /// Whether a codec configuration is applied.
    pub const fn is_codec_configured(self) -> bool {
        !matches!(self, AseState::Idle)
    }

    /// Whether audio may flow.
    pub const fn is_streaming(self) -> bool {
        matches!(self, AseState::Streaming)
    }
}

/// Direction of an ASE, from the server's point of view.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AseDirection {
    /// The server receives audio (a Sink ASE, characteristic `0x2BC4`).
    Sink,
    /// The server transmits audio (a Source ASE, characteristic `0x2BC5`).
    Source,
}

/// An operation written to the ASE Control Point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AseOperation {
    /// Apply a codec configuration.
    ConfigCodec,
    /// Apply a QoS configuration.
    ConfigQos,
    /// Enable the ASE.
    Enable,
    /// The receiver is ready to start (Sink ASEs only).
    ReceiverStartReady,
    /// Disable the ASE.
    Disable,
    /// The receiver has stopped (Sink ASEs only).
    ReceiverStopReady,
    /// Update the metadata without changing anything else.
    UpdateMetadata,
    /// Release the ASE.
    Release,
}

impl AseOperation {
    /// The wire value.
    pub const fn to_u8(self) -> u8 {
        match self {
            AseOperation::ConfigCodec => 0x01,
            AseOperation::ConfigQos => 0x02,
            AseOperation::Enable => 0x03,
            AseOperation::ReceiverStartReady => 0x04,
            AseOperation::Disable => 0x05,
            AseOperation::ReceiverStopReady => 0x06,
            AseOperation::UpdateMetadata => 0x07,
            AseOperation::Release => 0x08,
        }
    }

    /// Parse a wire value.
    pub const fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(AseOperation::ConfigCodec),
            0x02 => Some(AseOperation::ConfigQos),
            0x03 => Some(AseOperation::Enable),
            0x04 => Some(AseOperation::ReceiverStartReady),
            0x05 => Some(AseOperation::Disable),
            0x06 => Some(AseOperation::ReceiverStopReady),
            0x07 => Some(AseOperation::UpdateMetadata),
            0x08 => Some(AseOperation::Release),
            _ => None,
        }
    }
}

/// Result code returned for an ASE Control Point operation.
///
/// The codes are grouped by family. After the general codes comes
/// codec-specific configuration (0x07-0x09), QoS configuration (0x0A-0x0C),
/// `InsufficientResources`/`UnspecifiedError`, then metadata (0x0F-0x11) — each
/// family having an unsupported, rejected and invalid variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum AseResponse {
    Success,
    UnsupportedOpcode,
    InvalidLength,
    InvalidAseId,
    InvalidAseState,
    InvalidAseDirection,
    UnsupportedAudioCapability,
    UnsupportedCodecConfiguration,
    RejectedCodecConfiguration,
    InvalidCodecConfiguration,
    UnsupportedQosConfiguration,
    RejectedQosConfiguration,
    InvalidQosConfiguration,
    InsufficientResources,
    UnspecifiedError,
    UnsupportedMetadata,
    RejectedMetadata,
    InvalidMetadata,
}

impl AseResponse {
    /// The wire value.
    pub const fn to_u8(self) -> u8 {
        match self {
            AseResponse::Success => 0x00,
            AseResponse::UnsupportedOpcode => 0x01,
            AseResponse::InvalidLength => 0x02,
            AseResponse::InvalidAseId => 0x03,
            AseResponse::InvalidAseState => 0x04,
            AseResponse::InvalidAseDirection => 0x05,
            AseResponse::UnsupportedAudioCapability => 0x06,
            AseResponse::UnsupportedCodecConfiguration => 0x07,
            AseResponse::RejectedCodecConfiguration => 0x08,
            AseResponse::InvalidCodecConfiguration => 0x09,
            AseResponse::UnsupportedQosConfiguration => 0x0A,
            AseResponse::RejectedQosConfiguration => 0x0B,
            AseResponse::InvalidQosConfiguration => 0x0C,
            AseResponse::InsufficientResources => 0x0D,
            AseResponse::UnspecifiedError => 0x0E,
            AseResponse::UnsupportedMetadata => 0x0F,
            AseResponse::RejectedMetadata => 0x10,
            AseResponse::InvalidMetadata => 0x11,
        }
    }

    /// Parse a wire value.
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0x00 => AseResponse::Success,
            0x01 => AseResponse::UnsupportedOpcode,
            0x02 => AseResponse::InvalidLength,
            0x03 => AseResponse::InvalidAseId,
            0x04 => AseResponse::InvalidAseState,
            0x05 => AseResponse::InvalidAseDirection,
            0x06 => AseResponse::UnsupportedAudioCapability,
            0x07 => AseResponse::UnsupportedCodecConfiguration,
            0x08 => AseResponse::RejectedCodecConfiguration,
            0x09 => AseResponse::InvalidCodecConfiguration,
            0x0A => AseResponse::UnsupportedQosConfiguration,
            0x0B => AseResponse::RejectedQosConfiguration,
            0x0C => AseResponse::InvalidQosConfiguration,
            0x0D => AseResponse::InsufficientResources,
            0x0E => AseResponse::UnspecifiedError,
            0x0F => AseResponse::UnsupportedMetadata,
            0x10 => AseResponse::RejectedMetadata,
            0x11 => AseResponse::InvalidMetadata,
            _ => return None,
        })
    }

    /// Whether the operation was accepted.
    pub const fn is_success(self) -> bool {
        matches!(self, AseResponse::Success)
    }
}

/// One Audio Stream Endpoint.
#[derive(Debug, Clone, Copy)]
pub struct Ase {
    id: u8,
    direction: AseDirection,
    state: AseState,
}

impl Ase {
    /// Create an ASE in [`AseState::Idle`].
    pub const fn new(id: u8, direction: AseDirection) -> Self {
        Self {
            id,
            direction,
            state: AseState::Idle,
        }
    }

    /// The ASE ID.
    pub const fn id(&self) -> u8 {
        self.id
    }

    /// The current state.
    pub const fn state(&self) -> AseState {
        self.state
    }

    /// The ASE direction.
    pub const fn direction(&self) -> AseDirection {
        self.direction
    }

    /// Whether audio may flow.
    pub const fn is_streaming(&self) -> bool {
        self.state.is_streaming()
    }

    /// Apply an operation, moving to the next state if the operation is valid.
    ///
    /// On success the state is updated; on failure the state is left untouched
    /// and the returned [`AseResponse`] explains why.
    pub fn handle(&mut self, op: AseOperation) -> AseResponse {
        use AseOperation::*;
        use AseState::*;

        let next = match (op, self.state, self.direction) {
            // Codec configuration may be (re)applied until the ASE is enabled.
            (ConfigCodec, Idle | CodecConfigured | QosConfigured, _) => CodecConfigured,

            // QoS configuration may be (re)applied until the ASE is enabled.
            (ConfigQos, CodecConfigured | QosConfigured, _) => QosConfigured,

            (Enable, QosConfigured, _) => Enabling,

            // Sink-only: the receiver signals readiness.
            (ReceiverStartReady, Enabling, AseDirection::Sink) => Streaming,
            (ReceiverStartReady, _, AseDirection::Source) => return AseResponse::InvalidAseDirection,

            (Disable, Streaming, _) => Disabling,

            // Sink-only: the receiver signals it has stopped.
            (ReceiverStopReady, Disabling, AseDirection::Sink) => QosConfigured,
            (ReceiverStopReady, _, AseDirection::Source) => return AseResponse::InvalidAseDirection,

            // Metadata may be updated in any configured state, without a transition.
            (UpdateMetadata, CodecConfigured | QosConfigured | Enabling | Streaming | Disabling, _) => self.state,

            // Release is allowed from every state except Idle.
            (Release, CodecConfigured | QosConfigured | Enabling | Streaming | Disabling | Releasing, _) => Releasing,

            _ => return AseResponse::InvalidAseState,
        };

        self.state = next;
        AseResponse::Success
    }

    /// Whether [`Ase::handle`] would accept `op`, without changing the state.
    ///
    /// Used to check every ASE in a multi-ASE operation before applying it to
    /// any of them.
    pub fn can_handle(&self, op: AseOperation) -> AseResponse {
        let mut probe = *self;
        probe.handle(op)
    }

    /// Finish a release, moving from [`AseState::Releasing`] to [`AseState::Idle`].
    pub fn complete_release(&mut self) -> AseResponse {
        if self.state == AseState::Releasing {
            self.state = AseState::Idle;
            AseResponse::Success
        } else {
            AseResponse::InvalidAseState
        }
    }

    /// Finish a disable on a Source ASE, moving from [`AseState::Disabling`] to
    /// [`AseState::QosConfigured`].
    ///
    /// A Sink ASE uses [`AseOperation::ReceiverStopReady`] instead.
    pub fn complete_disable(&mut self) -> AseResponse {
        if self.state == AseState::Disabling {
            self.state = AseState::QosConfigured;
            AseResponse::Success
        } else {
            AseResponse::InvalidAseState
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_and_op_codes_roundtrip() {
        for v in 0x00..=0x06u8 {
            let s = AseState::from_u8(v).unwrap();
            assert_eq!(s.to_u8(), v);
        }
        assert_eq!(AseState::from_u8(0x07), None);

        for v in 0x01..=0x08u8 {
            let op = AseOperation::from_u8(v).unwrap();
            assert_eq!(op.to_u8(), v);
        }
        assert_eq!(AseOperation::from_u8(0x00), None);
        assert_eq!(AseOperation::from_u8(0x09), None);
    }

    #[test]
    fn response_codes_match_bap() {
        // Every code from Success through Invalid Metadata must round-trip.
        for v in 0x00..=0x11u8 {
            let code = AseResponse::from_u8(v).unwrap();
            assert_eq!(code.to_u8(), v);
        }
        assert_eq!(AseResponse::from_u8(0x12), None);

        assert_eq!(AseResponse::Success.to_u8(), 0x00);
        assert_eq!(AseResponse::InvalidAseState.to_u8(), 0x04);
        assert_eq!(AseResponse::InvalidAseDirection.to_u8(), 0x05);
        // The three families sit in separate ranges, each unsupported/rejected/
        // invalid in turn.
        assert_eq!(AseResponse::UnsupportedCodecConfiguration.to_u8(), 0x07);
        assert_eq!(AseResponse::RejectedCodecConfiguration.to_u8(), 0x08);
        assert_eq!(AseResponse::InvalidCodecConfiguration.to_u8(), 0x09);
        assert_eq!(AseResponse::UnsupportedQosConfiguration.to_u8(), 0x0A);
        assert_eq!(AseResponse::RejectedQosConfiguration.to_u8(), 0x0B);
        assert_eq!(AseResponse::InvalidQosConfiguration.to_u8(), 0x0C);
        assert_eq!(AseResponse::InsufficientResources.to_u8(), 0x0D);
        assert_eq!(AseResponse::UnspecifiedError.to_u8(), 0x0E);
        assert_eq!(AseResponse::UnsupportedMetadata.to_u8(), 0x0F);
        assert_eq!(AseResponse::RejectedMetadata.to_u8(), 0x10);
        assert_eq!(AseResponse::InvalidMetadata.to_u8(), 0x11);
    }

    #[test]
    fn can_handle_does_not_mutate() {
        let ase = Ase::new(0, AseDirection::Sink);
        assert!(ase.can_handle(AseOperation::ConfigCodec).is_success());
        assert_eq!(ase.state(), AseState::Idle);

        // Not valid from Idle, and still no mutation.
        assert_eq!(ase.can_handle(AseOperation::Enable), AseResponse::InvalidAseState);
        assert_eq!(ase.state(), AseState::Idle);
    }

    #[test]
    fn full_sink_lifecycle() {
        let mut ase = Ase::new(0, AseDirection::Sink);
        assert_eq!(ase.state(), AseState::Idle);

        assert!(ase.handle(AseOperation::ConfigCodec).is_success());
        assert_eq!(ase.state(), AseState::CodecConfigured);

        assert!(ase.handle(AseOperation::ConfigQos).is_success());
        assert_eq!(ase.state(), AseState::QosConfigured);

        assert!(ase.handle(AseOperation::Enable).is_success());
        assert_eq!(ase.state(), AseState::Enabling);

        assert!(ase.handle(AseOperation::ReceiverStartReady).is_success());
        assert_eq!(ase.state(), AseState::Streaming);
        assert!(ase.is_streaming());

        assert!(ase.handle(AseOperation::Disable).is_success());
        assert_eq!(ase.state(), AseState::Disabling);

        assert!(ase.handle(AseOperation::ReceiverStopReady).is_success());
        assert_eq!(ase.state(), AseState::QosConfigured);

        assert!(ase.handle(AseOperation::Release).is_success());
        assert_eq!(ase.state(), AseState::Releasing);

        assert!(ase.complete_release().is_success());
        assert_eq!(ase.state(), AseState::Idle);
    }

    #[test]
    fn source_disable_completes_without_receiver_stop_ready() {
        let mut ase = Ase::new(0, AseDirection::Source);
        // Drive a Source ASE up to Streaming.
        for op in [AseOperation::ConfigCodec, AseOperation::ConfigQos, AseOperation::Enable] {
            assert!(ase.handle(op).is_success());
        }
        assert_eq!(ase.state(), AseState::Enabling);

        // A Source ASE does not use Receiver Start Ready.
        assert_eq!(
            ase.handle(AseOperation::ReceiverStartReady),
            AseResponse::InvalidAseDirection
        );
        assert_eq!(ase.state(), AseState::Enabling);
    }

    #[test]
    fn receiver_ops_are_invalid_for_source() {
        let mut ase = Ase::new(1, AseDirection::Source);
        assert_eq!(
            ase.handle(AseOperation::ReceiverStartReady),
            AseResponse::InvalidAseDirection
        );
        assert_eq!(
            ase.handle(AseOperation::ReceiverStopReady),
            AseResponse::InvalidAseDirection
        );
    }

    #[test]
    fn invalid_state_transitions_are_rejected_and_leave_state_untouched() {
        let mut ase = Ase::new(0, AseDirection::Sink);

        // Cannot configure QoS before a codec configuration.
        assert_eq!(ase.handle(AseOperation::ConfigQos), AseResponse::InvalidAseState);
        assert_eq!(ase.state(), AseState::Idle);

        // Cannot enable before QoS is configured.
        assert_eq!(ase.handle(AseOperation::Enable), AseResponse::InvalidAseState);
        assert_eq!(ase.state(), AseState::Idle);

        // Cannot disable while idle.
        assert_eq!(ase.handle(AseOperation::Disable), AseResponse::InvalidAseState);

        // Cannot release from Idle.
        assert_eq!(ase.handle(AseOperation::Release), AseResponse::InvalidAseState);
        assert_eq!(ase.state(), AseState::Idle);
    }

    #[test]
    fn release_is_allowed_from_every_configured_state() {
        for start in [
            AseState::CodecConfigured,
            AseState::QosConfigured,
            AseState::Enabling,
            AseState::Streaming,
            AseState::Disabling,
        ] {
            // Walk to the target state.
            let mut ase = Ase::new(0, AseDirection::Sink);
            let path: &[AseOperation] = match start {
                AseState::CodecConfigured => &[AseOperation::ConfigCodec],
                AseState::QosConfigured => &[AseOperation::ConfigCodec, AseOperation::ConfigQos],
                AseState::Enabling => &[AseOperation::ConfigCodec, AseOperation::ConfigQos, AseOperation::Enable],
                AseState::Streaming => &[
                    AseOperation::ConfigCodec,
                    AseOperation::ConfigQos,
                    AseOperation::Enable,
                    AseOperation::ReceiverStartReady,
                ],
                AseState::Disabling => &[
                    AseOperation::ConfigCodec,
                    AseOperation::ConfigQos,
                    AseOperation::Enable,
                    AseOperation::ReceiverStartReady,
                    AseOperation::Disable,
                ],
                other => panic!("unexpected start state {:?}", other),
            };
            for &op in path {
                assert!(ase.handle(op).is_success(), "walk to {:?}", start);
            }
            assert_eq!(ase.state(), start);
            assert!(ase.handle(AseOperation::Release).is_success());
            assert_eq!(ase.state(), AseState::Releasing);
        }
    }

    #[test]
    fn update_metadata_keeps_the_state() {
        let mut ase = Ase::new(0, AseDirection::Sink);
        ase.handle(AseOperation::ConfigCodec);
        ase.handle(AseOperation::ConfigQos);
        ase.handle(AseOperation::Enable);
        ase.handle(AseOperation::ReceiverStartReady);
        assert_eq!(ase.state(), AseState::Streaming);

        assert!(ase.handle(AseOperation::UpdateMetadata).is_success());
        assert_eq!(ase.state(), AseState::Streaming);

        // ...but not while Idle.
        let mut idle = Ase::new(1, AseDirection::Sink);
        assert_eq!(idle.handle(AseOperation::UpdateMetadata), AseResponse::InvalidAseState);
    }

    #[test]
    fn codec_and_qos_can_be_reconfigured_before_enabling() {
        let mut ase = Ase::new(0, AseDirection::Sink);
        ase.handle(AseOperation::ConfigCodec);
        ase.handle(AseOperation::ConfigQos);
        // Re-apply both while still in QoS Configured.
        assert!(ase.handle(AseOperation::ConfigCodec).is_success());
        assert_eq!(ase.state(), AseState::CodecConfigured);
        assert!(ase.handle(AseOperation::ConfigQos).is_success());
        assert_eq!(ase.state(), AseState::QosConfigured);
    }
}
