//! LE Audio on top of [trouble](https://github.com/embassy-rs/trouble).
//!
//! This crate implements the host-side parts of Bluetooth LE Audio that sit above
//! the HCI ISO data path: the Basic Audio Profile (BAP) data model, the Audio
//! Stream Endpoint (ASE) state machine, and the codec plumbing.
//!
//! # Layering
//!
//! ```text
//!   application
//!   trouble-audio   <- this crate: BAP data model, ASE state machine, codecs
//!   trouble-host    <- GATT, L2CAP, HCI ISO (CIS/BIS) routing
//!   bt-hci          <- HCI command/event/ISO packet types
//!   controller      <- e.g. STM32WBA Full link layer, or a Linux HCI socket
//! ```
//!
//! The wire types here are portable: they do not depend on `trouble-host`, so
//! they can be unit tested and reused without a controller.
//!
//! # References
//!
//! * Bluetooth `Basic Audio Profile` (BAP) and the Core specification, for the
//!   wire formats and the ASE state machine.
//! * ST `STM32CubeWBA`, `Middlewares/ST/STM32_WPAN/ble/audio/Inc/`, used as a
//!   cross-check for the type values and the ASE operation/response codes.
//!
//! # Scope
//!
//! Implemented: codec capabilities/configuration/metadata as LTV, the PAC record
//! format, QoS configuration, the ASE state machine, and the codec abstraction.
//! Not yet implemented: the PACS/ASCS GATT services, CIS setup, and ISO
//! streaming glue — those land in a follow-up and will depend on `trouble-host`.

#![no_std]
#![warn(missing_docs)]

pub mod ase;
pub mod bap;
pub mod codec;
pub mod ltv;
pub mod types;

pub use ase::{Ase, AseDirection, AseOperation, AseResponse, AseState};
pub use bap::{CodecSpecificConfig, PacRecord, QosConfig};
pub use codec::{AudioCodec, CodecConfig, CodecError, PassthroughCodec};
pub use ltv::{Ltv, LtvIter, LtvWriter};
pub use types::{AudioContext, AudioLocation, CodecId, Framing, Packing, Phy, SampleRate};
