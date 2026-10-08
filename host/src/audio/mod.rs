//! LE Audio support.
//!
//! This is the profile layer of Bluetooth LE Audio: the GATT services a peer
//! needs in order to discover audio capabilities and configure a stream. It sits
//! above the ISO support that already lives in this crate
//! ([`crate::iso`]), which handles the isochronous channels themselves.
//!
//! # What is here
//!
//! * [`pacs`] — the Published Audio Capabilities Service (0x1850), which a peer
//!   reads to find out what the server can play or capture.
//!
//! # What is not here yet
//!
//! * ASCS (0x184e), the Audio Stream Control Service. It is the larger half: one
//!   characteristic per Audio Stream Endpoint, a writable control point, and
//!   indications for the control-point response. The control point's framing and
//!   validation are already implemented in the `trouble-audio` crate, which is
//!   deliberately dependency-free so that logic can be unit-tested without a
//!   controller; this module is where it becomes a service.
//! * The glue from an ASE reaching `Enabling` to a CIG/CIS being created and an
//!   ISO data path being set up.
//!
//! # UUIDs
//!
//! [`uuid`] holds the assigned numbers, kept local rather than added to `bt-hci`
//! because nothing outside this module needs them yet.

pub mod pacs;
pub mod uuid;

pub use pacs::PacsConfig;
