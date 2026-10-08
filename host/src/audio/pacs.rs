//! Published Audio Capabilities Service (PACS).
//!
//! PACS is how a peer learns what the server can play or capture before it
//! configures anything: for each direction, a set of PAC records describing the
//! codecs and codec-specific capabilities on offer, the physical loudspeaker or
//! microphone locations, and the audio contexts.
//!
//! # Scope
//!
//! This adds the six PACS characteristics as *readable* attributes. Most of them
//! are notifiable in the specification, because a server may add or remove a
//! capability while a peer is connected, but a device whose capabilities are
//! fixed at boot never needs to notify. Making them notifiable means using
//! [`CharacteristicProp::Notify`] instead of read-only, which also adds the CCCD
//! the peer subscribes through; it is not done speculatively here, because a
//! notify property without a CCCD would misdescribe the service.
//!
//! The PAC records are supplied already encoded, because encoding them is the
//! job of whatever knows the capabilities. `trouble-audio`'s `PacRecord` emits
//! exactly these bytes.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::attribute::{AttributeTable, Service};
use crate::prelude::*;

use super::uuid;

/// The PACS characteristics of one server.
///
/// A server that only sinks may pass an empty `source_pac`, and one that only
/// sources an empty `sink_pac`. Both characteristics are added either way, since
/// the peer reads them to find out which directions exist at all.
#[derive(Debug, Clone, Copy)]
pub struct PacsConfig<'a> {
    /// The sink PAC records, concatenated as they appear on the wire.
    pub sink_pac: &'a [u8],
    /// The sink's audio locations, a 32-bit bitfield.
    pub sink_locations: u32,
    /// The source PAC records, concatenated as they appear on the wire.
    pub source_pac: &'a [u8],
    /// The source's audio locations, a 32-bit bitfield.
    pub source_locations: u32,
    /// The audio contexts currently available to a sink.
    pub sink_available_contexts: u16,
    /// The audio contexts the server supports for a sink.
    pub sink_supported_contexts: u16,
    /// The audio contexts currently available to a source.
    pub source_available_contexts: u16,
    /// The audio contexts the server supports for a source.
    pub source_supported_contexts: u16,
}

impl<'a> PacsConfig<'a> {
    /// Add the PACS service to the attribute table.
    pub fn build<M: RawMutex, const MAX: usize>(self, table: &mut AttributeTable<'a, M, MAX>) {
        let mut builder = table.add_service(Service::new(uuid::service::PACS));

        builder.add_characteristic_ro(uuid::characteristic::PACS_SINK, self.sink_pac);
        builder.add_characteristic_small(
            uuid::characteristic::PACS_SINK_LOCATIONS,
            [CharacteristicProp::Read],
            self.sink_locations,
        );
        builder.add_characteristic_ro(uuid::characteristic::PACS_SOURCE, self.source_pac);
        builder.add_characteristic_small(
            uuid::characteristic::PACS_SOURCE_LOCATIONS,
            [CharacteristicProp::Read],
            self.source_locations,
        );
        builder.add_characteristic_small(
            uuid::characteristic::PACS_AVAILABLE_CONTEXTS,
            [CharacteristicProp::Read],
            self.sink_available_contexts,
        );
        builder.add_characteristic_small(
            uuid::characteristic::PACS_SUPPORTED_CONTEXTS,
            [CharacteristicProp::Read],
            self.sink_supported_contexts,
        );

        builder.build();
    }
}

/// A LC3 sink PAC record for 48 kHz, 10 ms, mono.
///
/// This is the shape a minimal unicast sink advertises, and it is here so that
/// callers do not have to hand-encode one. The encoding is
/// `Codec_ID(5) | Capabilities_Length(1) | Capabilities | Metadata_Length(1) |
/// Metadata`, with the capability block itself in LTV form.
pub const LC3_SINK_48K_10MS_MONO: &[u8] = &[
    // Codec_ID: LC3 is coding format 0x06, with no company or vendor id.
    0x06, 0x00, 0x00, 0x00, 0x00, // Capabilities: 19 octets, the five LTV entries below.
    0x13, // Supported_Sampling_Frequencies: 48 kHz is bit 15.
    0x03, 0x01, 0x00, 0x80, // Supported_Frame_Durations: bit 0 is 7.5 ms, bit 1 is 10 ms.
    0x02, 0x02, 0x02, // Supported_Audio_Channel_Counts: bit 0 is one channel.
    0x02, 0x03, 0x01, // Supported_Octets_Per_Codec_Frame: minimum 40, maximum 120.
    0x05, 0x04, 0x28, 0x00, 0x78, 0x00, // Supported_Max_Codec_Frames_Per_SDU.
    0x02, 0x05, 0x01, // No metadata.
    0x00,
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::uuid::Uuid;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    fn build(table: &mut AttributeTable<'static, NoopRawMutex, 32>) {
        PacsConfig {
            sink_pac: LC3_SINK_48K_10MS_MONO,
            sink_locations: 0x0000_0001,
            source_pac: &[],
            source_locations: 0,
            sink_available_contexts: 0x0001,
            sink_supported_contexts: 0x0003,
            source_available_contexts: 0,
            source_supported_contexts: 0,
        }
        .build(table);
    }

    fn has(table: &AttributeTable<'static, NoopRawMutex, 32>, want: Uuid) -> bool {
        (1..=table.len() as u16)
            .filter_map(|handle| table.uuid(handle))
            .any(|uuid| uuid == want)
    }

    #[test]
    fn adds_every_pacs_characteristic() {
        let mut table = AttributeTable::<NoopRawMutex, 32>::new();
        build(&mut table);

        for want in [
            uuid::characteristic::PACS_SINK,
            uuid::characteristic::PACS_SINK_LOCATIONS,
            uuid::characteristic::PACS_SOURCE,
            uuid::characteristic::PACS_SOURCE_LOCATIONS,
            uuid::characteristic::PACS_AVAILABLE_CONTEXTS,
            uuid::characteristic::PACS_SUPPORTED_CONTEXTS,
        ] {
            assert!(has(&table, want), "missing {want:?}");
        }
    }

    #[test]
    fn the_service_declaration_carries_the_pacs_uuid() {
        let mut table = AttributeTable::<NoopRawMutex, 32>::new();
        build(&mut table);

        // A primary service declaration's value is the service UUID; the
        // attribute's own UUID is the primary-service type.
        let mut value = [0u8; 2];
        let n = table.read(1, 0, &mut value).unwrap();
        assert_eq!(n, 2);
        assert_eq!(u16::from_le_bytes(value), 0x1850);
    }

    #[test]
    fn the_sink_pac_record_is_well_formed() {
        // Length prefixes have to add up, or a peer reading the characteristic
        // gets a truncated record with no way to tell.
        let caps_len = LC3_SINK_48K_10MS_MONO[5] as usize;
        let meta_at = 5 + 1 + caps_len;
        assert!(meta_at + 1 <= LC3_SINK_48K_10MS_MONO.len());
        let meta_len = LC3_SINK_48K_10MS_MONO[meta_at] as usize;
        assert_eq!(meta_at + 1 + meta_len, LC3_SINK_48K_10MS_MONO.len());
    }
}
