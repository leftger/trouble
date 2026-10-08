//! UUIDs for the LE Audio services implemented here.
//!
//! These are assigned numbers from the Bluetooth SIG. They were checked against
//! Zephyr's `include/zephyr/bluetooth/uuid.h` rather than written from memory:
//! the `ASE` characteristics in particular are easy to misremember, because ASCS
//! exposes one characteristic *per ASE* rather than a single state
//! characteristic.
//!
//! They are defined here rather than in `bt-hci`, whose `uuid` module supplies
//! the rest of the stack, because nothing outside this module needs them yet. If
//! the audio support grows past the host, that is where they belong.

/// LE Audio service UUIDs.
pub mod service {
    use crate::types::uuid::Uuid;

    /// Published Audio Capabilities Service.
    pub const PACS: Uuid = Uuid::new_short(0x1850);

    /// Audio Stream Control Service.
    pub const ASCS: Uuid = Uuid::new_short(0x184e);
}

/// LE Audio characteristic UUIDs.
pub mod characteristic {
    use crate::types::uuid::Uuid;

    /// Sink PAC: the published sink audio capabilities, as concatenated PAC
    /// records.
    pub const PACS_SINK: Uuid = Uuid::new_short(0x2bc9);
    /// Sink Audio Locations: a 32-bit bitfield of the sink's locations.
    pub const PACS_SINK_LOCATIONS: Uuid = Uuid::new_short(0x2bca);
    /// Source PAC: the published source audio capabilities.
    pub const PACS_SOURCE: Uuid = Uuid::new_short(0x2bcb);
    /// Source Audio Locations.
    pub const PACS_SOURCE_LOCATIONS: Uuid = Uuid::new_short(0x2bcc);
    /// Available Audio Contexts: the contexts that may still be used.
    pub const PACS_AVAILABLE_CONTEXTS: Uuid = Uuid::new_short(0x2bcd);
    /// Supported Audio Contexts: the contexts the server supports.
    pub const PACS_SUPPORTED_CONTEXTS: Uuid = Uuid::new_short(0x2bce);

    /// ASE Sink: one instance per sink Audio Stream Endpoint. The value is the
    /// ASE state and, when configured, the parameters for that state.
    pub const ASCS_ASE_SINK: Uuid = Uuid::new_short(0x2bc4);
    /// ASE Source: one instance per source Audio Stream Endpoint.
    pub const ASCS_ASE_SOURCE: Uuid = Uuid::new_short(0x2bc5);
    /// ASE Control Point: written by the peer, indicated back with the response.
    pub const ASCS_ASE_CONTROL_POINT: Uuid = Uuid::new_short(0x2bc6);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::uuid::Uuid;

    #[test]
    fn uuid_values_are_the_assigned_numbers() {
        assert_eq!(service::PACS, Uuid::new_short(0x1850));
        assert_eq!(service::ASCS, Uuid::new_short(0x184e));

        // PACS characteristics are consecutive from 0x2bc9.
        assert_eq!(characteristic::PACS_SINK, Uuid::new_short(0x2bc9));
        assert_eq!(characteristic::PACS_SINK_LOCATIONS, Uuid::new_short(0x2bca));
        assert_eq!(characteristic::PACS_SOURCE, Uuid::new_short(0x2bcb));
        assert_eq!(characteristic::PACS_SOURCE_LOCATIONS, Uuid::new_short(0x2bcc));
        assert_eq!(characteristic::PACS_AVAILABLE_CONTEXTS, Uuid::new_short(0x2bcd));
        assert_eq!(characteristic::PACS_SUPPORTED_CONTEXTS, Uuid::new_short(0x2bce));

        // ASCS characteristics are consecutive from 0x2bc4, and the ASE
        // characteristics come before the control point.
        assert_eq!(characteristic::ASCS_ASE_SINK, Uuid::new_short(0x2bc4));
        assert_eq!(characteristic::ASCS_ASE_SOURCE, Uuid::new_short(0x2bc5));
        assert_eq!(characteristic::ASCS_ASE_CONTROL_POINT, Uuid::new_short(0x2bc6));
    }
}
