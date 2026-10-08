//! Audio Stream Control Service (ASCS).
//!
//! Where PACS tells a peer what the device *can* do, ASCS is where the peer
//! actually configures it. Each Audio Stream Endpoint is its own characteristic,
//! and there is a single control point the peer writes to drive the ASEs through
//! their states.
//!
//! # Wiring it up
//!
//! The service only provides the attributes and the request handling; the
//! application owns the ASE state machine and drives the sequence, because only
//! it knows how to create a CIS. The shape of that is:
//!
//! ```text
//! // Once, when building the server:
//! let mut storage = AscsStorage::<2>::new();
//! let handles = ascs::add_service(&mut table, &ases, &initial_values, &mut storage);
//!
//! // In the GATT event loop:
//! GattEvent::Write(event) if event.handle() == handles.control_point => {
//!     let mut response = [0u8; ascs::MAX_RESPONSE_LEN];
//!     let outcome = event.with_data(|_, data| {
//!         ascs::handle_control_point(&mut ases, &capabilities, data, &mut response)
//!     });
//!     match outcome {
//!         ControlPointOutcome::Respond(len) => {
//!             event.accept()?;
//!             server.notify(&stack, handles.control_point, &response[..len]).await?;
//!         }
//!         ControlPointOutcome::Reject(err) => { event.reject(err)?; }
//!     }
//! }
//! ```
//!
//! Every ASE whose state changed then needs its new value written and reported,
//! which is [`encode_ase_value`] followed by
//! `AttributeServer::write_and_notify`, because an ASE characteristic's value
//! changes shape with its state.
//!
//! # Why the control point is not parsed here
//!
//! Parsing, validation and the ASE state machine live in the `trouble-audio`
//! crate, which has no dependencies and can therefore be tested without a
//! controller. [`handle_control_point`] is the two-line bridge to them.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::attribute::{AttributeTable, Service};
use crate::prelude::*;

use trouble_audio::{
    Ase, AseDirection, AseParams, AseResponse, AseState, AseStatusError, CodecCapabilities, ControlPointRequest,
    MAX_ASES_PER_OPERATION, MAX_VALUE_LEN,
};

use super::uuid;

/// The largest control point value the service will store.
///
/// A peer writes this characteristic with the longest configuration it can send,
/// and the attribute has to have room for it. It is also the ceiling on the
/// responses that can be indicated back.
pub const MAX_CONTROL_POINT_LEN: usize = 512;

/// The largest response that can be indicated back.
///
/// One entry per ASE the operation may address, each an identifier, a code and a
/// reason, behind the two-octet header.
pub const MAX_RESPONSE_LEN: usize = 2 + MAX_ASES_PER_OPERATION * 3;

/// One Audio Stream Endpoint, as the service needs to describe it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AseDesc {
    /// The identifier a peer uses to address this ASE.
    pub id: u8,
    /// Whether it sinks or sources audio, which decides its characteristic UUID.
    pub direction: AseDirection,
}

/// The backing storage the service needs.
///
/// The attribute table borrows this for as long as the server lives: an ASE's
/// value is rewritten whenever its state changes, so the characteristics cannot
/// be read-only.
pub struct AscsStorage<const N: usize> {
    /// Storage for each ASE characteristic value, in the order described.
    pub ases: [[u8; MAX_VALUE_LEN]; N],
    /// Storage for the control point value.
    pub control_point: [u8; MAX_CONTROL_POINT_LEN],
}

impl<const N: usize> AscsStorage<N> {
    /// Empty storage, ready to be handed to [`add_service`].
    pub const fn new() -> Self {
        Self {
            ases: [[0; MAX_VALUE_LEN]; N],
            control_point: [0; MAX_CONTROL_POINT_LEN],
        }
    }
}

impl<const N: usize> Default for AscsStorage<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Where the service's characteristics ended up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AscsHandles<const N: usize> {
    /// The value handle of each ASE characteristic, in the order described.
    pub ases: [u16; N],
    /// The value handle of the ASE Control Point.
    pub control_point: u16,
}

/// Add the ASCS service to `table`.
///
/// `initial` holds the encoded value of each ASE, which is `ASE_ID | ASE_State`
/// followed by the parameters that state carries; see [`encode_ase_value`].
/// `ases` and `initial` are in the same order, and that order is the one the
/// returned handles use.
///
/// Each ASE characteristic is readable and notifiable, and the control point is
/// writable and indicatable. Both properties bring a CCCD with them, which is
/// what a peer subscribes through and what
/// [`AttributeServer::notify`](crate::attribute_server::AttributeServer::notify)
/// checks before sending.
pub fn add_service<'a, M: RawMutex, const MAX: usize, const N: usize>(
    table: &mut AttributeTable<'a, M, MAX>,
    ases: &[AseDesc; N],
    initial: &[&[u8]; N],
    storage: &'a mut AscsStorage<N>,
) -> AscsHandles<N> {
    // Split the borrow of the storage up front: the table keeps each piece for
    // as long as the server lives, and the pieces are disjoint fields.
    let AscsStorage {
        ases: ase_stores,
        control_point: control_point_store,
    } = storage;

    let mut builder = table.add_service(Service::new(uuid::service::ASCS));
    let mut handles = AscsHandles {
        ases: [0; N],
        control_point: 0,
    };

    for (i, ((desc, value), store)) in ases.iter().zip(initial.iter()).zip(ase_stores.iter_mut()).enumerate() {
        let ase_uuid = match desc.direction {
            AseDirection::Sink => uuid::characteristic::ASCS_ASE_SINK,
            AseDirection::Source => uuid::characteristic::ASCS_ASE_SOURCE,
        };

        handles.ases[i] = builder
            .add_characteristic(
                ase_uuid,
                [CharacteristicProp::Read, CharacteristicProp::Notify],
                *value,
                store,
            )
            .build()
            .handle;
    }

    handles.control_point = builder
        .add_characteristic(
            uuid::characteristic::ASCS_ASE_CONTROL_POINT,
            [CharacteristicProp::Write, CharacteristicProp::Indicate],
            &[][..],
            &mut control_point_store[..],
        )
        .build()
        .handle;

    builder.build();
    handles
}

/// Encode an ASE's value into `out`, ready for its characteristic.
///
/// This is the `trouble-audio` encoder re-exported so that a caller driving the
/// service does not have to name both crates.
pub fn encode_ase_value(
    out: &mut [u8],
    ase_id: u8,
    state: AseState,
    params: AseParams<'_>,
) -> Result<usize, AseStatusError> {
    trouble_audio::status::encode(out, ase_id, state, params)
}

/// What to do with a write to the ASE Control Point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlPointOutcome {
    /// Accept the write, and indicate these octets of `response` back.
    Respond(usize),
    /// Reject the write with this ATT error, without a control point response.
    Reject(AttErrorCode),
}

/// Handle a write to the ASE Control Point.
///
/// The ASEs are left in whatever state the operation moved them to, and the
/// caller indicates `response[..len]` back through the control point.
///
/// A request whose framing cannot be parsed is refused at the ATT layer rather
/// than answered with a control point response, because the response has to name
/// the ASEs it is reporting on and a malformed request may not carry usable
/// identifiers. Zephyr takes the same route for a malformed length.
pub fn handle_control_point(
    ases: &mut [Ase],
    caps: &CodecCapabilities<'_>,
    request: &[u8],
    response: &mut [u8],
) -> ControlPointOutcome {
    let Ok(parsed) = ControlPointRequest::parse(request) else {
        return ControlPointOutcome::Reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH);
    };

    let result = trouble_audio::ascs::apply(ases, caps, &parsed);
    match result.write(response) {
        Ok(len) => ControlPointOutcome::Respond(len),
        // The response buffer is sized by `MAX_RESPONSE_LEN`, so this only
        // happens if a caller passed something smaller.
        Err(_) => ControlPointOutcome::Reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH),
    }
}

/// The code a peer receives when it writes a request this service cannot parse.
pub const MALFORMED_REQUEST: AseResponse = AseResponse::InvalidLength;

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    const ASE_IDLE: &[u8] = &[0x00, 0x00];

    fn build<'a>(table: &mut AttributeTable<'a, NoopRawMutex, 32>, storage: &'a mut AscsStorage<1>) -> AscsHandles<1> {
        let ases = [AseDesc {
            id: 0,
            direction: AseDirection::Sink,
        }];
        let initial: [&[u8]; 1] = [ASE_IDLE];
        add_service(table, &ases, &initial, storage)
    }

    fn uuid_at(table: &AttributeTable<'_, NoopRawMutex, 32>, handle: u16) -> Uuid {
        table.uuid(handle).expect("handle has a UUID")
    }

    #[test]
    fn adds_an_ase_and_a_control_point() {
        let mut storage = AscsStorage::<1>::new();
        let mut table = AttributeTable::<NoopRawMutex, 32>::new();
        let handles = build(&mut table, &mut storage);

        assert_eq!(uuid_at(&table, handles.ases[0]), uuid::characteristic::ASCS_ASE_SINK);
        assert_eq!(
            uuid_at(&table, handles.control_point),
            uuid::characteristic::ASCS_ASE_CONTROL_POINT
        );

        // Both need a CCCD: the ASE for its state notifications, the control
        // point for the response indication.
        for value_handle in [handles.ases[0], handles.control_point] {
            assert_eq!(
                uuid_at(&table, value_handle + 1),
                bt_hci::uuid::descriptors::CLIENT_CHARACTERISTIC_CONFIGURATION.into(),
                "missing CCCD for handle {value_handle}"
            );
        }

        // And the initial ASE value is readable.
        let mut value = [0u8; MAX_VALUE_LEN];
        let n = table.read(handles.ases[0], 0, &mut value).unwrap();
        assert_eq!(&value[..n], ASE_IDLE);
    }

    #[test]
    fn the_service_declaration_carries_the_ascs_uuid() {
        let mut storage = AscsStorage::<1>::new();
        let mut table = AttributeTable::<NoopRawMutex, 32>::new();
        build(&mut table, &mut storage);

        let mut value = [0u8; 2];
        let n = table.read(1, 0, &mut value).unwrap();
        assert_eq!(n, 2);
        assert_eq!(u16::from_le_bytes(value), 0x184e);
    }

    #[test]
    fn a_release_is_answered_with_a_success_response() {
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        // Drive the ASE to Codec Configured so that Release is accepted.
        ases[0].handle(trouble_audio::AseOperation::ConfigCodec);

        // Release ASE 0.
        let request = [0x08, 0x01, 0x00];
        let mut response = [0u8; MAX_RESPONSE_LEN];
        let outcome = handle_control_point(&mut ases, &CodecCapabilities::NONE, &request, &mut response);

        let ControlPointOutcome::Respond(len) = outcome else {
            panic!("expected a response, got {outcome:?}");
        };
        // Response opcode 0x00, one entry, ASE 0, success, no reason.
        assert_eq!(&response[..len], &[0x00, 0x01, 0x00, 0x00, 0x00]);
        assert_eq!(ases[0].state(), AseState::Releasing);
    }

    #[test]
    fn a_malformed_request_is_refused_at_the_att_layer() {
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        let mut response = [0u8; MAX_RESPONSE_LEN];

        // An unknown opcode.
        assert_eq!(
            handle_control_point(&mut ases, &CodecCapabilities::NONE, &[0x09, 0x01, 0x00], &mut response),
            ControlPointOutcome::Reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH)
        );

        // A release with trailing data.
        assert_eq!(
            handle_control_point(
                &mut ases,
                &CodecCapabilities::NONE,
                &[0x08, 0x01, 0x00, 0xAA],
                &mut response
            ),
            ControlPointOutcome::Reject(AttErrorCode::INVALID_ATTRIBUTE_VALUE_LENGTH)
        );

        // Nothing moved.
        assert_eq!(ases[0].state(), AseState::Idle);
    }

    #[test]
    fn an_unknown_ase_is_answered_per_ase_rather_than_refused() {
        let mut ases = [Ase::new(0, AseDirection::Sink)];
        let mut response = [0u8; MAX_RESPONSE_LEN];

        // Release ASE 7, which does not exist: this parses, so it is answered
        // rather than refused.
        let outcome = handle_control_point(&mut ases, &CodecCapabilities::NONE, &[0x08, 0x01, 0x07], &mut response);

        let ControlPointOutcome::Respond(len) = outcome else {
            panic!("expected a response, got {outcome:?}");
        };
        // ASE 7, response code 0x03 (Invalid ASE ID), no reason.
        assert_eq!(&response[..len], &[0x00, 0x01, 0x07, 0x03, 0x00]);
    }
}
