//! Isochronous (CIS/BIS) HCI commands and data.

use bt_hci::cmd::{AsyncCmd, SyncCmd};
use bt_hci::controller::{Controller, ControllerCmdAsync, ControllerCmdSync};
use bt_hci::data::IsoPacket;
use bt_hci::event::le::{LeCisEstablished, LeCisRequest};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Channel;

use crate::host::{BleHost, EventHandler};
use crate::{BleHostError, PacketPool};

/// A type for running isochronous-stream HCI commands and data.
pub struct Iso<'stack, C, P: PacketPool> {
    host: BleHost<'stack, C, P>,
}

impl<'stack, C: Controller, P: PacketPool> Iso<'stack, C, P> {
    pub(crate) fn new(host: BleHost<'stack, C, P>) -> Self {
        Self { host }
    }

    /// Run a synchronous HCI command and return its response.
    pub async fn command<Cmd>(&self, cmd: Cmd) -> Result<Cmd::Return, BleHostError<C::Error>>
    where
        Cmd: SyncCmd,
        C: ControllerCmdSync<Cmd>,
    {
        self.host.command(cmd).await
    }

    /// Run an asynchronous HCI command (one whose completion arrives as a separate event, e.g.
    /// `LE Accept CIS Request`'s `LE CIS Established`) without waiting for that event.
    pub async fn command_async<Cmd>(&self, cmd: Cmd) -> Result<(), BleHostError<C::Error>>
    where
        Cmd: AsyncCmd,
        C: ControllerCmdAsync<Cmd>,
    {
        self.host.async_command(cmd).await
    }

    /// Write a raw HCI ISO data packet to the controller.
    pub async fn send(&self, packet: &IsoPacket<'_>) -> Result<(), C::Error> {
        self.host.write_iso_data(packet).await
    }
}

/// Queues carrying the CIS control events from the runner to the application.
///
/// The runner delivers CIS events and incoming ISO data to an
/// [`EventHandler`](crate::host::EventHandler), which is otherwise a callback
/// with nowhere to await. This is a handler that turns the CIS half of those
/// events into something awaitable:
///
/// ```text
/// let events: IsoEvents<NoopRawMutex, 2, 2> = IsoEvents::new();
///
/// join(runner.run_with_handler(&events), async {
///     // Wait for the controller to ask whether we want this CIS...
///     let request = events.request().await;
///
///     // ...accept it, and wait for the outcome.
///     iso.command_async(LeAcceptCisRequest::new(request.cis_handle)).await?;
///     let established = events.established().await;
///     if !established.status.is_ok() { return; }
///
///     // The handle the data path and any traffic uses.
///     let handle = established.handle;
/// })
/// .await;
/// ```
///
/// The queues *drop* events when they are full rather than applying
/// backpressure: these arrive from the controller's event path, and there is no
/// sensible place to block there, so an application that lets them pile up loses
/// information instead of stalling the link. Sizing them is the application's
/// job.
pub struct IsoEvents<M: RawMutex, const REQ: usize, const EST: usize> {
    requests: Channel<M, LeCisRequest, REQ>,
    established: Channel<M, LeCisEstablished, EST>,
}

impl<M: RawMutex, const REQ: usize, const EST: usize> IsoEvents<M, REQ, EST> {
    /// Create the queues.
    pub const fn new() -> Self {
        Self {
            requests: Channel::new(),
            established: Channel::new(),
        }
    }

    /// Wait for the controller to ask whether to accept a CIS.
    pub async fn request(&self) -> LeCisRequest {
        self.requests.receive().await
    }

    /// Wait for a CIS to come up, whether or not it succeeded — check the
    /// status before using the handle.
    pub async fn established(&self) -> LeCisEstablished {
        self.established.receive().await
    }
}

impl<M: RawMutex, const REQ: usize, const EST: usize> Default for IsoEvents<M, REQ, EST> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M: RawMutex, const REQ: usize, const EST: usize> EventHandler for IsoEvents<M, REQ, EST> {
    fn on_cis_request(&self, event: &LeCisRequest) {
        let _ = self.requests.try_send(event.clone());
    }

    fn on_cis_established(&self, event: &LeCisEstablished) {
        let _ = self.established.try_send(event.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bt_hci::FromHciBytes;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    /// `LE CIS Request`: ACL handle, CIS handle, CIG id, CIS id.
    fn cis_request_bytes(acl: u16, cis: u16, cig: u8, cis_id: u8) -> [u8; 6] {
        let a = acl.to_le_bytes();
        let c = cis.to_le_bytes();
        [a[0], a[1], c[0], c[1], cig, cis_id]
    }

    #[test]
    fn a_cis_request_reaches_the_application() {
        let events: IsoEvents<NoopRawMutex, 2, 2> = IsoEvents::new();

        let request = LeCisRequest::from_hci_bytes_complete(&cis_request_bytes(0x0040, 0x0002, 0x03, 0x01)).unwrap();
        events.on_cis_request(&request);

        // The test module can look inside the queue, so no executor is needed.
        let queued = events.requests.try_receive().unwrap();
        assert_eq!(queued.acl_handle.0, 0x0040);
        assert_eq!(queued.cis_handle.0, 0x0002);
        assert_eq!(queued.cig_id, 0x03);
        assert_eq!(queued.cis_id, 0x01);
    }

    #[test]
    fn a_full_queue_drops_rather_than_blocking() {
        let events: IsoEvents<NoopRawMutex, 1, 1> = IsoEvents::new();

        for id in 0..5u8 {
            let request = LeCisRequest::from_hci_bytes_complete(&cis_request_bytes(1, 2, id, id)).unwrap();
            events.on_cis_request(&request);
        }

        // Only the first fits; the rest are dropped rather than stalling.
        let queued = events.requests.try_receive().unwrap();
        assert_eq!(queued.cig_id, 0);
        assert!(events.requests.try_receive().is_err());
    }
}
