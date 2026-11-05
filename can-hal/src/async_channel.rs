use core::future::Future;

use crate::confirm::Event;
use crate::error::CanError;
use crate::frame::{CanFdFrame, CanFrame, Frame, Timestamped};

/// Async transmit of classic CAN frames.
pub trait AsyncTransmit {
    type Error: CanError;

    /// Asynchronously send a classic CAN frame.
    fn transmit(
        &mut self,
        frame: &CanFrame,
    ) -> impl Future<Output = Result<(), Self::Error>> + Send;
}

/// Async receive of classic CAN frames.
pub trait AsyncReceive {
    type Error: CanError;
    /// The timestamp type used by this backend.
    type Timestamp: Clone + Send;

    /// Asynchronously receive a classic CAN frame.
    fn receive(
        &mut self,
    ) -> impl Future<Output = Result<Timestamped<CanFrame, Self::Timestamp>, Self::Error>> + Send;
}

/// Async transmit of CAN FD frames.
pub trait AsyncTransmitFd {
    type Error: CanError;

    /// Asynchronously send a CAN FD frame.
    fn transmit_fd(
        &mut self,
        frame: &CanFdFrame,
    ) -> impl Future<Output = Result<(), Self::Error>> + Send;
}

/// Async receive of any frame (classic or FD).
pub trait AsyncReceiveFd {
    type Error: CanError;
    /// The timestamp type used by this backend.
    type Timestamp: Clone + Send;

    /// Asynchronously receive any frame.
    fn receive_fd(
        &mut self,
    ) -> impl Future<Output = Result<Timestamped<Frame, Self::Timestamp>, Self::Error>> + Send;
}

/// Async receive of a channel's [`Event`]s: frames of type `F`, its transmissions' outcomes and
/// its bus state, in the order they happened (see [`ReceiveEvents`](crate::ReceiveEvents)).
pub trait AsyncReceiveEvents<F> {
    type Error: CanError;
    /// The timestamp type used by this backend, for both frames received and transmissions sent.
    type Timestamp: Clone + Send;

    /// Asynchronously receive the next event.
    fn receive_event(
        &mut self,
    ) -> impl Future<Output = Result<Event<F, Self::Timestamp>, Self::Error>> + Send;
}
