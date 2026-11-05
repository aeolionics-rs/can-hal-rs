//! Confirmed transmission: frames handed to a controller only when it has room for them,
//! with a report when the frame has been acknowledged on the bus.
//! Confirmations are interleaved with received frames in the order they were placed on the bus.
//!
//! [`Transmit`](crate::Transmit) returns once a backend has accepted a frame but before it is
//! actually transmitted. Transmission may be delayed due to buffering, a busy bus, or lost
//! arbitration. The confirmation informs the caller not only that the frame was successfully
//! transmitted but also the timestamp when that occurred. Some higher level protocols require this
//! to enforce quiet periods or pacing.
//!
//! A channel that confirms its transmissions implements [`ConfirmedTransmit`] and
//! [`ReceiveEvents`] for its frame type: [`CanFrame`] on a classic channel, [`Frame`] on an FD
//! channel. Each transmission handed over is assigned a [`TransmitId`], and exactly one
//! [`Event::Transmitted`] with that id will follow.
//!
//! # Plain and confirmed channels
//!
//! A confirmed channel implements neither [`Transmit`](crate::Transmit) nor
//! [`Receive`](crate::Receive) (nor their FD and async variants). A frame sent with `transmit`
//! would have no [`TransmitId`] and no report, and would fall outside the order that
//! [`ConfirmedTransmit::try_transmit`] promises; a frame taken by `receive` would be missing
//! from the [`Event`]s. Traits that don't move frames, such as
//! [`Filterable`](crate::Filterable) and [`BusStatus`](crate::BusStatus), may be implemented by
//! both.
//!
//! A backend therefore offers confirmation as a channel of its own, chosen in its builder: for
//! example `driver.channel(0).classic(500_000)?.confirmed().connect()?`, giving a channel that
//! implements [`ConfirmedTransmit`] and [`ReceiveEvents`] where `.connect()` alone gives one
//! that implements `Transmit` and `Receive`. Confirmation usually needs the controller set up
//! for it in any case: SocketCAN's `CAN_RAW_RECV_OWN_MSGS`, or KVASER CANlib's
//! `canIOCTL_SET_TXACK`.
//!
//! [`CanFrame`]: crate::CanFrame
//! [`Frame`]: crate::Frame

use core::fmt;
use core::time::Duration;

use crate::bus::BusState;
use crate::error::CanError;
use crate::frame::Timestamped;

/// Identifies a transmission handed over by [`ConfirmedTransmit::try_transmit`], to
/// match it with its [`Event::Transmitted`].
///
/// Assigned by the backend, and unique among the transmissions outstanding at one time; a backend
/// may reuse an id once its transmission has been reported.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TransmitId(pub u32);

/// The outcome of a transmission attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome<T> {
    /// The frame was transmitted and acknowledged; the timestamp is when, on the same clock as
    /// the frames received.
    Sent(T),
    /// The frame was withdrawn at the caller's request (see [`ConfirmedTransmit::abort`]); the
    /// [`Abort`] says whether that is certain.
    Aborted(Abort),
    /// The frame was not confirmed sent, and won't be tried again; the [`Failure`] says why.
    Failed(Failure),
}

/// How certain a backend is that a frame reported [`Outcome::Aborted`] was not sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Abort {
    /// The frame was definitely withdrawn before its transmission started and was not sent.
    Confirmed,
    /// An abort was attempted, but the frame might have been transmitted before it took effect.
    Unconfirmed,
}

/// Why a frame was reported [`Outcome::Failed`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Failure {
    /// The controller gave up on the frame without transmitting it: in single-shot mode after a
    /// failed attempt, or on going bus-off.
    GaveUp,
    /// The report of how the frame's transmission ended was lost, for example to a full event
    /// queue. The frame may have been transmitted.
    Unconfirmed,
}

/// Events reported by a channel.
#[derive(Debug, Clone)]
pub enum Event<F, T> {
    /// A frame received from the bus.
    Received(Timestamped<F, T>),
    /// A transmission initiated with [`ConfirmedTransmit::try_transmit`] has finished.
    Transmitted {
        /// The id the transmission was given.
        id: TransmitId,
        /// What became of it.
        outcome: Outcome<T>,
    },
    /// The controller's bus state changed.
    BusStateChanged(BusState),
}

/// An enumeration of possible error outcome from [`ConfirmedTransmit::try_transmit`].
///
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TryTransmitError<E> {
    /// [`capacity`](ConfirmedTransmit::capacity) transmissions are outstanding.
    /// There will be room once one of them is reported by an [`Event::Transmitted`].
    Full,
    /// The backend reported an error.
    Backend(E),
}

impl<E> From<E> for TryTransmitError<E> {
    fn from(error: E) -> Self {
        Self::Backend(error)
    }
}

impl<E: fmt::Display> fmt::Display for TryTransmitError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Full => f.write_str("no room for another transmission"),
            Self::Backend(error) => error.fmt(f),
        }
    }
}

impl<E: core::error::Error> core::error::Error for TryTransmitError<E> {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Full => None,
            Self::Backend(error) => error.source(),
        }
    }
}

/// Why [`ConfirmedTransmit::abort`] could not make its request.
///
/// That an abort came too late, because the transmission had started, is not an error: it is
/// reported by the transmission's [`Event::Transmitted`], as [`Outcome::Sent`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AbortError<E> {
    /// This channel can't withdraw frames: every frame handed over will be transmitted.
    Unsupported,
    /// The backend reported an error.
    Backend(E),
}

impl<E> From<E> for AbortError<E> {
    fn from(error: E) -> Self {
        Self::Backend(error)
    }
}

impl<E: fmt::Display> fmt::Display for AbortError<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unsupported => f.write_str("this channel can't withdraw frames"),
            Self::Backend(error) => error.fmt(f),
        }
    }
}

impl<E: core::error::Error> core::error::Error for AbortError<E> {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Unsupported => None,
            Self::Backend(error) => error.source(),
        }
    }
}

/// Transmit frames of type `F` with confirmation: [`CanFrame`](crate::CanFrame) on a classic
/// channel, [`Frame`](crate::Frame) on an FD channel.
///
/// There is no blocking variant: room is made by the [`Event::Transmitted`]s, which the caller
/// receives, so waiting for room in here could wait forever.
pub trait ConfirmedTransmit<F> {
    type Error: CanError;

    /// How many transmissions may be outstanding at once: awaiting transmission until reported by
    /// an [`Event::Transmitted`]. At least 1.
    fn capacity(&self) -> usize;

    /// Attempts to enqueue `frame` for transmission, returning a transmission identifier.
    ///
    /// Never blocks: if there is insufficient [`capacity`](Self::capacity) it returns
    /// [`TryTransmitError::Full`].
    ///
    /// # Order
    ///
    /// Frames are placed on the bus in the order they are handed over. A frame that is aborted
    /// or fails is skipped, and the rest keep their order.
    ///
    /// Each transmission is reported by exactly one [`Event::Transmitted`], and these reports
    /// also come in the order the frames were handed over, whatever their outcome: each one is
    /// for the oldest transmission not yet reported. An [`Outcome::Aborted`] or
    /// [`Outcome::Failed`] therefore comes after the reports of all frames handed over before
    /// it, even if the abort or failure happened first, and before the reports of all frames
    /// handed over after it. For example, with frames A, B and C handed over in that order and
    /// B aborted while A is still on its way, the reports are A `Sent`, B `Aborted`, C `Sent`.
    ///
    /// # Implementing
    ///
    /// - A controller with several transmit buffers that it arbitrates among by identifier
    ///   would reorder frames. Use its FIFO mode if it has one; otherwise keep one frame in the
    ///   hardware at a time, queueing any others in software.
    /// - A frame retried in software after a failure must go before any frame handed over after
    ///   it: hold the rest back while retrying, or report it [`Failure::GaveUp`] instead.
    /// - The report of an abort or failure that happens while earlier frames are still
    ///   outstanding is held back until they have been reported.
    fn try_transmit(&mut self, frame: &F) -> Result<TransmitId, TryTransmitError<Self::Error>>;

    /// Requests that transmission `id` be aborted if its transmission hasn't started. Its
    /// [`Event::Transmitted`] says which happened:
    ///
    /// - [`Outcome::Aborted`] with [`Abort::Confirmed`] if the frame was withdrawn before its
    ///   transmission started;
    /// - [`Outcome::Aborted`] with [`Abort::Unconfirmed`] if the controller can't say whether
    ///   the frame was transmitted first;
    /// - [`Outcome::Sent`] if its transmission had started.
    ///
    /// The report comes in its place in the order of handing over (see
    /// [`try_transmit`](Self::try_transmit)), not when the abort takes effect.
    ///
    /// `Ok` means the request was accepted, not that the frame was withdrawn: whether it was is
    /// only known from its report. An id already reported, or unknown, is ignored with `Ok`, as
    /// its report may be waiting unread, and a backend that reuses ids can't tell the two apart.
    ///
    /// The default returns [`AbortError::Unsupported`]. A backend that can withdraw frames, if
    /// only those still in a queue of its own, should override it.
    fn abort(&mut self, id: TransmitId) -> Result<(), AbortError<Self::Error>> {
        let _ = id;
        Err(AbortError::Unsupported)
    }
}

/// Receive a channel's [`Event`]s: frames of type `F` received, transmission outcomes and
/// changes in its bus state, in the order they happened.
///
/// Implemented with [`ConfirmedTransmit`] by a confirmed channel: see [the module](self) for why
/// such a channel offers neither [`Receive`](crate::Receive) nor [`Transmit`](crate::Transmit).
pub trait ReceiveEvents<F> {
    type Error: CanError;
    /// The timestamp type used by this backend, for both frames received and transmissions sent.
    type Timestamp: Clone;

    /// Blocks until an event is available.
    fn receive_event(&mut self) -> Result<Event<F, Self::Timestamp>, Self::Error>;

    /// Returns immediately with `Ok(None)` if no event is available.
    fn try_receive_event(&mut self) -> Result<Option<Event<F, Self::Timestamp>>, Self::Error>;

    /// Blocks until an event is available or the timeout expires.
    /// Returns `Ok(None)` on timeout.
    fn receive_event_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Option<Event<F, Self::Timestamp>>, Self::Error>;
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::frame::CanFrame;
    use crate::id::CanId;
    use std::collections::VecDeque;
    use std::fmt;
    use std::vec::Vec;

    #[derive(Debug)]
    struct MockError;

    impl fmt::Display for MockError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.write_str("mock error")
        }
    }

    impl std::error::Error for MockError {}

    /// A controller with `capacity` transmit slots, which sends what it is given when told to,
    /// at a time counted in microseconds.
    struct MockController {
        capacity: usize,
        next_id: u32,
        /// Handed over and not yet reported, oldest first: id, frame, and whether it has been
        /// withdrawn.
        pending: VecDeque<(TransmitId, CanFrame, bool)>,
        events: VecDeque<Event<CanFrame, u64>>,
        sent: Vec<CanFrame>,
        now: u64,
    }

    impl MockController {
        const fn new(capacity: usize) -> Self {
            Self {
                capacity,
                next_id: 0,
                pending: VecDeque::new(),
                events: VecDeque::new(),
                sent: Vec::new(),
                now: 0,
            }
        }

        /// The next frame not withdrawn goes on the bus.
        fn send_one(&mut self) {
            self.report_withdrawn();
            if let Some((id, frame, _)) = self.pending.pop_front() {
                self.now += 130;
                self.sent.push(frame);
                self.events.push_back(Event::Transmitted {
                    id,
                    outcome: Outcome::Sent(self.now),
                });
            }
            self.report_withdrawn();
        }

        /// Reports the withdrawn frames that are now the oldest outstanding: a report waits for
        /// those of the frames handed over before it.
        fn report_withdrawn(&mut self) {
            while let Some(&(id, _, true)) = self.pending.front() {
                self.pending.pop_front();
                self.events.push_back(Event::Transmitted {
                    id,
                    outcome: Outcome::Aborted(Abort::Confirmed),
                });
            }
        }

        /// A frame arrives from the bus.
        fn hear(&mut self, frame: CanFrame) {
            self.now += 130;
            self.events
                .push_back(Event::Received(Timestamped::new(frame, self.now)));
        }
    }

    impl ConfirmedTransmit<CanFrame> for MockController {
        type Error = MockError;

        fn capacity(&self) -> usize {
            self.capacity
        }

        fn try_transmit(
            &mut self,
            frame: &CanFrame,
        ) -> Result<TransmitId, TryTransmitError<MockError>> {
            let outstanding = self.pending.len()
                + self
                    .events
                    .iter()
                    .filter(|e| matches!(e, Event::Transmitted { .. }))
                    .count();
            if outstanding >= self.capacity {
                return Err(TryTransmitError::Full);
            }
            let id = TransmitId(self.next_id);
            self.next_id += 1;
            self.pending.push_back((id, frame.clone(), false));
            Ok(id)
        }

        fn abort(&mut self, id: TransmitId) -> Result<(), AbortError<MockError>> {
            if let Some(pending) = self.pending.iter_mut().find(|(pending, ..)| *pending == id) {
                pending.2 = true;
            }
            self.report_withdrawn();
            Ok(())
        }
    }

    impl ReceiveEvents<CanFrame> for MockController {
        type Error = MockError;
        type Timestamp = u64;

        fn receive_event(&mut self) -> Result<Event<CanFrame, u64>, MockError> {
            self.events.pop_front().ok_or(MockError)
        }

        fn try_receive_event(&mut self) -> Result<Option<Event<CanFrame, u64>>, MockError> {
            Ok(self.events.pop_front())
        }

        fn receive_event_timeout(
            &mut self,
            _timeout: Duration,
        ) -> Result<Option<Event<CanFrame, u64>>, MockError> {
            self.try_receive_event()
        }
    }

    fn frame(id: u32, data: &[u8]) -> CanFrame {
        CanFrame::new(CanId::new_extended(id).unwrap(), data).unwrap()
    }

    #[test]
    fn transmissions_are_refused_without_room() {
        let mut ch = MockController::new(1);
        assert_eq!(ch.capacity(), 1);
        let first = ch.try_transmit(&frame(0x100, &[1])).unwrap();
        assert!(matches!(
            ch.try_transmit(&frame(0x100, &[2])),
            Err(TryTransmitError::Full)
        ));

        ch.send_one();
        // Room is made once the confirmation has been received, not before.
        assert!(matches!(
            ch.try_transmit(&frame(0x100, &[2])),
            Err(TryTransmitError::Full)
        ));
        let event = ch.try_receive_event().unwrap().unwrap();
        assert!(matches!(
            event,
            Event::Transmitted { id, outcome: Outcome::Sent(130) } if id == first
        ));
        assert!(ch.try_transmit(&frame(0x100, &[2])).is_ok());
    }

    #[test]
    fn events_come_in_the_order_they_happened() {
        let mut ch = MockController::new(2);
        let id = ch.try_transmit(&frame(0x100, &[1])).unwrap();
        ch.hear(frame(0x200, &[9]));
        ch.send_one();

        match ch.receive_event().unwrap() {
            Event::Received(received) => {
                assert_eq!(received.frame(), &frame(0x200, &[9]));
                assert_eq!(*received.timestamp(), 130);
            }
            other => panic!("expected the frame received first, got {other:?}"),
        }
        match ch.receive_event().unwrap() {
            Event::Transmitted { id: done, outcome } => {
                assert_eq!(done, id);
                // On the same clock as the frame received, and after it.
                assert_eq!(outcome, Outcome::Sent(260));
            }
            other => panic!("expected the transmission, got {other:?}"),
        }
        assert!(ch.try_receive_event().unwrap().is_none());
    }

    /// The transmissions' reports received so far: id and outcome.
    fn reports(ch: &mut MockController) -> Vec<(TransmitId, Outcome<u64>)> {
        core::iter::from_fn(|| ch.try_receive_event().unwrap())
            .filter_map(|event| match event {
                Event::Transmitted { id, outcome } => Some((id, outcome)),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn an_abort_is_reported_in_its_place_in_the_order() {
        // The example in `try_transmit`'s docs: A, B and C handed over, and B aborted while A
        // is still on its way.
        let mut ch = MockController::new(3);
        let a = ch.try_transmit(&frame(0x100, &[1])).unwrap();
        let b = ch.try_transmit(&frame(0x100, &[2])).unwrap();
        let c = ch.try_transmit(&frame(0x100, &[3])).unwrap();
        ch.abort(b).unwrap();
        // B's report waits for A's.
        assert_eq!(reports(&mut ch), []);

        ch.send_one();
        ch.send_one();
        assert_eq!(
            reports(&mut ch),
            [
                (a, Outcome::Sent(130)),
                (b, Outcome::Aborted(Abort::Confirmed)),
                (c, Outcome::Sent(260))
            ]
        );
        assert_eq!(ch.sent, [frame(0x100, &[1]), frame(0x100, &[3])]);
    }

    #[test]
    fn an_abort_of_the_oldest_is_reported_at_once() {
        let mut ch = MockController::new(2);
        let a = ch.try_transmit(&frame(0x100, &[1])).unwrap();
        let b = ch.try_transmit(&frame(0x100, &[2])).unwrap();
        ch.abort(a).unwrap();
        assert_eq!(reports(&mut ch), [(a, Outcome::Aborted(Abort::Confirmed))]);

        ch.send_one();
        assert_eq!(reports(&mut ch), [(b, Outcome::Sent(130))]);
        // An id already reported is ignored.
        ch.abort(a).unwrap();
        assert_eq!(reports(&mut ch), []);
    }

    #[test]
    fn the_default_abort_is_unsupported() {
        /// A backend that can't withdraw frames: it keeps `abort`'s default.
        struct Unabortable(MockController);

        impl ConfirmedTransmit<CanFrame> for Unabortable {
            type Error = MockError;

            fn capacity(&self) -> usize {
                self.0.capacity()
            }

            fn try_transmit(
                &mut self,
                frame: &CanFrame,
            ) -> Result<TransmitId, TryTransmitError<MockError>> {
                self.0.try_transmit(frame)
            }
        }

        let mut ch = Unabortable(MockController::new(1));
        let a = ch.try_transmit(&frame(0x100, &[1])).unwrap();
        assert!(matches!(ch.abort(a), Err(AbortError::Unsupported)));
        // The frame goes all the same.
        ch.0.send_one();
        assert_eq!(reports(&mut ch.0), [(a, Outcome::Sent(130))]);
    }

    #[test]
    fn the_errors_are_can_errors() {
        use std::error::Error as _;
        use std::string::ToString;

        let full: TryTransmitError<MockError> = TryTransmitError::Full;
        assert_eq!(full.to_string(), "no room for another transmission");
        assert!(full.source().is_none());

        // A backend's error converts, so `?` works inside an implementation, and shows as itself.
        let backend: TryTransmitError<MockError> = MockError.into();
        assert!(matches!(backend, TryTransmitError::Backend(MockError)));
        assert_eq!(backend.to_string(), "mock error");

        // It is a CanError itself, so a caller can pass it on with `?`.
        fn is_can_error<E: CanError>(_: &E) {}
        is_can_error(&full);

        // And so is an abort's.
        let unsupported: AbortError<MockError> = AbortError::Unsupported;
        assert_eq!(
            unsupported.to_string(),
            "this channel can't withdraw frames"
        );
        assert!(unsupported.source().is_none());
        let backend: AbortError<MockError> = MockError.into();
        assert_eq!(backend.to_string(), "mock error");
        is_can_error(&unsupported);
    }

    /// A caller generic over the traits: sends each frame when there is room, and returns when
    /// each was on the bus.
    fn send_all<C, E>(channel: &mut C, frames: &[CanFrame]) -> Result<Vec<u64>, E>
    where
        C: ConfirmedTransmit<CanFrame, Error = E>
            + ReceiveEvents<CanFrame, Error = E, Timestamp = u64>,
        E: CanError,
    {
        let mut waiting = frames.iter();
        let mut next = waiting.next();
        let mut times = Vec::new();
        while times.len() < frames.len() {
            while let Some(frame) = next {
                match channel.try_transmit(frame) {
                    Ok(_) => next = waiting.next(),
                    Err(TryTransmitError::Full) => break,
                    Err(TryTransmitError::Backend(error)) => return Err(error),
                }
            }
            if let Event::Transmitted {
                outcome: Outcome::Sent(at),
                ..
            } = channel.receive_event()?
            {
                times.push(at);
            }
        }
        Ok(times)
    }

    #[test]
    fn generic_sender_over_the_traits() {
        struct Sending(MockController);

        impl ConfirmedTransmit<CanFrame> for Sending {
            type Error = MockError;

            fn capacity(&self) -> usize {
                self.0.capacity()
            }

            fn try_transmit(
                &mut self,
                frame: &CanFrame,
            ) -> Result<TransmitId, TryTransmitError<MockError>> {
                self.0.try_transmit(frame)
            }
        }

        impl ReceiveEvents<CanFrame> for Sending {
            type Error = MockError;
            type Timestamp = u64;

            fn receive_event(&mut self) -> Result<Event<CanFrame, u64>, MockError> {
                // The bus takes a frame whenever the sender waits for word of one.
                self.0.send_one();
                self.0.receive_event()
            }

            fn try_receive_event(&mut self) -> Result<Option<Event<CanFrame, u64>>, MockError> {
                self.0.try_receive_event()
            }

            fn receive_event_timeout(
                &mut self,
                timeout: Duration,
            ) -> Result<Option<Event<CanFrame, u64>>, MockError> {
                self.0.receive_event_timeout(timeout)
            }
        }

        let mut ch = Sending(MockController::new(1));
        let frames = [frame(0x100, &[1]), frame(0x100, &[2]), frame(0x100, &[3])];
        assert_eq!(send_all(&mut ch, &frames).unwrap(), [130, 260, 390]);
        assert_eq!(ch.0.sent, frames);
    }
}
