use crate::error::CanError;

/// The error state of a CAN bus controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BusState {
    /// Normal operation. Error counters are below 128.
    ErrorActive,
    /// Error counters have reached the warning threshold (128–255).
    /// The controller can still communicate but may be experiencing issues.
    ErrorPassive,
    /// The controller has gone off-bus due to excessive errors (counter > 255).
    /// No frames can be sent or received until recovery.
    BusOff,
}

/// Transmit and receive error counters from the CAN controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ErrorCounters {
    pub transmit: u8,
    pub receive: u8,
}

/// Query the health and error state of a CAN bus controller.
///
/// Not all backends support this - it is an optional trait.
pub trait BusStatus {
    type Error: CanError;

    /// Returns the current bus state of the controller.
    fn bus_state(&self) -> Result<BusState, Self::Error>;

    /// Returns the current transmit and receive error counters.
    fn error_counters(&self) -> Result<ErrorCounters, Self::Error>;
}

/// Start a controller's recovery from bus-off.
///
/// Not all backends support this - it is an optional trait.
///
/// Some controllers recover by themselves (SocketCAN's `restart-ms`, for instance).
pub trait Recover {
    type Error: CanError;

    /// Starts recovery from bus-off, and returns without waiting for it to finish.
    ///
    /// Recovery finishes once the controller has seen 128 occurrences of 11 recessive bits on
    /// the bus. To find out when that happens, poll [`BusStatus::bus_state`] until it is no
    /// longer [`BusState::BusOff`].
    fn recover(&mut self) -> Result<(), Self::Error>;
}

/// Running counts from a controller, since it was opened. Each is `None` where the backend
/// does not report it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct BusStatistics {
    /// Frames received.
    pub rx_frames: Option<u64>,
    /// Frames transmitted.
    pub tx_frames: Option<u64>,
    /// Frames received but lost before the caller took them: a full receive queue, or the
    /// controller's receive buffer overrunning.
    pub rx_dropped: Option<u64>,
    /// Of those, the ones lost to the controller's receive buffer overrunning.
    pub rx_overruns: Option<u64>,
    /// Transmissions that lost arbitration and were retried.
    pub arbitration_lost: Option<u64>,
    /// Bus errors seen (bit, stuff, form, CRC or acknowledgement errors).
    pub bus_errors: Option<u64>,
    /// Transmissions the controller gave up on.
    pub tx_failed: Option<u64>,
}

/// Query a controller's running counts.
///
/// Not all backends support this - it is an optional trait.
pub trait Statistics {
    type Error: CanError;

    /// The counts so far.
    fn statistics(&self) -> Result<BusStatistics, Self::Error>;
}
