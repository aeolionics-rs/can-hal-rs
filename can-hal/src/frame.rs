use crate::id::CanId;

const CAN_MAX_LEN: usize = 8;
const CAN_MAX_DLC: usize = 15;

/// The data length a classic DLC stands for: DLCs 9 to 15 mean 8 bytes, as
/// ISO 11898-1 has it.
const fn dlc_to_len(dlc: usize) -> usize {
    if dlc > CAN_MAX_LEN {
        CAN_MAX_LEN
    } else {
        dlc
    }
}

/// A classic CAN 2.0 frame: a data frame of up to 8 data bytes, or a remote
/// frame asking for up to 8.
///
/// The frame keeps its DLC (0--15) as well as its data. ISO 11898-1 has DLCs
/// 9 to 15 mean 8 data bytes, and a protocol above CAN may give the code
/// itself a meaning, so a frame with 8 bytes and a DLC of 12 is a different
/// frame from one with 8 bytes and a DLC of 8.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    id: CanId,
    /// The on-wire DLC (0--15).
    dlc: u8,
    data: [u8; 8],
    remote: bool,
}

impl CanFrame {
    /// Create a new classic CAN data frame, with a DLC of the data's
    /// length.
    ///
    /// Returns `None` if `data` is longer than 8 bytes.
    #[must_use]
    pub fn new(id: CanId, data: &[u8]) -> Option<Self> {
        Self::new_with_dlc(id, data, data.len())
    }

    /// Create a new classic CAN data frame with the given DLC, which may be
    /// 9 to 15 for 8 bytes of data.
    ///
    /// Returns `None` if `dlc` is over 15, or `data` is not the length it
    /// stands for: `dlc` bytes for a DLC of 0 to 8, and 8 bytes for 9 to 15.
    #[must_use]
    pub fn new_with_dlc(id: CanId, data: &[u8], dlc: usize) -> Option<Self> {
        if dlc > CAN_MAX_DLC || data.len() != dlc_to_len(dlc) {
            return None;
        }
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        #[allow(clippy::cast_possible_truncation)] // validated above: dlc <= 15
        Some(Self {
            id,
            dlc: dlc as u8,
            data: buf,
            remote: false,
        })
    }

    /// Create a new remote frame (RTR bit set), with the given DLC: asking
    /// for `dlc` bytes of data, or 8 for a DLC of 9 to 15.
    ///
    /// Returns `None` if `dlc` is over 15.
    #[must_use]
    pub const fn new_remote(id: CanId, dlc: usize) -> Option<Self> {
        if dlc > CAN_MAX_DLC {
            return None;
        }
        #[allow(clippy::cast_possible_truncation)] // validated above: dlc <= 15
        Some(Self {
            id,
            dlc: dlc as u8,
            data: [0; 8],
            remote: true,
        })
    }

    /// Returns the frame's CAN identifier.
    #[must_use]
    pub const fn id(&self) -> CanId {
        self.id
    }

    /// Returns `true` if this is a remote frame.
    #[must_use]
    pub const fn is_remote_frame(&self) -> bool {
        self.remote
    }

    /// Returns `true` if this is a data frame.
    #[must_use]
    pub const fn is_data_frame(&self) -> bool {
        !self.remote
    }

    /// Returns the data length in bytes (0--8).
    ///
    /// For a classic CAN 2.0 data frame, the data length and the on-wire DLC
    /// field are identical in the range 0--8, and DLCs 9--15 mean 8 bytes. A
    /// remote frame carries no data, so its length is 0; [`dlc`](Self::dlc)
    /// says how much it asks for.
    #[must_use]
    pub const fn len(&self) -> usize {
        if self.remote {
            0
        } else {
            dlc_to_len(self.dlc as usize)
        }
    }

    /// Returns `true` if the frame carries zero data bytes, as a remote frame
    /// always does.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the data length code (0--15). For a data frame it is the
    /// length of the data, or 9--15 for 8 bytes; a remote frame carries no
    /// data, yet its DLC is the length it asks for, and can be greater than
    /// 0.
    #[must_use]
    pub const fn dlc(&self) -> usize {
        self.dlc as usize
    }

    /// Returns the data payload, which is empty for a remote frame.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len()]
    }
}

/// A CAN FD frame (up to 64 data bytes).
///
/// The data length is stored as a byte count (0--64), **not** the 4-bit
/// on-wire DLC code. For DLC values 9--15 the CAN FD specification maps
/// them to 12, 16, 20, 24, 32, 48, and 64 bytes respectively; this struct
/// stores the decoded byte count directly; [`dlc`](Self::dlc) gives the code.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFdFrame {
    id: CanId,
    len: u8,
    data: [u8; 64],
    brs: bool,
    esi: bool,
}

impl CanFdFrame {
    /// Create a new CAN FD frame.
    ///
    /// Returns `None` if `data.len()` is not a valid FD data length
    /// (0, 1, ..., 8, 12, 16, 20, 24, 32, 48, or 64).
    #[must_use]
    pub fn new(id: CanId, data: &[u8], brs: bool, esi: bool) -> Option<Self> {
        if !matches!(data.len(), 0..=8 | 12 | 16 | 20 | 24 | 32 | 48 | 64) {
            return None;
        }
        let mut buf = [0u8; 64];
        buf[..data.len()].copy_from_slice(data);
        #[allow(clippy::cast_possible_truncation)] // validated above: data.len() <= 64
        Some(Self {
            id,
            len: data.len() as u8,
            data: buf,
            brs,
            esi,
        })
    }

    /// Returns the frame's CAN identifier.
    #[must_use]
    pub const fn id(&self) -> CanId {
        self.id
    }

    /// Returns the data length in bytes (not the 4-bit DLC code).
    ///
    /// The returned value is always one of the valid CAN FD data lengths:
    /// 0, 1, ..., 8, 12, 16, 20, 24, 32, 48, or 64.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// Returns `true` if the frame carries zero data bytes.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the data length code (0--15): the length for 0--8 bytes, and
    /// 9--15 for 12, 16, 20, 24, 32, 48 and 64 bytes.
    #[must_use]
    pub const fn dlc(&self) -> usize {
        match self.len {
            0..=8 => self.len as usize,
            12 => 9,
            16 => 10,
            20 => 11,
            24 => 12,
            32 => 13,
            48 => 14,
            // 64: the constructor allows no other length.
            _ => 15,
        }
    }

    /// Returns the data payload.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// Returns `true` if the Bit Rate Switch flag is set.
    #[must_use]
    pub const fn brs(&self) -> bool {
        self.brs
    }

    /// Returns `true` if the Error State Indicator flag is set.
    #[must_use]
    pub const fn esi(&self) -> bool {
        self.esi
    }
}

/// A frame of either type - used when receiving on an FD-capable bus.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    Can(CanFrame),
    Fd(CanFdFrame),
}

impl Frame {
    /// Returns the frame's CAN identifier regardless of frame type.
    #[must_use]
    pub const fn id(&self) -> CanId {
        match self {
            Self::Can(f) => f.id(),
            Self::Fd(f) => f.id(),
        }
    }

    /// Returns the data payload regardless of frame type.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        match self {
            Self::Can(f) => f.data(),
            Self::Fd(f) => f.data(),
        }
    }

    /// Returns the data length in bytes regardless of frame type.
    #[must_use]
    pub const fn len(&self) -> usize {
        match self {
            Self::Can(f) => f.len(),
            Self::Fd(f) => f.len(),
        }
    }

    /// Returns `true` if the frame carries zero data bytes.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        match self {
            Self::Can(f) => f.is_empty(),
            Self::Fd(f) => f.is_empty(),
        }
    }

    /// Returns the data length code (0--15) regardless of frame type.
    #[must_use]
    pub const fn dlc(&self) -> usize {
        match self {
            Self::Can(f) => f.dlc(),
            Self::Fd(f) => f.dlc(),
        }
    }

    /// Returns `true` if this is a remote frame, which only a classic frame
    /// can be.
    #[must_use]
    pub const fn is_remote_frame(&self) -> bool {
        match self {
            Self::Can(f) => f.is_remote_frame(),
            Self::Fd(_) => false,
        }
    }

    /// Returns `true` if this is a data frame, as every FD frame is.
    #[must_use]
    pub const fn is_data_frame(&self) -> bool {
        !self.is_remote_frame()
    }
}

/// A received frame paired with its timestamp.
///
/// The timestamp type `T` is chosen by the backend implementation.
/// On `std` platforms this is typically `std::time::Instant`; on `no_std`
/// targets it can be a hardware timer tick count or any `Clone` type.
#[derive(Debug, Clone)]
pub struct Timestamped<F, T> {
    frame: F,
    timestamp: T,
}

impl<F, T> Timestamped<F, T> {
    /// Create a new timestamped frame.
    #[must_use]
    pub const fn new(frame: F, timestamp: T) -> Self {
        Self { frame, timestamp }
    }

    /// Returns a reference to the inner frame.
    #[must_use]
    pub const fn frame(&self) -> &F {
        &self.frame
    }

    /// Returns a reference to the timestamp.
    #[must_use]
    pub const fn timestamp(&self) -> &T {
        &self.timestamp
    }

    /// Consumes self and returns the inner frame.
    #[must_use]
    pub fn into_frame(self) -> F {
        self.frame
    }
}
