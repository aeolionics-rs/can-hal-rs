use can_hal::{CanFdFrame, CanFrame, CanId, Frame};

use crate::error::KvaserError;
use crate::ffi::{
    CAN_MSG_BRS, CAN_MSG_ERROR_FRAME, CAN_MSG_ESI, CAN_MSG_EXT, CAN_MSG_FDF, CAN_MSG_RTR,
    CAN_MSG_STD,
};

/// Convert a `CanId` to a raw CANlib id and message flag.
#[allow(clippy::cast_lossless)] // From is not const-stable
pub const fn to_canlib_id(id: CanId) -> (u32, u32) {
    match id {
        CanId::Standard(v) => (v as u32, CAN_MSG_STD),
        CanId::Extended(v) => (v, CAN_MSG_EXT),
    }
}

/// Reconstruct a `CanId` from a raw CANlib id and message flags.
pub fn from_canlib_id(raw_id: u32, flags: u32) -> Result<CanId, KvaserError> {
    if flags & CAN_MSG_EXT != 0 {
        CanId::new_extended(raw_id).ok_or_else(|| {
            KvaserError::InvalidFrame(format!("extended ID out of range: {raw_id:#x}"))
        })
    } else {
        if raw_id > 0x7FF {
            return Err(KvaserError::InvalidFrame(format!(
                "standard ID out of range: {raw_id:#x}"
            )));
        }
        #[allow(clippy::cast_possible_truncation)] // validated above: raw_id <= 0x7FF
        CanId::new_standard(raw_id as u16).ok_or_else(|| {
            KvaserError::InvalidFrame(format!("standard ID out of range: {raw_id:#x}"))
        })
    }
}

/// Convert a raw CANlib receive buffer into a `Frame`: a classic remote frame
/// for an RTR one.
///
/// Returns `Ok(None)` for error frames, which are not represented in the
/// can-hal frame model.
pub fn from_canlib_frame(
    raw_id: u32,
    data: &[u8; 64],
    dlc: u32,
    flags: u32,
) -> Result<Option<Frame>, KvaserError> {
    // Skip frames we don't represent.
    if flags & CAN_MSG_ERROR_FRAME != 0 {
        return Ok(None);
    }

    let id = from_canlib_id(raw_id, flags)?;

    if flags & CAN_MSG_RTR != 0 {
        // A remote frame, which only a classic frame can be: dlc is the length it asks for.
        let frame = CanFrame::new_remote(id, (dlc as usize).min(15))
            .ok_or_else(|| KvaserError::InvalidFrame(format!("invalid remote DLC: {dlc}")))?;
        Ok(Some(Frame::Can(frame)))
    } else if flags & CAN_MSG_FDF != 0 {
        // CAN FD frame - dlc is already the byte count from CANlib.
        // Clamp to 64 to prevent panics on out-of-range values from FFI.
        let len = (dlc as usize).min(64);
        let brs = flags & CAN_MSG_BRS != 0;
        let esi = flags & CAN_MSG_ESI != 0;
        let frame = CanFdFrame::new(id, &data[..len], brs, esi)
            .ok_or_else(|| KvaserError::InvalidFrame(format!("invalid FD DLC: {dlc}")))?;
        Ok(Some(Frame::Fd(frame)))
    } else {
        // Classic CAN frame: a channel opened with canOPEN_ACCEPT_LARGE_DLC
        // reports a DLC of 9-15 as it is, with 8 bytes of data.
        let dlc = (dlc as usize).min(15);
        let len = dlc.min(8);
        let frame = CanFrame::new_with_dlc(id, &data[..len], dlc)
            .ok_or_else(|| KvaserError::InvalidFrame(format!("invalid classic CAN DLC: {dlc}")))?;
        Ok(Some(Frame::Can(frame)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_standard_id() {
        let id = CanId::new_standard(0x123).unwrap();
        let (raw, flags) = to_canlib_id(id);
        assert_eq!(raw, 0x123);
        assert_eq!(flags, CAN_MSG_STD);
        assert_eq!(from_canlib_id(raw, flags).unwrap(), id);
    }

    #[test]
    fn round_trip_extended_id() {
        let id = CanId::new_extended(0x1234_5678).unwrap();
        let (raw, flags) = to_canlib_id(id);
        assert_eq!(raw, 0x1234_5678);
        assert_eq!(flags, CAN_MSG_EXT);
        assert_eq!(from_canlib_id(raw, flags).unwrap(), id);
    }

    #[test]
    fn classic_frame_from_canlib() {
        let id = CanId::new_standard(0x100).unwrap();
        let (raw_id, flags) = to_canlib_id(id);
        let mut data = [0u8; 64];
        data[..3].copy_from_slice(&[0x01, 0x02, 0x03]);

        let frame = from_canlib_frame(raw_id, &data, 3, flags).unwrap().unwrap();
        match frame {
            Frame::Can(f) => {
                assert_eq!(f.id(), id);
                assert_eq!(f.data(), &[0x01, 0x02, 0x03]);
            }
            Frame::Fd(_) => panic!("expected classic frame"),
        }
    }

    #[test]
    fn fd_frame_from_canlib() {
        let id = CanId::new_extended(0x200).unwrap();
        let (raw_id, mut flags) = to_canlib_id(id);
        flags |= CAN_MSG_FDF | CAN_MSG_BRS;
        let mut data = [0u8; 64];
        data[..12].fill(0xAB);

        let frame = from_canlib_frame(raw_id, &data, 12, flags)
            .unwrap()
            .unwrap();
        match frame {
            Frame::Fd(f) => {
                assert_eq!(f.len(), 12);
                assert!(f.brs());
                assert!(!f.esi());
            }
            Frame::Can(_) => panic!("expected FD frame"),
        }
    }

    #[test]
    fn rtr_frame_is_a_remote_frame() {
        // CAN_MSG_RTR = 0x0001; combined with CAN_MSG_STD = 0x0002.
        let data = [0u8; 64];
        let Some(Frame::Can(frame)) =
            from_canlib_frame(0x100, &data, 4, CAN_MSG_STD | CAN_MSG_RTR).unwrap()
        else {
            panic!("an RTR frame wasn't a classic frame");
        };
        assert!(frame.is_remote_frame());
        assert_eq!(frame.id(), CanId::new_standard(0x100).unwrap());
        assert_eq!(frame.dlc(), 4);
        assert_eq!(frame.len(), 0);
    }

    #[test]
    fn large_dlc_is_kept() {
        let mut data = [0u8; 64];
        data[..8].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let Some(Frame::Can(frame)) = from_canlib_frame(0x100, &data, 12, CAN_MSG_STD).unwrap()
        else {
            panic!("a DLC 12 frame wasn't a classic frame");
        };
        assert_eq!(frame.dlc(), 12);
        assert_eq!(frame.data(), &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn error_frame_returns_none() {
        let data = [0u8; 64];
        assert!(
            from_canlib_frame(0x100, &data, 0, CAN_MSG_STD | CAN_MSG_ERROR_FRAME)
                .unwrap()
                .is_none()
        );
    }
}
