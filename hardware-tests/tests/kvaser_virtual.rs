//! Kvaser CANlib software-in-the-loop (SITL) tests over virtual channels.
//!
//! These tests exercise the `can-hal-kvaser` backend end to end, from loading
//! CANlib to reading frames back, without any physical hardware. CANlib's
//! virtual driver gives channels that share one virtual bus, so a frame
//! written on one is received on the other.
//!
//! ## Requirements
//!
//! CANlib and its virtual driver: on Linux, build linuxcan and install
//! `common`, `virtualcan` and `canlib` (`sudo make -C <dir> install`, and
//! `load` for `virtualcan`); on Windows, the CANlib driver installs virtual
//! channels by default.
//!
//! The two channels are CANlib channels 0 and 1 unless
//! `CAN_HAL_KVASER_VIRTUAL_CHANNELS` names others (`"2,3"`), as it must when
//! a physical Kvaser interface comes first.
//!
//! When CANlib or the channels are missing each test skips cleanly. Set
//! `CAN_HAL_REQUIRE_KVASER_VIRTUAL=1` to make that a hard failure instead.
//!
//! The suite is behind the off-by-default `kvaser-virtual` cargo feature:
//!
//! ```bash
//! CAN_HAL_REQUIRE_KVASER_VIRTUAL=1 cargo test -p hardware-tests \
//!   --features kvaser-virtual --test kvaser_virtual -- --test-threads=1
//! ```
#![cfg(feature = "kvaser-virtual")]

use std::time::Duration;

use can_hal::channel::{Receive, Transmit};
use can_hal::frame::CanFrame;
use can_hal::CanId;
use can_hal_kvaser::{Classic, KvaserChannel, KvaserChannelBuilder, KvaserDriver};

/// Environment variable naming the two virtual channels, as `"<tx>,<rx>"`.
const CHANNELS_ENV: &str = "CAN_HAL_KVASER_VIRTUAL_CHANNELS";

/// Environment variable that, when set to a non-empty value, turns missing
/// CANlib or virtual channels into a hard failure instead of a skip.
const REQUIRE_ENV: &str = "CAN_HAL_REQUIRE_KVASER_VIRTUAL";

const NOMINAL_HZ: u32 = 500_000;
const TIMEOUT: Duration = Duration::from_secs(1);

fn required() -> bool {
    std::env::var_os(REQUIRE_ENV).is_some_and(|v| !v.is_empty())
}

/// The transmit and receive channel indices.
fn channels() -> (u32, u32) {
    let Ok(value) = std::env::var(CHANNELS_ENV) else {
        return (0, 1);
    };
    let parsed = value
        .split_once(',')
        .and_then(|(tx, rx)| Some((tx.trim().parse().ok()?, rx.trim().parse().ok()?)));
    parsed.unwrap_or_else(|| panic!("{CHANNELS_ENV} must be \"<tx>,<rx>\", not {value:?}"))
}

/// Skip, or fail when [`required`], because `what` failed.
fn skip<T>(what: &str, error: &dyn std::fmt::Display) -> Option<T> {
    assert!(!required(), "{REQUIRE_ENV} is set but {what}: {error}");
    eprintln!("skipping Kvaser virtual test: {what}: {error}");
    None
}

fn driver() -> Option<KvaserDriver> {
    match KvaserDriver::new() {
        Ok(driver) => Some(driver),
        Err(e) => skip("CANlib could not be loaded", &e),
    }
}

fn open_classic(driver: &KvaserDriver, index: u32) -> Option<KvaserChannel<Classic>> {
    let channel = driver
        .channel(index)
        .accept_virtual(true)
        .classic(NOMINAL_HZ)
        .and_then(KvaserChannelBuilder::<Classic>::connect);
    match channel {
        Ok(channel) => Some(channel),
        Err(e) => skip(&format!("channel {index} could not be opened"), &e),
    }
}

fn classic_pair() -> Option<(KvaserChannel<Classic>, KvaserChannel<Classic>)> {
    let driver = driver()?;
    let (tx, rx) = channels();
    Some((open_classic(&driver, tx)?, open_classic(&driver, rx)?))
}

fn receive(rx: &mut KvaserChannel<Classic>) -> CanFrame {
    rx.receive_timeout(TIMEOUT)
        .expect("receive error")
        .expect("receive timeout")
        .into_frame()
}

#[test]
fn test_kvaser_virtual_classic_every_dlc() {
    let Some((mut tx, mut rx)) = classic_pair() else {
        return;
    };
    let id = CanId::new_standard(0x123).unwrap();
    let data = [1, 2, 3, 4, 5, 6, 7, 8];
    for dlc in 0..=15 {
        let frame = CanFrame::new_with_dlc(id, &data[..dlc.min(8)], dlc).unwrap();
        tx.transmit(&frame).expect("transmit");
        assert_eq!(receive(&mut rx), frame, "DLC {dlc}");
    }
}

#[test]
fn test_kvaser_virtual_remote_every_dlc() {
    let Some((mut tx, mut rx)) = classic_pair() else {
        return;
    };
    let id = CanId::new_extended(0x18EA_FFFE).unwrap();
    for dlc in 0..=15 {
        let frame = CanFrame::new_remote(id, dlc).unwrap();
        tx.transmit(&frame).expect("transmit");
        assert_eq!(receive(&mut rx), frame, "remote DLC {dlc}");
    }
}
