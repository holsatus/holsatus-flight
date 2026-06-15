//! TX15 SC switch -> active altitude ceiling.
//!
//! Watches the RC channel stream from `rc_kill` and toggles the runtime
//! ceiling used by `alt_hold`:
//!
//!   SC=Low        -> 2.0 m   (indoor / bench)
//!   SC=Mid/High   -> 100.0 m (outdoor; hard ceiling for the data-collection
//!                             mission. Stays under the EU Open A3 legal cap
//!                             of 120 m with ample margin for baro drift /
//!                             EWMA lag)
//!
//! Default-safe: the ceiling boots at 2.0 m. The pilot must consciously
//! raise SC on the TX15 to unlock outdoor altitude. The TX15 issues a
//! startup warning whenever a non-Low switch is left raised, so a
//! lingering "outdoor" position is hard to miss between flights.
//!
//! SC is RC channel 8 (0-indexed 7) per the EdgeTX mix documented in
//! `rc_kill.rs`. The decode threshold reuses `rc_kill::decode`.

use core::sync::atomic::Ordering;

use embassy_time::Timer;

use common::signals::MAG_TRUST_OUTDOOR;

use crate::alt_hold;
use crate::log as ulog;
use crate::rc_kill::{decode, Pos, RC_CHANNELS};

pub const INDOOR_CEILING_M: f32 = 2.0;
pub const OUTDOOR_CEILING_M: f32 = 100.0;

const SC_IDX: usize = 7;

#[embassy_executor::task]
pub async fn ceiling_mode_task() -> ! {
    let mut rcv = RC_CHANNELS.receiver().unwrap();
    let mut last: Option<Pos> = None;

    loop {
        let channels = rcv.changed().await;
        let pos = decode(channels.raw[SC_IDX]);
        if Some(pos) == last { continue; }

        let target = match pos {
            Pos::Low          => INDOOR_CEILING_M,
            Pos::Mid | Pos::High => OUTDOOR_CEILING_M,
        };
        alt_hold::set_ceiling(target);
        // Lean on the magnetometer for yaw only when the pilot has declared
        // outdoor; indoors the building's hard/soft-iron distortion makes the
        // mag heading unreliable. Same SC gate as the altitude ceiling.
        MAG_TRUST_OUTDOOR.store(!matches!(pos, Pos::Low), Ordering::Relaxed);
        last = Some(pos);

        let label = match pos {
            Pos::Low  => "Low (indoor 2m)",
            Pos::Mid  => "Mid (outdoor 100m)",
            Pos::High => "High (outdoor 100m)",
        };
        let mut s: heapless::String<64> = heapless::String::new();
        let _ = core::fmt::Write::write_fmt(
            &mut s,
            format_args!("[ceiling] SC={} -> {:.1}m", label, target),
        );
        ulog::log(s.as_str());

        // Cooperative yield to avoid spinning if the same change fires
        // back-to-back from rapid frames.
        Timer::after_millis(10).await;
    }
}
