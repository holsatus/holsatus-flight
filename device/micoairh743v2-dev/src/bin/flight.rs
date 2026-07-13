//! MicoAir H743 -- free-flight test binary.
//!
//! Built on flight.rs task structure verbatim (known working reference).
//! Flight behaviour is driven by `mission_fsm_task`, a state machine with
//! two operator-selectable flight modes plus the autonomous landing and
//! fault-handling states they fall through to.
//!
//! Modes (selected from the TX15 via rc_kill):
//!   Manual:  pure ACRO/rate mode. Sticks command angular rates directly
//!            (no self-levelling); throttle stick maps to collective thrust.
//!            Pre-arms on Manual entry while the stick is at idle (the
//!            governor's ~2.25 s arm window runs before the operator
//!            reaches the throttle; wait for the solid-green LED). Disarms
//!            after throttle + mode have been idle for
//!            MANUAL_IDLE_DWELL_MS. The arm is aborted if the throttle
//!            rises above ARM_GUARD_MAX_THR before the sequence completes
//!            (throttle-high-on-arm guard, D000020) and re-fires once the
//!            stick returns to idle.
//!   Auto:    closed-loop altitude-hold mission, trigger-armed from
//!            GroundIdle, running AutoTakeoff -> AutoHover -> AutoLand:
//!     AutoTakeoff: 1 s open-loop thrust ramp to BASE_THRUST (pushes through
//!                  stick-slip), then 3 s climb of ALTITUDE_SETPOINT 0 -> 1.0 m
//!                  (alt_hold owns TRUE_Z_THRUST_SP from here on).
//!     AutoHover:   hold 1.0 m. Exits to AutoLand on Land trigger, mode=Idle,
//!                  or AUTO_HOVER_TIMEOUT_S; a Hover trigger refreshes the
//!                  timeout to extend the hover.
//!     AutoLand:    1 s descent 1.0 -> 0.25 m, then 1 s 0.25 -> 0.05 m, with
//!                  lidar ground-detect disarm (h<0.15m for 300 ms) and a 5 s
//!                  safety timeout, then back to GroundIdle.
//!   rate-PID gains at flight.rs boot defaults.
//!
//! Failsafes: RC link loss while armed takes the autonomous landing path in
//! Auto and disarms immediately in Manual; low battery while armed latches a
//! forced descent ramp. flip_kill and gyro_runaway_kill back everything up.
//!
//! HIL bench mode (auto-selected at boot): if the battery sense reads USB
//! power (< 6 V -- FC powered from the FTDI wire, no pack), USART1 is
//! rerouted from the text-log mirror to the binary HIL link and the
//! physical BMI088 reader is replaced by host-injected IMU data
//! (hil_link_task / hil_imu_task at the bottom of this file); the BMI088
//! chip-mount rotation override and the GNSS reader are skipped. On pack
//! power everything runs the normal flight configuration. One binary, no
//! reflashing between bench HIL sessions and flights.
//!
//! Status LED (see src/led.rs for the renderer):
//!   DARK        booting / calibrating -- not ready yet
//!   BLUE        ready to arm (flip SA: Mid=Manual, High=Auto)
//!   GREEN blink arm sequence running -- hands off the throttle
//!   GREEN       armed, ready to fly -- push throttle up
//!   RED solid   an RC input / gate is preventing arming (fixable now)
//!   RED strobe  error: SD missing, boot abort, Fault, severe battery
//!
//! IMPORTANT: the drone WILL lift off in either mode. Be ready on the TX15
//! SE kill switch at all times.
//!
//! Gyro-runaway autoabort at 5 rad/s.
//!
//! SAFETY:
//!   - Props on, landing cage (Kapla sticks) installed, textured floor
//!     (newspaper or similar -- bare wood starves flow_hold per D000033)
//!   - 1.5-2 m clear on all sides
//!   - TX15 SE kill switch tested on the bench BEFORE arming flight
//!     (motors must go silent the instant SE flips)
//!   - Firmware backups: flip_kill (az < -3 m/s^2 for 10 ms),
//!     gyro_runaway_kill (any axis > 5 rad/s for 50 ms)
//!   - Battery is belly-mounted inside the landing cage; it is NOT
//!     reachable in flight, so SE on the TX15 is the only human kill

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};

use defmt_rtt as _;
// NOTE: panic_probe is intentionally NOT used in this binary; we provide
// our own #[panic_handler] below that writes "!!! PANIC !!! file:line"
// directly to USART1 via raw register pokes, then halts. See
// `panic_handler` near the bottom of this file.

/// Stack scan from `_defmt_panic` entry. Frame-walking via `r7` is
/// unreliable in optimized async code, so instead we scan the next N
/// stack words upward and keep any that look like valid `.text`
/// return addresses (low bit set for thumb, in `.text` VMA range).
/// Some of those will be saved-LR slots from real frames.
const PANIC_HITS: usize = 12;
static PANIC_LRS: [AtomicU32; PANIC_HITS] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// Set by each task at the top of its main work / before any await. The
/// value at panic time tells us which task's body was last running. Tasks
/// that don't set this read as 0 (TASK_NONE).
pub static CURRENT_TASK_ID: AtomicU32 = AtomicU32::new(0);
static PANIC_TASK_ID: AtomicU32 = AtomicU32::new(0);
pub const TASK_FSM: u32 = 1;
pub const TASK_BMI270: u32 = 2;
pub const TASK_MAG_YAW: u32 = 3;
pub const TASK_FLOW_POS_LOG: u32 = 4;
pub const TASK_MOTOR_MON: u32 = 5;
pub const TASK_IMU_MON: u32 = 6;
pub const TASK_FLIP_KILL: u32 = 7;
pub const TASK_GYRO_KILL: u32 = 8;
pub const TASK_ANGLE_TO_RATE: u32 = 9;
pub const TASK_FLOW_HOLD: u32 = 10;

/// The genuine BL+4 caller address, captured by the `#[naked]` `_defmt_panic`
/// override before any compiler-generated code can clobber LR.
static PANIC_REAL_LR: AtomicU32 = AtomicU32::new(0);

/// Counter incremented by `_defmt_panic` every time it is entered. Used by
/// the `#[panic_handler]` to distinguish "the current panic actually went
/// through defmt's _defmt_panic" (count > 0 — `PANIC_REAL_LR` is fresh)
/// from "the current panic is a Rust core panic that bypassed defmt"
/// (count == 0 — `PANIC_REAL_LR` is either zero or stale from a prior
/// reboot — but .bss is zeroed at boot so realistically zero means the
/// naked function never ran this run).
static PANIC_DEFMT_COUNT: AtomicU32 = AtomicU32::new(0);

/// First-arg register captured at `_defmt_panic` entry. Defmt's panic_
/// signature takes no args, so r0 should be a leftover from the caller's
/// last operation; comparing r0 to `panic_lr` may reveal whether the caller
/// is passing data via non-standard ABI.
static PANIC_R0_AT_ENTRY: AtomicU32 = AtomicU32::new(0);

/// First 4 stack words at `_defmt_panic` entry. If the panic's true caller
/// pushed args/return-address onto the stack before the BL, those will be
/// visible here and may give us a real return address even when LR is
/// already a scratch-register-corrupted value.
static PANIC_STACK_DUMP: [AtomicU32; 4] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// `#[naked]` override of defmt's `_defmt_panic`. With no compiler prologue,
/// LR at function entry is unambiguously the caller's BL+4 return address.
/// We snapshot it into `PANIC_REAL_LR`, increment `PANIC_DEFMT_COUNT` so the
/// panic handler can prove this run actually went through `_defmt_panic`,
/// save sp into r4 so the thunk can scan the stack, and tail-branch into
/// the thunk for the rest (task-id snapshot, stack scan, core::panic!()).
#[unsafe(naked)]
#[export_name = "_defmt_panic"]
unsafe extern "C" fn defmt_panic_override() -> ! {
    core::arch::naked_asm!(
        "mov r4, sp",                      // r4 = sp at entry (preserved across thunk call)
        "mov r5, r0",                      // r5 = r0 at entry (preserved)
        // Capture LR.
        "ldr r0, ={panic_lr}",
        "str lr, [r0]",
        // Capture r0-at-entry.
        "ldr r0, ={panic_r0}",
        "str r5, [r0]",
        // Capture first 4 stack words (sp+0, sp+4, sp+8, sp+12).
        "ldr r0, ={panic_stack0}",
        "ldr r1, [r4]",
        "str r1, [r0]",
        "ldr r1, [r4, #4]",
        "str r1, [r0, #4]",
        "ldr r1, [r4, #8]",
        "str r1, [r0, #8]",
        "ldr r1, [r4, #12]",
        "str r1, [r0, #12]",
        // (Layout note: PANIC_STACK_DUMP is `[AtomicU32; 4]`, so the four
        //  AtomicU32 elements ARE contiguous in `.bss` — accessing them
        //  via `r0+0/4/8/12` is sound.)
        // Increment defmt-entry counter.
        "ldr r0, ={defmt_count}",
        "ldr r1, [r0]",
        "adds r1, #1",
        "str r1, [r0]",
        // Tail-call into thunk with sp-at-entry as arg.
        "mov r0, r4",
        "b {thunk}",
        panic_lr = sym PANIC_REAL_LR,
        panic_r0 = sym PANIC_R0_AT_ENTRY,
        panic_stack0 = sym PANIC_STACK_DUMP,
        defmt_count = sym PANIC_DEFMT_COUNT,
        thunk = sym defmt_panic_thunk,
    )
}

extern "C" fn defmt_panic_thunk(sp_at_entry: u32) -> ! {
    // .text VMA range from the linker (objdump -h). Use generous bounds
    // to catch any future relayout. Upper bound is well above the
    // current ~0x0804C000 .text end so binary growth doesn't silently
    // make the scanner drop real return addresses.
    const TEXT_LO: u32 = 0x0800_0298;
    const TEXT_HI: u32 = 0x0808_0000;
    const STACK_HI: u32 = 0x2408_0000;
    let task_id = CURRENT_TASK_ID.load(Ordering::Relaxed);
    PANIC_TASK_ID.store(task_id, Ordering::Relaxed);
    let mut hits = 0usize;
    let mut addr = sp_at_entry;
    while hits < PANIC_HITS && addr + 4 <= STACK_HI {
        let v = unsafe { core::ptr::read_volatile(addr as *const u32) };
        if (v & 1) == 1 && (v & 0xFFFF_FFFE) >= TEXT_LO && (v & 0xFFFF_FFFE) < TEXT_HI {
            PANIC_LRS[hits].store(v, Ordering::Relaxed);
            hits += 1;
        }
        addr += 4;
    }
    core::panic!()
}

/// Custom panic handler. Writes "!!! PANIC !!! file:line\r\n" to USART1
/// via raw register pokes (NOT the embassy driver) and halts in WFI.
///
/// Constraints (deliberate, to avoid panic-during-panic):
///   - First instruction disables interrupts (CPSID I via cortex_m).
///   - No allocations, no `format!` / `write!`, no PAC abstractions.
///     Integer-to-decimal is done by hand into a fixed [u8; 12] stack
///     buffer. Bounded loops only.
///   - Direct volatile writes to USART1 TDR (0x4001_1028) with a polled
///     wait on the TXE_TXFNF bit (bit 7 of ISR at 0x4001_101C). Spin
///     counter caps each wait so a wedged UART can never deadlock the
///     handler.
///   - Brief NOP delay up front so any in-flight DMA write from the
///     embassy UART driver has time to drain before we start poking
///     TDR ourselves; otherwise our bytes interleave with the DMA's.
///   - Loop ends in WFI; no further code paths to fault on.
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    // Wait for any in-flight UART DMA byte to drain. ~10 ms at 100 MHz.
    // Generous; we're already halted, no rush.
    for _ in 0..1_000_000u32 {
        cortex_m::asm::nop();
    }

    // STM32H743 USART1 register addresses.
    const USART1_ISR: *mut u32 = 0x4001_101C as *mut u32;
    const USART1_TDR: *mut u32 = 0x4001_1028 as *mut u32;
    const TXE_TXFNF: u32 = 1 << 7;

    fn putc(b: u8) {
        // Spin until the transmit register/FIFO has room, with a hard
        // cap so a wedged peripheral can't hold us here forever.
        let mut spin: u32 = 0;
        loop {
            let isr = unsafe { core::ptr::read_volatile(USART1_ISR) };
            if isr & TXE_TXFNF != 0 {
                break;
            }
            spin = spin.wrapping_add(1);
            if spin > 1_000_000 {
                return;
            }
        }
        unsafe { core::ptr::write_volatile(USART1_TDR, b as u32) };
    }

    fn puts(s: &[u8]) {
        for &b in s {
            putc(b);
        }
    }

    puts(b"\r\n!!! PANIC !!!\r\n");

    if let Some(loc) = info.location() {
        puts(loc.file().as_bytes());
        putc(b':');

        // Manual integer-to-decimal of loc.line() into a 12-byte buffer.
        let line = loc.line();
        let mut buf = [0u8; 12];
        let mut idx: usize = 0;
        if line == 0 {
            buf[0] = b'0';
            idx = 1;
        } else {
            let mut n = line;
            while n > 0 && idx < buf.len() {
                buf[idx] = (n % 10) as u8 + b'0';
                idx += 1;
                n /= 10;
            }
            // Reverse in place.
            let mut i = 0;
            let mut j = idx - 1;
            while i < j {
                buf.swap(i, j);
                i += 1;
                j = j.saturating_sub(1);
            }
        }
        puts(&buf[..idx]);
    }

    putc(b'\r');
    putc(b'\n');

    // If a defmt::panic! / unwrap! / unreachable! routed through our
    // __defmt_panic, that handler captured LR (the address of the
    // instruction immediately after the BL __defmt_panic call). Print
    // it as 0x... so `addr2line -e target/.../flight 0x<value>`
    // can map back to the source line.
    fn put_hex32(v: u32, putc: fn(u8)) {
        for i in (0..8).rev() {
            let nibble = ((v >> (i * 4)) & 0xF) as u8;
            let c = if nibble < 10 {
                b'0' + nibble
            } else {
                b'a' + (nibble - 10)
            };
            putc(c);
        }
    }
    // Genuine BL+4 caller of `_defmt_panic`, captured by the naked-asm
    // override. Only meaningful if the defmt counter shows the override
    // actually ran for THIS panic — a non-zero `panic_lr` with a zero
    // counter means the current panic bypassed defmt entirely (e.g., a
    // Rust core panic from a `.unwrap()` outside any defmt macro), and
    // the LR value is stale.
    let real_lr = PANIC_REAL_LR.load(Ordering::Relaxed);
    let defmt_count = PANIC_DEFMT_COUNT.load(Ordering::Relaxed);
    puts(b"  defmt_count=");
    {
        let mut buf = [0u8; 12];
        let mut idx: usize = 0;
        if defmt_count == 0 {
            buf[0] = b'0';
            idx = 1;
        } else {
            let mut n = defmt_count;
            while n > 0 && idx < buf.len() {
                buf[idx] = (n % 10) as u8 + b'0';
                idx += 1;
                n /= 10;
            }
            let mut i = 0;
            let mut j = idx - 1;
            while i < j {
                buf.swap(i, j);
                i += 1;
                j = j.saturating_sub(1);
            }
        }
        puts(&buf[..idx]);
    }
    puts(b"\r\n");
    if real_lr != 0 {
        puts(b"  panic_lr=0x");
        put_hex32(real_lr, putc);
        puts(b"\r\n");
    }
    if defmt_count > 0 {
        let r0_at_entry = PANIC_R0_AT_ENTRY.load(Ordering::Relaxed);
        puts(b"  r0_at_entry=0x");
        put_hex32(r0_at_entry, putc);
        puts(b"\r\n");
        for (i, slot) in PANIC_STACK_DUMP.iter().enumerate() {
            let v = slot.load(Ordering::Relaxed);
            puts(b"  sp+");
            putc(b'0' + (i as u8) * 4);
            puts(b"=0x");
            put_hex32(v, putc);
            puts(b"\r\n");
        }
    }
    let mut any = false;
    for (i, slot) in PANIC_LRS.iter().enumerate() {
        let v = slot.load(Ordering::Relaxed);
        if v == 0 {
            break;
        }
        any = true;
        puts(b"  fr");
        putc(b'0' + i as u8);
        puts(b"=0x");
        put_hex32(v, putc);
        puts(b"\r\n");
    }
    if !any {
        puts(b"  no defmt frames\r\n");
    }
    // Dump the last 64 bytes of defmt-rtt's BUFFER, anchored at the
    // up-channel write cursor in the SEGGER RTT control block. The most
    // recent defmt frame ends at write_cursor-1; its leading symbol-id
    // varint identifies the panic message in the .defmt section. Cross-
    // reference with `arm-none-eabi-nm <elf> | grep defmt_error` to find
    // the matching disambiguator.
    const RTT_HEADER_WRITE_OFFSET: u32 = 0x2400_0140; // up_channel.write
    const RTT_BUFFER_BASE: u32 = 0x2400_8ae8; // BUFFER (1024 bytes)
    const RTT_BUFFER_SIZE: u32 = 1024;
    let write_cursor = unsafe { core::ptr::read_volatile(RTT_HEADER_WRITE_OFFSET as *const u32) };
    if write_cursor < RTT_BUFFER_SIZE {
        puts(b"  rtt_w=");
        put_hex32(write_cursor, putc);
        puts(b"\r\n  rtt_tail=");
        // Print the 64 bytes ending at write_cursor (wrapping).
        let n_bytes: u32 = 64;
        let start = (write_cursor + RTT_BUFFER_SIZE - n_bytes) % RTT_BUFFER_SIZE;
        for i in 0..n_bytes {
            let off = (start + i) % RTT_BUFFER_SIZE;
            let b = unsafe { core::ptr::read_volatile((RTT_BUFFER_BASE + off) as *const u8) };
            // Two hex digits per byte.
            let hi = b >> 4;
            let lo = b & 0xF;
            putc(if hi < 10 { b'0' + hi } else { b'a' + hi - 10 });
            putc(if lo < 10 { b'0' + lo } else { b'a' + lo - 10 });
            if i & 0xF == 0xF {
                putc(b' ');
            }
        }
        puts(b"\r\n");
    }
    let tid = PANIC_TASK_ID.load(Ordering::Relaxed);
    puts(b"  task_id=");
    let mut buf = [0u8; 12];
    let mut idx: usize = 0;
    if tid == 0 {
        buf[0] = b'0';
        idx = 1;
    } else {
        let mut n = tid;
        while n > 0 && idx < buf.len() {
            buf[idx] = (n % 10) as u8 + b'0';
            idx += 1;
            n /= 10;
        }
        let mut i = 0;
        let mut j = idx - 1;
        while i < j {
            buf.swap(i, j);
            i += 1;
            j = j.saturating_sub(1);
        }
    }
    puts(&buf[..idx]);
    puts(b"\r\n");

    loop {
        cortex_m::asm::wfi();
    }
}

/// Latest optical flow quality, shared between mtf01_reader and the lateral
/// controller for gating + logging.
static FLOW_QUALITY: AtomicU8 = AtomicU8::new(0);

// --- DJI-style Auto control plane (single-writer cross-task signals) ---
//
// All f32 values are stored as raw bits in an AtomicU32 (exact, lock-free),
// matching the `alt_hold::ACTIVE_CEILING_M_BITS` pattern. The FSM is the only
// writer of the *_SP / yaw / active signals; `lateral_controller` is the only
// writer of LAT_TILT_*; the FSM is the only reader of LAT_TILT_*.

/// Auto lateral stick deflection, normalised [-1, 1]: forward (+x, pitch stick)
/// and right (+y, roll stick). Written by the FSM; `lateral_controller` maps it
/// to a direct tilt and fades the GPS auto-brake out as the stick deflects.
static AUTO_LAT_STICK_FWD: AtomicU32 = AtomicU32::new(0);
static AUTO_LAT_STICK_RIGHT: AtomicU32 = AtomicU32::new(0);

/// Lateral tilt command (rad), body frame: roll (+ = right-down) and pitch
/// (+ = nose-up), produced by `lateral_controller` to achieve the commanded
/// velocity, consumed by the FSM when it composes TRUE_ATTITUDE_Q_SP.
static LAT_TILT_ROLL: AtomicU32 = AtomicU32::new(0);
static LAT_TILT_PITCH: AtomicU32 = AtomicU32::new(0);

/// Auto yaw-rate command (rad/s) from the yaw stick, injected into the rate
/// setpoint by `angle_to_rate_bridge` (which otherwise hard-zeros yaw).
static AUTO_YAW_RATE: AtomicU32 = AtomicU32::new(0);

/// True while the FSM is in a self-levelling state (Auto or Landing) and the
/// lateral controller should run and own LAT_TILT_*. When false the controller
/// idles and publishes zero tilt.
static AUTO_LATERAL_ACTIVE: AtomicBool = AtomicBool::new(false);

#[inline]
fn store_f32(a: &AtomicU32, v: f32) {
    a.store(v.to_bits(), Ordering::Relaxed);
}
#[inline]
fn load_f32(a: &AtomicU32) -> f32 {
    f32::from_bits(a.load(Ordering::Relaxed))
}

/// Unclamped flow-integrated body-frame displacement since the first flow
/// sample (mm). Updated by `flow_position_logger`, read at disarm by the
/// mission sequencer for landing-distance estimation. Separate from
/// `flow_hold`'s internal est_x/est_y because flow_hold clamps at +/-0.5 m,
/// which would saturate for the ~2 m drifts we see in practice and lose
/// information. These atomics are pure telemetry -- nothing in the control
/// loop reads them.
static FLOW_EST_X_MM: AtomicI32 = AtomicI32::new(0);
static FLOW_EST_Y_MM: AtomicI32 = AtomicI32::new(0);

use core::fmt::Write;

use common::errors::DeviceError;
use common::hw_abstraction::Imu6Dof;
use common::nalgebra::{UnitQuaternion, Vector3};
use common::signals;
use common::tasks::att_estimator;
use common::tasks::commander::COMMAD_ARM_VEHICLE;
use common::tasks::controller_angle;
use common::tasks::controller_rate;
use common::tasks::eskf::EskfEstimate;
use common::types::actuators::MotorsState;
use common::types::config::DshotConfig;
use common::types::measurements::Imu6DofData;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use micoairh743v2::alt_hold::ALTITUDE_SETPOINT;
use micoairh743v2::led::{self, LedMode};
use micoairh743v2::log as ulog;
use micoairh743v2::mtf01;
use micoairh743v2::resources::{
    self, Bmi270Resources, BtLogIrqs, BtLogResources, SensorIrqs, Mtf01Resources, SdmmcLogResources,
    UartLogIrqs, UartLogResources,
};
use micoairh743v2::sdlog::SdmmcResources;

/// Helper macro to create an interrupt executor at the given priority.
macro_rules! interrupt_executor {
    ($interrupt:ident, $prio:ident) => {{
        use embassy_executor::InterruptExecutor;
        use embassy_stm32::interrupt;
        use embassy_stm32::interrupt::{InterruptExt, Priority};

        interrupt::$interrupt.set_priority(Priority::$prio);
        static EXECUTOR: InterruptExecutor = InterruptExecutor::new();
        let spawner = EXECUTOR.start(interrupt::$interrupt);

        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $interrupt() {
            EXECUTOR.on_interrupt()
        }

        spawner
    }};
}

#[embassy_executor::main]
async fn main(thread_spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(micoairh743v2::config::embassy_config());

    let r = resources::split(p);

    // Status LED task owns the three LED pins; everything else publishes a
    // mode via led::set. Boots dark (Init) through cal -- see led.rs for
    // the full color legend.
    thread_spawner.spawn(micoairh743v2::led::led_task(r.leds).unwrap());

    // Battery monitor first: its seed sample decides HIL-vs-flight below.
    // It owns ADC1, samples at 10 Hz, publishes filtered mV via
    // BATTERY_FILTERED_MV. alt_hold reads the live signal each tick for its
    // voltage compensation; FSM/manual log can read it too.
    thread_spawner.spawn(micoairh743v2::battery::battery_monitor_task(r.battery).unwrap());

    // HIL-vs-flight switch. On USB bench power (pack sense < 6 V) USART1 is
    // rerouted from the text-log mirror to the binary HIL link and the
    // physical IMU/GNSS are replaced by host-injected data -- same binary
    // for bench and flight, no reflashing. The battery task publishes its
    // blocking seed sample before its first await, so this normally
    // resolves on the first poll; the timeout only covers a wedged ADC and
    // falls back to flight mode (real sensors -- safe in the air, merely
    // useless on the bench, whereas HIL mode in the air would fly on a
    // dead link).
    let hil_mode = {
        let start = embassy_time::Instant::now();
        loop {
            if let Some(mv) = micoairh743v2::battery::BATTERY_FILTERED_MV.try_get() {
                break micoairh743v2::battery::is_usb_power_range(mv);
            }
            if start.elapsed().as_millis() >= 2_000 {
                break false;
            }
            Timer::after_millis(10).await;
        }
    };
    // Route USART1 to exactly one owner.
    let (log_uart, hil_uart) = if hil_mode {
        (None, Some(r.uart_log))
    } else {
        (Some(r.uart_log), None)
    };

    thread_spawner.spawn(uart_writer_task(log_uart, r.bt_log, r.sdmmc).unwrap());

    ulog::log("[free] board init ok");
    // Git provenance line: every flight log now starts with the short SHA
    // that produced the binary (plus "-dirty" if the tree was not clean at
    // build time). Combined with the Makefile's `git-clean` pre-flash gate,
    // this gives end-to-end version tracing from log back to a commit.
    ulog::log(concat!("[free] git=", env!("GIT_SHA")));
    ulog::log(if hil_mode {
        "[free] USB power -- HIL mode: USART1=link, IMU=injected, GNSS off"
    } else {
        "[free] pack power -- flight mode: physical sensors"
    });

    signals::CONTROL_FREQUENCY.store(1000, Ordering::Relaxed);

    signals::ESKF_ESTIMATE.send(EskfEstimate {
        pos: Vector3::zeros(),
        vel: Vector3::zeros(),
        att: UnitQuaternion::identity(),
        gyr_bias: Vector3::zeros(),
        acc_bias: Vector3::zeros(),
    });

    signals::TRUE_RATE_SP.send([0.0_f32; 3]);
    signals::TRUE_Z_THRUST_SP.send(0.0_f32);
    signals::TRUE_ATTITUDE_Q_SP.send(UnitQuaternion::identity());

    thread_spawner.spawn(param_storage_task().unwrap());

    override_motor_params().await;
    override_pid_gains().await;
    if !hil_mode {
        // HIL data arrives pre-rotated in drone body frame (the host builds
        // it; there is no physical chip mounting to correct for), so the
        // imu_reader rotation stays at its Identity default in HIL mode.
        override_imu_rot().await;
    }

    Timer::after_millis(10).await;

    let level_0_spawner = interrupt_executor!(FDCAN1_IT0, P10);
    let level_1_spawner = interrupt_executor!(FDCAN1_IT1, P11);

    ulog::log("[free] executors started");

    Timer::after_millis(1).await;

    if !hil_mode {
        // In HIL mode hil_imu_task (below) feeds imu_reader::main_6dof from
        // the link instead of the BMI088 driver.
        level_0_spawner.spawn(resources::imu_reader_task(r.imu).unwrap());
    }
    level_0_spawner.spawn(resources::motor_governor_task(r.motors, DshotConfig::Dshot300).unwrap());
    level_0_spawner.spawn(controller_rate::main().unwrap());

    // Cal-independent tasks. Spawned before imu_cal so they are alive while
    // the operator holds the drone level for calibration:
    //   - rc_kill: kill switch must be functional during cal
    //   - ceiling_mode: just reads RC, no IMU dependency
    //   - gnss: ublox parser, independent of IMU
    //   - odid: legal Remote ID broadcast should start at power-on, not
    //     after a (potentially multi-minute) cal dance
    //   - phoenix_telem: must come up before cal too, otherwise on the
    //     bench (where the drone isn't held still) imu_cal::apply().await
    //     never returns and we never get past line 530, so the Pi-side
    //     logger sees zero MAVLink and the bench debug console is dark.
    //     Tracked to D000350/D000360/D000365 silent USART2 incident.
    thread_spawner.spawn(micoairh743v2::rc_kill::rc_kill_task(r.rc).unwrap());
    // Find-my-drone: beep the ESCs if RC is gone >30 s (crash / TX off / out of range).
    thread_spawner.spawn(micoairh743v2::resources::rc_loss_beacon_task().unwrap());
    thread_spawner.spawn(micoairh743v2::ceiling_mode::ceiling_mode_task().unwrap());
    match hil_uart {
        // HIL: the binary link + injected-IMU feed replace the GNSS reader
        // (nothing injects GNSS today; on the bench the auto-arm lateral
        // reference comes from the lidar path).
        Some(u) => {
            thread_spawner.spawn(hil_link_task(u).unwrap());
            thread_spawner.spawn(hil_imu_task().unwrap());
        }
        None => {
            thread_spawner.spawn(micoairh743v2::gnss::gnss_reader_task(r.gps).unwrap());
        }
    }
    thread_spawner.spawn(micoairh743v2::odid::odid_tx_task(r.odid).unwrap());
    thread_spawner.spawn(micoairh743v2::phoenix_telem::phoenix_telem_task(r.telem).unwrap());

    // Calibrate IMU BEFORE spawning att_estimator. Otherwise Madgwick
    // integrates 2-3 s of uncalibrated (biased) gyro data into the attitude
    // estimate before ReloadParams takes effect. At beta=0.03 the filter
    // takes ~30 s to un-wind that error via accel correction, and we arm in
    // ~10 s -- so the drone arms with 5-10 deg of fake tilt baked into the
    // estimate (see D000063: [cal] gyr bias ~3 deg/s, [ahrs] ready reported
    // p=8 deg when the drone was physically level). The angle controller then
    // commands 2+ rad/s "correction" on arming, tipping the real drone.
    micoairh743v2::imu_cal::apply().await;

    level_1_spawner.spawn(att_estimator::main().unwrap());
    level_1_spawner.spawn(ahrs_to_eskf_bridge().unwrap());
    level_1_spawner.spawn(controller_angle::main().unwrap());
    level_1_spawner.spawn(angle_to_rate_bridge().unwrap());

    thread_spawner.spawn(resources::alt_hold_task(r.baro).unwrap());
    thread_spawner.spawn(mtf01_reader_task(r.mtf01).unwrap());
    // flow_hold re-enabled 2026-04-22 in velocity-damping-only mode (KP_POS=0)
    // to counter the steady left-drift seen in D000116 without triggering the
    // D000049 positive-feedback failure at borderline liftoff. The activation
    // threshold in mtf01_reader_task was also raised from 5 cm to 10 cm so
    // flow only engages once the drone is solidly airborne, not in the
    // transition regime. See also: flow_hold::KP_POS = 0.0 below.
    // Unified lateral velocity controller (DJI-style motion). Tracks the
    // FSM's body-velocity setpoint using optical flow (low) or GPS (high) and
    // publishes a tilt command the FSM folds into the attitude setpoint.
    thread_spawner.spawn(lateral_controller().unwrap());
    thread_spawner.spawn(flip_kill().unwrap());
    thread_spawner.spawn(gyro_runaway_kill().unwrap());
    thread_spawner.spawn(mission_fsm_task().unwrap());
    thread_spawner.spawn(motor_monitor().unwrap());
    thread_spawner.spawn(imu_monitor().unwrap());
    thread_spawner.spawn(flow_position_logger().unwrap());
    thread_spawner.spawn(mag_yaw_logger().unwrap());
    thread_spawner.spawn(bmi270_logger_task(r.bmi270).unwrap());

    // Phoenix mission-logger MAVLink link to the RPi5 companion (USART2 on
    // the DJI-VTX header). Moved up into the cal-independent block above
    // because imu_cal::apply().await blocks indefinitely on the bench;
    // see the comment there. The task streams the topics from
    // `phoenix/references/holsatus_data_logging.md` at the design-doc
    // rates and answers Pi-initiated HEARTBEAT/TIMESYNC. Does not gate
    // arming today (no consumer of phoenix_telem::COMPANION_HEALTHY).

    ulog::log("[free] all tasks spawned");

    {
        let wait_start = embassy_time::Instant::now();
        while ulog::SD_MOUNTED.load(Ordering::Relaxed) == 255
            && wait_start.elapsed().as_millis() < 15_000
        {
            Timer::after_millis(100).await;
        }
    }
    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[free] SD ABORT -- motors will not arm (red strobe)");
        micoairh743v2::led::set(micoairh743v2::led::LedMode::Error);
        loop {
            Timer::after_secs(60).await;
        }
    }

    // dshot-rate log line every 2 s. All LED rendering (liveness flick,
    // battery-severe red strobe, FSM state colors) lives in led::led_task.
    let mut prev_tx =
        micoairh743v2::dshot_driver::DSHOT_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
    loop {
        Timer::after_secs(2).await;
        let cur_tx =
            micoairh743v2::dshot_driver::DSHOT_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
        let rate = (cur_tx - prev_tx) / 2;
        prev_tx = cur_tx;
        let mut s: heapless::String<64> = heapless::String::new();
        let _ = write!(s, "[free] hb dshot={}Hz", rate);
        ulog::log(s.as_str());
    }
}

async fn override_motor_params() {
    use common::tasks::motor_governor::params;
    use micoairh743v2::config::MOTOR_REVERSE_FLAGS;

    let mut p = params::TABLE.params.write().await;
    p.rev = MOTOR_REVERSE_FLAGS;
    p.timeout_ms = 500;
    drop(p);

    let mut s: heapless::String<64> = heapless::String::new();
    let _ = write!(
        s,
        "[free] rev=0x{:04X} timeout=500ms",
        MOTOR_REVERSE_FLAGS.bits()
    );
    ulog::log(s.as_str());
}

async fn override_pid_gains() {
    use common::tasks::controller_rate;

    let mut r = controller_rate::params::TABLE.params.write().await;
    r.x.kp = 0.05;
    r.x.ki = 0.0;
    r.x.kd = 0.002;
    r.y.kp = 0.05;
    r.y.ki = 0.0;
    r.y.kd = 0.002;
    // Yaw Kp bumped 0.04 -> 0.07 after D000106 showed ~0.26 rad/s yaw drift
    // at airborne step 3 due to unmodelled CW/CCW drag asymmetry. Ki stays 0
    // because the rate controller's integrators are gated off on this board
    // (in_flight_estimator is not spawned, ATTITUDE_INT_EN never fires) so
    // setting ki would be a no-op -- see project_rate_ki_gated_off memo.
    r.z.kp = 0.07;
    r.z.ki = 0.0;
    r.z.kd = 0.001;
    drop(r);

    ulog::log("[free] PID gains overridden");
}

/// Configure the BMI088 chip-to-drone-body rotation matrix so that
/// RAW_MULTI_IMU_DATA (and downstream CAL_MULTI_IMU_DATA, ESKF, Madgwick,
/// rate/angle controllers) operate in the drone's arrow-forward NED frame
/// rather than the chip's native frame. Without this override, the default
/// rotation is identity -- which is wrong because the BMI088 on the MicoAir
/// H743v2 is physically mounted rotated 180 deg around the (1,-1,0) diagonal
/// relative to the drone's arrow. See `BMI088_CHIP_TO_DRONE_ROT` in config.rs
/// for the derivation and the empirical tilt-test evidence (2026-04-21).
async fn override_imu_rot() {
    use common::tasks::imu_reader::params;
    use common::utils::rot_matrix::Rotation;
    use micoairh743v2::config::BMI088_CHIP_TO_DRONE_ROT;

    let mut p = params::TABLE.params.write().await;
    p.rot = Rotation::Custom(BMI088_CHIP_TO_DRONE_ROT);
    drop(p);

    ulog::log("[free] IMU rotation set (chip -> drone NED)");
}

async fn wait_for_ahrs_ready() {
    use signals::AHRS_ATTITUDE_Q;

    const WINDOW_N: usize = 50;
    const STABLE_THRESHOLD_DEG: f32 = 0.3;
    const LEVEL_THRESHOLD_DEG: f32 = 3.0;
    const MAX_WAIT_MS: u64 = 10_000;

    let mut rcv = AHRS_ATTITUDE_Q.receiver();
    let mut roll_buf = [0.0_f32; WINDOW_N];
    let mut pitch_buf = [0.0_f32; WINDOW_N];
    let mut idx: usize = 0;
    let mut filled = 0usize;

    ulog::log("[ahrs] waiting for attitude to settle...");
    let start = embassy_time::Instant::now();

    loop {
        let q = rcv.changed().await;
        let (r, p, _y) = q.euler_angles();
        roll_buf[idx] = r.to_degrees();
        pitch_buf[idx] = p.to_degrees();
        idx = (idx + 1) % WINDOW_N;
        if filled < WINDOW_N {
            filled += 1;
        }

        if filled == WINDOW_N {
            let r_min = roll_buf.iter().cloned().fold(f32::INFINITY, f32::min);
            let r_max = roll_buf.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            let p_min = pitch_buf.iter().cloned().fold(f32::INFINITY, f32::min);
            let p_max = pitch_buf.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
            let r_range = r_max - r_min;
            let p_range = p_max - p_min;

            if r_range < STABLE_THRESHOLD_DEG && p_range < STABLE_THRESHOLD_DEG {
                let r_mean: f32 = roll_buf.iter().sum::<f32>() / WINDOW_N as f32;
                let p_mean: f32 = pitch_buf.iter().sum::<f32>() / WINDOW_N as f32;
                // Stable but tilted: abort so the user can level the drone.
                // Arming from a non-zero attitude reference causes the drone
                // to accelerate horizontally at liftoff instead of climbing
                // straight up (see D000065: p=-4.6 deg, drone drifted sideways).
                if r_mean.abs() > LEVEL_THRESHOLD_DEG || p_mean.abs() > LEVEL_THRESHOLD_DEG {
                    let mut s: heapless::String<96> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[ahrs] NOT LEVEL: r={:.2} p={:.2} > {:.1} deg -- ABORT, level the drone",
                        r_mean, p_mean, LEVEL_THRESHOLD_DEG
                    );
                    ulog::log(s.as_str());
                    led::set(LedMode::Error);
                    loop {
                        Timer::after_secs(60).await;
                    }
                }
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "[ahrs] ready: r={:.2} p={:.2} range=({:.2},{:.2})",
                    r_mean, p_mean, r_range, p_range
                );
                ulog::log(s.as_str());
                return;
            }
        }

        if start.elapsed().as_millis() >= MAX_WAIT_MS {
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "[ahrs] WARNING: not settled after {}ms, continuing anyway",
                MAX_WAIT_MS
            );
            ulog::log(s.as_str());
            return;
        }
    }
}

#[embassy_executor::task]
async fn uart_writer_task(
    r: Option<UartLogResources>,
    bt: BtLogResources,
    sd: SdmmcLogResources,
) -> ! {
    use block_device_adapters::BufStream;
    use core::fmt::Write as FmtWrite;
    use embedded_fatfs::{FileSystem, FsOptions};
    use embedded_io_async_061::Write as _;

    // None in HIL mode: hil_link_task owns USART1 for the binary protocol
    // and text logs go to BT + SD only.
    let mut uart =
        r.and_then(|r| UartTx::new(r.usart, r.tx, r.dma, UartLogIrqs, UartConfig::default()).ok());
    // Onboard BT module (115200 by default, matches USART1). If init fails
    // we silently continue -- USART1 + SD logs still work.
    let mut bt_uart = UartTx::new(bt.usart, bt.tx, bt.dma, BtLogIrqs, UartConfig::default()).ok();

    // Mirror one log line (+ CRLF) to USART1 (wired FTDI) and UART8 (BT).
    // Either may be None if init failed; both writes are best-effort.
    async fn write_line(
        uart: &mut Option<UartTx<'static, embassy_stm32::mode::Async>>,
        bt: &mut Option<UartTx<'static, embassy_stm32::mode::Async>>,
        msg: &[u8],
    ) {
        if let Some(u) = uart.as_mut() {
            u.write(msg).await.ok();
            u.write(b"\r\n").await.ok();
        }
        if let Some(u) = bt.as_mut() {
            u.write(msg).await.ok();
            u.write(b"\r\n").await.ok();
        }
    }

    let mut device = SdmmcResources {
        periph: sd.periph,
        clk: sd.clk,
        cmd: sd.cmd,
        d0: sd.d0,
        d1: sd.d1,
        d2: sd.d2,
        d3: sd.d3,
    }
    .setup();

    let sd_ok = {
        let mut ok = false;
        for _ in 0u8..3 {
            if device.try_reset().await.is_ok() {
                ok = true;
                break;
            }
            Timer::after_millis(500).await;
        }
        ok
    };

    if !sd_ok {
        ulog::SD_MOUNTED.store(0, Ordering::Relaxed);
        ulog::log("[sd] NOT FOUND (tried 3x) -- motors will NOT arm");
        loop {
            let msg = ulog::CHANNEL.receive().await;
            write_line(&mut uart, &mut bt_uart, msg.as_bytes()).await;
        }
    }

    let stream = BufStream::new(device);
    let fs = match FileSystem::new(stream, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(_) => {
            ulog::SD_MOUNTED.store(0, Ordering::Relaxed);
            ulog::log("[sd] reset OK but FAT mount FAILED -- motors will NOT arm");
            loop {
                let msg = ulog::CHANNEL.receive().await;
                write_line(&mut uart, &mut bt_uart, msg.as_bytes()).await;
            }
        }
    };

    let mut session_idx: u32 = 0;
    let mut iter = fs.root_dir().iter();
    while let Some(Ok(entry)) = iter.next().await {
        if entry.is_dir() {
            let name = entry.short_file_name_as_bytes();
            if name.len() == 7 && (name[0] == b'D' || name[0] == b'd') {
                if let Some(idx) = name[1..].iter().try_fold(0u32, |acc, &b| {
                    if b >= b'0' && b <= b'9' {
                        Some(acc * 10 + (b - b'0') as u32)
                    } else {
                        None
                    }
                }) {
                    session_idx = session_idx.max(idx);
                }
            }
        }
    }
    session_idx += 1;

    let mut dir_name: heapless::String<8> = heapless::String::new();
    let session_dir = loop {
        dir_name.clear();
        let _ = FmtWrite::write_fmt(&mut dir_name, format_args!("D{:06}", session_idx));
        match fs.root_dir().create_dir(dir_name.as_str()).await {
            Ok(d) => break d,
            Err(embedded_fatfs::Error::AlreadyExists) => {
                session_idx += 1;
            }
            Err(_) => loop {
                let msg = ulog::CHANNEL.receive().await;
                write_line(&mut uart, &mut bt_uart, msg.as_bytes()).await;
            },
        }
    };

    let mut file_name: heapless::String<12> = heapless::String::new();
    let _ = FmtWrite::write_fmt(&mut file_name, format_args!("000001.LOG"));
    let mut file = match session_dir.create_file(file_name.as_str()).await {
        Ok(f) => f,
        Err(_) => {
            ulog::SD_MOUNTED.store(0, Ordering::Relaxed);
            ulog::log("[sd] create_file failed -- motors will NOT arm");
            loop {
                let msg = ulog::CHANNEL.receive().await;
                write_line(&mut uart, &mut bt_uart, msg.as_bytes()).await;
            }
        }
    };

    ulog::SD_MOUNTED.store(1, Ordering::Relaxed);
    let mut s: heapless::String<32> = heapless::String::new();
    let _ = FmtWrite::write_fmt(
        &mut s,
        format_args!("[sd] mounted -> {}/000001.LOG", dir_name.as_str()),
    );
    ulog::log(s.as_str());

    // Original select(critical, channel) writer loop. The earlier
    // select3-with-timer variant for time-based flushing tickled an
    // executor-halt regression around `[free] all tasks spawned` on
    // the H743v2 (D000145, D000147). Without that timer, persistence
    // of critical lines (FSM transitions, kill / restart, boot
    // heartbeat) is still guaranteed because every critical message
    // triggers an immediate `file.flush()` below. Regular messages
    // batch-flush every 5 writes as before. Sparse logs that would
    // benefit from a timer flush should use `log_critical` instead.
    let mut flush_counter: u16 = 0;
    loop {
        while let Ok(msg) = ulog::CRITICAL_CHANNEL.try_receive() {
            write_line(&mut uart, &mut bt_uart, msg.as_bytes()).await;
            file.write_all(msg.as_bytes()).await.ok();
            file.write_all(b"\r\n").await.ok();
            file.flush().await.ok();
        }

        use embassy_futures::select::{select, Either};
        let msg = match select(ulog::CRITICAL_CHANNEL.receive(), ulog::CHANNEL.receive()).await {
            Either::First(m) => {
                write_line(&mut uart, &mut bt_uart, m.as_bytes()).await;
                file.write_all(m.as_bytes()).await.ok();
                file.write_all(b"\r\n").await.ok();
                file.flush().await.ok();
                continue;
            }
            Either::Second(m) => m,
        };

        if !ulog::is_high_rate_telemetry(msg.as_str()) {
            write_line(&mut uart, &mut bt_uart, msg.as_bytes()).await;
        }
        file.write_all(msg.as_bytes()).await.ok();
        file.write_all(b"\r\n").await.ok();

        flush_counter += 1;
        if flush_counter >= 5 {
            flush_counter = 0;
            file.flush().await.ok();
        }
    }
}

const DUMMY_FLASH_SIZE: u32 = 262_144;

#[embassy_executor::task]
async fn param_storage_task() -> ! {
    common::tasks::param_storage::entry(DummyFlash, 0..DUMMY_FLASH_SIZE).await
}

struct DummyFlash;

impl common::embedded_storage_async::nor_flash::ErrorType for DummyFlash {
    type Error = core::convert::Infallible;
}

impl common::embedded_storage_async::nor_flash::ReadNorFlash for DummyFlash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, _offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        bytes.fill(0xFF);
        Ok(())
    }

    fn capacity(&self) -> usize {
        DUMMY_FLASH_SIZE as usize
    }
}

impl common::embedded_storage_async::nor_flash::NorFlash for DummyFlash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 131_072;

    async fn write(&mut self, _offset: u32, _bytes: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn erase(&mut self, _from: u32, _to: u32) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Continuous near-ground detector shared by AutoLand and the AutoHover
/// stick-descent disarm. Returns true once lidar has stayed below `detect_m`
/// for at least `hold_ms`. `first_hit` holds the first-crossing instant: it is
/// set on the first sub-threshold reading and cleared whenever lidar rises
/// back above the threshold, so a single noisy frame cannot satisfy the dwell.
fn ground_hold_elapsed(
    lidar: f32,
    first_hit: &mut Option<embassy_time::Instant>,
    detect_m: f32,
    hold_ms: u64,
) -> bool {
    if lidar < detect_m {
        let t = first_hit.get_or_insert_with(embassy_time::Instant::now);
        t.elapsed().as_millis() as u64 >= hold_ms
    } else {
        *first_hit = None;
        false
    }
}

/// Slew the attitude setpoint quaternion toward the target (level + the
/// lateral controller's small tilt) at a bounded angular rate, then publish it.
/// Seeded to the *current* attitude on entry to a self-levelling state, so the
/// angle error starts at zero and grows no faster than `max_rate` -- this keeps
/// the rate the (output-unsaturated) angle controller commands well under the
/// 5 rad/s gyro-runaway kill even when handing off from a large Manual attitude.
/// Yaw of the target tracks the measured yaw (heading is commanded as a rate via
/// AUTO_YAW_RATE), so only roll/pitch actually slew. Works in axis-angle space
/// (scaled_axis), which is robust near the 180 deg / inverted case where slerp
/// is ambiguous.
fn slew_attitude_sp(
    att_sp_q: &mut UnitQuaternion<f32>,
    tilt_roll: f32,
    tilt_pitch: f32,
    current_q: &UnitQuaternion<f32>,
    max_rate: f32,
    dt: f32,
) {
    let cur_yaw = current_q.euler_angles().2;
    let target = UnitQuaternion::from_euler_angles(tilt_roll, tilt_pitch, cur_yaw);
    let rotvec = (att_sp_q.inverse() * target).scaled_axis();
    let angle = rotvec.norm();
    let max_step = max_rate * dt;
    let step = if angle > max_step && angle > 1.0e-9 {
        rotvec * (max_step / angle)
    } else {
        rotvec
    };
    *att_sp_q *= UnitQuaternion::from_scaled_axis(step);
    signals::TRUE_ATTITUDE_Q_SP.send(*att_sp_q);
}

/// Mission FSM. Single owner of TRUE_RATE_SP / TRUE_ATTITUDE_Q_SP /
/// TRUE_Z_THRUST_SP / ALTITUDE_SETPOINT. Consumes RC channel state and
/// edge events from `rc_kill` and routes between GroundIdle, Manual,
/// AutoTakeoff, AutoHover, AutoLand, Fault. See mission_fsm.md for the
/// state diagram and SOP.
#[embassy_executor::task]
async fn mission_fsm_task() -> ! {
    use micoairh743v2::rc_kill::{Mode, RC_CHANNELS, RC_EVENT};
    // Single source of truth for the state enum + its wire encoding, shared
    // with phoenix_telem (which streams it as NAMED_VALUE_INT "FSM" and in
    // HEARTBEAT.custom_mode). Aliased to `State` so the body below is unchanged.
    use micoairh743v2::fsm_state::{self, FsmState as State};

    ulog::log("[fsm] waiting 5s for cal + sensors...");
    Timer::after_secs(5).await;

    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[fsm] ABORT: no SD card, not arming");
        led::set(LedMode::Error);
        loop {
            Timer::after_secs(60).await;
        }
    }

    wait_for_ahrs_ready().await;

    {
        ulog::log("[fsm] waiting for RC link (30 s timeout)...");
        let start = embassy_time::Instant::now();
        const RC_WAIT_MS: u64 = 30_000;
        while !micoairh743v2::rc_kill::RC_LINK_READY.load(Ordering::Relaxed) {
            if start.elapsed().as_millis() >= RC_WAIT_MS {
                ulog::log("[fsm] ABORT: no RC link after 30 s -- no arm");
                led::set(LedMode::Error);
                loop {
                    Timer::after_secs(60).await;
                }
            }
            Timer::after_millis(100).await;
        }
        ulog::log("[fsm] RC link established");
    }

    let mut rc_rcv = RC_CHANNELS.receiver().expect("RC_CHANNELS");
    let mut lidar_rcv = micoairh743v2::alt_hold::LIDAR_ALT_M.receiver().unwrap();
    let mut motors_rcv = signals::MOTORS_STATE.receiver();
    let mut bat_tier_rcv = micoairh743v2::battery::BATTERY_TIER
        .receiver()
        .expect("BATTERY_TIER receiver slot");

    // Pre-flight RC sanity gate: refuse to enter the FSM until *every*
    // switch and stick is in a known-safe boot position. Without this,
    // a power-up with SA in Manual would arm the drone the moment
    // throttle ticked above idle, and a power-up with SD in High would
    // fire AutoTakeoff on the first GroundIdle tick. The check loops
    // forever -- this is intentional: the operator must physically
    // correct the offending control before the drone will do anything.
    {
        use micoairh743v2::rc_kill::{
            stick_norm, stick_throttle, Maneuver, Mode, PITCH_IDX, ROLL_IDX, THROTTLE_IDX, YAW_IDX,
        };
        const STICK_CENTER_TOL: f32 = 0.10;
        // 0.05 matches the auto-arm throttle gate (AUTOARM_THROTTLE_MAX) and
        // tolerates a TX endpoint that rests slightly off zero: D000508 idled
        // at 0.03, which the old 0.02 limit rejected so preflight never cleared.
        const THROTTLE_BOTTOM_TOL: f32 = 0.05;
        let mut last_log = embassy_time::Instant::now() - embassy_time::Duration::from_secs(10);
        let mut warned = false;
        loop {
            let ch = match rc_rcv.try_get() {
                Some(c) => c,
                None => {
                    Timer::after_millis(100).await;
                    continue;
                }
            };
            let roll_n = stick_norm(ch.raw[ROLL_IDX]);
            let pitch_n = stick_norm(ch.raw[PITCH_IDX]);
            let yaw_n = stick_norm(ch.raw[YAW_IDX]);
            let thr_n = stick_throttle(ch.raw[THROTTLE_IDX]);
            let mode_ok = ch.mode == Mode::Idle;
            let select_ok = ch.maneuver == Maneuver::Takeoff;
            let trigger_ok = ch.raw[8] < 1400; // SD (CH9) not in High
            let throttle_ok = thr_n <= THROTTLE_BOTTOM_TOL;
            let sticks_ok = roll_n.abs() < STICK_CENTER_TOL
                && pitch_n.abs() < STICK_CENTER_TOL
                && yaw_n.abs() < STICK_CENTER_TOL;
            // Battery: HEALTHY or USB_POWER (bench test) allow proceeding;
            // any worse tier blocks until the pack is charged + the FC
            // rebooted. None (monitor hasn't sampled yet, <100 ms after
            // boot) is treated as ok so we don't get stuck here forever.
            let bat_tier_now = bat_tier_rcv.try_get();
            let battery_ok = bat_tier_now
                .map(micoairh743v2::battery::tier_allows_flight)
                .unwrap_or(true);
            if mode_ok && select_ok && trigger_ok && throttle_ok && sticks_ok && battery_ok {
                if warned {
                    ulog::log("[fsm] preflight OK -- proceeding");
                }
                led::set(LedMode::Ready);
                break;
            }
            warned = true;
            // Something the operator can fix (switch/stick position or
            // battery) is holding preflight: solid red.
            led::set(LedMode::Blocked);
            if last_log.elapsed().as_secs() >= 2 {
                last_log = embassy_time::Instant::now();
                // Two lines back-to-back; each fits in LOG_LEN=64. The
                // second line carries the actual stick normals so the
                // operator can see how far off centre they are when
                // `stk=BAD` persists. Threshold is `STICK_CENTER_TOL`
                // (0.10) on |r|, |p|, |y|; throttle is `<=
                // THROTTLE_BOTTOM_TOL` (0.02).
                let mode_str = match ch.mode {
                    Mode::Idle => "Idle",
                    Mode::Manual => "Manual",
                    Mode::Auto => "Auto",
                };
                let sel_str = match ch.maneuver {
                    Maneuver::Takeoff => "Takeoff",
                    Maneuver::Hover => "Hover",
                    Maneuver::Land => "Land",
                };
                let bat_str = match bat_tier_now {
                    Some(t) => t.label(),
                    None => "?",
                };
                let mut s: heapless::String<96> = heapless::String::new();
                let _ =
                    write!(
                    s,
                    "[fsm] preflight WAIT: mode={}({}) sel={}({}) trig={} thr={} stk={} bat={}({})",
                    if mode_ok { "ok" } else { "BAD" }, mode_str,
                    if select_ok { "ok" } else { "BAD" }, sel_str,
                    if trigger_ok { "ok" } else { "BAD" },
                    if throttle_ok { "ok" } else { "BAD" },
                    if sticks_ok { "ok" } else { "BAD" },
                    if battery_ok { "ok" } else { "BAD" }, bat_str,
                );
                ulog::log(s.as_str());

                let mut s2: heapless::String<64> = heapless::String::new();
                let _ = write!(
                    s2,
                    "[fsm]   sticks r={:+.2} p={:+.2} y={:+.2} thr={:.2}",
                    roll_n, pitch_n, yaw_n, thr_n,
                );
                ulog::log(s2.as_str());
            }
            Timer::after_millis(100).await;
        }
    }

    // -- tunables --
    const BASE_THRUST: f32 = 8.00;
    const GROUND_DETECT_M: f32 = 0.15;
    const GROUND_HOLD_MS: u64 = 300;
    // AutoLand descends the *current* commanded altitude at a controlled rate
    // from wherever the drone is, instead of snapping to a fixed 1 m target.
    // This makes the Land trigger / RC-loss failsafe safe from altitude: a
    // smooth ramp down + the validated ground-detect disarm, never a jump.
    // alt_hold's own open-loop flare (setpoint < 0.20 m) cushions touchdown.
    const AUTOLAND_RATE_MS: f32 = 2.0; // autonomous descent rate (m/s)
    const LAND_FLOOR_M: f32 = 0.05; // setpoint floor; below GROUND_DETECT_M
                                    // Floor-dwell safety: only force-disarm once the setpoint has bottomed
                                    // out (we have descended all the way) and sat at the floor this long
                                    // without a lidar ground-detect. Cannot fire at altitude because the
                                    // setpoint is far above the floor there -- this is the altitude-safe
                                    // replacement for the old fixed 5 s from-entry timeout. Kept unchanged as
                                    // the worst-case backstop; the settle detector below normally disarms
                                    // sooner.
    const FLOOR_TIMEOUT_MS: u64 = 3_000;

    // Touchdown confirmation (primary fallback when the MTF-01 lidar is gone):
    // once the setpoint is at the floor AND the filtered vertical speed has
    // stalled to near zero for SETTLE_DWELL_MS, the descent has physically
    // stopped -- i.e. we are on the ground -- so disarm promptly instead of
    // waiting out the full blind FLOOR_TIMEOUT_MS. Safe against an airborne
    // cut: a still-descending drone reads |v| ~ AUTOLAND_RATE_MS, far above
    // the threshold, so this can only fire after real ground contact. Baro
    // noise can only DELAY it (re-arming the dwell), never trip it early.
    const SETTLE_VSPEED_MPS: f32 = 0.25;
    const SETTLE_DWELL_MS: u64 = 500;

    // -- DJI-style Auto control --
    // Throttle stick position maps directly to a target altitude (natural for
    // the ratcheted throttle: park it = hold that height). Clamped to the SC
    // ceiling downstream by alt_hold, so the 2 m indoor / 100 m outdoor hard
    // ceiling always holds. Right stick commands body velocity; left-stick yaw
    // commands yaw rate. The attitude setpoint is slewed from the measured
    // attitude on entry so a Manual->Auto/Landing handoff cannot trip the kill.
    const MISSION_CEILING_M: f32 = 100.0; // absolute setpoint clamp (SC ceiling is tighter)
    const ALT_FLOOR_M: f32 = 0.0; // throttle-bottom target = ground (alt_hold idles there)
    const AUTO_MAX_YAW_RATE: f32 = 2.0; // yaw-stick full deflection -> rad/s
    const ATT_SLEW_RATE: f32 = 2.0; // attitude-setpoint slew (rad/s); < 5 rad/s kill
    const AUTO_DT: f32 = 0.02; // Auto/Landing tick period (s), 50 Hz
                               // Throttle must be essentially at the bottom to auto-arm, so the direct
                               // thrust starts near zero and the drone arms at idle instead of lurching
                               // up. Raising the throttle then commands the climb.
    const AUTOARM_THROTTLE_MAX: f32 = 0.05;
    const AUTOARM_LEVEL_DEG: f32 = 10.0; // max tilt to auto-arm into Auto

    const ON_GROUND_LIDAR_M: f32 = 0.20;
    // Pure ACRO (rate-mode) defaults for Manual. Bardwell-style: drone
    // does not self-level; sticks command angular rates directly. Roll
    // and pitch share a max rate; yaw is slower because most airframes
    // have less yaw authority. EXPO=0.3 gives a softer center for
    // small precise corrections, near-linear at the extremes.
    const MANUAL_MAX_ROLL_RATE: f32 = 3.0; // rad/s, ~170 deg/s
    const MANUAL_MAX_PITCH_RATE: f32 = 3.0; // rad/s
    const MANUAL_MAX_YAW_RATE: f32 = 2.0; // rad/s, ~115 deg/s
    const MANUAL_EXPO: f32 = 0.3;
    // Full-stick collective = BASE_THRUST * this. Being swept upward to
    // find lift-off TWR for the ~506 g RPi5 payload build: at 1.4 full
    // stick only commanded ~57% DShot (per-motor force 2.8 of a modeled
    // 6.74 max), so there is large headroom. Raise in ~25% steps and watch
    // the SD log for under-load pack voltage and motor temp on each run.
    // 1.4 -> 1.75 (step 1).
    const MANUAL_THRUST_GAIN: f32 = BASE_THRUST * 2.00;
    const MANUAL_IDLE_DWELL_MS: u64 = 500;
    // Throttle-high-on-arm guard. The governor's ~2.25 s arming sequence
    // ignores throttle, so a stick wound up during that window slams the
    // motors to the live command the instant arming completes (D000020:
    // stick at 72% at completion -> instant uncontrolled leap, gyro-runaway
    // kill 0.6 s later). If throttle rises above this while the arm
    // sequence is still running, the FSM aborts the arm (the governor
    // honors a disarm sent mid-sequence before applying any motor speed);
    // the throttle must then return to idle before a new arm is accepted.
    const ARM_GUARD_MAX_THR: f32 = 0.10;
    // Motor-saturation marker threshold (DShot command). Motors pin at the
    // governor's out_max, which defaults to common's DSHOT_MAX (2047); we
    // flag at >=2040 to absorb the output lowpass settling just shy of the
    // rail. When `mot_sat=1` appears, the firmware thrust cap is reached and
    // raising MANUAL_THRUST_GAIN further buys nothing -- you are at the
    // physical motor/ESC/battery limit, not the software one.
    const MANUAL_SAT_CMD: u16 = 2040;
    const MANUAL_LOG_PERIOD_MS: u64 = 200;
    /// Linear ramp duration for battery-triggered forced descent. From
    /// BASE_THRUST to 0 over this window. 8 s is your "5-10 seconds"
    /// midpoint: gentle enough to stay controllable, fast enough to land
    /// before a marginal pack collapses further. Roll/pitch/yaw stick
    /// authority is preserved during the ramp so the operator can guide
    /// the descent to a safe spot.
    const FORCE_DESCENT_RAMP_MS: u64 = 8_000;

    fn drain_rc_events() {
        while let Ok(_) = RC_EVENT.try_receive() {}
    }

    fn zero_setpoints() {
        signals::TRUE_RATE_SP.send([0.0_f32; 3]);
        signals::TRUE_Z_THRUST_SP.send(0.0_f32);
        signals::TRUE_ATTITUDE_Q_SP.send(UnitQuaternion::identity());
        ALTITUDE_SETPOINT.signal(0.0);
    }

    let mut state = State::GroundIdle;
    drain_rc_events();
    zero_setpoints();
    ulog::log("[fsm] state=GroundIdle");

    // RC freshness watchdog: track when ch.seq last advanced. While armed,
    // a stale link for >RC_LINK_TIMEOUT_MS is a Fault and triggers an
    // automatic landing or disarm depending on state. While not armed
    // (GroundIdle), a stale link is logged but does not transition --
    // there's no flight to safe.
    //
    // The timeout is ground-aware: 500 ms in air, 1500 ms when we detect
    // near-ground low-throttle conditions for >=200 ms. Touch-down
    // transients (motor deceleration current spike, mechanical jolt
    // wiggling a marginal connector) can brown out the RX-side BEC for
    // ~500 ms; widening the timeout there absorbs a single brownout
    // without ever weakening in-air protection.
    const RC_LINK_TIMEOUT_AIR_MS: u64 = 500;
    const RC_LINK_TIMEOUT_GROUND_MS: u64 = 1500;
    const GROUND_DETECT_LIDAR_M: f32 = 0.10;
    const GROUND_DETECT_THR_N: f32 = 0.10;
    const GROUND_DETECT_DWELL_MS: u64 = 200;
    /// Soft threshold: gaps >= this and < current timeout trigger a
    /// one-shot warning when the link recovers. Picks up the near-misses
    /// that don't trip failsafe but indicate the link is unhealthy.
    const RC_NEAR_LOSS_MS: u64 = 300;
    let mut last_seq: u32 = 0;
    let mut last_seq_change = embassy_time::Instant::now();
    // Ground-grace dwell start; None when the gate condition is broken.
    let mut ground_grace_since: Option<embassy_time::Instant> = None;

    // For Manual exit detection: how long the throttle stick has been
    // in the idle deadband AND MODE has been Idle. Reset to None when
    // either condition is broken.
    let mut manual_idle_since: Option<embassy_time::Instant> = None;
    let mut manual_log_throttle = embassy_time::Instant::now();
    // Edge-trigger for the motor-saturation marker: true while at least
    // one motor is pinned at the DShot rail, so the rising/falling edges
    // are logged once each instead of every tick. See MANUAL_SAT_CMD.
    let mut motor_saturated: bool = false;

    // One-shot flag for the Manual arming command. The motor governor's
    // 4.5 s arm sequence keeps `armed=false` for ~450 FSM ticks, during
    // which the per-tick `if !armed && thr_n > 0.0` would otherwise
    // re-send arm and re-log every 10 ms. Set on first send, cleared on
    // any Manual exit (to GroundIdle or Fault).
    let mut arm_command_sent: bool = false;
    // One-shot flag so an arm refusal by low-battery is logged once per
    // Manual session, not every tick.
    let mut arm_refused_logged: bool = false;
    // Set when the throttle-high-on-arm guard aborts an arm attempt.
    // Blocks re-arming until the throttle stick returns to idle, so the
    // abort does not immediately re-trigger a fresh arm from the still-
    // raised stick. See ARM_GUARD_MAX_THR.
    let mut arm_guard_wait_idle: bool = false;
    // Battery-triggered forced descent: set to Some(start_instant) when
    // the FSM detects a non-flight-allowing tier while armed. None when
    // normal flight is permitted.
    let mut force_descent_since: Option<embassy_time::Instant> = None;

    // Current auto-commanded altitude setpoint, shared across AutoHover and
    // AutoLand so the descent always continues smoothly from wherever the drone
    // is. In AutoHover (direct-thrust) it tracks the live filtered altitude so
    // an AutoLand handoff starts from the real height; in AutoLand it is ramped
    // down at a controlled rate. `ground_first_hit` is shared by the AutoHover
    // descent disarm and AutoLand to track continuous near-ground lidar (see
    // ground_hold_elapsed). `land_floor_since` times how long AutoLand has held
    // the floor for the altitude-safe disarm.
    let mut auto_alt_sp: f32 = 0.0;
    let mut hover_log_t = embassy_time::Instant::now();
    let mut ground_first_hit: Option<embassy_time::Instant> = None;
    let mut land_floor_since: Option<embassy_time::Instant> = None;
    // Times how long AutoLand's vertical speed has stayed stalled at the floor,
    // for the touchdown-confirmation disarm (primary fallback without lidar).
    let mut land_settle_since: Option<embassy_time::Instant> = None;
    // Slewing attitude setpoint for Auto/Landing (self-levelling states). Seeded
    // to the measured attitude whenever `self_level_seed` is set on entry, then
    // slewed toward level + the lateral tilt command. Single owner of
    // TRUE_ATTITUDE_Q_SP while armed in those states.
    let mut att_sp_q: UnitQuaternion<f32> = UnitQuaternion::identity();
    let mut self_level_seed: bool = false;
    // Edge-gate for auto-arm: only arm into Auto once per SA->Auto transition,
    // so that landing back to Idle with SA still at Auto does NOT immediately
    // re-arm and re-spin the props. Cleared on auto-arm, re-armed whenever SA is
    // seen away from Auto in Idle. Starts true so the first SA->Auto works.
    let mut auto_arm_latch: bool = true;
    // Periodic "FC is ready" log while sitting in GroundIdle, so the
    // operator on the BT terminal sees a continuous heartbeat instead
    // of silence after preflight passes.
    const GROUND_IDLE_LOG_PERIOD_MS: u64 = 2_000;
    let mut last_ground_idle_log = embassy_time::Instant::now()
        .checked_sub(embassy_time::Duration::from_millis(
            GROUND_IDLE_LOG_PERIOD_MS,
        ))
        .unwrap_or_else(embassy_time::Instant::now);

    loop {
        CURRENT_TASK_ID.store(TASK_FSM, Ordering::Relaxed);
        // Publish current state for telemetry (phoenix streams it to the web
        // UI). Done at the top so every path -- including the early `continue`s
        // below -- keeps the published value fresh within one tick.
        fsm_state::publish(state);
        // Per-tick: pull latest RC channels and lidar. Watches return None
        // only if no value has ever been sent; after RC_LINK_READY+lidar
        // active they are reliably populated.
        let ch = match rc_rcv.try_get() {
            Some(c) => c,
            None => {
                Timer::after_millis(20).await;
                continue;
            }
        };
        let lidar = lidar_rcv.try_get().unwrap_or(99.0);
        let armed = motors_rcv.try_get().map(|m| m.is_armed()).unwrap_or(false);
        let bat_tier = bat_tier_rcv
            .try_get()
            .unwrap_or(micoairh743v2::battery::Tier::Healthy);
        let battery_allows_flight = micoairh743v2::battery::tier_allows_flight(bat_tier);

        // Latch forced-descent when armed and battery falls below the
        // flight-allowing threshold. Once latched, only a reboot clears
        // it -- the battery has to be charged + the FC reset before any
        // further motors-on operation. Pre-armed transitions don't latch
        // (no flight to abort).
        if armed && !battery_allows_flight && force_descent_since.is_none() {
            force_descent_since = Some(embassy_time::Instant::now());
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(
                s,
                "[fsm] BATTERY {} -- forcing landing ramp ({} s)",
                bat_tier.label(),
                FORCE_DESCENT_RAMP_MS / 1000,
            );
            // Critical log: must reach the operator regardless of
            // telemetry flooding.
            ulog::log_critical(s.as_str()).await;
        }

        // Tell `angle_to_rate_bridge` and `alt_hold::main` whether
        // Manual owns the setpoints (rate + thrust). Set every tick so
        // any state change (including unexpected exit via Fault /
        // link-loss failsafe) is reflected immediately.
        micoairh743v2::alt_hold::MANUAL_BYPASS
            .store(matches!(state, State::Manual), Ordering::Relaxed);
        // AutoHover runs the throttle as direct thrust (pilot owns vertical),
        // so alt_hold yields TRUE_Z_THRUST_SP but keeps running for the AutoLand
        // handoff. Unlike MANUAL_BYPASS this leaves the self-level controller on.
        micoairh743v2::alt_hold::AUTO_DIRECT_THRUST
            .store(matches!(state, State::AutoHover), Ordering::Relaxed);
        // The lateral velocity controller runs in the self-levelling states
        // (Auto + Landing). Outside Auto, hold the Auto command signals at zero
        // so a stale yaw-rate / velocity setpoint can never leak into another
        // state (Landing wants zero lateral velocity = hold position).
        AUTO_LATERAL_ACTIVE.store(
            matches!(state, State::AutoHover | State::AutoLand),
            Ordering::Relaxed,
        );
        if !matches!(state, State::AutoHover) {
            store_f32(&AUTO_YAW_RATE, 0.0);
            store_f32(&AUTO_LAT_STICK_FWD, 0.0);
            store_f32(&AUTO_LAT_STICK_RIGHT, 0.0);
        }

        // Sample throttle here once per tick; ground-grace + Manual reuse.
        let thr_n_for_gate = micoairh743v2::rc_kill::stick_throttle(ch.raw[2]);

        // Ground-grace gate: near-ground (low lidar) + low throttle for
        // >=200 ms expands the RC freshness timeout to absorb BEC
        // brownouts during touchdown. Tracked independently of state so
        // it works in any mode that's near ground (Manual, AutoLand,
        // post-Fault drift).
        let near_ground = lidar < GROUND_DETECT_LIDAR_M && thr_n_for_gate < GROUND_DETECT_THR_N;
        if near_ground {
            ground_grace_since.get_or_insert_with(embassy_time::Instant::now);
        } else {
            ground_grace_since = None;
        }
        let in_ground_grace = ground_grace_since
            .map(|t| t.elapsed().as_millis() as u64 >= GROUND_DETECT_DWELL_MS)
            .unwrap_or(false);
        let rc_timeout_ms = if in_ground_grace {
            RC_LINK_TIMEOUT_GROUND_MS
        } else {
            RC_LINK_TIMEOUT_AIR_MS
        };

        // RC freshness check. If seq is bumping, the link is healthy.
        // If it has not advanced for >rc_timeout_ms while armed, we
        // declare link loss and force a transition. The check is gated on
        // `armed` so a quiet ground period (operator turned TX off
        // intentionally between flights while drone is disarmed) does
        // not spuriously trip Fault.
        if ch.seq != last_seq {
            // Recovery point: if the gap leading up to this advance was
            // >= RC_NEAR_LOSS_MS but < rc_timeout_ms (so we did NOT
            // failsafe), log it once. These near-misses are the data we
            // need to know whether the link is gradually degrading or
            // dropping cleanly.
            let gap_ms = last_seq_change.elapsed().as_millis() as u64;
            if armed && gap_ms >= RC_NEAR_LOSS_MS && gap_ms <= rc_timeout_ms {
                let mut s: heapless::String<64> = heapless::String::new();
                let _ = write!(
                    s,
                    "[fsm] RC link warning: gap={}ms (recovered{})",
                    gap_ms,
                    if in_ground_grace {
                        ", ground-grace"
                    } else {
                        ""
                    },
                );
                ulog::log(s.as_str());
            }
            last_seq = ch.seq;
            last_seq_change = embassy_time::Instant::now();
        }
        if armed
            && state != State::Fault
            && last_seq_change.elapsed().as_millis() as u64 > rc_timeout_ms
        {
            let mut lost_msg: heapless::String<80> = heapless::String::new();
            let _ = write!(
                lost_msg,
                "[fsm] RC LINK LOST > {}ms while armed -- failsafe",
                rc_timeout_ms,
            );
            ulog::log(lost_msg.as_str());
            // Diagnostic dump captured BEFORE the state transition so the
            // logged state is the pre-failsafe one. Helps distinguish
            // BEC sag (motors at high command, brief impact) from clean
            // RF loss (motors steady, sudden seq stop) from FC-side seq
            // mishandling (last_seq stale across many frames).
            let dt_seq_ms = last_seq_change.elapsed().as_millis() as u64;
            let motor_speeds = match motors_rcv.try_get() {
                Some(MotorsState::Armed(s)) => s,
                _ => [0u16; 4],
            };
            let mut diag: heapless::String<128> = heapless::String::new();
            let _ = write!(
                diag,
                "[fsm] failsafe diag: dt_seq={}ms last_seq={} motors=[{},{},{},{}] state={} ground_grace={}",
                dt_seq_ms, last_seq,
                motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3],
                state.label(),
                in_ground_grace as u8,
            );
            ulog::log(diag.as_str());
            match state {
                State::Manual => {
                    // Manual = sticks were the controller; without them, no
                    // safe attitude reference. Disarm immediately and let
                    // the flip/runaway kills catch tumbling if needed.
                    COMMAD_ARM_VEHICLE.send(false);
                    zero_setpoints();
                    state = State::Fault;
                    arm_command_sent = false;
                }
                State::AutoHover => {
                    // Auto already self-levels; hand to the autonomous landing
                    // path, which holds attitude/position and descends without
                    // any RC input. att_sp_q + auto_alt_sp carry over unchanged.
                    land_floor_since = None;
                    ground_first_hit = None;
                    state = State::AutoLand;
                }
                State::AutoLand => {
                    // Already landing; let it complete (ground-detect +
                    // floor-dwell disarm need no RC link).
                }
                // AutoTakeoff is no longer entered (auto-arm goes straight to
                // Auto); group it with the no-op states for exhaustiveness.
                State::AutoTakeoff | State::GroundIdle | State::Fault => {}
            }
        }

        match state {
            State::GroundIdle => {
                // Periodic "ready" heartbeat so the operator on the BT
                // terminal sees the FC is alive and waiting, instead of
                // silence after the preflight gate passes.
                if last_ground_idle_log.elapsed().as_millis() as u64 >= GROUND_IDLE_LOG_PERIOD_MS {
                    last_ground_idle_log = embassy_time::Instant::now();
                    ulog::log("[fsm] Idle ready -- SA: Mid=Manual, High=Auto (auto-arms)");
                }

                // On-ground gate: armed must be false, lidar low, AHRS ready
                // is implicit (we don't transition into GroundIdle until
                // wait_for_ahrs_ready returned). Mode level is sampled here.
                //
                // With the MTF-01 not streaming (unplugged/dead) lidar reads a
                // permanent 99.0 (no frames published), which would pin
                // on_ground=false forever and trap the FSM in GroundIdle --
                // blocking both Manual and Auto. When the module is absent,
                // fall back to "disarmed implies on the ground" (the operator
                // launches from the ground), so arming can proceed. Detected at
                // runtime, so a refitted module restores the gate on its own.
                let on_ground = !armed
                    && (!micoairh743v2::alt_hold::lidar_is_live() || lidar < ON_GROUND_LIDAR_M);

                // SA-only scheme: mode is read from the live level, not edge
                // events. Drain RC_EVENT so the bounded queue never backs up.
                drain_rc_events();

                if !on_ground {
                    // Defensive invariant: GroundIdle means disarmed. If the
                    // motors are armed while we sit here (any path -- known
                    // one was a pre-arm completing after Manual bounced back
                    // on an SA=Auto flip inside the governor's arm window),
                    // command disarm every tick until the governor confirms.
                    // Without this, alt_hold owns the thrust setpoint while
                    // no state reads the sticks -- the D000008 dead-stick
                    // armed-idle + baro-drift liftoff.
                    if armed {
                        COMMAD_ARM_VEHICLE.send(false);
                        zero_setpoints();
                        if last_ground_idle_log.elapsed().as_millis() as u64
                            >= GROUND_IDLE_LOG_PERIOD_MS
                        {
                            last_ground_idle_log = embassy_time::Instant::now();
                            ulog::log("[fsm] GroundIdle but motors armed -- disarming (defensive)");
                        }
                    }
                    // Cannot leave Idle until physically grounded.
                    // (e.g. just-landed; lidar still settling.)
                    led::set(LedMode::Blocked);
                    Timer::after_millis(50).await;
                    continue;
                }

                // Re-allow a future auto-arm whenever SA is parked away from Auto.
                // Combined with clearing the latch on arm, this makes auto-arm
                // edge-triggered: landing back to Idle with SA still High will
                // NOT silently re-arm; the operator must cycle SA out of Auto.
                if ch.mode != Mode::Auto {
                    auto_arm_latch = true;
                }

                match ch.mode {
                    Mode::Manual => {
                        ulog::log("[fsm] Idle -> Manual");
                        state = State::Manual;
                        manual_idle_since = Some(embassy_time::Instant::now());
                        zero_setpoints();
                        // Defensive reset: armed-gate one-shots start
                        // fresh each Manual entry.
                        arm_command_sent = false;
                        arm_refused_logged = false;
                        arm_guard_wait_idle = false;
                    }
                    Mode::Auto => {
                        // LED: SA requests Auto; unless the auto-arm fires
                        // below (which overrides with Arming), one of its
                        // gates is blocking -- solid red tells the operator
                        // to check throttle/level/battery/lateral-ref.
                        led::set(LedMode::Blocked);
                        // Auto-arm straight into Auto (no trigger). Gated:
                        // edge-latch (one arm per SA->Auto), on-ground (already
                        // true here), throttle near bottom (so position-based
                        // altitude targets the ground and does not lurch on
                        // capture), drone level, battery OK.
                        let (roll_q, pitch_q, _y) = signals::AHRS_ATTITUDE_Q
                            .try_get()
                            .unwrap_or_else(UnitQuaternion::identity)
                            .euler_angles();
                        let level = roll_q.to_degrees().abs() < AUTOARM_LEVEL_DEG
                            && pitch_q.to_degrees().abs() < AUTOARM_LEVEL_DEG;
                        let throttle_low = thr_n_for_gate < AUTOARM_THROTTLE_MAX;
                        // Lateral-reference gate: Auto holds position from optical
                        // flow (needs a live MTF-01) or GPS. With neither, the
                        // lateral stick is dead and the drone drifts downwind with
                        // no pilot authority (observed 2026-06-11). Refuse to
                        // auto-arm into Auto unless a usable lateral reference
                        // exists -- a good GPS fix, or the flow module present (it
                        // provides the low-altitude hold). Manual is unaffected.
                        let gnss = common::signals::RAW_GNSS_DATA.try_get();
                        let gps_ok = gnss.as_ref().map(gnss_lateral_ok).unwrap_or(false);
                        let lateral_ref_ok = gps_ok || micoairh743v2::alt_hold::lidar_is_live();
                        if auto_arm_latch
                            && throttle_low
                            && level
                            && battery_allows_flight
                            && lateral_ref_ok
                            && !micoairh743v2::dshot_driver::MOTOR_KILL.load(Ordering::Relaxed)
                        {
                            ulog::log("[fsm] Idle -> Auto (auto-arm)");
                            led::set(LedMode::Arming);
                            auto_arm_latch = false;
                            state = State::AutoHover;
                            // Direct-thrust AutoHover; auto_alt_sp tracks the
                            // live altitude from here, seeded at the floor.
                            auto_alt_sp = ALT_FLOOR_M;
                            self_level_seed = true;
                            ground_first_hit = None;
                            land_floor_since = None;
                            zero_setpoints();
                            COMMAD_ARM_VEHICLE.send(true);
                        } else if auto_arm_latch
                            && last_ground_idle_log.elapsed().as_millis() as u64
                                >= GROUND_IDLE_LOG_PERIOD_MS
                        {
                            // Share the heartbeat cadence so this never spams.
                            last_ground_idle_log = embassy_time::Instant::now();
                            let (sats, hacc) = gnss
                                .map(|g| (g.num_satellites, g.horizontal_accuracy))
                                .unwrap_or((0, -1.0));
                            let mut s: heapless::String<128> = heapless::String::new();
                            let _ = write!(
                                s,
                                "[fsm] Auto auto-arm BLOCKED: thr_low={} level={} batt_ok={} lat_ref={} (sats={} hacc={:.1}m)",
                                throttle_low as u8,
                                level as u8,
                                battery_allows_flight as u8,
                                lateral_ref_ok as u8,
                                sats,
                                hacc,
                            );
                            ulog::log(s.as_str());
                        }
                    }
                    Mode::Idle => {
                        // Disarmed, on ground, preflight long since passed:
                        // ready to arm.
                        led::set(LedMode::Ready);
                    }
                }

                Timer::after_millis(50).await;
            }

            State::Manual => {
                drain_rc_events();

                // Seamless mode switches out of Manual (read live SA level).
                // SA=Auto: if armed, hand off to Auto with the graceful
                // attitude slew; if the pre-arm is still completing, stay in
                // Manual and hand off on the tick the governor goes live (an
                // SA Mid->High flip is one fluid motion). SA=Idle in the air
                // commits to Landing (self-levelling descent); on the ground it
                // falls through to the existing throttle+mode-idle disarm.
                if ch.mode == Mode::Auto {
                    if armed {
                        ulog::log("[fsm] Manual -> Auto");
                        // Seed + publish the attitude setpoint to the CURRENT
                        // attitude now, while MANUAL_BYPASS is still set, so when
                        // the bypass clears next tick the angle controller sees a
                        // zero-error setpoint (not the stale level one) and never
                        // commands a violent snap-to-level. The slew takes over
                        // from there. Same reason in Manual->Landing below.
                        let cur = signals::AHRS_ATTITUDE_Q
                            .try_get()
                            .unwrap_or_else(UnitQuaternion::identity);
                        att_sp_q = cur;
                        signals::TRUE_ATTITUDE_Q_SP.send(cur);
                        self_level_seed = false;
                        state = State::AutoHover;
                        auto_alt_sp = if lidar < 8.0 {
                            lidar.clamp(ALT_FLOOR_M, MISSION_CEILING_M)
                        } else {
                            micoairh743v2::alt_hold::filtered_altitude()
                                .clamp(ALT_FLOOR_M, MISSION_CEILING_M)
                        };
                        ground_first_hit = None;
                        land_floor_since = None;
                        manual_idle_since = None;
                        continue;
                    } else {
                        // SA=Auto while the pre-arm is still completing:
                        // STAY in Manual and let the arm finish; the armed
                        // branch above hands off to Auto on the tick the
                        // governor goes live. This makes SA Mid->High one
                        // fluid motion with no timing trap. The previous
                        // designs both failed the operator here: bouncing
                        // to GroundIdle without cancelling the pre-arm
                        // orphaned an arm command (D000008 dead-stick armed
                        // idle), and bouncing WITH a cancel just landed at
                        // the auto-arm gate, which indoors (no GPS fix, no
                        // flow lidar) refuses forever -- the operator kept
                        // hitting blocked-red by flipping High inside the
                        // 2.25 s window. Falling through keeps the Manual
                        // guard active (throttle >10% still aborts) and
                        // GroundIdle's defensive disarm remains the
                        // backstop for any armed-while-idle state.
                    }
                }
                if ch.mode == Mode::Idle && armed && lidar > ON_GROUND_LIDAR_M {
                    ulog::log("[fsm] Manual -> Landing (SA=Idle in air)");
                    let cur = signals::AHRS_ATTITUDE_Q
                        .try_get()
                        .unwrap_or_else(UnitQuaternion::identity);
                    att_sp_q = cur;
                    signals::TRUE_ATTITUDE_Q_SP.send(cur);
                    self_level_seed = false;
                    state = State::AutoLand;
                    // Seed the descent from the real altitude. Without lidar,
                    // Manual does not track auto_alt_sp, so use the baro-filtered
                    // altitude rather than a stale value that could floor the
                    // setpoint and cut thrust mid-air.
                    auto_alt_sp = if lidar < 8.0 {
                        lidar.clamp(ALT_FLOOR_M, MISSION_CEILING_M)
                    } else {
                        micoairh743v2::alt_hold::filtered_altitude()
                            .clamp(ALT_FLOOR_M, MISSION_CEILING_M)
                    };
                    ground_first_hit = None;
                    land_floor_since = None;
                    manual_idle_since = None;
                    continue;
                }

                // Pitch is negated to match the TX15's Liftoff-game mapping
                // (operator preference: keep one radio config that works for
                // both the sim and this drone). Roll, yaw, and throttle
                // already match drone-NED convention.
                let roll_n = micoairh743v2::rc_kill::stick_norm(ch.raw[0]);
                let pitch_n = -micoairh743v2::rc_kill::stick_norm(ch.raw[1]);
                let yaw_n = micoairh743v2::rc_kill::stick_norm(ch.raw[3]);
                let thr_n = micoairh743v2::rc_kill::stick_throttle(ch.raw[2]);

                // Throttle-high-on-arm guard: abort the arm if the stick
                // rises past ARM_GUARD_MAX_THR while the governor's arming
                // sequence is still running (armed=false). Skips the rest
                // of the tick so no live thrust setpoint is published on
                // the abort tick.
                if arm_command_sent && !armed && thr_n > ARM_GUARD_MAX_THR {
                    let mut s: heapless::String<96> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[fsm] ARM ABORT: thr={:.2} > {:.2} during arm sequence -- lower stick to idle",
                        thr_n, ARM_GUARD_MAX_THR,
                    );
                    ulog::log_critical(s.as_str()).await;
                    COMMAD_ARM_VEHICLE.send(false);
                    zero_setpoints();
                    arm_command_sent = false;
                    arm_guard_wait_idle = true;
                    continue;
                }
                if arm_guard_wait_idle && thr_n == 0.0 {
                    arm_guard_wait_idle = false;
                    ulog::log("[fsm] arm guard cleared -- throttle at idle, arming re-enabled");
                }

                // LED: armed = green solid (fly), arm sequence running =
                // green blink (hands off the throttle), guard tripped or
                // stick too high to start the arm = solid red (lower the
                // stick), otherwise blue (arm fires on the next tick).
                led::set(if armed {
                    LedMode::Armed
                } else if arm_guard_wait_idle || thr_n > ARM_GUARD_MAX_THR {
                    LedMode::Blocked
                } else if arm_command_sent {
                    LedMode::Arming
                } else {
                    LedMode::Ready
                });

                fn expo(n: f32, k: f32) -> f32 {
                    (1.0 - k) * n + k * n * n * n
                }
                let roll_rate_sp = expo(roll_n, MANUAL_EXPO) * MANUAL_MAX_ROLL_RATE;
                let pitch_rate_sp = expo(pitch_n, MANUAL_EXPO) * MANUAL_MAX_PITCH_RATE;
                let yaw_rate_sp = expo(yaw_n, MANUAL_EXPO) * MANUAL_MAX_YAW_RATE;

                // Battery-triggered forced descent overrides the thrust
                // stick: linear ramp from BASE_THRUST -> 0 over
                // FORCE_DESCENT_RAMP_MS. Roll/pitch/yaw rate authority
                // is preserved so the operator can guide the landing.
                // When the ramp completes, disarm and transition to
                // Fault to block further arming.
                let thrust_sp = if let Some(start) = force_descent_since {
                    let elapsed = start.elapsed().as_millis() as u64;
                    if elapsed >= FORCE_DESCENT_RAMP_MS {
                        // Ramp complete: disarm and transition out of
                        // Manual. Fault state is intentional -- the
                        // battery is below flight-allowing threshold;
                        // re-arming until reboot would be unsafe.
                        ulog::log("[fsm] battery-descent complete -- disarming");
                        COMMAD_ARM_VEHICLE.send(false);
                        zero_setpoints();
                        state = State::Fault;
                        arm_command_sent = false;
                        0.0
                    } else {
                        let progress = elapsed as f32 / FORCE_DESCENT_RAMP_MS as f32;
                        BASE_THRUST * (1.0 - progress)
                    }
                } else {
                    thr_n * MANUAL_THRUST_GAIN
                };

                signals::TRUE_RATE_SP.send([roll_rate_sp, pitch_rate_sp, yaw_rate_sp]);
                signals::TRUE_Z_THRUST_SP.send(thrust_sp);

                // Final post-mixer/post-lowpass DShot commands actually sent
                // to the ESCs. Used both for the periodic log field and the
                // edge-triggered saturation marker below.
                let motor_speeds = match motors_rcv.try_get() {
                    Some(MotorsState::Armed(s)) => s,
                    _ => [0u16; 4],
                };
                let mot_max = motor_speeds.iter().copied().max().unwrap_or(0);
                let sat_now = mot_max >= MANUAL_SAT_CMD;

                // Edge-triggered motor-saturation marker: log once when any
                // motor first pins at the DShot rail, and once when it backs
                // off. This pinpoints exactly when full throttle is hitting
                // the firmware/physical cap during a thrust-gain sweep.
                if sat_now != motor_saturated {
                    motor_saturated = sat_now;
                    let mut s: heapless::String<128> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[manual] motor saturation {} mot=[{},{},{},{}] thr={:.2} thrust={:.1}",
                        if sat_now { "ENTER" } else { "EXIT" },
                        motor_speeds[0],
                        motor_speeds[1],
                        motor_speeds[2],
                        motor_speeds[3],
                        thr_n,
                        thrust_sp,
                    );
                    ulog::log(s.as_str());
                }

                // 5 Hz bench-debug log: stick normals + computed rates +
                // thrust + armed. Lets the operator verify polarity and
                // amplitude with props OFF before any flight.
                if manual_log_throttle.elapsed().as_millis() as u64 >= MANUAL_LOG_PERIOD_MS {
                    manual_log_throttle = embassy_time::Instant::now();
                    let mut s: heapless::String<128> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[manual] r={:+.2} p={:+.2} y={:+.2} thr={:.2} | rate r={:+.2} p={:+.2} y={:+.2} thrust={:.1} mot_max={} mot_sat={} armed={}",
                        roll_n, pitch_n, yaw_n, thr_n,
                        roll_rate_sp, pitch_rate_sp, yaw_rate_sp, thrust_sp,
                        mot_max, sat_now as u8,
                        armed as u8,
                    );
                    ulog::log(s.as_str());
                }

                // Keep the governor armed whenever Manual is active and the
                // guard is not holding. Entering Manual pre-arms immediately
                // (the stick is at idle then), so the ~2.25 s arm window
                // runs BEFORE the operator reaches for the throttle. The
                // previous arm-on-first-throttle trigger made every normal
                // smooth throttle raise abort (D000001/D000004: 17 ARM
                // ABORTs, zero completed arms -- a thumb crosses the 10%
                // guard line ~20 ms after leaving zero). After a guard
                // abort this re-fires automatically once the stick is back
                // at idle. One-shot per attempt via arm_command_sent.
                // Battery must permit flight: tiers below HEALTHY block
                // arming until the pack is charged + the FC rebooted.
                // Never arm while the MOTOR_KILL latch is set: the keepalive
                // would stream DShot 0 regardless, producing a solid-green
                // "armed" drone whose motors can never spin (D000003 -- an
                // SE edge at boot latched the kill for the whole session).
                // The LED task renders the latch as a red strobe; SF reboots.
                if micoairh743v2::dshot_driver::MOTOR_KILL.load(Ordering::Relaxed) {
                    if !arm_refused_logged {
                        arm_refused_logged = true;
                        ulog::log_critical(
                            "[fsm] arming blocked: KILL latched -- SF (reboot) to clear",
                        )
                        .await;
                    }
                } else if !armed
                    && !arm_command_sent
                    && !arm_guard_wait_idle
                    && thr_n <= ARM_GUARD_MAX_THR
                {
                    if battery_allows_flight {
                        ulog::log("[fsm] Manual: arming -- throttle up when LED is solid green");
                        COMMAD_ARM_VEHICLE.send(true);
                        arm_command_sent = true;
                    } else if !arm_refused_logged {
                        let mut s: heapless::String<80> = heapless::String::new();
                        let _ = write!(
                            s,
                            "[fsm] arm REFUSED: battery={} -- charge pack + reboot",
                            bat_tier.label(),
                        );
                        ulog::log_critical(s.as_str()).await;
                        arm_refused_logged = true;
                    }
                }

                // Exit Manual: throttle in idle AND MODE=Idle continuously
                // for MANUAL_IDLE_DWELL_MS.
                let throttle_idle = thr_n == 0.0;
                let mode_idle = ch.mode == Mode::Idle;
                if throttle_idle && mode_idle {
                    let since = manual_idle_since.get_or_insert_with(embassy_time::Instant::now);
                    if since.elapsed().as_millis() as u64 >= MANUAL_IDLE_DWELL_MS {
                        ulog::log("[fsm] Manual -> GroundIdle (throttle+mode idle)");
                        COMMAD_ARM_VEHICLE.send(false);
                        zero_setpoints();
                        state = State::GroundIdle;
                        manual_idle_since = None;
                        arm_command_sent = false;
                    }
                } else {
                    manual_idle_since = None;
                }

                Timer::after_millis(10).await;
            }

            State::AutoTakeoff => {
                // Unreachable in the SA-only scheme (auto-arm goes straight to
                // Auto). Defensive: if somehow entered, disarm back to Idle.
                COMMAD_ARM_VEHICLE.send(false);
                zero_setpoints();
                state = State::GroundIdle;
            }

            State::AutoHover => {
                // DJI-style Auto: self-level, position-based altitude on the
                // throttle, body velocity on the right stick, yaw rate on the
                // left stick. The lateral controller produces the tilt; this
                // state slews the attitude setpoint toward (level + tilt).
                // LED: green blink while the governor is still arming
                // (auto-arm path), solid green once live.
                led::set(if armed {
                    LedMode::Armed
                } else {
                    LedMode::Arming
                });
                let cur_q = signals::AHRS_ATTITUDE_Q
                    .try_get()
                    .unwrap_or_else(UnitQuaternion::identity);

                // Graceful-handoff seed: on entry, start the attitude setpoint
                // at the measured attitude so the angle error (and commanded
                // rate) start at zero, then slew toward level below.
                if self_level_seed {
                    att_sp_q = cur_q;
                    self_level_seed = false;
                }

                // Seamless mode switches.
                if ch.mode == Mode::Manual {
                    ulog::log("[fsm] Auto -> Manual");
                    state = State::Manual;
                    manual_idle_since = Some(embassy_time::Instant::now());
                    // Already armed; do not re-arm on first throttle in Manual.
                    arm_command_sent = true;
                    arm_refused_logged = false;
                    arm_guard_wait_idle = false;
                    continue;
                }
                if ch.mode == Mode::Idle {
                    if lidar > ON_GROUND_LIDAR_M {
                        ulog::log("[fsm] Auto -> Landing (SA=Idle in air)");
                        ground_first_hit = None;
                        land_floor_since = None;
                        state = State::AutoLand;
                    } else {
                        ulog::log("[fsm] Auto -> Idle (SA=Idle on ground)");
                        COMMAD_ARM_VEHICLE.send(false);
                        zero_setpoints();
                        state = State::GroundIdle;
                        drain_rc_events();
                    }
                    continue;
                }

                // Battery-tier forced descent -> Landing (self-levelling).
                if force_descent_since.is_some() {
                    ulog::log("[fsm] Auto -> Landing (battery)");
                    ground_first_hit = None;
                    land_floor_since = None;
                    state = State::AutoLand;
                    continue;
                }

                let thr_n = micoairh743v2::rc_kill::stick_throttle(ch.raw[2]);
                let yaw_n = micoairh743v2::rc_kill::stick_norm(ch.raw[3]);
                let roll_n = micoairh743v2::rc_kill::stick_norm(ch.raw[0]);
                // Same pitch sign as Manual (Liftoff mapping): stick forward
                // commands forward motion. Polarity to verify on the bench via
                // the [lat] log before flight.
                let pitch_n = -micoairh743v2::rc_kill::stick_norm(ch.raw[1]);

                // Throttle = DIRECT thrust, identical mapping to Manual (hover
                // near mid-stick), so the pilot owns vertical and the stick
                // feels like a normal quad -- not the old position-based
                // altitude over the full 0-100 m ceiling that surprised the
                // pilot into an unexpected climb (D000510). alt_hold yields
                // TRUE_Z_THRUST_SP via AUTO_DIRECT_THRUST but keeps running.
                signals::TRUE_Z_THRUST_SP.send(thr_n * MANUAL_THRUST_GAIN);
                // Track + shadow at the live filtered altitude (zero error, so
                // alt_hold's integrator stays parked) so an AutoLand handoff
                // starts the descent from the real altitude, never a stale or
                // floored setpoint that would cut thrust mid-air.
                auto_alt_sp = micoairh743v2::alt_hold::filtered_altitude()
                    .clamp(ALT_FLOOR_M, MISSION_CEILING_M);
                ALTITUDE_SETPOINT.signal(auto_alt_sp);

                // Yaw rate (bridge injects it) + lateral stick deflection (the
                // lateral controller maps it to a direct tilt, GPS auto-braking
                // when centred).
                store_f32(&AUTO_YAW_RATE, yaw_n * AUTO_MAX_YAW_RATE);
                store_f32(&AUTO_LAT_STICK_FWD, pitch_n);
                store_f32(&AUTO_LAT_STICK_RIGHT, roll_n);

                // Compose + slew the attitude setpoint toward (level + the
                // lateral controller's tilt). Bounded slew keeps the commanded
                // rate well under the 5 rad/s kill on a Manual handoff.
                let tilt_roll = load_f32(&LAT_TILT_ROLL);
                let tilt_pitch = load_f32(&LAT_TILT_PITCH);
                slew_attitude_sp(
                    &mut att_sp_q,
                    tilt_roll,
                    tilt_pitch,
                    &cur_q,
                    ATT_SLEW_RATE,
                    AUTO_DT,
                );

                if hover_log_t.elapsed().as_millis() as u64 >= MANUAL_LOG_PERIOD_MS {
                    hover_log_t = embassy_time::Instant::now();
                    let mut s: heapless::String<112> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[fsm] auto thr={:.2} thrust={:.1} alt={:.2}m yaw={:+.2} tilt=({:+.2},{:+.2})",
                        thr_n, thr_n * MANUAL_THRUST_GAIN, auto_alt_sp,
                        yaw_n * AUTO_MAX_YAW_RATE, tilt_roll, tilt_pitch,
                    );
                    ulog::log(s.as_str());
                }

                Timer::after_millis(20).await;
            }

            State::AutoLand => {
                // Self-level the whole way down. Landing can be entered from
                // Manual (acro) too, so seed + slew the attitude setpoint here
                // exactly like Auto. AUTO_VEL_SP is held at zero (top of loop),
                // so the lateral controller damps toward zero velocity = hold.
                led::set(if armed {
                    LedMode::Armed
                } else {
                    LedMode::Arming
                });
                let cur_q = signals::AHRS_ATTITUDE_Q
                    .try_get()
                    .unwrap_or_else(UnitQuaternion::identity);
                if self_level_seed {
                    att_sp_q = cur_q;
                    self_level_seed = false;
                }
                let tilt_roll = load_f32(&LAT_TILT_ROLL);
                let tilt_pitch = load_f32(&LAT_TILT_PITCH);
                slew_attitude_sp(
                    &mut att_sp_q,
                    tilt_roll,
                    tilt_pitch,
                    &cur_q,
                    ATT_SLEW_RATE,
                    AUTO_DT,
                );

                // Controlled descent from the *current* commanded altitude.
                // Ramp the setpoint down at AUTOLAND_RATE_MS from wherever the
                // drone is -- safe from any altitude (SA=Idle in air, battery,
                // or RC-loss failsafe). alt_hold's open-loop flare
                // (setpoint < 0.20 m) cushions the final touchdown.
                const LAND_DT: f32 = 0.05; // matches the 50 ms tick below
                auto_alt_sp = (auto_alt_sp - AUTOLAND_RATE_MS * LAND_DT).max(LAND_FLOOR_M);
                ALTITUDE_SETPOINT.signal(auto_alt_sp);

                // Primary disarm: lidar confirms ground contact.
                let ground = ground_hold_elapsed(
                    lidar,
                    &mut ground_first_hit,
                    GROUND_DETECT_M,
                    GROUND_HOLD_MS,
                );

                // Both fallbacks below require the setpoint to have bottomed
                // out (we descended all the way), so neither can fire at
                // altitude -- auto_alt_sp is far above the floor there.
                let at_floor = auto_alt_sp <= LAND_FLOOR_M + 0.001;

                // Touchdown confirmation (primary fallback without lidar): the
                // descent has physically stalled at the floor, so we are down.
                // Disarms promptly at real ground contact -- |v| ~ descent rate
                // while still airborne keeps this from firing early.
                let settled = if at_floor
                    && micoairh743v2::alt_hold::vertical_speed().abs() < SETTLE_VSPEED_MPS
                {
                    let t = land_settle_since.get_or_insert_with(embassy_time::Instant::now);
                    t.elapsed().as_millis() as u64 >= SETTLE_DWELL_MS
                } else {
                    land_settle_since = None;
                    false
                };

                // Backup disarm: sat at the floor without a lidar ground-detect
                // or a confirmed touchdown for FLOOR_TIMEOUT_MS. Worst-case net
                // so the drone can never hang armed on the ground.
                let floor_timeout = if at_floor {
                    let t = land_floor_since.get_or_insert_with(embassy_time::Instant::now);
                    t.elapsed().as_millis() as u64 >= FLOOR_TIMEOUT_MS
                } else {
                    land_floor_since = None;
                    false
                };

                if ground || settled || floor_timeout {
                    let reason = if ground {
                        "ground"
                    } else if settled {
                        "settled"
                    } else {
                        "floor-timeout"
                    };
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(s, "[fsm] AutoLand: {} h={:.2}m, disarming", reason, lidar);
                    ulog::log(s.as_str());
                    log_flow_disp().await;
                    COMMAD_ARM_VEHICLE.send(false);
                    zero_setpoints();
                    ground_first_hit = None;
                    land_floor_since = None;
                    land_settle_since = None;
                    state = State::GroundIdle;
                    drain_rc_events();
                    ulog::log("[fsm] state=GroundIdle");
                    continue;
                }

                Timer::after_millis(50).await;
            }

            State::Fault => {
                led::set(LedMode::Error);
                // Terminal landing pad after a hard failsafe. Hold motors
                // disarmed and setpoints zero. Exit only when:
                //   * RC link is back AND on-ground gate clears AND mode
                //     has been parked in Idle. Then return to GroundIdle
                //     so the operator can re-arm normally without an SF
                //     reboot.
                COMMAD_ARM_VEHICLE.send(false);
                zero_setpoints();
                let link_fresh =
                    last_seq_change.elapsed().as_millis() as u64 <= RC_LINK_TIMEOUT_AIR_MS;
                let on_ground = !armed && lidar < 0.10;
                if link_fresh && on_ground && ch.mode == Mode::Idle {
                    ulog::log("[fsm] Fault -> GroundIdle (link back, OK)");
                    drain_rc_events();
                    state = State::GroundIdle;
                }
                Timer::after_millis(100).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn ahrs_to_eskf_bridge() -> ! {
    let mut rcv = signals::AHRS_ATTITUDE_Q.receiver();
    let mut snd = signals::ESKF_ESTIMATE.sender();
    loop {
        let att = rcv.changed().await;
        snd.send(EskfEstimate {
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            att,
            gyr_bias: Vector3::zeros(),
            acc_bias: Vector3::zeros(),
        });
    }
}

#[embassy_executor::task]
async fn angle_to_rate_bridge() -> ! {
    let mut rcv = signals::ANGLE_TO_RATE_SP.receiver();
    let mut snd = signals::TRUE_RATE_SP.sender();
    loop {
        CURRENT_TASK_ID.store(TASK_ANGLE_TO_RATE, Ordering::Relaxed);
        let [roll, pitch, _yaw] = rcv.changed().await;
        // While the FSM is in Manual (ACRO), the sticks own
        // TRUE_RATE_SP directly; bridging here would race. Bypass.
        // Otherwise (Auto/Landing/Idle) inject the FSM's yaw-rate command
        // (AUTO_YAW_RATE) -- the angle controller's own yaw output is
        // discarded because heading is commanded as a rate from the left
        // stick, not as an absolute heading setpoint. AUTO_YAW_RATE is held
        // at 0 by the FSM outside Auto, so this is a no-op there.
        if !micoairh743v2::alt_hold::MANUAL_BYPASS.load(Ordering::Relaxed) {
            snd.send([roll, pitch, load_f32(&AUTO_YAW_RATE)]);
        }
    }
}

#[embassy_executor::task]
async fn imu_monitor() -> ! {
    use common::tasks::controller_rate::{RATE_PID_TERMS, RATE_REF_FILTERED};

    let mut imu_rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();
    let mut mtr_rcv = signals::MOTORS_STATE.receiver();
    let mut att_rcv = signals::AHRS_ATTITUDE_Q.receiver();
    let mut pid_rcv = RATE_PID_TERMS.receiver();
    let mut ref_rcv = RATE_REF_FILTERED.receiver();
    let mut thr_rcv = signals::TRUE_Z_THRUST_SP.receiver();

    let mut cycle: u8 = 0;

    loop {
        Timer::after_millis(10).await;

        let t = embassy_time::Instant::now().as_millis();

        if cycle == 0 {
            let Some(d) = imu_rcv.try_get() else { continue };
            let (m0, m1, m2, m3) = match mtr_rcv.try_get() {
                Some(MotorsState::Armed(s)) => (s[0], s[1], s[2], s[3]),
                Some(MotorsState::Arming) => (0u16, 0, 0, 0),
                _ => (0u16, 0, 0, 0),
            };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "A,{},{:.1},{:.1},{:.1},{:.2},{:.2},{:.2},{},{},{},{}",
                t, d.acc[0], d.acc[1], d.acc[2], d.gyr[0], d.gyr[1], d.gyr[2], m0, m1, m2, m3
            );
            ulog::log(s.as_str());

            // O: auto-mode outer loop. Altitude + climb rate, commanded vs
            // optical-flow-measured horizontal velocity, and the commanded
            // attitude from the velocity controller (measured attitude is in
            // the B line). Lets us localise auto jerkiness to the altitude,
            // velocity, or angle setpoint. All reads are non-consuming
            // try_get()/accessors -- no receiver slots taken.
            let vsp = signals::TRUE_VELOCITY_SP.try_get().unwrap_or([0.0; 3]);
            let vmeas = micoairh743v2::alt_hold::FLOW_VEL_MS
                .try_get()
                .unwrap_or([0.0; 2]);
            let (asp_roll, asp_pitch) = match signals::VEL_TO_ANGLE_SP.try_get() {
                Some(q) => {
                    let (r, p, _y) = q.euler_angles();
                    (r.to_degrees(), p.to_degrees())
                }
                None => (0.0, 0.0),
            };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "O,{},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.1},{:.1}",
                t,
                micoairh743v2::alt_hold::filtered_altitude(),
                micoairh743v2::alt_hold::vertical_speed(),
                vsp[0],
                vsp[1],
                vmeas[0],
                vmeas[1],
                asp_roll,
                asp_pitch
            );
            ulog::log(s.as_str());
        } else {
            let pid = pid_rcv.try_get();
            let rsp = ref_rcv.try_get();
            let thr = thr_rcv.try_get().unwrap_or(0.0);
            let att = att_rcv.try_get();

            if let Some(pid) = pid {
                let r = rsp.unwrap_or([0.0; 3]);
                let (rd, pd, yd) = match att {
                    Some(q) => {
                        let (r, p, y) = q.euler_angles();
                        (r.to_degrees(), p.to_degrees(), y.to_degrees())
                    }
                    None => (0.0, 0.0, 0.0),
                };
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "B,{},{:.1},{:.1},{:.1},{:.2},{:.3},{:.3},{:.2},{:.3},{:.3},{:.1}",
                    t,
                    rd,
                    pd,
                    yd,
                    r[0],
                    pid[0].p_out,
                    pid[0].i_out,
                    r[1],
                    pid[1].p_out,
                    pid[1].i_out,
                    thr
                );
                ulog::log(s.as_str());

                // K: completes the inner rate-loop picture the B line omits --
                // yaw rate setpoint + yaw P/I/D, plus roll/pitch D-terms (D =
                // derivative-on-measurement + on-error). With A (measured
                // rates, motors) and B (attitude, roll/pitch P/I, thrust) this
                // gives full per-axis rate-PID observability for smoothness
                // tuning. Yaw [2] is the axis under active tuning.
                let yd_term = pid[2].dr_out + pid[2].dm_out;
                let rd_term = pid[0].dr_out + pid[0].dm_out;
                let pd_term = pid[1].dr_out + pid[1].dm_out;
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "K,{},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3}",
                    t, r[2], pid[2].p_out, pid[2].i_out, yd_term, rd_term, pd_term
                );
                ulog::log(s.as_str());
            }
        }
        cycle = 1 - cycle;
    }
}

#[embassy_executor::task]
async fn motor_monitor() -> ! {
    let mut rcv = signals::MOTORS_STATE.receiver();
    let mut last_armed_log = embassy_time::Instant::now();
    loop {
        let state = rcv.changed().await;
        let mut s: heapless::String<64> = heapless::String::new();
        match state {
            MotorsState::Disarmed(reason) => {
                let _ = write!(s, "[mtr] disarmed ({:?})", reason);
                ulog::log(s.as_str());
            }
            MotorsState::Arming => {
                ulog::log("[mtr] arming");
            }
            MotorsState::ArmedIdle => {
                ulog::log("[mtr] armed-idle");
            }
            MotorsState::Armed(sp) => {
                if last_armed_log.elapsed().as_millis() >= 2000 {
                    let _ = write!(s, "[mtr] [{},{},{},{}]", sp[0], sp[1], sp[2], sp[3]);
                    ulog::log(s.as_str());
                    last_armed_log = embassy_time::Instant::now();
                }
            }
        }
    }
}

const FLIP_KILL_ENABLED: bool = true;

#[embassy_executor::task]
async fn flip_kill() -> ! {
    if !FLIP_KILL_ENABLED {
        ulog::log("[kill] flip-kill DISABLED");
        loop {
            Timer::after_secs(60).await;
        }
    }

    ulog::log("[kill] flip-kill active");

    let mut rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();

    const FLIP_COUNT_THRESHOLD: u32 = 10;
    // NED body frame (post override_imu_rot): level stationary drone reads
    // az = -g, fully inverted drone reads az = +g. Detect inverted by az
    // exceeding a positive threshold. The old `< -3.0` check was correct
    // for the previous chip-Z-up convention but fires immediately on boot
    // under NED (level az = -9.8 which passes the old threshold).
    const AZ_INVERTED_THRESHOLD: f32 = 3.0;

    let mut inverted_count: u32 = 0;

    loop {
        let d = rcv.changed().await;

        if d.acc[2] > AZ_INVERTED_THRESHOLD {
            inverted_count += 1;
            if inverted_count >= FLIP_COUNT_THRESHOLD {
                // Latch first (unblockable), then the normal disarm path.
                micoairh743v2::dshot_driver::MOTOR_KILL.store(true, Ordering::Relaxed);
                COMMAD_ARM_VEHICLE.send(false);
                ulog::log("[kill] FLIP DETECTED -- motors disarmed");

                for _ in 0..5 {
                    Timer::after_millis(200).await;
                    ulog::log("[kill] motors off (flip-kill)");
                }

                loop {
                    Timer::after_secs(60).await;
                }
            }
        } else {
            inverted_count = 0;
        }
    }
}

#[embassy_executor::task]
async fn gyro_runaway_kill() -> ! {
    ulog::log("[kill] gyro-runaway autoabort active");

    let mut rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();

    const GYRO_RUNAWAY_THRESHOLD: f32 = 5.0;
    const GYRO_RUNAWAY_COUNT: u32 = 50;

    let mut count: u32 = 0;

    loop {
        let d = rcv.changed().await;
        let gmax = d.gyr[0].abs().max(d.gyr[1].abs()).max(d.gyr[2].abs());

        if gmax > GYRO_RUNAWAY_THRESHOLD {
            count += 1;
            if count >= GYRO_RUNAWAY_COUNT {
                // Latch first (unblockable), then the normal disarm path.
                micoairh743v2::dshot_driver::MOTOR_KILL.store(true, Ordering::Relaxed);
                COMMAD_ARM_VEHICLE.send(false);
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "[kill] GYRO RUNAWAY gx={:.1} gy={:.1} gz={:.1} -- motors disarmed",
                    d.gyr[0], d.gyr[1], d.gyr[2]
                );
                ulog::log(s.as_str());
                for _ in 0..5 {
                    Timer::after_millis(200).await;
                    ulog::log("[kill] motors off (gyro-runaway)");
                }
                loop {
                    Timer::after_secs(60).await;
                }
            }
        } else {
            count = 0;
        }
    }
}

/// GPS quality thresholds for a usable horizontal hold. Shared by the lateral
/// controller (which consumes the fix) and the Auto auto-arm gate (which
/// refuses to arm without one), so "armable into Auto" always implies the
/// lateral controller will actually accept the fix -- no arm-then-no-hold gap.
const GNSS_MIN_SATS: u8 = 8;
const GNSS_HACC_MAX_M: f32 = 5.0;

/// True when the GNSS fix is good enough to drive the lateral velocity hold:
/// a 3D fix, enough satellites, and a tight, positive horizontal accuracy.
fn gnss_lateral_ok(g: &common::types::measurements::GnssData) -> bool {
    use common::types::measurements::GnssFix;
    matches!(g.fix, GnssFix::Fix3D)
        && g.num_satellites >= GNSS_MIN_SATS
        && g.horizontal_accuracy > 0.0
        && g.horizontal_accuracy < GNSS_HACC_MAX_M
}

/// Lateral controller for Auto/Landing: direct angle stick + GPS auto-brake.
/// Reads the lateral stick deflection (`AUTO_LAT_STICK_*`, written by the FSM)
/// and publishes a tilt command (`LAT_TILT_*`) that the FSM folds into
/// TRUE_ATTITUDE_Q_SP. It never writes the attitude setpoint itself, so there
/// is a single attitude writer (the FSM) at all times.
///
/// Two superimposed terms:
///   - Direct angle: stick deflection maps straight to a tilt up to MAX_TILT_RAD
///     (full pilot authority to fly / fight wind, like the now-direct throttle).
///   - GPS auto-brake: velocity damping toward zero, faded out as the stick
///     deflects (weight 1-|stick|), so full stick = pure direct authority and
///     centred sticks = full brake (drift hold). With the stick centred this is
///     EXACTLY the validated flow_hold damping law (pitch = +KP_V*vmeas_fwd,
///     roll = -KP_V*vmeas_right), so the proven sign convention is preserved.
///
/// Velocity feedback source, selected by altitude:
///   - low (lidar in range, < FLOW_MAX_M): MTF-01 optical flow body velocity
///   - high (above flow range): GPS NED velocity rotated to body via AHRS yaw
///   - neither available: direct-only (no brake), like manual angle mode
///
/// D000518 showed the old velocity-tracking controller saturated at 9.7 deg and
/// could not fight wind; this gives the pilot real authority (MAX_TILT_RAD) and
/// stronger braking. NOTE: lateral stick polarity is still bench-unverified --
/// confirm via the [lat] log before flight. True position-lock (return to a
/// held point) is the D000145 gyro-runaway risk and is intentionally not here.
#[embassy_executor::task]
async fn lateral_controller() -> ! {
    use common::types::actuators::MotorsState;

    // Auto-brake velocity-damping gain (rad per m/s); 0.10 is the validated
    // flow_hold value. Clamped to MAX_TILT_RAD, so braking now reaches the full
    // bank instead of the old 9.7 deg cap that could not arrest an outdoor
    // drift (D000518).
    const KP_V: f32 = 0.10;
    // Max bank the pilot can command (and the brake can use). ~25 deg gives
    // g*tan(25) ~ 4.6 m/s^2 of horizontal authority -- enough to fight wind.
    // Bench/flight-tunable; raise for more authority, lower if it feels twitchy.
    const MAX_TILT_RAD: f32 = 0.44; // ~25 deg
    const FLOW_MAX_M: f32 = 7.0; // use flow only while lidar is solidly in range
    const MAX_FLOW_VEL: f32 = 1.5; // reject flow spikes above this (m/s)
                                   // Position hold: outer-P gain maps position error (m) to a velocity setpoint
                                   // (m/s), clamped to MAX_HOLD_VEL. That setpoint is then fed into the existing
                                   // velocity damping loop as the reference, giving a cascaded P-P position hold
                                   // that eliminates steady-state wind/CoM drift without a true integrator.
    const KP_POS: f32 = 0.5; // (m/s) / m
    const MAX_HOLD_VEL: f32 = 1.0; // m/s -- cap on position-derived vel setpoint
    const STICK_CENTER_THRESH: f32 = 0.05; // deadband for "sticks centred"

    let mut log_div: u8 = 0;
    // Captured GPS position target (lat_raw, lon_raw in 1e-7 deg). Set when
    // sticks centre with a good fix; cleared on stick movement or GPS loss.
    let mut pos_target: Option<(i32, i32)> = None;

    loop {
        Timer::after_millis(20).await; // 50 Hz

        let active = AUTO_LATERAL_ACTIVE.load(Ordering::Relaxed);
        let armed = common::signals::MOTORS_STATE
            .try_get()
            .map(|m| matches!(m, MotorsState::Armed(_) | MotorsState::ArmedIdle))
            .unwrap_or(false);
        if !active || !armed {
            store_f32(&LAT_TILT_ROLL, 0.0);
            store_f32(&LAT_TILT_PITCH, 0.0);
            pos_target = None;
            continue;
        }

        let stick_fwd = load_f32(&AUTO_LAT_STICK_FWD);
        let stick_right = load_f32(&AUTO_LAT_STICK_RIGHT);

        // Pick a velocity feedback source by altitude. Flow is only trusted
        // between 0.25 m and FLOW_MAX_M: below 0.25 m the MTF-01 reader stops
        // publishing (FLOW_VEL_MS goes stale), and acting on that stale value on
        // the ground produced phantom tilt jitter (D000490). Mirror the reader's
        // 0.25 m gate so the ground/landing regime is pure self-level.
        let lidar = micoairh743v2::alt_hold::LIDAR_ALT_M.try_get();
        let flow_in_range = lidar
            .map(|h| (0.25..FLOW_MAX_M).contains(&h))
            .unwrap_or(false);
        let fq = FLOW_QUALITY.load(Ordering::Relaxed);

        let mut src = 'n'; // 'f' flow, 'g' gps, 'n' none
        let mut vmeas_fwd = 0.0_f32;
        let mut vmeas_right = 0.0_f32;

        if flow_in_range && fq >= 50 {
            if let Some([vx, vy]) = micoairh743v2::alt_hold::FLOW_VEL_MS.try_get() {
                if vx.abs() <= MAX_FLOW_VEL && vy.abs() <= MAX_FLOW_VEL {
                    vmeas_fwd = vx;
                    vmeas_right = vy;
                    src = 'f';
                }
            }
        }

        // Fetch GPS and yaw once; both are needed for velocity rotation and
        // for the position hold correction below.
        let gnss_data = common::signals::RAW_GNSS_DATA.try_get();
        let yaw = signals::AHRS_ATTITUDE_Q
            .try_get()
            .map(|q| q.euler_angles().2)
            .unwrap_or(0.0);
        let (s_yaw, c_yaw) = (libm::sinf(yaw), libm::cosf(yaw));

        if src == 'n' {
            // Try GPS (rotate NED velocity into body via AHRS yaw).
            if let Some(g) = gnss_data {
                if gnss_lateral_ok(&g) {
                    vmeas_fwd = g.velocity_north * c_yaw + g.velocity_east * s_yaw;
                    vmeas_right = -g.velocity_north * s_yaw + g.velocity_east * c_yaw;
                    src = 'g';
                }
            }
        }

        // Position hold: reset target on any stick deflection or GPS loss so the
        // drone re-captures its position when the pilot releases the stick.
        let sticks_centered =
            stick_fwd.abs() < STICK_CENTER_THRESH && stick_right.abs() < STICK_CENTER_THRESH;
        if src != 'g' || !sticks_centered {
            pos_target = None;
        }
        if src == 'g' && sticks_centered && pos_target.is_none() {
            if let Some(g) = gnss_data {
                pos_target = Some((g.latitude_raw, g.longitude_raw));
            }
        }

        // Convert position error to a velocity setpoint (outer-P loop).
        // 1e-7 deg * 111_111 m/deg = m per raw unit of lat/lon.
        // Longitude metres are additionally scaled by cos(lat).
        let (vel_sp_fwd, vel_sp_right) = match pos_target {
            Some((lat_t, lon_t)) => {
                if let Some(g) = gnss_data {
                    let lat_err_m = (lat_t - g.latitude_raw) as f32 * (1e-7 * 111_111.0);
                    let cos_lat =
                        libm::cosf(g.latitude_raw as f32 * (1e-7 * core::f32::consts::PI / 180.0));
                    let lon_err_m = (lon_t - g.longitude_raw) as f32 * (1e-7 * 111_111.0 * cos_lat);
                    // NED error rotated to body frame (same transform as vmeas).
                    let sp_fwd = (KP_POS * (lat_err_m * c_yaw + lon_err_m * s_yaw))
                        .clamp(-MAX_HOLD_VEL, MAX_HOLD_VEL);
                    let sp_right = (KP_POS * (-lat_err_m * s_yaw + lon_err_m * c_yaw))
                        .clamp(-MAX_HOLD_VEL, MAX_HOLD_VEL);
                    (sp_fwd, sp_right)
                } else {
                    (0.0, 0.0)
                }
            }
            None => (0.0, 0.0),
        };

        // Direct angle from the stick: full pilot authority, no velocity
        // reference needed. Pitch sign was FIELD-CORRECTED after the first
        // flight of the direct-angle controller flew pitch inverted (forward
        // stick drove the drone backward); roll flew correct and is unchanged.
        // The auto-brake sign below is independently correct (it opposed the
        // drift in D000518), so flipping only the direct term keeps the two
        // consistent: a forward command and a forward-drift brake now tilt
        // opposite ways (accelerate vs decelerate).
        let pitch_direct = stick_fwd * MAX_TILT_RAD;
        let roll_direct = stick_right * MAX_TILT_RAD;

        // GPS/flow auto-brake: velocity damping toward zero, faded out as the
        // stick deflects so full stick = pure direct authority and centred =
        // full brake. Both signs are -KP*vmeas (symmetric with the +stick*MAX
        // direct terms): D000547 proved the old pitch brake (+KP*vmeas_fwd) was
        // positive feedback -- centred sticks accelerated the forward drift --
        // because the pitch axis is inverted vs roll (the pitch_n negation
        // upstream). Roll was always correct. Skipped with no velocity
        // reference (src='n') -> direct-only, like manual angle mode.
        // Brake on velocity error vs the position-hold setpoint. When no target
        // is held (vel_sp = 0) this reduces exactly to the original velocity
        // damper. When a target is held the drone drives velocity toward vel_sp,
        // which in turn drives position toward the captured point.
        let (pitch_brake, roll_brake) = if src != 'n' {
            (
                -KP_V * (vmeas_fwd - vel_sp_fwd),
                -KP_V * (vmeas_right - vel_sp_right),
            )
        } else {
            (0.0, 0.0)
        };
        let w_fwd = 1.0 - stick_fwd.abs().min(1.0);
        let w_right = 1.0 - stick_right.abs().min(1.0);
        let pitch_cmd = (pitch_direct + w_fwd * pitch_brake).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll_cmd = (roll_direct + w_right * roll_brake).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        store_f32(&LAT_TILT_ROLL, roll_cmd);
        store_f32(&LAT_TILT_PITCH, pitch_cmd);

        log_div = log_div.wrapping_add(1);
        if log_div >= 10 {
            log_div = 0;
            let mut s: heapless::String<112> = heapless::String::new();
            let _ = write!(
                s,
                "[lat] src={} hold={} stk=({:+.2},{:+.2}) vm=({:+.2},{:+.2}) sp=({:+.2},{:+.2}) p={:+.2} r={:+.2}",
                src,
                if pos_target.is_some() { 'y' } else { 'n' },
                stick_fwd, stick_right,
                vmeas_fwd, vmeas_right,
                vel_sp_fwd, vel_sp_right,
                pitch_cmd, roll_cmd
            );
            ulog::log(s.as_str());
        }
    }
}

/// Integrate flow-reported body-frame velocity into a displacement estimate,
/// independent of `flow_hold` so we don't clamp at +/-0.5 m. Pure telemetry.
/// Uses actual elapsed time between samples rather than a nominal DT, so
/// dropped flow frames don't bias the integration.
#[embassy_executor::task]
async fn flow_position_logger() -> ! {
    let mut rcv = micoairh743v2::alt_hold::FLOW_VEL_MS.receiver().unwrap();
    let mut est_x = 0.0_f32;
    let mut est_y = 0.0_f32;
    let mut last_t: Option<embassy_time::Instant> = None;
    loop {
        let [vx, vy] = rcv.changed().await;
        let now = embassy_time::Instant::now();
        if let Some(prev) = last_t {
            // `Instant - Instant` panics on underflow. Use checked variant:
            // although `Instant::now()` is monotonic in principle, the H743
            // TIM2 time driver has rarely been observed to return a value
            // briefly less than a previously-read one (likely a tearing /
            // overflow-IRQ race in the 32->64-bit extension). A None here
            // just skips this integration step; integration resumes next tick.
            if let Some(dur) = now.checked_duration_since(prev) {
                let dt = dur.as_micros() as f32 / 1_000_000.0;
                // Sanity: reject absurd gaps (task paused, first sample
                // after a long idle, etc.) to keep the integration honest.
                if (0.001..0.5).contains(&dt) {
                    est_x += vx * dt;
                    est_y += vy * dt;
                    FLOW_EST_X_MM.store((est_x * 1000.0) as i32, Ordering::Relaxed);
                    FLOW_EST_Y_MM.store((est_y * 1000.0) as i32, Ordering::Relaxed);
                }
            }
        }
        last_t = Some(now);
    }
}

/// Log the flow-integrated landing displacement (dead-reckoned distance from
/// takeoff to touchdown) just before an AutoLand disarm. Emitted on the
/// critical channel because the regular log's 5-message flush policy would
/// otherwise leave this final line in the pending bucket when the operator
/// yanks the LiPo right after touchdown -- exactly the loss seen in
/// D000079-D000082. Reads the statics `flow_position_logger` integrates.
async fn log_flow_disp() {
    let dx = FLOW_EST_X_MM.load(Ordering::Relaxed) as f32 / 1000.0;
    let dy = FLOW_EST_Y_MM.load(Ordering::Relaxed) as f32 / 1000.0;
    let dist = libm::sqrtf(dx * dx + dy * dy);
    let mut s: heapless::String<96> = heapless::String::new();
    let _ = write!(
        s,
        "[fsm] flow_disp dx={:.2}m dy={:.2}m |d|={:.2}m (body frame, fwd/right)",
        dx, dy, dist
    );
    ulog::log_critical(s.as_str()).await;
}

/// Compute a tilt-compensated magnetic heading and log it alongside the
/// Madgwick-derived gyro yaw. Pure read-only telemetry tap: does not feed
/// into any controller. The mag is already calibrated (bias + soft-iron
/// matrix applied in `resources::compass_reader`). The `diff` column is
/// the quantity to watch: if it stays roughly constant over a flight, the
/// gyro-based yaw is not drifting. If it walks, we have empirical
/// quantification of gyro-yaw drift rate.
///
/// Field magnitude |B| is logged as a sanity check. Earth's field indoors
/// typically reads 25-60 uT depending on latitude and shielding; if |B|
/// jumps with throttle that's motor-current interference on the mag.
#[embassy_executor::task]
async fn mag_yaw_logger() -> ! {
    // Bug fix 2026-04-23: this task was originally reading CAL_MAG_DATA,
    // which is declared in common::signals but nothing writes to it on this
    // board. The device-side compass_reader publishes the calibrated mag
    // vector to CAL_MULTI_MAG_DATA[0] (a multi_watch of [f32;3]). That's
    // what we actually need to subscribe to. D000079-D000082 produced zero
    // [mag] log lines because of this mismatch.
    let mut mag_rcv = common::signals::CAL_MULTI_MAG_DATA[0].receiver();
    let mut att_rcv = signals::AHRS_ATTITUDE_Q.receiver();
    loop {
        Timer::after_millis(500).await;

        let Some(mag) = mag_rcv.try_get() else {
            continue;
        };
        let Some(q) = att_rcv.try_get() else { continue };

        let (roll, pitch, yaw_gyro_rad) = q.euler_angles();
        let yaw_gyro = yaw_gyro_rad.to_degrees();

        let mx = mag[0];
        let my = mag[1];
        let mz = mag[2];
        let b_mag = libm::sqrtf(mx * mx + my * my + mz * mz);

        // Tilt-compensated heading for body frame (x-forward, y-right,
        // z-down). Standard formula (see Honeywell AN-203):
        //   Xh = mx*cos(p) + my*sin(r)*sin(p) + mz*cos(r)*sin(p)
        //   Yh = my*cos(r) - mz*sin(r)
        //   yaw = atan2(-Yh, Xh)
        // atan2(-Yh, Xh) returns yaw positive CW viewed from above, matching
        // NED body-frame yaw convention. If the drone is actually aligned
        // with magnetic north at arm time, yaw_mag == 0.
        let cr = libm::cosf(roll);
        let sr = libm::sinf(roll);
        let cp = libm::cosf(pitch);
        let sp = libm::sinf(pitch);
        let xh = mx * cp + my * sr * sp + mz * cr * sp;
        let yh = my * cr - mz * sr;
        let yaw_mag = libm::atan2f(-yh, xh).to_degrees();

        // Wrap diff to (-180, 180] so growing drift is readable as a
        // continuously-changing value rather than a ±360 jump.
        let mut diff = yaw_mag - yaw_gyro;
        while diff > 180.0 {
            diff -= 360.0;
        }
        while diff < -180.0 {
            diff += 360.0;
        }

        // NOTE: b_mag is the norm of the calibrated mag vector. Because the
        // soft-iron matrix (MAG_CAL_MAT in resources.rs) normalizes output
        // to the unit sphere, b_mag is dimensionless and should read ~1.0
        // on a well-calibrated sensor. Labelled |B|norm to avoid the earlier
        // "uT" mis-label: a value like 0.8 means the sample is 20% off the
        // cal sphere (mild residual distortion or motor-EMI shift), not
        // that the field magnitude is 0.8 uT.
        let mut s: heapless::String<96> = heapless::String::new();
        let _ = write!(
            s,
            "[mag] yaw_mag={:.1} yaw_gyro={:.1} diff={:.1} |B|norm={:.2}",
            yaw_mag, yaw_gyro, diff, b_mag
        );
        ulog::log(s.as_str());
    }
}

/// Passive second-IMU logger. Reads the BMI270 on SPI3 at 100 Hz and emits
/// `C,t,ax,ay,az,gx,gy,gz` rows (same units/format as the BMI088 `A` rows
/// from imu_monitor). Does not publish to any shared signal, does not feed
/// the attitude estimator, does not influence control. Purpose: build up
/// a side-by-side dataset comparing BMI088 and BMI270 so we can later
/// decide whether to fuse the two or use BMI270 as a BMI088 fallback.
///
/// Notes:
/// - BMI270 init uploads a ~8 KB config blob over SPI; takes ~200 ms.
///   The task logs `[bmi270] init OK` on success or `init FAILED` and
///   parks itself on failure (no retry loop, since the binary keeps
///   flying on BMI088 regardless).
/// - Units match BMI088: m/s^2 and rad/s. Scales from datasheet: accel
///   +-2g -> 16384 LSB/g; gyro +-2000 dps -> 16.384 LSB/dps.
/// - Currently reads ONLY BMI270 -- no chip-to-drone rotation applied,
///   no calibration bias removal. The raw comparison is what we want
///   for the "could we trust it?" question; cal can come later if we
///   decide to use it.
#[embassy_executor::task]
async fn bmi270_logger_task(r: Bmi270Resources) -> ! {
    use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
    use embassy_stm32::mode::Async;
    use embassy_stm32::spi::{mode::Master, Config as SpiConfig, Spi};
    use embassy_stm32::time::Hertz;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::mutex::Mutex;
    use micoairh743v2::bmi270::{Bmi270, BMI270_CONFIG_FILE};
    use static_cell::StaticCell;

    type Spi3Bus = Mutex<NoopRawMutex, Spi<'static, Async, Master>>;
    static SPI3_BUS: StaticCell<Spi3Bus> = StaticCell::new();

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = Hertz(8_000_000);
    spi_cfg.mode = embassy_stm32::spi::MODE_3;
    spi_cfg.miso_pull = embassy_stm32::gpio::Pull::Up;

    let spi = Spi::new(
        r.spi,
        r.sclk,
        r.mosi,
        r.miso,
        r.dma_tx,
        r.dma_rx,
        resources::Spi3Irqs,
        spi_cfg,
    );

    let bus = SPI3_BUS.init(Mutex::new(spi));
    let cs = Output::new(r.cs, Level::High, Speed::High);
    let spi_dev = SpiDeviceWithConfig::new(bus, cs, spi_cfg);

    let mut imu = Bmi270::new(spi_dev);

    ulog::log("[bmi270] initializing (config blob ~200 ms)...");
    match imu.init(&BMI270_CONFIG_FILE).await {
        Ok(()) => {
            ulog::log("[bmi270] init OK");
            ulog::SENSORS_READY.fetch_or(
                ulog::SENSOR_BMI270_BIT,
                core::sync::atomic::Ordering::Release,
            );
        }
        Err(_) => {
            ulog::log("[bmi270] init FAILED -- logger disabled");
            loop {
                Timer::after_secs(60).await;
            }
        }
    }

    // Pin ranges explicitly so the ACC_SCALE / GYR_SCALE constants below
    // are guaranteed correct. The config blob defaults to ±8g accel, which
    // makes the stationary-az reading come out at ~2.4 instead of ~9.8
    // and is the cause of the BMI088-vs-BMI270 factor-of-4 scale mismatch
    // seen in D000087-090. See bmi270::set_acc_range for the full context.
    imu.set_acc_range(0x00).await.ok(); // +/-2g  -> 1 g = 16384 LSB
    imu.set_gyr_range(0x00).await.ok(); // +/-2000 dps (explicit)

    // Scale: accel +-2g range, 16384 LSB/g -> g per LSB = 1/16384
    // Gyro: +-2000 dps, 16.384 LSB/dps, wrapped to rad/s.
    const ACC_SCALE_MS2: f32 = 9.80665 / 16384.0;
    const GYR_SCALE_RADS: f32 = (2000.0_f32 * core::f32::consts::PI / 180.0) / 32768.0;

    // BMI270 is mounted rotated 180 deg around its X axis relative to the
    // BMI088 / drone body frame. Empirical observation in D000086: roll
    // rates (gx) match across the two IMUs, pitch (gy) and yaw (gz) are
    // inverted. Applying [+1, -1, -1] to BMI270 readings brings its C-row
    // logs into the same NED body frame as the A-row BMI088 logs, so
    // side-by-side comparison is apples-to-apples.
    const BMI270_CHIP_TO_DRONE: [f32; 3] = [1.0, -1.0, -1.0];

    // Publish frame-aligned samples on CAL_MULTI_IMU_DATA[1] so consumers
    // like `phoenix_telem` (SCALED_IMU2 -> Pi /fc/imu/1) can see this IMU.
    // Note: there is no separate cal stage for BMI270 in this firmware; the
    // raw chip output (with axis rotation applied) is published as "cal" so
    // downstream code can treat both IMU slots uniformly.
    let mut snd_imu1 = signals::CAL_MULTI_IMU_DATA[1].sender();

    loop {
        Timer::after_millis(10).await; // 100 Hz logging cadence
        match imu.read().await {
            Ok(d) => {
                let t = embassy_time::Instant::now().as_millis();
                let ax = d.accel.x as f32 * ACC_SCALE_MS2 * BMI270_CHIP_TO_DRONE[0];
                let ay = d.accel.y as f32 * ACC_SCALE_MS2 * BMI270_CHIP_TO_DRONE[1];
                let az = d.accel.z as f32 * ACC_SCALE_MS2 * BMI270_CHIP_TO_DRONE[2];
                let gx = d.gyro.x as f32 * GYR_SCALE_RADS * BMI270_CHIP_TO_DRONE[0];
                let gy = d.gyro.y as f32 * GYR_SCALE_RADS * BMI270_CHIP_TO_DRONE[1];
                let gz = d.gyro.z as f32 * GYR_SCALE_RADS * BMI270_CHIP_TO_DRONE[2];
                snd_imu1.send(common::types::measurements::Imu6DofData {
                    timestamp_us: embassy_time::Instant::now().as_micros(),
                    acc: [ax, ay, az],
                    gyr: [gx, gy, gz],
                });
                let mut s: heapless::String<96> = heapless::String::new();
                let _ = write!(
                    s,
                    "C,{},{:.1},{:.1},{:.1},{:.2},{:.2},{:.2}",
                    t, ax, ay, az, gx, gy, gz
                );
                ulog::log(s.as_str());
            }
            Err(_) => {
                // Swallow silently; BMI270 occasionally returns bad SPI on
                // contention. Logging each error would spam. If reliability
                // becomes a concern we can count + periodically report.
            }
        }
    }
}

#[embassy_executor::task]
async fn mtf01_reader_task(r: Mtf01Resources) -> ! {
    use embassy_stm32::usart::{Config as UartConfig, UartRx};

    let mut cfg = UartConfig::default();
    cfg.baudrate = 115_200;
    let mut uart = UartRx::new(r.usart, r.rx, r.dma, SensorIrqs, cfg).unwrap();

    ulog::log("[mtf01] UART4 RX started");

    let snd_lidar = micoairh743v2::alt_hold::LIDAR_ALT_M.sender();
    let snd_flow = micoairh743v2::alt_hold::FLOW_VEL_MS.sender();
    // Gyro receiver for body-rotation compensation of optical flow.
    // The MTF-01 is a downward-looking camera: when the drone rotates, the
    // ground image shifts even if the drone is stationary, producing false
    // translation reports. The shift scales as omega * h (angular rate times
    // altitude), so we subtract that term from the flow velocity before
    // publishing to FLOW_VEL_MS. This removes the feedback path that created
    // altitude-dependent oscillation in D000145/147 and the drift bias
    // suggested by D000084's ~1 deg/s Madgwick yaw drift.
    let mut gyro_rcv = signals::RAW_MULTI_IMU_DATA[0].receiver();
    let mut last_height_m: f32 = 0.0;

    let mut b = [0u8; 1];
    let mut hdr = [0u8; 6];
    let mut pbuf = [0u8; mtf01::MAX_PAYLOAD + 1];

    loop {
        loop {
            uart.read(&mut b).await.ok();
            if b[0] != b'$' {
                continue;
            }
            uart.read(&mut b).await.ok();
            if b[0] == b'X' {
                break;
            }
        }

        uart.read(&mut hdr).await.ok();
        let size = u16::from_le_bytes([hdr[4], hdr[5]]) as usize;
        if size > mtf01::MAX_PAYLOAD {
            continue;
        }

        uart.read(&mut pbuf[..size + 1]).await.ok();

        match mtf01::parse(hdr, &pbuf[..size + 1]) {
            Some(mtf01::Frame::Lidar(l)) => {
                if l.quality > 0 && l.distance_mm >= 0 {
                    let h = l.distance_mm as f32 / 1000.0;
                    snd_lidar.send(h);
                    last_height_m = h;
                    // Stamp liveness so alt_hold / the FSM can auto-detect a
                    // missing module (no frames -> baro-only + ground-gate bypass).
                    micoairh743v2::alt_hold::note_lidar_frame();
                }
            }
            Some(mtf01::Frame::Flow(f)) => {
                FLOW_QUALITY.store(f.quality, Ordering::Relaxed);
                // Gate on 25 cm AND quality >= 60.
                // Raised 10 cm -> 25 cm after D000026 flipped the drone at
                // ~20 cm during descent: flow_hold's tilt commands (+/-4-6
                // deg from velocity-induced-by-body-rotation noise) destabi-
                // lise the drone in the final landing approach, where the
                // drone cannot tolerate any lateral excursion without tip-
                // ping into a prop-down crash. Silencing flow below 25 cm
                // means the last metre of descent is attitude-only control,
                // which is stable enough to land without wobbling. The 25 cm
                // floor is also above the typical lidar minimum-range stuck
                // regime (D000009), so flow genuinely stops when descent
                // begins rather than receiving noisy data near ground.
                // Quality >= 60 threshold unchanged from D000136 fix.
                if f.quality > 60 && last_height_m > 0.25 {
                    const FLOW_SCALE: f32 = 0.25;
                    let vx_raw = f.motion_y as f32 * FLOW_SCALE * last_height_m;
                    let vy_raw = -(f.motion_x as f32) * FLOW_SCALE * last_height_m;

                    // Gyro compensation: a downward-looking flow sensor on
                    // a rotating drone sees an apparent translation of
                    // omega * h even when the drone is stationary. For NED
                    // body frame (x-fwd, y-right, z-down) with pitch rate
                    // omega_y positive = nose-up and roll rate omega_x
                    // positive = right-wing-down, the correction is:
                    //   vx = vx_raw - omega_y * h
                    //   vy = vy_raw + omega_x * h
                    // Signs are empirically verified-to-first-approximation;
                    // if the first post-fix flight shows drift INCREASING
                    // under flow_hold active, flip both signs.
                    let (omega_x, omega_y) = match gyro_rcv.try_get() {
                        Some(d) => (d.gyr[0], d.gyr[1]),
                        None => (0.0, 0.0),
                    };
                    let vx = vx_raw - omega_y * last_height_m;
                    let vy = vy_raw + omega_x * last_height_m;
                    snd_flow.send([vx, vy]);
                }
            }
            None => {}
        }
    }
}

// =============================================================================
// HIL Phase 1 -- IMU injection over the link (see references/hil_implementation_plan.md).
//
// Active only in HIL mode (USB power at boot, see main). hil_link_task owns
// USART1 (the same wire uart_writer_task mirrors text logs to in flight
// mode; main routes the resource to exactly one of them) and speaks a small
// fixed-size binary protocol: it decodes inbound SensorFrames into
// Imu6DofData and pushes them into HIL_IMU_CHAN, and replies with the
// latest AHRS_ATTITUDE_Q as an AttitudeFrame. hil_imu_task feeds that
// channel into common::tasks::imu_reader::main_6dof via the HilImu adapter,
// exactly as the real BMI088 reader would.
// =============================================================================

/// Link baud rate. NOTE: the "2,000,000 baud" Phase 0 result in
/// tools/hil_link_test_result_h743v2.md is not trustworthy -- hil_echo.rs
/// passed UartConfig::default() unmodified, and embassy-stm32's default
/// baudrate is 115200 (see embassy-stm32/src/usart/mod.rs), so that run
/// actually had the host at 2M talking to firmware pinned at 115200. The
/// ~54% loss it measured was a baud mismatch artefact, not a real 2M
/// link result -- and Phase 1 (which does set this field correctly) saw
/// 100% loss at a genuine, matched 2M, so 2M is unproven on this
/// adapter/cable. 115200 is the only rate with a real matched-baud
/// measurement (n=49941/50000, ~99.9%) behind it, and lockstep doesn't
/// need more than that -- use it until there's a reason to revisit.
const HIL_LINK_BAUD: u32 = 115_200;

/// Host -> FC sensor frame (Phase 1: IMU only). Fixed 32 bytes, all
/// multi-byte fields little-endian:
///   [0]       sync  = 0xA5
///   [1]       type  = 0x01
///   [2..6)    seq   : u32   -- host-side monotonic frame counter, echoed back
///   [6..18)   acc   : f32x3 -- m/s^2, drone body frame
///   [18..30)  gyr   : f32x3 -- rad/s, drone body frame
///   [30..32)  crc16 : u16   -- CRC-16/CCITT-FALSE over bytes [0..30)
const SENSOR_FRAME_LEN: usize = 32;
const SENSOR_FRAME_SYNC: u8 = 0xA5;
const SENSOR_FRAME_TYPE: u8 = 0x01;

/// FC -> Host attitude debug frame. Fixed 25 bytes:
///   [0]       sync  = 0x5A
///   [1]       type  = 0x02
///   [2..6)    seq   : u32   -- echoes the sensor frame that produced this estimate
///   [6..22)   q     : f32x4 -- AHRS_ATTITUDE_Q, nalgebra coords order (i, j, k, w)
///   [22]      flags : u8    -- bit0 = imu_cal::CAL_DONE. Before this is set,
///                              q is just hil_link_task's identity fallback
///                              (att_estimator isn't spawned yet), not a real
///                              estimate -- a host waiting to inject a
///                              non-level test profile should poll this
///                              instead of guessing a fixed warm-up duration.
///   [23..25)  crc16 : u16   -- CRC-16/CCITT-FALSE over bytes [0..23)
const ATTITUDE_FRAME_LEN: usize = 25;
const ATTITUDE_FRAME_SYNC: u8 = 0x5A;
const ATTITUDE_FRAME_TYPE: u8 = 0x02;
const ATTITUDE_FLAG_CAL_DONE: u8 = 1 << 0;

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Matches the Python host's
/// crcmod/manual implementation -- see phase1_imu_inject.py.
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

struct SensorFrame {
    seq: u32,
    acc: [f32; 3],
    gyr: [f32; 3],
}

fn decode_sensor_frame(buf: &[u8; SENSOR_FRAME_LEN]) -> Option<SensorFrame> {
    if buf[0] != SENSOR_FRAME_SYNC || buf[1] != SENSOR_FRAME_TYPE {
        return None;
    }
    let crc_rx = u16::from_le_bytes([buf[30], buf[31]]);
    if crc16_ccitt(&buf[0..30]) != crc_rx {
        return None;
    }
    let seq = u32::from_le_bytes(buf[2..6].try_into().unwrap());
    let mut acc = [0.0_f32; 3];
    let mut gyr = [0.0_f32; 3];
    for i in 0..3 {
        acc[i] = f32::from_le_bytes(buf[6 + i * 4..10 + i * 4].try_into().unwrap());
        gyr[i] = f32::from_le_bytes(buf[18 + i * 4..22 + i * 4].try_into().unwrap());
    }
    Some(SensorFrame { seq, acc, gyr })
}

fn encode_attitude_frame(
    seq: u32,
    q: &UnitQuaternion<f32>,
    cal_done: bool,
) -> [u8; ATTITUDE_FRAME_LEN] {
    let mut buf = [0u8; ATTITUDE_FRAME_LEN];
    buf[0] = ATTITUDE_FRAME_SYNC;
    buf[1] = ATTITUDE_FRAME_TYPE;
    buf[2..6].copy_from_slice(&seq.to_le_bytes());
    let c = q.coords; // nalgebra order: [i, j, k, w]
    buf[6..10].copy_from_slice(&c[0].to_le_bytes());
    buf[10..14].copy_from_slice(&c[1].to_le_bytes());
    buf[14..18].copy_from_slice(&c[2].to_le_bytes());
    buf[18..22].copy_from_slice(&c[3].to_le_bytes());
    buf[22] = if cal_done { ATTITUDE_FLAG_CAL_DONE } else { 0 };
    let crc = crc16_ccitt(&buf[0..23]);
    buf[23..25].copy_from_slice(&crc.to_le_bytes());
    buf
}

const HIL_CHAN_N: usize = 4;
static HIL_IMU_CHAN: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    Imu6DofData<f32>,
    HIL_CHAN_N,
> = embassy_sync::channel::Channel::new();

struct HilImu {
    rx: embassy_sync::channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        Imu6DofData<f32>,
        HIL_CHAN_N,
    >,
}

impl HilImu {
    fn new(
        rx: embassy_sync::channel::Receiver<
            'static,
            CriticalSectionRawMutex,
            Imu6DofData<f32>,
            HIL_CHAN_N,
        >,
    ) -> Self {
        Self { rx }
    }
}

impl Imu6Dof for HilImu {
    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError> {
        Ok(self.rx.receive().await)
    }
    async fn read_acc(&mut self) -> Result<[f32; 3], DeviceError> {
        Ok(self.read_acc_gyr().await?.acc)
    }
    async fn read_gyr(&mut self) -> Result<[f32; 3], DeviceError> {
        Ok(self.read_acc_gyr().await?.gyr)
    }
}

#[embassy_executor::task]
async fn hil_link_task(r: UartLogResources) -> ! {
    use embassy_stm32::usart::Uart;

    let mut cfg = UartConfig::default();
    cfg.baudrate = HIL_LINK_BAUD;

    let (mut tx, mut rx) = Uart::new(r.usart, r.rx, r.tx, r.dma_rx, r.dma, UartLogIrqs, cfg)
        .unwrap()
        .split();

    let snd_imu = HIL_IMU_CHAN.sender();
    let mut att_rcv = signals::AHRS_ATTITUDE_Q.receiver();

    let mut buf = [0u8; SENSOR_FRAME_LEN];
    loop {
        if rx.read(&mut buf).await.is_err() {
            continue;
        }
        let Some(frame) = decode_sensor_frame(&buf) else {
            continue;
        };

        // Blocks if imu_reader hasn't drained the previous sample yet --
        // this backpressure is what makes the link lockstep: the host's
        // send rate self-paces to the controller's consumption rate.
        snd_imu
            .send(Imu6DofData {
                timestamp_us: embassy_time::Instant::now().as_micros(),
                acc: frame.acc,
                gyr: frame.gyr,
            })
            .await;

        let q = att_rcv.try_get().unwrap_or(UnitQuaternion::identity());
        let cal_done = micoairh743v2::imu_cal::CAL_DONE.load(core::sync::atomic::Ordering::Relaxed);
        let out = encode_attitude_frame(frame.seq, &q, cal_done);
        tx.write(&out).await.ok();
    }
}

#[embassy_executor::task]
async fn hil_imu_task() -> ! {
    common::tasks::imu_reader::main_6dof(HilImu::new(HIL_IMU_CHAN.receiver())).await
}
