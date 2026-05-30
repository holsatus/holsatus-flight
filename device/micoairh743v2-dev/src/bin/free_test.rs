//! MicoAir H743 -- thrust staircase test binary.
//!
//! Built on flight.rs task structure verbatim (known working reference).
//! The only difference from flight.rs is the mission body: instead of the
//! altitude-hold climb/hover/descend sequence, this binary executes a
//! 6-step thrust staircase for motor and controller characterisation.
//!
//! Mission:
//!   P0: 1 s open-loop thrust ramp to BASE_THRUST (pushes through stick-slip)
//!   P1: 3 s closed-loop climb 0 -> 1.0 m (alt_hold takes over TRUE_Z_THRUST_SP)
//!   P2: 1 s hover at 1.0 m (shortened from 15 s to avoid drift into walls
//!       before we solve flow-induced lateral drift)
//!   P3: 1 s fast descent 1.0 -> 0.25 m, then 1 s slow approach 0.25 -> 0.05 m,
//!       with lidar-based ground-detect disarm (h<0.15m for 300 ms) and a
//!       5 s safety timeout. flow_hold silent below 25 cm.
//!   Total arm-to-disarm: ~22 s
//!   rate-PID gains at flight.rs boot defaults
//!
//! Range shifted down two stops in D000064+ after the landing frame was
//! changed from Kapla blocks to corks + chopstick criss-cross (lighter,
//! wider stance). D000063 showed the drone already yaw-spinning at 120
//! percent, so hover thrust on the new build is below the old starting
//! point. 90-120 percent now covers the pre-liftoff to lift-transition
//! range where the interesting behaviour happens.
//!
//! IMPORTANT: the drone WILL lift off during this test, possibly within
//! the first step. Be ready on the TX15 SE kill switch at all times.
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

use core::sync::atomic::{AtomicI32, AtomicU32, AtomicU8, Ordering};

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
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
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
    AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0),
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
    // it as 0x... so `addr2line -e target/.../free_test 0x<value>`
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
        if defmt_count == 0 { buf[0] = b'0'; idx = 1; }
        else {
            let mut n = defmt_count;
            while n > 0 && idx < buf.len() { buf[idx] = (n % 10) as u8 + b'0'; idx += 1; n /= 10; }
            let mut i = 0; let mut j = idx - 1;
            while i < j { buf.swap(i, j); i += 1; j = j.saturating_sub(1); }
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
    const RTT_BUFFER_BASE: u32 = 0x2400_8ae8;         // BUFFER (1024 bytes)
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
            let b = unsafe {
                core::ptr::read_volatile((RTT_BUFFER_BASE + off) as *const u8)
            };
            // Two hex digits per byte.
            let hi = b >> 4;
            let lo = b & 0xF;
            putc(if hi < 10 { b'0' + hi } else { b'a' + hi - 10 });
            putc(if lo < 10 { b'0' + lo } else { b'a' + lo - 10 });
            if i & 0xF == 0xF { putc(b' '); }
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

/// Latest optical flow quality, shared between mtf01_reader and flow_hold for logging.
static FLOW_QUALITY: AtomicU8 = AtomicU8::new(0);

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

use common::nalgebra::{UnitQuaternion, Vector3};
use common::signals;
use common::tasks::att_estimator;
use common::tasks::commander::COMMAD_ARM_VEHICLE;
use common::tasks::controller_angle;
use common::tasks::controller_rate;
use common::tasks::eskf::EskfEstimate;
use common::types::actuators::MotorsState;
use common::types::config::DshotConfig;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::Timer;
use micoairh743v2::alt_hold::ALTITUDE_SETPOINT;
use micoairh743v2::log as ulog;
use micoairh743v2::mtf01;
use micoairh743v2::resources::{
    self, Bmi270Resources, BtLogResources, Mtf01Resources, SdmmcLogResources, UartLogResources,
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

    let mut led_green = Output::new(r.leds.green, Level::High, Speed::Low);
    let mut led_blue = Output::new(r.leds.blue, Level::Low, Speed::Low);
    let mut led_red = Output::new(r.leds.red, Level::Low, Speed::Low);

    thread_spawner.spawn(uart_writer_task(r.uart_log, r.bt_log, r.sdmmc).unwrap());

    ulog::log("[free] board init ok");
    // Git provenance line: every flight log now starts with the short SHA
    // that produced the binary (plus "-dirty" if the tree was not clean at
    // build time). Combined with the Makefile's `git-clean` pre-flash gate,
    // this gives end-to-end version tracing from log back to a commit.
    ulog::log(concat!("[free] git=", env!("GIT_SHA")));

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
    override_imu_rot().await;

    Timer::after_millis(10).await;

    let level_0_spawner = interrupt_executor!(FDCAN1_IT0, P10);
    let level_1_spawner = interrupt_executor!(FDCAN1_IT1, P11);

    ulog::log("[free] executors started");

    Timer::after_millis(1).await;

    level_0_spawner.spawn(resources::imu_reader_task(r.imu).unwrap());
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
    thread_spawner.spawn(micoairh743v2::ceiling_mode::ceiling_mode_task().unwrap());
    thread_spawner.spawn(micoairh743v2::gnss::gnss_reader_task(r.gps).unwrap());
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

    // Battery monitor owns ADC1, samples at 10 Hz, publishes filtered mV via
    // BATTERY_FILTERED_MV. alt_hold reads the live signal each tick for its
    // voltage compensation; FSM/manual log can read it too. Replaces the
    // previous one-shot boot read.
    thread_spawner.spawn(micoairh743v2::battery::battery_monitor_task(r.battery).unwrap());

    thread_spawner.spawn(resources::alt_hold_task(r.baro).unwrap());
    thread_spawner.spawn(mtf01_reader_task(r.mtf01).unwrap());
    // flow_hold re-enabled 2026-04-22 in velocity-damping-only mode (KP_POS=0)
    // to counter the steady left-drift seen in D000116 without triggering the
    // D000049 positive-feedback failure at borderline liftoff. The activation
    // threshold in mtf01_reader_task was also raised from 5 cm to 10 cm so
    // flow only engages once the drone is solidly airborne, not in the
    // transition regime. See also: flow_hold::KP_POS = 0.0 below.
    thread_spawner.spawn(flow_hold().unwrap());
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
        ulog::log("[free] SD ABORT -- motors will not arm (3x blue + 1x red forever)");
        led_green.set_low();
        loop {
            for _ in 0..3 {
                led_blue.set_high();
                Timer::after_millis(150).await;
                led_blue.set_low();
                Timer::after_millis(150).await;
            }
            led_red.set_high();
            Timer::after_millis(400).await;
            led_red.set_low();
            Timer::after_millis(600).await;
        }
    }

    // Visual executor-liveness + battery-tier indicator.
    //
    // Normal (HEALTHY / USB_POWER):
    //   GREEN toggles every 500 ms (1 Hz blink). Red off.
    //   Same liveness signal as before; decoupled from the log so that
    //   if the writer task dies but the executor is alive, the LED keeps
    //   blinking. If the LED freezes, the whole executor has panicked.
    //
    // Severe battery tier (LAND_NOW / CRITICAL / DAMAGE):
    //   RED strobes at 5 Hz (toggles every 100 ms). Green off.
    //   Visible from the drone without reading the BT log -- the only
    //   in-flight cue that battery-driven forced descent is active or
    //   that the pack is below the flight threshold.
    //
    // dshot-rate log line still fires every 2 s as before.
    let mut prev_tx =
        micoairh743v2::dshot_driver::DSHOT_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
    let mut bat_tier_rcv_hb = micoairh743v2::battery::BATTERY_TIER
        .receiver()
        .expect("BATTERY_TIER receiver slot (heartbeat)");
    let mut led_tick: u8 = 0;
    loop {
        Timer::after_millis(100).await;
        led_tick = led_tick.wrapping_add(1);

        let bat_tier = bat_tier_rcv_hb
            .try_get()
            .unwrap_or(micoairh743v2::battery::Tier::Healthy);

        if bat_tier.is_severe() {
            // 5 Hz red strobe; green held off so the alarm is unambiguous.
            led_green.set_low();
            led_red.toggle();
        } else {
            // Normal 1 Hz green heartbeat (toggle every 5 ticks = 500 ms).
            led_red.set_low();
            if led_tick % 5 == 0 {
                led_green.toggle();
            }
        }

        // Log dshot frame rate every 2 s (20 ticks).
        if led_tick % 20 == 0 {
            let cur_tx = micoairh743v2::dshot_driver::DSHOT_TX_COUNT
                .load(core::sync::atomic::Ordering::Relaxed);
            let rate = (cur_tx - prev_tx) / 2;
            prev_tx = cur_tx;
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "[free] hb dshot={}Hz", rate);
            ulog::log(s.as_str());
        }
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
    let _ = write!(s, "[free] rev=0x{:04X} timeout=500ms", MOTOR_REVERSE_FLAGS.bits());
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
        if filled < WINDOW_N { filled += 1; }

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
                    loop { Timer::after_secs(60).await; }
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
                s, "[ahrs] WARNING: not settled after {}ms, continuing anyway", MAX_WAIT_MS
            );
            ulog::log(s.as_str());
            return;
        }
    }
}

#[embassy_executor::task]
async fn uart_writer_task(
    r: UartLogResources,
    bt: BtLogResources,
    sd: SdmmcLogResources,
) -> ! {
    use block_device_adapters::BufStream;
    use core::fmt::Write as FmtWrite;
    use embedded_fatfs::{FileSystem, FsOptions};
    use embedded_io_async_061::Write as _;

    bind_interrupts!(struct UartIrqs {
        DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
        USART1       => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
    });
    bind_interrupts!(struct BtUartIrqs {
        DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH3>;
        UART8        => embassy_stm32::usart::InterruptHandler<peripherals::UART8>;
    });

    let mut uart = UartTx::new(r.usart, r.tx, r.dma, UartIrqs, UartConfig::default()).ok();
    // Onboard BT module (115200 by default, matches USART1). If init fails
    // we silently continue -- USART1 + SD logs still work.
    let mut bt_uart =
        UartTx::new(bt.usart, bt.tx, bt.dma, BtUartIrqs, UartConfig::default()).ok();

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
    let _ = FmtWrite::write_fmt(&mut s, format_args!("[sd] mounted -> {}/000001.LOG", dir_name.as_str()));
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
        let msg = match select(
            ulog::CRITICAL_CHANNEL.receive(),
            ulog::CHANNEL.receive(),
        )
        .await
        {
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

/// Mission FSM. Single owner of TRUE_RATE_SP / TRUE_ATTITUDE_Q_SP /
/// TRUE_Z_THRUST_SP / ALTITUDE_SETPOINT. Consumes RC channel state and
/// edge events from `rc_kill` and routes between GroundIdle, Manual,
/// AutoTakeoff, AutoHover, AutoLand, Fault. See mission_fsm.md for the
/// state diagram and SOP.
#[embassy_executor::task]
async fn mission_fsm_task() -> ! {
    use micoairh743v2::rc_kill::{Maneuver, Mode, RcEvent, RC_CHANNELS, RC_EVENT};
    // Single source of truth for the state enum + its wire encoding, shared
    // with phoenix_telem (which streams it as NAMED_VALUE_INT "FSM" and in
    // HEARTBEAT.custom_mode). Aliased to `State` so the body below is unchanged.
    use micoairh743v2::fsm_state::{self, FsmState as State};

    ulog::log("[fsm] waiting 5s for cal + sensors...");
    Timer::after_secs(5).await;

    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[fsm] ABORT: no SD card, not arming");
        loop { Timer::after_secs(60).await; }
    }

    wait_for_ahrs_ready().await;

    {
        ulog::log("[fsm] waiting for RC link (30 s timeout)...");
        let start = embassy_time::Instant::now();
        const RC_WAIT_MS: u64 = 30_000;
        while !micoairh743v2::rc_kill::RC_LINK_READY.load(Ordering::Relaxed) {
            if start.elapsed().as_millis() >= RC_WAIT_MS {
                ulog::log("[fsm] ABORT: no RC link after 30 s -- no arm");
                loop { Timer::after_secs(60).await; }
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
        const THROTTLE_BOTTOM_TOL: f32 = 0.02;
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
                break;
            }
            warned = true;
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
                let _ = write!(
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
    const TARGET_ALT: f32 = 1.0;
    const P0_RAMP_MS: u64 = 1_000;
    const P1_RAMP_MS: u64 = 3_000;
    const AUTO_HOVER_TIMEOUT_S: u64 = 8;
    const P3A_RAMP_MS: u64 = 1_000;
    const P3A_FINAL: f32 = 0.25;
    const P3B_RAMP_MS: u64 = 1_000;
    const P3B_FINAL: f32 = 0.05;
    const GROUND_DETECT_M: f32 = 0.15;
    const GROUND_HOLD_MS: u64 = 300;
    const DESCENT_TIMEOUT_MS: u64 = 5_000;

    const ON_GROUND_LIDAR_M: f32 = 0.10;
    // Pure ACRO (rate-mode) defaults for Manual. Bardwell-style: drone
    // does not self-level; sticks command angular rates directly. Roll
    // and pitch share a max rate; yaw is slower because most airframes
    // have less yaw authority. EXPO=0.3 gives a softer center for
    // small precise corrections, near-linear at the extremes.
    const MANUAL_MAX_ROLL_RATE: f32 = 3.0;  // rad/s, ~170 deg/s
    const MANUAL_MAX_PITCH_RATE: f32 = 3.0; // rad/s
    const MANUAL_MAX_YAW_RATE: f32 = 2.0;   // rad/s, ~115 deg/s
    const MANUAL_EXPO: f32 = 0.3;
    const MANUAL_THRUST_GAIN: f32 = BASE_THRUST * 1.4;
    const MANUAL_IDLE_DWELL_MS: u64 = 500;
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
    let mut entered_at = embassy_time::Instant::now();
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

    // One-shot flag for the Manual arming command. The motor governor's
    // 4.5 s arm sequence keeps `armed=false` for ~450 FSM ticks, during
    // which the per-tick `if !armed && thr_n > 0.0` would otherwise
    // re-send arm and re-log every 10 ms. Set on first send, cleared on
    // any Manual exit (to GroundIdle or Fault).
    let mut arm_command_sent: bool = false;
    // One-shot flag so an arm refusal by low-battery is logged once per
    // Manual session, not every tick.
    let mut arm_refused_logged: bool = false;
    // Battery-triggered forced descent: set to Some(start_instant) when
    // the FSM detects a non-flight-allowing tier while armed. None when
    // normal flight is permitted.
    let mut force_descent_since: Option<embassy_time::Instant> = None;
    // Periodic "FC is ready" log while sitting in GroundIdle, so the
    // operator on the BT terminal sees a continuous heartbeat instead
    // of silence after preflight passes.
    const GROUND_IDLE_LOG_PERIOD_MS: u64 = 2_000;
    let mut last_ground_idle_log = embassy_time::Instant::now()
        .checked_sub(embassy_time::Duration::from_millis(GROUND_IDLE_LOG_PERIOD_MS))
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
            None => { Timer::after_millis(20).await; continue; }
        };
        let lidar = lidar_rcv.try_get().unwrap_or(99.0);
        let armed = motors_rcv.try_get().map(|m| m.is_armed()).unwrap_or(false);
        let bat_tier = bat_tier_rcv
            .try_get()
            .unwrap_or(micoairh743v2::battery::Tier::Healthy);
        let battery_allows_flight =
            micoairh743v2::battery::tier_allows_flight(bat_tier);

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

        // Sample throttle here once per tick; ground-grace + Manual reuse.
        let thr_n_for_gate = micoairh743v2::rc_kill::stick_throttle(ch.raw[2]);

        // Ground-grace gate: near-ground (low lidar) + low throttle for
        // >=200 ms expands the RC freshness timeout to absorb BEC
        // brownouts during touchdown. Tracked independently of state so
        // it works in any mode that's near ground (Manual, AutoLand,
        // post-Fault drift).
        let near_ground = lidar < GROUND_DETECT_LIDAR_M
            && thr_n_for_gate < GROUND_DETECT_THR_N;
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
                    if in_ground_grace { ", ground-grace" } else { "" },
                );
                ulog::log(s.as_str());
            }
            last_seq = ch.seq;
            last_seq_change = embassy_time::Instant::now();
        }
        if armed && state != State::Fault
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
                State::AutoTakeoff | State::AutoHover => {
                    // Take the autonomous landing path, which does not
                    // depend on the RC link at all.
                    state = State::AutoLand;
                }
                State::AutoLand => {
                    // Already landing; let it complete. AutoLand has its
                    // own descent timeout + ground-detect disarm.
                }
                State::GroundIdle | State::Fault => {}
            }
            entered_at = embassy_time::Instant::now();
        }

        match state {
            State::GroundIdle => {
                // Periodic "ready" heartbeat so the operator on the BT
                // terminal sees the FC is alive and waiting, instead of
                // silence after the preflight gate passes.
                if last_ground_idle_log.elapsed().as_millis() as u64
                    >= GROUND_IDLE_LOG_PERIOD_MS
                {
                    last_ground_idle_log = embassy_time::Instant::now();
                    ulog::log("[fsm] GroundIdle ready -- flip SA to Manual or trigger Auto");
                }

                // On-ground gate: armed must be false, lidar low, AHRS ready
                // is implicit (we don't transition into GroundIdle until
                // wait_for_ahrs_ready returned). Mode level is sampled here.
                let on_ground = !armed && lidar < ON_GROUND_LIDAR_M;

                // Drain edge events; we only act on the *current* level for
                // mode, and on a fresh Trigger event for Auto takeoff.
                let mut trigger_pressed = false;
                while let Ok(ev) = RC_EVENT.try_receive() {
                    if let RcEvent::TriggerPressed = ev {
                        trigger_pressed = true;
                    }
                }

                if !on_ground {
                    // Cannot leave GroundIdle until physically grounded.
                    // (e.g. just-landed; lidar still settling.)
                    Timer::after_millis(50).await;
                    continue;
                }

                match ch.mode {
                    Mode::Manual => {
                        ulog::log("[fsm] GroundIdle -> Manual");
                        state = State::Manual;
                        entered_at = embassy_time::Instant::now();
                        manual_idle_since = Some(embassy_time::Instant::now());
                        zero_setpoints();
                        // Defensive reset: armed-gate one-shots start
                        // fresh each Manual entry.
                        arm_command_sent = false;
                        arm_refused_logged = false;
                    }
                    Mode::Auto => {
                        if trigger_pressed && ch.maneuver == Maneuver::Takeoff {
                            ulog::log("[fsm] GroundIdle -> AutoTakeoff (trigger)");
                            state = State::AutoTakeoff;
                            entered_at = embassy_time::Instant::now();
                            zero_setpoints();
                            COMMAD_ARM_VEHICLE.send(true);
                        }
                    }
                    Mode::Idle => { /* stay */ }
                }

                Timer::after_millis(50).await;
            }

            State::Manual => {
                drain_rc_events();

                // Pitch is negated to match the TX15's Liftoff-game mapping
                // (operator preference: keep one radio config that works for
                // both the sim and this drone). Roll, yaw, and throttle
                // already match drone-NED convention.
                let roll_n = micoairh743v2::rc_kill::stick_norm(ch.raw[0]);
                let pitch_n = -micoairh743v2::rc_kill::stick_norm(ch.raw[1]);
                let yaw_n = micoairh743v2::rc_kill::stick_norm(ch.raw[3]);
                let thr_n = micoairh743v2::rc_kill::stick_throttle(ch.raw[2]);

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
                        entered_at = embassy_time::Instant::now();
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

                // 5 Hz bench-debug log: stick normals + computed rates +
                // thrust + armed. Lets the operator verify polarity and
                // amplitude with props OFF before any flight.
                if manual_log_throttle.elapsed().as_millis() as u64 >= MANUAL_LOG_PERIOD_MS {
                    manual_log_throttle = embassy_time::Instant::now();
                    let mut s: heapless::String<128> = heapless::String::new();
                    let _ = write!(
                        s,
                        "[manual] r={:+.2} p={:+.2} y={:+.2} thr={:.2} | rate r={:+.2} p={:+.2} y={:+.2} thrust={:.1} armed={}",
                        roll_n, pitch_n, yaw_n, thr_n,
                        roll_rate_sp, pitch_rate_sp, yaw_rate_sp, thrust_sp,
                        armed as u8,
                    );
                    ulog::log(s.as_str());
                }

                // Arm on first non-idle throttle. One-shot per Manual entry:
                // motor governor takes ~4.5 s to flip `armed` to true, and
                // the per-tick re-fire would otherwise log + re-send ~450
                // times during that window. Battery must also permit
                // flight: tiers below HEALTHY block arming until the
                // pack is charged + the FC rebooted.
                if !armed && !arm_command_sent && thr_n > 0.0 {
                    if battery_allows_flight {
                        ulog::log("[fsm] Manual: throttle raised -- arming");
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
                        entered_at = embassy_time::Instant::now();
                        manual_idle_since = None;
                        arm_command_sent = false;
                    }
                } else {
                    manual_idle_since = None;
                }

                Timer::after_millis(10).await;
            }

            State::AutoTakeoff => {
                let elapsed = entered_at.elapsed().as_millis() as u64;
                if elapsed < P0_RAMP_MS {
                    let f = elapsed as f32 / P0_RAMP_MS as f32;
                    signals::TRUE_Z_THRUST_SP.send(BASE_THRUST * f);
                    Timer::after_millis(20).await;
                } else if elapsed < P0_RAMP_MS + P1_RAMP_MS {
                    let t = (elapsed - P0_RAMP_MS) as f32 / P1_RAMP_MS as f32;
                    let sp = TARGET_ALT * t;
                    ALTITUDE_SETPOINT.signal(sp);
                    Timer::after_millis(50).await;
                } else {
                    ALTITUDE_SETPOINT.signal(TARGET_ALT);
                    ulog::log("[fsm] AutoTakeoff -> AutoHover");
                    drain_rc_events();
                    state = State::AutoHover;
                    entered_at = embassy_time::Instant::now();
                }
            }

            State::AutoHover => {
                ALTITUDE_SETPOINT.signal(TARGET_ALT);

                let mut go_land = false;
                let mut refresh = false;
                while let Ok(ev) = RC_EVENT.try_receive() {
                    if let RcEvent::TriggerPressed = ev {
                        match ch.maneuver {
                            Maneuver::Land => go_land = true,
                            Maneuver::Hover => refresh = true,
                            Maneuver::Takeoff => { /* ignore */ }
                        }
                    }
                }

                if refresh {
                    ulog::log("[fsm] AutoHover: timeout refreshed");
                    entered_at = embassy_time::Instant::now();
                }

                if go_land || ch.mode == Mode::Idle
                    || entered_at.elapsed().as_secs() >= AUTO_HOVER_TIMEOUT_S
                {
                    let reason = if go_land {
                        "trigger=Land"
                    } else if ch.mode == Mode::Idle {
                        "mode=Idle"
                    } else {
                        "timeout"
                    };
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(s, "[fsm] AutoHover -> AutoLand ({})", reason);
                    ulog::log(s.as_str());
                    state = State::AutoLand;
                    entered_at = embassy_time::Instant::now();
                } else {
                    Timer::after_millis(50).await;
                }
            }

            State::AutoLand => {
                let elapsed = entered_at.elapsed().as_millis() as u64;
                if elapsed >= DESCENT_TIMEOUT_MS {
                    ulog::log("[fsm] AutoLand: descent timeout, disarming");
                    COMMAD_ARM_VEHICLE.send(false);
                    zero_setpoints();
                    state = State::GroundIdle;
                    entered_at = embassy_time::Instant::now();
                    drain_rc_events();
                    ulog::log("[fsm] state=GroundIdle");
                    continue;
                }

                let sp = if elapsed < P3A_RAMP_MS {
                    let f = elapsed as f32 / P3A_RAMP_MS as f32;
                    TARGET_ALT * (1.0 - f) + P3A_FINAL * f
                } else if elapsed < P3A_RAMP_MS + P3B_RAMP_MS {
                    let f = (elapsed - P3A_RAMP_MS) as f32 / P3B_RAMP_MS as f32;
                    P3A_FINAL * (1.0 - f) + P3B_FINAL * f
                } else {
                    P3B_FINAL
                };
                ALTITUDE_SETPOINT.signal(sp);

                if lidar < GROUND_DETECT_M {
                    // Reuse entered_at as ground-since timer once we cross
                    // the threshold; reset entered_at on first crossing.
                    // To avoid clobbering the descent timer, use a separate
                    // tracking variable. Hold for GROUND_HOLD_MS continuous.
                    static GROUND_FIRST_HIT: AtomicI32 = AtomicI32::new(-1);
                    let now_ms = embassy_time::Instant::now().as_millis() as i32;
                    let first = GROUND_FIRST_HIT.load(Ordering::Relaxed);
                    if first < 0 {
                        GROUND_FIRST_HIT.store(now_ms, Ordering::Relaxed);
                    } else if (now_ms - first) as u64 >= GROUND_HOLD_MS {
                        let mut s: heapless::String<64> = heapless::String::new();
                        let _ = write!(s, "[fsm] AutoLand: ground at h={:.2}m, disarming", lidar);
                        ulog::log(s.as_str());
                        COMMAD_ARM_VEHICLE.send(false);
                        zero_setpoints();
                        GROUND_FIRST_HIT.store(-1, Ordering::Relaxed);
                        state = State::GroundIdle;
                        entered_at = embassy_time::Instant::now();
                        drain_rc_events();
                        ulog::log("[fsm] state=GroundIdle");
                        continue;
                    }
                } else {
                    static GROUND_FIRST_HIT: AtomicI32 = AtomicI32::new(-1);
                    GROUND_FIRST_HIT.store(-1, Ordering::Relaxed);
                }

                Timer::after_millis(50).await;
            }

            State::Fault => {
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
                    entered_at = embassy_time::Instant::now();
                }
                Timer::after_millis(100).await;
            }
        }
    }
}

/// Original 6-step thrust staircase mission. Kept as a reference for the
/// old behaviour; not spawned in main(). The FSM (mission_fsm_task)
/// supersedes it.
#[allow(dead_code)]
#[embassy_executor::task]
async fn staircase_mission() -> ! {
    ulog::log("[mission] waiting 5s for cal + sensors...");
    Timer::after_secs(5).await;

    if ulog::SD_MOUNTED.load(Ordering::Relaxed) != 1 {
        ulog::log("[mission] ABORT: no SD card, not arming");
        loop { Timer::after_secs(60).await; }
    }

    wait_for_ahrs_ready().await;

    {
        ulog::log("[mission] waiting for RC link (30 s timeout)...");
        let start = embassy_time::Instant::now();
        const RC_WAIT_MS: u64 = 30_000;
        while !micoairh743v2::rc_kill::RC_LINK_READY
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            if start.elapsed().as_millis() >= RC_WAIT_MS {
                ulog::log("[mission] ABORT: no RC link after 30 s -- motors will NOT arm");
                loop { Timer::after_secs(60).await; }
            }
            Timer::after_millis(100).await;
        }
        ulog::log("[mission] RC link established");
    }

    ulog::log("[mission] arming motors");
    COMMAD_ARM_VEHICLE.send(true);
    Timer::after_secs(3).await;

    // Closed-loop altitude-hold mission. Replaces the open-loop thrust
    // staircase used through D000128. The staircase was always going to
    // bounce in and out of ground effect: open-loop thrust at a fixed
    // multiple of the firmware's hover-thrust estimate has no authority to
    // compensate for (a) pack-voltage sag, (b) ground-effect lift boost, or
    // (c) thrust-to-weight-estimate mismatch. Altitude-hold closes the loop
    // on lidar altitude and commands whatever thrust is needed to reach and
    // hold a target height, so we don't have to guess.
    //
    // Sequence:
    //   P0: 1 s open-loop ramp to BASE_THRUST (4.5). Gets through the
    //       stick-slip ground-contact zone fast before alt_hold takes over.
    //   P1: 3 s ramp of ALTITUDE_SETPOINT from 0 -> 1.0 m. alt_hold's PID
    //       grabs TRUE_Z_THRUST_SP at its 10 Hz rate once we start signalling
    //       setpoints. 3 s is fast enough to clear ground effect quickly,
    //       slow enough to avoid a climb-rate overshoot.
    //   P2: 15 s hover at 1.0 m. Long enough for flow damping to reach
    //       steady state and for any residual drift to be observable.
    //   P3: 3 s ramp setpoint 1.0 -> 0.0 m. Gentle descent.
    //   P4: setpoint = 0, wait 2 s, disarm.
    // Keep in sync with alt_hold::BASE_THRUST. Lowered 8.50 -> 8.00 after
    // D000018 over-climbed at ~1-2 m/s (baseline thrust 7.8 post-v_comp at
    // BASE=8.5 was ~10% above the drone's actual out-of-ground-effect
    // hover of ~7.0-7.3). See alt_hold.rs for the full calibration chain.
    const BASE_THRUST: f32 = 8.00;
    const TARGET_ALT: f32 = 1.0;
    const P0_RAMP_MS: u64 = 1_000;
    const P1_RAMP_MS: u64 = 3_000;
    // Hover shortened 15s -> 1s after D000028: with flow-induced drift
    // still unresolved, a 15 s hover window gives the drone time to
    // wander into a wall before P3 descent begins. 1 s is enough to
    // confirm altitude hold reached setpoint before triggering landing.
    // Extend again once drift is under control.
    const P2_HOVER_S: u64 = 1;

    ulog::log("[mission] P0: thrust ramp 0 -> base (1s, open-loop)");
    let ramp_start = embassy_time::Instant::now();
    loop {
        let elapsed = ramp_start.elapsed().as_millis() as u64;
        if elapsed >= P0_RAMP_MS { break; }
        let f = elapsed as f32 / P0_RAMP_MS as f32;
        signals::TRUE_Z_THRUST_SP.send(BASE_THRUST * f);
        Timer::after_millis(20).await;
    }

    ulog::log("[mission] P1: climb 0 -> 1.0m (3s, alt_hold owns thrust)");
    let climb_start = embassy_time::Instant::now();
    loop {
        let elapsed = climb_start.elapsed().as_millis() as u64;
        if elapsed >= P1_RAMP_MS { break; }
        let sp = TARGET_ALT * (elapsed as f32 / P1_RAMP_MS as f32);
        ALTITUDE_SETPOINT.signal(sp);
        Timer::after_millis(100).await;
    }
    ALTITUDE_SETPOINT.signal(TARGET_ALT);

    ulog::log("[mission] P2: hover 1.0m (15s)");
    Timer::after_secs(P2_HOVER_S).await;

    // P3 redesigned after D000026 flipped the drone at ~20 cm on descent:
    //
    //   - Fast phase: setpoint 1.0 -> 0.25 m over 1.0 s (~0.75 m/s). Brings
    //     the drone down quickly from cruise altitude to flare-start height.
    //   - Slow phase: setpoint 0.25 -> 0.05 m over 1.0 s (~0.20 m/s). Gentle
    //     approach that trades descent speed for stability near the ground.
    //   - Ground detection: any time lidar reads below 0.15 m continuously
    //     for 300 ms, break out of the loop and disarm. This guards against
    //     the drone sitting at the borderline altitude where flow_hold and
    //     ground effect conspire to flip it.
    //
    // mtf01_reader_task's flow-activation threshold was also raised from
    // 10 cm to 25 cm in the same change, so flow_hold goes silent before
    // the slow phase even begins -- no tilt commands during the critical
    // last metre.
    ulog::log("[mission] P3: descend 1.0 -> 0.05m with ground-detect disarm");
    const P3A_RAMP_MS: u64 = 1_000;
    const P3A_FINAL: f32 = 0.25;
    const P3B_RAMP_MS: u64 = 1_000;
    const P3B_FINAL: f32 = 0.05;
    const GROUND_DETECT_M: f32 = 0.15;
    const GROUND_HOLD_MS: u64 = 300;
    const DESCENT_TIMEOUT_MS: u64 = 5_000;

    let mut lidar_rcv = micoairh743v2::alt_hold::LIDAR_ALT_M.receiver().unwrap();
    let mut ground_since: Option<embassy_time::Instant> = None;
    let desc_start = embassy_time::Instant::now();
    let mut landed = false;
    loop {
        let elapsed = desc_start.elapsed().as_millis() as u64;
        if elapsed >= DESCENT_TIMEOUT_MS { break; }

        let sp = if elapsed < P3A_RAMP_MS {
            let f = elapsed as f32 / P3A_RAMP_MS as f32;
            TARGET_ALT * (1.0 - f) + P3A_FINAL * f
        } else if elapsed < P3A_RAMP_MS + P3B_RAMP_MS {
            let f = (elapsed - P3A_RAMP_MS) as f32 / P3B_RAMP_MS as f32;
            P3A_FINAL * (1.0 - f) + P3B_FINAL * f
        } else {
            P3B_FINAL
        };
        ALTITUDE_SETPOINT.signal(sp);

        if let Some(h) = lidar_rcv.try_get() {
            if h < GROUND_DETECT_M {
                let below = ground_since
                    .map(|t| t.elapsed().as_millis() as u64 >= GROUND_HOLD_MS)
                    .unwrap_or(false);
                if below {
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(s, "[mission] P3: ground detected at h={:.2}m, disarming", h);
                    ulog::log(s.as_str());
                    landed = true;
                    break;
                }
                if ground_since.is_none() {
                    ground_since = Some(embassy_time::Instant::now());
                }
            } else {
                ground_since = None;
            }
        }

        Timer::after_millis(50).await;
    }
    if !landed {
        ulog::log("[mission] P3: descent timeout, disarming anyway");
    }
    ALTITUDE_SETPOINT.signal(0.0);

    // Log the flow-integrated body-frame displacement before disarming so
    // each flight report includes a dead-reckoned "how far did I land from
    // where I took off" number. Compare against the operator's tape-measure
    // reading to ground-truth the flow scale factor over time.
    //
    // log_critical (not log) because the regular log channel's 5-message
    // flush policy would otherwise leave these final lines in the pending
    // bucket when the operator yanks the LiPo after touchdown -- which is
    // exactly what happened in D000079-D000082, all of which ended at the
    // "ground detected" line with flow_disp lost. Critical-channel messages
    // are drained + flushed immediately by uart_writer_task.
    let dx = FLOW_EST_X_MM.load(Ordering::Relaxed) as f32 / 1000.0;
    let dy = FLOW_EST_Y_MM.load(Ordering::Relaxed) as f32 / 1000.0;
    let dist = libm::sqrtf(dx * dx + dy * dy);
    let mut s: heapless::String<96> = heapless::String::new();
    let _ = write!(
        s,
        "[mission] flow_disp dx={:.2}m dy={:.2}m |d|={:.2}m (body frame, fwd/right)",
        dx, dy, dist
    );
    ulog::log(s.as_str());

    ulog::log("[mission] disarming");
    COMMAD_ARM_VEHICLE.send(false);

    // Best-effort drain of the regular channel too, so any telemetry from
    // the final descent isn't lost on power-off.
    ulog::wait_for_drain(4, 500).await;

    ulog::log("[mission] test complete");
    loop {
        Timer::after_secs(60).await;
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
        // TRUE_RATE_SP directly; bridging here would race and lose
        // yaw. Bypass.
        if !micoairh743v2::alt_hold::MANUAL_BYPASS.load(Ordering::Relaxed) {
            snd.send([roll, pitch, 0.0]);
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
                loop { Timer::after_secs(60).await; }
            }
        } else {
            count = 0;
        }
    }
}

#[embassy_executor::task]
async fn flow_hold() -> ! {
    let mut rcv = micoairh743v2::alt_hold::FLOW_VEL_MS.receiver().unwrap();

    let mut snd_att = signals::TRUE_ATTITUDE_Q_SP.sender();

    // KP_POS reverted 0.05 -> 0.0 after D000145. Confirmed in D000147 that
    // position hold was not the driver: with KP_POS=0 the drone still
    // oscillated at altitude and tripped the 5 rad/s gyro-runaway kill.
    // KD_VEL now lowered 0.25 -> 0.10 to reduce the flow-velocity -> tilt
    // gain. At D000147's ~0.5 m peak, flow reported vx = +/-0.8 m/s which
    // with KD_VEL=0.25 produced pitch commands at the +/-0.17 rad clamp.
    // With 0.10 the same input yields 0.08 rad (~4.6 deg) -- well below the
    // clamp and small enough that the body-rotation-driven feedback loop
    // cannot close. Proper structural fix is gyro-comp on the flow reading
    // (subtract omega*h), but this buys stability without changing shape.
    const KP_POS: f32 = 0.0;
    const KD_VEL: f32 = 0.10;
    const MAX_TILT_RAD: f32 = 0.17;

    let mut est_x = 0.0_f32;
    let mut est_y = 0.0_f32;

    {
        use common::types::actuators::MotorsState;
        let mut mtr_rcv = common::signals::MOTORS_STATE.receiver();
        loop {
            match mtr_rcv.changed().await {
                MotorsState::ArmedIdle | MotorsState::Armed(_) => break,
                _ => {}
            }
        }
    }

    ulog::log("[flow] motors armed, position hold active");

    let mut log_div: u8 = 0;

    loop {
        let [vx, vy] = rcv.changed().await;

        const MAX_FLOW_VEL: f32 = 1.0;
        if vx.abs() > MAX_FLOW_VEL || vy.abs() > MAX_FLOW_VEL {
            continue;
        }

        let fq = FLOW_QUALITY.load(Ordering::Relaxed);
        if fq < 50 {
            continue;
        }

        const DT: f32 = 0.02;
        est_x += vx * DT;
        est_y += vy * DT;

        const POS_CLAMP: f32 = 0.5;
        let pos_ok = est_x.abs() < POS_CLAMP && est_y.abs() < POS_CLAMP;
        est_x = est_x.clamp(-POS_CLAMP, POS_CLAMP);
        est_y = est_y.clamp(-POS_CLAMP, POS_CLAMP);

        let kp = if pos_ok { KP_POS } else { 0.0 };

        let pitch_cmd = ( (kp * est_x + KD_VEL * vx)).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);
        let roll_cmd  = (-(kp * est_y + KD_VEL * vy)).clamp(-MAX_TILT_RAD, MAX_TILT_RAD);

        let q = UnitQuaternion::from_euler_angles(roll_cmd, pitch_cmd, 0.0);
        snd_att.send(q);

        log_div = log_div.wrapping_add(1);
        if log_div >= 5 {
            log_div = 0;
            let mode = if pos_ok { 'P' } else { 'V' };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "[flow] {}q={} ex={:.2} ey={:.2} vx={:.2} vy={:.2} p={:.2} r={:.2}",
                mode, fq, est_x, est_y, vx, vy, pitch_cmd, roll_cmd
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

        let Some(mag) = mag_rcv.try_get() else { continue };
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
        while diff > 180.0 { diff -= 360.0; }
        while diff < -180.0 { diff += 360.0; }

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
        r.spi, r.sclk, r.mosi, r.miso,
        r.dma_tx, r.dma_rx,
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
    const GYR_SCALE_RADS: f32 =
        (2000.0_f32 * core::f32::consts::PI / 180.0) / 32768.0;

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

    bind_interrupts!(struct Mtf01Irqs {
        DMA1_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH3>;
        UART4        => embassy_stm32::usart::InterruptHandler<peripherals::UART4>;
    });

    let mut cfg = UartConfig::default();
    cfg.baudrate = 115_200;
    let mut uart = UartRx::new(r.usart, r.rx, r.dma, Mtf01Irqs, cfg).unwrap();

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
