# Holsatus HIL — Implementation Plan + Test Runbooks

Grounded in the `micoairh743v2-dev` branch and the architecture diagram. Two binaries: a HIL firmware variant on the H743, and a host program on the Mac (Python first, native later).

> **Update (2026-07-12):** the separate `bin/hil.rs` firmware variant has been merged into `flight.rs`. The one binary selects HIL mode at boot when the battery sense reads USB power (< 6 V) and flight mode on pack power, so `make flash BIN=flight` covers both; references to `BIN=hil` / `bin/hil.rs` below are historical.

## 0. The seam, in one line

The firmware reads sensors through traits/signals and publishes to a signal bus; HIL swaps the bottom layer to come from a serial link and swaps the motor output to send over the link. Nothing between the estimator and the mixer changes.

## 1. Injection / tap points (confirmed in code)

| Quantity | Direction | Attachment point | Notes |
|---|---|---|---|
| IMU (accel+gyro) | host -> FC | `impl Imu6Dof` for `HilImu` -> `imu_reader::main_6dof` | runs calibration -> publishes `CAL_MULTI_IMU_DATA` |
| Mag | host -> FC | mag signal (no clean 6dof trait; `Imu9Dof` adds `read_mag`) | confirm publish point |
| Baro | host -> FC | baro signal read inside `alt_hold` | confirm; inject at signal |
| GNSS | host -> FC | `RAW_GNSS_DATA` | direct signal publish |
| RC | host -> FC | `RC_CHANNELS_RAW` (`Option<[u16;16]>`) + `RC_STATUS` | exercises RC parse -> axes mapping |
| Motors | FC -> host | `impl OutputGroup` for `HilMotors` (`set_motor_speeds([u16;4])`) | replaces DShot; sends final speeds out |

Trait shapes (confirmed in `common/src/hw_abstraction/mod.rs`):

```rust
pub trait Imu6Dof {
    async fn read_acc(&mut self) -> Result<[f32;3], DeviceError>;
    async fn read_gyr(&mut self) -> Result<[f32;3], DeviceError>;
    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError>;
}
pub trait OutputGroup {
    async fn set_motor_speeds(&mut self, speeds: [u16;4]);
    async fn set_motor_speeds_min(&mut self);
    async fn set_reverse_dir(&mut self, rev: [bool;4]);
    async fn make_beep(&mut self);
}
```

## 2. Key decisions (now grounded)

- **Lockstep is clean.** `controller_rate` does `rcv_imu_data.changed().await`, so the rate loop is paced by IMU-sample arrival. A blocking `HilImu` paces the whole loop; the host controls cadence. No timer fight.
- **Python first, paired with lockstep.** Lockstep neutralises Python non-determinism (both sides block). Python carries Phases 1-3 (bring-up, scenarios, analysis). Free-running (Phase 4) moves the hot path to native Rust and may reuse `holsatus-sim`'s rapier3d `Sim`.
- **Inject IMU at the trait** so calibration runs on target. Load a **neutral calibration profile** for HIL so clean injected data passes through; later, inject a known bias and run `calibrator` to verify recovery.
- **Link = a spare UART** at 2-3+ Mbit to a USB-serial adapter. The board uses `embassy_stm32::usart` + DMA everywhere; USB CDC is not set up. Repurpose the GNSS UART (GNSS is injected, port is free).
- **Flash = `make flash-release BIN=<name>`** (stm32flash over UART bootloader; probe-rs/SWD is not connected). Before each flash: hold BOOT0, power-cycle the FC, release BOOT0. defmt RTT is unavailable without a probe; firmware-side logging is only visible if a log task is running over BT/UART.

## 3. Open questions still to confirm

1. Mag/baro publish points (exact signals).
2. Exact UART peripheral + pins to repurpose (read `resources.rs`).
3. Whether the MicoAir board exposes SWD pads for the probe (it should; confirm physically).

---

# Phases

Each phase has a goal, the firmware delta, the host delta, a **test runbook**, and a **pass/fail** rule.

## Phase 0 — Link characterisation spike (the gate)

**Goal:** decide the link and whether free-running at 1 kHz is reachable, before any flight logic.

**Firmware:** `bin/hil_echo.rs` — configure the HIL UART (USARTx, DMA) at the target baud; loop: read a fixed-size frame, write it straight back.

**Host:** `phase0_link_char.py` — ping-pong fixed frames, measure round-trip latency + jitter.

### Runbook

1. Wire the USB-serial adapter: adapter TX -> FC RX, adapter RX -> FC TX, GND -> GND. 3.3 V logic only.
2. Flash the echo firmware (from `device/micoairh743v2-dev`):

   ```bash
   # Hold BOOT0, power-cycle the FC, release BOOT0, then:
   make flash-release BIN=hil_echo PORT=/dev/cu.usbserial-XXXX
   # Power-cycle again after flash to boot into the firmware.
   ```

3. Find the serial device on macOS: `ls /dev/tty.usbserial-*`
4. Run the host harness:

   ```bash
   python3 phase0_link_char.py /dev/tty.usbserial-XXXX 2000000
   ```

5. Read the printed median / p99 / max RTT and jitter.

```python
# phase0_link_char.py
import serial, struct, sys, time, statistics
port, baud = sys.argv[1], int(sys.argv[2])
FRAME = 32
ser = serial.Serial(port, baud, timeout=0.05)
rtts = []
for seq in range(50000):
    pkt = struct.pack("<BBI", 0xA5, 0x01, seq) + b"\x00" * (FRAME - 6)
    t0 = time.perf_counter()
    ser.write(pkt)
    echo = ser.read(FRAME)
    t1 = time.perf_counter()
    if len(echo) == FRAME:
        rtts.append((t1 - t0) * 1e6)   # microseconds
rtts.sort()
p = lambda q: rtts[int(q * len(rtts))]
print(f"n={len(rtts)} median={statistics.median(rtts):.1f}us "
      f"p99={p(0.99):.1f}us max={rtts[-1]:.1f}us "
      f"jitter(sd)={statistics.pstdev(rtts):.1f}us")
```

### Pass/fail

- p99 RTT well under 1000 us and bounded max -> free-running at 1 kHz is viable (Phase 4 on).
- p99 approaching or exceeding 1000 us -> stay lockstep, or get a faster link before Phase 4.
- **macOS gotcha:** a stock FTDI adapter defaults to a 16 ms USB latency timer. If you see ~16000 us RTT, that is the adapter, not the link. Lower the latency timer or use a low-latency adapter (e.g. an FT2232H with the timer set, or a CDC-ACM adapter). Resolve this before trusting any number.

## Phase 1 — Lockstep, open-loop injection sanity

**Goal:** prove injected sensors propagate through calibration and the estimator on real silicon.

**Firmware:** `bin/flight.rs`

- Do not spawn the physical IMU reader. Spawn `imu_reader::main_6dof(HilImu::new(rx))`.
- Spawn `hil_link_task` owning the UART: decode inbound frames, push IMU samples to `HilImu`'s channel.
- Keep `att_estimator` and `eskf`. No motor tap yet.
- Add a small FC->host debug frame carrying `AHRS_ATTITUDE_Q` (subscribe and TX). This also seeds the TX path for Phase 2.

`HilImu` skeleton:

```rust
struct HilImu { rx: Receiver<'static, CriticalSectionRawMutex, Imu6DofData<f32>, N> }
impl Imu6Dof for HilImu {
    async fn read_acc_gyr(&mut self) -> Result<Imu6DofData<f32>, DeviceError> {
        Ok(self.rx.receive().await)         // blocks until host sends a frame -> paces the loop
    }
    async fn read_acc(&mut self) -> Result<[f32;3], DeviceError> { Ok(self.read_acc_gyr().await?.acc) }
    async fn read_gyr(&mut self) -> Result<[f32;3], DeviceError> { Ok(self.read_acc_gyr().await?.gyr) }
}
```

### Runbook

1. Flash: hold BOOT0, power-cycle, release BOOT0, then `make flash-release BIN=hil PORT=/dev/cu.usbserial-XXXX`. Power-cycle again to boot. Task startup is visible on the BT/UART log if the log task is running.
2. Static test: from Python, send a steady "level, stationary" IMU frame (acc = [0,0,-9.81], gyr = [0,0,0]) at ~1 kHz for a few seconds. Read back the attitude debug frame.
3. Tilt test: send "30 deg roll, stationary" (acc rotated 30 deg about X, gyr = 0). Read attitude.
4. Dynamic test: send a constant roll rate (gyr_x = 0.5 rad/s) and watch reported roll ramp.

### Pass/fail

- Static level -> reported attitude converges to level.
- 30 deg tilt -> reported roll converges to ~30 deg.
- Constant roll rate -> reported roll ramps at the commanded rate.
- If attitude looks biased, you forgot the neutral calibration profile (calibration is subtracting real-sensor bias from clean data).

## Phase 2 — Close the loop (lockstep, Python physics)

**Goal:** first real HIL. The board flies the model.

**Firmware:** add the motor path and RC injection.

- Motor tap, low-friction option: a small task subscribing `CTRL_MOTORS` (`[f32;4]`), TX over the link. Leave ESCs disconnected.
- Motor tap, faithful option: implement `OutputGroup` for `HilMotors`, feed it to the governor so you tap final `[u16;4]` DShot values (mirrors the IMU trait injection). Needs the governor to accept an injected output.
- RC injection: link task publishes `RC_CHANNELS_RAW` + `RC_STATUS`. Inject baro/GNSS as the flight mode needs.

**Host:** `phase2_hil.py` — minimal rigid-body quad. Loop: recv MotorFrame -> step physics by dt -> derive IMU/baro from state -> send SensorFrame. Use the airframe params from `holsatus-sim/sim_config.toml` (mass, inertia, thrust map) so the model matches what the controller was tuned for. A scenario script sets RC to arm, then commands throttle to hover.

```python
# phase2_hil.py (sketch)
# state: pos, vel, quat, omega ; params from sim_config.toml
while True:
    motors = recv_motor_frame(ser)          # [f32;4] or [u16;4]
    thrust, torque = mix(motors, PARAMS)    # thrust curve + geometry
    state = integrate(state, thrust, torque, dt)
    acc_body, gyr_body = imu_from(state)    # specific force + body rates
    baro = baro_from(state)
    send_sensor_frame(ser, acc_body, gyr_body, baro, rc=current_rc(t))
    t += dt
```

### Runbook

1. Flash: hold BOOT0, power-cycle, release BOOT0, then `make flash-release BIN=hil PORT=/dev/cu.usbserial-XXXX`. Power-cycle to boot.
2. Start the host: `python3 phase2_hil.py /dev/tty.usbserial-XXXX 2000000`.
3. In the scenario script: hold disarm 1 s, send arm RC (throttle low, arm switch), then ramp throttle to hover.
4. Log and plot: motor outputs, attitude, body rates, altitude.

### Pass/fail

- Vehicle arms on the injected RC.
- It lifts off and holds a bounded hover (altitude band, attitude near level) in the model.
- If it diverges immediately, the physics model does not match the controller's tuning. Recheck mass/inertia/thrust map against `sim_config.toml` and the motor mixing sign/order.

## Phase 3 — Scenario engine (lockstep)

**Goal:** the V&V layer — parameterised scenarios and pass/fail, tied to STPA UCAs.

**Host:** scenarios = initial conditions + wind/turbulence + fault injection (GNSS dropout, IMU bias, baro step) + mission/RC + pass/fail constraints. Lockstep is not real-time-bound, so run many (slowly).

Scenario format (example):

```yaml
name: gnss_dropout_hover
init: { alt: 5.0, level: true }
mission: [arm, takeoff_to(5.0), hover(30s)]
faults:
  - { t: 10.0, type: gnss_dropout }
asserts:
  - { quantity: alt, band: [4.0, 6.0] }      # holds altitude through dropout
  - { quantity: tilt_deg, max: 25 }
  - { quantity: armed, equals: true }         # never disarms
```

### Runbook

1. Flash `cargo run --bin hil --release`.
2. `python3 run_suite.py scenarios/ /dev/tty.usbserial-XXXX 2000000`
3. The runner loads each scenario, drives the rig, evaluates asserts, writes a pass/fail report + per-run logs.

### Pass/fail

- The suite runs headless and emits pass/fail per scenario. This report is the artifact that reads as V&V, not a demo. Map each assert back to a UCA / safety constraint from the Capella trace.

## Phase 4 — Free-running real-time (native host)

**Goal:** the timing evidence HIL adds over SITL.

**Firmware:** switch `HilImu::read_acc_gyr` from blocking-on-every-frame to "return freshest sample, run on the MCU clock." Add a loop-period logger (defmt) to measure on-target jitter.

**Host (native Rust):** move the link loop + physics off Python. Reuse `holsatus-sim`'s rapier3d `Sim`, or bridge to Gazebo. Hold RTF ~= 1 with bounded jitter using the Phase 0 budget.

### Runbook

1. Flash the free-running `hil` build.
2. Run the native host; log achieved RTF and the firmware's loop-period histogram.
3. Re-run a Phase 3 scenario in free-running and diff outputs against the lockstep reference.

### Pass/fail

- RTF sustained near 1.0; 1 kHz loop period holds with bounded jitter on target.
- Free-running and lockstep agree on the same scenario within tolerance. Divergence means either timing-sensitive behaviour (the thing you wanted to find) or a rig artifact; the lockstep reference tells you which.

## 4. What never changes

`att_estimator`, `eskf`, `controller_angle`, `controller_rate`, `angle_to_rate_bridge`, the mission FSM, and the commander run unmodified on the real H743 across all phases. That invariance is the source of the evidence.

## 5. Wire protocol (minimal, fixed-size)

```
Host -> FC  SensorFrame:
  u8  sync (0xA5), u8 type (0x01), u32 sim_time_us
  f32 acc[3], gyr[3]      # IMU (always)
  f32 mag[3]              # flagged
  f32 baro_alt            # flagged
  ... gnss                # flagged
  u16 rc[16]
  u16 crc
FC -> Host  MotorFrame:
  u8  sync (0x5A), u8 type (0x02), u32 fc_time_us
  f32 motors[4]  (or u16[4] if tapping OutputGroup)
  u16 crc
Handshake (lockstep): SensorFrame N -> FC runs one cycle -> MotorFrame N -> host steps physics -> SensorFrame N+1.
```
