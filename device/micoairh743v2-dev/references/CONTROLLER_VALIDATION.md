# Controller Validation on Hardware

Progressive validation of the H743v2 flight controller to determine the
regimes in which the rate-PID + mixer + motor-governor pipeline is stable
before committing to free flight. Each test adds one new variable.

## Test binaries

| Binary | Thrust | Hold | Restraint | Gyro autoabort | Purpose |
|---|---|---|---|---|---|
| `sub_hover_test`  | configurable (30 / 50 / 80%) | 25 s | rigid | 5 rad/s | Progressive thrust ramp, verify stability under load |
| `pid_sweep_test`  | 50% hover | 5 × 3 s | rigid | 5 rad/s | Walk `rate_{roll,pitch}.kp` through [0.02, 0.03, 0.05, 0.07, 0.10] live |
| `flight`       | 10% hover | 15 s | **none** (soft surface) | 2 rad/s | Probe for mechanical resonance / unexpected yaw drift when ground-friction-limited |

All three share the same init path, SD-card gate (3-blue + 1-red LED abort
pattern if card missing), `flip_kill`, `gyro_runaway_kill`, and the
`flight.rs` override gains (`kp = 0.05 / 0.04`, `ki = 0`, `kd = 0.002 / 0.001`).

## Reference values used by the analysis below

| Quantity | Value | Source |
|---|---|---|
| BMI088 ACC bias | `[-0.0154, -0.6074, -0.0071]` | `imu_cal` on flat desk, 2026-04-17 |
| Physical hover thrust | `mass × g = 0.18 × 9.81 = 1.77 N` | measured mass |
| `BASE_THRUST` | 4.50 mixer units | empirical (= physical hover) |
| `angle_kp` | 15.0 (roll + pitch) | common default |
| Control loop dt | 0.001 s (1 kHz) | `SIM_FREQUENCY` / `CONTROL_FREQUENCY` |
| Rate-PID D-gain | `kd × kp / dt` | `rate_pid.rs` implementation |

At the current override values, rate-PID D-gain = `0.002 × 0.05 / 0.001 = 0.100`.
For context, `kd = 0.015` flipped the drone in D000308/309 (D-gain = 0.75).
Sim-hardened mid-band passing D-gain is ~0.08; tumble edge ~0.60.

## D000016 — `sub_hover_test` at 80% hover (restrained)

### Setup

- Drone zip-tied to a cutting board clamped under a weighted object
- Props on
- `TEST_THRUST = HOVER_THRUST × 0.80 = 3.6 mixer units`
- Battery at boot: 15.24 V (`[bat] voltage=15242 mV`)
- Runtime cal: `[cal] acc = [-0.108, -0.634, -0.010]` (Y = -0.634 is within 0.03 of the `imu_cal` ground truth)
- AHRS ready: `r = 0.04°, p = -0.01°`

### Motor spread over the 25 s hold

| t (ms) | [BR, FR, BL, FL] | Spread |
|---|---|---|
| 1314 (ramp end) | [605, 611, 616, 624] | 19 |
| 1518            | [637, 647, 649, 658] | 21 |
| 1721            | [626, 651, 646, 669] | 43 |
| 1924            | [630, 651, 646, 665] | 35 |
| 2726 (mid-hold) | [628, 651, 645, 668] | 40 |
| 3526            | [641, 653, 644, 655] | 14 |
| 3726 (end hold) | [639, 651, 646, 656] | **17** |

### Findings

- **Motor spread at 80% is tighter than at 50% or 30%** (D000014 30% peak 243; D000015 50% peak 127; D000016 80% peak 43).
- `az` vibration envelope unchanged from lower thrust runs (stddev ~0.08 m/s²).
- Gyro under 0.02 rad/s across the full hold — restraint held.
- No gyro autoabort, no flip-kill, clean `[mtr] disarmed (UserCommand)`.
- Controller-math check against `kp_angle = 15`:
  - Observed peak roll estimate 2.4°, expected rate SP = `15 × 0.042 = 0.63 rad/s`
  - Observed rate SP = `-0.65 rad/s` → matches within noise.

### Conclusion

Firmware is demonstrably stable under load. Mixer signs are correct.
IMU cal procedure produces values within 0.03 of the `imu_cal` reference
when the surface is flat. **Last-stop restrained thrust validation
passes.** What restrained tests cannot tell us about the controller is
addressed by the PID sweep and future unrestrained / tethered runs.

## D000018 — `pid_sweep_test` (restrained)

### Setup

- Same zip-tie + cutting-board restraint as D000016
- Props on
- `TEST_THRUST = HOVER_THRUST × 0.50 = 2.25 mixer units` held constant
- `rate_roll.kp = rate_pitch.kp` stepped through `[0.02, 0.03, 0.05, 0.07, 0.10]` at 3 s each
- `ki` and `kd` held at override values throughout
- Yaw gains unchanged
- Runtime cal: `[cal] acc = [-0.083, -0.632, -0.007]`, AHRS r=0.01°, p=0.02°

### Motor spread by sweep segment

| idx | kp | Motor snapshot during dwell | Spread |
|---|---|---|---|
| 0 | **0.02** | [514, 516, 520, 521] | 7 |
| 1 | **0.03** | [513, 512, 524, 522] | 12 |
| 2 | **0.05** | [511, 512, 522, 526] | 15 |
| 3 | **0.07** | [502, 500, 535, 534] | 35 |
| 4 | **0.10** | [506, 492, 542, 529] | 50 |

### Findings

- Motor spread grows **roughly linearly** with kp (7 → 12 → 15 → 35 → 50 as kp doubles from 0.02 to 0.10).
- No high-frequency oscillation at any gain.
- No gyro autoabort, no flip-kill, clean disarm.
- Controller remains linear and stable across **2×** the flight.rs override value.

### Why the steps are subtle in the plot

With the drone restrained, the motor command is `base + kp × (attitude_error × angle_kp × dt)`. Because the attitude error is nearly constant (Madgwick settled on a small residual tilt, drone physically cannot rotate), the motor-command spread scales **directly** with kp. The step transitions are visible as small plateaus in the motor-command trace but are not dramatic — because there's no disturbance to trigger the rise-time / overshoot / oscillation behaviour that distinguishes good from bad gains in a free drone.

What this test **does** prove: no unstable gain in the range tested.
What this test **does not** prove: controller stability under real rotation dynamics. That requires unrestrained or tethered testing.

### Conclusion

On-hardware controller is linear up to `kp = 0.10` (2× the current flight gains).
Current D-gain (0.100) is ~6× below the sim-predicted tumble edge (0.60) and
~7.5× below the D000308/309 flip point (0.75). No sign of instability margin
being close on this axis at the current gains.

## Cross-test summary

| Regime | Test | Result | What we learned |
|---|---|---|---|
| Firmware + SD + safety nets | sub_hover_test 30% (D000014) | Clean | Cal procedure works, gyro quiet, mixer balanced |
| Restrained, rising thrust | sub_hover_test 50% (D000015) | Clean | Motor spread peak 127, vibration envelope tight |
| Restrained, near-hover | sub_hover_test 80% (D000016) | Clean, tightest spread | Controller healthy at flight thrust |
| Live gain change | pid_sweep_test (D000018) | Linear, no oscillation | Stability margin present at `kp ≤ 0.10` |

**What these tests cannot answer**: whether the rate loop is stable when the drone actually rotates, and whether Madgwick behaves correctly under real body rotation + accel coupling. That's the purpose of `flight` (low-thrust ground dynamics) and eventually a tethered free-flight attempt.

## Next planned test

`flight` at 10% hover on a soft surface. Tightened gyro autoabort
(2 rad/s) since the drone physically cannot spin that fast from motor
torque alone at 10% hover. Purpose: probe for mechanical resonance or
slow yaw drift that restrained tests artificially damp.
