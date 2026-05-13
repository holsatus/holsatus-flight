# PID Tuning Log -- MicoAir H743v2

## Hardware
- Frame: ~250g with 4S LiPo (158g without battery)
- Motors: KV1960, 4-in-1 ESC (BLHeli32), DShot300
- FC: MicoAir H743v2, BMI088 IMU, SPL06 baro, QMC5883L compass
- Optical flow: MTF-01 (lidar + PMW3901)
- Props: micro quad props (CW/CCW matched)

## Key Fixes Applied (in order of discovery)
1. Motor CHANNEL_MAP = [1,0,3,2] (Betaflight motor_output_reordering confirmed)
2. Motor reverse: MOTOR_0 | MOTOR_1 | MOTOR_2 (MOTOR_3 default correct)
3. IMU axis: cal_acc.scale = [1,-1,-1], cal_gyr.scale = [-1,-1,-1] (compensates rot_x_180 in att_estimator + roll gyro sign fix)
4. Baro address: 0x77 (not 0x76)
5. Compass: QMC5883L (not IST8310)
6. DShot keepalive: 1kHz sender via DshotProxy (motor_governor only sends ~20Hz during arming)
7. Battery voltage compensation: thrust * (15200/actual_mV)^2
8. Flip-kill: az < -3.0 for 10 consecutive samples at 1kHz = 10ms

## Angle Controller (common/, cannot override)
- Roll kp=15.0, ki=0.0, kd=0.0
- Pitch kp=15.0, ki=0.0, kd=0.0
- Yaw kp=25.0, ki=0.0, kd=0.03

## Altitude Hold
- BASE_THRUST = 4.50 (mixer scaling: DShot ~ 211 + 127*thrust)
- KP = 0.5, KI = 0.1, DT = 0.1s
- Integral clamp = +/-3.0
- Thrust clamp = 5.0
- NOMINAL_MV = 15200 (4S storage voltage)
- Altitude source: MTF-01 lidar (primary), SPL06 baro (fallback)

## Flow Hold
- KP_FLOW = 0.10, MAX_TILT_RAD = 0.17 (~10 deg)
- FLOW_SCALE = 0.25 (empirical: raw 5 counts at 8cm ~ 0.1 m/s)
- Axis mapping: drone_vx = +motion_y, drone_vy = -motion_x

---

## Tuning Iterations

### Iteration 1 (2026-04-03)
**Rate PID:** kp=0.08, ki=0.5, kd=0.03 (defaults from common/)
**Gyro scale:** [1.0, -1.0, -1.0]
**Result:** Severe yaw oscillation. Diagonal motor pairs alternate 147/500+. Yaw rate 4.7 rad/s burst. Motors never produce balanced thrust.
**Diagnosis:** Yaw ki=0.5 too aggressive. All axes too hot for this frame.

### Iteration 2 (2026-04-03)
**Rate PID:** roll/pitch kp=0.04 ki=0.20 kd=0.015, yaw kp=0.04 ki=0.05 kd=0.01
**Result:** Much better motor balance. Yaw oscillation reduced. Drone slides on ground but doesn't lift (insufficient thrust). First test with balanced motors [272,271,251,255].

### Iteration 3 (2026-04-04)
**Rate PID:** roll/pitch kp=0.03 ki=0.10 kd=0.010, yaw kp=0.01 ki=0.01 kd=0.005
**Result:** Very stable on ground. All motors within 30 counts. No yaw oscillation. But too weak to correct disturbances -- flips when tether cable is pulled.
**Log:** D000085 (35+ seconds stable on ground, motor avg 620-700)

### Iteration 4 (2026-04-05)
**Rate PID:** roll/pitch kp=0.05 ki=0.10 kd=0.015, yaw kp=0.01 ki=0.01 kd=0.005
**Result:** Motors initially balanced [736,738,730,733]. Drone drifts at walking speed (expected, no position hold). Flips when tether cable is pulled. Kill switch tested.

### Iteration 5 (2026-04-05)
**Gyro scale changed:** [1.0, -1.0, -1.0] -> [-1.0, -1.0, -1.0]
**Rationale:** PID plot from D000175 showed roll rate setpoint and actual gyro going in OPPOSITE directions (positive feedback). Inverting gyro X fixes the sign.
**Rate PID:** roll/pitch kp=0.05 ki=0.10 kd=0.015
**Result:** Setpoint and gyro now move in same direction (negative feedback confirmed). But drone still flips -- setpoint ramps to 2.5 rad/s because angle controller (kp=15) amplifies small tilt errors and rate controller (kp=0.05) is too weak to track.

### Iteration 6 (2026-04-06, current)
**Rate PID:** roll/pitch kp=0.10 ki=0.15 kd=0.02, yaw kp=0.01 ki=0.01 kd=0.005
**Gyro scale:** [-1.0, -1.0, -1.0]
**Rationale:** Rate controller must be strong enough to track angle controller commands. kp=0.10 is 2x the original default (0.08). With correct gyro sign, higher gain should stabilize rather than oscillate.
**Result:** Roll axis now correct (setpoint and gyro same direction). But PITCH axis shows positive feedback -- gy explodes to -20 rad/s while P output also goes negative (-3.5). Drone flips on pitch axis. Slightly slower flip than before but still violent.
**Log:** D000180
**Diagnosis:** Root cause identified: rate controller reads CAL gyro (pre-rot_x_180) but receives setpoints from angle controller in post-rot_x_180 frame. The Y (pitch) and Z (yaw) axes are INVERTED between these frames. Roll was fixed by scale change but pitch/yaw setpoints arrive in wrong sign convention.

### Iteration 7 (2026-04-06, current)
**Rate PID:** roll/pitch kp=0.10 ki=0.15 kd=0.02, yaw kp=0.01 ki=0.01 kd=0.005
**Gyro scale:** [-1.0, -1.0, -1.0]
**Bridge change:** Negate pitch setpoint in angle_to_rate_bridge: `snd.send([roll, -pitch, 0.0])`
**Rationale:** The angle controller outputs setpoints in the att_estimator's post-rot_x_180 frame, but the rate controller reads CAL gyro with Y inverted. Negating pitch in the bridge makes the setpoint match the measurement frame.
**Note:** Props showing wear from repeated crashes -- nicks and folds. Not the cause of the flip (that's the sign mismatch) but may affect thrust balance and vibration.
**Result:** Still violent pitch flip. Pitch setpoint +10.74 while gyro -13.6 (opposite). The bridge negation made pitch WORSE.
**Log:** D000181, battery 16.1V
**Diagnosis:** MISDIAGNOSIS in iterations 5-7. The D000175 plot showing roll setpoint vs gyro going "opposite" was actually CORRECT controller behavior (setpoint commands correction against disturbance). Changing gyro sign and negating bridge both BROKE previously-working axes. Reverting to iteration 3/4 settings which were proven stable for 35 seconds.

### Iteration 8 (2026-04-06, current)
**Rate PID:** roll/pitch kp=0.04 ki=0.10 kd=0.010, yaw kp=0.01 ki=0.01 kd=0.005
**Gyro scale:** [1.0, -1.0, -1.0] (REVERTED to original)
**Bridge:** No negation (REVERTED to original)
**Rationale:** Iteration 3 was the most stable configuration (35s on ground, balanced motors within 30 counts). The sign changes in iterations 5-7 were based on misdiagnosis. Reverting to proven-stable config with slightly higher gains than iteration 3.
**Battery:** test at storage voltage (15.2V) for consistency if possible
**Result:** Still flips on PITCH axis. Pitch angle reaches 49 degrees, pitch rate setpoint reaches 62 rad/s. Roll setpoint at -0.3 even when level (persistent bias). Battery 15.28V.
**Log:** D000182

## Comprehensive Analysis (after iterations 1-8)

**The fundamental problem:** The rate controller reads gyro from CAL_MULTI_IMU_DATA (pre-rot_x_180 frame) but receives setpoints from the angle controller which works in the post-rot_x_180 frame. rot_x_180 flips Y and Z, so pitch and yaw setpoints arrive in the OPPOSITE sign convention from the gyro measurements.

Proof: With scale [1,-1,-1], no bridge negation:
- Drone tilts nose-down 1 deg. Angle controller commands +0.26 rad/s (nose up).
- Physical pitch rate starts positive (nose going up to correct).
- raw_gy = +0.1. CAL = -0.1 (scale -1). Error = 0.26-(-0.1) = +0.36.
- As correction works and raw_gy increases to +0.5: CAL = -0.5. Error = 0.26-(-0.5) = +0.76.
- Error GROWS as the correction works! Classic positive feedback.

**Why iteration 3 appeared "stable":** BASE_THRUST was 0.25-0.30, motors barely spun, drone physically couldn't move. The positive feedback was there but hidden because the drone was ground-constrained and thrust was too low to create physical motion.

**The CORRECT fix IS bridge negation** (iteration 7 approach):
- Scale [1,-1,-1] (correct for att_estimator)
- Bridge: negate pitch to match rate controller's CAL frame
- This WAS correct in iteration 7 but was tested with WRONG X scale [-1,-1,-1]

**Why iteration 7 failed:** It used scale [-1,-1,-1] which also changed the X (roll) axis AND the att_estimator behavior. The combination created new problems on roll that masked the pitch fix.

### Iteration 9 (2026-04-07, current)
**Rate PID:** roll/pitch kp=0.06 ki=0.10 kd=0.012, yaw kp=0.01 ki=0.01 kd=0.005
**Gyro scale:** [1.0, -1.0, -1.0] (original, correct for att_estimator)
**Bridge:** Negate pitch: `snd.send([roll, -pitch, 0.0])`
**Rationale:** The ONLY correct combination: original scale for att_estimator + bridge negation for rate controller pitch axis. Roll axis doesn't need negation (rot_x_180 doesn't affect X). Moderate gains (between iteration 3's too-low and iteration 6's too-high).
**Battery:** storage voltage if possible
**Result:** Still flips on ROLL axis. Roll setpoint goes to -8 rad/s, gyro to +3 rad/s (opposite). Pitch also diverges (angle reaches 79 deg). Both right propellers broke off motors. Battery 15.4V.
**Log:** D000183

## Root Cause Analysis (after 9 failed iterations)

**The actual problem:** The `rot_x_180` in `att_estimator.rs` was written for a different board where the IMU was NOT in NED orientation. On the MicoAir H743v2, the BMI088 IS natively NED (confirmed: az=+9.81 when level = Z-down). The rot_x_180 CONVERTS the data FROM NED to [X,-Y,-Z], which is wrong for this board.

**Why no scale/bridge combination can fix this:**
- The att_estimator needs data in one frame (post-rot_x_180)
- The rate controller reads pre-rot_x_180 data (CAL)  
- The mixer expects NED (physical frame)
- These three frames CANNOT all agree when rot_x_180 is applied to NED-native data
- Scale inversions fix one consumer but break another

**Proof by exhaustion of all attempted combinations:**
- [1,-1,-1] no bridge: att_estimator OK, rate controller pitch/yaw WRONG (iters 1-4,8)
- [-1,-1,-1] no bridge: att_estimator changed, rate controller pitch/yaw WRONG (iter 5-6)
- [-1,-1,-1] + bridge negate pitch: roll broken by X scale change (iter 7)
- [1,-1,-1] + bridge negate pitch: creates double-error on pitch (iter 9)

**The ONLY correct fix:** Remove rot_x_180 from att_estimator.rs since the BMI088 on this board is natively NED. Then use scale [1,1,1] (identity). All three consumers (att_estimator, rate controller, mixer) will work in the same NED frame.

### Iteration 10 (2026-04-07)
**Change:** Removed rot_x_180 from att_estimator.rs, identity scale [1,1,1], no bridge negation
**Rate PID:** roll/pitch kp=0.06 ki=0.10 kd=0.012, yaw kp=0.01 ki=0.01 kd=0.005
**Result:** Still flips to front and right. Clear positive feedback on pitch: front motors (M1,M3) go to max while back (M0,M2) go to min, creating nose-down torque that accelerates the nose-down flip. gy reaches -30 rad/s.
**Log:** D000191, battery 15.2V
**Diagnosis:** Removing rot_x_180 was the WRONG fix. The "Root Cause Analysis" above was based on a flawed assumption: it claimed az=+9.81 means "NED / Z-down" but +9.81 actually means Z-UP (accelerometer measures upward reaction force). The BMI088's native frame is Z-up, NOT NED.

## Corrected Analysis (after D000191)

**The original config was CORRECT all along.** Here is why:

The ahrs crate Madgwick assumes Z-up (gravity reference = [0,0,1]). Scale [1,-1,-1] converts the Z-up raw data to a Z-down frame for the rate controller and mixer. The rot_x_180 inside att_estimator cancels the scale, recovering the original Z-up data for the Madgwick filter. The euler angle rot_x_180 converts the Madgwick's Z-up attitude output back to the Z-down convention used by the angle controller.

Net result: Madgwick gets correct Z-up data, rate controller and mixer get consistent Z-down data, euler angles bridge the two. All three consumers agree.

**What was ACTUALLY wrong in iterations 1-4:** The rate controller gains were too low. Iteration 3 was stable for 35+ seconds with balanced motors -- it only flipped when the tether cable was pulled (external disturbance exceeding controller authority). This was a GAIN problem, not a SIGN problem.

**What went wrong in iterations 5-9:** The D000175 plot showing "opposite" setpoint vs gyro was misinterpreted as positive feedback. It was actually correct negative feedback (controller commanding correction against the disturbance). Every subsequent sign change broke a previously-working axis.

### Iteration 11 (2026-04-07, next)
**Rate PID:** roll/pitch kp=0.08 ki=0.15 kd=0.02, yaw kp=0.01 ki=0.01 kd=0.005
**Gyro scale:** [1.0, -1.0, -1.0] (RESTORED -- required for rate controller / mixer frame)
**Accel scale:** [1.0, -1.0, -1.0] (RESTORED)
**att_estimator:** rot_x_180 RESTORED (required for Madgwick Z-up assumption)
**Bridge:** No negation (KEPT from iteration 10)
**Rationale:** Restore proven-stable config from iterations 3-4 with higher gains. kp=0.08 is the original default (iteration 1). With correct signs confirmed by 35s ground stability in iteration 3, higher gains should provide enough authority to handle disturbances and achieve hover.
**Result:** PENDING (waiting for replacement props + screws)

---

## Pre-test Notes for Iteration 11

**Binary is built and ready to flash** (cargo build --release --bin flight).

**Before testing:**
- Install fresh props with new screws. Verify CW/CCW matched to motor directions.
- Visually inspect ESC solder joints on all 4 outputs (right-side motors had
  USB spin issues -- likely just voltage but worth a glance).
- Test on battery at storage voltage (~15.2V) for consistency with earlier data.

**What to watch for:**
- If motors are balanced within ~30 counts at idle thrust (like iteration 3),
  the signs are confirmed correct and this is purely a gain tuning exercise.
- If there is still a consistent tilt bias (like the -0.3 deg roll in iter 8),
  note the direction -- could indicate CG offset or prop thrust imbalance.
- If the drone lifts off and oscillates but does NOT flip, gains need tuning
  down. If it flips, we have a remaining sign or frame issue.

**Gain adjustment strategy if iteration 11 is stable but oscillates:**
- Reduce kp first (try 0.06), keep ki and kd.
- If yaw oscillation returns, reduce yaw ki (currently 0.01, try 0.005).
- Target: stable hover for 10+ seconds without external disturbance before
  increasing to full altitude hold.

### Iteration 12 (2026-04-12 to 2026-04-13)
**Motor direction fix:** The right-side motors (FR, BR) were consistently spinning
wrong direction in flight.rs but correct in motor_dir_test. Root cause: the
keepalive sender called `driver.set_motor_speeds([0,0,0,0])` which goes through
`throttle_clamp(0)` = DShot 48 (min throttle, motors spin). BLHeli ESCs reject
direction commands when the motor is not at DShot 0 (disarm/stopped).
Fix: keepalive now sends `driver.set_motor_speeds_min()` (DShot 0) when all
speeds are zero. Also fixed burst delivery (10 consecutive direction frames).
Also corrected MOTOR_REVERSE_FLAGS: MOTOR_0 | MOTOR_1 (removed MOTOR_2).
**Logs:** D000217-219 (broken, DShot 48), D000221 (working, DShot 0)

### Iteration 13 (2026-04-13)
**Rate PID:** kp=0.08, ki=0.15, kd=0.02
**Result:** Handheld test D000233. Controller stable and correcting in right
direction on all axes. Over-reactive to hand tilts. Motor spread up to 956.
No oscillation or ringing. I-term stays near zero.

### Iteration 14 (2026-04-13)
**Rate PID:** kp=0.05, ki=0.15, kd=0.02
**Result:** Free-flight bounce test D000235. Flipped at liftoff. Motors started
balanced [721,721,720,720] but within 80ms the pitch diverged due to ground-
effect disturbance. Rate controller with kp=0.05 produced peak P-output of
only 0.109, insufficient to track angle controller setpoints (up to 1.6 rad/s).
Controller was correcting in the RIGHT direction (front motors up for nose-down
correction) but too weakly. Three propellers broken.
**Diagnosis:** kp=0.05 too low for free flight. kp=0.08 minimum needed.

### Iteration 15 (2026-04-13)
**Rate PID:** kp=0.08, ki=0.15, kd=0.03
**Change:** kd bumped from 0.02 to 0.03 for more damping against fast transients.
**Status:** Superseded by iteration 16.

### Iteration 16 (2026-04-14): frame measurements + mixer fix
**Physical measurements:**
- Mass: 180g (with tether), 168g (bare frame for pendulum)
- Motor spacing: 10.5 cm front-back, 11.25 cm left-right
- Arm half-lengths: x_half=0.0525 m, y_half=0.05625 m
- Principal inertia (bifilar pendulum): Ix=0.0000165, Iy=0.0000206, Iz=0.0000488 kg*m^2
- CG at geometric center

**Changes:**
1. Mixer arm lengths corrected: quad_x_basic(0.105, 0.1125) -- was (0.165, 0.225).
   Old mixer under-commanded motor differentials by ~1.6x.
2. Rate PID reduced to compensate: kp=0.05, ki=0.10, kd=0.015 (from 0.08/0.15/0.02)
3. IMU bias re-calibrated on flat surface: [-0.0891, -0.1883, 0.0110]

**D000273:** Handheld. Fast vibration -- kp=0.08 was too aggressive with corrected mixer.
**D000278:** Handheld, kp=0.05. Stable, small pitch offset (3.5 deg). No fast oscillation.
**D000283:** Handheld, kp=0.05. Clean data, drone resists rotation. Slight jitter on release (finger artifact, not PID).

### Iteration 17 (2026-04-14): angle ki attempt
**Change:** Angle controller ki=0.5 for roll/pitch.
**D000280:** Handheld. Self-accelerating feel: integral windup from hand constraint.
**D000284:** String-suspended. 12-degree pitch offset (string pendulum corrupts Madgwick). Integral wound up, violent spinout when string broke. Cut user's thumb.
**Conclusion:** Angle ki is unsafe with ANY constraint (hand, string, stick). Only for free flight.
**Reverted:** angle ki=0 for subsequent tests.

### Iteration 18 (2026-04-14): yaw fix
**Problem:** D000290 free hover: drone jumped up and spun rapidly in yaw.
**Root cause:** cal_gyr.scale[2]=-1.0 inverts the Z-axis gyro. The mixer's reaction torque
signs (outspin=false) assumed un-inverted Z. Result: yaw correction in wrong direction
(positive feedback). Confirmed by D000292 handheld: gz=-2.5 rad/s sustained.
**Fix:** outspin=true in DEV_QUAD_MOTOR_SETUP. Flips reaction torque signs to match
inverted Z-axis gyro convention.
**D000292 (before fix):** Handheld. Continuous yaw spin at -2.5 rad/s. Yaw correction inverted.
**D000295 (after fix):** Handheld. Yaw rate < 0.2 rad/s. Slow yaw drift (0.3 deg/s) from
gyro bias -- normal for rate-hold without compass yaw reference. Roll/pitch stable.
**Yaw gains:** kp=0.04, ki=0.02, kd=0.01 (increased from 0.01/0.01/0.005).

### Iteration 19 (2026-04-14): free hover attempt
**Rate PID:** roll/pitch kp=0.05, ki=0.10, kd=0.015; yaw kp=0.04, ki=0.02, kd=0.01
**Angle PID:** kp=15, ki=0, kd=0 (P-only, no integral)
**Mixer:** outspin=true, arm lengths (0.105, 0.1125)
**D000297:** Free hover from ground. Drone lifts off, immediate pitch disturbance from
ground effect (pitch swings from -1.6 to +3.3 deg in 80ms). The P-only angle controller
(kp=15) generates rate_sp of only -0.19 rad/s at 3.3 degrees. Rate controller generates
modest motor differential but insufficient to counter the liftoff impulse. Pitch diverges
to 4.7 deg, then pitch correction saturates motors (back motors maxed, front at minimum).
The saturated pitch correction has a yaw side-effect: different reaction torques between
maxed and min'd motors create net yaw torque. Yaw accelerates, drone tumbles.

**Key observations:**
- Before liftoff: roll=0.3, pitch=-1.6, motors balanced [680,759,672,743]. Clean start.
- Liftoff impulse at t=8456: gz jumps to -0.48 (ground effect), pitch swings +5 deg in 80ms.
- Pitch correction saturates by t=8537: motors=[986,530,858,296]. Two motors near minimum.
- Yaw diverges at t=8557: saturated pitch correction creates asymmetric yaw torque.
- Tumble by t=8578: all axes diverge, unrecoverable.

**Failure chain:** ground-effect pitch impulse -> angle controller too weak (P-only, no ki)
-> pitch error persists -> rate controller saturates motors -> yaw side-effect from
saturation -> yaw runaway -> tumble.

**Root cause is NOT a sign error.** The corrections are in the right direction on all axes.
The problem is insufficient authority to handle the liftoff transient on a 180g drone with
very low inertia (Iy=0.0000206 kg*m^2). The ground-effect impulse creates angular
acceleration of ~240 rad/s^2 (based on gz change of 0.48/0.020s), which the weak
P-only angle controller cannot overcome.

**Options for next iteration:**
1. Reduce liftoff transient: launch from an elevated surface (table edge) so ground effect
   is minimal. Or use a launch pad with a hole underneath to allow airflow.
2. Increase angle kp (try 20-25) to react faster to the liftoff impulse. Risk: oscillation.
3. Add angle ki=0.3 for free flight (proven safe in D000295 handheld when axes are free).
   The integrator would help maintain correction authority during the sustained pitch error.
4. Add feed-forward: predict the liftoff transient and pre-compensate.
5. Increase rate controller gains slightly (kp=0.06) for more motor authority.

### Iteration 20 (2026-04-14): elevated platform launch
**D000298:** Launched from narrow platform on bike pump (~45cm above ground).
Drone rolled over immediately and never recovered. No ground effect this time.

**Root cause: hardcoded accel bias mismatch.**
Raw accel at startup: ax=0.3, ay=0.6, az=9.8. After hardcoded bias [-0.089,-0.188,0.011]:
calibrated ay=0.79 m/s^2 -> arctan(0.79/9.8) = 4.6 degrees phantom roll.
Madgwick converged to 4.7 degrees roll before motors spun. Angle controller with kp=15
commanded rate_sp=-1.23 rad/s to correct the phantom error. First motor frame:
[BR=820,FR=775,BL=664,FL=561]. Right side much higher than left -> drone tipped left
off the narrow platform and tumbled.

Compare D000297 (same code, different surface): raw ay=-0.1, calibrated ay=+0.09,
Madgwick roll=0.3 degrees. Much better starting attitude, different failure mode.

**The fundamental problem:** hardcoded accel bias only works if the drone starts on the
exact surface orientation where calibration was performed. The BMI088 bias drifts with
temperature and the starting surface may not be perfectly level. A 0.7 m/s^2 difference
in raw ay between runs translates to 4 degrees of phantom roll -- enough to flip.

**Solution: restore runtime calibration** but skip the long pickup window. The drone
must sit level and still for the first ~1 second, then the Madgwick will converge to the
correct attitude. This is what every commercial flight controller does (Betaflight, ArduPilot):
calibrate gyro+accel on every boot, require the drone to be stationary and level.

The hardcoded bias can serve as a fallback/initial value, but the runtime average over the
first 500 samples is essential for correcting temperature drift and surface-specific offsets.

### Iteration 21 (2026-04-14): runtime cal + direct thrust + elevated platform

**Changes:**
1. Restored runtime accel+gyro calibration (2000 samples, ~2s). Also calibrates gyro bias.
2. Direct thrust control for entire mission (bypasses altitude controller).
   ALTITUDE_SETPOINT=-10 suppresses alt_hold; mission writes TRUE_Z_THRUST_SP at 50Hz.
3. 10-second thrust ramp: 0 -> BASE_THRUST (4.50) linearly.
4. Drone on speaker platform (~35cm above ground).

**D000302 (direct thrust, but still had alt_hold fight):**
`acc=[-0.223,-0.381,0.000]`. Thrust alternated between mission ramp (2.6) and alt_hold
BASE_THRUST (4.0) every 100ms -- 50% thrust oscillation at 10Hz physically shaking the
drone. Attitude was perfect during ramp but the oscillation caused pitch divergence at
liftoff. Fixed by setting ALTITUDE_SETPOINT=-10 to suppress alt_hold.

**D000306 (alt_hold suppressed, clean thrust ramp):**
`acc=[0.084,1.025,-0.068]` -- speaker surface tilted ~6 degrees! Despite this, the
Madgwick converged to roll=0.1, pitch=0.0 since the biased accel IS the gravity reference
during calibration. Thrust ramp was perfectly smooth (0.3 -> 2.9 over 6 seconds).
Attitude rock-solid until t=14400.

Failure at t=14562-14684: roll drifted from 0.4 to 3.8 degrees over 5 seconds during
the ramp as prop wash vibration corrupted the accelerometer gravity reference. At liftoff
(thrust ~2.9, 65% hover), the 3.8-degree roll caused an immediate tip-off the platform.
Then cross-coupled pitch oscillation -> tumble. Broke one propeller.

**Key positive finding:** the thrust ramp and direct thrust control work correctly.
No altitude controller interference. Attitude was stable for 6+ seconds of ramp.

**Root cause of all liftoff failures:** the Madgwick filter drifts 2-4 degrees during the
ramp phase due to prop wash vibration on the accelerometer. On a heavy drone this would
be negligible, but on a 180g / 0.0000165 kg*m^2 drone, 3 degrees of phantom tilt at
liftoff creates an unrecoverable roll moment.

**Remaining options:**
1. Launch from the ground (no platform) and accept the ground-effect pitch transient.
   The ground-effect impulse lasts ~50ms; the Madgwick drift is a slower, more sustained
   error. The impulse might be survivable if the controller is fast enough.
2. Increase Madgwick filter trust in gyro vs accel (reduce accelerometer weight) so prop
   wash doesn't corrupt the attitude estimate. This is the fundamental fix.
3. Add a vibration notch filter on the accelerometer before the Madgwick.
4. Accept the current state and validate in simulation first before more hardware tests.

---

## Handheld Axis-Check Procedure (Phase 1)

The mission_sequencer in flight.rs logs prompts at 5-second intervals. Hold
the drone with propellers spinning and follow the sequence below. Tilt gently
(10-15 degrees). The goal is to verify each axis corrects in the right
direction before attempting free flight.

### Sequence

| Time window | Action | Expected motor response |
|-------------|--------|------------------------|
| 0 - 5 s | Hold LEVEL | All four motors roughly equal (~720 DShot) |
| 5 - 10 s | Tilt NOSE DOWN | Front (FR, FL) INCREASE, back (BR, BL) DECREASE |
| 10 - 15 s | Tilt NOSE UP | Back (BR, BL) INCREASE, front (FR, FL) DECREASE |
| 15 - 20 s | Tilt RIGHT | Right (FR, BR) INCREASE, left (FL, BL) DECREASE |
| 20 - 25 s | Tilt LEFT | Left (FL, BL) INCREASE, right (FR, BR) DECREASE |
| 25 - 30 s | Hold LEVEL | Return to baseline |

The controller FIGHTS the tilt: it increases thrust on the side that went
DOWN to push it back up.

### How to verify from the log

1. Grep for `[mission]` lines to find the timestamps of each phase.
2. Look at A lines between those timestamps:
   `A,t,ax,ay,az,gx,gy,gz,m0(BR),m1(FR),m2(BL),m3(FL)`
3. For each tilt phase, check that the motor pair on the LOW side (the
   side that went DOWN) has HIGHER DShot values (controller pushing it
   back up).
4. If any axis shows the OPPOSITE response (motors increase on the side
   that went UP instead of DOWN), that axis has a sign error and must be
   fixed before free flight.

### Motor index reference (A line)

- m0 = BR (back-right)
- m1 = FR (front-right)
- m2 = BL (back-left)
- m3 = FL (front-left)

### DShot speed log channel order (dshot lines)

spd=[FR, BR, FL, BL]

### Iteration 22 (2026-04-14): sim-first approach + position hold + slow ramp

**Changes (validated in holsatus-sim SITL first):**
1. Slow thrust ramp: P0 phase ramps thrust from 0 to BASE_THRUST over 10s.
   Liftoff at ~5-6s when thrust exceeds weight. 5s abort window.
2. Alt hold: added D-term (KD=0.8) for velocity damping, prevents altitude overshoot.
3. Flow hold upgraded from velocity-only to PD position hold using dead-reckoned
   flow integration. Safeguards: spike rejection (>1 m/s), quality gate (<50),
   position clamp (0.5m fallback to velocity-only).
4. PID gains: kp=0.05 ki=0.10 kd=0.015, yaw kp=0.04 ki=0.02 kd=0.01
   (same as iteration 19 -- carried over from before sim work).
5. Runtime IMU calibration (500 samples).
6. Mission: P0 ramp (10s), P1 climb to 1m (10s), P2 hover (10s), P3 descend (10s).

**D000308:** Drone on speaker platform. Thrust ramp smooth, motors balanced at
first spin [386,387,369,377]. Within 80ms of producing thrust, motors diverge
violently: [495,532,153,188]. Two motors at 147 (min) while others overdrive.
Continuous yaw spin 0 -> +27 degrees. Full tumble within 400ms. Log ends at
t=12.3s. Flow sensor reported all zeros throughout -- never engaged.
**Battery:** not recorded.

**D000309:** Same code, different run. Motor divergence within 80ms again. Motor 3
saturates at 147 first, others ramp to 2046. Roll reached 37 degrees in 400ms.
Drone fell off speaker platform.

**Root cause:** The D-term (kd=0.015) amplifies the liftoff transient (ground
effect or platform edge disturbance) into massive torque demands. At 1kHz,
d_gain = kd*kp/dt = 0.015*0.05/0.001 = 0.75. A 10 rad/s^2 angular acceleration
spike produces 0.75*10 = 7.5 N*m torque demand, which the mixer maps to >8N
per-motor differentials. Two motors clip at minimum (147), the other two overdrive.
The one-sided clipping creates asymmetric yaw torque -> yaw runaway -> tumble.

This is the same failure mode as D000297/306 but faster because the D-term
amplifies the initial transient before the P-term even gets involved.

**Lesson:** The D-term must be ZERO until hover is proven stable. It can be
added later for damping once the P-only controller survives liftoff.

### Iteration 23 (2026-04-15, next): kd=0 + slow ramp

**Rate PID:** roll/pitch kp=0.05, ki=0, kd=0. Yaw kp=0.04, ki=0, kd=0.
**Rationale:** kp=0.05 proven handheld-stable (D000278/283) with corrected mixer.
ki=0 and kd=0 eliminate integral windup and D-term transient amplification.
The P-only controller is the minimum viable configuration for first hover.
**Mission:** P0 thrust ramp (10s, liftoff ~5-6s), P1 climb 1m, P2 hover, P3 descend.
**Flow hold:** PD position hold with dead-reckoned flow integration + safeguards.
**Alt hold:** PID with D-term velocity damping (KD=0.8).
**Status:** waiting for replacement propellers.
