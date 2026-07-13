# Mission FSM and RC SOP - flight.rs

This document describes the mission state machine that replaces the linear
`staircase_mission` in `src/bin/flight.rs`, the TX15 channel map driving
it, and the standard operating procedure for ground-test flights.

## 1. TX15 channel map

ELRS / CRSF carries 16 channel values per packet. There is no native concept
of a "maneuver" or "mode" command in the protocol. We assign meaning to each
channel in firmware. The physical EdgeTX mix on the TX15 model must match
the table below before the binary is flashed.

| CRSF | TX15 source        | Role                                           | Detection           |
|------|--------------------|------------------------------------------------|---------------------|
| CH1  | RH stick horiz (A) | Roll setpoint                                  | continuous          |
| CH2  | RH stick vert  (E) | Pitch setpoint                                 | continuous          |
| CH3  | LH stick vert  (T) | Throttle / thrust setpoint                     | continuous          |
| CH4  | LH stick horiz (R) | Yaw rate setpoint                              | continuous          |
| CH5  | SE  (2-pos button) | KILL  (latched disarm)                         | edge from baseline  |
| CH6  | SA  (3-pos)        | MODE: Idle (Low) / Manual (Mid) / Auto (High)  | level + on-ground gate |
| CH7  | SB  (3-pos)        | SELECT: Takeoff (Low) / Hover (Mid) / Land (High) | level (sampled at trigger time) |
| CH8  | SC  (3-pos)        | reserved -- decoded by sniffer, ignored by FSM | (logged only)       |
| CH9  | SD  (3-pos)        | TRIGGER (HIGH = press; flip up to fire, back down to re-arm edge) | rising edge to High |
| CH10 | SF  (button)       | RESTART (`SCB::sys_reset`)                     | edge from baseline  |

**EdgeTX prerequisite.** The TX15's default mix on this radio aligns CH5..CH9
to SE/SA/SB/SC/SD; SF, T1-T4 trim buttons, S1/S2 pots, and the 1-6 function
buttons are **not** mixed by default. To enable RESTART, the operator must
add `CH10 = source SF` in *Model Setup -> Mixes*. Until that mix exists,
CH10 reads a constant default value -- safe (no spurious restarts) but
the SF button does nothing.

To verify any other TX15 control (T1-T4, 1-6, S1, S2) on the bench, mix
it onto CH11..CH16 in the same screen, power-cycle the TX, and the
`rc_data` sniffer will report changes via its generic AUX tracker. The
FSM does not consume CH11+ -- those are operator-defined extras.

**ELRS switch mode prerequisite.** Even with the EdgeTX mix in place,
CH9..CH16 do not reach the FC unless the internal ELRS module is set to
*12ch wide* (or *Hybrid 16ch* on newer firmware). The default
*Hybrid 8ch* truncates to CH1..CH8 + a packed AUX byte. To change:
*SYS -> Tools -> ExpressLRS -> Switch Mode = 12ch wide*. SF on CH10,
S1/S2 on CH11..CH12 etc. will only show up after this. If a mix looks
correct in EdgeTX but the sniffer prints nothing on the corresponding
channel, switch mode is the first thing to check. SF on the TX15 is a
true momentary push-button -- a single press emits both press and
release transitions in the log; both are normal.

Position decoding uses the standard CRSF 11-bit calibration (172 / 992 / 1811).
A 3-pos switch maps to `Pos::Low / Mid / High`; a 2-pos to `Low / High`.

### MODE is a level, TRIGGER is an edge

This distinction matters for the operator:

- **MODE (SA)** is *level-triggered*. The FSM samples SA's current position
  on every tick from `GroundIdle`. So the operator can park SA in MID
  (Manual) before powering up, and the drone enters Manual the moment the
  on-ground gate clears - no need to wiggle the switch.
- **TRIGGER (SD)** is *edge-triggered*. Only a Low/Mid->High transition
  fires a maneuver. SD is a 3-pos switch, used as a "stiff momentary":
  flip up to High to fire, flip back down to Low/Mid to re-arm the edge.
  Holding SD in High does nothing after the initial edge.
- **SELECT (SB)** is sampled at the moment TRIGGER fires. So the operator
  pre-selects the maneuver on SB, then flips SD up to fire it.

### Why this split

- The TX15 mix on this radio has SE/SA/SB/SC/SD on CH5..CH9. Treating SD
  as the single trigger source means a single switch fires any number of
  maneuvers - we don't need one AUX channel per action.
- MODE on a 3-pos switch with a low position ("Idle") forces the operator
  to pass through Idle before flipping between Manual and Auto. This is
  exactly the on-ground gating rule: every flying state is reached *via*
  GroundIdle, never directly from another flying state.

## 2. State machine

There is **one** `GroundIdle` node. All flying paths return to it, and
from it either Manual or Auto is reachable. Re-entering Manual after an
Auto sequence is just `AutoLand -> GroundIdle -> Manual`. There is no
fast path between Manual and Auto: the round trip always passes through
GroundIdle, which re-checks the on-ground gate every time.

```
                          ┌───────────────────────────┐
                          │        GroundIdle         │◄─────────┐
                  ┌──────►│  disarmed, sp=0,          │          │
                  │       │  sticks ignored           │          │
                  │       └─┬───────────────────────┬─┘          │
                  │         │                       │            │
   exit Manual:   │         │ MODE=Manual           │ MODE=Auto  │
   MODE=Idle AND  │         │ + on-ground gate      │ + Trigger  │
   throttle idle  │         │                       │ + Select=  │
   for 500 ms     │         ▼                       │   Takeoff  │
                  │      Manual                     │ + on-ground│
                  │     (sticks own roll/pitch/     │   gate     │
                  │      yaw/thrust)                ▼            │
                  │         │                  AutoTakeoff       │
                  │         │                  (P0 ramp +        │
                  └─────────┘                   P1 climb 0->1m)  │
                                                     │           │
                                                     ▼           │
                                              ┌─►  AutoHover ────┤ MODE=Idle
                                              │  (hold 1.0 m,    │
                                Trigger +     │   timeout 8 s)   │
                                Select=Hover  │       │          │
                                (refresh      │       │          │
                                 timeout)     │       │ Trigger+Select=Land,
                                              │       │ OR hover timeout,
                                              │       │ OR MODE=Idle
                                              │       ▼          │
                                              │   AutoLand ──────┤
                                              │   (descent +     │
                                              │    ground detect)│
                                              │       │          │
                                              │       ▼          │
                                              └── GroundIdle ────┘
                                                  (single node;
                                                   shown again for
                                                   layout only)

  Hard transitions:
    SE edge:           latched permanent kill (handled in rc_kill task)
    flip_kill:         az > 3 m/s^2 for 10 ms
    gyro_runaway_kill: any axis > 5 rad/s for 50 ms
    RC link lost:      seq stale > 500 ms while armed -> failsafe:
                         Manual         -> Fault (disarm immediately)
                         AutoTakeoff/Hover -> AutoLand (autonomous descent)
                         AutoLand       -> continue (already landing)
                         GroundIdle     -> stay (no flight to safe)

  Hard transition (any state -> reboot):
    SF edge:           SCB::sys_reset (handled in rc_kill task)
```

> The diagram shows two GroundIdle boxes for visual flow only - they refer
> to the same single state. There is no second idle node in the code.

### State table

| State        | Owns setpoints?           | Armed | Entry condition                               | Exit condition                                       |
|--------------|---------------------------|-------|-----------------------------------------------|------------------------------------------------------|
| GroundIdle   | yes (zeros)               | no    | boot, or any flying state ends                | MODE=Manual + on-ground gate, OR MODE=Auto + Trigger + Select=Takeoff + on-ground gate |
| Manual       | yes (sticks)              | yes   | from GroundIdle, MODE=Manual                  | MODE=Idle AND throttle stick idle for 500 ms         |
| AutoTakeoff  | yes (P0 ramp + P1 climb)  | yes   | from GroundIdle, MODE=Auto + Trigger + Select=Takeoff | climb ramp complete                          |
| AutoHover    | yes (alt = 1.0 m)         | yes   | takeoff done, OR Trigger + Select=Hover from AutoLand setpoint not yet at ground | Trigger + Select=Land, OR hover timeout, OR MODE=Idle |
| AutoLand     | yes (descent + ground)    | yes   | hover exit                                    | ground detected (lidar < 0.15 m, 300 ms) or descent timeout (5 s) |
| Fault        | yes (zeros + disarm)      | no    | safety task fired (kill, runaway, link loss)  | manual SF reboot, or auto-return to GroundIdle once disarmed and on ground |

### On-ground gate

A flying-state entry is only honored when **all** of:

- `state == GroundIdle`
- `armed == false` (mirrors `MotorsState::Disarmed*`)
- `lidar < 0.10 m`
- `RC_LINK_READY`
- `AHRS_READY` (drone level, attitude settled)

Any switch edge or level mismatch arriving while not on the ground is
logged and discarded. The Manual round-trip works as follows:

1. Operator in Manual at altitude.
2. Lands manually (throttle to idle, drone descends, lidar < 0.10 m).
3. Operator flips SA -> LOW (Idle). FSM observes throttle-idle dwell of
   500 ms AND MODE=Idle, transitions Manual -> GroundIdle, disarms.
4. Operator now flips SA -> HIGH (Auto), pre-selects SB, flips SD up to
   HIGH. GroundIdle re-evaluates the gate (clean) and transitions to
   AutoTakeoff.

There is no `Manual -> AutoTakeoff` direct edge in the FSM. The forced
detour through GroundIdle is the safety property.

### Single setpoint owner

The FSM is the **only** task that writes:

- `signals::TRUE_RATE_SP`
- `signals::TRUE_ATTITUDE_Q_SP`
- `signals::TRUE_Z_THRUST_SP`
- `alt_hold::ALTITUDE_SETPOINT`

Stick-to-setpoint conversion happens *inside* the `Manual` state, not in a
separate task. Auto-mode setpoint ramps happen inside the Auto states.
This avoids races on the same `Watch` channel.

`flow_hold` continues to write `TRUE_ATTITUDE_Q_SP` while airborne, which
is a known cooperative writer with its own activation gate (lidar > 25 cm
+ flow quality > 60). In Auto modes flow_hold's roll/pitch correction is
welcome. In Manual mode the sticks will fight flow; for first manual
flights flow_hold should be disabled (a separate decision, tracked
elsewhere).

## 3. Standard Operating Procedure

### 3.1 Pre-flight (battery NOT plugged in)

1. Drone on its corks-and-chopsticks landing cage on a textured floor
   (newspaper or similar).
2. TX15 on, model `H743v2-free` selected.
3. SE in **LOW** baseline (any LOW->HIGH or HIGH->LOW transition is a kill,
   so the physical position is whichever the operator started in last
   session - what matters is that it does not change at boot).
4. SF in **LOW** baseline.
5. SA in **LOW** (Idle).
6. SB in **LOW** (Takeoff selected, default).
7. SD in **LOW** (trigger released).
8. All gimbals centered. Throttle stick fully down.
9. 1.5-2 m clear on all sides of the drone.

### 3.2 Power-up

1. Plug in battery. Drone boots, runs IMU cal (3-5 s), AHRS settle (~5 s).
2. `[free] all tasks spawned` should appear on UART within ~2 s.
3. Wait for `[ahrs] ready` (drone must be level: `< 3 deg` roll/pitch).
4. Wait for `[rc_kill] baseline SE=... SF=...` confirming the RC link.
5. **RC pre-flight gate.** The FSM refuses to enter GroundIdle until *every*
   one of the following is true on the live CRSF stream:
   - SA = LOW (Mode=Idle)
   - SB = LOW (Select=Takeoff)
   - SD not in HIGH (Trigger released)
   - Throttle stick at the bottom (within 2% of idle)
   - Roll/pitch/yaw sticks centered (within 10% of center)

   While any control is out of position, the FSM logs every 2 s:
   `[fsm] preflight WAIT: mode=BAD sel=ok trig=ok thr=BAD sticks=ok ...`
   The operator must physically correct the offending control. There is
   no override and no timeout. SE/SF baseline positions are *not* part
   of this gate -- they are recorded by `rc_kill` as whatever they were
   at the first packet, so any subsequent flip triggers kill / restart.

   **If `stk=BAD` persists with all sticks visibly centered**, the TX
   stick calibration is offset. Re-calibrate in EdgeTX (*Model Setup ->
   Inputs -> Calibrate*) until `rc_data` reports raw values within
   ~100 LSB of CRSF_MID (992) when sticks are at rest. Alternatively,
   widen `STICK_CENTER_TOL` in `mission_fsm_task` if a particular
   radio's centring is consistently off.
6. Once all controls are in position, log: `[fsm] preflight: all controls
   in safe position, proceeding`. The drone enters `GroundIdle`. Status
   LED: green slow blink.

### 3.3 Flight: Auto takeoff -> hover -> land

1. Confirm SE / SF haven't moved from baseline.
2. SB in **LOW** (Takeoff selected).
3. Flip SA: LOW -> HIGH (Auto).
   - Log: `[fsm] mode=Auto (on-ground gate OK)`.
4. **Flip SD up to HIGH** (then back down to LOW to re-arm the edge).
   - Log: `[fsm] trigger Takeoff`.
   - Drone arms (`[mtr] arming` -> `[mtr] armed-idle` -> `[mtr] [...]`).
   - Climbs to 1.0 m over 4 s (1 s thrust ramp + 3 s alt ramp).
5. Drone hovers at 1.0 m for up to `AUTO_HOVER_TIMEOUT_S` (8 s default).
   While hovering the operator can:
   - SB -> MID + SD->HIGH edge: refresh hover timer (stays at 1.0 m for another 8 s).
   - SB -> HIGH + SD->HIGH edge: trigger AutoLand immediately.
   - SA -> LOW: trigger AutoLand (graceful degrade to "land now").
   - Do nothing: AutoLand fires automatically at timeout expiry.
6. AutoLand: 1 s ramp 1.0 m -> 0.25 m, then 1 s ramp 0.25 m -> 0.05 m.
   Drone disarms when lidar reads < 0.15 m for 300 ms, or after 5 s.
7. State returns to `GroundIdle`. To run again: SA -> LOW, SB -> LOW,
   then back to step 3.

### 3.4 Flight: Manual (ACRO / rate mode)

Manual is **pure ACRO** -- self-levelling is OFF. All three stick axes
command angular *rates*, not target attitudes. Sticks released = drone
holds whatever attitude it's at. This is the same control scheme used
by Liftoff and most freestyle drones; Joshua Bardwell's standard
learn-to-fly path uses ACRO from day one.

While in Manual the FSM sets `MANUAL_BYPASS=true` and
`angle_to_rate_bridge` skips its own write, so the sticks own
`TRUE_RATE_SP` directly without racing the angle controller.

1. Drone in `GroundIdle`. Throttle stick fully DOWN. SE / SF baseline.
2. Flip SA: LOW -> MID (Manual).
   - Log: `[fsm] GroundIdle -> Manual (gate OK)`.
   - Drone arms when throttle stick raises above 5%.
3. Stick mapping (body-frame angular rates, expo 0.3):
   - Roll  (CH1):  +/- 3.0 rad/s (~170 deg/s)
   - Pitch (CH2):  +/- 3.0 rad/s
   - Yaw   (CH4):  +/- 2.0 rad/s
   - Thrust (CH3): 0 -> `BASE_THRUST * 1.4` (max ~11.2)
4. **Bench-debug log** at 5 Hz while in Manual:

   ```
   [manual] r=+0.05 p=+0.00 y=-0.00 thr=0.00 | rate r=+0.04 p=+0.00 y=-0.00 thrust=0.0 armed=0
   ```

   `r/p/y/thr` are normalised stick values [-1, 1] / [0, 1].
   `rate r/p/y` are the rad/s rate setpoints sent to `controller_rate`.
   `thrust` is the thrust setpoint. `armed` is 0 or 1.
5. To exit Manual: throttle stick fully DOWN, then flip SA -> LOW (Idle).
   - Drone disarms when throttle stick has been at idle for 500 ms AND
     MODE=Idle.
   - State returns to `GroundIdle`.

**Bench verification (props OFF) recommended before any flight:**

- Boot, get to GroundIdle.
- Flip SA to MID, raise throttle slightly to arm motors (they'll
  spin -- props OFF).
- Push roll stick right: motors on the right should spin slower,
  motors on the left should spin faster (right-roll = roll right).
- Push pitch stick forward (down on the gimbal): rear motors spin
  faster, front motors slower (pitch forward = nose down).
- Push yaw stick right: diagonal pair changes RPM (depends on motor
  spin direction; Betaflight-style mix is documented in
  `motor_governor`).
- The `[manual]` log line lets you confirm the rate setpoints are
  the sign and magnitude you expect.

### 3.5 Round-trip: Manual <-> Auto

To switch from Manual to Auto mid-session:

1. Land manually (throttle down) until drone is on the ground.
2. Flip SA -> LOW (Idle). Wait for "[fsm] state=GroundIdle".
3. Flip SB to the desired auto maneuver (typically LOW = Takeoff).
4. Flip SA -> HIGH (Auto). Flip SD up to HIGH (then back down).

To switch from Auto to Manual mid-session:

1. Wait for AutoLand to disarm and return to GroundIdle (or trigger
   AutoLand early via Trigger + Select=Land, or SA -> LOW).
2. Flip SA -> MID (Manual).
3. Manual mode active; drone arms when throttle stick rises.

There is **no** mid-air mode swap. Every transition between Manual and
Auto necessarily includes a landing.

### 3.6 Emergency

| Situation                                       | Action                                |
|-------------------------------------------------|---------------------------------------|
| Drone misbehaving in any way                    | **Flip SE.** Kill latched. Power-cycle to recover. (See note below.) |
| Need to soft-reboot firmware after kill         | **Flip SF.** *Currently unreliable post-SE; investigate task #3.* |
| Drone has flipped on its back                   | flip_kill fires at az > 3 m/s^2 for 10 ms |
| Gyro runaway                                    | gyro_runaway_kill fires at any axis > 5 rad/s for 50 ms |
| RC link lost mid-flight                         | Failsafe: Manual disarms immediately (Fault); Auto* triggers AutoLand; AutoLand continues. Recovery: re-establish link, park SA in LOW, drone returns to GroundIdle. |

**Note on SF:** in current testing SF only restarts the firmware *before*
SE has been triggered. Once SE is latched, SF appears not to reset the
MCU. Until that's diagnosed, treat SE as "kill + power cycle required",
not "kill + try SF to recover".

## 4. Implementation notes

### 4.1 RC frontend (`src/rc_kill.rs`)

`rc_kill_task` owns USART6 (the only CRSF reader on the board) and
provides:

- A static `RC_CHANNELS: Watch<...>` publishing the latest decoded
  channel state every CRSF frame. Receivers: the FSM, optional debug
  loggers.
- A static `RC_EVENT: Channel<..., RcEvent, 8>` carrying edge events
  (mode changed, select changed, trigger pressed). The FSM consumes
  these via `try_receive` so it never blocks on RC.
- SE kill (latched disarm) and SF restart (`SCB::sys_reset`) on raw u16
  hysteresis from baseline (100 LSB), so they fire even for button
  outputs that don't cross the 3-pos LOW/MID/HIGH thresholds.

**Custom inline CRSF framer.** The crate's
`common::parsers::crsf::CrsfParser` was confirmed to halt the H743v2
thread executor on production CRSF traffic from the RP3 receiver
(reproduced D000186/189/191/194/196; static analysis did not identify
the exact panic site). `rc_kill.rs` therefore uses a small
self-contained CRSF framer (`CrsfFramer` in the same file): SYNC = 0xC8,
length 2..=62, CRC8 polynomial 0xD5 over [type, payload], type 0x16 =
RcChannelsPacked. Other CRSF frame types (LinkStatistics 0x14,
telemetry, etc.) are CRC-validated and dropped silently. The 16
channels are decoded by the public `rc_channels_packed::raw_decode`
function (which is straightforward bit-packing math, not the broken
state machine).

**Sensor-init gate.** `rc_kill_task` waits for `SENSORS_READY ==
SENSOR_COMPASS_BIT | SENSOR_BARO_BIT | SENSOR_BMI270_BIT` before
`UartRx::new` (5 s timeout fallback). This avoids AHB-matrix bus
contention between USART6 RX DMA and the I2C2 / SPI3 sensor inits
that otherwise wedges the inits on a quiet boot. See the
`project_h743_uart_i2c_contention` memory entry for the diagnosis.

### 4.2 Mission FSM (`src/bin/flight.rs`)

Replaces the linear `staircase_mission` task. Single `enum FsmState`,
single outer loop:

```text
loop {
    let next = match state {
        GroundIdle  => ground_idle_tick(...).await,
        Manual      => manual_tick(...).await,
        AutoTakeoff => auto_takeoff_tick(...).await,
        AutoHover   => auto_hover_tick(...).await,
        AutoLand    => auto_land_tick(...).await,
        Fault       => fault_tick(...).await,
    };
    if next != state { log_transition(state, next); on_exit(state); on_enter(next); }
    state = next;
}
```

State `*_tick` functions:

- Drive their setpoints at the cadence appropriate to the state
  (1 kHz for sticks, 50 Hz for ramps).
- Poll `RC_EVENT::try_receive()` and current `RC_CHANNELS` levels.
- Return either the same state (continue) or the next state.
- Have their own internal timers (entry instant, ramp progress).

### 4.3 What is NOT in this change

- IWDG watchdog reset (independent SF reset path) - separate task #3.
- Disabling `flow_hold` in Manual mode - first manual flights should
  disable flow_hold by commenting its spawn in main(), per a separate
  decision.
