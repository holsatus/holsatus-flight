//! Commander module
//!
//!

use crate::signals as s;

use crate::{signals::{CALIBRATOR_STATE, CMD_CALIBRATE}, sync::{procedure::Procedure, watch::Watch}, tasks::calibrator::CalibratorState};
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};
use mutex::raw_impls::cs::CriticalSectionRawMutex as M;

const CHANNEL_LEN: usize = 4;

pub mod message;
pub use message::{Command, Origin, Request, Response};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

use crate::tasks::configurator2::CONFIGURATOR;

/// Channel for sending commands to the [`Commander`] task
///
/// In order for sub-systems (like a GCS server) to get some feedback
/// from the command, this is implemented using the [`Procedure`] type,
/// which has the [`Procedure::request`] method that waits for a response.
/// If waiting for a response is not necessarry, like commands from an
/// RC transmitter, just use [`Procedure::send`] instead.
pub static PROCEDURE: Procedure<M, Request, Response, CHANNEL_LEN> = Procedure::new();

#[derive(Schema, Serialize, Deserialize)]
struct CommanderConfig {
    ticker_dur_ms: u16,
}

impl Default for CommanderConfig {
    fn default() -> Self {
        CommanderConfig { ticker_dur_ms: 500 }
    }
}

/// The main commander task
struct Commander {
    name: &'static str,
    ticker: Ticker,
    disarm_info: DisarmInfo,
    actuator_override_active: bool,
}

/// Information about the last disarm event
struct DisarmInfo {
    time: Instant,
    origin: Origin,
}

impl Commander {
    fn new(config: CommanderConfig) -> Self {
        Self {
            name: "commander",
            ticker: Ticker::every(Duration::from_millis(config.ticker_dur_ms as u64)),
            disarm_info: DisarmInfo {
                time: Instant::MIN,
                origin: Origin::Unspecified,
            },
            actuator_override_active: false,
        }
    }
}

impl Default for Commander {
    fn default() -> Self {
        Self::new(CommanderConfig::default())
    }
}

#[embassy_executor::task]
pub async fn commander_entry_task() -> ! {
    commander_entry().await
}

pub async fn commander_entry() -> ! {
    let config = CONFIGURATOR.load_or_default("commander").await;

    let mut commander = Commander::new(config);
    CMD_ARM_VEHICLE.send(false);

    commander.main_loop().await
}

impl Commander {
    async fn main_loop(&mut self) -> ! {
        trace!("[{}] Starting main loop", self.name);
        loop {
            match select(PROCEDURE.receive_request(), self.ticker.next()).await {
                Either::First((command, handle)) => {
                    let response = self.handle_command(command);
                    handle.respond(response)
                }
                Either::Second(()) => self.run_periodics(),
            }
        }
    }

    /// Handle an incoming command.
    ///
    /// This is intentionally NOT async, to enforce that no request can
    /// hold up the commander. If some action takes time, it should be
    /// delegated to another task, and if some action is temporarily
    /// rejected, this should just be reflected in the response.
    fn handle_command(&mut self, request: Request) -> Response {
        trace!("[{}] Handling command: {:?}", self.name, request);
        match request.command {
            Command::ArmDisarm(cmd) => match cmd.arm {
                true => self.arm_vehicle(cmd.force),
                false => self.disarm_vehicle(cmd.force, request.origin),
            },
            Command::DoCalibration(cmd) => {
                use crate::calibration::{Calibrate, AccCalib, GyrCalib};

                if CMD_ARM_VEHICLE.is(&true) {
                    error!("[{}] Cannot calibrate when armed", self.name);
                    return Response::Rejected;
                }

                if !CALIBRATOR_STATE.is(&CalibratorState::Idle) {
                    error!("[{}] Another calibration is already in progress", self.name);
                }

                match cmd.sensor_type {
                    message::SensorType::Accelerometer => {
                        CMD_CALIBRATE.send(Calibrate::Acc(AccCalib::default(), cmd.sensor_id));
                        return Response::Accepted;
                    }
                    message::SensorType::Gyroscope => {
                        CMD_CALIBRATE.send(Calibrate::Gyr(GyrCalib::default(), cmd.sensor_id));
                        return Response::Accepted;
                    }
                    message::SensorType::Magnetometer => todo!("Not yet implemented"),
                    message::SensorType::Barometer => todo!("Not yet implemented"),
                }
            }
            Command::SetActuators { group, values } => {
                if !self.actuator_override_active {
                    error!("[{}] Actuator override not active", self.name);
                    return Response::Rejected;
                }

                if CMD_ARM_VEHICLE.is(&false) {
                    error!("[{}] Must be armed to override actuators", self.name);
                    return Response::Rejected;
                }

                match CMD_ACTUATOR_OVERRIDE.get(group as usize) {
                    None => {
                        error!("[{}] Invalid actuator group: {}", self.name, group);
                        Response::Unsupported
                    }
                    Some(cmd) => {
                        cmd.send(Some(values));
                        Response::Accepted
                    }
                }
            }
            Command::SetActuatorOverride { active } => {
                if active {
                    if STATUS_ON_GROUND.is(&false) {
                        error!("[{}] Vehicle must be on ground to override", self.name);
                        return Response::Rejected;
                    }
                    warn!("[{}] Actuator override activated", self.name);
                } else {
                    warn!("[{}] Actuator override deactivated", self.name);
                    CMD_ACTUATOR_OVERRIDE.iter().for_each(|cmd| cmd.send(None));
                }

                self.actuator_override_active = active;
                Response::Accepted
            }
            Command::RunArmChecks => match self.arm_checks() {
                true => Response::Accepted,
                false => Response::Rejected,
            },
            Command::SetControlMode { .. } => Response::Unsupported,
        }
    }

    /// Run periodic functions (2 Hz by default)
    fn run_periodics(&mut self) {
        trace!("[{}] Running periodic functions", self.name);
    }

    /// Run necessary checks and arm the vehicle
    fn arm_vehicle(&mut self, force: bool) -> Response {
        if CMD_ARM_VEHICLE.is(&true) {
            info!("[{}] Vehicle already armed", self.name);
            return Response::Unchanged;
        }

        if force {
            info!("[{}] Force arming vehicle", self.name);
            CMD_ARM_VEHICLE.send(true);
            return Response::Accepted;
        }

        if self.arm_skip_condition() || self.arm_checks() {
            CMD_ARM_VEHICLE.send(true);
            info!("[{}] Arming vehicle", self.name);
            Response::Accepted
        } else {
            warn!("[{}] Not arming vehicle", self.name);
            Response::Rejected
        }
    }

    /// Run necessary checks and disarm the vehicle
    fn disarm_vehicle(&mut self, force: bool, origin: Origin) -> Response {
        if CMD_ARM_VEHICLE.is(&false) {
            info!("[{}] Vehicle already disarmed", self.name);
            return Response::Unchanged;
        }

        let disarm_info = DisarmInfo {
            time: Instant::now(),
            origin,
        };

        if force {
            info!("[{}] Force disarming vehicle", self.name);
            CMD_ARM_VEHICLE.send(false);
            self.disarm_info = disarm_info;
            return Response::Accepted;
        }

        if self.disarm_checks() {
            CMD_ARM_VEHICLE.send(false);
            self.disarm_info = disarm_info;
            info!("[{}] Disarming vehicle", self.name);
            Response::Accepted
        } else {
            warn!("[{}] Not disarming vehicle", self.name);
            Response::Rejected
        }
    }

    /// Check for conditions to skip arming checks,
    /// e.g. if re-arming within a grace period
    fn arm_skip_condition(&self) -> bool {
        // Skip checks if manually disarmed within last 5 seconds
        if self.disarm_info.time.elapsed() < Duration::from_secs(5)
            && self.disarm_info.origin == Origin::RemoteControl
        {
            info!("[{}] Within 5 second arm grace period", self.name);
            true
        } else {
            false
        }
    }

    /// Execute a preflight check
    fn arm_checks(&self) -> bool {
        info!("[{}] Running arm checks", self.name);

        let Some(blocker) = s::ARMING_BLOCKER.try_get() else {
            warn!("[{}] Could not fetch ArmingBlocker flag", self.name);
            return false;
        };

        if STATUS_ON_GROUND.is(&false) {
            warn!("[{}] Vehicle not on ground", self.name);
            return false;
        }

        if !blocker.is_empty() {
            warn!("[{}] ArmingBlocker is not empty: {:?}", self.name, blocker);
            return false;
        }
        
        info!("[{}] Preflight checks passed", self.name);
        true
    }

    /// Execute a preflight check
    fn disarm_checks(&self) -> bool {
        trace!("[{}] Running disarm checks", self.name);

        let passed_checks = true;

        if passed_checks {
            info!("[{}] Disarm checks passed", self.name);
        } else {
            warn!("[{}] Disarm checks failed", self.name);
        }

        passed_checks
    }
}

/// Whether to arm the vehicle. True for arm, false for disarm.
static CMD_ARM_VEHICLE: Watch<bool, M> = Watch::new();

/// Whether the vehicle is on the ground or airborne
static STATUS_ON_GROUND: Watch<bool, M> = Watch::new();

/// Whether the vehicle is on the ground or airborne
pub static CMD_RATE_REF: Watch<&'static Watch<[f32; 4], M>, M> = Watch::new();
pub static IMU_PRIM_CAL: Watch<&'static Watch<[f32; 3], M>, M> = Watch::new();

const NUM_OUT_GROUPS: usize = 4;
pub static CMD_ACTUATOR_OVERRIDE: [Watch<Option<[f32; 4]>, M>; NUM_OUT_GROUPS] = {
    const REPEAT: Watch<Option<[f32; 4]>, M> = Watch::new();
    [REPEAT; NUM_OUT_GROUPS]
};

#[cfg(test)]
mod tests {
    use crate::tasks::commander::message::*;

    use super::*;

    #[test]
    fn arming_accepted() {
        let config = CommanderConfig::default();
        let mut commander = Commander::new(config);

        CMD_ARM_VEHICLE.send(false);
        STATUS_ON_GROUND.send(true);

        let request = (
            ArmDisarm {
                arm: true,
                force: false,
            },
            Origin::RemoteControl
        ).into();

        let response = commander.handle_command(request);
        assert_eq!(response, Response::Accepted);
    }

    #[test]
    fn arming_rejected() {
        let config = CommanderConfig::default();
        let mut commander = Commander::new(config);

        CMD_ARM_VEHICLE.send(false); // Disarmed
        STATUS_ON_GROUND.send(false); // In air

        let request = (
            ArmDisarm {
                arm: true,
                force: false,
            },
            Origin::Unspecified
        ).into();

        let response = commander.handle_command(request);
        assert_eq!(response, Response::Rejected);
    }

    #[test]
    fn arming_unchanged() {
        let config = CommanderConfig::default();
        let mut commander = Commander::new(config);

        CMD_ARM_VEHICLE.send(true);
        STATUS_ON_GROUND.send(false);

        let request = (
            ArmDisarm {
                arm: true,
                force: false,
            },
            Origin::RemoteControl
        ).into();

        let response = commander.handle_command(request);
        assert_eq!(response, Response::Unchanged);
    }
}
