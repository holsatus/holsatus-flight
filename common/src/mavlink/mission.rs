use mavio::dialects::common::messages::MissionItemInt;

use crate::errors::MavlinkError;

pub async fn handle_mission_item_int(mission: MissionItemInt) -> Result<(), MavlinkError> {
    // TODO: Implement mission protocol
    match mission.command {
        // MavCmd::MAV_CMD_MISSION_START => todo!(),
        // MavCmd::MAV_CMD_NAV_WAYPOINT => todo!(),
        // MavCmd::MAV_CMD_NAV_LAND_LOCAL => todo!(),
        // MavCmd::MAV_CMD_NAV_TAKEOFF_LOCAL => todo!(),

        // MavCmd::MAV_CMD_NAV_SET_YAW_SPEED => todo!(),
        // MavCmd::MAV_CMD_DO_CHANGE_SPEED => todo!(),

        // MavCmd::MAV_CMD_CONDITION_DELAY => todo!(),
        // MavCmd::MAV_CMD_CONDITION_DISTANCE => todo!(),

        // MavCmd::MAV_CMD_DO_SET_HOME => todo!(),
        _ => error!("Unknown mission command: {:?}", mission.command as u32),
    }

    Ok(())
}
