use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Duration;
use heapless::spsc::Queue;
use mavlink::holsatus::{MavCmd, MISSION_ITEM_INT_DATA};

use crate::{errors::MavlinkError, geo::Waypoint3D};

struct MissionCommand {
    /// The index of the command in the mission (monotonic)
    index: u16,
    /// The MAVLink command ID
    cmd_id: u16,
    /// The MAVLink command parameters
    content: Content,
}

enum Content {
    Waypoint(MissionWaypoint),
    LoiterUnlim,
    LouterTurns,
    LoiterTime,
    ReturnToLaunch,
    Land,
}

// mission state enumeration
enum State {
    Stopped,
    Running,
    Complete
}

struct Mission {
    state: State,
    current_cmd: MissionCommand,
    content: Content,
}

struct MissionWaypoint {
    location: Waypoint3D,
    hold_time: Duration,
    accept_radius: f32,
    pass_radius: f32,
}

struct MissionStart {

}

static MISSION: Mutex<CriticalSectionRawMutex, Option<Mission>> = Mutex::new(None);

pub async fn handle_mission_item_int(mission: MISSION_ITEM_INT_DATA) -> Result<(), MavlinkError> {

    match mission.command {
        MavCmd::MAV_CMD_MISSION_START => todo!(),
        MavCmd::MAV_CMD_NAV_WAYPOINT => todo!(),
        MavCmd::MAV_CMD_NAV_LAND_LOCAL => todo!(),
        MavCmd::MAV_CMD_NAV_TAKEOFF_LOCAL => todo!(),

        MavCmd::MAV_CMD_NAV_SET_YAW_SPEED => todo!(),
        MavCmd::MAV_CMD_DO_CHANGE_SPEED => todo!(),

        MavCmd::MAV_CMD_CONDITION_DELAY => todo!(),
        MavCmd::MAV_CMD_CONDITION_DISTANCE => todo!(),

        MavCmd::MAV_CMD_DO_SET_HOME => todo!(),
        _ => error!("Unknown mission command: {:?}", mission.command as u32),
    }

    Ok(())
}