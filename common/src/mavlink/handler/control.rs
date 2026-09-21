use mavio::{
    Frame,
    default_dialect::{enums::AttitudeTargetTypemask, messages::SetAttitudeTarget},
    protocol::MaybeVersioned,
};
use nalgebra::{Quaternion, UnitQuaternion};

use crate::{
    mavlink::{MavlinkServer, params::Identity},
    vehicle::{SetFlightMode, Vehicle},
};

impl<V: MaybeVersioned> super::Handler<V> for SetAttitudeTarget {
    async fn handle_inner(
        self,
        server: &mut MavlinkServer,
        _: Frame<V>,
    ) -> Result<(), crate::mavlink::Error> {
        let target = Identity {
            sys: self.target_system,
            com: self.target_component,
        };

        if !server.param.id.is_target_of(target) {
            return Ok(());
        }

        if !self.type_mask.intersects(
            AttitudeTargetTypemask::THROTTLE_IGNORE | AttitudeTargetTypemask::THRUST_BODY_SET,
        ) {
            Vehicle::set_thrust_setpoint(self.thrust);
        }

        if !self
            .type_mask
            .intersects(AttitudeTargetTypemask::ATTITUDE_IGNORE)
        {
            // NOTE: MAVLink uses [w, x, y, z] nalgebra uses [x, y, z, w] ordering
            let att = UnitQuaternion::from_quaternion(Quaternion::new(
                self.q[3], self.q[0], self.q[1], self.q[2],
            ));

            Vehicle::set_attitude_setpoint(att);

            return Ok(());
        }

        if !self.type_mask.intersects(
            AttitudeTargetTypemask::BODY_ROLL_RATE_IGNORE
                | AttitudeTargetTypemask::BODY_PITCH_RATE_IGNORE
                | AttitudeTargetTypemask::BODY_YAW_RATE_IGNORE,
        ) {
            let rate = [
                self.body_roll_rate,
                self.body_pitch_rate,
                self.body_yaw_rate,
            ];

            Vehicle::set_angular_rate_setpoint(rate);

            return Ok(());
        }

        Ok(())
    }
}
