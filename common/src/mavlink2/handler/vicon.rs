use mavio::{default_dialect::messages::ViconPositionEstimate, prelude::MaybeVersioned, Frame};

use crate::{
    mavlink2::MavlinkServer, signals::VICON_POSITION_ESTIMATE, types::measurements::ViconData,
};

impl<V: MaybeVersioned> super::Handler<V> for ViconPositionEstimate {
    async fn handle_inner(
        _server: &mut MavlinkServer,
        msg: Self,
        _: Frame<V>,
    ) -> Result<(), crate::mavlink2::Error> {
        // TODO Refer to read-out implementation on XPS

        debug!("[mavlink] Received Vicon position estimate");

        let c = &msg.covariance;

        let pos_var = nalgebra::matrix![
            c[ 0], c[ 1], c[ 2];
            c[ 1], c[ 6], c[ 7];
            c[ 2], c[ 7], c[11];
        ];

        let att_var = nalgebra::matrix![
            c[15], c[16], c[17];
            c[16], c[18], c[19];
            c[17], c[19], c[20];
        ];

        VICON_POSITION_ESTIMATE.send(ViconData {
            timestamp_us: msg.usec,
            position: [msg.x, msg.y, msg.z],
            attitude: [msg.roll, msg.pitch, msg.yaw],
            pos_var: pos_var.data.0,
            att_var: att_var.data.0,
        });

        Ok(())
    }
}
