use crate::signals as s;
use kalman_filter::kalman::KalmanFilter;
use nalgebra::{Matrix1, Matrix2, Vector2, Vector3};

const TD: f32 = crate::POS_LOOP_DIV as f32 / crate::MAIN_LOOP_FREQ as f32 / 10.;

const A: Matrix2<f32> = Matrix2::new(1., TD, 0., 1.);

const B: Vector2<f32> = Vector2::new(0.5 * TD * TD, TD);
const C: Vector2<f32> = Vector2::new(1., 0.);

#[embassy_executor::task]
pub async fn main() -> ! {
    static STR_ID: &str = "position_estimator";
    info!("{}: Task started", STR_ID);

    let mut rcv_gnss_data = s::RAW_GNSS_DATA.receiver().unwrap();
    let mut rcv_attitude_q = s::AHRS_ATTITUDE_Q.receiver().unwrap();
    let mut rcv_imu_data = s::CAL_IMU_DATA.receiver().unwrap();

    let snd_estimated_pos = s::ESTIMATED_POS.sender();
    let snd_estimated_vel = s::ESTIMATED_VEL.sender();

    let mut kalman_x = KalmanFilter::<2, 1, _>::new(
        A,
        Some(B),
        Matrix2::identity().scale(0.1),
        Vector2::new(0.0, 0.0),
        Matrix2::identity().scale(0.01),
    );
    let mut kalman_y = KalmanFilter::<2, 1, _>::new(
        A,
        Some(B),
        Matrix2::identity().scale(0.1),
        Vector2::new(0.0, 0.0),
        Matrix2::identity().scale(0.01),
    );
    let mut kalman_z = KalmanFilter::<2, 1, _>::new(
        A,
        Some(B),
        Matrix2::identity().scale(0.1),
        Vector2::new(0.0, 0.0),
        Matrix2::identity().scale(0.01),
    );

    info!("{}: Entering main loop at {} Hz", STR_ID, 1. / TD);
    loop {
        let gnss_data = rcv_gnss_data.changed().await;
        let (pos_x, pos_y, altitude) = (gnss_data.lon_raw, gnss_data.lat_raw, gnss_data.altitude);

        // for i in 0..10 {

        //     let attitude_q = rcv_attitude_q.changed().await;
        //     let imu_data = rcv_imu_data.get().await;

        //     if i != 9 {
        //         let state_x = kalman_x.get_state();
        //         let state_y = kalman_y.get_state();
        //         let state_z = kalman_z.get_state();

        //         comp_x = alpha * ( state_x[1] * TD + comp_x )+ (1.0 - alpha) * state_x[0];
        //         comp_y = alpha * ( state_y[1] * TD + comp_y )+ (1.0 - alpha) * state_y[0];
        //         comp_z = alpha * ( state_z[1] * TD + comp_z )+ (1.0 - alpha) * state_z[0];

        //         snd_estimated_pos.send([comp_x, comp_y, comp_z]);
        //         snd_estimated_vel.send([state_x[1], state_y[1], state_z[1]]);
        //     }
        // }

        let attitude_q = rcv_attitude_q.changed().await;
        let imu_data = rcv_imu_data.get().await;

        let linear_acceleration =
            attitude_q * Vector3::from(imu_data.acc) + Vector3::new(0.0, 0.0, 9.81);

        kalman_x.predict_with_input(Matrix1::new(linear_acceleration[0]));
        kalman_y.predict_with_input(Matrix1::new(linear_acceleration[1]));
        kalman_z.predict_with_input(Matrix1::new(linear_acceleration[2]));

        kalman_x.update(
            &(C.transpose()),
            &Matrix1::new(6.0),
            &Matrix1::new(pos_x as f32),
        );
        kalman_y.update(
            &(C.transpose()),
            &Matrix1::new(6.0),
            &Matrix1::new(pos_y as f32),
        );
        kalman_z.update(
            &(C.transpose()),
            &Matrix1::new(6.0),
            &Matrix1::new(-altitude),
        );

        let state_x = kalman_x.get_state();
        let state_y = kalman_y.get_state();
        let state_z = kalman_z.get_state();

        snd_estimated_pos.send([state_x[0], state_y[0], state_z[0]]);
        snd_estimated_vel.send([state_x[1], state_y[1], state_z[1]]);
    }
}
