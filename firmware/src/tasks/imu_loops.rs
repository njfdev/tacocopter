use embassy_futures::yield_now;
use embassy_rp::{
    i2c::{Async, I2c},
    peripherals::I2C1,
};
use embassy_time::{Instant, Timer};
use micromath::F32Ext;
use mpu6050::Mpu6050;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::{
    consts::{ACCEL_BIASES, GYRO_BIASES, UPDATE_LOOP_FREQUENCY, USB_LOGGER_RATE},
    global::{IMU_FETCH_FREQUENCY_SIGNAL, IMU_RAW_SIGNAL, IMU_SIGNAL, SHARED},
    tools::kalman::KalmanFilterQuat,
};

#[embassy_executor::task]
pub async fn mpu6050_fetcher_loop(mut mpu: Mpu6050<I2c<'static, I2C1, Async>>) {
    let mut last_loop = Instant::now();
    let mut last_log = Instant::now();
    loop {
        let time_to_wait = ((1_000_000.0 / UPDATE_LOOP_FREQUENCY) as u64)
            .checked_sub(last_loop.elapsed().as_micros());
        Timer::after_micros(if time_to_wait.is_some() {
            time_to_wait.unwrap()
        } else {
            0
        })
        .await;
        let frequency = 1_000_000.0 / last_loop.elapsed().as_micros() as f32;
        last_loop = Instant::now();

        let gyro_data: [f32; 3] = correct_biases(
            mpu.get_gyro().unwrap().as_slice().try_into().unwrap(),
            GYRO_BIASES,
        );
        let accel_data: [f32; 3] = correct_biases(
            mpu.get_acc().unwrap().as_slice().try_into().unwrap(),
            ACCEL_BIASES,
        );

        IMU_RAW_SIGNAL.signal((gyro_data, accel_data));

        if last_log.elapsed().as_millis() >= (1000.0 / USB_LOGGER_RATE) as u64 {
            last_log = Instant::now();
            IMU_FETCH_FREQUENCY_SIGNAL.signal(frequency);
        }
    }
}

#[embassy_executor::task]
pub async fn mpu6050_processor_loop() {
    // TODO: fix integration of gyro data (e.g. tilting, moving yaw, then tilting back changes yaw from starting point)
    let mut rotation_q = UnitQuaternion::identity();
    let mut kalman = KalmanFilterQuat::new();

    // let mut filtered_orientation: [f32; 3] = [0.0; 3];

    let imu_watch_sender = IMU_SIGNAL.sender();

    // let mut kalman_angle_roll: f32 = 0.0;
    // let mut kalman_angle_roll_uncertainty: f32 = 2.0.powi(2);
    // let mut kalman_angle_pitch: f32 = 0.0;
    // let mut kalman_angle_pitch_uncertainty: f32 = 2.0.powi(2);
    // let mut gyro_angle_yaw: f32 = 0.0;

    // the first index is the angle prediction, and the second is the kalman uncertainty
    // let mut kalman_output: [f32; 2] = [0.0; 2];

    let mut start = Instant::now();
    let mut since_last = Instant::now();
    let mut iterations = 0;
    loop {
        let dur_since = Instant::now().checked_duration_since(start);
        let time_to_wait = (((1_000_000.0 * iterations as f64) / (UPDATE_LOOP_FREQUENCY)) as i64)
            - dur_since.unwrap().as_micros() as i64;
        Timer::after_micros(if time_to_wait > 0 {
            time_to_wait as u64
        } else {
            0
        })
        .await;

        // fetch imu data from other task
        let (gyro_data, accel_data) = IMU_RAW_SIGNAL.wait().await;

        let ending = Instant::now();
        let delta = (ending
            .checked_duration_since(since_last)
            .unwrap()
            .as_micros() as f32)
            / 1_000_000.0;
        since_last = ending;
        kalman.update(
            Vector3::from_row_slice(&gyro_data),
            Vector3::from_row_slice(&accel_data),
            delta,
        );
        integrate_quaternion(
            gyro_data.as_slice().try_into().unwrap(),
            &mut rotation_q,
            delta,
        );

        // get euler angles
        let accel_angles = accel_to_angles(accel_data);
        let gyro_angles: [f32; 4] = rotation_q.as_vector().as_slice().try_into().unwrap();

        // filtered_orientation[0] = kalman_1d(
        //     &mut kalman_angle_pitch,
        //     &mut kalman_angle_pitch_uncertainty,
        //     gyro_data[0],
        //     accel_angles[0],
        //     delta,
        // )[0];
        // filtered_orientation[1] = kalman_1d(
        //     &mut kalman_angle_roll,
        //     &mut kalman_angle_roll_uncertainty,
        //     gyro_data[1],
        //     accel_angles[1],
        //     delta,
        // )[0];
        // filtered_orientation[2] = gyro_angles[2];
        let orientation_quaternion = kalman.q.as_vector().as_slice().try_into().unwrap();
        let orientation_vector = kalman.q.euler_angles();
        imu_watch_sender.send((
            gyro_data.try_into().unwrap(),
            orientation_vector,
            accel_data.try_into().unwrap(),
        ));

        // let mut should_start_gyro_calib = false;
        if Instant::now()
            .checked_duration_since(start)
            .unwrap()
            .as_millis()
            >= (1000.0 / USB_LOGGER_RATE) as u64
        {
            start = Instant::now();
            let mut shared = SHARED.lock().await;
            // shared.imu_sensor_data.gyroscope = gyro_data;
            // shared.imu_sensor_data.accelerometer = accel_data;
            shared.imu_sensor_data.accel_orientation =
                UnitQuaternion::from_euler_angles(accel_angles[0], accel_angles[1], 0.0)
                    .as_vector()
                    .as_slice()
                    .try_into()
                    .unwrap();
            shared.imu_sensor_data.gyro_orientation = gyro_angles;
            shared.imu_sensor_data.orientation = orientation_quaternion;
            shared.state_data.imu_process_rate = 1.0 / delta;
            shared.calibration_data.accel_calibration = accel_data;
            iterations = 0;
            //info!("Estimated rotation: {:?}", rotation);

            // should_start_gyro_calib = !shared.gyro_calibration_state.is_finished;
        }

        // TODO: make gyro calibration work again
        // // if need to start gyro calibration, do it
        // if should_start_gyro_calib {
        //     get_gyro_offsets(&mut mpu).await;
        //     start = Instant::now();
        // }

        iterations += 1;
    }
}

fn accel_to_angles(accel_data: [f32; 3]) -> [f32; 3] {
    let roll = (-accel_data[0]).atan2((accel_data[2].powi(2) + accel_data[1].powi(2)).sqrt());
    let sign = if accel_data[2] > 0.0 { 1.0 } else { -1.0 };
    let miu = 0.01;
    let pitch =
        accel_data[1].atan2(sign * (accel_data[2].powi(2) + miu * accel_data[0].powi(2)).sqrt());
    [pitch, roll, 0.0]
}

fn correct_biases(data: &[f32; 3], biases: [f32; 3]) -> [f32; 3] {
    let mut new_data = data.clone();
    new_data[0] -= biases[0];
    new_data[1] -= biases[1];
    new_data[2] -= biases[2];
    new_data
}

// TODO: better understand this
fn integrate_quaternion(new_angular_vel: &[f32; 3], q: &mut UnitQuaternion<f32>, delta: f32) {
    let omega = Quaternion::new(
        0.0,
        new_angular_vel[0],
        new_angular_vel[1],
        new_angular_vel[2],
    );
    let dq = 0.5 * q.as_ref() * omega;
    let new_q = Quaternion::new(
        q.w + dq.w * delta,
        q.i + dq.i * delta,
        q.j + dq.j * delta,
        q.k + dq.k * delta,
    );

    *q = UnitQuaternion::from_quaternion(new_q);
}
