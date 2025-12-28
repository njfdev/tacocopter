use embassy_futures::yield_now;
use embassy_rp::{
    i2c::{Async, I2c},
    peripherals::I2C1,
};
use embassy_time::Instant;
use log::{info, warn};
use micromath::F32Ext;
use mpu6050::Mpu6050;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use tc_interface::{GyroCalibrationProgressData, SensorCalibrationType};

use crate::{
    consts::{UPDATE_LOOP_FREQUENCY, USB_LOGGER_RATE},
    drivers::tc_store::{types::SensorCalibrationData, TcStore},
    global::{
        CalibrationSensorType, CALIBRATION_FEEDBACK_SIGNAL, IMU_CALIB_SIGNAL,
        IMU_PROCESSOR_FREQUENCY_WATCH, IMU_WATCH, START_CALIBRATION_SIGNAL,
    },
    tools::{
        calibrators::imu::GyroCalibrator, kalman::KalmanFilterQuat, yielding_timer::YieldingTimer,
    },
};

#[embassy_executor::task]
pub async fn mpu6050_processor_loop(mut mpu: Mpu6050<I2c<'static, I2C1, Async>>) {
    let mut last_loop = Instant::now();
    let mut last_log = Instant::now();
    let mut start = Instant::now();

    let mut calibration_type: Option<CalibrationSensorType> = None;
    let mut gyro_calibrator = GyroCalibrator::new();
    let mut sensor_calibration = tc_interface::SensorCalibrationData::default();

    let imu_watch_sender = IMU_WATCH.sender();
    let imu_processor_freq_sender = IMU_PROCESSOR_FREQUENCY_WATCH.sender();

    loop {
        let new_since_last = YieldingTimer::after_micros(
            ((1_000_000.0 / UPDATE_LOOP_FREQUENCY) as u64)
                .checked_sub(last_loop.elapsed().as_micros())
                .unwrap_or_default(),
        )
        .await;
        let frequency = 1_000_000.0 / last_loop.elapsed().as_micros() as f32;
        last_loop = new_since_last;

        let gyro_data: [f32; 3] = mpu.get_gyro().unwrap().as_slice().try_into().unwrap();
        let accel_data: [f32; 3] = mpu.get_acc().unwrap().as_slice().try_into().unwrap();

        // if calibration needs to occur, interrupt the loop early for calibration step
        if START_CALIBRATION_SIGNAL.signaled() {
            calibration_type = Some(START_CALIBRATION_SIGNAL.try_take().unwrap());
        }
        calibration_handler(
            &mut calibration_type,
            gyro_data,
            accel_data,
            &mut gyro_calibrator,
            &mut start,
        )
        .await;

        // correct biases
        let calib_res = IMU_CALIB_SIGNAL.try_take();
        if calib_res.is_some() {
            sensor_calibration = calib_res.unwrap();
        }

        let (gyro_data, accel_data) = (
            correct_biases(&gyro_data, sensor_calibration.gyro_calibration),
            correct_biases(&accel_data, sensor_calibration.accel_calibration),
        );

        imu_watch_sender.send((
            gyro_data.try_into().unwrap(),
            Default::default(), //orientation_vector,
            accel_data.try_into().unwrap(),
        ));

        if last_log.elapsed().as_millis() >= (1000.0 / USB_LOGGER_RATE) as u64 {
            last_log = Instant::now();
            imu_processor_freq_sender.send(frequency);
        }
    }
}

// returns true if should move on the next loop
async fn calibration_handler(
    calibration_type: &mut Option<CalibrationSensorType>,
    gyro_data: [f32; 3],
    accel_data: [f32; 3],
    gyro_calibrator: &mut GyroCalibrator,
    last_update: &mut Instant,
) -> bool {
    if calibration_type.is_some() {
        match calibration_type.clone().unwrap() {
            CalibrationSensorType::Gyro(settings) => {
                let bias_res = gyro_calibrator.process_data(gyro_data.into(), settings);
                if bias_res.is_ok() {
                    let bias_data = bias_res.unwrap();
                    *calibration_type = None;
                    let mut current_calib = TcStore::get::<SensorCalibrationData>().await;
                    current_calib.gyro_biases = bias_data;
                    TcStore::set(current_calib.clone()).await;
                    IMU_CALIB_SIGNAL.signal(current_calib.into());
                    CALIBRATION_FEEDBACK_SIGNAL.signal(SensorCalibrationType::GyroFinished);
                } else {
                    if last_update.elapsed().as_millis() >= (1000.0 / USB_LOGGER_RATE) as u64 {
                        *last_update = Instant::now();
                        let progress = bias_res.unwrap_err();
                        CALIBRATION_FEEDBACK_SIGNAL.signal(SensorCalibrationType::GyroProgress(
                            GyroCalibrationProgressData {
                                seconds_remaining: progress.0,
                                samples: progress.1,
                            },
                        ));
                    }
                }
                // skip over normal logic because the calibrator is only what matters now
                return true;
            }
            CalibrationSensorType::Accel => {
                warn!("Accelerometer calibration is not yet implemented...");
                *calibration_type = None;
                return false;
            }
        }
    } else {
        return false;
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
