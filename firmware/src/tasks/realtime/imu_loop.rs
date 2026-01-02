use embassy_futures::yield_now;
use embassy_rp::{
    i2c::{Async, I2c, Instance},
    peripherals::I2C1,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Sender, Watch},
};
use embassy_time::{Instant, Timer};
use log::{info, warn};
use micromath::F32Ext;
use mpu6050::Mpu6050;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use tc_interface::{GyroCalibrationProgressData, SensorCalibrationType};

use crate::{
    consts::{UPDATE_LOOP_FREQUENCY, USB_LOGGER_RATE},
    drivers::tc_store::{types::SensorCalibrationData, TcStore},
    global::{
        CalibrationSensorType, CALIBRATION_FEEDBACK_SIGNAL, IMU_CALIB_SIGNAL, IMU_WATCH,
        START_CALIBRATION_SIGNAL,
    },
    tools::{
        calibrators::imu::GyroCalibrator, kalman::KalmanFilterQuat, yielding_timer::YieldingTimer,
    },
};

#[derive(Clone)]
pub struct ImuData {
    pub gyro_data: (f32, f32, f32),
    pub accel_data: (f32, f32, f32),
}

pub struct ImuLoop<T: Instance + 'static> {
    start: Instant,

    calibration_type: Option<CalibrationSensorType>,
    gyro_calibrator: GyroCalibrator,
    sensor_calibration: tc_interface::SensorCalibrationData,

    imu_sender: Sender<'static, CriticalSectionRawMutex, ImuData, 4>,

    mpu: Mpu6050<I2c<'static, T, Async>>,
}

impl<T: Instance + 'static> ImuLoop<T> {
    pub fn new(mpu: Mpu6050<I2c<'static, T, Async>>) -> Self {
        Self {
            start: Instant::now(),

            calibration_type: None,
            gyro_calibrator: GyroCalibrator::new(),
            sensor_calibration: tc_interface::SensorCalibrationData::default(),

            imu_sender: IMU_WATCH.sender(),

            mpu: mpu,
        }
    }

    pub async fn process(&mut self) -> ImuData {
        let mut gyro_data: [f32; 3] = self.mpu.get_gyro().unwrap().as_slice().try_into().unwrap();
        let mut accel_data: [f32; 3] = self.mpu.get_acc().unwrap().as_slice().try_into().unwrap();

        // correct for sensor orientation
        gyro_data[0] *= -1.0;
        gyro_data[1] *= -1.0;
        accel_data[0] *= -1.0;
        accel_data[1] *= -1.0;

        // if calibration needs to occur, interrupt the loop early for calibration step
        if START_CALIBRATION_SIGNAL.signaled() {
            self.calibration_type = Some(START_CALIBRATION_SIGNAL.try_take().unwrap());
        }
        calibration_handler(
            &mut self.calibration_type,
            gyro_data,
            accel_data,
            &mut self.gyro_calibrator,
            &mut self.start,
        )
        .await;

        // correct biases
        let calib_res = IMU_CALIB_SIGNAL.try_take();
        if calib_res.is_some() {
            self.sensor_calibration = calib_res.unwrap();
        }

        let (gyro_data, accel_data) = (
            correct_biases(&gyro_data, self.sensor_calibration.gyro_calibration),
            correct_biases(&accel_data, self.sensor_calibration.accel_calibration),
        );

        let imu_data = ImuData {
            gyro_data: gyro_data.try_into().unwrap(),
            accel_data: accel_data.try_into().unwrap(),
        };

        self.imu_sender.send(imu_data.clone());

        imu_data
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
