#![no_std]

use core::fmt::Debug;
use core::prelude::rust_2024::derive;
use heapless::String;
use serde::{Deserialize, Serialize};

pub const TC_VID: u16 = 0x8216;
pub const TC_PID: u16 = 0x1248;

//----------------------------------------------------------//
//--------------- Flight Controller Messages ---------------//
//----------------------------------------------------------//

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ImuSensorData {
    // pub gyroscope: [f32; 3],
    // pub accelerometer: [f32; 3],
    pub gyro_orientation: [f32; 4],
    pub accel_orientation: [f32; 4],
    pub orientation: [f32; 4],
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct SensorData {
    pub estimated_altitude: f32,
    pub ultrasonic_dist: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct SensorCalibrationData {
    pub gyro_calibration: [f32; 3],
    pub accel_calibration: [f32; 3],
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct StateData {
    pub sensor_update_rate: f32,
}

// #[derive(Serialize, Deserialize, Clone, Debug)]
// pub enum LogLevel {
//     Trace,
//     Debug,
//     Info,
//     Warning,
//     Error,
// }

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct LogData {
    pub id: u8,
    pub text: String<60>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct GyroCalibrationProgressData {
    pub progress: f32,
    pub samples: u32,
    pub seconds_remaining: f32,
    pub is_finished: bool,
    pub options: StartGyroCalibrationData,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum TCMessage {
    // if true, then configurator should wait for more received packets
    // before sending data, otherwise, it is safe to send a packet.
    PacketIndicator(bool),
    State(StateData),
    ImuSensor(ImuSensorData),
    Sensor(SensorData),
    SensorCalibration(SensorCalibrationData),
    ElrsChannels([u16; 16]),
    Log(LogData),
    GyroCalibrationProgress(GyroCalibrationProgressData),
}

unsafe impl Send for ImuSensorData {}

//-----------------------------------------------------------//
//---------------- Configurator App Messages ----------------//
//-----------------------------------------------------------//

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct StartGyroCalibrationData {
    pub sampling_time: f32,
    pub sampling_rate: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ConfiguratorMessage {
    StartGyroCalibration(StartGyroCalibrationData),
}
