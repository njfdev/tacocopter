#![no_std]

use core::fmt::Debug;
use core::prelude::rust_2024::derive;
use heapless::{String, Vec};
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};

pub const TC_VID: u16 = 0x8216;
pub const TC_PID: u16 = 0x1248;

pub const LOG_SEGMENT_SIZE: usize = 54;
pub const BLACKBOX_SEGMENT_SIZE: usize = 60;

//----------------------------------------------------------//
//--------------- Flight Controller Messages ---------------//
//----------------------------------------------------------//

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct ImuSensorData {
    pub gyroscope: [f32; 3],
    pub accelerometer: [f32; 3],
    // pub gyro_orientation: [f32; 4],
    // pub accel_orientation: [f32; 4],
    // pub orientation: [f32; 4],
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
    pub target_update_rate: f32,
    pub imu_process_rate: f32,
    pub control_loop_update_rate: f32,
    pub blheli_passthrough: bool,
    // in seconds
    pub uptime: u32,
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
    pub log_id: u16,
    pub log_part_index: u8,
    pub log_level: log::Level,
    pub text: String<LOG_SEGMENT_SIZE>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct GyroCalibrationProgressData {
    pub samples: usize,
    pub seconds_remaining: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum SensorCalibrationType {
    Data(SensorCalibrationData),
    GyroProgress(GyroCalibrationProgressData),
    GyroFinished,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum BlackboxInfoType {
    Length(u32),
    SerializedSegment(Vec<u8, BLACKBOX_SEGMENT_SIZE>),
    DownloadFinished(u32),
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct PIDSettings {
    pub pitch: [f32; 3],
    pub roll: [f32; 3],
    pub yaw: [f32; 3],
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum TCMessage {
    // if true, then configurator should wait for more received packets
    // before sending data, otherwise, it is safe to send a packet.
    PacketIndicator(bool),
    State(StateData),
    ImuSensor(ImuSensorData),
    Sensor(SensorData),
    SensorCalibration(SensorCalibrationType),
    ElrsChannels([u16; 16]),
    Log(LogData),
    BlackboxInfo(BlackboxInfoType),
    PIDSettings(PIDSettings),
    Blackbox(bool),
}

unsafe impl Send for ImuSensorData {}

//-----------------------------------------------------------//
//---------------- Configurator App Messages ----------------//
//-----------------------------------------------------------//

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct StartGyroCalibrationData {
    pub sampling_time: f32,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub enum ConfiguratorMessage {
    StartGyroCalibration(StartGyroCalibrationData),
    StartBlackboxDownload,
    ToggleBlHeliPassthrough,
    SetPidSettings(PIDSettings),
    SetBlackboxEnabled(bool),
}

//-----------------------------------------------------------//
//---------------------- Shared Types -----------------------//
//-----------------------------------------------------------//

#[derive(Clone, Serialize, Deserialize, Default, MaxSize)]
pub struct BlackboxLogData {
    pub timestamp_secs: f64,
    pub imu_process_rate: f32,
    pub control_loop_update_rate: f32,
    pub throttle: f32,
    pub target_rate_pitch: f32,
    pub target_rate_roll: f32,
    pub target_rate_yaw: f32,
    pub actual_rate_pitch: f32,
    pub actual_rate_roll: f32,
    pub actual_rate_yaw: f32,
    pub p_term_pitch: f32,
    pub p_term_roll: f32,
    pub p_term_yaw: f32,
    pub i_term_pitch: f32,
    pub i_term_roll: f32,
    pub i_term_yaw: f32,
    pub d_term_pitch: f32,
    pub d_term_roll: f32,
    pub d_term_yaw: f32,
    pub pid_output_pitch: f32,
    pub pid_output_roll: f32,
    pub pid_output_yaw: f32,
    pub g_force: f32,
}

impl BlackboxLogData {
    pub fn new(
        time_secs: f64,
        imu_process_rate: f32,
        control_loop_update_rate: f32,
        throttle: f32,
        target_rate: [f32; 3],
        actual_rate: [f32; 3],
        p_term: [f32; 3],
        i_term: [f32; 3],
        d_term: [f32; 3],
        pid_output: [f32; 3],
        g_force: f32,
    ) -> Self {
        Self {
            timestamp_secs: time_secs,
            imu_process_rate,
            control_loop_update_rate,
            throttle,
            target_rate_pitch: target_rate[0],
            target_rate_roll: target_rate[1],
            target_rate_yaw: target_rate[2],
            actual_rate_pitch: actual_rate[0],
            actual_rate_roll: actual_rate[1],
            actual_rate_yaw: actual_rate[2],
            p_term_pitch: p_term[0],
            p_term_roll: p_term[1],
            p_term_yaw: p_term[2],
            i_term_pitch: i_term[0],
            i_term_roll: i_term[1],
            i_term_yaw: i_term[2],
            d_term_pitch: d_term[0],
            d_term_roll: d_term[1],
            d_term_yaw: d_term[2],
            pid_output_pitch: pid_output[0],
            pid_output_roll: pid_output[1],
            pid_output_yaw: pid_output[2],
            g_force,
        }
    }
}
