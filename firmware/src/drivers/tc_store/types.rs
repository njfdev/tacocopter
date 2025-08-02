use core::str::FromStr;

use heapless::String;
use serde::{Deserialize, Serialize};

use crate::drivers::tc_store::TcKeyValueStoreData;

#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct SensorCalibrationData {
    pub accel_biases: (f32, f32, f32),
    pub gyro_biases: (f32, f32, f32),
}

impl TcKeyValueStoreData for SensorCalibrationData {
    fn key() -> String<16> {
        String::from_str("SENSOR_CALIB").unwrap()
    }
}

impl Into<tc_interface::SensorCalibrationData> for SensorCalibrationData {
    fn into(self) -> tc_interface::SensorCalibrationData {
        tc_interface::SensorCalibrationData {
            accel_calibration: self.accel_biases.into(),
            gyro_calibration: self.gyro_biases.into(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PIDValues {
    pub pitch: [f32; 3],
    pub roll: [f32; 3],
    pub yaw: [f32; 3],
}

impl Default for PIDValues {
    fn default() -> Self {
        Self {
            pitch: [0.0015, 0.00002, 0.006],
            roll: [0.0015, 0.00002, 0.006],
            yaw: [0.008, 0.0001, 0.0005],
        }
    }
}

impl TcKeyValueStoreData for PIDValues {
    fn key() -> String<16> {
        String::from_str("PID_VALUES").unwrap()
    }
}

impl Into<tc_interface::PIDSettings> for PIDValues {
    fn into(self) -> tc_interface::PIDSettings {
        tc_interface::PIDSettings {
            pitch: self.pitch,
            roll: self.roll,
            yaw: self.yaw,
        }
    }
}

impl From<tc_interface::PIDSettings> for PIDValues {
    fn from(settings: tc_interface::PIDSettings) -> Self {
        Self {
            pitch: settings.pitch,
            roll: settings.roll,
            yaw: settings.yaw,
        }
    }
}
