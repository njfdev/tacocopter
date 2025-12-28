use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, lazy_lock::LazyLock,
    mutex::Mutex, once_lock::OnceLock, signal::Signal, watch::Watch,
};
use embassy_time::Instant;
use tc_interface::{
    ImuSensorData, LogData, SensorCalibrationData, SensorCalibrationType, SensorData,
    StartGyroCalibrationData, StateData,
};

use crate::{
    consts::UPDATE_LOOP_FREQUENCY,
    drivers::{
        m100_gps::GPSPayload,
        tc_store::types::{BlackboxSettings, PIDValues},
    },
    setup::flash::FlashType,
};

#[derive(Clone)]
pub enum CalibrationSensorType {
    Gyro(StartGyroCalibrationData),
    Accel,
}

pub static BOOT_TIME: LazyLock<Instant> = LazyLock::new(|| Instant::now());

pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogData, 128> = Channel::new();

pub static ELRS_WATCH: Watch<CriticalSectionRawMutex, [u16; 16], 2> = Watch::new();
// The first 3 numbers are the IMU Rates in radians per second, the second 3 are for the estimated orientation, and the last three are the accel values
pub static IMU_WATCH: Watch<
    CriticalSectionRawMutex,
    ((f32, f32, f32), (f32, f32, f32), (f32, f32, f32)),
    4,
> = Watch::new();
// (armed, throttle_percent, motor_powers)
pub static CONTROL_LOOP_VALUES: Signal<CriticalSectionRawMutex, (bool, f32, [f32; 4])> =
    Signal::new();
pub static GPS_SIGNAL: Watch<CriticalSectionRawMutex, GPSPayload, 2> = Watch::new();
// measured height in m
pub static ULTRASONIC_WATCH: Watch<CriticalSectionRawMutex, Option<f32>, 2> = Watch::new();
pub static PID_WATCH: Watch<CriticalSectionRawMutex, PIDValues, 1> = Watch::new();
pub static BLACKBOX_SETTINGS_WATCH: Watch<CriticalSectionRawMutex, BlackboxSettings, 1> =
    Watch::new();
// in format of (pressure in kPa, temperature in kelvin, estimated altitude in m)
pub static BMP390_WATCH: Watch<CriticalSectionRawMutex, (f32, f32, f32), 1> = Watch::new();
// processed in position hold control loop, for use in the control loop under position hold mode (altitude, vertical velocity)
pub static CURRENT_ALTITUDE: Watch<CriticalSectionRawMutex, (Option<f32>, Option<f32>), 3> =
    Watch::new();
pub static ARMED_WATCH: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();
// calibration updates
pub static IMU_CALIB_SIGNAL: Signal<CriticalSectionRawMutex, SensorCalibrationData> = Signal::new();
// In celsius
pub static TEMPERATURE: Signal<CriticalSectionRawMutex, f32> = Signal::new();

pub static FC_PASSTHROUGH_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// for negotiating and sending sensor calibration between imu and usb loops
pub static START_CALIBRATION_SIGNAL: Signal<CriticalSectionRawMutex, CalibrationSensorType> =
    Signal::new();
pub static CALIBRATION_FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, SensorCalibrationType> =
    Signal::new();

// frequency signals (for usb logging)
pub static IMU_PROCESSOR_FREQUENCY_WATCH: Watch<CriticalSectionRawMutex, f32, 2> = Watch::new();
pub static CONTROL_LOOP_FREQUENCY_SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();
pub static IMU_PROCESSOR_SIGNAL: Signal<CriticalSectionRawMutex, (f32, ImuSensorData)> =
    Signal::new();

// store access to Flash behind a mutex
pub static FLASH_MUTEX: OnceLock<Mutex<CriticalSectionRawMutex, FlashType>> = OnceLock::new();

pub static USB_ENABLED: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();
