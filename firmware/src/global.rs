use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, lazy_lock::LazyLock,
    mutex::Mutex, signal::Signal, watch::Watch,
};
use embassy_time::Instant;
use heapless::String;
use tc_interface::{
    GyroCalibrationProgressData, ImuSensorData, LogData, SensorCalibrationData, SensorData,
    StartGyroCalibrationData, StateData,
};

use crate::{consts::UPDATE_LOOP_FREQUENCY, drivers::m100_gps::GPSPayload};

// TODO: split this shared state for faster performance
#[derive(Default)]
pub struct SharedState {
    pub state_data: StateData,
    pub imu_sensor_data: ImuSensorData,
    pub sensor_data: SensorData,
    pub calibration_data: SensorCalibrationData,
    pub elrs_channels: [u16; 16],
    pub gyro_calibration_state: GyroCalibrationProgressData,
}

pub static BOOT_TIME: LazyLock<Instant> = LazyLock::new(|| Instant::now());

// pub static SHARED_LOG: Mutex<ThreadModeRawMutex, String<16384>> = Mutex::new(String::new());
pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogData, 128> = Channel::new();

pub static ELRS_SIGNAL: Signal<CriticalSectionRawMutex, [u16; 16]> = Signal::new();
// The first 3 numbers are the IMU Rates in radians per second, the second 3 are for the estimated orientation, and the last three are the accel values
pub static IMU_SIGNAL: Watch<
    CriticalSectionRawMutex,
    ((f32, f32, f32), (f32, f32, f32), (f32, f32, f32)),
    3,
> = Watch::new();
// (armed, throttle_percent, motor_powers)
pub static CONTROL_LOOP_VALUES: Signal<CriticalSectionRawMutex, (bool, f32, [f32; 4])> =
    Signal::new();
pub static GPS_SIGNAL: Watch<CriticalSectionRawMutex, GPSPayload, 2> = Watch::new();
// measured height in m
pub static ULTRASONIC_WATCH: Watch<CriticalSectionRawMutex, Option<f32>, 2> = Watch::new();
// in format of (pressure in kPa, temperature in kelvin, estimated altitude in m)
pub static BMP390_WATCH: Watch<CriticalSectionRawMutex, (f32, f32, f32), 1> = Watch::new();
// processed in position hold control loop, for use in the control loop under position hold mode (altitude, vertical velocity)
pub static CURRENT_ALTITUDE: Watch<CriticalSectionRawMutex, (Option<f32>, Option<f32>), 2> =
    Watch::new();
pub static ARMED_WATCH: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();
// first 3 f32s are the gyroscope data, the second are the accelerometer data
pub static IMU_RAW_SIGNAL: Signal<CriticalSectionRawMutex, ([f32; 3], [f32; 3])> = Signal::new();
// In celsius
pub static TEMPERATURE: Signal<CriticalSectionRawMutex, f32> = Signal::new();

// frequency signals (for usb logging)
pub static IMU_FETCH_FREQUENCY_SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();
pub static IMU_PROCESSOR_FREQUENCY_SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();
pub static CONTROL_LOOP_FREQUENCY_SIGNAL: Signal<CriticalSectionRawMutex, f32> = Signal::new();
pub static POSITION_HOLD_LOOP_FREQUENCY_SIGNAL: Signal<CriticalSectionRawMutex, f32> =
    Signal::new();
pub static IMU_PROCESSOR_SIGNAL: Signal<CriticalSectionRawMutex, (f32, ImuSensorData)> =
    Signal::new();

pub static SHARED: Mutex<CriticalSectionRawMutex, SharedState> = Mutex::new(SharedState {
    state_data: StateData {
        target_update_rate: UPDATE_LOOP_FREQUENCY as f32,
        imu_fetch_rate: 0.0,
        imu_process_rate: 0.0,
        control_loop_update_rate: 0.0,
        position_hold_loop_update_rate: 0.0,
        uptime: 0,
    },
    imu_sensor_data: ImuSensorData {
        // gyroscope: [0.0, 0.0, 0.0],
        // accelerometer: [0.0, 0.0, 0.0],
        gyro_orientation: [0.0; 4],
        accel_orientation: [0.0; 4],
        orientation: [0.0; 4],
    },
    sensor_data: SensorData {
        estimated_altitude: 0.0,
        ultrasonic_dist: 0.0,
    },
    calibration_data: SensorCalibrationData {
        gyro_calibration: [0.0, 0.0, 0.0],
        accel_calibration: [0.0, 0.0, 0.0],
    },
    elrs_channels: [0; 16],
    gyro_calibration_state: GyroCalibrationProgressData {
        progress: 0.0,
        samples: 0,
        seconds_remaining: 0.0,
        is_finished: true,
        options: StartGyroCalibrationData {
            sampling_time: 0.0,
            sampling_rate: 0.0,
        },
    },
});
