#![no_std]
#![no_main]

//pub mod bmp390;
// pub mod dshot;
pub mod altitude_estimator;
pub mod elrs;
pub mod hc_sr04;
pub mod kalman;
pub mod m100_gps;
pub mod pm02d;
pub mod tc_log;
// pub mod mpu6050;

use core::cell::RefCell;
use core::cmp::min;
use core::f32::consts::PI;
use core::f32::NAN;
use core::ops::Deref;
use core::str::FromStr;

use crate::altitude_estimator::{AltitudeEstimator, ACCEL_VERTICAL_BIAS};
use crate::m100_gps::GPSPayload;
use bmp390::{Bmp390, OdrSel, Oversampling, PowerMode};
use defmt::println;
use dshot_pio::dshot_embassy_rp::DshotPio;
use dshot_pio::DshotPioTrait;
use elrs::{crc8, init_elrs};
use embassy_embedded_hal::shared_bus;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_futures::select::{select, Either};
use embassy_futures::yield_now;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::{I2C0, I2C1, PIN_16, PIN_17, PIO0, UART0, USB};
use embassy_rp::pwm::{Pwm, SetDutyCycle};
use embassy_rp::usb::{Driver, Endpoint, In, InterruptHandler, Out};
use embassy_rp::{bind_interrupts, pwm};
use embassy_sync::blocking_mutex::raw::{
    CriticalSectionRawMutex, NoopRawMutex, ThreadModeRawMutex,
};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_usb::driver::{EndpointIn, EndpointOut};
use embassy_usb::{Builder, UsbDevice};
use embedded_io_async::Write;
use heapless::{String, Vec};
use kalman::KalmanFilterQuat;
use kfilter::measurement::{LinearMeasurement, Measurement};
use kfilter::system::System;
use kfilter::{
    Kalman, Kalman1M, KalmanFilter, KalmanLinear, KalmanPredict, KalmanPredictInput, KalmanUpdate,
};
use log::{error, info, warn};
use m100_gps::init_gps;
use micromath::F32Ext;
use mpu6050::Mpu6050;
use nalgebra::{
    Matrix1, Matrix1x2, Matrix2, Matrix2x1, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3,
};
use pid::Pid;
use pm02d::PM02D;
use postcard::from_bytes;
use static_cell::StaticCell;
use tc_interface::{
    ConfiguratorMessage, GyroCalibrationProgressData, ImuSensorData, LogData,
    SensorCalibrationData, SensorData, StartGyroCalibrationData, StateData, TCMessage, TC_PID,
    TC_VID,
};
use uom::si::length::meter;
use uom::si::pressure::kilopascal;
use uom::si::thermodynamic_temperature::kelvin;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct UsbIrq {
    USBCTRL_IRQ => InterruptHandler<USB>;
});
bind_interrupts!(struct I2C0Irqs {
  I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});
bind_interrupts!(struct I2C1Irqs {
  I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});
bind_interrupts!(struct Pio0Irqs {
  PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
});

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

// TODO: have a way to calibrate this, and then store it in something like Flash memory
// update based on calibration data
pub const ACCEL_BIASES: [f32; 3] = [0.044174805, -0.063529054, 0.07425296];
pub const GYRO_BIASES: [f32; 3] = [-0.0356924, -0.0230041, -0.03341522];
pub const GRAVITY: f32 = 9.80665;

// TODO: split this shared state for faster performance
#[derive(Default)]
struct SharedState {
    pub state_data: StateData,
    pub imu_sensor_data: ImuSensorData,
    pub sensor_data: SensorData,
    pub calibration_data: SensorCalibrationData,
    pub elrs_channels: [u16; 16],
    pub gyro_calibration_state: GyroCalibrationProgressData,
}

// pub static SHARED_LOG: Mutex<ThreadModeRawMutex, String<16384>> = Mutex::new(String::new());
static LOG_CHANNEL: Channel<CriticalSectionRawMutex, String<60>, 64> = Channel::new();

static ELRS_SIGNAL: Signal<ThreadModeRawMutex, [u16; 16]> = Signal::new();
// The first 3 numbers are the IMU Rates in radians per second, the second 3 are for the estimated orientation, and the last three are the accel values
static IMU_SIGNAL: Watch<
    CriticalSectionRawMutex,
    ((f32, f32, f32), (f32, f32, f32), (f32, f32, f32)),
    3,
> = Watch::new();
// (armed, throttle_percent, motor_powers)
static CONTROL_LOOP_VALUES: Signal<ThreadModeRawMutex, (bool, f32, [f32; 4])> = Signal::new();
static GPS_SIGNAL: Watch<CriticalSectionRawMutex, GPSPayload, 2> = Watch::new();
// measured height in m
static ULTRASONIC_WATCH: Watch<CriticalSectionRawMutex, f32, 1> = Watch::new();
// in format of (pressure in kPa, temperature in kelvin, estimated altitude in m)
static BMP390_WATCH: Watch<CriticalSectionRawMutex, (f32, f32, f32), 1> = Watch::new();
// processed in position hold control loop, for use in the control loop under position hold mode
static CURRENT_ALTITUDE: Watch<CriticalSectionRawMutex, f32, 1> = Watch::new();
static ARMED_WATCH: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();
// In celsius
static TEMPERATURE: Signal<ThreadModeRawMutex, f32> = Signal::new();

pub static SHARED: Mutex<ThreadModeRawMutex, SharedState> = Mutex::new(SharedState {
    state_data: StateData {
        sensor_update_rate: UPDATE_LOOP_FREQUENCY as f32,
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

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // set up serial logging over USB
    let driver = Driver::new(p.USB, UsbIrq);
    // spawner.spawn(logger_task(driver)).unwrap();

    // turn on the onboard LED to make it clear the device is on
    let mut led = Output::new(p.PIN_25, Level::Low);
    led.set_high();

    let i2c = i2c::I2c::new_async(p.I2C1, p.PIN_15, p.PIN_14, I2C1Irqs, i2c::Config::default());
    let mut mpu = Mpu6050::new(i2c);
    let mut delay = Delay;
    mpu.init(&mut delay).unwrap();
    mpu.set_accel_range(mpu6050::device::AccelRange::G8)
        .unwrap();
    mpu.set_gyro_range(mpu6050::device::GyroRange::D1000)
        .unwrap();
    // calibrate_accel(&mut mpu, 10.0).await;
    // calibrate_gyro(&mut mpu, 10.0).await;

    // Timer::after_secs(5).await;

    let sensor_data = SensorCalibrationData {
        accel_calibration: ACCEL_BIASES, //get_accel_offsets(&mut mpu, 10.0).await,
        gyro_calibration: GYRO_BIASES,
    };
    {
        let mut shared = SHARED.lock().await;
        shared.calibration_data = sensor_data;
    }

    // setup the USB driver
    let mut config = embassy_usb::Config::new(TC_VID, TC_PID);
    config.manufacturer = Some("Nicholas Fasching");
    config.product = Some("Tacocopter Flight Controller");
    config.max_power = 100; // 100mA

    // USB buffers
    static mut DEVICE_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];

    // create USB builder
    let mut builder = Builder::new(
        driver,
        config,
        unsafe { &mut DEVICE_DESCRIPTOR },
        unsafe { &mut CONFIG_DESCRIPTOR },
        unsafe { &mut BOS_DESCRIPTOR },
        unsafe { &mut CONTROL_BUF },
    );

    // Add bulk endpoints (OUT and IN)
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    let mut bulk_out_ep = alt.endpoint_bulk_out(64); // 64-byte packets
    let mut bulk_in_ep = alt.endpoint_bulk_in(64); // 64-byte packets
    drop(function);

    // Build and run USB device
    let usb = builder.build();
    spawner.spawn(usb_task(usb)).unwrap();

    let mut elrs_tx = init_elrs(p.PIN_0, p.PIN_1, p.UART0, &spawner).await;

    let mut gps = init_gps(p.PIN_8, p.PIN_9, p.UART1, &spawner).await;

    spawner.spawn(usb_updater(bulk_in_ep, bulk_out_ep)).unwrap();
    spawner.spawn(mpu6050_loop(mpu)).unwrap();

    // spawner
    //     .spawn(calc_ultrasonic_distance(p.PIN_16, p.PIN_17))
    //     .unwrap();

    static I2C0_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'_, I2C0, Async>>> =
        StaticCell::new();
    let i2c0: I2c<'static, I2C0, Async> =
        I2c::new_async(p.I2C0, p.PIN_21, p.PIN_20, I2C0Irqs, i2c::Config::default());
    let i2c0_bus = Mutex::new(i2c0);
    let i2c0_bus = I2C0_BUS.init(i2c0_bus);

    // let i2c0_bus: &'static _ = shared_bus::new_cortexm!(<I2c<'static, I2C0, Async> = i2c0).unwrap();

    let mut bmp390_conf = bmp390::Configuration::default();
    bmp390_conf.iir_filter.iir_filter = bmp390::IirFilter::coef_3;
    bmp390_conf.output_data_rate.odr_sel = OdrSel::ODR_25;
    bmp390_conf.oversampling.pressure = Oversampling::X16;
    bmp390_conf.power_control.enable_temperature = true;
    bmp390_conf.power_control.enable_pressure = true;
    bmp390_conf.power_control.mode = PowerMode::Normal;
    // let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, I2C0Irqs, i2c::Config::default());
    let mut bmp: Bmp390<I2cDevice<'_, CriticalSectionRawMutex, I2c<'_, I2C0, Async>>> =
        Bmp390::try_new(
            I2cDevice::new(i2c0_bus),
            bmp390::Address::Up,
            Delay,
            &bmp390_conf,
        )
        .await
        .unwrap();
    spawner.spawn(bmp_loop(bmp)).unwrap();

    let mut dshot = DshotPio::<4, _>::new(
        p.PIO0,
        Pio0Irqs,
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        p.PIN_5,
        (31, 74), // divider ratio of 31.25
    );

    // let desired_freq_hz = 24_000;
    // let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    // let divider = 16u8;
    // let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    // let mut c = pwm::Config::default();
    // c.top = period;
    // c.divider = divider.into();

    // let mut pwm = Pwm::new_output_a(p.PWM_SLICE1, p.PIN_2, c.clone());
    // pwm.set_duty_cycle_percent(10).unwrap();
    // Timer::after_secs(2).await;
    let _ = spawner.spawn(dshot_handler(dshot));

    // run dshot and ultrasonic sensor on other core
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor = EXECUTOR1.init(Executor::new());
            executor.run(|spawner| {
                let _ = spawner.spawn(calc_ultrasonic_height_agl(p.PIN_16, p.PIN_17));
                let _ = spawner.spawn(control_loop());
                let _ = spawner.spawn(position_hold_loop());
            })
        },
    );

    let mut pm02d_interface = PM02D::new(I2cDevice::new(i2c0_bus)).await;
    let mut gps_receiver = GPS_SIGNAL.receiver().unwrap();

    loop {
        // let alt_packed = get_altitude_packed(measurement.value - base_altitude);
        // info!(
        //     "Altitude({:4x?}): {} m",
        //     alt_packed,
        //     measurement.value - base_altitude
        // );
        // elrs_tx
        //     .write_all(&[
        //         0xc8,
        //         0x05,
        //         0x09,
        //         (alt_packed >> 8) as u8,
        //         (alt_packed & 0xff) as u8,
        //         0x02,
        //         crc8(
        //             &[
        //                 0x09,
        //                 (alt_packed >> 8) as u8,
        //                 (alt_packed & 0xff) as u8,
        //                 0x02,
        //             ],
        //             4,
        //         ),
        //     ])
        //     .await
        //     .unwrap();
        led.set_low();
        Timer::after_millis(10).await;
        led.set_high();
        Timer::after_millis(10).await;
        let voltage = pm02d_interface.get_voltage().await;
        let current = pm02d_interface.get_current().await;
        let capacity = 5200;
        let (percent, mins_remaining) = pm02d_interface.estimate_battery_charge(4, capacity).await;
        // // tc_println!("Voltage: {}V", voltage);
        // // tc_println!("Current: {}A", current);
        // // tc_println!(
        // //     "Estimated State: {:.2}%, {:.2} mins remaining",
        // //     percent,
        // //     mins_remaining
        // // );
        // // tc_println!(
        // //     "Used capacity: {:.1}mah",
        // //     (pm02d_interface.get_used_capacity())
        // // );

        let voltage_bytes = ((voltage * 10.0) as u16).to_be_bytes();
        let current_bytes = ((current * 10.0) as u16).to_be_bytes();
        let capacity_bytes = (capacity as u32).to_be_bytes();
        let remaining_bytes = (percent as u8).to_be_bytes();
        elrs_tx
            .write_all(&[
                0xc8,
                0x0a,
                0x08,
                voltage_bytes[0],
                voltage_bytes[1],
                current_bytes[0],
                current_bytes[1],
                capacity_bytes[1],
                capacity_bytes[2],
                capacity_bytes[3],
                remaining_bytes[0],
                crc8(
                    &[
                        0x08,
                        voltage_bytes[0],
                        voltage_bytes[1],
                        current_bytes[0],
                        current_bytes[1],
                        capacity_bytes[1],
                        capacity_bytes[2],
                        capacity_bytes[3],
                        remaining_bytes[0],
                    ],
                    9,
                ),
            ])
            .await
            .unwrap();

        let gps_payload_res = gps_receiver.try_get();
        if gps_payload_res.is_some() {
            let gps_payload = gps_payload_res.unwrap();

            let latitude_bytes = ((gps_payload.latitude * 10_000_000.0) as i32).to_be_bytes();
            let longitude_bytes = ((gps_payload.longitude * 10_000_000.0) as i32).to_be_bytes();
            let ground_speed_bytes =
                ((gps_payload.ground_speed * 10_000.0 / 3600.0) as u16).to_be_bytes();
            let gps_heading = ((gps_payload.motion_heading * 100.0) as u16).to_be_bytes();
            let altitude = ((gps_payload.msl_height + 1000.0) as u16).to_be_bytes();
            let num_sats = (gps_payload.sat_num as u8).to_be_bytes();
            elrs_tx
                .write_all(&[
                    0xc8,
                    0x11,
                    0x02,
                    latitude_bytes[0],
                    latitude_bytes[1],
                    latitude_bytes[2],
                    latitude_bytes[3],
                    longitude_bytes[0],
                    longitude_bytes[1],
                    longitude_bytes[2],
                    longitude_bytes[3],
                    ground_speed_bytes[0],
                    ground_speed_bytes[1],
                    gps_heading[0],
                    gps_heading[1],
                    altitude[0],
                    altitude[1],
                    num_sats[0],
                    crc8(
                        &[
                            0x02,
                            latitude_bytes[0],
                            latitude_bytes[1],
                            latitude_bytes[2],
                            latitude_bytes[3],
                            longitude_bytes[0],
                            longitude_bytes[1],
                            longitude_bytes[2],
                            longitude_bytes[3],
                            ground_speed_bytes[0],
                            ground_speed_bytes[1],
                            gps_heading[0],
                            gps_heading[1],
                            altitude[0],
                            altitude[1],
                            num_sats[0],
                        ],
                        16,
                    ),
                ])
                .await
                .unwrap();
        }
    }
}

// Measurement noise for sensors
const R_BAROMETER: f64 = 0.5;
const R_GPS: f64 = 4.0;

fn create_altitude_filter() -> Kalman<f64, 2, 1, kfilter::system::LinearSystem<f64, 2, 1>> {
    // Stater transition matrix (F)
    let f = Matrix2::new(1.0, 1.0 / UPDATE_LOOP_FREQUENCY, 0.0, 1.0);
    // Process Noise (q)
    let b = Matrix2x1::new(
        0.5 * (1.0 / UPDATE_LOOP_FREQUENCY) * (1.0 / UPDATE_LOOP_FREQUENCY),
        1.0 / UPDATE_LOOP_FREQUENCY,
    );
    // Observation matrix (h) - only measuring altitude
    let q = Matrix2::new(0.01, 0.0, 0.0, 0.1);

    // initial state with (altitude, velocity)
    let x_initial = Matrix2x1::new(0.0, 0.0);
    // high uncertainty
    let p_initial = Matrix2::identity() * 10.0;

    KalmanLinear::new_with_input(f, q, b, x_initial, p_initial)
}

// in m
const ULTRASONIC_HEIGHT_ABOVE_BOTTOM: f32 = 0.1515;
// the offset of the ultrasonic sensor from the center that is corrected by pitch
const ULTRASONIC_DISTANCE_TO_CENTER_PITCH: f32 = 0.093;

#[embassy_executor::task]
async fn position_hold_loop() {
    let mut gps_receiver = GPS_SIGNAL.receiver().unwrap();
    let mut imu_receiver = IMU_SIGNAL.receiver().unwrap();
    let mut ultrasonic_receiver = ULTRASONIC_WATCH.receiver().unwrap();
    let mut barometer_receiver = BMP390_WATCH.receiver().unwrap();

    let mut gps_altitude: f32 = 0.0;
    let mut gps_available: bool = false;
    let mut gps_altitude_acc: f32 = 0.0;
    let mut gps_locked_sats: u8 = 0;
    let mut accel_vertical_speed: f32 = 0.0;
    let mut accel_rel_altitude: f32 = 0.0;
    let mut ultrasonic_rel_altitude: f32 = 0.0;
    let mut is_ultrasonic_valid: bool = false;
    let mut barometer_pressure: f32 = 0.0;
    let mut barometer_height: f32 = 0.0;
    let mut h0 = NAN;
    let mut temperature: f32 = 0.0;
    let mut t0 = NAN;

    let mut last_imu = Instant::now();

    let mut last_print = Instant::now();
    let mut last_update = Instant::now();

    let mut estimator = AltitudeEstimator::new();
    let mut can_estimate_altitude = false;
    let since_start = Instant::now();
    // let mut state = BaroErrorState::new();
    // let mut filter = create_altitude_filter();
    // let mut baro_measurement = LinearMeasurement::new(
    //     Matrix1x2::new(1.0, 0.0),
    //     SMatrix::identity(),
    //     Matrix1::new(R_BAROMETER),
    // );
    // let mut gps_measurement = LinearMeasurement::new(
    //     Matrix1x2::new(1.0, 0.0),
    //     SMatrix::identity(),
    //     Matrix1::new(R_GPS),
    // );

    let altitude_sender = CURRENT_ALTITUDE.sender();
    let mut armed_receiver = ARMED_WATCH.receiver().unwrap();
    let mut is_armed = false;

    loop {
        let dt = (last_update.elapsed().as_micros() as f32) / 1_000_000.0;
        last_update = Instant::now();

        let armed_recv = armed_receiver.try_changed();
        if armed_recv.is_some() {
            is_armed = armed_recv.unwrap();
        }

        let imu_recv = imu_receiver.try_changed();
        if imu_recv.is_some() {
            let imu_data = imu_recv.unwrap();

            let mut vertical_accel = ((-imu_data.1 .1.sin()) * imu_data.2 .0
                + (imu_data.1 .1.cos() * imu_data.1 .0.sin()) * imu_data.2 .1
                + (imu_data.1 .1.cos() * imu_data.1 .0.cos()) * imu_data.2 .2)
                - 1.0;
            let dt = (last_imu.elapsed().as_micros() as f32) / 1_000_000.0;
            last_imu = Instant::now();
            // tc_println!("Accel: {}", vertical_accel);
            vertical_accel -= ACCEL_VERTICAL_BIAS;
            accel_vertical_speed += (vertical_accel * GRAVITY) * dt;
            accel_rel_altitude += accel_vertical_speed * dt;

            estimator.update_accel(vertical_accel);

            // use the update accel data to predict the state
            // filter.predict(Matrix1::new(accel_rel_altitude as f64));
        }

        let barometer_recv = barometer_receiver.try_changed();
        if barometer_recv.is_some() {
            let barometer_payload = barometer_recv.unwrap();
            if h0.is_nan() {
                h0 = barometer_payload.0;
                t0 = barometer_payload.1;
            }
            barometer_pressure = barometer_payload.0;
            temperature = barometer_payload.1;
            barometer_height = barometer_payload.2;

            estimator.update_barometer(barometer_height);

            // baro_measurement.set_measurement(Matrix1::new(barometer_altitude as f64));
            // filter.update(&baro_measurement);
        }

        let gps_payload_recv = gps_receiver.try_changed();
        if gps_payload_recv.is_some() {
            let gps_payload = gps_payload_recv.unwrap();
            gps_altitude = gps_payload.msl_height;
            gps_altitude_acc = gps_payload.vertical_accuracy_estimate;
            gps_locked_sats = gps_payload.sat_num;
            gps_available = gps_payload.sat_num >= 4;

            if gps_altitude_acc < 1.0 && gps_locked_sats >= 10 {
                estimator.update_gps(gps_altitude, -gps_payload.down_vel);
            }

            // gps_measurement.set_measurement(Matrix1::new(gps_altitude as f64));
            // filter.update(&gps_measurement);
        } else {
            gps_available = false;
        }

        let ultrasonic_recv = ultrasonic_receiver.try_changed();
        if ultrasonic_recv.is_some() {
            let ultrasonic_payload = ultrasonic_recv.unwrap();
            // if not a number or ultrasonic sensor is displaying number too close (e.g., 10cm into the ground, do trust it because something might be blocking it)
            if ultrasonic_payload.is_nan()
                || ultrasonic_payload < ULTRASONIC_HEIGHT_ABOVE_BOTTOM / 3.0
            {
                is_ultrasonic_valid = false;
            } else {
                is_ultrasonic_valid = true;
                ultrasonic_rel_altitude =
                    (ultrasonic_payload - ULTRASONIC_HEIGHT_ABOVE_BOTTOM).max(0.0);
                estimator.update_ultrasonic(ultrasonic_rel_altitude);
            }
        }

        altitude_sender.send(if can_estimate_altitude {
            estimator.get_altitude_msl()
        } else {
            barometer_height
        });

        if (last_print.elapsed().as_millis() > 100) {
            if gps_locked_sats > 0 {
                tc_println!(
                    "GPS Altitude ({}): {}m (Error: {}m)",
                    (gps_locked_sats),
                    (gps_altitude),
                    (gps_altitude_acc)
                );
            } else {
                tc_println!("GPS Altitude: sats not locked yet!");
            }
            tc_println!(
                "Accel: Altitude={} m  -  VS={} m/s",
                accel_rel_altitude,
                accel_vertical_speed
            );
            if is_ultrasonic_valid {
                tc_println!("Ultrasonic Altitude: {} m", ultrasonic_rel_altitude);
            } else {
                tc_println!("Ultrasonic Altitude: invalid");
            }
            // tc_println!("Barometer Altitude: {} m", barometer_altitude);
            if can_estimate_altitude {
                tc_println!("Filtered Altitude: {} m", (estimator.get_altitude_msl()));
            } else {
                tc_println!("Filtered Altitude: not ready");
            }

            last_print = Instant::now();
        }

        if is_ultrasonic_valid && barometer_pressure != 0.0 && is_armed {
            if gps_altitude_acc < 1.0 && gps_locked_sats >= 10 && !can_estimate_altitude {
                can_estimate_altitude = true;
                estimator.reset(barometer_height, gps_altitude);
            }
        } else {
            can_estimate_altitude = false;
        }

        Timer::after_micros(
            ((1_000_000.0 / UPDATE_LOOP_FREQUENCY) - last_update.elapsed().as_micros() as f64)
                as u64,
        )
        .await;
    }
}

#[embassy_executor::task]
async fn calc_ultrasonic_height_agl(trig_pin_peripheral: PIN_16, echo_pin_peripheral: PIN_17) {
    let mut trig_pin = Output::new(trig_pin_peripheral, Level::Low);
    let mut echo_pin = Input::new(echo_pin_peripheral, Pull::None);

    let mut imu_rotation = (0.0, 0.0, 0.0);

    let mut imu_reciever = IMU_SIGNAL.receiver().unwrap();
    let mut ultrasonic_sender = ULTRASONIC_WATCH.sender();

    let mut distance = 0.0;
    let mut height_agl_m = 0.0;

    // wait for 10 milliseconds for any signals to clear (e.g. the pin was held high by default, then low, so it triggers once)
    Timer::after_millis(10).await;
    loop {
        // println!("Temp: {:?}", (TEMPERATURE.try_take()));
        trig_pin.set_low();
        Timer::after_micros(2).await;
        trig_pin.set_high();
        Timer::after_micros(8).await;
        trig_pin.set_low();

        echo_pin.wait_for_rising_edge().await;
        let start = Instant::now();
        echo_pin.wait_for_falling_edge().await;
        let after = Instant::now();
        let time = Instant::checked_duration_since(&after, start)
            .unwrap()
            .as_micros();
        distance = if time / 1000 <= 50 {
            ((time as f32) * 0.000343) / 2.0
        } else {
            f32::NAN
        };

        let imu_recv = imu_reciever.try_get();
        if imu_recv.is_some() {
            imu_rotation = imu_recv.unwrap().1;
        }

        let angle_to_down_cos = imu_rotation.0.cos() * imu_rotation.1.cos();
        if angle_to_down_cos.acos() < PI / 6.0 && !distance.is_nan() {
            height_agl_m = (angle_to_down_cos * distance)
                - (imu_rotation.0.sin() * ULTRASONIC_DISTANCE_TO_CENTER_PITCH);
            ultrasonic_sender.send(height_agl_m);
        } else {
            ultrasonic_sender.send(f32::NAN);
        }
        // tc_println!("Angle to Down: {}", (angle_to_down_cos.acos() / PI * 180.0));
        // tc_println!("Height: {}m", height_agl_m);

        // {
        //     let mut shared = SHARED.lock().await;
        //     shared.sensor_data.ultrasonic_dist = distance;
        // }
        // tc_println!("Distance ({}us): {:.2?} cm", time, distance);
        Timer::after_millis(1000 / 30).await;
    }
}

// if deadzone is passed as a value, then the output is scaled to -1.0 to 1.0, otherwise it is scaled from 0.0 to 1.0
fn elrs_input_to_percent(input: u16, deadzone_opt: Option<f32>) -> f32 {
    let input_percent = ((input - 172) as f32) / 1638.0;

    if deadzone_opt.is_none() {
        return input_percent.clamp(0.0, 1.0);
    }

    let rescaled_input_percent = (input_percent - 0.5) * 2.0;

    let deadzone = deadzone_opt.unwrap();

    if rescaled_input_percent.abs() < deadzone * 2.0 {
        return 0.0;
    };

    if rescaled_input_percent > 0.0 {
        return ((rescaled_input_percent - deadzone) / (1.0 - deadzone)).clamp(0.0, 1.0);
    }
    return ((rescaled_input_percent + deadzone) / (1.0 - deadzone)).clamp(-1.0, 0.0);
}

/*
Quadcopter motor orientation

With motor 1 and 4 CCW and motor 2 and 3 CW

     (front)

/---\       /---\
| 1 |       | 2 |
\---/       \---/
     \ ___ /
      |   |
      |___|
     /     \
/---\       /---\
| 3 |       | 4 |
\---/       \---/

      (back)
*/
const MAX_ACRO_RATE: f32 = 200.0; // What is the target rotation rate at full throttle input?
#[embassy_executor::task]
async fn control_loop() {
    // rotation pid
    let mut pid_yaw = Pid::new(0.0, 0.25);
    pid_yaw.p(0.008, 0.5);
    pid_yaw.i(0.0001, 0.1);
    pid_yaw.d(0.0005, 0.1);
    let mut pid_roll = Pid::new(0.0, 0.25);
    pid_roll.p(0.0011, 0.5);
    pid_roll.i(0.000014, 0.1);
    pid_roll.d(0.00013, 0.1);
    let mut pid_pitch = Pid::new(0.0, 0.25);
    pid_pitch.p(0.0011, 0.5);
    pid_pitch.i(0.000014, 0.1);
    pid_pitch.d(0.00013, 0.1);

    // vertical pid
    let mut pid_altitude = Pid::new(0.0, 7.5); // up to 7.5 m/s corrections
    pid_altitude.p(5.0, 7.5);
    pid_altitude.i(0.0, 7.5);
    pid_altitude.d(0.8, 7.5);
    let mut pid_vs = Pid::new(0.0, 0.5);
    pid_vs.p(0.06, 0.4);
    pid_vs.i(0.0002, 0.1);
    pid_vs.d(0.0, 0.4);

    // elrs controls
    let mut armed = false;
    let mut position_hold = false;
    let mut throttle_input = 0.0;
    let mut yaw_input = 0.0;
    let mut roll_input = 0.0;
    let mut pitch_input = 0.0;

    // imu stuff
    let mut rate_errors = (0.0, 0.0, 0.0);
    let mut imu_values = (0.0, 0.0, 0.0);
    let mut imu_rates = (0.0, 0.0, 0.0);

    // altitude stuff
    let mut altitude_receiver = CURRENT_ALTITUDE.receiver().unwrap();
    let mut current_altitude = None;
    let mut last_altitude = None;
    let mut target_altitude = 0.0;
    let mut current_vertical_speed = 0.0;

    let mut since_last_loop = Instant::now();
    let mut since_last_elrs_update = Instant::now();

    let mut imu_reciever = IMU_SIGNAL.receiver().unwrap();
    let mut armed_sender = ARMED_WATCH.sender();

    loop {
        Timer::after_micros(
            1_000_000 / (UPDATE_LOOP_FREQUENCY as u64) - since_last_loop.elapsed().as_micros(),
        )
        .await;
        let dt = (since_last_loop.elapsed().as_micros() as f32) / 1_000_000.0;
        since_last_loop = Instant::now();

        let altitude_recv = altitude_receiver.try_changed();
        if altitude_recv.is_some() && (current_altitude.is_some() || altitude_recv.unwrap() != 0.0)
        {
            if current_altitude.is_none() {
                target_altitude = altitude_recv.unwrap();
            }
            current_altitude = altitude_recv;
            current_vertical_speed = if last_altitude.is_some() {
                (current_altitude.unwrap() - last_altitude.unwrap()) * dt
            } else {
                0.0
            };
        }

        let imu_recv = imu_reciever.try_get();
        if imu_recv.is_some() {
            let mut imu_recv_values = imu_recv.unwrap();
            imu_recv_values.1 = (
                imu_recv_values.1 .0 * 180.0 / PI,
                imu_recv_values.1 .1 * 180.0 / PI,
                imu_recv_values.1 .2 * 180.0 / PI,
            );
            imu_recv_values.0 = (
                imu_recv_values.0 .0 * 180.0 / PI,
                imu_recv_values.0 .1 * 180.0 / PI,
                imu_recv_values.0 .2 * 180.0 / PI,
            );
            imu_rates = (
                imu_recv_values.0 .0,
                imu_recv_values.0 .1,
                imu_recv_values.0 .2,
            );
            imu_values = imu_recv_values.1;
        }

        let chnls_recv = ELRS_SIGNAL.try_take();
        if chnls_recv.is_some() {
            let chnls = chnls_recv.unwrap();
            let new_armed = elrs_input_to_percent(chnls[4], None) > 0.5;
            if !armed && new_armed {
                pid_yaw.reset_integral_term();
                pid_pitch.reset_integral_term();
                pid_roll.reset_integral_term();
                // rate_errors = imu_values;
            }
            armed = new_armed;
            armed_sender.send(armed);
            let new_position_hold = elrs_input_to_percent(chnls[7], None) > 0.5;
            if position_hold == false && new_position_hold == true {
                pid_altitude.reset_integral_term();
                pid_vs.reset_integral_term();
                if current_altitude.is_some() {
                    target_altitude = current_altitude.unwrap();
                }
            }
            position_hold = new_position_hold;
            throttle_input = elrs_input_to_percent(chnls[2], None);
            yaw_input = elrs_input_to_percent(chnls[0], Some(0.01)) * MAX_ACRO_RATE;
            roll_input = elrs_input_to_percent(chnls[3], Some(0.01)) * MAX_ACRO_RATE;
            pitch_input = elrs_input_to_percent(chnls[1], Some(0.01)) * MAX_ACRO_RATE;
            rate_errors.0 = pitch_input + imu_rates.0;
            rate_errors.1 = -roll_input + imu_rates.1;
            rate_errors.2 = yaw_input + imu_rates.2;
            since_last_elrs_update = Instant::now();
        }

        // tc_println!("Position hold? {}", position_hold);

        // calc pid
        let pid_pitch_output = pid_pitch.next_control_output(rate_errors.0).output;
        let pid_roll_output = pid_roll.next_control_output(rate_errors.1).output;
        let pid_yaw_output = pid_yaw.next_control_output(rate_errors.2).output;

        let mut pid_vs_output = 0.0;
        let should_use_position_hold =
            position_hold && current_altitude.is_some() && throttle_input > 0.3;
        if should_use_position_hold {
            let pid_altitude_output = pid_altitude
                .next_control_output(current_altitude.unwrap() - target_altitude)
                .output;
            pid_vs_output = pid_vs
                .next_control_output(current_vertical_speed - pid_altitude_output)
                .output;
            tc_println!("PID Alt: {}", pid_altitude_output);
            tc_println!("PID Output: {}", pid_vs_output);
        }
        // tc_println!("Current: {}", current_altitude);

        let current_throttle_value = if should_use_position_hold {
            pid_vs_output + 0.5
        } else {
            throttle_input
        };

        let t1 = (current_throttle_value + pid_pitch_output + pid_roll_output - pid_yaw_output)
            .clamp(0.0, 1.0);
        let t2 = (current_throttle_value + pid_pitch_output - pid_roll_output + pid_yaw_output)
            .clamp(0.0, 1.0);
        let t3 = (current_throttle_value - pid_pitch_output + pid_roll_output + pid_yaw_output)
            .clamp(0.0, 1.0);
        let t4 = (current_throttle_value - pid_pitch_output - pid_roll_output - pid_yaw_output)
            .clamp(0.0, 1.0);

        // TODO: implement return to home like stuff in this case scenario
        // if signal to controller is lost, turn off motors for safety
        if since_last_elrs_update.elapsed().as_millis() > 500 {
            armed = false;
        }
        CONTROL_LOOP_VALUES.signal((armed, throttle_input, [t1, t2, t3, t4]));

        last_altitude = current_altitude;
    }
}

const MAX_THROTTLE_PERCENT: f32 = 1.0;
#[embassy_executor::task]
async fn dshot_handler(mut dshot: DshotPio<'static, 4, PIO0>) {
    // let before = Instant::now();
    // while (before.elapsed().as_secs() < 2) {
    //     dshot.command([0, 0, 0, 0]);
    //     Timer::after_micros(1000).await;
    // }

    let mut armed = false;
    let mut mtr_pwrs = [0.0, 0.0, 0.0, 0.0];
    let mut since_last_throttle_update = Instant::now();
    let mut time_since_armed = Instant::now();

    loop {
        let control_loop_recv = CONTROL_LOOP_VALUES.try_take();
        if control_loop_recv.is_some() {
            // && since_last_throttle_update.elapsed().as_micros() > 50000 {
            let (armed_recv, throttle_percent, mtr_pwrs_recv) = control_loop_recv.unwrap();
            mtr_pwrs = mtr_pwrs_recv;
            // tc_println!("Motor pwrs: {:?}", mtr_pwrs);
            if armed_recv != armed {
                armed = throttle_percent < 0.01 && armed_recv;
                if armed {
                    time_since_armed = Instant::now();
                } else if armed_recv == false {
                    // if just disarmed, send the motor stop packet
                    dshot.command([0, 0, 0, 0]);
                }
            }
            since_last_throttle_update = Instant::now();
        }
        // let pwm_pwr = (((throttle - 176) as f32) / 1634.0) * 90.0 + 10.0;
        // pwm.set_duty_cycle_percent(dshot_cmd as u8).unwrap();
        if armed {
            let mut dshot_msgs: [u16; 4] = [0, 0, 0, 0];
            for (i, motor_pwr) in mtr_pwrs.iter().enumerate() {
                let dshot_cmd = if armed && time_since_armed.elapsed().as_millis() > 1000 {
                    (motor_pwr.max(0.02).min(MAX_THROTTLE_PERCENT) * 1999.0) as u16 + 48
                } else {
                    0
                };
                let dshot_data = dshot_cmd << 1;
                let dshot_crc = (dshot_data ^ (dshot_data >> 4) ^ (dshot_data >> 8)) & 0x0f;
                dshot_msgs[i] = (dshot_data << 4) + dshot_crc;
            }
            dshot.command(dshot_msgs);
            Timer::after_micros(1000).await;
        } else {
            Timer::after_millis(10).await;
        }
        yield_now().await;
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn bmp_loop(
    mut bmp: Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>,
) {
    let barometer_sender = BMP390_WATCH.sender();
    let base_altitude = get_base_altitude(&mut bmp).await;
    loop {
        Timer::after_millis(50).await;
        let measurement = bmp.altitude().await.unwrap();
        {
            let mut shared = SHARED.lock().await;
            shared.sensor_data.estimated_altitude = measurement.value - base_altitude;
        }
        let pressure = bmp.pressure().await.unwrap();
        let temp = bmp.temperature().await.unwrap();
        barometer_sender.send((
            pressure.get::<kilopascal>(),
            temp.get::<kelvin>(),
            measurement.get::<meter>(),
        ));
        TEMPERATURE.signal(temp.value);
    }
}

const LOGGER_RATE: f32 = 30.0;

#[embassy_executor::task]
async fn usb_updater(
    mut usb_send: Endpoint<'static, USB, In>,
    mut usb_read: Endpoint<'static, USB, Out>,
) {
    let time_between = Duration::from_millis((1000.0 / LOGGER_RATE) as u64);
    let mut cur_log_id: u8 = 0;
    loop {
        let start = Instant::now();

        let mut buffer = [0u8; 64];
        let state_data: StateData;
        let imu_sensor_data: ImuSensorData;
        let sensor_data: SensorData;
        let calibration_data: SensorCalibrationData;
        let elrs_channels: [u16; 16];
        {
            let mut shared = SHARED.lock().await;
            state_data = shared.state_data.clone();
            imu_sensor_data = shared.imu_sensor_data.clone();
            sensor_data = shared.sensor_data.clone();
            calibration_data = shared.calibration_data.clone();
            elrs_channels = shared.elrs_channels.clone();
        }

        // indicate start of data
        postcard::to_slice(&TCMessage::PacketIndicator(true), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send state data
        usb_send.write(&buffer).await.unwrap();

        // send imu sensor data
        postcard::to_slice(&TCMessage::ImuSensor(imu_sensor_data), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send sensor data
        postcard::to_slice(&TCMessage::Sensor(sensor_data), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send sensor calibration data
        postcard::to_slice(&TCMessage::SensorCalibration(calibration_data), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send elrs channel data
        postcard::to_slice(&TCMessage::ElrsChannels(elrs_channels), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // while log_buffer.len() > 0 {
        //     let amount_this_cycle = min(log_buffer.len(), 63);
        //     // send log channel data
        //     postcard::to_slice(
        //         &TCMessage::Log(LogData {
        //             id: cur_log_id,
        //             text: String::from_str(&log_buffer[0..amount_this_cycle]).unwrap(),
        //         }),
        //         &mut buffer,
        //     )
        //     .unwrap();
        //     usb_send.write(&buffer).await.unwrap();
        //     log_buffer = String::from_str(&log_buffer[amount_this_cycle..]).unwrap();
        //     cur_log_id = cur_log_id.wrapping_add(1);
        // }
        while let Ok(log_msg) = LOG_CHANNEL.try_receive() {
            postcard::to_slice(
                &TCMessage::Log(LogData {
                    id: cur_log_id,
                    text: String::from_str(&log_msg).unwrap(),
                }),
                &mut buffer,
            )
            .unwrap();
            usb_send.write(&buffer).await.unwrap();
            cur_log_id = cur_log_id.wrapping_add(1);
        }

        // indicate end of data
        postcard::to_slice(&TCMessage::PacketIndicator(false), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        match select(
            usb_read.read(&mut buffer),
            Timer::after(Instant::now().duration_since(start)),
        )
        .await
        {
            Either::First(Ok(n)) => {
                let msg = from_bytes::<ConfiguratorMessage>(&buffer[..n]).unwrap();

                match msg {
                    ConfiguratorMessage::StartGyroCalibration(data) => {
                        {
                            let mut shared = SHARED.lock().await;
                            shared.gyro_calibration_state.options = data;
                            shared.gyro_calibration_state.is_finished = false;
                        }
                        loop {
                            Timer::after(time_between).await;

                            let mut shared = SHARED.lock().await;

                            // indicate end of data
                            postcard::to_slice(
                                &TCMessage::GyroCalibrationProgress(
                                    shared.gyro_calibration_state.clone(),
                                ),
                                &mut buffer,
                            )
                            .unwrap();
                            usb_send.write(&buffer).await.unwrap();

                            if shared.gyro_calibration_state.is_finished == true {
                                break;
                            }
                        }
                    }
                }
            }
            _ => {} // either failed or timeout, so continue normally
        }
    }
}

const GYRO_UNCERTAINTY: f32 = 2.0;
const ACCEL_UNCERTAINTY: f32 = 3.0;
fn kalman_1d(
    state: &mut f32,
    uncertainty: &mut f32,
    input: f32,
    measurement: f32,
    delta: f32,
) -> [f32; 2] {
    *state += delta * input;
    *uncertainty += delta.powi(2) * GYRO_UNCERTAINTY;
    let gain = *uncertainty / (*uncertainty + ACCEL_UNCERTAINTY.powi(2));
    *state += gain * (measurement - *state);
    *uncertainty *= 1.0 - gain;
    [state.clone(), uncertainty.clone()]
}

const UPDATE_LOOP_FREQUENCY: f64 = 200.0;
#[embassy_executor::task]
async fn mpu6050_loop(mut mpu: Mpu6050<I2c<'static, I2C1, Async>>) {
    // TODO: fix integration of gyro data (e.g. tilting, moving yaw, then tilting back changes yaw from starting point)
    let mut rotation_q = UnitQuaternion::identity();
    let mut kalman = KalmanFilterQuat::new();

    let mut filtered_orientation: [f32; 3] = [0.0; 3];

    let imu_watch_sender = IMU_SIGNAL.sender();

    let mut kalman_angle_roll: f32 = 0.0;
    let mut kalman_angle_roll_uncertainty: f32 = 2.0.powi(2);
    let mut kalman_angle_pitch: f32 = 0.0;
    let mut kalman_angle_pitch_uncertainty: f32 = 2.0.powi(2);
    let mut gyro_angle_yaw: f32 = 0.0;

    // the first index is the angle prediction, and the second is the kalman uncertainty
    let mut kalman_output: [f32; 2] = [0.0; 2];

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
        let gyro_data: [f32; 3] = correct_biases(
            mpu.get_gyro().unwrap().as_slice().try_into().unwrap(),
            GYRO_BIASES,
        );
        let accel_data: [f32; 3] = correct_biases(
            mpu.get_acc().unwrap().as_slice().try_into().unwrap(),
            ACCEL_BIASES,
        );
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

        let mut should_start_gyro_calib = false;
        if Instant::now()
            .checked_duration_since(start)
            .unwrap()
            .as_millis()
            >= (1000.0 / LOGGER_RATE) as u64
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
            shared.state_data.sensor_update_rate = (iterations as f32) * LOGGER_RATE;
            shared.calibration_data.accel_calibration = accel_data;
            iterations = 0;
            //info!("Estimated rotation: {:?}", rotation);

            should_start_gyro_calib = !shared.gyro_calibration_state.is_finished;
        }

        // if need to start gyro calibration, do it
        if should_start_gyro_calib {
            get_gyro_offsets(&mut mpu).await;
            start = Instant::now();
        }

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

fn q_to_euler(q: [f32; 4]) -> [f32; 3] {
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        // Use 90° or -90° if out of range
        PI / 2.0 * sinp.signum()
    } else {
        sinp.asin()
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    [roll, pitch, yaw]
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

const CALIBRATION_STEPS: usize = 1000;

// async fn calibrate_accel(mpu: &mut MPU6050, duration: f32) {
//     let time_between = ((duration / (CALIBRATION_STEPS as f32)) * 1_000_000.0) as u64;

//     let mut data_points: [[f64; 3]; CALIBRATION_STEPS] = [[0.0; 3]; CALIBRATION_STEPS];

//     let mut start;
//     for i in 0..CALIBRATION_STEPS {
//         start = Instant::now();
//         data_points[i] = mpu.read_accel_data().await;
//         Timer::after_micros(time_between - (Instant::now() - start).as_micros()).await
//     }

//     let mut bias = calc_averages(data_points);
//     bias[2] -= 1.0; // 1 g of gravity should be discounted from bias

//     info!("Accel biases are: {:?}", bias);
// }

async fn get_gyro_offsets(mpu: &mut Mpu6050<I2c<'static, I2C1, Async>>) -> [f32; 3] {
    let mut settings: StartGyroCalibrationData = StartGyroCalibrationData::default();
    {
        let mut shared = SHARED.lock().await;
        settings = shared.gyro_calibration_state.options.clone();
    }

    let micros_between = ((1.0 / settings.sampling_rate) * 1_000_000.0) as u64;
    let total_samples = ((settings.sampling_time * 1_000_000.0) / micros_between as f32) as usize;

    let mut data_points = Vec::<[f32; 3], 0>::new();
    data_points.resize_default(total_samples).unwrap();

    let mut start;
    for i in 0..total_samples {
        start = Instant::now();
        data_points[i] = (*mpu.get_gyro().unwrap().as_mut_slice())
            .try_into()
            .unwrap();
        Timer::after_micros(micros_between - (Instant::now() - start).as_micros()).await
    }

    // let mut bias = calc_averages(data_points);

    // bias
    [0.0, 0.0, 0.0]
}

async fn get_accel_offsets(
    mpu: &mut Mpu6050<I2c<'static, I2C1, Async>>,
    duration: f32,
) -> [f32; 3] {
    let time_between = ((duration / (CALIBRATION_STEPS as f32)) * 1_000_000.0) as u64;

    let mut data_points: [[f32; 3]; CALIBRATION_STEPS] = [[0.0; 3]; CALIBRATION_STEPS];

    let mut start;
    for i in 0..CALIBRATION_STEPS {
        start = Instant::now();
        data_points[i] = (*mpu.get_acc().unwrap().as_mut_slice()).try_into().unwrap();
        Timer::after_micros(time_between - (Instant::now() - start).as_micros()).await
    }

    let mut bias = calc_averages(data_points);

    // adjust for gravity
    bias[2] -= 1.0;

    bias
}

fn bubble_sort<T: PartialOrd>(arr: &mut [T]) {
    let len = arr.len();
    for i in 0..len {
        for j in 0..len - 1 - i {
            if arr[j] > arr[j + 1] {
                arr.swap(j, j + 1);
            }
        }
    }
}

async fn get_base_altitude(
    bmp: &mut Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>,
) -> f32 {
    const AMOUNT_OF_SAMPLES: usize = 200;
    let mut data_points: [f32; AMOUNT_OF_SAMPLES] = [0.0; AMOUNT_OF_SAMPLES];

    for i in 0..AMOUNT_OF_SAMPLES {
        data_points[i] = bmp.altitude().await.unwrap().value;
        Timer::after_millis(10).await
    }

    bubble_sort(&mut data_points);

    data_points[data_points.len() / 2]
}

fn calc_averages(data: [[f32; 3]; CALIBRATION_STEPS]) -> [f32; 3] {
    let mut avgs: [f32; 3] = [0.0; 3];
    for val in data.iter() {
        avgs[0] += val[0];
        avgs[1] += val[1];
        avgs[2] += val[2];
    }
    avgs[0] /= CALIBRATION_STEPS as f32;
    avgs[1] /= CALIBRATION_STEPS as f32;
    avgs[2] /= CALIBRATION_STEPS as f32;
    return avgs;
}
