#![no_std]
#![no_main]

//pub mod bmp390;
// pub mod dshot;
pub mod elrs;
pub mod hc_sr04;
pub mod kalman;
pub mod m100_gps;
pub mod tc_log;
// pub mod mpu6050;

use core::cmp::min;
use core::f32::consts::PI;
use core::str::FromStr;

use bmp390::{Bmp390, OdrSel, Oversampling, PowerMode};
use defmt::println;
use dshot_pio::dshot_embassy_rp::DshotPio;
use dshot_pio::DshotPioTrait;
use elrs::init_elrs;
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
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_usb::driver::{EndpointIn, EndpointOut};
use embassy_usb::{Builder, UsbDevice};
use heapless::{String, Vec};
use kalman::KalmanFilterQuat;
use log::{error, info, warn};
use m100_gps::init_gps;
use micromath::F32Ext;
use mpu6050::Mpu6050;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use pid::Pid;
use postcard::from_bytes;
use static_cell::StaticCell;
use tc_interface::{
    ConfiguratorMessage, GyroCalibrationProgressData, ImuSensorData, LogData,
    SensorCalibrationData, SensorData, StartGyroCalibrationData, StateData, TCMessage, TC_PID,
    TC_VID,
};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct UsbIrq {
    USBCTRL_IRQ => InterruptHandler<USB>;
});
bind_interrupts!(struct I2CIrqs {
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
pub const ACCEL_BIASES: [f32; 3] = [0.0425, -0.005, 0.05]; //[0.132174805, -0.028529054, 0.07425296];
pub const GYRO_BIASES: [f32; 3] = [-0.0356924, -0.0230041, -0.03341522];

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
static IMU_SIGNAL: Signal<ThreadModeRawMutex, (f32, f32, f32)> = Signal::new();
// (armed, throttle_percent, motor_powers)
static CONTROL_LOOP_VALUES: Signal<ThreadModeRawMutex, (bool, f32, [f32; 4])> = Signal::new();

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

    let mut bmp390_conf = bmp390::Configuration::default();
    bmp390_conf.iir_filter.iir_filter = bmp390::IirFilter::coef_3;
    bmp390_conf.output_data_rate.odr_sel = OdrSel::ODR_25;
    bmp390_conf.oversampling.pressure = Oversampling::X16;
    bmp390_conf.power_control.enable_temperature = false;
    bmp390_conf.power_control.enable_pressure = true;
    bmp390_conf.power_control.mode = PowerMode::Normal;
    let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, I2CIrqs, i2c::Config::default());
    let mut bmp = Bmp390::try_new(i2c, bmp390::Address::Up, Delay, &bmp390_conf)
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

    // run dshot and ultrasonic sensor on other core
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor = EXECUTOR1.init(Executor::new());
            executor.run(|spawner| {
                let _ = spawner.spawn(calc_ultrasonic_distance(p.PIN_16, p.PIN_17));
                let _ = spawner.spawn(dshot_handler(dshot));
                let _ = spawner.spawn(control_loop());
            })
        },
    );
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
        Timer::after_millis(250).await;
        led.set_high();
        Timer::after_millis(250).await;
    }
}

fn elrs_input_to_percent(input: u16) -> f32 {
    return (((input - 172) as f32) / 1638.0).clamp(0.0, 1.0);
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
const MAX_ACRO_RATE: f32 = 400.0; // What is the target rotation rate at full throttle input?
#[embassy_executor::task]
async fn control_loop() {
    // pid
    let mut pid_yaw = Pid::new(0.0, 0.25);
    pid_yaw.p(0.0006, 0.2);
    pid_yaw.i(0.0000, 0.1);
    pid_yaw.d(0.0, 0.1);
    let mut pid_roll = Pid::new(0.0, 0.25);
    pid_roll.p(0.0008, 0.2);
    pid_roll.i(0.0000, 0.1);
    pid_roll.d(0.0000, 0.1);
    let mut pid_pitch = Pid::new(0.0, 0.25);
    pid_pitch.p(0.0008, 0.2);
    pid_pitch.i(0.0000, 0.1);
    pid_pitch.d(0.00000, 0.1);

    // elrs controls
    let mut armed = false;
    let mut throttle_input = 0.0;
    let mut yaw_input = 0.0;
    let mut roll_input = 0.0;
    let mut pitch_input = 0.0;

    // imu stuff
    let mut imu_values = (0.0, 0.0, 0.0);
    let mut imu_rates = (0.0, 0.0, 0.0);

    let mut since_last_loop = Instant::now();
    loop {
        Timer::after_micros(
            1_000_000 / (UPDATE_LOOP_FREQUENCY as u64) - since_last_loop.elapsed().as_micros(),
        )
        .await;
        let dt = (since_last_loop.elapsed().as_micros() as f32) / 1_000_000.0;
        since_last_loop = Instant::now();

        let chnls_recv = ELRS_SIGNAL.try_take();
        if chnls_recv.is_some() {
            let chnls = chnls_recv.unwrap();
            let new_armed = elrs_input_to_percent(chnls[4]) > 0.5;
            if (!armed && new_armed) {}
            throttle_input = elrs_input_to_percent(chnls[2]);
            yaw_input = (elrs_input_to_percent(chnls[0]) - 0.5) * 2.0 * MAX_ACRO_RATE;
            roll_input = (elrs_input_to_percent(chnls[3]) - 0.5) * 2.0 * MAX_ACRO_RATE;
            pitch_input = (elrs_input_to_percent(chnls[1]) - 0.5) * 2.0 * MAX_ACRO_RATE;
        }

        let imu_recv = IMU_SIGNAL.try_take();
        if imu_recv.is_some() {
            let mut imu_recv_values = imu_recv.unwrap();
            imu_recv_values = (
                imu_recv_values.0 * 180.0 / PI,
                imu_recv_values.1 * 180.0 / PI,
                imu_recv_values.2 * 180.0 / PI,
            );
            imu_rates = (
                (imu_recv_values.0 - imu_values.0) / dt,
                (imu_recv_values.1 - imu_values.1) / dt,
                (imu_recv_values.2 - imu_values.2) / dt,
            );
            imu_values = imu_recv_values;
        }

        // calc pid
        let pid_pitch_output = pid_pitch
            .next_control_output(imu_rates.0 + pitch_input)
            .output;
        let pid_roll_output = pid_roll
            .next_control_output(imu_rates.1 - roll_input)
            .output;
        let pid_yaw_output = pid_yaw.next_control_output(imu_rates.2 + yaw_input).output;

        let t1 =
            (throttle_input + pid_pitch_output + pid_roll_output - pid_yaw_output).clamp(0.0, 1.0);
        let t2 =
            (throttle_input + pid_pitch_output - pid_roll_output + pid_yaw_output).clamp(0.0, 1.0);
        let t3 =
            (throttle_input - pid_pitch_output + pid_roll_output + pid_yaw_output).clamp(0.0, 1.0);
        let t4 =
            (throttle_input - pid_pitch_output - pid_roll_output - pid_yaw_output).clamp(0.0, 1.0);

        CONTROL_LOOP_VALUES.signal((armed, throttle_input, [t1, t2, t3, t4]));
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
            tc_println!("Motor pwrs: {:?}", mtr_pwrs);
            if armed_recv != armed {
                armed = throttle_percent < 0.01 && armed_recv;
                if armed {
                    time_since_armed = Instant::now();
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
async fn bmp_loop(mut bmp: Bmp390<I2c<'static, I2C0, Async>>) {
    let base_altitude = get_base_altitude(&mut bmp).await;
    loop {
        Timer::after_millis(50).await;
        let measurement = bmp.altitude().await.unwrap();
        {
            let mut shared = SHARED.lock().await;
            shared.sensor_data.estimated_altitude = measurement.value - base_altitude;
        }
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
        IMU_SIGNAL.signal(orientation_vector);

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

#[embassy_executor::task]
async fn calc_ultrasonic_distance(trig_pin_peripheral: PIN_16, echo_pin_peripheral: PIN_17) {
    let mut trig_pin = Output::new(trig_pin_peripheral, Level::Low);
    let mut echo_pin = Input::new(echo_pin_peripheral, Pull::None);
    // wait for 10 milliseconds for any signals to clear (e.g. the pin was held high by default, then low, so it triggers once)
    Timer::after_millis(10).await;
    loop {
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
        let distance = if time / 1000 <= 50 {
            ((time as f32) * 0.0343) / 2.0
        } else {
            f32::NAN
        };
        // {
        //     let mut shared = SHARED.lock().await;
        //     shared.sensor_data.ultrasonic_dist = distance;
        // }
        // info!("Distance ({}us): {} cm", time, distance);
        Timer::after_millis(1000 / 30).await;
    }
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

async fn get_base_altitude(bmp: &mut Bmp390<I2c<'_, I2C0, Async>>) -> f32 {
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
