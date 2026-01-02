use core::iter::Inspect;

use cortex_m::{delay::Delay, peripheral::DWT};
use embassy_rp::{
    clocks::clk_sys_freq,
    i2c::{self, I2c},
    pac::TIMER0,
    peripherals::{I2C1, PIO0, USB},
    usb::Driver,
};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex},
    mutex::Mutex,
};
use embassy_time::{Instant, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use log::info;
use mpu6050::Mpu6050;
use static_cell::StaticCell;

use crate::{
    consts::UPDATE_LOOP_FREQUENCY,
    drivers::esc::{blheli_passthrough::BlHeliPassthrough, dshot_pio::DshotPio, EscPins},
    global::REALTIME_LOOP_FREQUENCY_SIGNAL,
    tasks::realtime::{control_loop::ControlLoop, esc_loop::EscLoop, imu_loop::ImuLoop},
    tools::yielding_timer::YieldingTimer,
};

pub mod control_loop;
pub mod esc_loop;
pub mod imu_loop;
// TODO: Add this back when working on position holding again
//pub mod position_hold_loop;

#[embassy_executor::task]
pub async fn realtime_loop(
    // for imu_loop
    mpu: Mpu6050<I2c<'static, I2C1, i2c::Async>>,

    // for esc_loop
    esc_pins: EscPins<'static, PIO0>,
    dshot: DshotPio<'static, 4, PIO0>,
) {
    let mut imu_loop = ImuLoop::new(mpu);
    let mut control_loop = ControlLoop::new().await;
    let mut esc_loop = EscLoop::new(esc_pins, dshot).await;

    let mut last_cycle = TIMER0.timerawl().read();
    let mut last_log = Instant::now();

    loop {
        let mut cur = TIMER0.timerawl().read();
        while cur.wrapping_sub(last_cycle) < (1e6 / UPDATE_LOOP_FREQUENCY) as u32 {
            cur = TIMER0.timerawl().read();
        }
        if last_log.elapsed().as_millis() > 100 {
            let micros_time = cur.wrapping_sub(last_cycle);
            REALTIME_LOOP_FREQUENCY_SIGNAL.signal(1e6 / (micros_time as f32));
            last_log = Instant::now();
        }
        last_cycle = cur;

        let imu_data = imu_loop.process().await;
        let control_data = control_loop.process(imu_data).await;
        esc_loop.process(control_data).await;
    }
}
