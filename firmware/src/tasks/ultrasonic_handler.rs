use core::f32::consts::PI;

use embassy_rp::{
    gpio::{Input, Level, Output, Pull},
    peripherals::{PIN_16, PIN_17},
    Peri,
};
use embassy_time::{Instant, Timer};
use micromath::F32Ext;

use crate::{
    consts::ULTRASONIC_DISTANCE_TO_CENTER_PITCH,
    global::{IMU_SIGNAL, ULTRASONIC_WATCH},
    tools::yielding_timer::YieldingTimer,
};

#[embassy_executor::task]
pub async fn calc_ultrasonic_height_agl(
    trig_pin_peripheral: Peri<'static, PIN_16>,
    echo_pin_peripheral: Peri<'static, PIN_17>,
) {
    let mut trig_pin = Output::new(trig_pin_peripheral, Level::Low);
    let mut echo_pin = Input::new(echo_pin_peripheral, Pull::None);

    let mut imu_rotation = (0.0, 0.0, 0.0);

    let mut imu_reciever = IMU_SIGNAL.receiver().unwrap();
    let ultrasonic_sender = ULTRASONIC_WATCH.sender();

    // wait for 10 milliseconds for any signals to clear (e.g. the pin was held high by default, then low, so it triggers once)
    Timer::after_millis(10).await;
    loop {
        // println!("Temp: {:?}", (TEMPERATURE.try_take()));
        trig_pin.set_low();
        YieldingTimer::after_micros(2).await;
        trig_pin.set_high();
        YieldingTimer::after_micros(8).await;
        trig_pin.set_low();

        echo_pin.wait_for_rising_edge().await;
        let start = Instant::now();
        echo_pin.wait_for_falling_edge().await;
        let after = Instant::now();
        let time = Instant::checked_duration_since(&after, start)
            .unwrap()
            .as_micros();
        let distance = if time / 1000 <= 50 {
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
            let height_agl_m = (angle_to_down_cos * distance)
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
        YieldingTimer::after_millis(1000 / 30).await;
    }
}
