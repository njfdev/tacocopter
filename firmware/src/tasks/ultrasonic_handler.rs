use core::f32::consts::PI;

use embassy_rp::{
    gpio::{AnyPin, Input, Level, Output, Pull},
    peripherals::{PIN_16, PIN_17},
    Peri,
};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use micromath::F32Ext;

use crate::{
    consts::{ULTRASONIC_DISTANCE_TO_CENTER_PITCH, ULTRASONIC_HEIGHT_ABOVE_BOTTOM},
    drivers::hc_sr04::HcSr04,
    global::{IMU_SIGNAL, SHARED, TEMPERATURE, ULTRASONIC_WATCH},
    tc_println,
    tools::yielding_timer::YieldingTimer,
};

#[embassy_executor::task]
pub async fn calc_ultrasonic_height_agl(
    trig_pin_peripheral: Peri<'static, AnyPin>,
    echo_pin_peripheral: Peri<'static, AnyPin>,
) {
    let mut ultrasonic_sensor = HcSr04::new(trig_pin_peripheral, echo_pin_peripheral);

    let mut imu_rotation = (0.0, 0.0, 0.0);

    let mut imu_reciever = IMU_SIGNAL.receiver().unwrap();
    let ultrasonic_sender = ULTRASONIC_WATCH.sender();

    // wait for 10 milliseconds for any signals to clear (e.g. the pin was held high by default, then low, so it triggers once)
    Timer::after_millis(10).await;
    loop {
        YieldingTimer::after_millis(1000 / 30).await;

        let ultrasonic_res =
            with_timeout(Duration::from_millis(150), ultrasonic_sensor.get_dist()).await;

        if ultrasonic_res.is_err() {
            continue;
        }

        let distance = ultrasonic_res.unwrap();

        let imu_recv = imu_reciever.try_get();
        if imu_recv.is_some() {
            imu_rotation = imu_recv.unwrap().1;
        }

        let angle_to_down_cos = imu_rotation.0.cos() * imu_rotation.1.cos();
        if angle_to_down_cos.acos() < PI / 6.0 && distance.is_some() {
            let height_agl_m = ((angle_to_down_cos * distance.unwrap())
                - (imu_rotation.0.sin() * ULTRASONIC_DISTANCE_TO_CENTER_PITCH))
                - ULTRASONIC_HEIGHT_ABOVE_BOTTOM;
            ultrasonic_sender.send(Some(height_agl_m));
        } else {
            ultrasonic_sender.send(None);
        }

        // tc_println!("Height: {}m", height_agl_m);

        // tc_println!("Distance ({}us): {:.2?} cm", time, distance);
    }
}
