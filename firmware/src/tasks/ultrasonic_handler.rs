use core::f32::consts::PI;

use crate::{
    consts::{ULTRASONIC_DISTANCE_TO_CENTER_PITCH, ULTRASONIC_HEIGHT_ABOVE_BOTTOM},
    drivers::hc_sr04::HcSr04,
    global::{IMU_SIGNAL, ULTRASONIC_WATCH},
};
use embassy_rp::{
    peripherals::{PIN_16, PIN_17, PIO1},
    Peri,
};
use embassy_time::Timer;
use micromath::F32Ext;

#[embassy_executor::task]
pub async fn calc_ultrasonic_height_agl(
    trig_pin_peripheral: Peri<'static, PIN_16>,
    echo_pin_peripheral: Peri<'static, PIN_17>,
    pio: Peri<'static, PIO1>,
) {
    let mut ultrasonic_sensor = HcSr04::new(trig_pin_peripheral, echo_pin_peripheral, pio);

    let mut imu_rotation = (0.0, 0.0, 0.0);

    let mut imu_reciever = IMU_SIGNAL.receiver().unwrap();
    let ultrasonic_sender = ULTRASONIC_WATCH.sender();

    // wait for 10 milliseconds for any signals to clear (e.g. the pin was held high by default, then low, so it triggers once)
    Timer::after_millis(10).await;
    loop {
        let distance = ultrasonic_sensor.get_dist().await;

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
    }
}
