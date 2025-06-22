use core::f32::NAN;

use embassy_futures::yield_now;
use embassy_time::{Instant, Timer};
use micromath::F32Ext;

use crate::{
    consts::{GRAVITY, ULTRASONIC_HEIGHT_ABOVE_BOTTOM, UPDATE_LOOP_FREQUENCY},
    global::{
        ARMED_WATCH, BMP390_WATCH, CURRENT_ALTITUDE, GPS_SIGNAL, IMU_SIGNAL, SHARED,
        ULTRASONIC_WATCH,
    },
    tools::altitude_estimator::{AltitudeEstimator, ACCEL_VERTICAL_BIAS},
};

#[embassy_executor::task]
pub async fn position_hold_loop() {
    let mut gps_receiver = GPS_SIGNAL.receiver().unwrap();
    let mut imu_receiver = IMU_SIGNAL.receiver().unwrap();
    let mut ultrasonic_receiver = ULTRASONIC_WATCH.receiver().unwrap();
    let mut barometer_receiver = BMP390_WATCH.receiver().unwrap();

    let mut gps_altitude: f32 = 0.0;
    let mut _gps_available: bool = false;
    let mut gps_altitude_acc: f32 = 0.0;
    let mut gps_locked_sats: u8 = 0;
    let mut accel_vertical_speed: f32 = 0.0;
    let mut _accel_rel_altitude: f32 = 0.0;
    let mut _ultrasonic_rel_altitude: f32 = 0.0;
    let mut _is_ultrasonic_valid: bool = false;
    let mut barometer_pressure: f32 = 0.0;
    let mut barometer_height: f32 = 0.0;
    let mut h0 = NAN;
    let mut _temperature: f32 = 0.0;
    let mut _t0 = NAN;

    let mut last_imu = Instant::now();

    let mut last_print = Instant::now();
    let mut last_update = Instant::now();

    let mut estimator = AltitudeEstimator::new();
    let mut can_estimate_altitude = false;
    // let since_start = Instant::now();
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
            _accel_rel_altitude += accel_vertical_speed * dt;

            estimator.update_accel(vertical_accel);

            // use the update accel data to predict the state
            // filter.predict(Matrix1::new(accel_rel_altitude as f64));
        }

        let barometer_recv = barometer_receiver.try_changed();
        if barometer_recv.is_some() {
            let barometer_payload = barometer_recv.unwrap();
            if h0.is_nan() {
                h0 = barometer_payload.0;
                _t0 = barometer_payload.1;
            }
            barometer_pressure = barometer_payload.0;
            _temperature = barometer_payload.1;
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
            _gps_available = gps_payload.sat_num >= 4;

            if gps_altitude_acc < 1.0 && gps_locked_sats >= 10 {
                estimator.update_gps(gps_altitude, -gps_payload.down_vel);
            }

            // gps_measurement.set_measurement(Matrix1::new(gps_altitude as f64));
            // filter.update(&gps_measurement);
        } else {
            _gps_available = false;
        }

        let ultrasonic_recv = ultrasonic_receiver.try_changed();
        if ultrasonic_recv.is_some() {
            let ultrasonic_payload = ultrasonic_recv.unwrap();
            // if not a number or ultrasonic sensor is displaying number too close (e.g., 10cm into the ground, do trust it because something might be blocking it)
            if ultrasonic_payload.is_nan()
                || ultrasonic_payload < ULTRASONIC_HEIGHT_ABOVE_BOTTOM / 3.0
            {
                _is_ultrasonic_valid = false;
            } else {
                _is_ultrasonic_valid = true;
                _ultrasonic_rel_altitude =
                    (ultrasonic_payload - ULTRASONIC_HEIGHT_ABOVE_BOTTOM).max(0.0);
                estimator.update_ultrasonic(_ultrasonic_rel_altitude);
            }
        }

        altitude_sender.send(if can_estimate_altitude {
            (
                Some(estimator.get_altitude_msl()),
                Some(estimator.get_vertical_velocity()),
            )
        } else {
            (None, None)
        });

        if last_print.elapsed().as_millis() > 100 {
            // if gps_locked_sats > 0 {
            //     tc_println!(
            //         "GPS Altitude ({}): {}m (Error: {}m)",
            //         (gps_locked_sats),
            //         (gps_altitude),
            //         (gps_altitude_acc)
            //     );
            // } else {
            //     tc_println!("GPS Altitude: sats not locked yet!");
            // }
            // tc_println!(
            //     "Accel: Altitude={} m  -  VS={} m/s",
            //     accel_rel_altitude,
            //     accel_vertical_speed
            // );
            // if is_ultrasonic_valid {
            //     tc_println!("Ultrasonic Altitude: {} m", ultrasonic_rel_altitude);
            // } else {
            //     tc_println!("Ultrasonic Altitude: invalid");
            // }
            // tc_println!("Barometer Altitude: {} m", barometer_height);
            // if can_estimate_altitude {
            //     tc_println!("Filtered Altitude: {} m", (estimator.get_altitude_msl()));
            //     tc_println!("Filtered VS: {} m/s", (estimator.get_vertical_velocity()));
            // } else {
            //     tc_println!("Filtered Altitude: not ready");
            // }

            last_print = Instant::now();
        }

        if barometer_pressure != 0.0 && is_armed {
            //l&& is_ultrasonic_valid
            if !can_estimate_altitude && gps_altitude_acc < 1.0 && gps_locked_sats >= 10 {
                can_estimate_altitude = true;
                estimator.reset(barometer_height, gps_altitude);
            }
        } else {
            can_estimate_altitude = false;
        }

        {
            let mut shared = SHARED.lock().await;
            shared.state_data.position_hold_loop_update_rate = 1.0 / dt;
        }

        while ((1_000_000.0 / UPDATE_LOOP_FREQUENCY) - last_update.elapsed().as_micros() as f64)
            > 0.0
        {
            yield_now().await;
        }
    }
}
