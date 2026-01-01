use core::f32::consts::PI;

use biquad::{Biquad, Coefficients, DirectForm2Transposed, ToHertz, Q_BUTTERWORTH_F32};
use embassy_time::Instant;
use log::{error, info};
use micromath::F32Ext;
use pid::Pid;
use tc_interface::BlackboxLogData;

use crate::{
    consts::UPDATE_LOOP_FREQUENCY,
    drivers::{
        elrs::Elrs,
        tc_store::{
            blackbox::TcBlackbox,
            types::{BlackboxSettings, PIDValues},
            TcStore,
        },
    },
    global::{
        ARMED_WATCH, BLACKBOX_SETTINGS_WATCH, BOOT_TIME, CONTROL_LOOP_FREQUENCY_SIGNAL,
        CONTROL_LOOP_VALUES, CURRENT_ALTITUDE, ELRS_WATCH, IMU_PROCESSOR_FREQUENCY_WATCH,
        IMU_WATCH, PID_WATCH,
    },
    tools::yielding_timer::YieldingTimer,
};

pub fn set_pid_values(
    pid_values: &PIDValues,
    pid_pitch: &mut Pid<f32>,
    pid_roll: &mut Pid<f32>,
    pid_yaw: &mut Pid<f32>,
) {
    pid_pitch.p(pid_values.pitch[0], 0.5);
    pid_pitch.i(pid_values.pitch[1], 0.1);
    pid_pitch.d(pid_values.pitch[2], 0.1);

    pid_roll.p(pid_values.roll[0], 0.5);
    pid_roll.i(pid_values.roll[1], 0.1);
    pid_roll.d(pid_values.roll[2], 0.1);

    pid_yaw.p(pid_values.yaw[0], 0.5);
    pid_yaw.i(pid_values.yaw[1], 0.1);
    pid_yaw.d(pid_values.yaw[2], 0.1);
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
| 4 |       | 3 |
\---/       \---/

      (back)
*/
const MAX_ACRO_RATE: f32 = 200.0; // What is the target rotation rate at full throttle input?
#[embassy_executor::task]
pub async fn control_loop() {
    // filter setup for d-term
    let cutoff = 90.hz();
    let sample_freq = UPDATE_LOOP_FREQUENCY.hz();
    let coeffs = Coefficients::<f32>::from_params(
        biquad::Type::LowPass,
        sample_freq,
        cutoff,
        Q_BUTTERWORTH_F32,
    )
    .unwrap();

    // rotation pid
    let mut yaw_d_filter = DirectForm2Transposed::new(coeffs);
    let mut pid_yaw = Pid::new(0.0, 0.25);
    let mut roll_d_filter = DirectForm2Transposed::new(coeffs);
    let mut pid_roll = Pid::new(0.0, 0.25);
    let mut pitch_d_filter = DirectForm2Transposed::new(coeffs);
    let mut pid_pitch = Pid::new(0.0, 0.25);

    set_pid_values(
        &TcStore::get::<PIDValues>().await,
        &mut pid_pitch,
        &mut pid_roll,
        &mut pid_yaw,
    );

    // vertical pid
    let mut pid_altitude = Pid::new(0.0, 2.0); // up to 2 m/s corrections
    pid_altitude.p(4.0, 7.5);
    pid_altitude.i(0.0, 7.5);
    pid_altitude.d(0.8, 7.5);
    let mut pid_vs = Pid::new(0.0, 0.3);
    pid_vs.p(0.05, 0.4);
    pid_vs.i(0.001, 0.1);
    pid_vs.d(0.0, 0.4);

    // elrs controls
    let mut armed = false;
    let mut position_hold = false;
    let mut throttle_input = 0.0;
    let mut yaw_input;
    let mut roll_input;
    let mut pitch_input;

    // imu stuff
    let mut _imu_values = (0.0, 0.0, 0.0);
    let mut imu_rates = (0.0, 0.0, 0.0);
    let mut target_rates = (0.0, 0.0, 0.0);
    let mut rate_errors = (0.0, 0.0, 0.0);

    // altitude stuff
    let mut altitude_receiver = CURRENT_ALTITUDE.receiver().unwrap();
    let mut current_altitude = None;
    let mut target_altitude = 0.0;
    let mut current_vertical_speed = 0.0;

    let mut since_last_loop = Instant::now();
    let mut since_last_elrs_update = Instant::now();

    let mut imu_reciever = IMU_WATCH.receiver().unwrap();
    let armed_sender = ARMED_WATCH.sender();

    let mut pid_receiver = PID_WATCH.receiver().unwrap();

    let mut blackbox_settings = TcStore::get::<BlackboxSettings>().await;
    let mut blackbox_enabled = blackbox_settings.enabled;
    let mut blackbox_settings_receiver = BLACKBOX_SETTINGS_WATCH.receiver().unwrap();

    let mut elrs_reciever = ELRS_WATCH.receiver().unwrap();

    let mut should_use_position_hold = false;
    let mut since_last_log = 0;

    let mut g_force = 1.0;

    let mut since_blackbox_erased = Instant::now();

    let mut imu_processor_freq_receiver = IMU_PROCESSOR_FREQUENCY_WATCH.receiver().unwrap();
    let mut last_imu_processor_freq = 0.0;

    loop {
        let new_since_last_loop = YieldingTimer::after_micros(
            (1_000_000 / (UPDATE_LOOP_FREQUENCY as u64))
                .checked_sub(since_last_loop.elapsed().as_micros())
                .unwrap_or_default(),
        )
        .await;
        let dt = (since_last_loop.elapsed().as_micros() as f32) / 1_000_000.0;
        since_last_loop = new_since_last_loop;

        let pid_recv = pid_receiver.try_changed();
        if pid_recv.is_some() {
            let new_pid_values = pid_recv.unwrap();
            set_pid_values(&new_pid_values, &mut pid_pitch, &mut pid_roll, &mut pid_yaw);
        }

        let blackbox_settings_recv = blackbox_settings_receiver.try_changed();
        if blackbox_settings_recv.is_some() {
            blackbox_settings = blackbox_settings_recv.unwrap();
        }
        if !armed && blackbox_enabled != blackbox_settings.enabled {
            blackbox_enabled = blackbox_settings.enabled;
        }

        let altitude_recv = altitude_receiver.try_changed();
        if altitude_recv.is_some()
            && (current_altitude.is_some() || altitude_recv.unwrap().0.is_some())
        {
            let altitude_data = altitude_recv.unwrap();
            if altitude_data.0.is_some() && altitude_data.1.is_some() {
                if current_altitude.is_none() {
                    target_altitude = altitude_data.0.unwrap();
                }
                current_altitude = altitude_data.0;
                current_vertical_speed = altitude_data.1.unwrap();
            } else {
                current_altitude = None;
            }
            // current_vertical_speed = if last_altitude.is_some() {
            //     (current_altitude.unwrap() - last_altitude.unwrap()) * dt
            // } else {
            //     0.0
            // };
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
            _imu_values = imu_recv_values.1;
            g_force = (imu_recv_values.2 .0.powi(2)
                + imu_recv_values.2 .1.powi(2)
                + imu_recv_values.2 .2.powi(2))
            .sqrt();
        }

        let chnls_recv = elrs_reciever.try_changed();
        if chnls_recv.is_some() {
            let chnls = chnls_recv.unwrap();
            let new_armed = Elrs::elrs_input_to_percent(chnls[4], None) > 0.5;
            if !armed && new_armed {
                pid_yaw.reset_integral_term();
                pid_pitch.reset_integral_term();
                pid_roll.reset_integral_term();
                yaw_d_filter.reset_state();
                pitch_d_filter.reset_state();
                roll_d_filter.reset_state();
                // rate_errors = imu_values;
                TcBlackbox::start_log().await;
            }
            if armed && !new_armed {
                TcBlackbox::stop_log().await;
            }
            armed = new_armed;
            armed_sender.send(armed);
            let new_position_hold = Elrs::elrs_input_to_percent(chnls[7], None) > 0.5;
            if position_hold == false && new_position_hold == true {
                pid_altitude.reset_integral_term();
                pid_vs.reset_integral_term();
                if current_altitude.is_some() {
                    target_altitude = current_altitude.unwrap();
                }
            }
            position_hold = new_position_hold;
            // if button on channel 9 is pressed, stop everything and erase the flash to make space for blackbox logs, but only on the ground
            if Elrs::elrs_input_to_percent(chnls[8], None) > 0.5
                && !armed
                && since_blackbox_erased.elapsed().as_millis() > 500
            {
                let result = TcBlackbox::erase_blackbox_flash_space().await;
                if result.is_ok() {
                    info!("Successfully erase blackbox log space.")
                } else {
                    error!("Unable to erase blackbox log space (the drone can still fly but the logs will be corrupted): {:?}", result.unwrap_err());
                }
                since_blackbox_erased = Instant::now();
            }
            throttle_input = Elrs::elrs_input_to_percent(chnls[2], None);
            yaw_input = Elrs::elrs_input_to_percent(chnls[0], Some(0.01)) * MAX_ACRO_RATE;
            roll_input = Elrs::elrs_input_to_percent(chnls[3], Some(0.01)) * MAX_ACRO_RATE;
            pitch_input = Elrs::elrs_input_to_percent(chnls[1], Some(0.01)) * MAX_ACRO_RATE;
            target_rates.0 = -pitch_input;
            target_rates.1 = roll_input;
            target_rates.2 = -yaw_input;
            rate_errors.0 = imu_rates.0 - target_rates.0;
            rate_errors.1 = imu_rates.1 - target_rates.1;
            rate_errors.2 = imu_rates.2 - target_rates.2;
            since_last_elrs_update = Instant::now();
        }

        // tc_println!("Position hold? {}", position_hold);

        // calc pid
        pid_pitch.setpoint(target_rates.0);
        let pid_pitch_raw_output = pid_pitch.next_control_output(imu_rates.0);
        let pid_pitch_filtered_d = pitch_d_filter.run(pid_pitch_raw_output.d);
        let pid_pitch_output =
            pid_pitch_raw_output.p + pid_pitch_raw_output.i + pid_pitch_filtered_d;

        pid_roll.setpoint(target_rates.1);
        let pid_roll_raw_output = pid_roll.next_control_output(imu_rates.1);
        let pid_roll_filtered_d = roll_d_filter.run(pid_roll_raw_output.d);
        let pid_roll_output = pid_roll_raw_output.p + pid_roll_raw_output.i + pid_roll_filtered_d;

        pid_yaw.setpoint(target_rates.2);
        let pid_yaw_raw_output = pid_yaw.next_control_output(imu_rates.2);
        let pid_yaw_filtered_d = yaw_d_filter.run(pid_yaw_raw_output.d);
        let pid_yaw_output = pid_yaw_raw_output.p + pid_yaw_raw_output.i + pid_yaw_filtered_d;

        let mut pid_vs_output = 0.0;
        let new_should_use_position_hold = position_hold
            && current_altitude.is_some()
            && (throttle_input > 0.3 || should_use_position_hold)
            && armed;
        let reset_position_hold_data = new_should_use_position_hold && !should_use_position_hold;
        if reset_position_hold_data {
            target_altitude = current_altitude.unwrap();
            pid_altitude.reset_integral_term();
            pid_vs.reset_integral_term();
        }
        should_use_position_hold = new_should_use_position_hold;
        if should_use_position_hold {
            let pid_altitude_output = pid_altitude
                .next_control_output(target_altitude - current_altitude.unwrap())
                .output;
            pid_vs_output = pid_vs
                .next_control_output(pid_altitude_output - current_vertical_speed)
                .output;
            // tc_println!("PID Alt: {}", pid_altitude_output);
            // tc_println!("PID Output: {}", pid_vs_output);
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
        let t3 = (current_throttle_value - pid_pitch_output - pid_roll_output - pid_yaw_output)
            .clamp(0.0, 1.0);
        let t4 = (current_throttle_value - pid_pitch_output + pid_roll_output + pid_yaw_output)
            .clamp(0.0, 1.0);

        if armed && blackbox_enabled && since_last_log > (UPDATE_LOOP_FREQUENCY / 30.0) as u32 {
            let imu_processor_freq_recv = imu_processor_freq_receiver.try_changed();
            if imu_processor_freq_recv.is_some() {
                last_imu_processor_freq = imu_processor_freq_recv.unwrap();
            }

            TcBlackbox::log(BlackboxLogData::new(
                (BOOT_TIME.get().elapsed().as_micros() as f64) / 1_000_000.0,
                1.0 / dt,
                last_imu_processor_freq,
                throttle_input,
                target_rates.into(),
                imu_rates.into(),
                [
                    pid_pitch_raw_output.p,
                    pid_roll_raw_output.p,
                    pid_yaw_raw_output.p,
                ],
                [
                    pid_pitch_raw_output.i,
                    pid_roll_raw_output.i,
                    pid_yaw_raw_output.i,
                ],
                [
                    pid_pitch_filtered_d,
                    pid_roll_filtered_d,
                    pid_yaw_filtered_d,
                ],
                [pid_pitch_output, pid_roll_output, pid_yaw_output],
                g_force,
            ))
            .await;
            since_last_log = 0;
        }
        since_last_log += 1;

        // TODO: implement return to home like stuff in this case scenario
        // if signal to controller is lost, turn off motors for safety
        if since_last_elrs_update.elapsed().as_millis() > 500 {
            armed = false;
        }
        CONTROL_LOOP_VALUES.signal((armed, throttle_input, [t1, t2, t3, t4]));
        CONTROL_LOOP_FREQUENCY_SIGNAL.signal(1.0 / dt);
    }
}
