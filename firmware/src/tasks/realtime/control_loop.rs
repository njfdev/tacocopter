use core::f32::consts::PI;

use biquad::{Biquad, Coefficients, DirectForm2Transposed, ToHertz, Q_BUTTERWORTH_F32};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use embassy_time::Instant;
use log::{error, info};
use micromath::F32Ext;
use pid::Pid;

use crate::{
    consts::UPDATE_LOOP_FREQUENCY,
    drivers::{
        elrs::Elrs,
        tc_store::{blackbox::TcBlackbox, types::PIDValues, TcStore},
    },
    global::{ELRS_WATCH, PID_WATCH},
    tasks::realtime::imu_loop::ImuData,
};

const MAX_ACRO_RATE: f32 = 200.0; // What is the target rotation rate at full throttle input?

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

pub struct ControlLoop {
    // imu
    imu_rates: (f32, f32, f32),
    g_force: f32,

    // elrs
    elrs_receiver: watch::Receiver<'static, CriticalSectionRawMutex, [u16; 16], 2>,
    armed: bool,
    since_last_elrs_update: Instant,
    throttle_input: f32,
    target_rates: (f32, f32, f32),

    // pid
    pid_pitch: Pid<f32>,
    pid_roll: Pid<f32>,
    pid_yaw: Pid<f32>,
    pitch_d_filter: DirectForm2Transposed<f32>,
    roll_d_filter: DirectForm2Transposed<f32>,
    yaw_d_filter: DirectForm2Transposed<f32>,
    pid_receiver: watch::Receiver<'static, CriticalSectionRawMutex, PIDValues, 1>,

    // blackbox
    since_blackbox_erased: Instant,
}

impl ControlLoop {
    pub async fn new() -> Self {
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

        let mut pid_pitch = Pid::new(0.0, 0.25);
        let mut pid_roll = Pid::new(0.0, 0.25);
        let mut pid_yaw = Pid::new(0.0, 0.25);
        set_pid_values(
            &TcStore::get::<PIDValues>().await,
            &mut pid_pitch,
            &mut pid_roll,
            &mut pid_yaw,
        );

        // let mut blackbox_settings = TcStore::get::<BlackboxSettings>().await;
        // let mut blackbox_enabled = blackbox_settings.enabled;
        // let mut blackbox_settings_receiver = BLACKBOX_SETTINGS_WATCH.receiver().unwrap();
        // let mut since_last_log = 0;

        Self {
            imu_rates: (0.0, 0.0, 0.0),
            g_force: 1.0,

            elrs_receiver: ELRS_WATCH.receiver().unwrap(),
            armed: false,
            since_last_elrs_update: Instant::now(),
            throttle_input: 0.0,
            target_rates: (0.0, 0.0, 0.0),

            pid_pitch,
            pid_roll,
            pid_yaw,
            pitch_d_filter: DirectForm2Transposed::new(coeffs),
            roll_d_filter: DirectForm2Transposed::new(coeffs),
            yaw_d_filter: DirectForm2Transposed::new(coeffs),
            pid_receiver: PID_WATCH.receiver().unwrap(),

            since_blackbox_erased: Instant::now(),
        }
    }

    fn process_imu(&mut self, imu_data: ImuData) {
        // TODO: switch this to radians and tune PID in radians (why did I have to use degrees...)
        self.imu_rates = (
            imu_data.gyro_data.0 * 180.0 / PI,
            imu_data.gyro_data.1 * 180.0 / PI,
            imu_data.gyro_data.2 * 180.0 / PI,
        );
        self.g_force = (imu_data.accel_data.0.powi(2)
            + imu_data.accel_data.1.powi(2)
            + imu_data.accel_data.2.powi(2))
        .sqrt();
    }

    async fn process_elrs(&mut self) {
        let chnls_recv = self.elrs_receiver.try_changed();
        if chnls_recv.is_some() {
            let chnls = chnls_recv.unwrap();
            let new_armed = Elrs::elrs_input_to_percent(chnls[4], None) > 0.5;
            if !self.armed && new_armed {
                self.pid_yaw.reset_integral_term();
                self.pid_pitch.reset_integral_term();
                self.pid_roll.reset_integral_term();
                self.yaw_d_filter.reset_state();
                self.pitch_d_filter.reset_state();
                self.roll_d_filter.reset_state();
                TcBlackbox::start_log().await;
            }
            if self.armed && !new_armed {
                TcBlackbox::stop_log().await;
            }
            self.armed = new_armed;
            // if button on channel 9 is pressed, stop everything and erase the flash to make space for blackbox logs, but only on the ground
            if Elrs::elrs_input_to_percent(chnls[8], None) > 0.5
                && !self.armed
                && self.since_blackbox_erased.elapsed().as_millis() > 500
            {
                let result = TcBlackbox::erase_blackbox_flash_space().await;
                if result.is_ok() {
                    info!("Successfully erase blackbox log space.")
                } else {
                    error!("Unable to erase blackbox log space (the drone can still fly but the logs will be corrupted): {:?}", result.unwrap_err());
                }
                self.since_blackbox_erased = Instant::now();
            }
            self.throttle_input = Elrs::elrs_input_to_percent(chnls[2], None);
            let yaw_input = Elrs::elrs_input_to_percent(chnls[0], Some(0.01)) * MAX_ACRO_RATE;
            let roll_input = Elrs::elrs_input_to_percent(chnls[3], Some(0.01)) * MAX_ACRO_RATE;
            let pitch_input = Elrs::elrs_input_to_percent(chnls[1], Some(0.01)) * MAX_ACRO_RATE;
            self.target_rates.0 = -pitch_input;
            self.target_rates.1 = roll_input;
            self.target_rates.2 = -yaw_input;
            self.since_last_elrs_update = Instant::now();
        }

        // TODO: implement return to home like stuff in this case scenario
        // if signal to controller is lost, turn off motors for safety
        if self.since_last_elrs_update.elapsed().as_millis() > 500 {
            self.armed = false;
        }
    }

    fn process_pid(&mut self) -> (f32, f32, f32) {
        let pid_recv = self.pid_receiver.try_changed();
        if pid_recv.is_some() {
            let new_pid_values = pid_recv.unwrap();
            set_pid_values(
                &new_pid_values,
                &mut self.pid_pitch,
                &mut self.pid_roll,
                &mut self.pid_yaw,
            );
        }

        // calc pid
        self.pid_pitch.setpoint(self.target_rates.0);
        let pid_pitch_raw_output = self.pid_pitch.next_control_output(self.imu_rates.0);
        let pid_pitch_filtered_d = self.pitch_d_filter.run(pid_pitch_raw_output.d);
        let pid_pitch_output =
            pid_pitch_raw_output.p + pid_pitch_raw_output.i + pid_pitch_filtered_d;

        self.pid_roll.setpoint(self.target_rates.1);
        let pid_roll_raw_output = self.pid_roll.next_control_output(self.imu_rates.1);
        let pid_roll_filtered_d = self.roll_d_filter.run(pid_roll_raw_output.d);
        let pid_roll_output = pid_roll_raw_output.p + pid_roll_raw_output.i + pid_roll_filtered_d;

        self.pid_yaw.setpoint(self.target_rates.2);
        let pid_yaw_raw_output = self.pid_yaw.next_control_output(self.imu_rates.2);
        let pid_yaw_filtered_d = self.yaw_d_filter.run(pid_yaw_raw_output.d);
        let pid_yaw_output = pid_yaw_raw_output.p + pid_yaw_raw_output.i + pid_yaw_filtered_d;

        (pid_pitch_output, pid_roll_output, pid_yaw_output)
    }

    fn process_blackbox(&mut self) {
        // TODO: add back blackbox logic when logging to Jetson

        // let blackbox_settings_recv = blackbox_settings_receiver.try_changed();
        // if blackbox_settings_recv.is_some() {
        //     blackbox_settings = blackbox_settings_recv.unwrap();
        // }
        // if !armed && blackbox_enabled != blackbox_settings.enabled {
        //     blackbox_enabled = blackbox_settings.enabled;
        // }

        // if armed && blackbox_enabled && since_last_log > (UPDATE_LOOP_FREQUENCY / 30.0) as u32 {
        //     let imu_processor_freq_recv = imu_processor_freq_receiver.try_changed();
        //     if imu_processor_freq_recv.is_some() {
        //         last_imu_processor_freq = imu_processor_freq_recv.unwrap();
        //     }

        //     TcBlackbox::log(BlackboxLogData::new(
        //         (BOOT_TIME.get().elapsed().as_micros() as f64) / 1_000_000.0,
        //         1.0 / dt,
        //         last_imu_processor_freq,
        //         throttle_input,
        //         target_rates.into(),
        //         imu_rates.into(),
        //         [
        //             pid_pitch_raw_output.p,
        //             pid_roll_raw_output.p,
        //             pid_yaw_raw_output.p,
        //         ],
        //         [
        //             pid_pitch_raw_output.i,
        //             pid_roll_raw_output.i,
        //             pid_yaw_raw_output.i,
        //         ],
        //         [
        //             pid_pitch_filtered_d,
        //             pid_roll_filtered_d,
        //             pid_yaw_filtered_d,
        //         ],
        //         [pid_pitch_output, pid_roll_output, pid_yaw_output],
        //         g_force,
        //     ))
        //     .await;
        //     since_last_log = 0;
        // }
        // since_last_log += 1;
    }

    pub async fn process(&mut self, imu_data: ImuData) -> ControlData {
        self.process_imu(imu_data);
        self.process_elrs().await;
        let (pid_pitch_output, pid_roll_output, pid_yaw_output) = self.process_pid();

        self.process_blackbox();

        let t1 = (self.throttle_input + pid_pitch_output + pid_roll_output - pid_yaw_output)
            .clamp(0.0, 1.0);
        let t2 = (self.throttle_input + pid_pitch_output - pid_roll_output + pid_yaw_output)
            .clamp(0.0, 1.0);
        let t3 = (self.throttle_input - pid_pitch_output - pid_roll_output - pid_yaw_output)
            .clamp(0.0, 1.0);
        let t4 = (self.throttle_input - pid_pitch_output + pid_roll_output + pid_yaw_output)
            .clamp(0.0, 1.0);

        ControlData {
            armed: self.armed,
            throttle_input: self.throttle_input,
            motor_pwrs: [t1, t2, t3, t4],
        }
    }
}

pub struct ControlData {
    pub armed: bool,
    pub throttle_input: f32,
    pub motor_pwrs: [f32; 4],
}
