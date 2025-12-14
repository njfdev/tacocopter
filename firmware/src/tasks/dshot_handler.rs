use embassy_rp::peripherals::PIO0;
use embassy_time::Instant;

use crate::{
    consts::DSHOT_UPDATE_FREQ,
    drivers::esc::dshot_pio::{DshotPio, DshotPioTrait},
    global::CONTROL_LOOP_VALUES,
    tools::yielding_timer::YieldingTimer,
};

const MAX_THROTTLE_PERCENT: f32 = 1.0;
#[embassy_executor::task]
pub async fn dshot_handler(mut dshot: DshotPio<'static, 4, PIO0>) {
    let mut armed = false;
    let mut mtr_pwrs = [0.0, 0.0, 0.0, 0.0];
    let mut _since_last_throttle_update = Instant::now();
    let mut time_since_armed = Instant::now();
    let mut since_last = Instant::now();

    loop {
        since_last = YieldingTimer::after_micros(
            ((1_000_000.0 / DSHOT_UPDATE_FREQ) as u64)
                .checked_sub(since_last.elapsed().as_micros())
                .unwrap_or_default(),
        )
        .await;

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
            _since_last_throttle_update = Instant::now();
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
        }
    }
}
