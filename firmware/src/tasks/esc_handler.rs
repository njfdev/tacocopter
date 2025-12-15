use embassy_futures::select::select;
use embassy_rp::peripherals::USB;
use embassy_rp::pio;
use embassy_rp::rom_data::{reboot, reset_to_usb_boot};
use embassy_rp::{peripherals::PIO0, usb::Driver};
use embassy_time::Instant;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;
use log::info;

use crate::global::FC_PASSTHROUGH_SIGNAL;
use crate::{
    consts::DSHOT_UPDATE_FREQ,
    drivers::esc::{
        blheli_passthrough::BlHeliPassthrough,
        dshot_pio::{DshotPio, DshotPioTrait},
        EscPins,
    },
    global::{CONTROL_LOOP_VALUES, SHARED},
    tools::yielding_timer::YieldingTimer,
};

const MAX_THROTTLE_PERCENT: f32 = 1.0;
#[embassy_executor::task]
pub async fn esc_handler(
    mut esc_pins: EscPins<'static, PIO0>,
    mut dshot: DshotPio<'static, 4, PIO0>,
    mut blheli_passthrough: BlHeliPassthrough<'static, PIO0>,
    mut serial_class: CdcAcmClass<'static, Driver<'static, USB>>,
) {
    dshot.enable_dshot(&mut esc_pins);

    let mut armed = false;
    let mut mtr_pwrs = [0.0, 0.0, 0.0, 0.0];
    let mut time_since_armed = Instant::now();
    let mut since_last = Instant::now();

    loop {
        since_last = YieldingTimer::after_micros(
            ((1_000_000.0 / DSHOT_UPDATE_FREQ) as u64)
                .checked_sub(since_last.elapsed().as_micros())
                .unwrap_or_default(),
        )
        .await;

        if FC_PASSTHROUGH_SIGNAL.try_take().unwrap_or(false) {
            info!("Enabling BlHeli FC Passthrough");
            dshot.disable_dshot(&mut esc_pins);
            blheli_passthrough.enable_passthrough(&mut esc_pins);

            select(
                handle_passthrough_session(
                    &mut esc_pins,
                    &mut blheli_passthrough,
                    &mut serial_class,
                ),
                async {
                    loop {
                        if !FC_PASSTHROUGH_SIGNAL.wait().await {
                            return;
                        }
                    }
                },
            )
            .await;

            info!("Disabling BlHeli FC Passthrough");
            blheli_passthrough.disable_passthrough(&mut esc_pins);
            dshot.enable_dshot(&mut esc_pins);

            {
                let mut shared = SHARED.lock().await;
                shared.state_data.blheli_passthrough = false;
            }
        }

        handle_dshot_logic(
            &mut esc_pins,
            &mut dshot,
            &mut armed,
            &mut mtr_pwrs,
            &mut time_since_armed,
        )
        .await;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn handle_passthrough_session(
    esc_pins: &mut EscPins<'static, PIO0>,
    blheli_passthrough: &mut BlHeliPassthrough<'static, PIO0>,
    serial_class: &mut CdcAcmClass<'static, Driver<'static, USB>>,
) -> Result<(), Disconnected> {
    serial_class.wait_connection().await;
    let mut buf = [0_u8; 512];
    loop {
        let n = serial_class.read_packet(&mut buf).await?;
        info!("rx data: {:?}", &buf[..n]);
        let data_res = blheli_passthrough.process_serial_data(&buf[..n]);
        if data_res.is_some() {
            let data = data_res.unwrap();
            let tx_data = &data.0[..data.1];
            info!("tx data: {:?}", &data.0[..data.1]);
            for chunk in tx_data.chunks(serial_class.max_packet_size() as usize) {
                serial_class.write_packet(chunk).await?;
            }
        }
    }
}

async fn handle_dshot_logic<'a, PIO: pio::Instance + 'a>(
    esc_pins: &mut EscPins<'a, PIO>,
    dshot: &mut DshotPio<'a, 4, PIO>,
    armed: &mut bool,
    mtr_pwrs: &mut [f32; 4],
    time_since_armed: &mut Instant,
) {
    let control_loop_recv = CONTROL_LOOP_VALUES.try_take();
    if control_loop_recv.is_some() {
        // && since_last_throttle_update.elapsed().as_micros() > 50000 {
        let (armed_recv, throttle_percent, mtr_pwrs_recv) = control_loop_recv.unwrap();
        *mtr_pwrs = mtr_pwrs_recv;
        // tc_println!("Motor pwrs: {:?}", mtr_pwrs);
        if &armed_recv != armed {
            *armed = throttle_percent < 0.01 && armed_recv;
            if *armed {
                *time_since_armed = Instant::now();
            } else if armed_recv == false {
                // if just disarmed, send the motor stop packet
                dshot.command(esc_pins, [0, 0, 0, 0]);
            }
        }
    }

    // let pwm_pwr = (((throttle - 176) as f32) / 1634.0) * 90.0 + 10.0;
    // pwm.set_duty_cycle_percent(dshot_cmd as u8).unwrap();
    if *armed {
        let mut dshot_msgs: [u16; 4] = [0, 0, 0, 0];
        for (i, motor_pwr) in mtr_pwrs.iter().enumerate() {
            let dshot_cmd = if *armed && time_since_armed.elapsed().as_millis() > 1000 {
                (motor_pwr.max(0.02).min(MAX_THROTTLE_PERCENT) * 1999.0) as u16 + 48
            } else {
                0
            };
            let dshot_data = dshot_cmd << 1;
            let dshot_crc = (dshot_data ^ (dshot_data >> 4) ^ (dshot_data >> 8)) & 0x0f;
            dshot_msgs[i] = (dshot_data << 4) + dshot_crc;
        }
        dshot.command(esc_pins, dshot_msgs);
    }
}
