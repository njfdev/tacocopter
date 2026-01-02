use embassy_futures::select::select;
use embassy_rp::peripherals::USB;
use embassy_rp::pio;
use embassy_rp::rom_data::{reboot, reset_to_usb_boot};
use embassy_rp::{peripherals::PIO0, usb::Driver};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Instant;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;
use log::info;
use static_cell::StaticCell;

use crate::global::FC_PASSTHROUGH_SIGNAL;
use crate::tasks::realtime::control_loop::ControlData;
use crate::{
    consts::DSHOT_UPDATE_FREQ,
    drivers::esc::{
        blheli_passthrough::BlHeliPassthrough,
        dshot_pio::{DshotPio, DshotPioTrait},
        EscPins,
    },
    tools::yielding_timer::YieldingTimer,
};

const MAX_THROTTLE_PERCENT: f32 = 1.0;

pub struct EscLoop {
    esc_pins: EscPins<'static, PIO0>,
    passthrough_enabled: bool,

    // DShot
    dshot: DshotPio<'static, 4, PIO0>,
    armed: bool,
    time_since_armed: Instant,
}

impl EscLoop {
    pub async fn new(
        mut esc_pins: EscPins<'static, PIO0>,
        mut dshot: DshotPio<'static, 4, PIO0>,
    ) -> Self {
        dshot.enable_dshot(&mut esc_pins);

        Self {
            esc_pins,
            passthrough_enabled: false,

            dshot,
            armed: false,
            time_since_armed: Instant::now(),
        }
    }

    pub async fn process(&mut self, control_data: ControlData) {
        let passthrough_recv = FC_PASSTHROUGH_SIGNAL.try_take();
        if passthrough_recv.is_some() {
            let new_passthrough_enabled = passthrough_recv.unwrap();
            if new_passthrough_enabled != self.passthrough_enabled {
                if new_passthrough_enabled {
                    self.dshot.disable_dshot(&mut self.esc_pins);
                } else {
                    self.dshot.enable_dshot(&mut self.esc_pins);
                }
            }
            self.passthrough_enabled = new_passthrough_enabled;
        }

        if self.passthrough_enabled {
            return;
        }

        self.handle_dshot_logic(control_data).await;
    }

    async fn handle_dshot_logic<'a>(&mut self, control_data: ControlData) {
        if control_data.armed != self.armed {
            self.armed = control_data.throttle_input < 0.01 && control_data.armed;
            if self.armed {
                self.time_since_armed = Instant::now();
            } else if control_data.armed == false {
                // if just disarmed, send the motor stop packet
                self.dshot.command(&mut self.esc_pins, [0, 0, 0, 0]);
            }
        }

        if self.armed {
            let mut dshot_msgs: [u16; 4] = [0, 0, 0, 0];
            for (i, motor_pwr) in control_data.motor_pwrs.iter().enumerate() {
                let dshot_cmd = if self.armed && self.time_since_armed.elapsed().as_millis() > 1000
                {
                    (motor_pwr.max(0.02).min(MAX_THROTTLE_PERCENT) * 1999.0) as u16 + 48
                } else {
                    0
                };
                let dshot_data = dshot_cmd << 1;
                let dshot_crc = (dshot_data ^ (dshot_data >> 4) ^ (dshot_data >> 8)) & 0x0f;
                dshot_msgs[i] = (dshot_data << 4) + dshot_crc;
            }
            self.dshot.command(&mut self.esc_pins, dshot_msgs);
        }
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
