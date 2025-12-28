use embassy_rp::pio::Instance;
use embassy_time::{Duration, Timer};
use log::{info, warn};

use crate::drivers::esc::{blheli_passthrough::BlHeliPassthrough, EscPins};

const CMD_DEVICE_FLASH_INIT: u8 = 0x37;

pub async fn process_4_way<'a, PIO: Instance>(
    passthrough: &mut BlHeliPassthrough<'a, PIO>,
    esc_pins: &mut EscPins<'a, PIO>,
) -> Option<([u8; 263], usize)> {
    let rx_buffer = passthrough.rx_buffer;
    let rx_len = passthrough.rx_len;

    let cmd = rx_buffer[1];
    let addr_high = rx_buffer[2];
    let addr_low = rx_buffer[3];
    let rx_param_len = rx_buffer[4];
    let param = rx_buffer[5];

    let mut crc = 0;
    for i in 0..(rx_param_len as usize + 5) {
        crc = crc_xmodem_update(crc, rx_buffer[i]);
    }
    let mut rx_crc = ((rx_buffer[rx_len - 2] as u16) << 8) | (rx_buffer[rx_len - 1] as u16);

    if (rx_crc != crc) {
        warn!(
            "Received crc {:04x?} does not match expected crc {:04x?}",
            rx_crc, crc
        );
    }

    let mut tx_buffer: [u8; 263] = [0; 263];

    match cmd {
        CMD_DEVICE_FLASH_INIT => {
            return None;
            let esc_pin = param;

            // we only have 4 ESCs available
            if esc_pin < 4 {
                let boot_init: [u8; 17] = [
                    0, 0, 0, 0, 0, 0, 0, 0, 0x0D, b'B', b'L', b'H', b'e', b'l', b'i', 0xF4, 0x7D,
                ];
                passthrough.send_data_to_esc(
                    esc_pins,
                    esc_pin as usize,
                    &boot_init,
                    boot_init.len(),
                    false,
                );
                // give Esc time to respond
                // passthrough
                //     .delay_while_read(Duration::from_millis(50))
                //     .await;
            } else {
                warn!(
                    "4 Way command tried to access esc {}, but only 4 are available!",
                    esc_pin + 1
                );
            }
        }
        _ => {
            warn!("Unexpected 4 way command: {:02x?}", cmd);
        }
    }

    None
}

fn crc_xmodem_update(crc: u16, byte: u8) -> u16 {
    let mut new_crc = crc ^ ((byte as u16) << 8);
    for i in 0..8 {
        if (new_crc & 0x8000) != 0 {
            new_crc = (new_crc << 1) ^ 0x1021;
        } else {
            new_crc <<= 1;
        }
    }
    return new_crc;
}
