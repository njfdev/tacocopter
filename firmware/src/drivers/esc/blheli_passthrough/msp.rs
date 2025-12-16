use embassy_rp::pio::Instance;
use log::{info, warn};

use crate::drivers::esc::blheli_passthrough::BlHeliPassthrough;

// commands
const MSP_API_VERSION: u8 = 0x01;
const MSP_FC_VARIANT: u8 = 0x02;
const MSP_FC_VERSION: u8 = 0x03;
const MSP_BOARD_INFO: u8 = 0x04;
const MSP_BUILD_INFO: u8 = 0x05;
const MSP_FEATURE_CONFIG: u8 = 0x24;
const MSP_MOTOR: u8 = 0x68;
const MSP_BATTERY_STATE: u8 = 0x82;
const MSP_UID: u8 = 0xA0;

const MSP_PROTOCOL_VERSION: u8 = 0;
const API_VERSION_MAJOR: u8 = 1;
const API_VERSION_MINOR: u8 = 42;

fn calc_crc(data: &[u8]) -> u8 {
    let mut crc = data[3] ^ data[4];
    for i in 5..data.len() {
        crc ^= data[i];
    }
    crc
}

pub fn process_msp<'a, PIO: Instance>(
    // passthrough: &mut BlHeliPassthrough<'a, PIO>,
    rx_buffer: &[u8],
    rx_len: usize,
) -> Option<([u8; 263], usize)> {
    let cmd = rx_buffer[4];
    let rx_crc = rx_buffer[rx_len - 1];

    let mut tx_buffer: [u8; 263] = [0; 263];
    // share response data between all commands
    tx_buffer[0] = rx_buffer[0];
    tx_buffer[1] = rx_buffer[1];
    tx_buffer[2] = 0x3E;
    tx_buffer[4] = cmd;

    if calc_crc(&rx_buffer[..(rx_len - 1)]) != rx_crc {
        warn!("Command ({:x?}) did not match crc ({:x?}", cmd, rx_crc);
        return None;
    }

    match cmd {
        MSP_API_VERSION => {
            tx_buffer[3] = 3; // len

            // data
            tx_buffer[5] = MSP_PROTOCOL_VERSION;
            tx_buffer[6] = API_VERSION_MAJOR;
            tx_buffer[7] = API_VERSION_MINOR;
        }
        MSP_FC_VARIANT => {
            tx_buffer[3] = 4;

            tx_buffer[5] = b'T';
            tx_buffer[6] = b'A';
            tx_buffer[7] = b'C';
            tx_buffer[8] = b'O';
        }
        MSP_FC_VERSION => {
            tx_buffer[3] = 3;

            tx_buffer[5] = env!("CARGO_PKG_VERSION_MAJOR").parse::<u8>().unwrap();
            tx_buffer[6] = env!("CARGO_PKG_VERSION_MINOR").parse::<u8>().unwrap();
            tx_buffer[7] = env!("CARGO_PKG_VERSION_PATCH").parse::<u8>().unwrap();
        }
        MSP_BOARD_INFO => {
            tx_buffer[3] = 79;

            tx_buffer[5] = b'T';
            tx_buffer[6] = b'A';
            tx_buffer[7] = b'C';
            tx_buffer[8] = b'O';

            tx_buffer[13] = 28; // Name string length
            tx_buffer[14] = b'T';
            tx_buffer[15] = b'a';
            tx_buffer[16] = b'c';
            tx_buffer[17] = b'o';
            tx_buffer[18] = b'C';
            tx_buffer[19] = b'o';
            tx_buffer[20] = b'p';
            tx_buffer[21] = b't';
            tx_buffer[22] = b'e';
            tx_buffer[23] = b'e';
            tx_buffer[24] = b' ';
            tx_buffer[25] = b'F';
            tx_buffer[26] = b'l';
            tx_buffer[27] = b'i';
            tx_buffer[28] = b'g';
            tx_buffer[29] = b'h';
            tx_buffer[30] = b't';
            tx_buffer[31] = b' ';
            tx_buffer[32] = b'C';
            tx_buffer[33] = b'o';
            tx_buffer[34] = b'n';
            tx_buffer[35] = b't';
            tx_buffer[36] = b'r';
            tx_buffer[37] = b'o';
            tx_buffer[38] = b'l';
            tx_buffer[39] = b'l';
            tx_buffer[40] = b'e';
            tx_buffer[41] = b'r';
        }
        MSP_BUILD_INFO => {
            tx_buffer[3] = 26;
        }
        MSP_FEATURE_CONFIG => {
            tx_buffer[3] = 4;
        }
        MSP_MOTOR => {
            tx_buffer[3] = 16; // 8 total motors (we'll only have 4)

            // 1000 for running motors, 0 for stopped
            for i in 0..4 {
                tx_buffer[5 + (i * 2)] = 0xE8;
                tx_buffer[5 + (i * 2) + 1] = 0x03;
            }
        }
        MSP_BATTERY_STATE => {
            tx_buffer[3] = 10;
            // TODO: actually pull PM02D battery levels here
        }
        MSP_UID => {
            tx_buffer[3] = 12;
        }
        _ => {
            warn!("Unrecognized command: {:x?}", cmd);
            return None;
        }
    }

    let tx_len = tx_buffer[3] as usize + 5;
    tx_buffer[tx_len] = calc_crc(&tx_buffer[..tx_len]);

    return Some((tx_buffer, tx_len + 1));
}
