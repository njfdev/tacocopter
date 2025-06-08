use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::{PIN_0, PIN_1, UART0},
    uart::{self, BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx},
    Peripheral, Peripherals,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    pubsub::{PubSubChannel, Publisher},
};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use log::{error, info, warn};
use static_cell::StaticCell;

use crate::{tc_println, ELRS_PUBSUB_CHANNEL, SHARED};

bind_interrupts!(struct UartIrq {
  UART0_IRQ => BufferedInterruptHandler<UART0>;
});

const CRC8TAB: [u8; 256] = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
];

pub fn crc8(data: &[u8], len: usize) -> u8 {
    let mut crc: u8 = 0;
    for i in 0..len {
        crc = CRC8TAB[(crc ^ data[i as usize]) as usize];
    }
    crc
}

pub async fn init_elrs(
    tx_pin: PIN_0,
    rx_pin: PIN_1,
    uart: UART0,
    spawner: &Spawner,
) -> BufferedUartTx<'_, UART0> {
    // initialize UART connection
    static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 1024])[..];
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 1024])[..];

    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 416666;
    let uart = BufferedUart::new(uart, UartIrq, tx_pin, rx_pin, tx_buf, rx_buf, uart_config);
    let (mut tx, rx) = uart.split();

    spawner.spawn(reader(rx)).unwrap();

    tx
}

pub fn get_altitude_packed(altitude: f32) -> u16 {
    let altitude_dm = ((altitude * 10.0) + 0.5) as i32;
    if altitude_dm < -10000 {
        return 0; // if less than minimum altitude, return min
    } else if altitude_dm > 0x7ffe * 10 - 5 {
        return 0xfffe;
    } else if altitude_dm < 0x8000 - 10000 {
        return (altitude_dm + 10000) as u16; // if altitude is in dm-resolution range
    }
    return (((altitude_dm + 5) / 10) as u16) | 0x8000;
}

#[embassy_executor::task]
async fn reader(mut rx: BufferedUartRx<'static, UART0>) {
    info!("Reading...");

    let mut current_packet: [u8; 1024] = [0; 1024];
    let mut current_len = 0;
    loop {
        let mut buf = [0; 1024];
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                //info!("RX first {} bytes: {:2x?}", n, &buf[..n]);
                current_packet[current_len..(current_len + n)].copy_from_slice(&buf[..n]);
                current_len += n;
            }
            Ok(0) => {
                info!("Read 0 bytes");
                Timer::after_millis(100).await;
            }
            Err(e) => {
                info!("Read error: {:?}", e);
                Timer::after_millis(100).await;
            }
            _ => {
                Timer::after_millis(100).await;
            }
        }
        // info!(
        //     "Packet ({}): {:2x?}",
        //     current_len,
        //     &current_packet[..current_len]
        // );

        if current_len == 0 {
            continue;
        }

        // process packet
        if current_packet[0] != 0xc8 {
            let mut new_starting = 0;
            for (i, byte) in current_packet.iter().enumerate() {
                if *byte == 0xc8 {
                    new_starting = i;
                    break;
                }
            }
            current_packet.copy_within(new_starting.., 0);
            current_len -= new_starting;
        }

        if current_packet[0] != 0xc8 {
            continue;
        }

        // check if packet is complete
        let len = current_packet[1] as usize + 2;
        if current_len >= len {
            handle_packet(&current_packet[..len], len).await;

            // now remove it
            current_packet.copy_within(len.., 0);
            current_len -= len;
        }
    }
}

fn unpack_rc_bits(data: &[u8; 22]) -> [u16; 16] {
    let mut channels: [u16; 16] = [0; 16];

    let mut bit_offset = 0;
    while bit_offset != 21 * 8 {
        let cur_channel = bit_offset / 11;
        let cur_channel_bit_offset = bit_offset % 11;

        let cur_byte = bit_offset / 8;
        let cur_byte_bit_offset = bit_offset % 8;

        channels[cur_channel] |=
            (((data[cur_byte] >> cur_byte_bit_offset) & 1) as u16) << cur_channel_bit_offset;
        bit_offset += 1;
    }

    return channels;
}

fn print_link_statistics(data: &[u8; 10]) {
    let up_rssi_ant1 = data[0] as i16 * -1;
    let up_rssi_ant2 = data[1] as i16 * -1;
    let up_link_quality = data[2];
    let up_snr = data[3] as i8;
    let active_ant = data[4];
    let rf_profile = data[5];
    let up_rf_power = data[6];

    let down_rssi = data[7] as i16 * -1;
    let down_link_quality = data[8];
    let down_snr = data[9] as i8;

    info!(
        "Uplink: Ant1- {} dBm, Ant2- {} dBm, Qlty-{}%, SNR-{} dB, Actv-{}, Prof-{}, Pwr-{}dB",
        up_rssi_ant1,
        up_rssi_ant2,
        up_link_quality,
        up_snr,
        active_ant + 1,
        rf_profile,
        up_rf_power
    );

    info!(
        "Downlink: RSSI- {} dBm, Qlty-{}%, SNR-{} dB",
        down_rssi, down_link_quality, down_snr,
    );
}

// TODO: Verify with CRC
async fn handle_packet(data: &[u8], len: usize) {
    let frame_type = data[2];

    match frame_type {
        // RC Channels Packed Payload
        0x16 => {
            /* The data should be 25 bytes long because this packet
             * type packets 16 channels into 22 bytes. The check byte,
             * the length byte, the frame type byte, and crc byte,
             * add 4 to that.
             */
            if len != 26 {
                error!("RC Channels Packed Payload is {} bytes too long!", len - 26);
            } else {
                let chnls = unpack_rc_bits(&data[3..25].try_into().unwrap());

                // {
                //     let mut shared = SHARED.lock().await;
                //     shared.elrs_channels = chnls.clone();
                // }
                ELRS_PUBSUB_CHANNEL.signal(chnls);

                // info!(
                //     "Channels: {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
                //     chnls[0],
                //     chnls[1],
                //     chnls[2],
                //     chnls[3],
                //     chnls[4],
                //     chnls[5],
                //     chnls[6],
                //     chnls[7],
                //     chnls[8],
                //     chnls[9],
                //     chnls[10],
                //     chnls[11],
                //     chnls[12],
                //     chnls[13],
                //     chnls[14],
                //     chnls[15]
                // );
            }
        }
        0x14 => {
            //print_link_statistics(data[3..13].try_into().unwrap());
        }
        _ => {
            warn!("Unhandled frame type: {:2x?}", frame_type);
        }
    }
}
