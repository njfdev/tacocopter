use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::{PIN_0, PIN_1, UART0},
    uart::{self, BufferedInterruptHandler, BufferedUart, BufferedUartTx},
    Peri,
};
use embedded_io_async::Write;
use heapless::Vec;
use static_cell::StaticCell;

use crate::{
    concat_elrs_bytes,
    drivers::elrs::{
        elrs_rx::elrs_receive_handler,
        elrs_tx_packets::{BarometerAltitudePacket, BatteryStatePacket, GPSPacket},
    },
};
pub mod elrs_rx;
pub mod elrs_tx_packets;

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

pub struct Elrs {
    uart_tx: BufferedUartTx,
}

pub enum ElrsTxPacket {
    BarometerAltitude(BarometerAltitudePacket),
    BatteryState(BatteryStatePacket),
    GPS(GPSPacket),
}

impl Elrs {
    pub fn new(
        tx_pin: Peri<'static, PIN_0>,
        rx_pin: Peri<'static, PIN_1>,
        uart: Peri<'static, UART0>,
        spawner: Spawner,
    ) -> Self {
        // initialize UART connection
        static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
        let tx_buf = &mut TX_BUF.init([0; 1024])[..];
        static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
        let rx_buf = &mut RX_BUF.init([0; 1024])[..];

        let mut uart_config = uart::Config::default();
        uart_config.baudrate = 416666;
        let uart = BufferedUart::new(uart, tx_pin, rx_pin, UartIrq, tx_buf, rx_buf, uart_config);
        let (tx, rx) = uart.split();

        spawner.spawn(elrs_receive_handler(rx)).unwrap();

        Self { uart_tx: tx }
    }

    pub fn crc8(data: &[u8]) -> u8 {
        let len = data.len();
        let mut crc: u8 = 0;
        for i in 0..len {
            crc = CRC8TAB[(crc ^ data[i as usize]) as usize];
        }
        crc
    }

    fn get_altitude_packed(altitude: f32) -> u16 {
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

    pub async fn send_packet(&mut self, packet: ElrsTxPacket) {
        let frame_type: u8;
        let frame_bytes: Vec<u8, 64>;
        // let frame_bytes_len: usize;

        match packet {
            ElrsTxPacket::BarometerAltitude(packet) => {
                frame_type = 0x09;
                let packed_altitude = Elrs::get_altitude_packed(packet.altitude).to_be_bytes();
                let packed_vs = ((packet.vertical_speed * 100.0) as i16).to_be_bytes();
                frame_bytes = concat_elrs_bytes!(packed_altitude, packed_vs);
            }
            ElrsTxPacket::BatteryState(packet) => {
                frame_type = 0x08;
                let packed_voltage = ((packet.voltage * 10.0) as u16).to_be_bytes();
                let packed_current = ((packet.current * 10.0) as u16).to_be_bytes();
                let packed_capacity = (packet.capacity as u32).to_be_bytes();
                let packed_remaining = (packet.battery_percentage as u8).to_be_bytes();
                frame_bytes = concat_elrs_bytes!(
                    packed_voltage,
                    packed_current,
                    [packed_capacity[1], packed_capacity[2], packed_capacity[3]],
                    packed_remaining
                );
            }
            ElrsTxPacket::GPS(packet) => {
                frame_type = 0x02;
                let packed_latitude = ((packet.latitude * 10_000_000.0) as i32).to_be_bytes();
                let packed_longitude = ((packet.longitude * 10_000_000.0) as i32).to_be_bytes();
                let packed_ground_speed =
                    ((packet.ground_speed * 10_000.0 / 3600.0) as u16).to_be_bytes();
                let packed_gps_heading = ((packet.gps_heading * 100.0) as u16).to_be_bytes();
                let packed_altitude = ((packet.gps_altitude + 1000.0) as u16).to_be_bytes();
                let packed_num_sats = (packet.num_sats as u8).to_be_bytes();
                frame_bytes = concat_elrs_bytes!(
                    packed_latitude,
                    packed_longitude,
                    packed_ground_speed,
                    packed_gps_heading,
                    packed_altitude,
                    packed_num_sats
                )
            }
        }

        let frame_data = concat_elrs_bytes!([frame_type], frame_bytes.iter().map(|val| *val));

        let checksum = Elrs::crc8(frame_data.as_slice());

        let _ = self
            .uart_tx
            .write_all(
                concat_elrs_bytes!(
                    [0xc8, frame_data.len() as u8 + 1],
                    frame_data.iter().map(|val| *val),
                    [checksum]
                )
                .as_slice(),
            )
            .await;
    }

    // if deadzone is passed as a value, then the output is scaled to -1.0 to 1.0, otherwise it is scaled from 0.0 to 1.0
    pub fn elrs_input_to_percent(input: u16, deadzone_opt: Option<f32>) -> f32 {
        let input_percent = if input < 172 {
            0.0
        } else if input > 1810 {
            1.0
        } else {
            ((input - 172) as f32) / 1638.0
        };

        if deadzone_opt.is_none() {
            return input_percent.clamp(0.0, 1.0);
        }

        let rescaled_input_percent = (input_percent - 0.5) * 2.0;

        let deadzone = deadzone_opt.unwrap();

        if rescaled_input_percent.abs() < deadzone * 2.0 {
            return 0.0;
        };

        if rescaled_input_percent > 0.0 {
            return ((rescaled_input_percent - deadzone) / (1.0 - deadzone)).clamp(0.0, 1.0);
        }
        return ((rescaled_input_percent + deadzone) / (1.0 - deadzone)).clamp(-1.0, 0.0);
    }
}
