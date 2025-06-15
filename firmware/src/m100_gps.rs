use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::{PIN_8, PIN_9, UART1},
    uart::{self, BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx},
    Peripheral, Peripherals,
};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use log::{error, info, warn};
use static_cell::StaticCell;

use crate::{tc_print, tc_println, GPS_SIGNAL, SHARED};

bind_interrupts!(struct UartIrq {
  UART1_IRQ => BufferedInterruptHandler<UART1>;
});

pub struct GPSPayload {
    pub itow: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub nanoseconds: i32,
    pub time_accuracy: u32,
    // TODO: implement this with a enum
    pub fix_type: u8,
    pub valid_fix: bool,
    pub valid_date: bool,
    pub valid_time: bool,
    pub sat_num: u8,
    pub longitude: f64,
    pub latitude: f64,
    // in meters
    pub ellipsoid_height: f32,
    pub msl_height: f32,
    pub horizontal_accuracy_estimate: f32,
    pub vertical_accuracy_estimate: f32,
    // in m/s
    pub north_vel: f32,
    pub east_vel: f32,
    pub down_vel: f32,
    pub ground_speed: f32,
    pub ground_speed_accuracy: f32,
    pub motion_heading: f32,
    pub heading_accuracy: f32,
    // dilution of position
    pub dop: f32,
}

pub async fn init_gps(
    tx_pin: PIN_8,
    rx_pin: PIN_9,
    uart: UART1,
    spawner: &Spawner,
) -> BufferedUartTx<'_, UART1> {
    // initialize UART connection
    static TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 1024])[..];
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 1024])[..];

    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 115200;
    let uart = BufferedUart::new(uart, UartIrq, tx_pin, rx_pin, tx_buf, rx_buf, uart_config);
    let (mut tx, rx) = uart.split();

    spawner.spawn(reader(rx)).unwrap();

    tx
}

#[embassy_executor::task]
async fn reader(mut rx: BufferedUartRx<'static, UART1>) {
    info!("Reading...");
    let mut current_packet: [u8; 8192] = [0; 8192];
    let mut current_len = 0;
    let mut time_since_last = Instant::now();
    loop {
        let mut buf = [0; 8192];
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

        if current_len < 2 {
            continue;
        }

        // process packet
        if current_packet[0] != 0xb5 || current_packet[1] != 0x62 {
            let mut new_starting = 0;
            for (i, byte) in current_packet.iter().enumerate() {
                if *byte == 0xb5 {
                    new_starting = i;
                    current_packet.copy_within(new_starting.., 0);
                    current_len -= new_starting;
                    break;
                }
            }
        }

        if current_packet[0] != 0xb5 || current_packet[1] != 0x62 {
            continue;
        }

        // the length bytes are at bytes 5-6, so continue if we don't have that much yet
        if current_len < 6 {
            continue;
        }

        // check if packet is complete
        let len = current_packet[4] as usize + ((current_packet[5] as usize) << 8) + 8; // 8 bytes not included in payload length
        if current_len >= len {
            if Instant::now()
                .checked_duration_since(time_since_last)
                .unwrap()
                .as_millis()
                > 0
            {
                handle_packet(&current_packet[..len], len).await;
                time_since_last = Instant::now();
            }

            // now remove it
            current_packet.copy_within(len.., 0);
            current_len -= len;
        }
    }
}

async fn handle_packet(data: &[u8], len: usize) {
    let message_class = data[2];
    let message_id = data[3];
    let payload = &data[6..(len - 2)];
    let payload_len = len - 8;

    // tc_println!(
    //     "Received message with class {:02x?} and id {:02x?} and length {}: ",
    //     message_class,
    //     message_id,
    //     payload_len
    // );

    match message_class {
        // RC Channels Packed Payload
        0x01 => match message_id {
            0x07 => {
                let itow = payload[0] as u32
                    + ((payload[1] as u32) << 8)
                    + ((payload[2] as u32) << 16)
                    + ((payload[3] as u32) << 24);
                // tc_println!("iTOW: {}ms", itow);

                let year = payload[4] as u16 + ((payload[5] as u16) << 8);
                // tc_println!("Year: {}", year);

                let month = payload[6];
                // tc_println!("Month: {}", month);

                let day = payload[7];
                // tc_println!("Day: {}", day);

                let hour = payload[8];
                // tc_println!("Hour: {}", hour);

                let min = payload[9];
                // tc_println!("Minute: {}", min);

                let sec = payload[10];
                // tc_println!("Seconds: {}", sec);

                let nano = payload[16] as i32
                    + ((payload[17] as i32) << 8)
                    + ((payload[18] as i32) << 16)
                    + ((payload[19] as i32) << 24);
                // tc_println!("Nanoseconds: {}", nano);

                let time_accuracy = payload[12] as u32
                    + ((payload[13] as u32) << 8)
                    + ((payload[14] as u32) << 16)
                    + ((payload[15] as u32) << 24);
                // tc_println!("Time Accuracy: {}ns", time_accuracy);

                let fix_type = payload[20];
                // tc_println!(
                //     "Fix Type: {}",
                //     (match fix_type {
                //         0 => "No Fix",
                //         1 => "Dead Reckoning only",
                //         2 => "2D Fix",
                //         3 => "3D Fix",
                //         4 => "GNSS + Dead Reckoning combined",
                //         5 => "Time Only Fix",
                //         _ => "Unknown",
                //     })
                // );

                let is_fix_valid = payload[21] & 1 == 1;
                // tc_println!("Is Fix Valid: {}", is_fix_valid);

                let is_date_valid = payload[22] & 0b1000000 == 1;
                // tc_println!("Is Date Valid: {}", is_date_valid);

                let is_time_valid = payload[22] & 0b10000000 == 1;
                // tc_println!("Is Time Valid: {}", is_time_valid);

                let num_sats = payload[23];
                // tc_println!("Number of Satellites: {}", num_sats);

                let lon = ((payload[24] as i32
                    + ((payload[25] as i32) << 8)
                    + ((payload[26] as i32) << 16)
                    + ((payload[27] as i32) << 24)) as f64)
                    / 10_000_000.0;
                // tc_println!("Longitude: {}", lon);

                let lat = ((payload[28] as i32
                    + ((payload[29] as i32) << 8)
                    + ((payload[30] as i32) << 16)
                    + ((payload[31] as i32) << 24)) as f64)
                    / 10_000_000.0;
                // tc_println!("Latitude: {}", lat);

                let height_ellipsoid = ((payload[32] as i32
                    + ((payload[33] as i32) << 8)
                    + ((payload[34] as i32) << 16)
                    + ((payload[35] as i32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Height (Ellipsoid): {}m", height_ellipsoid);

                let height_msl = ((payload[36] as i32
                    + ((payload[37] as i32) << 8)
                    + ((payload[38] as i32) << 16)
                    + ((payload[39] as i32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Height (MSL): {}m", height_msl);

                let h_acc = ((payload[40] as u32
                    + ((payload[41] as u32) << 8)
                    + ((payload[42] as u32) << 16)
                    + ((payload[43] as u32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Horizontal Accuracy Estimate: {}m", h_acc);

                let v_acc = ((payload[44] as u32
                    + ((payload[45] as u32) << 8)
                    + ((payload[46] as u32) << 16)
                    + ((payload[47] as u32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Vertical Accuracy Estimate: {}m", v_acc);

                let vel_n = ((payload[48] as i32
                    + ((payload[49] as i32) << 8)
                    + ((payload[50] as i32) << 16)
                    + ((payload[51] as i32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Velocity North: {}m/s", vel_n);

                let vel_e = ((payload[52] as i32
                    + ((payload[53] as i32) << 8)
                    + ((payload[54] as i32) << 16)
                    + ((payload[55] as i32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Velocity East: {}m/s", vel_e);

                let vel_d = ((payload[56] as i32
                    + ((payload[57] as i32) << 8)
                    + ((payload[58] as i32) << 16)
                    + ((payload[59] as i32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Velocity Down: {}m/s", vel_d);

                let ground_speed = ((payload[60] as i32
                    + ((payload[61] as i32) << 8)
                    + ((payload[62] as i32) << 16)
                    + ((payload[63] as i32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Ground Speed: {}m/s", ground_speed);

                let motion_heading = ((payload[64] as i32
                    + ((payload[65] as i32) << 8)
                    + ((payload[66] as i32) << 16)
                    + ((payload[67] as i32) << 24)) as f32)
                    / 100_000.0;
                // tc_println!("Motion Heading: {}", motion_heading);

                let speed_acc = ((payload[68] as u32
                    + ((payload[69] as u32) << 8)
                    + ((payload[70] as u32) << 16)
                    + ((payload[71] as u32) << 24)) as f32)
                    / 1_000.0;
                // tc_println!("Ground Speed Accuracy: {}", speed_acc);

                let heading_acc = ((payload[72] as u32
                    + ((payload[73] as u32) << 8)
                    + ((payload[74] as u32) << 16)
                    + ((payload[75] as u32) << 24)) as f32)
                    / 100_000.0;
                // tc_println!("Heading Accuracy: {}", heading_acc);

                let p_dop = ((payload[72] as u16 + ((payload[73] as u16) << 8)) as f32) / 100.0;
                // tc_println!("Position Dilution of Precision (DOP): {}", p_dop);

                let gps_payload = GPSPayload {
                    itow,
                    year,
                    month,
                    day,
                    hour,
                    minute: min,
                    second: sec,
                    nanoseconds: nano,
                    time_accuracy,
                    fix_type,
                    valid_fix: is_fix_valid,
                    valid_date: is_date_valid,
                    valid_time: is_time_valid,
                    sat_num: num_sats,
                    longitude: lon,
                    latitude: lat,
                    ellipsoid_height: height_ellipsoid,
                    msl_height: height_msl,
                    horizontal_accuracy_estimate: h_acc,
                    vertical_accuracy_estimate: v_acc,
                    north_vel: vel_n,
                    east_vel: vel_e,
                    down_vel: vel_d,
                    ground_speed,
                    ground_speed_accuracy: speed_acc,
                    motion_heading,
                    heading_accuracy: heading_acc,
                    dop: p_dop,
                };

                GPS_SIGNAL.signal(gps_payload);
            }
            _ => {
                tc_println!("Unhandled message id: {:2x?}", message_id);
            }
        },
        _ => {
            tc_println!("Unhandled message class: {:2x?}", message_class);
        }
    }
}
