use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::{PIN_8, PIN_9, UART1},
    uart::{self, BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx},
    Peripheral, Peripherals,
};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use log::{error, info, warn};
use static_cell::StaticCell;

use crate::SHARED;

bind_interrupts!(struct UartIrq {
  UART1_IRQ => BufferedInterruptHandler<UART1>;
});

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

        if current_len == 0 {
            continue;
        }

        // process packet
        for byte in current_packet[0..current_len] {
            continue;
        }
    }
}

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

                {
                    let mut shared = SHARED.lock().await;
                    shared.elrs_channels = chnls;
                }

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
