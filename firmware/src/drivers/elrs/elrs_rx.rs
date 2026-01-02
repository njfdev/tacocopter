use embassy_rp::uart::BufferedUartRx;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Sender};
use embedded_io_async::Read;
use log::{debug, error, trace, warn};

use crate::global::ELRS_WATCH;

const BUF_LENGTH: usize = 1024;

#[embassy_executor::task]
pub async fn elrs_receive_handler(mut rx: BufferedUartRx) {
    let mut current_packet: [u8; BUF_LENGTH] = [0; BUF_LENGTH];
    let mut current_len = 0;
    let mut elrs_sender = ELRS_WATCH.sender();
    loop {
        let mut buf = [0; BUF_LENGTH];
        match rx.read(&mut buf).await {
            Ok(n) if n > 0 => {
                trace!("ELRS RX first {} bytes: {:2x?}", n, &buf[..n]);
                if n + current_len <= BUF_LENGTH {
                    defmt::info!("Buf Len: {}, Set End: {}", BUF_LENGTH, current_len + n);
                    current_packet[current_len..(current_len + n)].copy_from_slice(&buf[..n]);
                    current_len += n;
                } else {
                    warn!("ELRS UART Buffer is too full... Dropping {} bytes.", n);
                }
            }
            Ok(0) => {
                debug!("ELRS Read 0 bytes");
            }
            Err(e) => {
                debug!("ELRS Read error: {:?}", e);
            }
            _ => {}
        }
        trace!(
            "ELRS Packet ({}): {:2x?}",
            current_len,
            &current_packet[..current_len]
        );

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
            handle_packet(&current_packet[..len], len, &mut elrs_sender).await;

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

// fn print_link_statistics(data: &[u8; 10]) {
//     let up_rssi_ant1 = data[0] as i16 * -1;
//     let up_rssi_ant2 = data[1] as i16 * -1;
//     let up_link_quality = data[2];
//     let up_snr = data[3] as i8;
//     let active_ant = data[4];
//     let rf_profile = data[5];
//     let up_rf_power = data[6];

//     let down_rssi = data[7] as i16 * -1;
//     let down_link_quality = data[8];
//     let down_snr = data[9] as i8;

//     tc_println!(
//         "Uplink: Ant1- {} dBm, Ant2- {} dBm, Qlty-{}%, SNR-{} dB, Actv-{}, Prof-{}, Pwr-{}dB",
//         up_rssi_ant1,
//         up_rssi_ant2,
//         up_link_quality,
//         up_snr,
//         (active_ant + 1),
//         rf_profile,
//         up_rf_power
//     );

//     tc_println!(
//         "Downlink: RSSI- {} dBm, Qlty-{}%, SNR-{} dB",
//         down_rssi,
//         down_link_quality,
//         down_snr
//     );
// }

// TODO: Verify with CRC
async fn handle_packet<const N: usize>(
    data: &[u8],
    len: usize,
    elrs_sender: &mut Sender<'static, CriticalSectionRawMutex, [u16; 16], N>,
) {
    let frame_type = data[2];

    match frame_type {
        // RC Channels Packed Payload
        0x16 => {
            /* The data should be 26 bytes long because this packet
             * type packets 16 channels into 22 bytes. The check byte,
             * the length byte, the frame type byte, and crc byte,
             * add 4 to that.
             */
            if len != 26 {
                error!(
                    "RC Channels Packed Payload is {} bytes too long!",
                    (len - 26)
                );
            } else {
                let chnls = unpack_rc_bits(&data[3..25].try_into().unwrap());

                elrs_sender.send(chnls);
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
