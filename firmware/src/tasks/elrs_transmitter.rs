use embassy_time::Instant;

use crate::{
    consts::ELRS_TX_UPDATE_FREQ,
    drivers::{
        elrs::{
            elrs_tx_packets::{BarometerAltitudePacket, BatteryStatePacket, GPSPacket},
            Elrs, ElrsTxPacket,
        },
        pm02d::PM02D,
    },
    global::{CURRENT_ALTITUDE, GPS_SIGNAL},
    tools::yielding_timer::YieldingTimer,
};

#[embassy_executor::task]
pub async fn elrs_transmitter(mut elrs_handle: Elrs, mut pm02d_interface: Option<PM02D>) {
    let mut gps_receiver = GPS_SIGNAL.receiver().unwrap();
    let mut altitude_receiver = CURRENT_ALTITUDE.receiver().unwrap();

    let mut since_last = Instant::now();

    loop {
        since_last = YieldingTimer::after_micros(
            ((1_000_000.0 / ELRS_TX_UPDATE_FREQ) as u64)
                .checked_sub(since_last.elapsed().as_micros())
                .unwrap_or_default(),
        )
        .await;

        let altitude_recv = altitude_receiver.try_changed();
        if altitude_recv.is_some() {
            let altitude_packet = altitude_recv.unwrap();
            // This is technically not just a barometer reading, but this is a better estimation using more sensors
            elrs_handle
                .send_packet(ElrsTxPacket::BarometerAltitude(BarometerAltitudePacket {
                    altitude: altitude_packet.0.unwrap_or_default(),
                    vertical_speed: altitude_packet.1.unwrap_or_default(),
                }))
                .await;
        }

        if pm02d_interface.is_some() {
            let pm02d = pm02d_interface.as_mut().unwrap();
            let voltage = pm02d.get_voltage().await;
            let current = pm02d.get_current().await;
            let capacity = 5200;
            let (percent, _mins_remaining) = pm02d.estimate_battery_charge(4, capacity).await;
            elrs_handle
                .send_packet(ElrsTxPacket::BatteryState(BatteryStatePacket {
                    voltage,
                    current,
                    capacity: capacity as u32,
                    battery_percentage: percent,
                }))
                .await;
        }

        let gps_payload_res = gps_receiver.try_get();
        if gps_payload_res.is_some() {
            let gps_payload = gps_payload_res.unwrap();
            elrs_handle
                .send_packet(ElrsTxPacket::GPS(GPSPacket {
                    latitude: gps_payload.latitude,
                    longitude: gps_payload.longitude,
                    ground_speed: gps_payload.ground_speed,
                    gps_heading: gps_payload.motion_heading,
                    gps_altitude: gps_payload.msl_height,
                    num_sats: gps_payload.sat_num,
                }))
                .await;
        }
    }
}
