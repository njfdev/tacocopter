use embassy_rp::uart::{BufferedUartRx, BufferedUartTx};
use embassy_time::Instant;
use embers::gps::ublox::{UBlox, SUGGESTED_UBLOX_BUFFER_SIZE};
use log::error;

use crate::{
    consts::UPDATE_LOOP_FREQUENCY, global::GPS_SIGNAL, tools::yielding_timer::YieldingTimer,
};

#[embassy_executor::task]
pub async fn gps_handler(
    mut ublox: UBlox<BufferedUartRx, BufferedUartTx, SUGGESTED_UBLOX_BUFFER_SIZE>,
) {
    let gps_sender = GPS_SIGNAL.sender();
    let mut since_last = Instant::now();
    loop {
        since_last = YieldingTimer::after_micros(
            ((1_000_000.0 / UPDATE_LOOP_FREQUENCY) as u64)
                .checked_sub(since_last.elapsed().as_micros())
                .unwrap_or_default(),
        )
        .await;

        let res = ublox.update().await;
        if res.is_err() {
            error!("UBlox GPS Processing Error: {:?}", res.unwrap_err());
        }

        let res = ublox.get_new();
        if res.is_some() {
            gps_sender.send(res.unwrap());
        }
    }
}
