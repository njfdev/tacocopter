use bmp390::Bmp390;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::{
    i2c::{Async, I2c},
    peripherals::I2C0,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Instant, Timer};
use uom::si::{length::meter, pressure::kilopascal, thermodynamic_temperature::kelvin};

use crate::{
    consts::BMP_UPDATE_FREQ,
    global::{BMP390_WATCH, CURRENT_ALTITUDE, TEMPERATURE},
    tools::{sorting::bubble_sort, yielding_timer::YieldingTimer},
};

#[embassy_executor::task]
pub async fn bmp_loop(
    mut bmp: Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>,
) {
    let barometer_sender = BMP390_WATCH.sender();
    let altitude_sender = CURRENT_ALTITUDE.sender();
    let base_altitude = get_base_altitude(&mut bmp).await;
    let mut since_last = Instant::now();
    loop {
        since_last = YieldingTimer::after_micros(
            ((1_000_000.0 / BMP_UPDATE_FREQ) as u64)
                .checked_sub(since_last.elapsed().as_micros())
                .unwrap_or_default(),
        )
        .await;
        let measurement = bmp.altitude().await.unwrap();
        let estimated_altitude = measurement.value - base_altitude;
        altitude_sender.send((Some(estimated_altitude), None));
        let pressure = bmp.pressure().await.unwrap();
        let temp = bmp.temperature().await.unwrap();
        barometer_sender.send((
            pressure.get::<kilopascal>(),
            temp.get::<kelvin>(),
            measurement.get::<meter>(),
        ));
        TEMPERATURE.signal(temp.value);
    }
}

async fn get_base_altitude(
    bmp: &mut Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>,
) -> f32 {
    const AMOUNT_OF_SAMPLES: usize = 200;
    let mut data_points: [f32; AMOUNT_OF_SAMPLES] = [0.0; AMOUNT_OF_SAMPLES];

    for i in 0..AMOUNT_OF_SAMPLES {
        data_points[i] = bmp.altitude().await.unwrap().value;
        Timer::after_millis(10).await
    }

    bubble_sort(&mut data_points);

    data_points[data_points.len() / 2]
}
