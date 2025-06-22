use bmp390::{Bmp390, OdrSel, Oversampling, PowerMode};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::{
    i2c::{Async, I2c},
    peripherals::I2C0,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Delay;

use crate::setup::i2c::I2c0BusType;

pub async fn setup_barometer(
    i2c0_bus: I2c0BusType,
) -> Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>> {
    let mut bmp390_conf = bmp390::Configuration::default();
    bmp390_conf.iir_filter.iir_filter = bmp390::IirFilter::coef_3;
    bmp390_conf.output_data_rate.odr_sel = OdrSel::ODR_25;
    bmp390_conf.oversampling.pressure = Oversampling::X16;
    bmp390_conf.power_control.enable_temperature = true;
    bmp390_conf.power_control.enable_pressure = true;
    bmp390_conf.power_control.mode = PowerMode::Normal;
    // let i2c = I2c::new_async(p.I2C0, p.PIN_13, p.PIN_12, I2C0Irqs, i2c::Config::default());
    Bmp390::try_new(
        I2cDevice::new(i2c0_bus),
        bmp390::Address::Up,
        Delay,
        &bmp390_conf,
    )
    .await
    .unwrap()
}
