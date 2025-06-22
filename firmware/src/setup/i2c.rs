use embassy_rp::{
    bind_interrupts,
    i2c::{self, Async, I2c},
    peripherals::{I2C0, PIN_20, PIN_21},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use static_cell::StaticCell;

bind_interrupts!(struct I2C0Irqs {
  I2C0_IRQ => i2c::InterruptHandler<I2C0>;
});

pub type I2c0BusType = &'static Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, Async>>;

pub fn setup_i2c_bus(i2c_interface: I2C0, scl: PIN_21, sda: PIN_20) -> I2c0BusType {
    static I2C0_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'_, I2C0, Async>>> =
        StaticCell::new();
    let i2c0: I2c<'static, I2C0, Async> =
        I2c::new_async(i2c_interface, scl, sda, I2C0Irqs, i2c::Config::default());
    let i2c0_bus = Mutex::new(i2c0);
    I2C0_BUS.init(i2c0_bus)
}
