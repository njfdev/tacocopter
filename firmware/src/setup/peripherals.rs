use crate::{
    drivers::{elrs::Elrs, esc::dshot_pio::DshotPio, m100_gps::init_gps, pm02d::PM02D},
    setup::{barometer::setup_barometer, i2c::setup_i2c_bus, imu::setup_imu},
};
use bmp390::Bmp390;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::clk_sys_freq,
    gpio::{AnyPin, Level, Output},
    i2c::{Async, I2c},
    peripherals::{
        I2C0, I2C1, PIN_0, PIN_1, PIN_14, PIN_15, PIN_2, PIN_20, PIN_21, PIN_3, PIN_4, PIN_5,
        PIN_8, PIN_9, PIO0, UART0, UART1,
    },
    uart::BufferedUartTx,
    Peri,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use micromath::F32Ext;
use mpu6050::Mpu6050;

bind_interrupts!(struct Pio0Irqs {
  PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
});

pub struct SetupPeripherals {
    pub status_led: Peri<'static, AnyPin>,
    pub elrs_tx: Peri<'static, PIN_0>,
    pub elrs_rx: Peri<'static, PIN_1>,
    pub elrs_uart: Peri<'static, UART0>,
    pub gps_tx: Peri<'static, PIN_8>,
    pub gps_rx: Peri<'static, PIN_9>,
    pub gps_uart: Peri<'static, UART1>,
    pub i2c0_interface: Peri<'static, I2C0>,
    pub i2c0_scl: Peri<'static, PIN_21>,
    pub i2c0_sda: Peri<'static, PIN_20>,
    pub mpu_i2c1_interface: Peri<'static, I2C1>,
    pub mpu_scl: Peri<'static, PIN_15>,
    pub mpu_sda: Peri<'static, PIN_14>,
    pub dshot_pio: Peri<'static, PIO0>,
    pub dshot_mtr_1: Peri<'static, PIN_2>,
    pub dshot_mtr_2: Peri<'static, PIN_3>,
    pub dshot_mtr_3: Peri<'static, PIN_4>,
    pub dshot_mtr_4: Peri<'static, PIN_5>,
}

pub struct TcDevices {
    pub mpu: Mpu6050<I2c<'static, I2C1, Async>>,
    pub bmp: Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>,
    pub gps: BufferedUartTx,
    pub elrs: Elrs,
    pub dshot: DshotPio<'static, 4, PIO0>,
    pub pm02d_interface: Option<PM02D>,
    pub status_led: Output<'static>,
}

pub async fn setup_peripherals(spawner: Spawner, p: SetupPeripherals) -> TcDevices {
    // turn on the onboard LED to make it clear the device is on
    let mut led = Output::new(p.status_led, Level::Low);
    led.set_high();

    let elrs_handle = Elrs::new(p.elrs_tx, p.elrs_rx, p.elrs_uart, spawner.clone());

    let gps = init_gps(p.gps_tx, p.gps_rx, p.gps_uart, spawner.clone()).await;

    let i2c0_bus = setup_i2c_bus(p.i2c0_interface, p.i2c0_scl, p.i2c0_sda);

    let bmp = setup_barometer(i2c0_bus).await;

    let mpu = setup_imu(p.mpu_i2c1_interface, p.mpu_scl, p.mpu_sda).await;

    let target_freq = 8 * 600_000;
    let dshot = DshotPio::<4, _>::new(
        p.dshot_pio,
        Pio0Irqs,
        p.dshot_mtr_1,
        p.dshot_mtr_2,
        p.dshot_mtr_3,
        p.dshot_mtr_4,
        target_freq, // freq of DShot600 x8 (for extra time to do operations)
    );

    let pm02d_interface = PM02D::new(I2cDevice::new(i2c0_bus)).await;

    TcDevices {
        mpu,
        bmp,
        gps,
        elrs: elrs_handle,
        dshot,
        pm02d_interface,
        status_led: led,
    }
}
