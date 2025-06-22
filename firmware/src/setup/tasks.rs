use bmp390::Bmp390;
use dshot_pio::dshot_embassy_rp::DshotPio;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    i2c::{self, Async, I2c},
    multicore::{spawn_core1, Stack},
    peripherals::{CORE1, I2C0, I2C1, PIN_16, PIN_17, PIO0, USB},
    usb::{Driver, Endpoint, In, Out},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_usb::UsbDevice;
use mpu6050::Mpu6050;
use static_cell::StaticCell;

use crate::{
    drivers::{elrs::Elrs, pm02d::PM02D},
    tasks::{
        bmp390_handler::bmp_loop,
        control_loop::control_loop,
        dshot_handler::dshot_handler,
        elrs_transmitter::elrs_transmitter,
        imu_loops::{mpu6050_fetcher_loop, mpu6050_processor_loop},
        position_hold_loop::position_hold_loop,
        ultrasonic_handler::calc_ultrasonic_height_agl,
        usb_updater::{usb_task, usb_updater},
    },
};

static mut CORE1_STACK: Stack<4096> = Stack::new();
// static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

pub struct TaskPeripherals {
    pub core1: CORE1,
    pub bmp: Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>,
    pub usb: UsbDevice<'static, Driver<'static, USB>>,
    pub usb_bulk_in: Endpoint<'static, USB, In>,
    pub usb_bulk_out: Endpoint<'static, USB, Out>,
    pub dshot: DshotPio<'static, 4, PIO0>,
    pub mpu: Mpu6050<i2c::I2c<'static, I2C1, i2c::Async>>,
    pub ultrasonic_trig: PIN_16,
    pub ultrasonic_echo: PIN_17,
    pub elrs: Elrs,
    pub pm02d_interface: Option<PM02D>,
}

pub fn spawn_tasks(spawner: Spawner, p: TaskPeripherals) {
    spawner.spawn(bmp_loop(p.bmp)).unwrap();
    spawner.spawn(usb_task(p.usb)).unwrap();

    spawner
        .spawn(usb_updater(p.usb_bulk_in, p.usb_bulk_out))
        .unwrap();
    let _ = spawner.spawn(dshot_handler(p.dshot));

    spawner.spawn(mpu6050_fetcher_loop(p.mpu)).unwrap();
    spawner
        .spawn(elrs_transmitter(p.elrs, p.pm02d_interface))
        .unwrap();

    // run dshot and ultrasonic sensor on other core
    spawn_core1(
        p.core1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor = EXECUTOR1.init(Executor::new());
            executor.run(|spawner| {
                let _ = spawner.spawn(calc_ultrasonic_height_agl(
                    p.ultrasonic_trig,
                    p.ultrasonic_echo,
                ));
                let _ = spawner.spawn(control_loop());
                let _ = spawner.spawn(position_hold_loop());
                spawner.spawn(mpu6050_processor_loop()).unwrap();
            })
        },
    );
}
