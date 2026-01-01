use bmp390::Bmp390;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    gpio::{AnyPin, Output},
    i2c::{self, Async, I2c},
    multicore::{spawn_core1, Stack},
    peripherals::{CORE1, I2C0, I2C1, PIN_16, PIN_17, PIN_18, PIN_19, PIO0, PIO1, USB},
    usb::{Driver, Endpoint, In, Out},
    Peri,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_usb::{class::cdc_acm::CdcAcmClass, UsbDevice};
use log::info;
use mpu6050::Mpu6050;
use static_cell::StaticCell;

use crate::{
    drivers::{
        elrs::Elrs,
        esc::{blheli_passthrough::BlHeliPassthrough, dshot_pio::DshotPio, EscPins},
        pm02d::PM02D,
    },
    tasks::{
        bmp390_handler::bmp_loop,
        control_loop::control_loop,
        elrs_transmitter::elrs_transmitter,
        esc_handler::esc_handler,
        imu_loops::mpu6050_processor_loop,
        position_hold_loop::position_hold_loop,
        ultrasonic_handler::calc_ultrasonic_height_agl,
        usb_updater::{usb_task, usb_updater},
    },
};

static mut CORE1_STACK: Stack<65_536> = Stack::new();
// static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

pub struct TaskPeripherals {
    pub core1: Peri<'static, CORE1>,
    pub bmp: Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, Async>>>,
    pub usb: UsbDevice<'static, Driver<'static, USB>>,
    pub usb_bulk_in: Endpoint<'static, USB, In>,
    pub usb_bulk_out: Endpoint<'static, USB, Out>,
    pub esc_pins: EscPins<'static, PIO0>,
    pub dshot: DshotPio<'static, 4, PIO0>,
    pub blheli_passthrough: BlHeliPassthrough<'static, PIO0>,
    pub serial_class: CdcAcmClass<'static, Driver<'static, USB>>,
    pub mpu: Mpu6050<i2c::I2c<'static, I2C1, i2c::Async>>,
    pub ultrasonic_trig: Peri<'static, PIN_18>,
    pub ultrasonic_echo: Peri<'static, PIN_19>,
    pub pio: Peri<'static, PIO1>,
    pub elrs: Elrs,
    pub pm02d_interface: Option<PM02D>,
}

pub fn spawn_tasks(spawner: Spawner, p: TaskPeripherals) {
    /*
    NOTE: All interfacing tasks must run on core 0.

    Tasks interfacing hardware peripherals have instability when run
    on core 0 (which I've deduced through experimentation).

     */

    // This task actually handles the USB traffic, and is I/O and timing heavy (runs on core 0)
    spawner.spawn(usb_task(p.usb)).unwrap();

    spawner.spawn(mpu6050_processor_loop(p.mpu)).unwrap();

    spawner
        .spawn(elrs_transmitter(p.elrs, p.pm02d_interface))
        .unwrap();

    let _ = spawner.spawn(esc_handler(
        p.esc_pins,
        p.dshot,
        p.blheli_passthrough,
        p.serial_class,
    ));
    spawner.spawn(bmp_loop(p.bmp)).unwrap();
    spawner
        .spawn(calc_ultrasonic_height_agl(
            p.ultrasonic_trig,
            p.ultrasonic_echo,
            p.pio,
        ))
        .unwrap();

    // spawn stuff one core 1 before core 0 gets busy
    spawn_core1(
        p.core1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor = EXECUTOR1.init(Executor::new());
            executor.run(|spawner| {
                let _ = spawner.spawn(control_loop());
                // let _ = spawner.spawn(position_hold_loop());
                spawner
                    .spawn(usb_updater(p.usb_bulk_in, p.usb_bulk_out))
                    .unwrap();
            });
        },
    );
}
