#![no_std]
#![no_main]

pub mod consts;
pub mod drivers;
pub mod global;
pub mod setup;
pub mod tasks;
pub mod tc_log;
pub mod tools;

use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_sync::lazy_lock::LazyLock;

use crate::global::BOOT_TIME;
use crate::setup::peripherals::{setup_peripherals, SetupPeripherals};
use crate::setup::tasks::{spawn_tasks, TaskPeripherals};
use crate::setup::usb::setup_usb_interface;
use crate::tools::blinker::blink_led;

use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // this initializes BOOT_TIME with the current clock time immediately
    LazyLock::get(&BOOT_TIME);

    let p = embassy_rp::init(Default::default());

    let mut tc_devices = setup_peripherals(
        spawner.clone(),
        SetupPeripherals {
            status_led: p.PIN_25.into(),
            elrs_tx: p.PIN_0,
            elrs_rx: p.PIN_1,
            elrs_uart: p.UART0,
            gps_tx: p.PIN_8,
            gps_rx: p.PIN_9,
            gps_uart: p.UART1,
            i2c0_interface: p.I2C0,
            i2c0_scl: p.PIN_21,
            i2c0_sda: p.PIN_20,
            mpu_i2c1_interface: p.I2C1,
            mpu_scl: p.PIN_15,
            mpu_sda: p.PIN_14,
            dshot_pio: p.PIO0,
            dshot_mtr_1: p.PIN_2,
            dshot_mtr_2: p.PIN_3,
            dshot_mtr_3: p.PIN_4,
            dshot_mtr_4: p.PIN_5,
        },
    )
    .await;

    let (usb, bulk_in, bulk_out) = setup_usb_interface(p.USB);

    spawn_tasks(
        spawner.clone(),
        TaskPeripherals {
            core1: p.CORE1,
            bmp: tc_devices.bmp,
            usb,
            usb_bulk_in: bulk_in,
            usb_bulk_out: bulk_out,
            dshot: tc_devices.dshot,
            mpu: tc_devices.mpu,
            ultrasonic_trig: p.PIN_16,
            ultrasonic_echo: p.PIN_17,
            elrs: tc_devices.elrs,
            pm02d_interface: tc_devices.pm02d_interface,
        },
    );

    loop {
        // blinking the onboard led can let us determine 2 pieces of important information without a debugger probe
        // 1. If the LED isn't blinking, the FC crashed
        // 2. If the LED blinking speed is inconsistent, the FC is overloaded
        blink_led(&mut tc_devices.status_led, 3.0).await;

        // tc_println!("Voltage: {}V", voltage);
        // tc_println!("Current: {}A", current);
        // tc_println!(
        //     "Estimated State: {:.2}%, {:.2} mins remaining",
        //     percent,
        //     mins_remaining
        // );
        // tc_println!(
        //     "Used capacity: {:.1}mah",
        //     (pm02d_interface.get_used_capacity())
        // );
    }
}
