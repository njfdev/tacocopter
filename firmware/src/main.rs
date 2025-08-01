#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

pub mod consts;
pub mod drivers;
pub mod global;
pub mod setup;
pub mod tasks;
pub mod tools;

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::block::ImageDef;
use embassy_rp::config::Config;
use embassy_rp::gpio::Level;
use embassy_sync::lazy_lock::LazyLock;
use embassy_time::Timer;
use heapless::String;
use log::warn;

use crate::drivers::tc_log::TcUsbLogger;
use crate::global::{BOOT_TIME, CONTROL_LOOP_FREQUENCY_SIGNAL, USB_ENABLED};
use crate::setup::clock::setup_clocks;
use crate::setup::flash::setup_flash_store;
use crate::setup::peripherals::{setup_peripherals, SetupPeripherals};
use crate::setup::tasks::{spawn_tasks, TaskPeripherals};
use crate::setup::usb::setup_usb_interface;
use crate::tools::blinker::blink_led;

use defmt_rtt as _; //, panic_probe as _};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[link_section = ".noinit.panic_msg"]
static mut PANIC_STRING: String<1024> = String::new();

#[link_section = ".noinit.panic_msg"]
static mut PANIC_FLAG: u32 = 0x0;

const DID_PANIC_FLAG_NUMBER: u32 = 0xDEADBEEF;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("Panic: {}", info);
    unsafe {
        let mut panic_string = String::new();
        // if this fails, continue anyways because we are already in a panicked state
        let _ = panic_string.push_str(
            info.message()
                .as_str()
                .unwrap_or("Unable to get panic message"),
        );
        PANIC_STRING = panic_string;
        PANIC_FLAG = DID_PANIC_FLAG_NUMBER;
    }
    // reboot
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let clock_config = setup_clocks(Some(300_000_000));
    let p = embassy_rp::init(Config::new(clock_config));

    // this initializes BOOT_TIME with the current clock time immediately
    LazyLock::get(&BOOT_TIME);

    let (usb, bulk_in, bulk_out) = setup_usb_interface(p.USB);

    // setup logging
    TcUsbLogger::init().unwrap();

    // setup the key-store db
    setup_flash_store(&spawner, p.FLASH, p.DMA_CH0).await;

    // TcStore::set(SensorCalibrationData {
    //     gyro_biases: (-0.0356924, -0.0230041, -0.03341522),
    //     accel_biases: (0.044174805, -0.063529054, 0.07425296),
    // })
    // .await;

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
            pio: p.PIO1,
            elrs: tc_devices.elrs,
            pm02d_interface: tc_devices.pm02d_interface,
        },
    );

    let mut imu_process_freq = 1.0;

    unsafe {
        if PANIC_FLAG == 0xDEADBEEF {
            PANIC_FLAG = 0x0;

            // if panicked, go into a mode where it just prints the message
            loop {
                Timer::after_secs(2).await;

                warn!("Panic occurred: \n\n{:?}", PANIC_STRING);
            }
        }
    }

    loop {
        // blinking the onboard led can let us determine 2 pieces of important information without a debugger probe
        // 1. If the LED isn't blinking, the FC crashed
        // 2. If the LED blinking speed is inconsistent, the FC is overloaded
        let new_imu_process_freq = CONTROL_LOOP_FREQUENCY_SIGNAL.try_take();
        if new_imu_process_freq.is_some() {
            // decrease the frequency by a factor of 50 to better see the result
            imu_process_freq = new_imu_process_freq.unwrap() / 50.0;
        }

        // if the frequency is too low, don't blink the LED (or it might take forever to blink if it was slow briefly)
        if imu_process_freq < 1.0 {
            yield_now().await;
        } else {
            blink_led(&mut tc_devices.status_led, imu_process_freq).await;
        }

        // TcStore::set(SensorCalibrationData {
        //     gyro_biases: (-0.0356924, -0.0230041, -0.03341522),
        //     accel_biases: (0.044174805, -0.063529054, 0.07425296),
        // })
        // .await;
        // info!("Data: {:?}", TcStore::get::<SensorCalibrationData>().await);

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
