use embassy_rp::{
    bind_interrupts,
    i2c::{Async, Config, I2c, Instance, InterruptHandler, SclPin, SdaPin},
    peripherals::I2C0,
    Peripheral,
};
use log::error;
use log::info;

use crate::tc_println;

const PM02D_ADDR: u8 = 0x45;
const MAX_CURRENT: f32 = 80.0;
const CURRENT_LSB: f32 = MAX_CURRENT / 524288.0_f32;
const R_SHUNT: f32 = 0.0005;
const SHUNT_CAL: u16 = (13107.2e5_f32 * CURRENT_LSB * R_SHUNT) as u16;
const ESTIMATE_VOLTAGE_DROP_PER_AMP: f32 = 0.0007;

enum PM02DReg {
    ShuntCalibration = 0x02,
    BusVoltage = 0x05,
    Current = 0x07,
}

bind_interrupts!(struct I2CIrqs {
  I2C0_IRQ => InterruptHandler<I2C0>;
});

pub struct PM02D {
    i2c: I2c<'static, I2C0, Async>,
}

const LIPO_VOLTAGE_CHARGE_LUT: [(f32, f32); 21] = [
    (4.20, 1.0),
    (4.15, 0.95),
    (4.11, 0.90),
    (4.08, 0.85),
    (4.02, 0.80),
    (3.98, 0.75),
    (3.95, 0.70),
    (3.91, 0.65),
    (3.87, 0.60),
    (3.85, 0.55),
    (3.84, 0.50),
    (3.82, 0.45),
    (3.80, 0.40),
    (3.79, 0.35),
    (3.77, 0.30),
    (3.75, 0.25),
    (3.73, 0.20),
    (3.71, 0.15),
    (3.69, 0.10),
    (3.61, 0.05),
    (3.27, 0.00),
];

impl PM02D {
    pub async fn new<Sda: SdaPin<I2C0>, Scl: SclPin<I2C0>>(
        sda: Sda,
        scl: Scl,
        i2c_interface: I2C0,
    ) -> Self {
        let i2c = I2c::new_async(i2c_interface, scl, sda, I2CIrqs, Config::default());

        let mut new_pm02d = Self { i2c };
        new_pm02d.set_shunt_cal().await;

        new_pm02d
    }

    async fn set_shunt_cal(&mut self) {
        let result = self
            .i2c
            .write_async(
                PM02D_ADDR,
                [
                    PM02DReg::ShuntCalibration as u8,
                    (SHUNT_CAL >> 8) as u8,
                    (SHUNT_CAL & 0xff) as u8,
                ],
            )
            .await;
    }

    pub async fn get_voltage(&mut self) -> f32 {
        let mut bus_voltage_bytes: [u8; 3] = [0, 0, 0];
        let result = self
            .i2c
            .write_read_async(
                PM02D_ADDR,
                [PM02DReg::BusVoltage as u8],
                &mut bus_voltage_bytes,
            )
            .await;

        if result.is_err() {
            return -1.0;
        }

        let bus_voltage = (((bus_voltage_bytes[0] as u32) << 12)
            + ((bus_voltage_bytes[1] as u32) << 4)
            + ((bus_voltage_bytes[2] as u32) >> 4)) as f32
            * 195.3125e-6;

        // tc_println!("Bus Voltage: {:?}", bus_voltage);

        return bus_voltage;
    }

    pub async fn get_current(&mut self) -> f32 {
        let mut current_bytes: [u8; 3] = [0, 0, 0];
        let result = self
            .i2c
            .write_read_async(PM02D_ADDR, [PM02DReg::Current as u8], &mut current_bytes)
            .await;

        if result.is_err() {
            return -1.0;
        }

        let raw_current = (((current_bytes[0] as u32) << 16)
            + ((current_bytes[1] as u32) << 8)
            + (current_bytes[2] as u32))
            >> 4;

        let current = (if raw_current & (1 << 19) != 0 {
            (raw_current | 0xfff00000) as i32
        } else {
            raw_current as i32
        }) as f32
            * CURRENT_LSB;

        return current;
    }

    // returns the battery charge as (percentage, estimated_time_remaining_in_minutes_based_on_current_power_draw)
    pub async fn estimate_battery_charge(
        &mut self,
        lipo_s_value: u8,
        battery_mah: u16,
    ) -> (f32, f32) {
        let mut voltage = self.get_voltage().await / (lipo_s_value as f32);
        let current = self.get_current().await;
        voltage = (voltage + (ESTIMATE_VOLTAGE_DROP_PER_AMP * current)).clamp(
            LIPO_VOLTAGE_CHARGE_LUT[LIPO_VOLTAGE_CHARGE_LUT.len() - 1].0,
            LIPO_VOLTAGE_CHARGE_LUT[0].0,
        );

        // find voltage range and interpolate percentage
        let mut percent: f32 = 0.0;
        for i in 0..(LIPO_VOLTAGE_CHARGE_LUT.len() - 2) {
            let (v1, percent1) = LIPO_VOLTAGE_CHARGE_LUT[i + 1];
            let (v2, percent2) = LIPO_VOLTAGE_CHARGE_LUT[i];
            if voltage >= v1 && voltage <= v2 {
                percent = percent1 + (percent2 - percent1) * ((voltage - v1) / (v2 - v1));
                break;
            }
        }

        let remaining_ah = ((battery_mah as f32) / 1000.0) * percent;
        let remaining_mins = (remaining_ah / current) * 60.0;

        return (percent * 100.0, remaining_mins);
    }
}
