use embassy_rp::{
    bind_interrupts,
    i2c::{Async, Config, I2c, Instance, InterruptHandler, SclPin, SdaPin},
    peripherals::I2C1,
    Peripheral,
};
use log::error;
use log::info;

pub const ADDR: u8 = 0x68;

enum MPU6050Reg {
    GyroConf = 0x1B,
    AccelConf = 0x1C,
    AccelOut = 0x3B,
    GyroOut = 0x43,
    PwrMngt = 0x6B,
}

bind_interrupts!(struct I2CIrqs {
  I2C1_IRQ => InterruptHandler<I2C1>;
});

pub struct MPU6050 {
    i2c: I2c<'static, I2C1, Async>,
}

impl MPU6050 {
    pub fn new<Sda: SdaPin<I2C1>, Scl: SclPin<I2C1>>(
        sda: Sda,
        scl: Scl,
        i2c_interface: I2C1,
    ) -> Self {
        let i2c = I2c::new_async(i2c_interface, scl, sda, I2CIrqs, Config::default());

        Self { i2c }
    }

    pub async fn wake_up(&mut self) {
        self.i2c
            .write_async(ADDR, [MPU6050Reg::PwrMngt as u8, 0x11])
            .await
            .unwrap();
    }

    pub async fn set_gyro_range(&mut self, setting: u8) {
        self.i2c
            .write_async(ADDR, [MPU6050Reg::GyroConf as u8, (setting & 3) << 3])
            .await
            .unwrap();
    }

    pub async fn get_gryo_range(&mut self) -> u8 {
        let mut gyro_conf: [u8; 1] = [0];
        self.i2c
            .write_read_async(ADDR, [MPU6050Reg::GyroConf as u8], &mut gyro_conf)
            .await
            .unwrap();

        let fs_sel = 0x3 & (gyro_conf[0] >> 3);

        return fs_sel;
    }

    pub async fn read_gyro_data(&mut self) -> [f64; 3] {
        let gyro_modifier = match self.get_gryo_range().await {
            0 => 131.0,
            1 => 65.5,
            2 => 32.8,
            3 => 16.4,
            _ => 131.0, // assume option 0 if invalid data
        };

        let mut buffer: [u8; 6] = [0; 6];

        self.i2c
            .write_read_async(ADDR, [MPU6050Reg::GyroOut as u8], &mut buffer)
            .await
            .unwrap();
        let gyro_x = byte_pair_to_measurement(buffer[0], buffer[1], gyro_modifier);
        let gyro_y = byte_pair_to_measurement(buffer[2], buffer[3], gyro_modifier);
        let gyro_z = byte_pair_to_measurement(buffer[4], buffer[5], gyro_modifier);

        return [gyro_x, gyro_y, gyro_z];
    }

    pub async fn set_accel_range(&mut self, setting: u8) {
        self.i2c
            .write_async(ADDR, [MPU6050Reg::AccelConf as u8, (setting & 3) << 3])
            .await
            .unwrap();
    }

    pub async fn get_accel_range(&mut self) -> u8 {
        let mut accel_conf: [u8; 1] = [0];
        self.i2c
            .write_read_async(ADDR, [MPU6050Reg::AccelConf as u8], &mut accel_conf)
            .await
            .unwrap();

        let afs_sel = 0x3 & (accel_conf[0] >> 3);

        return afs_sel;
    }

    pub async fn read_accel_data(&mut self) -> [f64; 3] {
        let accel_modifier = match 0 {
            //self.get_accel_range().await {
            0 => 16384.0,
            1 => 8192.0,
            2 => 4096.0,
            3 => 2048.0,
            _ => 16384.0, // assume option 0 if invalid data
        };

        let mut buffer: [u8; 6] = [0; 6];

        self.i2c
            .write_read_async(ADDR, [MPU6050Reg::AccelOut as u8], &mut buffer)
            .await
            .unwrap();
        let accel_x = byte_pair_to_measurement(buffer[0], buffer[1], accel_modifier);
        let accel_y = byte_pair_to_measurement(buffer[2], buffer[3], accel_modifier);
        let accel_z = byte_pair_to_measurement(buffer[4], buffer[5], accel_modifier);

        return [accel_x, accel_y, accel_z];
    }
}

fn byte_pair_to_measurement(byte_h: u8, byte_l: u8, modifier: f64) -> f64 {
    let mut measurement = ((byte_h as i32) << 8) + byte_l as i32;
    if measurement >= 0x8000 {
        measurement = 0 - ((65535 - measurement) + 1);
    }
    (measurement as f64) / modifier
}
