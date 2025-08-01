use embassy_rp::{
    bind_interrupts, i2c,
    peripherals::{I2C1, PIN_14, PIN_15},
    Peri,
};
use embassy_time::Delay;
use mpu6050::Mpu6050;

use crate::{
    drivers::tc_store::{types::SensorCalibrationData, TcStore},
    global::{IMU_CALIB_SIGNAL, SHARED},
};

bind_interrupts!(struct I2C1Irqs {
  I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

pub async fn setup_imu(
    i2c1: Peri<'static, I2C1>,
    scl: Peri<'static, PIN_15>,
    sda: Peri<'static, PIN_14>,
) -> Mpu6050<i2c::I2c<'static, I2C1, i2c::Async>> {
    let i2c = i2c::I2c::new_async(i2c1, scl, sda, I2C1Irqs, i2c::Config::default());
    let mut mpu = Mpu6050::new(i2c);
    let mut delay = Delay;
    mpu.init(&mut delay).unwrap();
    mpu.set_accel_range(mpu6050::device::AccelRange::G8)
        .unwrap();
    mpu.set_gyro_range(mpu6050::device::GyroRange::D1000)
        .unwrap();
    // enable hardware 94/98hz lowpass filter
    mpu.write_bits(0x1A, 2, 3, 0x02).unwrap();
    // calibrate_accel(&mut mpu, 10.0).await;
    // calibrate_gyro(&mut mpu, 10.0).await;

    let sensor_data: tc_interface::SensorCalibrationData =
        TcStore::get::<SensorCalibrationData>().await.into();
    // SensorCalibrationData {
    //     accel_calibration: ACCEL_BIASES, //get_accel_offsets(&mut mpu, 10.0).await,
    //     gyro_calibration: GYRO_BIASES,
    // };
    {
        let mut shared = SHARED.lock().await;
        shared.calibration_data = sensor_data.clone();
    }
    IMU_CALIB_SIGNAL.signal(sensor_data);
    mpu
}
