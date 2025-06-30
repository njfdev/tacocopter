// use embassy_rp::{
//     i2c::{self, Async, I2c},
//     peripherals::I2C1,
// };
// use embassy_time::{Instant, Timer};
// use heapless::Vec;
// use mpu6050::Mpu6050;
// use tc_interface::StartGyroCalibrationData;

// use crate::{global::SHARED, tc_println};

use embassy_time::Instant;
use heapless::Vec;
use log::info;
use tc_interface::StartGyroCalibrationData;

const MAX_CALIBRATION_STEPS: usize = 20000;

pub struct GyroCalibrator {
    data_points: Vec<(f32, f32, f32), MAX_CALIBRATION_STEPS>,
    start_time: Instant,
    is_running: bool,
}

impl GyroCalibrator {
    pub fn new() -> Self {
        Self {
            data_points: Vec::new(),
            start_time: Instant::now(),
            is_running: false,
        }
    }

    // returns the calibrated values when done, otherwise it returns the time remaining and samples collected
    pub fn process_data(
        &mut self,
        data: (f32, f32, f32),
        settings: StartGyroCalibrationData,
    ) -> Result<(f32, f32, f32), (f32, usize)> {
        if self.is_running == false {
            self.data_points.clear();
            self.is_running = true;
            self.start_time = Instant::now();
        }

        let elapsed = self.start_time.elapsed().as_micros() as f32 / 1_000_000.0;
        if elapsed > settings.sampling_time {
            let bias = calc_averages(&self.data_points);
            self.is_running = false;
            return Ok(bias);
        }

        self.data_points.push(data);

        return Err((settings.sampling_time - elapsed, self.data_points.len()));
    }
}

fn calc_averages(data: &Vec<(f32, f32, f32), MAX_CALIBRATION_STEPS>) -> (f32, f32, f32) {
    let mut avgs: (f32, f32, f32) = (0.0, 0.0, 0.0);
    for val in data.iter() {
        avgs.0 += val.0;
        avgs.1 += val.1;
        avgs.2 += val.2;
    }
    avgs.0 /= data.len() as f32;
    avgs.1 /= data.len() as f32;
    avgs.2 /= data.len() as f32;
    return avgs;
}

// async fn calibrate_accel(mpu: &mut Mpu6050<i2c::I2c<'static, I2C1, i2c::Async>>, duration: f32) {
//     let time_between = ((duration / (CALIBRATION_STEPS as f32)) * 1_000_000.0) as u64;

//     let mut data_points: [[f64; 3]; CALIBRATION_STEPS] = [[0.0; 3]; CALIBRATION_STEPS];

//     let mut start;
//     for i in 0..CALIBRATION_STEPS {
//         start = Instant::now();
//         data_points[i] = mpu.read_accel_data().await;
//         Timer::after_micros(time_between - (Instant::now() - start).as_micros()).await
//     }

//     let mut bias = calc_averages(data_points);
//     bias[2] -= 1.0; // 1 g of gravity should be discounted from bias

//     tc_println!("Accel biases are: {:?}", bias);
// }

// async fn get_gyro_offsets(mpu: &mut Mpu6050<I2c<'static, I2C1, Async>>) -> [f32; 3] {
//     let settings: StartGyroCalibrationData;
//     {
//         let shared = SHARED.lock().await;
//         settings = shared.gyro_calibration_state.options.clone();
//     }

//     let micros_between = ((1.0 / settings.sampling_rate) * 1_000_000.0) as u64;
//     let total_samples = ((settings.sampling_time * 1_000_000.0) / micros_between as f32) as usize;

//     let mut data_points = Vec::<[f32; 3], 0>::new();
//     data_points.resize_default(total_samples).unwrap();

//     let mut start;
//     for i in 0..total_samples {
//         start = Instant::now();
//         data_points[i] = (*mpu.get_gyro().unwrap().as_mut_slice())
//             .try_into()
//             .unwrap();
//         Timer::after_micros(micros_between - (Instant::now() - start).as_micros()).await
//     }

//     // let mut bias = calc_averages(data_points);

//     // bias
//     [0.0, 0.0, 0.0]
// }

// async fn get_accel_offsets(
//     mpu: &mut Mpu6050<I2c<'static, I2C1, Async>>,
//     duration: f32,
// ) -> [f32; 3] {
//     let time_between = ((duration / (CALIBRATION_STEPS as f32)) * 1_000_000.0) as u64;

//     let mut data_points: [[f32; 3]; CALIBRATION_STEPS] = [[0.0; 3]; CALIBRATION_STEPS];

//     let mut start;
//     for i in 0..CALIBRATION_STEPS {
//         start = Instant::now();
//         data_points[i] = (*mpu.get_acc().unwrap().as_mut_slice()).try_into().unwrap();
//         Timer::after_micros(time_between - (Instant::now() - start).as_micros()).await
//     }

//     let mut bias = calc_averages(data_points);

//     // adjust for gravity
//     bias[2] -= 1.0;

//     bias
// }
