use core::str::FromStr;

use embassy_futures::select::{select, Either};
use embassy_rp::{
    peripherals::USB,
    usb::{Driver, Endpoint, In, Out},
};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::driver::{EndpointIn, EndpointOut};
use embassy_usb::UsbDevice;
use heapless::String;
use postcard::from_bytes;
use tc_interface::{
    ConfiguratorMessage, ImuSensorData, LogData, SensorCalibrationData, SensorData, StateData,
    TCMessage,
};

use crate::{
    consts::USB_LOGGER_RATE,
    global::{BOOT_TIME, IMU_FETCH_FREQUENCY_SIGNAL, LOG_CHANNEL, SHARED},
};

#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    // loop shouldn't be needed, but just in case to prevent usb from being dropped
    loop {
        usb.run().await;
    }
}

#[embassy_executor::task]
pub async fn usb_updater(
    mut usb_send: Endpoint<'static, USB, In>,
    mut usb_read: Endpoint<'static, USB, Out>,
) {
    let time_between = Duration::from_millis((1000.0 / USB_LOGGER_RATE) as u64);
    let mut cur_log_id: u8 = 0;
    loop {
        let mut buffer = [0u8; 64];
        let state_data: StateData;
        let imu_sensor_data: ImuSensorData;
        let sensor_data: SensorData;
        let calibration_data: SensorCalibrationData;
        let elrs_channels: [u16; 16];
        {
            let mut shared = SHARED.lock().await;
            shared.state_data.uptime = BOOT_TIME.get().elapsed().as_secs() as u32;
            let imu_fetch_freq = IMU_FETCH_FREQUENCY_SIGNAL.try_take();
            if imu_fetch_freq.is_some() {
                shared.state_data.imu_fetch_rate = imu_fetch_freq.unwrap();
            }
            state_data = shared.state_data.clone();
            imu_sensor_data = shared.imu_sensor_data.clone();
            sensor_data = shared.sensor_data.clone();
            calibration_data = shared.calibration_data.clone();
            elrs_channels = shared.elrs_channels.clone();
        }

        // indicate start of data
        postcard::to_slice(&TCMessage::PacketIndicator(true), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send state data
        postcard::to_slice(&TCMessage::State(state_data), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send imu sensor data
        postcard::to_slice(&TCMessage::ImuSensor(imu_sensor_data), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send sensor data
        postcard::to_slice(&TCMessage::Sensor(sensor_data), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send sensor calibration data
        postcard::to_slice(&TCMessage::SensorCalibration(calibration_data), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send elrs channel data
        postcard::to_slice(&TCMessage::ElrsChannels(elrs_channels), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // while log_buffer.len() > 0 {
        //     let amount_this_cycle = min(log_buffer.len(), 63);
        //     // send log channel data
        //     postcard::to_slice(
        //         &TCMessage::Log(LogData {
        //             id: cur_log_id,
        //             text: String::from_str(&log_buffer[0..amount_this_cycle]).unwrap(),
        //         }),
        //         &mut buffer,
        //     )
        //     .unwrap();
        //     usb_send.write(&buffer).await.unwrap();
        //     log_buffer = String::from_str(&log_buffer[amount_this_cycle..]).unwrap();
        //     cur_log_id = cur_log_id.wrapping_add(1);
        // }
        while let Ok(log_msg) = LOG_CHANNEL.try_receive() {
            postcard::to_slice(
                &TCMessage::Log(LogData {
                    id: cur_log_id,
                    text: String::from_str(&log_msg).unwrap(),
                }),
                &mut buffer,
            )
            .unwrap();
            usb_send.write(&buffer).await.unwrap();
            cur_log_id = cur_log_id.wrapping_add(1);
        }

        // indicate end of data
        postcard::to_slice(&TCMessage::PacketIndicator(false), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        match select(
            usb_read.read(&mut buffer),
            Timer::after(time_between - Instant::now().elapsed()),
        )
        .await
        {
            Either::First(Ok(n)) => {
                let msg = from_bytes::<ConfiguratorMessage>(&buffer[..n]).unwrap();

                match msg {
                    ConfiguratorMessage::StartGyroCalibration(data) => {
                        {
                            let mut shared = SHARED.lock().await;
                            shared.gyro_calibration_state.options = data;
                            shared.gyro_calibration_state.is_finished = false;
                        }
                        loop {
                            Timer::after(time_between).await;

                            let calib_state;
                            {
                                let shared = SHARED.lock().await;
                                calib_state = shared.gyro_calibration_state.clone();
                            }

                            // indicate end of data
                            postcard::to_slice(
                                &TCMessage::GyroCalibrationProgress(calib_state.clone()),
                                &mut buffer,
                            )
                            .unwrap();
                            usb_send.write(&buffer).await.unwrap();

                            if calib_state.is_finished == true {
                                break;
                            }
                        }
                    }
                }
            }
            _ => {} // either failed or timeout, so continue normally
        }
    }
}
