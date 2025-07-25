use core::f32;
use embassy_futures::select::{select, Either};
use embassy_rp::{
    gpio::{AnyPin, Level, Output},
    peripherals::USB,
    usb::{Driver, Endpoint, In, Out},
};
use embassy_time::Instant;
use embassy_usb::driver::{EndpointIn, EndpointOut};
use embassy_usb::UsbDevice;
use heapless_7::Vec;
use postcard::from_bytes;
use tc_interface::{
    ConfiguratorMessage, ImuSensorData, SensorCalibrationData, SensorData, StateData, TCMessage,
    BLACKBOX_SEGMENT_SIZE,
};

use crate::{
    consts::USB_LOGGER_RATE,
    drivers::tc_store::blackbox::{BlackboxLogData, TcBlackbox, DOUBLE_LOG_DATA_SIZE},
    global::{
        CalibrationSensorType, BOOT_TIME, CALIBRATION_FEEDBACK_SIGNAL,
        CONTROL_LOOP_FREQUENCY_SIGNAL, IMU_FETCH_FREQUENCY_SIGNAL, IMU_PROCESSOR_FREQUENCY_SIGNAL,
        LOG_CHANNEL, POSITION_HOLD_LOOP_FREQUENCY_SIGNAL, SHARED, START_CALIBRATION_SIGNAL,
        ULTRASONIC_WATCH, USB_ENABLED,
    },
    tools::yielding_timer::YieldingTimer,
};

#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    // loop shouldn't be needed, but just in case to prevent usb from being dropped
    loop {
        usb.run_until_suspend().await;
        usb.disable().await;
    }
}

#[embassy_executor::task]
pub async fn usb_updater(
    mut usb_send: Endpoint<'static, USB, In>,
    mut usb_read: Endpoint<'static, USB, Out>,
) {
    let mut since_last = Instant::now();
    let mut cur_log_id: u8 = 0;
    let mut ultrasonic_receiver = ULTRASONIC_WATCH.receiver().unwrap();
    let mut usb_enabled_receiver = USB_ENABLED.receiver().unwrap();
    let mut is_usb_enabled = usb_enabled_receiver.try_get().unwrap_or_default();
    let mut since_last_toggle = Instant::now();
    loop {
        // wait to run until USB is plugged in
        loop {
            let enabled_res = usb_enabled_receiver.try_changed();
            if enabled_res.is_some() {
                is_usb_enabled = enabled_res.unwrap();
            }

            if is_usb_enabled {
                break;
            }

            YieldingTimer::after_millis(50).await;
        }

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
            let imu_process_freq = IMU_PROCESSOR_FREQUENCY_SIGNAL.try_take();
            if imu_process_freq.is_some() {
                shared.state_data.imu_process_rate = imu_process_freq.unwrap();
            }
            let control_loop_freq = CONTROL_LOOP_FREQUENCY_SIGNAL.try_take();
            if control_loop_freq.is_some() {
                shared.state_data.control_loop_update_rate = control_loop_freq.unwrap();
            }
            let position_hold_loop_freq = POSITION_HOLD_LOOP_FREQUENCY_SIGNAL.try_take();
            if position_hold_loop_freq.is_some() {
                shared.state_data.position_hold_loop_update_rate = position_hold_loop_freq.unwrap();
            }
            state_data = shared.state_data.clone();
            imu_sensor_data = shared.imu_sensor_data.clone();
            let ultrasonic_recv = ultrasonic_receiver.try_changed();
            if ultrasonic_recv.is_some() {
                shared.sensor_data.ultrasonic_dist = ultrasonic_recv.unwrap().unwrap_or(f32::NAN);
            }
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
        let calibration_status_res = CALIBRATION_FEEDBACK_SIGNAL.try_take();
        if calibration_status_res.is_some() {
            postcard::to_slice(
                &TCMessage::SensorCalibration(calibration_status_res.unwrap()),
                &mut buffer,
            )
            .unwrap();
            usb_send.write(&buffer).await.unwrap();
        } else {
            postcard::to_slice(
                &TCMessage::SensorCalibration(tc_interface::SensorCalibrationType::Data(
                    calibration_data,
                )),
                &mut buffer,
            )
            .unwrap();
            usb_send.write(&buffer).await.unwrap();
        }

        // send elrs channel data
        postcard::to_slice(&TCMessage::ElrsChannels(elrs_channels), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        while let Ok(log_part) = LOG_CHANNEL.try_receive() {
            postcard::to_slice(&TCMessage::Log(log_part), &mut buffer).unwrap();
            usb_send.write(&buffer).await.unwrap();
            cur_log_id = cur_log_id.wrapping_add(1);
        }

        // indicate end of data
        postcard::to_slice(&TCMessage::PacketIndicator(false), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        match select(
            usb_read.read(&mut buffer),
            YieldingTimer::after_micros(
                ((1_000_000.0 / USB_LOGGER_RATE) as u64)
                    .checked_sub(since_last.elapsed().as_micros())
                    .unwrap_or_default(),
            ),
        )
        .await
        {
            Either::First(Ok(n)) => {
                let msg = from_bytes::<ConfiguratorMessage>(&buffer[..n]).unwrap();

                match msg {
                    ConfiguratorMessage::StartGyroCalibration(data) => {
                        {
                            START_CALIBRATION_SIGNAL.signal(CalibrationSensorType::Gyro(data));
                        }
                        // loop {
                        //     Timer::after(time_between).await;

                        //     let calib_state;
                        //     {
                        //         let shared = SHARED.lock().await;
                        //         calib_state = shared.gyro_calibration_state.clone();
                        //     }

                        //     // indicate end of data
                        //     postcard::to_slice(
                        //         &TCMessage::GyroCalibrationProgress(calib_state.clone()),
                        //         &mut buffer,
                        //     )
                        //     .unwrap();
                        //     usb_send.write(&buffer).await.unwrap();

                        //     if calib_state.is_finished == true {
                        //         break;
                        //     }
                        // }
                    }
                    ConfiguratorMessage::StartBlackboxDownload => {
                        let mut data_buf = Vec::<u8, DOUBLE_LOG_DATA_SIZE>::new();
                        let mut total_log_count: usize = 0;
                        loop {
                            let log_res = TcBlackbox::collect_logs_raw().await;

                            if log_res.is_some() {
                                data_buf.extend(log_res.clone().unwrap());
                                total_log_count += 1;
                            }

                            while data_buf.len() >= BLACKBOX_SEGMENT_SIZE {
                                let section_to_send =
                                    heapless::Vec::<u8, BLACKBOX_SEGMENT_SIZE>::from_slice(
                                        &data_buf[0..BLACKBOX_SEGMENT_SIZE],
                                    )
                                    .unwrap();
                                data_buf = Vec::from_slice(
                                    &data_buf[BLACKBOX_SEGMENT_SIZE..data_buf.len()],
                                )
                                .unwrap();

                                postcard::to_slice(
                                    &TCMessage::BlackboxInfo(
                                        tc_interface::BlackboxInfoType::SerializedSegment(
                                            section_to_send,
                                        ),
                                    ),
                                    &mut buffer,
                                )
                                .unwrap();
                                usb_send.write(&buffer).await.unwrap();
                            }

                            if log_res.is_none() {
                                break;
                            }
                        }

                        postcard::to_slice(
                            &TCMessage::BlackboxInfo(
                                tc_interface::BlackboxInfoType::DownloadFinished(
                                    total_log_count as u32,
                                ),
                            ),
                            &mut buffer,
                        )
                        .unwrap();
                        usb_send.write(&buffer).await.unwrap();
                    }
                }
            }
            _ => {} // either failed or timeout, so continue normally
        }
        since_last = Instant::now();
    }
}
