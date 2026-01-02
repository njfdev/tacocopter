use core::f32;
use embassy_futures::select::{select, Either};
use embassy_rp::{
    gpio::{AnyPin, Level, Output},
    peripherals::USB,
    rom_data::reset_to_usb_boot,
    usb::{Driver, Endpoint, In, Out},
};
use embassy_time::Instant;
use embassy_usb::driver::{EndpointIn, EndpointOut};
use embassy_usb::UsbDevice;
use heapless_7::Vec;
use postcard::from_bytes;
use tc_interface::{
    ConfiguratorMessage, ImuSensorData, SensorData, StateData, TCMessage, BLACKBOX_SEGMENT_SIZE,
};

use crate::{
    consts::{UPDATE_LOOP_FREQUENCY, USB_LOGGER_RATE},
    drivers::tc_store::{
        blackbox::{TcBlackbox, DOUBLE_LOG_DATA_SIZE},
        types::{BlackboxSettings, PIDValues, SensorCalibrationData},
        TcStore,
    },
    global::{
        CalibrationSensorType, BLACKBOX_SETTINGS_WATCH, BOOT_TIME, CALIBRATION_FEEDBACK_SIGNAL,
        CURRENT_ALTITUDE, ELRS_WATCH, FC_PASSTHROUGH_SIGNAL, IMU_CALIB_SIGNAL, IMU_WATCH,
        LOG_CHANNEL, PID_WATCH, REALTIME_LOOP_FREQUENCY_SIGNAL, START_CALIBRATION_SIGNAL,
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
    let mut cur_pid_values = TcStore::get::<PIDValues>().await;
    let pid_sender = PID_WATCH.sender();
    let mut imu_receiver = IMU_WATCH.receiver().unwrap();
    let mut blackbox_settings = TcStore::get::<BlackboxSettings>().await;
    let blackbox_settings_sender = BLACKBOX_SETTINGS_WATCH.sender();
    let mut altitude_reciever = CURRENT_ALTITUDE.receiver().unwrap();
    let mut elrs_receiver = ELRS_WATCH.receiver().unwrap();

    let mut last_passthrough_enabled = false;
    let mut last_imu_process_rate = 0.0;
    let mut last_realtime_loop_rate = 0.0;
    let mut last_sensor_data = SensorData::default();
    let mut last_imu_sensor_data = ImuSensorData::default();
    let mut last_calibration_data = TcStore::get::<SensorCalibrationData>().await;
    let mut last_elrs_data: [u16; 16] = Default::default();

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

            // if usb disabled, then disable USB fc passthrough
            if last_passthrough_enabled {
                last_passthrough_enabled = false;
                FC_PASSTHROUGH_SIGNAL.signal(last_passthrough_enabled);
            }

            YieldingTimer::after_millis(50).await;
        }

        let mut buffer = [0u8; 64];

        let new_passthrough_setting = FC_PASSTHROUGH_SIGNAL.try_take();
        if new_passthrough_setting.is_some() {
            last_passthrough_enabled = new_passthrough_setting.unwrap();
        }

        let new_realtime_loop_freq = REALTIME_LOOP_FREQUENCY_SIGNAL.try_take();
        if new_realtime_loop_freq.is_some() {
            last_realtime_loop_rate = new_realtime_loop_freq.unwrap();
        }

        let new_calibration_data = IMU_CALIB_SIGNAL.try_take();
        if new_calibration_data.is_some() {
            let calibration_data = new_calibration_data.unwrap();
            last_calibration_data = SensorCalibrationData {
                accel_biases: calibration_data.accel_calibration.into(),
                gyro_biases: calibration_data.gyro_calibration.into(),
            };
        }

        let elrs_recv = elrs_receiver.try_changed();
        if elrs_recv.is_some() {
            last_elrs_data = elrs_recv.unwrap();
        }

        let ultrasonic_recv = ultrasonic_receiver.try_changed();
        if ultrasonic_recv.is_some() {
            last_sensor_data.ultrasonic_dist = ultrasonic_recv.unwrap().unwrap_or(f32::NAN);
        }
        let altitude_recv = altitude_reciever.try_changed();
        if altitude_recv.is_some() {
            let altitude_data = altitude_recv.unwrap();
            if altitude_data.0.is_some() {
                last_sensor_data.estimated_altitude = altitude_data.0.unwrap();
            }
        }

        let imu_res = imu_receiver.try_changed();
        if imu_res.is_some() {
            let imu_data = imu_res.unwrap();
            last_imu_sensor_data = ImuSensorData {
                gyroscope: imu_data.gyro_data.into(),
                accelerometer: imu_data.accel_data.into(),
            };
        }

        // indicate start of data
        postcard::to_slice(&TCMessage::PacketIndicator(true), &mut buffer).unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send state data
        postcard::to_slice(
            &TCMessage::State(StateData {
                target_update_rate: UPDATE_LOOP_FREQUENCY as f32,
                realtime_loop_update_rate: last_realtime_loop_rate,
                blheli_passthrough: last_passthrough_enabled,
                uptime: BOOT_TIME.get().elapsed().as_secs() as u32,
            }),
            &mut buffer,
        )
        .unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send PID settings
        postcard::to_slice(
            &TCMessage::PIDSettings(cur_pid_values.clone().into()),
            &mut buffer,
        )
        .unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send PID settings
        postcard::to_slice(
            &TCMessage::Blackbox(blackbox_settings.enabled.clone()),
            &mut buffer,
        )
        .unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send imu sensor data
        postcard::to_slice(
            &TCMessage::ImuSensor(last_imu_sensor_data.clone()),
            &mut buffer,
        )
        .unwrap();
        usb_send.write(&buffer).await.unwrap();

        // send sensor data
        postcard::to_slice(&TCMessage::Sensor(last_sensor_data.clone()), &mut buffer).unwrap();
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
                    last_calibration_data.clone().into(),
                )),
                &mut buffer,
            )
            .unwrap();
            usb_send.write(&buffer).await.unwrap();
        }

        // send elrs channel data
        postcard::to_slice(&TCMessage::ElrsChannels(last_elrs_data), &mut buffer).unwrap();
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
                    ConfiguratorMessage::SetPidSettings(new_settings) => {
                        let new_values: PIDValues = new_settings.into();
                        cur_pid_values = new_values.clone();
                        TcStore::set(new_values.clone()).await;
                        pid_sender.send(new_values);
                    }
                    ConfiguratorMessage::SetBlackboxEnabled(is_enabled) => {
                        blackbox_settings.enabled = is_enabled;
                        let new_settings = BlackboxSettings {
                            enabled: is_enabled,
                        };
                        TcStore::set(new_settings.clone()).await;
                        blackbox_settings_sender.send(new_settings);
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

                            while data_buf.len() >= BLACKBOX_SEGMENT_SIZE
                                || (log_res.is_none() && data_buf.len() > 0)
                            {
                                let section_size = data_buf.len().min(BLACKBOX_SEGMENT_SIZE);
                                let section_to_send =
                                    heapless::Vec::<u8, BLACKBOX_SEGMENT_SIZE>::from_slice(
                                        &data_buf[0..section_size],
                                    )
                                    .unwrap();
                                data_buf = Vec::from_slice(&data_buf[section_size..data_buf.len()])
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
                    ConfiguratorMessage::ToggleBlHeliPassthrough => {
                        last_passthrough_enabled = !last_passthrough_enabled;
                        FC_PASSTHROUGH_SIGNAL.signal(last_passthrough_enabled);
                    }
                    ConfiguratorMessage::ResetToUsbBoot => {
                        reset_to_usb_boot(25, 0);
                    }
                }
            }
            _ => {} // either failed or timeout, so continue normally
        }
        since_last = Instant::now();
    }
}
