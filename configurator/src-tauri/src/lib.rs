use std::{
    path::Path,
    sync::Mutex,
    thread::{self, sleep_ms},
    time::Duration,
};

use csv::Writer;
use dirs::download_dir;
use log::{info, Level};
use postcard::{experimental::max_size::MaxSize, from_bytes};
use rusb::{open_device_with_vid_pid, Context, DeviceHandle, GlobalContext};
use serde::{Deserialize, Serialize};
use serde_json::value::Serializer;
use tauri::{async_runtime, AppHandle, Emitter, Manager};
use tc_interface::{
    BlackboxInfoType, BlackboxLogData, ConfiguratorMessage, LogData, PIDSettings,
    StartGyroCalibrationData, TCMessage, TC_PID, TC_VID,
};

#[derive(Default)]
struct AppState {
    is_usb_loop_running: bool,
    start_gyro_calibration: bool,
    toggle_blheli_passthrough: bool,
    reset_to_usb_boot: bool,
    start_blackbox_download: Option<(String, async_runtime::Sender<Result<usize, String>>)>,
    new_pid_settings: Option<PIDSettings>,
    blackbox_enabled: Option<bool>,
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_opener::init())
        .setup(|app| {
            app.manage(Mutex::new(AppState::default()));
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            start_usb_loop,
            start_gyro_calibration,
            start_blackbox_download,
            toggle_blheli_passthrough,
            reset_to_usb_boot,
            set_pid_settings,
            set_blackbox_enabled
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

fn get_handle() -> Option<DeviceHandle<GlobalContext>> {
    let handle = open_device_with_vid_pid(TC_VID, TC_PID);

    if handle.is_none() {
        return None;
    }

    let unwrapped_handle = handle.unwrap();

    // Claim the interface (adjust based on your USB descriptor)
    let result = unwrapped_handle
        .claim_interface(0)
        .map_err(|e| e.to_string());

    if result.is_err() {
        return None;
    }

    Some(unwrapped_handle)
}

#[tauri::command]
async fn start_gyro_calibration(app: AppHandle) -> Result<(), ()> {
    let state = app.state::<Mutex<AppState>>();
    let mut state = state.lock().unwrap();
    state.start_gyro_calibration = true;
    Ok(())
}

#[tauri::command]
async fn start_blackbox_download(app: AppHandle, path: String) -> Result<usize, String> {
    let (tx, mut rx) = async_runtime::channel::<Result<usize, String>>(1);
    {
        let state = app.state::<Mutex<AppState>>();
        let mut state = state.lock().unwrap();
        state.start_blackbox_download = Some((path, tx));
    };
    let result = rx.recv().await.unwrap();
    return result;
}

#[tauri::command]
async fn toggle_blheli_passthrough(app: AppHandle) -> Result<(), ()> {
    let state = app.state::<Mutex<AppState>>();
    let mut state = state.lock().unwrap();
    state.toggle_blheli_passthrough = true;
    Ok(())
}

#[tauri::command]
async fn reset_to_usb_boot(app: AppHandle) -> Result<(), ()> {
    let state = app.state::<Mutex<AppState>>();
    let mut state = state.lock().unwrap();
    state.reset_to_usb_boot = true;
    Ok(())
}

#[tauri::command]
async fn set_pid_settings(app: AppHandle, pid_settings: PIDSettings) -> Result<(), ()> {
    let state = app.state::<Mutex<AppState>>();
    let mut state = state.lock().unwrap();
    state.new_pid_settings = Some(pid_settings);
    Ok(())
}

#[tauri::command]
async fn set_blackbox_enabled(app: AppHandle, enabled: bool) -> Result<(), ()> {
    let state = app.state::<Mutex<AppState>>();
    let mut state = state.lock().unwrap();
    state.blackbox_enabled = Some(enabled);
    Ok(())
}

#[derive(Serialize, Deserialize, Clone, Debug)]
struct LogLine {
    id: usize,
    level: Level,
    text: String,
}

#[tauri::command]
async fn start_usb_loop(app: AppHandle) -> Result<(), ()> {
    {
        let state = app.state::<Mutex<AppState>>();
        let mut state = state.lock().unwrap();
        if state.is_usb_loop_running == true {
            return Err(());
        }
        state.is_usb_loop_running = true;
    }
    println!("Starting!");

    let mut handle;

    loop {
        let handle_result = get_handle();
        if handle_result.is_some() {
            handle = handle_result.unwrap();
            break;
        }
    }

    let mut message_flag = false;
    let out_endpoint = 0x01;
    let in_endpoint = 0x81;
    let mut log_buffer: Vec<LogData> = Vec::new();
    let mut blackbox_buffer: Vec<u8> = Vec::new();

    let mut blackbox_download_path = String::from("");
    let mut blackbox_download_response_tx: Option<async_runtime::Sender<Result<usize, String>>> =
        None;
    // Read from bulk IN endpoint (example: 0x81)
    loop {
        if message_flag == false && log_buffer.len() > 0 {
            let processed_logs = combine_logs(&log_buffer);
            println!("Log: {:#?}", processed_logs);

            app.emit("tc_log", processed_logs).unwrap();
            log_buffer.clear();
        }

        let mut buf = [0u8; 64];

        // if needing to send gyro message, then do it
        let should_start_gyro_calib: bool;
        {
            let state = app.state::<Mutex<AppState>>();
            let mut state = state.lock().unwrap();
            should_start_gyro_calib = state.start_gyro_calibration;
            state.start_gyro_calibration = false;
        }
        if should_start_gyro_calib {
            postcard::to_slice(
                &ConfiguratorMessage::StartGyroCalibration(StartGyroCalibrationData {
                    sampling_time: 3.0,
                }),
                &mut buf,
            )
            .unwrap();

            let n_result: Result<usize, String> = handle
                .write_bulk(out_endpoint, &mut buf, std::time::Duration::from_secs(999))
                .map_err(|e| e.to_string());

            println!("Start Gyro Calibration USB Result: {:#?}", n_result);
        }

        // if needing to start blackbox download, then do it
        let should_start_blackbox_download: bool;
        {
            let state = app.state::<Mutex<AppState>>();
            let mut state = state.lock().unwrap();
            if state.start_blackbox_download.is_some() {
                let blackbox_download_info = state.start_blackbox_download.take().unwrap();
                blackbox_download_path = blackbox_download_info.0;
                blackbox_download_response_tx = Some(blackbox_download_info.1);
                should_start_blackbox_download = true;
                state.start_blackbox_download = None;
            } else {
                should_start_blackbox_download = false;
            }
        }
        if should_start_blackbox_download {
            postcard::to_slice(&ConfiguratorMessage::StartBlackboxDownload, &mut buf).unwrap();

            let n_result: Result<usize, String> = handle
                .write_bulk(out_endpoint, &mut buf, std::time::Duration::from_secs(999))
                .map_err(|e| e.to_string());

            println!("Start Blackbox Download USB Result: {:#?}", n_result);
        }

        // if needing to set new pid settings, then do it
        let new_pid_settings: Option<PIDSettings>;
        {
            let state = app.state::<Mutex<AppState>>();
            let mut state = state.lock().unwrap();
            new_pid_settings = state.new_pid_settings.clone();
            state.new_pid_settings = None;
        }
        if new_pid_settings.is_some() {
            postcard::to_slice(
                &ConfiguratorMessage::SetPidSettings(new_pid_settings.unwrap()),
                &mut buf,
            )
            .unwrap();

            let n_result: Result<usize, String> = handle
                .write_bulk(out_endpoint, &mut buf, std::time::Duration::from_secs(999))
                .map_err(|e| e.to_string());

            println!(
                "Set request to send PID settings with result: {:#?}",
                n_result
            );
        }

        // if needing to set blackbox enabled setting, then do it
        let blackbox_enabled: Option<bool>;
        {
            let state = app.state::<Mutex<AppState>>();
            let mut state = state.lock().unwrap();
            blackbox_enabled = state.blackbox_enabled.clone();
            state.blackbox_enabled = None;
        }
        if blackbox_enabled.is_some() {
            postcard::to_slice(
                &ConfiguratorMessage::SetBlackboxEnabled(blackbox_enabled.unwrap()),
                &mut buf,
            )
            .unwrap();

            let n_result: Result<usize, String> = handle
                .write_bulk(out_endpoint, &mut buf, std::time::Duration::from_secs(999))
                .map_err(|e| e.to_string());

            println!(
                "Set request to set Blackbox settings with result: {:#?}",
                n_result
            );
        }

        // if needing to toggle fc passthrough, then do it
        let fc_passthrough_toggle: bool;
        {
            let state = app.state::<Mutex<AppState>>();
            let mut state = state.lock().unwrap();
            fc_passthrough_toggle = state.toggle_blheli_passthrough;
            state.toggle_blheli_passthrough = false;
        }
        if fc_passthrough_toggle {
            postcard::to_slice(&ConfiguratorMessage::ToggleBlHeliPassthrough, &mut buf).unwrap();

            let n_result: Result<usize, String> = handle
                .write_bulk(out_endpoint, &mut buf, std::time::Duration::from_secs(999))
                .map_err(|e| e.to_string());

            println!(
                "Set request to toggle BlHeli FC Passthrough with result: {:#?}",
                n_result
            );
        }

        // if needing to reset to usb boot, then do it
        let reset_to_usb_boot: bool;
        {
            let state = app.state::<Mutex<AppState>>();
            let mut state = state.lock().unwrap();
            reset_to_usb_boot = state.reset_to_usb_boot;
            state.reset_to_usb_boot = false;
        }
        if reset_to_usb_boot {
            postcard::to_slice(&ConfiguratorMessage::ResetToUsbBoot, &mut buf).unwrap();

            let n_result: Result<usize, String> = handle
                .write_bulk(out_endpoint, &mut buf, std::time::Duration::from_secs(999))
                .map_err(|e| e.to_string());

            println!(
                "Set request to reset to USB boot with result: {:#?}",
                n_result
            );
        }

        // println!("Waiting for data!");
        let n_result = handle
            .read_bulk(in_endpoint, &mut buf, std::time::Duration::from_secs(999))
            .map_err(|e| e.to_string());

        if n_result.is_err() {
            loop {
                // println!("Running search loop!");
                let handle_result = get_handle();
                if handle_result.is_some() {
                    println!("Found device!");
                    handle = handle_result.unwrap();
                    break;
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
            // println!("Returning from search loop!");
            continue;
        }

        let n = n_result.unwrap();

        if n > 0 {
            // println!("Data received!");
            let data = from_bytes::<TCMessage>(&buf[..n]);
            if data.is_ok() {
                // let value = data.unwrap().serialize(Serializer).unwrap();
                // let serialized = value.as_str().unwrap();
                let message = data.unwrap();
                // println!("Serialized: {:?}", &message);
                // if the message is a packet indicator, intercept and handle it
                match message {
                    TCMessage::PacketIndicator(value) => {
                        message_flag = value;
                    }
                    TCMessage::Log(data) => {
                        log_buffer.push(data);
                    }
                    TCMessage::BlackboxInfo(data) => {
                        println!("Blackbox Info: {:?}", data);
                        match data {
                            BlackboxInfoType::SerializedSegment(byte_segment) => {
                                blackbox_buffer.extend_from_slice(&byte_segment);
                            }
                            BlackboxInfoType::DownloadFinished(length) => {
                                let serialized_size = BlackboxLogData::POSTCARD_MAX_SIZE;
                                let blackbox_download_resonse_tx_unwrapped =
                                    blackbox_download_response_tx.unwrap();
                                if length as usize * serialized_size != blackbox_buffer.len() {
                                    println!(
                                        "Total: {}, Expected: {}",
                                        blackbox_buffer.len(),
                                        length as usize * serialized_size
                                    );
                                    blackbox_download_resonse_tx_unwrapped.send(Err(String::from("ERROR: Received blackbox data length did not match the expected number of bytes to be received!"))).await.unwrap();
                                } else {
                                    let mut blackbox_logs: Vec<BlackboxLogData> = vec![];

                                    for log_bytes in blackbox_buffer.chunks(serialized_size) {
                                        println!(
                                            "Log len: {}, Serialized Size: {}",
                                            log_bytes.len(),
                                            serialized_size
                                        );
                                        let blackbox_log =
                                            from_bytes::<BlackboxLogData>(log_bytes).unwrap();
                                        blackbox_logs.push(blackbox_log);
                                    }

                                    let mut wtr =
                                        Writer::from_path(Path::new(&blackbox_download_path))
                                            .unwrap();
                                    for log in blackbox_logs {
                                        wtr.serialize(log).unwrap();
                                    }
                                    wtr.flush().unwrap();
                                    blackbox_download_resonse_tx_unwrapped
                                        .send(Ok(length as usize))
                                        .await
                                        .unwrap();
                                }
                                blackbox_buffer.clear();
                                // wait for other tasks to realize the task is finished
                                while !blackbox_download_resonse_tx_unwrapped.is_closed() {
                                    tokio::time::sleep(Duration::from_millis(10)).await;
                                }
                                blackbox_download_response_tx = None;
                            }
                            _ => {}
                        }
                    }
                    _ => {
                        app.emit("tc_data", message).unwrap();
                    }
                }
            }
        }
    }

    Ok(())
}

fn combine_logs(log_parts: &Vec<LogData>) -> Vec<LogLine> {
    // sort the log parts so they are in order by id and the part number within that log
    let mut sorted_log_parts = log_parts.clone();
    sorted_log_parts.sort_by(|a, b| {
        ((a.log_id as u32) << 8 | a.log_part_index as u32)
            .cmp(&((b.log_id as u32) << 8 | b.log_part_index as u32))
    });

    // combine the parts of a log with an id so each id is 1 log line
    let mut sorted_logs = Vec::new();
    let mut cur_log_text = String::new();
    let mut cur_log_level = sorted_log_parts[0].log_level;
    let mut cur_log_id = sorted_log_parts[0].log_id;
    for i in 0..sorted_log_parts.len() {
        let cur_log_part = &sorted_log_parts[i];
        if cur_log_id != cur_log_part.log_id {
            sorted_logs.push(LogLine {
                id: cur_log_id.into(),
                level: cur_log_level,
                text: cur_log_text,
            });
            cur_log_text = String::new();
        }
        cur_log_id = cur_log_part.log_id;
        cur_log_level = cur_log_part.log_level;
        cur_log_text += cur_log_part.text.as_str();
    }

    // handle last log
    if cur_log_text.len() > 0 {
        sorted_logs.push(LogLine {
            id: cur_log_id.into(),
            level: cur_log_level,
            text: cur_log_text,
        });
    }

    sorted_logs
}
