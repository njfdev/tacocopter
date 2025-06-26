use std::{
    sync::Mutex,
    thread::{self, sleep, sleep_ms},
    time::Duration,
};

use log::Level;
use postcard::from_bytes;
use rusb::{open_device_with_vid_pid, Context, DeviceHandle, GlobalContext};
use serde::{Deserialize, Serialize};
use serde_json::value::Serializer;
use tauri::{AppHandle, Emitter, Manager};
use tc_interface::{
    ConfiguratorMessage, LogData, StartGyroCalibrationData, TCMessage, TC_PID, TC_VID,
};

#[derive(Default)]
struct AppState {
    is_usb_loop_running: bool,
    start_gyro_calibration: bool,
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .setup(|app| {
            app.manage(Mutex::new(AppState::default()));
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            start_usb_loop,
            start_gyro_calibration
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
                    sampling_time: 10.0,
                }),
                &mut buf,
            )
            .unwrap();

            let n_result: Result<usize, String> = handle
                .write_bulk(out_endpoint, &mut buf, std::time::Duration::from_secs(999))
                .map_err(|e| e.to_string());

            println!("Start Gyro Calibration USB Result: {:#?}", n_result);
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
                sleep(Duration::from_millis(100));
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
