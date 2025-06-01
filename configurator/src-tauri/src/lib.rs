use std::{
    sync::Mutex,
    thread::{self, sleep, sleep_ms},
    time::Duration,
};

use postcard::from_bytes;
use rusb::{open_device_with_vid_pid, Context, DeviceHandle, GlobalContext};
use serde::Serialize;
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
    let mut log_buffer = String::new();
    let out_endpoint = 0x01;
    let in_endpoint = 0x81;
    // Read from bulk IN endpoint (example: 0x81)
    loop {
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
                    sampling_rate: 100.0,
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
                    _ => {
                        app.emit("tc_data", message).unwrap();
                    }
                }
            }
        }
    }

    Ok(())
}
