/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use core::{fmt::Debug, str::FromStr};

use crate::{
    drivers::tc_store::{blackbox::TcBlackbox, storage::FlashStorage},
    other_task_runner_setup,
    setup::flash::{FlashType, CONFIG_SIZE},
};
use embassy_executor::Spawner;
use embassy_sync::once_lock::OnceLock;
use embassy_time::Timer;
use heapless::String;
use heapless_7::Vec;
use littlefs2::{
    fs::{Allocation, File, FileAllocation, Filesystem},
    io::SeekFrom,
    path,
    path::Path,
};
use log::{error, info, warn};
use postcard::from_bytes;
use sequential_storage::{
    cache::NoCache,
    map::{fetch_item, store_item},
    Error,
};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

pub mod blackbox;
mod storage;

const BLACKBOX_LOGGING_PATH: &Path = path!("blackbox.bin");

const VALUE_BUFFER_SIZE: usize = 256;

// assume remaining database storage space will be for flight logging
const KEY_STORE_SIZE: u32 = 0x10000;

enum FlashRequest {
    Set((String<16>, Vec<u8, VALUE_BUFFER_SIZE>)),
    Get(String<16>),
}

#[derive(Clone)]
enum FlashResponse {
    Set(Result<(), sequential_storage::Error<embassy_rp::flash::Error>>),
    Get(
        Result<
            Option<Vec<u8, VALUE_BUFFER_SIZE>>,
            sequential_storage::Error<embassy_rp::flash::Error>,
        >,
    ),
}

other_task_runner_setup!(FLASH, FlashRequest, FlashResponse, 32);

pub trait TcKeyValueStoreData: Serialize + DeserializeOwned + Default + Clone + Debug {
    fn key() -> String<16>;
}

#[derive(Serialize, Deserialize, Debug, Default, Clone)]
pub struct SensorCalibrationData {
    pub accel_biases: (f32, f32, f32),
    pub gyro_biases: (f32, f32, f32),
}

impl TcKeyValueStoreData for SensorCalibrationData {
    fn key() -> String<16> {
        String::from_str("SENSOR_CALIB").unwrap()
    }
}

impl Into<tc_interface::SensorCalibrationData> for SensorCalibrationData {
    fn into(self) -> tc_interface::SensorCalibrationData {
        tc_interface::SensorCalibrationData {
            accel_calibration: self.accel_biases.into(),
            gyro_calibration: self.gyro_biases.into(),
        }
    }
}

extern "C" {
    // Flash storage used for configuration
    static __config_start: u32;
}

pub struct TcStore;

// TODO: add better error handling so DB issues won't mess up a flight
impl TcStore {
    pub async fn init<'a>(spawner: &Spawner, flash: FlashType) {
        // let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
        // erase_all(&mut flash, (config_start)..(config_start + MAP_SIZE))
        //     .await
        //     .unwrap();

        // init flash handler
        spawner
            .spawn(flash_handler(flash, spawner.clone()))
            .unwrap();
    }

    pub async fn set<T: TcKeyValueStoreData>(data: T) {
        let data_bytes: Vec<u8, VALUE_BUFFER_SIZE> = postcard::to_vec(&data).unwrap();
        let res = flash_call_request!(Set, (T::key(), data_bytes));
        if res.is_err() {
            error!(
                "Error setting value for {}: {:?}",
                T::key(),
                res.unwrap_err()
            );
        } else {
            info!("Saved: {:?}", res);
        }
    }

    pub async fn get<T: TcKeyValueStoreData>() -> T {
        let res = flash_call_request!(Get, T::key());
        match res {
            Ok(val_res) => {
                if val_res.is_some() {
                    return from_bytes::<T>(val_res.unwrap().as_slice()).unwrap();
                } else {
                    warn!("No value found for key {}, returning default...", &T::key());
                    return Default::default();
                }
            }
            Err(e) => match e {
                _ => {
                    panic!("Error occurred retrieving value: {:?}", e);
                }
            },
        }
    }
}

// (read_index, write_index)
async fn get_log_indices(storage: &mut FlashStorage) -> (u32, u32) {
    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
    let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
    let res = storage
        .with_flash_async(async |flash| {
            fetch_item(
                flash,
                (config_start)..(config_start + KEY_STORE_SIZE),
                &mut NoCache::new(),
                &mut buffer,
                &String::<16>::from_str("LOG_INDICES").unwrap(),
            )
            .await
        })
        .await;
    let data: &[u8] = res.unwrap().unwrap_or_else(|| {
        let a: &[u8] = &[0; 8];
        a
    });
    (
        u32::from_le_bytes(data[..4].try_into().unwrap()),
        u32::from_le_bytes(data[4..8].try_into().unwrap()),
    )
}

async fn set_log_indices(storage: &mut FlashStorage, read_index: u32, write_index: u32) {
    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
    let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
    let read_bytes = read_index.to_le_bytes();
    let write_bytes = write_index.to_le_bytes();
    let res = storage
        .with_flash_async(async |flash| {
            store_item(
                flash,
                (config_start)..(config_start + KEY_STORE_SIZE),
                &mut NoCache::new(),
                &mut buffer,
                &String::<16>::from_str("LOG_INDICES").unwrap(),
                &[
                    read_bytes[0],
                    read_bytes[1],
                    read_bytes[2],
                    read_bytes[3],
                    write_bytes[0],
                    write_bytes[1],
                    write_bytes[2],
                    write_bytes[3],
                ],
            )
            .await
        })
        .await;
}

pub fn get_config_start() -> u32 {
    return unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
}

#[embassy_executor::task]
async fn flash_handler(mut flash: FlashType, spawner: Spawner) {
    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;

    Timer::after_secs(2).await;
    let mut storage = FlashStorage::new(flash);

    // init blackbox
    TcBlackbox::init(
        &spawner,
        &storage,
        (config_start + KEY_STORE_SIZE) as usize,
        CONFIG_SIZE - KEY_STORE_SIZE as usize,
    )
    .await;

    flash_request_handler!({
        FlashRequest::Set(data) => {
            let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
            let res_data = storage
                .with_flash_async(async |flash| {
                    store_item(
                        flash,
                        (config_start)..(config_start + KEY_STORE_SIZE),
                        &mut NoCache::new(),
                        &mut buffer,
                        &data.0,
                        &data.1.as_slice(),
                    )
                    .await
                })
                .await;
            FlashResponse::Set(res_data)
        },
        FlashRequest::Get(data) => {
            let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
            let res = storage
                .with_flash_async(async |flash| {
                    fetch_item(
                        flash,
                        (config_start)..(config_start + KEY_STORE_SIZE),
                        &mut NoCache::new(),
                        &mut buffer,
                        &data,
                    )
                    .await
                })
                .await;

            if res.is_ok() {
                if res.as_ref().unwrap().is_some() {
                    FlashResponse::Get(Ok(Some(
                            Vec::<u8, VALUE_BUFFER_SIZE>::from_slice(
                                res.unwrap().unwrap_or_default(),
                            )
                            .unwrap())))
                } else {
                    FlashResponse::Get(Ok(None))
                }
            } else {
                FlashResponse::Get(Err(res.unwrap_err()))
            }
        }
    });
}
