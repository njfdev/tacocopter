/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use core::{cell::Cell, fmt::Debug, str::FromStr};

use crate::{
    drivers::tc_store::lfs2::{Lfs2Storage, LFS2_FS_OVERHEAD},
    other_task_runner_setup,
    setup::flash::{FlashType, CONFIG_SIZE},
};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex,
    once_lock::OnceLock, watch::Watch,
};
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
};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

mod lfs2;

const BLACKBOX_LOGGING_PATH: &Path = path!("blackbox.bin");

// assume 75% of the database storage space will be for flight logging
const SPACE_FOR_LOGS: usize = CONFIG_SIZE * 3 / 4;
const VALUE_BUFFER_SIZE: usize = 1024;

const MAP_SIZE: u32 = 0x10000;

#[derive(Clone, Serialize, Deserialize, Default)]
pub struct BlackboxLogData {
    timestamp_micros: u64,
    target_rate: [f32; 3],
    actual_rate: [f32; 3],
    p_term: [f32; 3],
    i_term: [f32; 3],
    d_term: [f32; 3],
    pid_output: [f32; 3],
    g_force: f32,
}

static LOG_DATA_SIZE: OnceLock<usize> = OnceLock::new();
impl BlackboxLogData {
    fn serialized_size() -> usize {
        LOG_DATA_SIZE
            .get_or_init(|| {
                postcard::serialize_with_flavor(
                    &Self::default(),
                    postcard::ser_flavors::Size::default(),
                )
                .unwrap()
            })
            .clone()
    }
}

enum FlashRequest {
    Set((String<16>, Vec<u8, VALUE_BUFFER_SIZE>)),
    Get(String<16>),
    StartLog(()),
    Log(BlackboxLogData),
    StopLog(()),
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
    StartLog(()),
    Log(()),
    StopLog(()),
}

other_task_runner_setup!(FLASH, FlashRequest, FlashResponse);

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
        spawner.spawn(flash_handler(flash)).unwrap();
    }

    pub async fn set<T: TcKeyValueStoreData>(data: T) {
        let data_bytes: Vec<u8, VALUE_BUFFER_SIZE> = postcard::to_vec(&data).unwrap();
        let res = flash_send_request!(Set, (T::key(), data_bytes));
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
        let res = flash_send_request!(Get, T::key());
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

    pub async fn start_log() {
        let _res = flash_send_request!(StartLog, ());
    }

    pub async fn stop_log() {
        let _res = flash_send_request!(StopLog, ());
    }

    pub async fn log(log_data: BlackboxLogData) {
        let res = flash_send_request!(Log, log_data);
    }
}

fn setup_lfs2<'a>(
    storage: &'a mut Lfs2Storage,
    alloc: &'a mut Allocation<Lfs2Storage>,
) -> Filesystem<'a, Lfs2Storage> {
    let mountable = Filesystem::is_mountable(storage);

    if !mountable {
        // Format the filesystem because not being mountable is likely due to the filesystem not being formatted
        let format_res = Filesystem::format(storage);
        if format_res.is_err() {
            format_res
                .inspect_err(|err| {
                    error!("Formatting error: {:?}", err.code());
                })
                .unwrap();
            panic!("Unable to format the filesystem");
        }
    }

    let fs = Filesystem::mount(alloc, storage);
    if fs.is_err() {
        fs.inspect_err(|err| {
            error!("Filesystem Error: {:?}", err.code());
        })
        .unwrap();
        panic!("Unable to mount filesystem");
    } else {
        return fs.unwrap();
    }
}

// (read_index, write_index)
async fn get_log_indices(storage: &mut Lfs2Storage) -> (u32, u32) {
    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
    let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
    let res = storage
        .with_flash_async(async |flash| {
            fetch_item(
                flash,
                (config_start)..(config_start + MAP_SIZE),
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

async fn set_log_indices(storage: &mut Lfs2Storage, read_index: u32, write_index: u32) {
    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
    let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
    let read_bytes = read_index.to_le_bytes();
    let write_bytes = write_index.to_le_bytes();
    let res = storage
        .with_flash_async(async |flash| {
            store_item(
                flash,
                (config_start)..(config_start + MAP_SIZE),
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

#[embassy_executor::task]
async fn flash_handler(mut flash: FlashType) {
    Timer::after_secs(2).await;
    let mut storage = Lfs2Storage::new(flash);

    // init littlefs2
    let mut alloc = Allocation::new();
    let mut lfs2_storage = storage.clone_with_shared_flash();
    let mut fs = setup_lfs2(&mut lfs2_storage, &mut alloc);
    info!("Filesystem mounted!");

    let mut file_alloc = FileAllocation::new();

    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
    let sender = FLASH_RES_WATCH.sender();

    let mut log_file: Option<File<Lfs2Storage>> = None;
    const HEADER_LENGTH: u32 = 0;
    let (mut read_index, mut write_index) = get_log_indices(&mut storage).await;

    flash_request_handler!({
        FlashRequest::Set(data) => {
            let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
            let res_data = storage
                .with_flash_async(async |flash| {
                    store_item(
                        flash,
                        (config_start)..(config_start + MAP_SIZE),
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
                        (config_start)..(config_start + MAP_SIZE),
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
        FlashRequest::Log(data) => {
            let mut buf = [0_u8; 256];
            let data_bytes = postcard::to_slice(&data, &mut buf).unwrap();

            if let Some(log_handle) = &log_file {
                log_handle.seek(SeekFrom::Start(write_index)).unwrap();
                log_handle.write(&data_bytes).unwrap();
                set_log_indices(&mut storage, read_index, write_index).await;
            }

            write_index += data_bytes.len() as u32;
            if write_index > SPACE_FOR_LOGS as u32 - LFS2_FS_OVERHEAD {
                write_index = HEADER_LENGTH;
            }
            if read_index < write_index {
                read_index = write_index;
            }

            FlashResponse::Log(())
        },
        FlashRequest::StartLog(data) => {
            if log_file.is_some() {
                unsafe {
                    log_file.unwrap().close();
                };
            }
            let log_exists = fs.exists(BLACKBOX_LOGGING_PATH);
            if log_exists {
                fs.remove(BLACKBOX_LOGGING_PATH).unwrap();
            }
            let log_handle =
                unsafe { File::create(&fs, &mut file_alloc, BLACKBOX_LOGGING_PATH) }.unwrap();
            log_handle.seek(SeekFrom::Start(0));
            read_index = HEADER_LENGTH;
            write_index = HEADER_LENGTH;
            log_handle.write(&read_index.to_le_bytes());
            log_handle.write(&write_index.to_le_bytes());
            log_file = Some(log_handle);

            FlashResponse::StartLog(())
        },
        FlashRequest::StopLog(data) => {
            if log_file.is_some() {
                unsafe { log_file.unwrap().close() };
                log_file = None;
            }
            FlashResponse::StopLog(())
        },
    });
}
