/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use core::{
    cell::{Cell, RefCell},
    fmt::Debug,
    ptr::addr_of_mut,
    str::FromStr,
};

use crate::{
    drivers::tc_store::lfs2::Lfs2Storage,
    setup::flash::{FlashType, CONFIG_SIZE},
};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, watch::Watch,
};
use embassy_time::{Instant, Timer};
use heapless::String;
use heapless_7::Vec;
use littlefs2::{
    fs::{Allocation, File, FileAllocation, Filesystem},
    io::SeekFrom,
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

const BLACKBOX_LOGGING_PATH: &str = "blackbox.bin";

// assume 75% of the database storage space will be for flight logging
const SPACE_FOR_LOGS: usize = CONFIG_SIZE * 3 / 4;
const VALUE_BUFFER_SIZE: usize = 1024;

const MAP_SIZE: u32 = 0x10000;

#[derive(Clone)]
struct FlashInterface<T> {
    id: usize,
    data: T,
}

#[derive(Clone)]
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

struct RawChannelPtr<R>(*mut Channel<CriticalSectionRawMutex, R, 1>);
unsafe impl<R> Send for RawChannelPtr<R> {}

enum FlashRequest {
    Set(FlashInterface<(String<16>, Vec<u8, VALUE_BUFFER_SIZE>)>),
    Get(FlashInterface<String<16>>),
    StartLog,
    Log(BlackboxLogData),
    StopLog,
}

#[derive(Clone)]
enum FlashResponse {
    Set(FlashInterface<Result<(), sequential_storage::Error<embassy_rp::flash::Error>>>),
    Get(
        FlashInterface<
            Result<
                Option<Vec<u8, VALUE_BUFFER_SIZE>>,
                sequential_storage::Error<embassy_rp::flash::Error>,
            >,
        >,
    ),
}

static REQ_ID: Mutex<CriticalSectionRawMutex, Cell<usize>> = Mutex::new(Cell::new(0));
static FLASH_REQ_CHANNEL: Channel<CriticalSectionRawMutex, FlashRequest, 128> = Channel::new();
static FLASH_RES_WATCH: Watch<CriticalSectionRawMutex, FlashResponse, 128> = Watch::new();

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

    async fn get_req_id() -> usize {
        let req_id_mutex = REQ_ID.lock().await;
        let req_id = req_id_mutex.get();
        req_id_mutex.set(req_id + 1);
        req_id
    }

    pub async fn set<T: TcKeyValueStoreData>(data: T) {
        let data_bytes: Vec<u8, VALUE_BUFFER_SIZE> = postcard::to_vec(&data).unwrap();

        let req_id = Self::get_req_id().await;
        FLASH_REQ_CHANNEL
            .send(FlashRequest::Set(FlashInterface {
                id: req_id,
                data: (T::key(), data_bytes),
            }))
            .await;
        let mut recv = FLASH_RES_WATCH.receiver().unwrap();
        let mut res = recv.changed().await;
        loop {
            match res.clone() {
                FlashResponse::Set(interface) => {
                    if interface.id == req_id {
                        if interface.data.is_err() {
                            error!(
                                "Error setting value for {}: {:?}",
                                T::key(),
                                interface.data.unwrap_err()
                            );
                        } else {
                            info!("Saved: {:?}", data);
                        }
                        break;
                    }
                }
                _ => {}
            }
            res = recv.changed().await;
        }
    }

    pub async fn get<T: TcKeyValueStoreData>() -> T {
        let req_id = Self::get_req_id().await;
        FLASH_REQ_CHANNEL
            .send(FlashRequest::Get(FlashInterface {
                id: req_id,
                data: T::key(),
            }))
            .await;
        let mut recv = FLASH_RES_WATCH.receiver().unwrap();
        let mut res = recv.changed().await;
        loop {
            match res.clone() {
                FlashResponse::Get(interface) => {
                    if interface.id == req_id {
                        match interface.data {
                            Ok(val_res) => {
                                if val_res.is_some() {
                                    return from_bytes::<T>(val_res.unwrap().as_slice()).unwrap();
                                } else {
                                    warn!(
                                        "No value found for key {}, returning default...",
                                        &T::key()
                                    );
                                }
                            }
                            Err(e) => match e {
                                _ => {
                                    error!("Error occurred retrieving value: {:?}", e);
                                }
                            },
                        }
                    }

                    return Default::default();
                }
                _ => {}
            }
            res = recv.changed().await;
        }
    }

    pub async fn start_log() {}
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

    let mut log_path = Path::from_str_with_nul(BLACKBOX_LOGGING_PATH).unwrap();
    let mut log_file: Option<File<Lfs2Storage>> = None;
    let mut write_index: u32 = 0;
    let mut read_index: u32 = 0;
    const HEADER_LENGTH: usize = 8;

    loop {
        let request = FLASH_REQ_CHANNEL.receive().await;

        match request {
            FlashRequest::Set(params) => {
                let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
                let res = storage
                    .with_flash_async(async |flash| {
                        store_item(
                            flash,
                            (config_start)..(config_start + MAP_SIZE),
                            &mut NoCache::new(),
                            &mut buffer,
                            &params.data.0,
                            &params.data.1.as_slice(),
                        )
                        .await
                    })
                    .await;
                sender.send(FlashResponse::Set(FlashInterface {
                    id: params.id,
                    data: res,
                }));
            }
            FlashRequest::Get(params) => {
                let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
                let res = storage
                    .with_flash_async(async |flash| {
                        fetch_item(
                            flash,
                            (config_start)..(config_start + MAP_SIZE),
                            &mut NoCache::new(),
                            &mut buffer,
                            &params.data,
                        )
                        .await
                    })
                    .await;

                if res.is_ok() {
                    if res.as_ref().unwrap().is_some() {
                        sender.send(FlashResponse::Get(FlashInterface {
                            id: params.id,
                            data: Ok(Some(
                                Vec::<u8, VALUE_BUFFER_SIZE>::from_slice(
                                    res.unwrap().unwrap_or_default(),
                                )
                                .unwrap(),
                            )),
                        }));
                    } else {
                        sender.send(FlashResponse::Get(FlashInterface {
                            id: params.id,
                            data: Ok(None),
                        }));
                    }
                } else {
                    sender.send(FlashResponse::Get(FlashInterface {
                        id: params.id,
                        data: Err(res.unwrap_err()),
                    }));
                }
            }
            FlashRequest::StartLog => {
                if log_file.is_some() {
                    unsafe {
                        log_file.unwrap().close();
                    };
                }
                let log_exists = fs.exists(log_path);
                if log_exists {
                    fs.remove(log_path).unwrap();
                }
                let log_handle = unsafe { File::create(&fs, &mut file_alloc, log_path) }.unwrap();
                log_handle.seek(SeekFrom::Start(0));
                log_handle.write(&read_index.to_le_bytes());
                log_handle.write(&write_index.to_le_bytes());
                log_file = Some(log_handle);
            }
            FlashRequest::StopLog => {
                if log_file.is_some() {
                    unsafe { log_file.unwrap().close() };
                    log_file = None;
                }
            }
            _ => {}
        }
    }
}
