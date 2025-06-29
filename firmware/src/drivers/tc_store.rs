/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use core::{
    cell::{Cell, OnceCell},
    fmt::Debug,
    mem::transmute,
    str::FromStr,
};

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::flash::Flash;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{self, Channel},
    mutex::Mutex,
    signal::{self, Signal},
    watch::{Sender, Watch},
};
use embassy_time::Timer;
use heapless::{pool::arc::Arc, String};
use heapless_7::Vec;
use log::{error, info, warn};
use postcard::from_bytes;
use rand::Rng;
use sequential_storage::{
    cache::NoCache,
    erase_all,
    map::{fetch_item, store_item},
};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use static_cell::StaticCell;

use crate::{
    global::FLASH_MUTEX,
    setup::flash::{FlashType, CONFIG_SIZE},
};

// assume 75% of the database storage space will be for flight logging
static SPACE_FOR_LOGS: usize = CONFIG_SIZE * 3 / 4;
static VALUE_BUFFER_SIZE: usize = 1024;

static MAP_SIZE: u32 = 0x10000;

#[derive(Clone)]
struct FlashInterface<T> {
    id: usize,
    data: T,
}

struct RawChannelPtr<R>(*mut Channel<CriticalSectionRawMutex, R, 1>);
unsafe impl<R> Send for RawChannelPtr<R> {}

enum FlashRequest {
    Set(FlashInterface<(String<16>, Vec<u8, VALUE_BUFFER_SIZE>)>),
    Get(FlashInterface<String<16>>),
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
    pub async fn init(spawner: &Spawner, mut flash: FlashType) {
        let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
        erase_all(&mut flash, (config_start)..(config_start + MAP_SIZE))
            .await
            .unwrap();
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
                            info!("Saved");
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
}

#[embassy_executor::task]
async fn flash_handler(mut flash: FlashType) {
    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
    let sender = FLASH_RES_WATCH.sender();
    loop {
        let request = FLASH_REQ_CHANNEL.receive().await;

        match request {
            FlashRequest::Set(mut params) => {
                let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
                let res = store_item(
                    &mut flash,
                    (config_start)..(config_start + MAP_SIZE),
                    &mut NoCache::new(),
                    &mut buffer,
                    &params.data.0,
                    &params.data.1.as_slice(),
                )
                .await;
                sender.send(FlashResponse::Set(FlashInterface {
                    id: params.id,
                    data: res,
                }));
            }
            FlashRequest::Get(mut params) => {
                let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
                let res = fetch_item(
                    &mut flash,
                    (config_start)..(config_start + MAP_SIZE),
                    &mut NoCache::new(),
                    &mut buffer,
                    &params.data,
                )
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
        }
    }
}
