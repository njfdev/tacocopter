/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use core::{cell::OnceCell, mem::transmute, str::FromStr};

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::flash::Flash;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{self, Channel},
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

struct Request<T, R> {
    input: T,
    responder: RawChannelPtr<R>,
}

struct RawChannelPtr<R>(*mut Channel<CriticalSectionRawMutex, R, 1>);
unsafe impl<R> Send for RawChannelPtr<R> {}

enum FlashRequest {
    Set(
        Request<
            (String<16>, Vec<u8, VALUE_BUFFER_SIZE>),
            Result<(), sequential_storage::Error<embassy_rp::flash::Error>>,
        >,
    ),
    Get(
        Request<
            String<16>,
            Result<
                Option<Vec<u8, VALUE_BUFFER_SIZE>>,
                sequential_storage::Error<embassy_rp::flash::Error>,
            >,
        >,
    ),
}

static FLASH_CHANNEL: Channel<CriticalSectionRawMutex, FlashRequest, 128> = Channel::new();

pub trait TcKeyValueStoreData: Serialize + DeserializeOwned + Default {
    fn key() -> String<16>;
}

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct SensorCalibrationData {
    pub accel_biases: (f32, f32, f32),
    pub gyro_biases: (f32, f32, f32),
}

impl TcKeyValueStoreData for SensorCalibrationData {
    fn key() -> String<16> {
        String::from_str("SENSOR_CALIB").unwrap()
    }
}

extern "C" {
    // Flash storage used for configuration
    static __config_start: u32;
}

pub struct TcStore;

// TODO: add better error handling so DB issues won't mess up a flight
impl TcStore {
    pub async fn init(spawner: &Spawner, flash: FlashType) {
        spawner.spawn(flash_handler(flash)).unwrap();
    }

    pub async fn set<T: TcKeyValueStoreData>(data: T) {
        let data_bytes: Vec<u8, VALUE_BUFFER_SIZE> = postcard::to_vec(&data).unwrap();

        let mut channel: Channel<
            CriticalSectionRawMutex,
            Result<(), sequential_storage::Error<embassy_rp::flash::Error>>,
            1,
        > = Channel::new();
        FLASH_CHANNEL
            .send(FlashRequest::Set(Request {
                input: (T::key(), data_bytes),
                responder: RawChannelPtr(&mut channel as *mut _),
            }))
            .await;
        let res = channel.receive().await;

        if res.is_err() {
            error!(
                "Error setting value for {}: {:?}",
                T::key(),
                res.unwrap_err()
            );
        }
    }

    pub async fn get<T: TcKeyValueStoreData>() -> T {
        let mut channel: Channel<
            CriticalSectionRawMutex,
            Result<
                Option<Vec<u8, VALUE_BUFFER_SIZE>>,
                sequential_storage::Error<embassy_rp::flash::Error>,
            >,
            1,
        > = Channel::new();
        FLASH_CHANNEL
            .send(FlashRequest::Get(Request {
                input: T::key(),
                responder: RawChannelPtr(&mut channel as *mut _),
            }))
            .await;
        let res = channel.receive().await;

        match res {
            Ok(val_res) => {
                if val_res.is_some() {
                    from_bytes::<T>(val_res.unwrap().as_slice()).unwrap()
                } else {
                    warn!("No value found for key {}, returning default...", &T::key());
                    Default::default()
                }
            }
            Err(e) => match e {
                _ => {
                    error!("Error occurred retrieving value: {:?}", e);
                    panic!()
                }
            },
        }
    }
}

#[embassy_executor::task]
async fn flash_handler(mut flash: FlashType) {
    let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
    let request_receiver = FLASH_CHANNEL.receiver();
    loop {
        let request = request_receiver.receive().await;

        match request {
            FlashRequest::Set(mut params) => {
                let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
                let res = store_item(
                    &mut flash,
                    (config_start)..(config_start + MAP_SIZE),
                    &mut NoCache::new(),
                    &mut buffer,
                    &params.input.0,
                    &params.input.1.as_slice(),
                )
                .await;
                unsafe {
                    (*params.responder.0).send(res).await;
                }
            }
            FlashRequest::Get(mut params) => {
                let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
                let res = fetch_item(
                    &mut flash,
                    (config_start)..(config_start + MAP_SIZE),
                    &mut NoCache::new(),
                    &mut buffer,
                    &params.input,
                )
                .await;

                if res.is_ok() {
                    if res.as_ref().unwrap().is_some() {
                        unsafe {
                            (*params.responder.0)
                                .send(Ok(Some(
                                    Vec::<u8, VALUE_BUFFER_SIZE>::from_slice(
                                        res.unwrap().unwrap_or_default(),
                                    )
                                    .unwrap(),
                                )))
                                .await;
                        }
                    } else {
                        unsafe {
                            (*params.responder.0).send(Ok(None)).await;
                        }
                    }
                } else {
                    unsafe {
                        (*params.responder.0).send(Err(res.unwrap_err())).await;
                    }
                }
            }
        }
    }
}
