/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use core::{cell::OnceCell, str::FromStr};

use async_oneshot::oneshot;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_rp::flash::Flash;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
    signal::{self, Signal}, watch::Watch,
};
use heapless::{pool::arc::Arc, String};
use heapless_7::Vec;
use log::{error, info, warn};
use postcard::from_bytes;
use sequential_storage::{
    cache::NoCache,
    map::{fetch_item, store_item},
};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

use crate::{
    global::FLASH_MUTEX,
    setup::flash::{FlashType, CONFIG_SIZE},
};

// assume 75% of the database storage space will be for flight logging
static SPACE_FOR_LOGS: usize = CONFIG_SIZE * 3 / 4;
static VALUE_BUFFER_SIZE: usize = 1024;

static MAP_SIZE: u32 = 0x10000;

struct Request<T, R: Clone> {
    input: T,
    responder: Watch<CriticalSectionRawMutex, R, 1>,
}

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

        let res_cell = OnceCell::new();
        FLASH_CHANNEL.send(FlashRequest::Set(Request {
            input: (T::key(), data_bytes),
            responder: res_cell.,
        }));
        let res = r.await.unwrap();

        if res.is_err() {
            error!(
                "Error setting value for {}: {:?}",
                T::key(),
                res.unwrap_err()
            );
        }
    }

    pub async fn get<T: TcKeyValueStoreData>() -> T {
        let (s, r) = oneshot();
        FLASH_CHANNEL.send(FlashRequest::Get(Request {
            input: T::key(),
            responder: s,
        }));
        let res = r.await.unwrap();

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
                params.responder.send(res);
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
                        params.responder.send(Ok(Some(
                            Vec::<u8, VALUE_BUFFER_SIZE>::from_slice(
                                res.unwrap().unwrap_or_default(),
                            )
                            .unwrap(),
                        )));
                    } else {
                        params.responder.send(Ok(None));
                    }
                } else {
                    params.responder.send(Err(res.unwrap_err()));
                }
            }
        }
    }
}
