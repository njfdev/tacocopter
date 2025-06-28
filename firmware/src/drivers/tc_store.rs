/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use core::str::FromStr;

use embassy_rp::flash::Flash;
use heapless::String;
use log::{error, info, warn};
use postcard::from_bytes;
use sequential_storage::{
    cache::NoCache,
    map::{fetch_item, store_item},
};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

use crate::{global::FLASH_MUTEX, setup::flash::CONFIG_SIZE};

// assume 75% of the database storage space will be for flight logging
static SPACE_FOR_LOGS: usize = CONFIG_SIZE * 3 / 4;
static VALUE_BUFFER_SIZE: usize = 1024;

static MAP_SIZE: u32 = 0x10000;

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
    pub async fn set<T: TcKeyValueStoreData>(data: T) {
        let mut postcard_buffer = [0_u8; VALUE_BUFFER_SIZE];
        let data_bytes = postcard::to_slice(&data, &mut postcard_buffer).unwrap();

        let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
        let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
        let res = store_item(
            &mut *FLASH_MUTEX.get().await.lock().await,
            (config_start)..(config_start + MAP_SIZE),
            &mut NoCache::new(),
            &mut buffer,
            &T::key(),
            &&*data_bytes,
        )
        .await;

        if res.is_err() {
            error!("Failed to set key-value data: {:?}", res.unwrap_err());
        }
    }

    pub async fn get<T: TcKeyValueStoreData>() -> T {
        let config_start = unsafe { &__config_start as *const u32 as u32 } - 0x10000000;
        let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
        let res: Result<Option<&[u8]>, sequential_storage::Error<embassy_rp::flash::Error>> =
            fetch_item(
                &mut *FLASH_MUTEX.get().await.lock().await,
                (config_start)..(config_start + MAP_SIZE),
                &mut NoCache::new(),
                &mut buffer,
                &T::key(),
            )
            .await;

        match res {
            Ok(val_res) => {
                if val_res.is_some() {
                    from_bytes::<T>(val_res.unwrap()).unwrap()
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
