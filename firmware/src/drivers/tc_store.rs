/* This file is designed to be an abstraction over the ekv key-value
 * persistent storage for specific TacoCopter structs (e.g., by using
 * postcard and structs rather than raw bytes or byte strings).
 */

use ekv::ReadError;
use postcard::from_bytes;
use serde::{de::DeserializeOwned, Deserialize, Serialize};

use crate::{global::DATABASE, setup::flash::CONFIG_SIZE};

// assume 75% of the database storage space will be for flight logging
static SPACE_FOR_LOGS: usize = CONFIG_SIZE * 3 / 4;
static VALUE_BUFFER_SIZE: usize = 1024;

pub trait TcKeyValueStoreData: Serialize + DeserializeOwned + Default {
    fn key() -> &'static [u8];
}

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct SensorCalibrationData {
    pub accel_biases: (f32, f32, f32),
    pub gyro_biases: (f32, f32, f32),
}

impl TcKeyValueStoreData for SensorCalibrationData {
    fn key() -> &'static [u8] {
        b"SENSOR_CALIB"
    }
}

pub struct TcStore;

// TODO: add better error handling so DB issues won't mess up a flight
impl TcStore {
    pub async fn set<T: TcKeyValueStoreData>(data: T) {
        let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
        let data_bytes = postcard::to_slice(&data, &mut buffer).unwrap();

        let mut wtx = DATABASE.get().await.write_transaction().await;
        wtx.write(T::key(), data_bytes).await.unwrap();
        wtx.commit().await.unwrap();
    }

    pub async fn get<T: TcKeyValueStoreData>() -> T {
        let mut buffer = [0_u8; VALUE_BUFFER_SIZE];
        let rtx = DATABASE.get().await.read_transaction().await;
        let rtx_res = rtx.read(T::key(), &mut buffer).await;

        match rtx_res {
            Ok(n) => from_bytes::<T>(&buffer[..n]).unwrap(),
            Err(e) => match e {
                ReadError::KeyNotFound => Default::default(),
                _ => panic!(),
            },
        }
    }
}
