use crate::{
    drivers::tc_store::{__config_start, storage::FlashStorage},
    other_task_runner_setup,
    setup::flash::{FlashType, CONFIG_SIZE},
};
use crc::{Algorithm, Crc, CRC_64_ECMA_182};
use embassy_embedded_hal::flash;
use embassy_executor::Spawner;
use embassy_rp::flash::ERASE_SIZE;
use embassy_sync::once_lock::OnceLock;
use embassy_time::Timer;
use embedded_storage_async::nor_flash::NorFlash;
use heapless_7::Vec;
use log::{error, info};
use serde::{Deserialize, Serialize};
use static_assertions::const_assert;

const LOG_ENTRY_SIZE: usize = 128;
const_assert!(ERASE_SIZE % LOG_ENTRY_SIZE == 0);
// ensure room for crc (u64 so 8 bytes)
pub const LOG_DATA_SIZE: usize = LOG_ENTRY_SIZE as usize - 8;
pub const DOUBLE_LOG_DATA_SIZE: usize = LOG_DATA_SIZE * 2;

pub const BLACKBOX_CRC_ALGO: Algorithm<u64> = CRC_64_ECMA_182;

enum BlackboxRequest {
    StartLog(()),
    Log(BlackboxLogData),
    StopLog(()),
    CollectRawLogs(()),
}

#[derive(Clone)]
enum BlackboxResponse {
    StartLog(()),
    Log(()),
    StopLog(()),
    CollectRawLogs(Option<Vec<u8, LOG_DATA_SIZE>>),
}

other_task_runner_setup!(BLACKBOX, BlackboxRequest, BlackboxResponse);

#[derive(Clone, Serialize, Deserialize, Default)]
pub struct BlackboxLogData {
    pub timestamp_micros: u64,
    pub target_rate: [f32; 3],
    pub actual_rate: [f32; 3],
    pub p_term: [f32; 3],
    pub i_term: [f32; 3],
    pub d_term: [f32; 3],
    pub pid_output: [f32; 3],
    pub g_force: f32,
}

static BLACKBOX_SERIALIZED_DATA_SIZE: OnceLock<usize> = OnceLock::new();
impl BlackboxLogData {
    pub fn serialized_size() -> usize {
        BLACKBOX_SERIALIZED_DATA_SIZE
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

pub struct TcBlackbox {
    cur_entry: usize,
    total_entries: usize,
    collect_index: usize,
    flash_start: usize,
    flash_len: usize,
}

impl TcBlackbox {
    pub fn init(spawner: &Spawner, storage: &FlashStorage, flash_start: usize, flash_len: usize) {
        // let flash_len: u32 = 4096;
        spawner
            .spawn(blackbox_handler(
                Self {
                    cur_entry: 0,
                    total_entries: 0,
                    collect_index: 0,
                    flash_start: flash_start,
                    flash_len: flash_len - (flash_len % LOG_ENTRY_SIZE),
                },
                storage.clone_with_shared_flash(),
            ))
            .unwrap();
    }

    pub fn reset(&mut self) {
        self.cur_entry = 0;
        self.total_entries = 0;
        self.collect_index = 0;
    }

    pub async fn log_implementation(&mut self, storage: &FlashStorage, data: BlackboxLogData) {
        let entry_index = self.cur_entry * LOG_ENTRY_SIZE;

        let mut data_bytes: Vec<u8, LOG_ENTRY_SIZE> = postcard::to_vec(&data).unwrap();

        info!("Address: {}", (self.flash_start + entry_index) as u32);
        info!("Logged bytes: {:?}", data_bytes);

        for _ in data_bytes.len()..LOG_DATA_SIZE {
            data_bytes.push(0).unwrap();
        }

        let checksum = Crc::<u64>::new(&BLACKBOX_CRC_ALGO).checksum(&data_bytes);
        info!("Checksum: {}", checksum);
        data_bytes.extend(checksum.to_le_bytes());

        let res = storage
            .with_flash_async(async |flash| {
                if entry_index % ERASE_SIZE == 0 {
                    flash
                        .erase(
                            (self.flash_start + entry_index) as u32,
                            (self.flash_start + entry_index + ERASE_SIZE) as u32,
                        )
                        .await;
                }
                flash
                    .write((self.flash_start + entry_index) as u32, &data_bytes)
                    .await
            })
            .await;

        if res.is_err() {
            error!("Blackbox error: {:?}", res.unwrap_err());
        } else {
            self.cur_entry += 1;
            self.total_entries = (self.total_entries + 1).min(self.flash_len / LOG_ENTRY_SIZE);
            if self.cur_entry * LOG_ENTRY_SIZE > self.flash_len {
                self.cur_entry = 0;
            }
        }
    }

    pub async fn collect_logs_raw_implementation(
        &mut self,
        storage: &FlashStorage,
    ) -> Option<Vec<u8, LOG_DATA_SIZE>> {
        let log_res = storage
            .with_flash_async(async |flash| loop {
                if self.collect_index >= self.total_entries {
                    return None;
                }

                let address = self.flash_start + self.collect_index * LOG_ENTRY_SIZE;
                info!("Address: {}", address);
                self.collect_index += 1;

                let log_bytes = &mut [0u8; LOG_ENTRY_SIZE as usize];
                let res = flash.read(address as u32, log_bytes).await;
                if res.is_err() {
                    error!(
                        "Could not read at address {:#8x}: {:?}",
                        address,
                        res.unwrap_err()
                    );
                    return None;
                }
                info!("Read bytes: {:?}", log_bytes);

                let expected_checksum =
                    Crc::<u64>::new(&BLACKBOX_CRC_ALGO).checksum(&log_bytes[..LOG_DATA_SIZE]);
                let checksum = u64::from_le_bytes(
                    log_bytes[LOG_DATA_SIZE..LOG_ENTRY_SIZE as usize]
                        .try_into()
                        .unwrap(),
                );
                if checksum != expected_checksum {
                    error!(
                        "Checksum check failed, expected crc to be {}, but got {}.",
                        expected_checksum, checksum
                    );
                    return None;
                }

                let data: BlackboxLogData =
                    postcard::from_bytes(&log_bytes[0..LOG_DATA_SIZE]).unwrap();

                return Some(
                    Vec::from_slice(&log_bytes[0..BlackboxLogData::serialized_size()]).unwrap(),
                );
            })
            .await;

        if log_res.is_none() {
            self.collect_index = 0;
        }

        log_res
    }

    pub fn get_log_count(&self) -> u32 {
        self.total_entries as u32
    }

    pub async fn start_log() {
        let _res = blackbox_call_request!(StartLog, ());
    }

    pub async fn stop_log() {
        let _res = blackbox_call_request!(StopLog, ());
    }

    pub async fn log(log_data: BlackboxLogData) {
        // use send_request to not wait on the data logged (let it happen in the background)
        let res = blackbox_send_request!(Log, log_data);
    }

    pub async fn collect_logs_raw() -> Option<Vec<u8, LOG_DATA_SIZE>> {
        // use send_request to not wait on the data logged (let it happen in the background)
        blackbox_call_request!(CollectRawLogs, ())
    }
}

#[embassy_executor::task]
async fn blackbox_handler(mut tc_blackbox: TcBlackbox, storage: FlashStorage) {
    blackbox_request_handler!({
        BlackboxRequest::Log(data) => {
            tc_blackbox.log_implementation(&storage, data).await;

            BlackboxResponse::Log(())
        },
        BlackboxRequest::StartLog(data) => {
            tc_blackbox.reset();
            BlackboxResponse::StartLog(())
        },
        BlackboxRequest::StopLog(data) => {
            BlackboxResponse::StopLog(())
        },
        BlackboxRequest::CollectRawLogs(data) => {
            BlackboxResponse::CollectRawLogs(tc_blackbox.collect_logs_raw_implementation(&storage).await)
        },
    });
}
