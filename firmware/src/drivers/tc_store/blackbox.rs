use crate::{
    drivers::tc_store::{__config_start, storage::FlashStorage},
    other_task_runner_setup,
    setup::flash::{FlashType, CONFIG_SIZE},
};
use crc::{Algorithm, Crc, CRC_64_ECMA_182};
use embassy_embedded_hal::flash;
use embassy_executor::Spawner;
use embassy_rp::flash::ERASE_SIZE;
use embedded_storage_async::nor_flash::NorFlash;
use heapless_7::Vec;
use log::error;
use serde::{Deserialize, Serialize};
use static_assertions::const_assert;

const LOG_ENTRY_SIZE: u32 = 128;
const_assert!(ERASE_SIZE as u32 % LOG_ENTRY_SIZE == 0);
// ensure room for crc (u64 so 8 bytes)
const LOG_DATA_SIZE: usize = LOG_ENTRY_SIZE as usize - 8;

enum BlackboxRequest {
    StartLog(()),
    Log(BlackboxLogData),
    StopLog(()),
}

#[derive(Clone)]
enum BlackboxResponse {
    StartLog(()),
    Log(()),
    StopLog(()),
}

other_task_runner_setup!(BLACKBOX, BlackboxRequest, BlackboxResponse);

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

pub struct TcBlackbox {
    cur_entry: u32,
    flash_start: u32,
    flash_len: u32,
}

impl TcBlackbox {
    pub fn init(spawner: &Spawner, storage: &FlashStorage, flash_start: u32, flash_len: u32) {
        spawner
            .spawn(blackbox_handler(
                Self {
                    cur_entry: 0,
                    flash_start: flash_start,
                    flash_len: flash_len - (flash_len % LOG_ENTRY_SIZE),
                },
                storage.clone_with_shared_flash(),
            ))
            .unwrap();
    }

    pub fn reset(&mut self) {
        self.cur_entry = 0;
    }

    pub async fn log_implementation(&mut self, storage: &FlashStorage, data: BlackboxLogData) {
        let entry_index = if self.cur_entry == 0 {
            0
        } else {
            self.flash_len % (self.cur_entry * LOG_ENTRY_SIZE)
        };

        let mut data_bytes: Vec<u8, LOG_DATA_SIZE> = postcard::to_vec(&data).unwrap();

        data_bytes.extend([0; LOG_DATA_SIZE - size_of::<BlackboxLogData>()]);

        let checksum = Crc::<u64>::new(&CRC_64_ECMA_182).checksum(&data_bytes);

        data_bytes.extend(checksum.to_le_bytes());

        let res = storage
            .with_flash_async(async |flash| {
                if entry_index % ERASE_SIZE as u32 == 0 {
                    flash
                        .erase(
                            self.flash_start + entry_index,
                            self.flash_start + entry_index + ERASE_SIZE as u32,
                        )
                        .await;
                }

                flash
                    .write(self.flash_start + entry_index as u32, &data_bytes)
                    .await
            })
            .await;

        if res.is_err() {
            error!("Blackbox error: {:?}", res.unwrap_err());
        }
    }

    pub async fn start_log() {
        let _res = blackbox_send_request!(StartLog, ());
    }

    pub async fn stop_log() {
        let _res = blackbox_send_request!(StopLog, ());
    }

    pub async fn log(log_data: BlackboxLogData) {
        let res = blackbox_send_request!(Log, log_data);
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
    });
}
