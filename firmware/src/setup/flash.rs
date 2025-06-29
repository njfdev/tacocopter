use embassy_executor::Spawner;
use embassy_rp::{dma::Channel, flash::Flash, peripherals::FLASH, Peri};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex},
    mutex::Mutex,
};
use heapless::String;
use log::info;
use sequential_storage::{cache::NoCache, map::fetch_item};

use crate::{drivers::tc_store::TcStore, global::FLASH_MUTEX};

const FLASH_SIZE: usize = 4 * 1024 * 1024; // 2 MiB
pub const CONFIG_SIZE: usize = 2 * 1024 * 1024; // 2 MiB

pub type FlashType = Flash<'static, FLASH, embassy_rp::flash::Async, FLASH_SIZE>;

pub async fn setup_flash_store(
    spawner: &Spawner,
    flash_peri: Peri<'static, FLASH>,
    dma: Peri<'static, impl Channel>,
) {
    TcStore::init(spawner, Flash::new(flash_peri, dma)).await;
}
