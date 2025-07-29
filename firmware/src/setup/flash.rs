use crate::drivers::tc_store::TcStore;
use embassy_executor::Spawner;
use embassy_rp::{dma::Channel, flash::Flash, peripherals::FLASH, Peri};

const FLASH_SIZE: usize = 4 * 1024 * 1024; // 2 MiB
pub const CONFIG_SIZE: usize = (3.75 * 1024.0 * 1024.0) as usize; // 2 MiB

pub type FlashType = Flash<'static, FLASH, embassy_rp::flash::Async, FLASH_SIZE>;

pub async fn setup_flash_store(
    spawner: &Spawner,
    flash_peri: Peri<'static, FLASH>,
    dma: Peri<'static, impl Channel>,
) {
    TcStore::init(spawner, Flash::new(flash_peri, dma)).await;
}
