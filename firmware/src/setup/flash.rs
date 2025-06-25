use ekv::{
    config,
    flash::{self, PageID},
    Database,
};
use embassy_rp::{flash::Flash, peripherals::FLASH, Peri};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use log::info;

use crate::global::DATABASE;

const FLASH_SIZE: usize = 4 * 1024 * 1024; // 2 MiB
pub const CONFIG_SIZE: usize = 2 * 1024 * 1024; // 2 MiB

extern "C" {
    // Flash storage used for configuration
    static __config_start: u32;
}

// Workaround for alignment requirements.
#[repr(C, align(4))]
struct AlignedBuf<const N: usize>([u8; N]);

pub struct DbFlash<T: NorFlash + ReadNorFlash> {
    start: usize,
    flash: T,
}

impl<T: NorFlash + ReadNorFlash> flash::Flash for DbFlash<T> {
    type Error = T::Error;

    fn page_count(&self) -> usize {
        CONFIG_SIZE / config::PAGE_SIZE
    }

    async fn erase(&mut self, page_id: PageID) -> Result<(), <DbFlash<T> as flash::Flash>::Error> {
        self.flash.erase(
            (self.start + page_id.index() * config::PAGE_SIZE) as u32,
            (self.start + page_id.index() * config::PAGE_SIZE + config::PAGE_SIZE) as u32,
        )
    }

    async fn read(
        &mut self,
        page_id: PageID,
        offset: usize,
        data: &mut [u8],
    ) -> Result<(), <DbFlash<T> as flash::Flash>::Error> {
        let address = self.start + page_id.index() * config::PAGE_SIZE + offset;
        let mut buf = AlignedBuf([0; config::PAGE_SIZE]);
        self.flash.read(address as u32, &mut buf.0[..data.len()])?;
        data.copy_from_slice(&buf.0[..data.len()]);
        Ok(())
    }

    async fn write(
        &mut self,
        page_id: PageID,
        offset: usize,
        data: &[u8],
    ) -> Result<(), <DbFlash<T> as flash::Flash>::Error> {
        let address = self.start + page_id.index() * config::PAGE_SIZE + offset;
        let mut buf = AlignedBuf([0; config::PAGE_SIZE]);
        buf.0[..data.len()].copy_from_slice(data);
        self.flash.write(address as u32, &buf.0[..data.len()])
    }
}

pub type DbType = Database<
    DbFlash<Flash<'static, FLASH, embassy_rp::flash::Blocking, FLASH_SIZE>>,
    CriticalSectionRawMutex,
>;

pub async fn setup_flash_store(flash_peri: Peri<'static, FLASH>) {
    let flash = DbFlash {
        start: unsafe { &__config_start as *const u32 as usize } - 0x10000000,
        flash: Flash::new_blocking(flash_peri),
    };

    let db: DbType = Database::<_, CriticalSectionRawMutex>::new(flash, ekv::Config::default());

    if db.mount().await.is_err() {
        info!("Formatting flash database...");
        db.format().await.unwrap();
        info!("...Done!");
    }

    if DATABASE.init(db).is_err() {
        panic!();
    }
}
