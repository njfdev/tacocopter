// Implementation and code to support LittleFS2

use core::alloc;
use core::cell::{OnceCell, RefCell, RefMut};

use embassy_rp::flash;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Instant;
// use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use heapless::pool::arc::{Arc, ArcBlock, ArcPool};
use heapless::{arc_pool, pool};
use littlefs2::consts::*;
use littlefs2::driver::Storage;
use log::{debug, error, info};

use crate::drivers::tc_store::SPACE_FOR_LOGS;
use crate::setup::flash::{FlashType, CONFIG_SIZE};

const LFS2_REL_START: u32 = (CONFIG_SIZE - SPACE_FOR_LOGS) as u32;
pub const LFS2_FS_OVERHEAD: u32 = 65_536;

arc_pool!(FLASH_ARC_POOL: RefCell<Mutex<NoopRawMutex, FlashType>>);
static mut ARC_BLOCK: ArcBlock<RefCell<Mutex<NoopRawMutex, FlashType>>> = ArcBlock::new();

struct ReadEraseRequest {
    id: usize,
    off: usize,
    len: usize,
}
// struct WriteRequest {
//     off: usize,
//     data: &[u8],
// }

// enum FlashBlockingRequest {
//     Read(ReadRequest),
//     Write(WriteRequest),
//     Erase(EraseRequest),
// }

pub struct Lfs2Storage {
    arc: Arc<FLASH_ARC_POOL>,
}

impl Lfs2Storage {
    pub fn new(flash: FlashType) -> Self {
        FLASH_ARC_POOL.manage(unsafe { &mut ARC_BLOCK });
        let arc = FLASH_ARC_POOL
            .alloc(RefCell::new(Mutex::new(flash)))
            .ok()
            .unwrap();

        Self { arc }
    }

    pub fn clone_with_shared_flash(&self) -> Self {
        Self {
            arc: self.arc.clone(),
        }
    }

    pub fn with_flash<T, F: FnOnce(&mut FlashType) -> T>(&mut self, f: F) -> T {
        let mut borrowed = self.arc.borrow_mut();
        let flash = borrowed.get_mut();
        f(flash)
    }

    pub async fn with_flash_async<T, F: AsyncFnOnce(&mut FlashType) -> T>(&mut self, f: F) -> T {
        let mut borrowed = self.arc.borrow_mut();
        let flash = borrowed.get_mut();
        f(flash).await
    }

    pub async fn with_flash_blocking_async<T, F: AsyncFnOnce(&mut FlashType) -> T>(
        &mut self,
        f: F,
    ) -> T {
        let mut borrowed = self.arc.borrow_mut();
        let flash = borrowed.get_mut();
        f(flash).await
    }
}

#[embassy_executor::task]
async fn flash_async_runner() {}

impl Storage for Lfs2Storage {
    type CACHE_SIZE = U256;
    type LOOKAHEAD_SIZE = U4;

    const READ_SIZE: usize = 4;
    const WRITE_SIZE: usize = 4;
    const BLOCK_SIZE: usize = flash::ERASE_SIZE;
    const BLOCK_COUNT: usize = SPACE_FOR_LOGS / flash::ERASE_SIZE;

    fn read(&mut self, off: usize, buf: &mut [u8]) -> littlefs2::io::Result<usize> {
        info!(
            "Read Start: {}, Offset: {}, Len: {}",
            LFS2_REL_START,
            off,
            buf.len()
        );
        let read_result: Result<(), flash::Error> =
            self.with_flash(|flash| flash.blocking_read(LFS2_REL_START + off as u32, buf));
        if read_result.is_err() {
            error!("Read invalid: {:?}", read_result.unwrap_err());
            return Err(littlefs2::io::Error::INVALID);
        }
        return Ok(buf.len());
    }

    fn write(&mut self, off: usize, data: &[u8]) -> littlefs2::io::Result<usize> {
        info!(
            "Write Start: {}, Offset: {}, Len: {}",
            LFS2_REL_START,
            off,
            data.len()
        );
        let write_result = self.with_flash(|flash| flash.write(LFS2_REL_START + off as u32, data));
        if write_result.is_err() {
            error!("Write invalid: {:?}", write_result.unwrap_err());
            return Err(littlefs2::io::Error::INVALID);
        }
        return Ok(data.len());
    }

    fn erase(&mut self, off: usize, len: usize) -> littlefs2::io::Result<usize> {
        info!(
            "Erase Start: {}, Offset: {}, Len: {}",
            LFS2_REL_START, off, len
        );
        let erase_result = self.with_flash(|flash| {
            flash.erase(
                LFS2_REL_START + off as u32,
                LFS2_REL_START + off as u32 + len as u32,
            )
        });
        if erase_result.is_err() {
            error!("Erase invalid: {:?}", erase_result.unwrap_err());
            return Err(littlefs2::io::Error::INVALID);
        }
        return Ok(len);
    }
}
