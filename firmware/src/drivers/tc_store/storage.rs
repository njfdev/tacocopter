// Implementation and code to support LittleFS2

use crate::setup::flash::FlashType;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::arc_pool;
use heapless::pool::arc::{Arc, ArcBlock};

arc_pool!(FLASH_ARC_POOL: RefCell<Mutex<NoopRawMutex, FlashType>>);
static mut ARC_BLOCK: ArcBlock<RefCell<Mutex<NoopRawMutex, FlashType>>> = ArcBlock::new();

pub struct FlashStorage {
    arc: Arc<FLASH_ARC_POOL>,
}

impl FlashStorage {
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

    pub fn with_flash<T, F: FnOnce(&mut FlashType) -> T>(&self, f: F) -> T {
        let mut borrowed = self.arc.borrow_mut();
        let flash = borrowed.get_mut();
        f(flash)
    }

    pub async fn with_flash_async<T, F: AsyncFnOnce(&mut FlashType) -> T>(&self, f: F) -> T {
        let mut borrowed = self.arc.borrow_mut();
        let flash = borrowed.get_mut();
        f(flash).await
    }

    pub async fn with_flash_blocking_async<T, F: AsyncFnOnce(&mut FlashType) -> T>(
        &self,
        f: F,
    ) -> T {
        let mut borrowed = self.arc.borrow_mut();
        let flash = borrowed.get_mut();
        f(flash).await
    }
}
