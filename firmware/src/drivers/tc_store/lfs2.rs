// Implementation and code to support LittleFS2

use embassy_rp::flash;
use littlefs2::consts::*;
use littlefs2::driver::Storage;
use log::{debug, error, info};

use crate::drivers::tc_store::SPACE_FOR_LOGS;
use crate::setup::flash::{FlashType, CONFIG_SIZE};

const LFS2_REL_START: u32 = (CONFIG_SIZE - SPACE_FOR_LOGS) as u32;
pub const LFS2_FS_OVERHEAD: u32 = 65_536;

pub struct Lfs2Storage {
    flash: FlashType,
}

impl Lfs2Storage {
    pub fn new(flash: FlashType) -> Self {
        Self { flash }
    }

    pub fn flash(&mut self) -> &mut FlashType {
        &mut self.flash
    }
}

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
            self.flash.blocking_read(LFS2_REL_START + off as u32, buf);
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
        let write_result = self.flash.blocking_write(LFS2_REL_START + off as u32, data);
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
        let erase_result = self.flash.blocking_erase(
            LFS2_REL_START + off as u32,
            LFS2_REL_START + off as u32 + len as u32,
        );
        if erase_result.is_err() {
            error!("Erase invalid: {:?}", erase_result.unwrap_err());
            return Err(littlefs2::io::Error::INVALID);
        }
        return Ok(len);
    }
}
