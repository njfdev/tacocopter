use core::{
    fmt::Error,
    str::FromStr,
    sync::atomic::{AtomicU16, Ordering},
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use format_no_std::show;
use heapless::String;
use log::{Level, SetLoggerError};
use tc_interface::{LogData, LOG_SEGMENT_SIZE};

use crate::global::LOG_CHANNEL;

pub struct TcUsbLogger;

static LOGGER: TcUsbLogger = TcUsbLogger;
static LOG_ID: AtomicU16 = AtomicU16::new(0);

impl TcUsbLogger {
    pub fn init() -> Result<(), SetLoggerError> {
        log::set_logger(&LOGGER).map(|()| log::set_max_level(log::LevelFilter::Info))
    }

    fn send_log(level: Level, text: &str) {
        let text_string = String::<1024>::from_str(text).unwrap();
        let mut start = 0;
        let len = text_string.len();
        let mut part_id = 0;
        let log_id = LOG_ID.fetch_add(1, Ordering::SeqCst);

        while start < len {
            let mut end = core::cmp::min(start + LOG_SEGMENT_SIZE, len);

            // Move end back until it points at a char boundary
            while !text_string.is_char_boundary(end) && end > start {
                end -= 1;
            }

            let slice = &text_string[start..end];

            // Push slice safely
            let part = String::<LOG_SEGMENT_SIZE>::from_str(slice);

            if part.is_ok() {
                let _ = LOG_CHANNEL.try_send(LogData {
                    log_id,
                    log_part_index: part_id,
                    log_level: level,
                    text: part.unwrap(),
                });
            }

            start = end;
            part_id += 1;
        }
    }
}

impl log::Log for TcUsbLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            let mut log_buf = [0_u8; 1024];
            let format_res = show(&mut log_buf, *record.args());
            match format_res {
                Ok(log_value) => {
                    TcUsbLogger::send_log(record.level(), log_value);
                }
                Err(e) => {
                    TcUsbLogger::send_log(
                        Level::Error,
                        "Failure to send log: likely large than 1024 character limit",
                    );
                }
            }
        }
    }

    fn flush(&self) {}
}
