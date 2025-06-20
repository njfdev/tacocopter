use embassy_rp::{
    clocks::{self, ClockConfig, SysClkConfig, XoscConfig},
    config::Config,
    peripherals::WATCHDOG,
    watchdog::Watchdog,
};
use nalgebra::ComplexField;

pub fn setup_clock_speeds(overclock_frequency: u64) -> Config {
    let mut clocks = ClockConfig::system_freq(overclock_frequency);
    let conf = Config::new(clocks);
    conf
}
