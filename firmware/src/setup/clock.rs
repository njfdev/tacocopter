use embassy_rp::clocks::{ClockConfig, CoreVoltage};

const BASE_FREQ: u32 = 150_000_000;

pub fn setup_clocks(overclock_freq: Option<u32>) -> ClockConfig {
    let freq = overclock_freq.unwrap_or(BASE_FREQ);
    let mut clock_config = ClockConfig::system_freq(freq).unwrap();
    if freq <= BASE_FREQ {
        clock_config.core_voltage = CoreVoltage::V1_10;
    } else if freq <= 200_000_000 {
        clock_config.core_voltage = CoreVoltage::V1_15;
    } else if freq <= 250_000_000 {
        clock_config.core_voltage = CoreVoltage::V1_20;
    } else if freq <= 300_000_000 {
        clock_config.core_voltage = CoreVoltage::V1_25;
    } else {
        clock_config.core_voltage = CoreVoltage::V1_30;
    }
    clock_config
}
