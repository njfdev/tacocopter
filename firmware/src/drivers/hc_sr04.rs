use embassy_rp::{
    gpio::{AnyPin, Input, Level, Output, Pull},
    Peri,
};
use embassy_time::Instant;

use crate::tools::yielding_timer::YieldingTimer;

pub struct HcSr04 {
    trig: Output<'static>,
    echo: Input<'static>,
}

impl HcSr04 {
    pub fn new(
        trig_pin_peripheral: Peri<'static, AnyPin>,
        echo_pin_peripheral: Peri<'static, AnyPin>,
    ) -> Self {
        let trig_pin = Output::new(trig_pin_peripheral, Level::Low);
        let echo_pin = Input::new(echo_pin_peripheral, Pull::None);

        Self {
            trig: trig_pin,
            echo: echo_pin,
        }
    }

    // returns raw distance in m
    pub async fn get_dist(&mut self) -> Option<f32> {
        self.trig.set_low();
        YieldingTimer::after_micros(2).await;
        self.trig.set_high();
        YieldingTimer::after_micros(8).await;
        self.trig.set_low();

        self.echo.wait_for_rising_edge().await;
        let start = Instant::now();
        self.echo.wait_for_falling_edge().await;
        let time = start.elapsed().as_micros();

        if time / 1000 <= 45 {
            Some(((time as f32) * 0.000343) / 2.0)
        } else {
            None
        }
    }
}
