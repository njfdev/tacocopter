use embassy_rp::gpio::Output;
use embassy_time::Timer;

pub async fn blink_led(led: &mut Output<'static>, freq: f32) {
    led.set_low();
    Timer::after_millis((500.0 / freq) as u64).await;
    led.set_high();
    Timer::after_millis((500.0 / freq) as u64).await;
}
