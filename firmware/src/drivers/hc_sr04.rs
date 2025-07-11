use embassy_rp::{
    bind_interrupts,
    clocks::clk_sys_freq,
    peripherals::PIO1,
    pio::{program::pio_asm, Config, InterruptHandler, Pio, PioPin, ShiftDirection, StateMachine},
    Peri,
};
use fixed::FixedU32;
use log::debug;

pub struct HcSr04 {
    // trig: Output<'static>,
    // echo: Input<'static>,
    sm: StateMachine<'static, PIO1, 1>,
}

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

impl HcSr04 {
    pub fn new(
        trig_pin_peripheral: Peri<'static, impl PioPin>,
        echo_pin_peripheral: Peri<'static, impl PioPin>,
        pio: Peri<'static, PIO1>,
    ) -> Self {
        let Pio {
            mut common,
            mut sm1,
            ..
        } = Pio::new(pio, Irqs);

        // let trig_pin = Output::new(trig_pin_peripheral, Level::Low);
        // let echo_pin = Input::new(echo_pin_peripheral, Pull::None);
        let trig_pin = common.make_pio_pin(trig_pin_peripheral);
        let echo_pin = common.make_pio_pin(echo_pin_peripheral);

        sm1.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&trig_pin]);
        sm1.set_pin_dirs(embassy_rp::pio::Direction::In, &[&echo_pin]);

        // assuming a frequency where each command takes 0.5 microseconds
        // the value in x is the time x 0.5 us
        let prog = pio_asm!(
            "   set pins, 0"
            "send_pulse:"
            "   set pins, 1 [19]"
            "   set pins, 0"
            "receive_echo:"
            "   wait 1 pin 0"
            "   set x, 2 [1]"
            "   mov x, ~x"
            "high_echo:"
            "   jmp X--, after_jmp"
            "after_jmp:"
            "   jmp pin, high_echo"
            "handle_echo:"
            "   mov x, ~x"
            "   in x, 32"
            "   push"
            "   jmp send_pulse"
        );

        let mut conf = Config::default();
        conf.use_program(&common.load_program(&prog.program), &[]);
        conf.clock_divider = FixedU32::from_num(clk_sys_freq() as f32 / 2.0e6);
        conf.set_out_pins(&[&trig_pin]);
        conf.set_in_pins(&[&echo_pin]);
        conf.set_jmp_pin(&echo_pin);
        conf.set_set_pins(&[&trig_pin]);
        conf.shift_in.auto_fill = false;
        conf.shift_in.direction = ShiftDirection::Left;

        sm1.set_config(&conf);
        sm1.set_enable(true);

        debug!("Start PIO and State Machine for the ultrasonic sensor.");

        Self { sm: sm1 }
    }

    // returns raw distance in m
    pub async fn get_dist(&mut self) -> Option<f32> {
        // self.trig.set_low();
        // YieldingTimer::after_micros(2).await;
        // self.trig.set_high();
        // YieldingTimer::after_micros(8).await;
        // self.trig.set_low();

        // self.echo.wait_for_rising_edge().await;
        // let start = Instant::now();
        // self.echo.wait_for_falling_edge().await;
        // let time = start.elapsed().as_micros();
        let pulse_count = self.sm.rx().wait_pull().await;

        if pulse_count < 70500 {
            Some(pulse_count as f32 * 5e-6 * 34.3)
        } else {
            None
        }
    }
}
