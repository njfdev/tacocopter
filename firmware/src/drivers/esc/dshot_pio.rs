// modified from https://github.com/peterkrull/dshot-pio

use super::dshot_encoder as dshot;

pub trait DshotPioTrait<const N: usize> {
    fn command(&mut self, command: [u16; N]);
    fn reverse(&mut self, reverse: [bool; N]);
    fn throttle_clamp(&mut self, throttle: [u16; N]);
    fn throttle_minimum(&mut self);
}

use embassy_rp::{
    interrupt::typelevel::Binding,
    pio::{Config, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection::Left},
    pio_programs::clock_divider::calculate_pio_clock_divider,
    Peri,
};
#[allow(dead_code)]
pub struct DshotPio<'a, const N: usize, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
}

fn configure_pio_instance<'a, PIO: Instance>(
    pio: Peri<'a, PIO>,
    irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
    target_freq: u32,
) -> (Config<'a, PIO>, Pio<'a, PIO>) {
    // Define program
    let dshot_pio_program = embassy_rp::pio::program::pio_asm!(
        "set pindirs, 1",
        "entry:"
        "   pull"
        "   out null 16"
        "   set x 15"
        "loop:"
        "   set pins 1"
        "   out y 1"
        "   jmp !y zero"
        "   nop [2]"
        "one:" // 6 and 2
        "   set pins 0"
        "   jmp x-- loop"
        "   jmp reset"
        "zero:" // 3 and 5
        "   set pins 0 [3]"
        "   jmp x-- loop"
        "   jmp reset"
        "reset:" // Blank frame
        "   nop [31]"
        "   nop [31]"
        "   nop [31]"
        "   jmp entry [31]"
    );

    // Configure program
    let mut cfg = Config::default();
    let mut pio = Pio::new(pio, irq);
    cfg.use_program(
        &pio.common.load_program(&dshot_pio_program.program.into()),
        &[],
    );
    cfg.clock_divider = calculate_pio_clock_divider(target_freq);

    cfg.shift_in = ShiftConfig {
        auto_fill: true,
        direction: Default::default(),
        threshold: 32,
    };

    cfg.shift_out = ShiftConfig {
        auto_fill: Default::default(),
        direction: Left,
        threshold: Default::default(),
    };

    (cfg, pio)
}

///
/// Defining constructor functions
///

impl<'a, PIO: Instance> DshotPio<'a, 4, PIO> {
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin>,
        pin1: Peri<'a, impl PioPin>,
        pin2: Peri<'a, impl PioPin>,
        pin3: Peri<'a, impl PioPin>,
        target_freq: u32,
    ) -> DshotPio<'a, 4, PIO> {
        let (mut cfg, mut pio) = configure_pio_instance(pio, irq, target_freq);

        // Set pins and enable all state machines
        let pin0 = pio.common.make_pio_pin(pin0);
        cfg.set_set_pins(&[&pin0]);
        pio.sm0.set_config(&cfg);
        pio.sm0.set_enable(true);

        let pin1 = pio.common.make_pio_pin(pin1);
        cfg.set_set_pins(&[&pin1]);
        pio.sm1.set_config(&cfg);
        pio.sm1.set_enable(true);

        let pin2 = pio.common.make_pio_pin(pin2);
        cfg.set_set_pins(&[&pin2]);
        pio.sm2.set_config(&cfg);
        pio.sm2.set_enable(true);

        let pin3 = pio.common.make_pio_pin(pin3);
        cfg.set_set_pins(&[&pin3]);
        pio.sm3.set_config(&cfg);
        pio.sm3.set_enable(true);

        // Return struct of 4 configured DShot state machines
        DshotPio { pio_instance: pio }
    }
}

///
/// Implementing DshotPioTrait
///
///
impl<'d, PIO: Instance> DshotPioTrait<4> for DshotPio<'d, 4, PIO> {
    /// Send any valid DShot value to the ESC.
    fn command(&mut self, command: [u16; 4]) {
        if !self.pio_instance.sm0.tx().full() {
            self.pio_instance.sm0.tx().push(command[0] as u32);
            self.pio_instance.sm1.tx().push(command[1] as u32);
            self.pio_instance.sm2.tx().push(command[2] as u32);
            self.pio_instance.sm3.tx().push(command[3] as u32);
        }
    }

    /// Set the direction of rotation for each motor
    fn reverse(&mut self, reverse: [bool; 4]) {
        self.pio_instance
            .sm0
            .tx()
            .push(dshot::reverse(reverse[0]) as u32);
        self.pio_instance
            .sm1
            .tx()
            .push(dshot::reverse(reverse[1]) as u32);
        self.pio_instance
            .sm2
            .tx()
            .push(dshot::reverse(reverse[2]) as u32);
        self.pio_instance
            .sm3
            .tx()
            .push(dshot::reverse(reverse[3]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&mut self, throttle: [u16; 4]) {
        self.pio_instance
            .sm0
            .tx()
            .push(dshot::throttle_clamp(throttle[0], false) as u32);
        self.pio_instance
            .sm1
            .tx()
            .push(dshot::throttle_clamp(throttle[1], false) as u32);
        self.pio_instance
            .sm2
            .tx()
            .push(dshot::throttle_clamp(throttle[2], false) as u32);
        self.pio_instance
            .sm3
            .tx()
            .push(dshot::throttle_clamp(throttle[3], false) as u32);
    }

    /// Set the throttle for each motor to zero (DShot command 48)
    fn throttle_minimum(&mut self) {
        self.pio_instance
            .sm0
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
        self.pio_instance
            .sm1
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
        self.pio_instance
            .sm2
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
        self.pio_instance
            .sm3
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
    }
}
