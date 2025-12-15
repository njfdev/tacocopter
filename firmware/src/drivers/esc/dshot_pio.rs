// modified from https://github.com/peterkrull/dshot-pio

use core::any::Any;

use crate::drivers::esc::EscPins;

use super::dshot_encoder as dshot;

pub trait DshotPioTrait<'a, const N: usize, PIO: Instance> {
    fn command(&self, esc_pins: &mut EscPins<'a, PIO>, command: [u16; N]);
    fn reverse(&self, esc_pins: &mut EscPins<'a, PIO>, reverse: [bool; N]);
    fn throttle_clamp(&self, esc_pins: &mut EscPins<'a, PIO>, throttle: [u16; N]);
    fn throttle_minimum(&self, esc_pins: &mut EscPins<'a, PIO>);
}

use embassy_rp::{
    interrupt::typelevel::Binding,
    pio::{
        Config, Instance, InstanceMemory, InterruptHandler, LoadedProgram, Pio, PioPin,
        ShiftConfig, ShiftDirection::Left,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
    Peri,
};
use log::error;
use pio::Program;

// DShot600
const DSHOT_TARGET_FREQ: u32 = 8 * 600_000;
#[allow(dead_code)]
pub struct DshotPio<'a, const N: usize, PIO: Instance> {
    is_enabled: bool,
    loaded_prog: Option<LoadedProgram<'a, PIO>>,
}

fn configure_pio_instance<'a, PIO: Instance>(
    pio: &mut Pio<'a, PIO>,
) -> (Config<'a, PIO>, LoadedProgram<'a, PIO>) {
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
    let prog = pio.common.load_program(&dshot_pio_program.program.into());
    cfg.use_program(&prog, &[]);
    cfg.clock_divider = calculate_pio_clock_divider(DSHOT_TARGET_FREQ);

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

    (cfg, prog)
}

///
/// Defining constructor functions
///

impl<'a, PIO: Instance> DshotPio<'a, 4, PIO> {
    pub fn new() -> Self {
        Self {
            is_enabled: false,
            loaded_prog: None,
        }
    }

    pub fn enable_dshot(&mut self, esc_pins: &mut EscPins<'a, PIO>) {
        if self.is_enabled {
            return;
        }

        let (mut cfg, loaded_prog) = configure_pio_instance(&mut esc_pins.pio);

        self.loaded_prog = Some(loaded_prog);

        // Set pins and enable all state machines
        cfg.set_set_pins(&[&esc_pins.pins[0]]);
        esc_pins.pio.sm0.set_config(&cfg);
        esc_pins.pio.sm0.set_enable(true);

        cfg.set_set_pins(&[&esc_pins.pins[1]]);
        esc_pins.pio.sm1.set_config(&cfg);
        esc_pins.pio.sm1.set_enable(true);

        cfg.set_set_pins(&[&esc_pins.pins[2]]);
        esc_pins.pio.sm2.set_config(&cfg);
        esc_pins.pio.sm2.set_enable(true);

        cfg.set_set_pins(&[&esc_pins.pins[3]]);
        esc_pins.pio.sm3.set_config(&cfg);
        esc_pins.pio.sm3.set_enable(true);

        self.is_enabled = true;
    }

    pub fn disable_dshot(&mut self, esc_pins: &mut EscPins<'a, PIO>) {
        if !self.is_enabled {
            return;
        }

        esc_pins.pio.sm0.set_enable(false);
        esc_pins.pio.sm0.clear_fifos();

        esc_pins.pio.sm1.set_enable(false);
        esc_pins.pio.sm1.clear_fifos();

        esc_pins.pio.sm2.set_enable(false);
        esc_pins.pio.sm2.clear_fifos();

        esc_pins.pio.sm3.set_enable(false);
        esc_pins.pio.sm3.clear_fifos();

        unsafe {
            esc_pins
                .pio
                .common
                .free_instr(self.loaded_prog.take().unwrap().used_memory);
        }

        self.is_enabled = false;
    }
}

///
/// Implementing DshotPioTrait
///
///
impl<'a, PIO: Instance> DshotPioTrait<'a, 4, PIO> for DshotPio<'a, 4, PIO> {
    /// Send any valid DShot value to the ESC.
    fn command(&self, esc_pins: &mut EscPins<'a, PIO>, command: [u16; 4]) {
        if !self.is_enabled {
            error!("Tried using DShot but it is disabled!");
            return;
        }

        if !esc_pins.pio.sm0.tx().full() {
            esc_pins.pio.sm0.tx().push(command[0] as u32);
            esc_pins.pio.sm1.tx().push(command[1] as u32);
            esc_pins.pio.sm2.tx().push(command[2] as u32);
            esc_pins.pio.sm3.tx().push(command[3] as u32);
        }
    }

    /// Set the direction of rotation for each motor
    fn reverse(&self, esc_pins: &mut EscPins<'a, PIO>, reverse: [bool; 4]) {
        if !self.is_enabled {
            error!("Tried using DShot but it is disabled!");
            return;
        }

        esc_pins
            .pio
            .sm0
            .tx()
            .push(dshot::reverse(reverse[0]) as u32);
        esc_pins
            .pio
            .sm1
            .tx()
            .push(dshot::reverse(reverse[1]) as u32);
        esc_pins
            .pio
            .sm2
            .tx()
            .push(dshot::reverse(reverse[2]) as u32);
        esc_pins
            .pio
            .sm3
            .tx()
            .push(dshot::reverse(reverse[3]) as u32);
    }

    /// Set the throttle for each motor. All values are clamped between 48 and 2047
    fn throttle_clamp(&self, esc_pins: &mut EscPins<'a, PIO>, throttle: [u16; 4]) {
        if !self.is_enabled {
            error!("Tried using DShot but it is disabled!");
            return;
        }

        esc_pins
            .pio
            .sm0
            .tx()
            .push(dshot::throttle_clamp(throttle[0], false) as u32);
        esc_pins
            .pio
            .sm1
            .tx()
            .push(dshot::throttle_clamp(throttle[1], false) as u32);
        esc_pins
            .pio
            .sm2
            .tx()
            .push(dshot::throttle_clamp(throttle[2], false) as u32);
        esc_pins
            .pio
            .sm3
            .tx()
            .push(dshot::throttle_clamp(throttle[3], false) as u32);
    }

    /// Set the throttle for each motor to zero (DShot command 48)
    fn throttle_minimum(&self, esc_pins: &mut EscPins<'a, PIO>) {
        if !self.is_enabled {
            error!("Tried using DShot but it is disabled!");
            return;
        }

        esc_pins
            .pio
            .sm0
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
        esc_pins
            .pio
            .sm1
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
        esc_pins
            .pio
            .sm2
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
        esc_pins
            .pio
            .sm3
            .tx()
            .push(dshot::throttle_minimum(false) as u32);
    }
}
