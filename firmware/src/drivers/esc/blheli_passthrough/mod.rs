// modified implementation from https://github.com/BrushlessPower/BlHeli-Passthrough

mod four_way;
mod msp;

use embassy_rp::{
    clocks::clk_sys_freq,
    interrupt::typelevel::Binding,
    pio::{
        Config, Instance, InterruptHandler, LoadedProgram, Pin, Pio, PioPin, ShiftConfig,
        ShiftDirection,
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
    Peri,
};
use log::{error, info};
use nalgebra::ComplexField;

use crate::drivers::esc::{blheli_passthrough::msp::process_msp, EscPins};

pub struct BlHeliPassthrough<'a, PIO: Instance> {
    cfg_rx: Option<Config<'a, PIO>>,
    cfg_tx: Option<Config<'a, PIO>>,
    loaded_rx_prog: Option<LoadedProgram<'a, PIO>>,
    loaded_tx_prog: Option<LoadedProgram<'a, PIO>>,

    // processing vars
    len: usize,
    buffer: [u8; 263], // max message size is 256 bytes + 7 overhead
}

fn configure_programs<'a, PIO: Instance>(
    pio: &mut Pio<'a, PIO>,
) -> (
    Config<'a, PIO>,
    Config<'a, PIO>,
    LoadedProgram<'a, PIO>,
    LoadedProgram<'a, PIO>,
) {
    // Define programs
    let passthrough_tx_pio_program = embassy_rp::pio::program::pio_asm!(
        // a small delay here tends to reduce transmission errors if the pullup is missing
        "set pins, 1 [2]"
        "set pindirs, 1 [2]"
        ".wrap_target"
        "pull block"
        "set pins, 0 [15] ; start bit"
        "write_bit:"
        "out pins, 1 [14] ; 8 data bits"
        "jmp !osre, write_bit"
        "set pins, 1 [14] ; stop bit"
        ".wrap"
    );
    let passthrough_rx_pio_program = embassy_rp::pio::program::pio_asm!(
        // a small delay here tends to reduce transmission errors if the pullup is missing
        "set pindirs, 0 [5]"
        "discard:"
        "mov isr, null"
        ".wrap_target"
        "set X, 7"
        "wait 0 pin, 0 [23]" // wait for start bit, delay 1.5 bits to sample in the center of each bit
        "read_bit:"
        "in pins, 1 [14]" // sample 8 bits
        "jmp X--, read_bit"
        "jmp pin, push_byte" // discard bit if the stop bit is not present
        "jmp discard [2]" // reduced delay to leave room for slight clock deviations
        "push_byte:"
        "push block [3]" // reduced delay to leave room for slight clock deviations
        ".wrap"
    );

    // Configure programs
    let mut cfg_tx = Config::default();
    let mut cfg_rx = Config::default();

    let loaded_tx_prog = pio
        .common
        .load_program(&passthrough_tx_pio_program.program.into());
    cfg_tx.use_program(&loaded_tx_prog, &[]);

    let loaded_rx_prog = pio
        .common
        .load_program(&passthrough_rx_pio_program.program.into());
    cfg_rx.use_program(&loaded_rx_prog, &[]);

    let divider = calculate_pio_clock_divider((1.0 / 3.25e-6) as u32);
    cfg_tx.clock_divider = divider;
    cfg_rx.clock_divider = divider;

    cfg_rx.shift_in = ShiftConfig {
        auto_fill: true,
        direction: ShiftDirection::Left,
        threshold: 32,
    };

    cfg_tx.shift_out = ShiftConfig {
        auto_fill: true,
        direction: ShiftDirection::Left,
        threshold: 8,
    };

    (cfg_rx, cfg_tx, loaded_rx_prog, loaded_tx_prog)
}

impl<'a, PIO: Instance> BlHeliPassthrough<'a, PIO> {
    pub fn new() -> Self {
        Self {
            cfg_rx: None,
            cfg_tx: None,
            loaded_rx_prog: None,
            loaded_tx_prog: None,
            len: 0,
            buffer: [0; 263],
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.cfg_rx.is_some()
            && self.cfg_tx.is_some()
            && self.loaded_rx_prog.is_some()
            && self.loaded_tx_prog.is_some()
    }

    fn configure_rx(&mut self, esc_pins: &mut EscPins<'a, PIO>) {
        if !self.is_enabled() {
            error!("Tried configuring passthrough but PIO isn't set up!");
            return;
        }

        let mut cfg_rx = self.cfg_rx.unwrap();

        esc_pins.pio.sm0.set_enable(false);
        cfg_rx.set_set_pins(&[&esc_pins.pins[0]]);
        cfg_rx.set_in_pins(&[&esc_pins.pins[0]]);
        cfg_rx.set_jmp_pin(&esc_pins.pins[0]);
        esc_pins.pio.sm0.set_config(&cfg_rx);
        esc_pins.pio.sm0.set_enable(true);

        esc_pins.pio.sm1.set_enable(false);
        cfg_rx.set_set_pins(&[&esc_pins.pins[1]]);
        cfg_rx.set_in_pins(&[&esc_pins.pins[1]]);
        cfg_rx.set_jmp_pin(&esc_pins.pins[1]);
        esc_pins.pio.sm1.set_config(&cfg_rx);
        esc_pins.pio.sm1.set_enable(true);

        esc_pins.pio.sm2.set_enable(false);
        cfg_rx.set_set_pins(&[&esc_pins.pins[2]]);
        cfg_rx.set_in_pins(&[&esc_pins.pins[2]]);
        cfg_rx.set_jmp_pin(&esc_pins.pins[2]);
        esc_pins.pio.sm2.set_config(&cfg_rx);
        esc_pins.pio.sm2.set_enable(true);

        esc_pins.pio.sm3.set_enable(false);
        cfg_rx.set_set_pins(&[&esc_pins.pins[3]]);
        cfg_rx.set_in_pins(&[&esc_pins.pins[3]]);
        cfg_rx.set_jmp_pin(&esc_pins.pins[3]);
        esc_pins.pio.sm3.set_config(&cfg_rx);
        esc_pins.pio.sm3.set_enable(true);
    }

    fn configure_tx(&mut self, esc_pins: &mut EscPins<'a, PIO>) {
        if !self.is_enabled() {
            error!("Tried configuring passthrough but PIO isn't set up!");
            return;
        }

        let mut cfg_tx = self.cfg_tx.unwrap();

        esc_pins.pio.sm0.set_enable(false);
        cfg_tx.set_set_pins(&[&esc_pins.pins[0]]);
        cfg_tx.set_out_pins(&[&esc_pins.pins[0]]);
        esc_pins.pio.sm0.set_config(&cfg_tx);
        esc_pins.pio.sm0.set_enable(true);

        esc_pins.pio.sm1.set_enable(false);
        cfg_tx.set_set_pins(&[&esc_pins.pins[1]]);
        cfg_tx.set_out_pins(&[&esc_pins.pins[1]]);
        esc_pins.pio.sm1.set_config(&cfg_tx);
        esc_pins.pio.sm1.set_enable(true);

        esc_pins.pio.sm2.set_enable(false);
        cfg_tx.set_set_pins(&[&esc_pins.pins[2]]);
        cfg_tx.set_out_pins(&[&esc_pins.pins[2]]);
        esc_pins.pio.sm2.set_config(&cfg_tx);
        esc_pins.pio.sm2.set_enable(true);

        esc_pins.pio.sm3.set_enable(false);
        cfg_tx.set_set_pins(&[&esc_pins.pins[3]]);
        cfg_tx.set_out_pins(&[&esc_pins.pins[3]]);
        esc_pins.pio.sm3.set_config(&cfg_tx);
        esc_pins.pio.sm3.set_enable(true);
    }

    pub fn enable_passthrough(&mut self, esc_pins: &mut EscPins<'a, PIO>) {
        let (cfg_rx, cfg_tx, loaded_rx_prog, loaded_tx_prog) =
            configure_programs(&mut esc_pins.pio);
        self.cfg_rx = Some(cfg_rx);
        self.cfg_tx = Some(cfg_tx);
        self.loaded_rx_prog = Some(loaded_rx_prog);
        self.loaded_tx_prog = Some(loaded_tx_prog);

        self.configure_rx(esc_pins);
    }

    pub fn disable_passthrough(&mut self, esc_pins: &mut EscPins<'a, PIO>) {
        self.cfg_rx = None;
        self.cfg_tx = None;

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
                .free_instr(self.loaded_rx_prog.take().unwrap().used_memory);
            esc_pins
                .pio
                .common
                .free_instr(self.loaded_tx_prog.take().unwrap().used_memory);
        }
    }

    pub fn process_serial_data(&mut self, data: &[u8]) -> Option<([u8; 263], usize)> {
        self.buffer[self.len..(self.len + data.len())].copy_from_slice(&data[..data.len()]);
        self.len += data.len();

        if self.len == 0 {
            return None;
        }

        // 4 way command (256 bytes + 7 byte overhead)
        if self.buffer[0] == 0x2F
            && self.len > 4
            && self.len
                == (if self.buffer[4] != 0 {
                    self.buffer[0] as usize
                } else {
                    256
                }) + 7
        {
            self.len = 0;
            return None;

            // let tx_data = process_4_way(&self.buffer, self.len);
            // self.len = 0;
            // return tx_data;
        }

        // MSP Command (6 byte overhead)
        if self.len > 3
            && self.len as u8 == self.buffer[3] + 6
            && self.buffer[0] == 0x24
            && self.buffer[1] == 0x4D
            && self.buffer[2] == 0x3C
        {
            let tx_data = process_msp::<PIO>(&self.buffer, self.len);
            self.len = 0;
            return tx_data;
        }

        None
    }
}
