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
use embassy_time::{Duration, Instant};
use log::{error, info, warn};
use nalgebra::ComplexField;

use crate::{
    drivers::esc::{
        blheli_passthrough::{four_way::process_4_way, msp::process_msp},
        EscPins,
    },
    panic,
};

pub struct BlHeliPassthrough<'a, PIO: Instance> {
    cfg_rx: Option<Config<'a, PIO>>,
    cfg_tx: Option<Config<'a, PIO>>,
    loaded_rx_prog: Option<LoadedProgram<'a, PIO>>,
    loaded_tx_prog: Option<LoadedProgram<'a, PIO>>,

    // processing vars
    rx_len: usize,
    rx_buffer: [u8; 263], // max message size is 256 bytes + 7 overhead
    tx_len: usize,
    tx_buffer: [u8; 263],
    is_4_way_passthrough_enabled: bool,
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

            rx_len: 0,
            rx_buffer: [0; 263],
            tx_len: 0,
            tx_buffer: [0; 263],
            is_4_way_passthrough_enabled: false,
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

    pub async fn process_serial_data(
        &mut self,
        data: &[u8],
        esc_pins: &mut EscPins<'a, PIO>,
    ) -> Option<([u8; 263], usize)> {
        self.rx_buffer[self.rx_len..(self.rx_len + data.len())]
            .copy_from_slice(&data[..data.len()]);
        self.rx_len += data.len();

        if self.rx_len == 0 {
            return None;
        }

        if self.rx_len > 263 {
            warn!("Passthrough buffer overflow! (likely improper data handling)");
            self.rx_len = 0;
            return None;
        }

        // 4 way command (256 bytes + 7 byte overhead)
        if self.rx_buffer[0] == 0x2F
            && self.rx_len > 4
            && self.rx_len
                == (if self.rx_buffer[4] != 0 {
                    self.rx_buffer[4] as usize
                } else {
                    256
                }) + 7
        {
            let tx_data = process_4_way(self, esc_pins).await;
            self.rx_len = 0;
            return tx_data;
        }

        // MSP Command (6 byte overhead)
        if self.rx_len > 3
            && self.rx_len as u8 == self.rx_buffer[3] + 6
            && self.rx_buffer[0] == 0x24
            && self.rx_buffer[1] == 0x4D
            && self.rx_buffer[2] == 0x3C
        {
            let tx_data = process_msp::<PIO>(self);
            self.rx_len = 0;
            return tx_data;
        }

        None
    }

    pub fn send_data_to_esc(
        &mut self,
        esc_pins: &mut EscPins<'a, PIO>,
        pin: usize,
        data: &[u8],
        len: usize,
        crc: bool,
    ) {
        let buf_len = if len == 0 { 256 } else { 0 };
        let mut tx_crc = 0;

        self.configure_tx(esc_pins);

        for byte in &data[..buf_len] {
            BlHeliPassthrough::send_esc_byte(esc_pins, pin, *byte);
            tx_crc = passthrough_byte_crc(*byte, tx_crc);
        }

        if crc {
            BlHeliPassthrough::send_esc_byte(esc_pins, pin, (tx_crc & 0xFF) as u8);
            BlHeliPassthrough::send_esc_byte(esc_pins, pin, ((tx_crc >> 8) & 0xFF) as u8);
        }

        self.configure_rx(esc_pins);
    }

    // pub fn get_esc_data_for_duration(esc_pins: &mut EscPins<'a, PIO>, duration: Duration) {
    //   let start = Instant::now();
    //   while start.elapsed() < duration &&
    // }

    fn send_esc_byte(esc_pins: &mut EscPins<'a, PIO>, pin: usize, byte: u8) {
        match pin {
            0 => esc_pins.pio.sm0.tx().push(byte as u32),
            1 => esc_pins.pio.sm1.tx().push(byte as u32),
            2 => esc_pins.pio.sm2.tx().push(byte as u32),
            3 => esc_pins.pio.sm3.tx().push(byte as u32),
            _ => warn!("Can't send byte to state machine {}, ignoring...", pin),
        }
    }
}

fn passthrough_byte_crc(data: u8, crc: u16) -> u16 {
    let mut xb = data;
    let mut new_crc = crc;
    for i in 0..8 {
        if ((xb as u16 & 0x01) ^ (new_crc & 0x0001)) != 0 {
            new_crc = new_crc >> 1;
            new_crc = new_crc ^ 0xA001;
        } else {
            new_crc = new_crc >> 1;
        }
        xb = xb >> 1;
    }
    return new_crc;
}
