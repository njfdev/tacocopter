use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{self, Level, Output, Pin},
    peripherals::{self, PIN_8, PIN_9, PIO0, UART1},
    pio::{InterruptHandler, Pio, ShiftConfig, ShiftDirection},
    uart::{self, BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx},
    Peripheral, Peripherals,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use log::{error, info, warn};
use micromath::F32Ext;
use pio::pio_asm;
use static_cell::StaticCell;

use crate::{tc_print, tc_println, SHARED};

pub enum DShotCommand {
    MotorStop = 0,
    Beep1 = 1,
    Beep2 = 2,
    Beep3 = 3,
    Beep4 = 4,
    Beep5 = 5,
    SpinDirection1 = 7,
    SpinDirection2 = 8,
    SpinDirectionNormal = 20,
    SpinDirectionReverse = 21,
}

pub enum DShotSpeed {
    DShot150,
    DShot300,
    DShot600,
    DShot1200,
}

pub struct DShotESC {
    esc_pwr_sender: Sender<'static, ThreadModeRawMutex, u16, 1>,
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

// TODO: Add telemetry and bidirectional support
impl DShotESC {
    pub fn new<P: Pin>(pin: P, dshot_speed: DShotSpeed, spawner: &Spawner) -> Self {
        let esc_output = Output::new(pin, Level::Low);

        static ESC_PWR_CHANNEL: StaticCell<Channel<ThreadModeRawMutex, u16, 1>> = StaticCell::new();
        let esc_pwr_channel = ESC_PWR_CHANNEL.init(Channel::new());
        let receiver = esc_pwr_channel.receiver();
        let sender = esc_pwr_channel.sender();

        spawner
            .spawn(dshot_handler(esc_output, receiver, dshot_speed))
            .unwrap();

        Self {
            esc_pwr_sender: sender,
        }
    }

    // Input value should be between 0 and 1
    pub fn set_pwr(&self, value: f32) {
        let dshot_val = (value.clamp(0.0, 1.0) * (2000.0 - 1.0)) as u16 + 48;
        let _ = self.esc_pwr_sender.try_send(dshot_val);
    }

    pub fn run_cmd(&self, value: u16) {
        // TODO: implement
        // let _ = self.esc_pwr_sender.try_send(value);
    }

    pub fn calc_crc(data: u16) -> u16 {
        return (data ^ (data >> 4) ^ (data >> 8)) & 0x0f;
    }

    pub fn setup_pio_task(pio_controller: PIO0) {
        let pio_handle = Pio::new(pio_controller, Irqs);

        // TODO: fix timing between packets not being consistent or of correct timing
        // let mut prog = pio_asm!(
        //     ".wrap_target",
        //     "pull noblock",     // Pull 32-bit data from TX FIFO, but if empty used stored in x register
        //     "out isr, 16",      // push frame from OSR to ISR
        //     "mov x, isr",       // copy frame to X register
        //     "jmp !OSRE, main",  // if the OSR is not empty, skip this next step
        //     "mov x, osr",       // copy the latter 16 bits to x for what we should use when no new frames are sent
        //     "main:",
        //     "mov osr, isr",     // copy the frame we actually need to send from ISR to OSR
        //     "mov isr, x",       // temporary hold the frame to save from X to ISR (so we can use X as a scratch register)
        //     "set y, 16",        // start the 16 bit counter to loop through the frame
        //     "bit_output_loop:",
        //     "out x, 1",         // shift 1 bit into register x
        //     "set pins, 1 [31]", // pull pin high
        //     "mov y, y [31]",
        //     "jmp X--, longer_bit_hold [10]",  // if the bit is a 1, skip pulling the pin low for now
        //     "set pins, 0",
        //     "jmp if_zero",
        //     "longer_bit_hold:"
        //     "mov y, y [1]",
        //     "if_zero:"
        //     "mov y, y [29]",    // more delaying steps
        //     "mov y, y [31]",
        //     "mov y, y [10]",
        //     "set pins, 0 [31]", // set the pin low (either already low if 0, or needs to be low for a 1)
        //     "mov y, y [14]",
        //     "mov x, null",       // reset register X
        //     "jmp Y--, bit_output_loop", // if more bits, keep processing
        //     // copy saved frame from ISR to X, and copy it twice
        //     "mov osr, isr",
        //     "out x, 16",
        //     "mov osr, isr",
        //     "out x, 16",
        //     "mov isr, null",       // reset ISR
        //     ".wrap"
        // );

        // let mut config = embassy_rp::pio::Config::default();
        // config.use_program(&pio_handle.common.load_program(&prog.program), &[]);
        // config.shift_out = ShiftConfig {
        //     auto_fill: false,
        //     threshold: 32,
        //     direction: ShiftDirection::Right,
        // }
    }
}

// order of return params in nanoseconds are: (frame length, bit length, 1 length, 0 length)
fn dshot_enum_to_timings(dshot_speed: DShotSpeed) -> (u64, u64, u64, u64) {
    let bit_length_ns: f64 = 1_000_000_000.0
        / (match dshot_speed {
            DShotSpeed::DShot150 => 150_000.0,
            DShotSpeed::DShot300 => 300_000.0,
            DShotSpeed::DShot600 => 600_000.0,
            DShotSpeed::DShot1200 => 1200_000.0,
        });

    return (
        (bit_length_ns * 16.0 + 0.5) as u64,
        (bit_length_ns + 0.5) as u64,
        (bit_length_ns * 0.75 + 0.5) as u64,
        (bit_length_ns * 0.375 + 0.5) as u64,
    );
}

#[embassy_executor::task]
async fn dshot_handler(
    mut esc_output: Output<'static>,
    value_receiver: Receiver<'static, ThreadModeRawMutex, u16, 1>,
    dshot_speed: DShotSpeed,
) {
    let (frame_len, bit_len, one_len, zero_len) = dshot_enum_to_timings(dshot_speed);

    let mut esc_value: u16 = 0;
    let mut cur_bit: u8 = 0;
    let mut data: u16 = 0;

    let mut time_since_frame_start = Instant::now();

    loop {
        if cur_bit > 15 {
            // if new value, get and set it
            if value_receiver.capacity() > 0 {
                let received_res = value_receiver.try_receive();
                if (received_res.is_ok()) {
                    esc_value = received_res.unwrap();
                }
            }
            let crc = DShotESC::calc_crc(esc_value << 1);
            data = (esc_value << 5) + crc;
            cur_bit = 0;
            time_since_frame_start = Instant::now();
        }

        esc_output.set_high();

        let is_one = (data >> (11 - cur_bit)) & 1 == 1;

        Timer::after_nanos(if is_one { one_len } else { zero_len });

        esc_output.set_low();

        let time_to_wait = Instant::now().duration_since(time_since_frame_start);
    }
}
