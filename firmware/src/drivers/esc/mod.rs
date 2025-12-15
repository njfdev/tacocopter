use embassy_rp::{
    interrupt::typelevel::Binding,
    pio::{Instance, InterruptHandler, Pin, Pio, PioPin},
    Peri,
};

pub mod blheli_passthrough;
mod dshot_encoder;
pub mod dshot_pio;

pub struct EscPins<'a, PIO: Instance> {
    pio: Pio<'a, PIO>,
    pins: [Pin<'a, PIO>; 4],
}

impl<'a, PIO: Instance> EscPins<'a, PIO> {
    pub fn new(
        pio_peri: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin_peri_0: Peri<'a, impl PioPin>,
        pin_peri_1: Peri<'a, impl PioPin>,
        pin_peri_2: Peri<'a, impl PioPin>,
        pin_peri_3: Peri<'a, impl PioPin>,
    ) -> Self {
        let mut pio = Pio::new(pio_peri, irq);
        Self {
            pins: [
                pio.common.make_pio_pin(pin_peri_0),
                pio.common.make_pio_pin(pin_peri_1),
                pio.common.make_pio_pin(pin_peri_2),
                pio.common.make_pio_pin(pin_peri_3),
            ],
            pio,
        }
    }
}
