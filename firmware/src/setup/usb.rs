use embassy_rp::{
    bind_interrupts,
    peripherals::USB,
    usb::{Driver, Endpoint, In, InterruptHandler, Out},
    Peri,
};
use embassy_usb::{Builder, Handler, UsbDevice};
use log::info;
use tc_interface::{TC_PID, TC_VID};

use crate::global::USB_ENABLED;

bind_interrupts!(struct UsbIrq {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

struct UsbStateHandler {}

static mut USB_STATE_HANDLER: UsbStateHandler = UsbStateHandler {};

impl Handler for UsbStateHandler {
    fn configured(&mut self, configured: bool) {
        USB_ENABLED.sender().send(configured);
    }

    fn suspended(&mut self, suspended: bool) {
        USB_ENABLED.sender().send(!suspended);
    }
}

pub fn setup_usb_interface(
    usb_peripheral: Peri<'static, USB>,
) -> (
    UsbDevice<'static, Driver<'static, USB>>,
    Endpoint<'static, USB, In>,
    Endpoint<'static, USB, Out>,
) {
    let driver = Driver::new(usb_peripheral, UsbIrq);

    // setup the USB driver
    let mut config = embassy_usb::Config::new(TC_VID, TC_PID);
    config.manufacturer = Some("Nicholas Fasching");
    config.product = Some("Tacocopter Flight Controller");
    config.max_power = 100; // 100mA

    // USB buffers
    static mut DEVICE_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
    static mut CONTROL_BUF: [u8; 64] = [0; 64];

    // create USB builder
    let mut builder = Builder::new(
        driver,
        config,
        unsafe { &mut *&raw mut DEVICE_DESCRIPTOR },
        unsafe { &mut *&raw mut CONFIG_DESCRIPTOR },
        unsafe { &mut *&raw mut BOS_DESCRIPTOR },
        unsafe { &mut *&raw mut CONTROL_BUF },
    );

    // add handler to detect USB events (e.g., to determine if USB is connected)
    builder.handler(unsafe { &mut USB_STATE_HANDLER });

    // Add bulk endpoints (OUT and IN)
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    let bulk_out_ep = alt.endpoint_bulk_out(None, 64); // 64-byte packets
    let bulk_in_ep = alt.endpoint_bulk_in(None, 64); // 64-byte packets
    drop(function);

    // Build and run USB device
    let usb = builder.build();

    (usb, bulk_in_ep, bulk_out_ep)
}
