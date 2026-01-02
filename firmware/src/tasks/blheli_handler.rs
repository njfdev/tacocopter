use critical_section::CriticalSection;
use embassy_futures::{select::select, yield_now};
use embassy_rp::{
    peripherals::{PIO0, USB},
    usb::Driver,
};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex},
    mutex::Mutex,
};
use embassy_usb::{class::cdc_acm::CdcAcmClass, driver::EndpointError};
use log::info;

use crate::{
    drivers::esc::{blheli_passthrough::BlHeliPassthrough, EscPins},
    global::FC_PASSTHROUGH_SIGNAL,
};

#[embassy_executor::task]
pub async fn blheli_handler(
    mut blheli_passthrough: BlHeliPassthrough<'static, PIO0>,
    mut serial_class_mutex: Mutex<
        CriticalSectionRawMutex,
        CdcAcmClass<'static, Driver<'static, USB>>,
    >,
    esc_pins_mutex: Mutex<CriticalSectionRawMutex, EscPins<'static, PIO0>>,
) {
    loop {
        if FC_PASSTHROUGH_SIGNAL.try_take().unwrap_or(false) {
            info!("Enabling BlHeli FC Passthrough");

            let serial_class = serial_class_mutex.get_mut();
            let mut esc_pins = esc_pins_mutex.lock().await;

            blheli_passthrough.enable_passthrough(&mut esc_pins);

            select(
                blheli_passthrough_processor(&mut blheli_passthrough, serial_class, &mut esc_pins),
                async {
                    loop {
                        if !FC_PASSTHROUGH_SIGNAL.wait().await {
                            return;
                        }
                    }
                },
            )
            .await;

            info!("Disabling BlHeli FC Passthrough");
            blheli_passthrough.disable_passthrough(&mut esc_pins);

            FC_PASSTHROUGH_SIGNAL.signal(false);
        }
        yield_now().await;
    }
}

async fn blheli_passthrough_processor(
    blheli_passthrough: &mut BlHeliPassthrough<'static, PIO0>,
    serial_class: &mut CdcAcmClass<'static, Driver<'static, USB>>,
    esc_pins: &mut EscPins<'static, PIO0>,
) -> Result<(), EndpointError> {
    serial_class.wait_connection().await;
    let mut buf = [0_u8; 512];
    loop {
        let n = serial_class.read_packet(&mut buf).await?;
        info!("rx data: {:?}", &buf[..n]);
        let data_res = blheli_passthrough
            .process_serial_data(&buf[..n], esc_pins)
            .await;
        if data_res.is_some() {
            let data = data_res.unwrap();
            let tx_data = &data.0[..data.1];
            info!("tx data: {:?}", &data.0[..data.1]);
            for chunk in tx_data.chunks(serial_class.max_packet_size() as usize) {
                serial_class.write_packet(chunk).await?;
            }
        }
    }
}
