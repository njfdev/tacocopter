use crate::SHARED;

#[macro_export]
macro_rules! tc_print {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        let mut buf = heapless::String::<256>::new(); // Adjust size as needed
        core::write!(&mut buf, $($arg)*).unwrap();

        let text = buf.as_str();
        let mut cur_index = 0;

        while cur_index < text.len() {
            {
                let mut shared = $crate::SHARED.lock().await;
                let available = core::cmp::min(text.len() - cur_index, shared.log_buffer.capacity() - shared.log_buffer.len());
                shared.log_buffer.push_str(&text[cur_index..(cur_index + available)]).unwrap();
                cur_index += available;
            }
            if cur_index < text.len() {
                Timer::after_millis(50).await;
            }
        }
    }};
}

#[macro_export]
macro_rules! tc_println {
    () => {
        $crate::tc_print!("\n");
    };
    ($fmt:literal $(, $arg:tt)*) => {
        $crate::tc_print!(concat!($fmt, "\n") $(, $arg)*);
    };
}
