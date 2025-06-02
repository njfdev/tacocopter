#[macro_export]
macro_rules! tc_print {
    ($($arg:tt)*) => {{
        use core::fmt::Write;
        use heapless::String;

        let mut s: String<1024> = String::new();
        let _ = write!(s, $($arg)*);

        let mut start = 0;
        let len = s.len();

        while start < len {
            let mut end = core::cmp::min(start + 60, len);

            // Move end back until it points at a char boundary
            while !s.is_char_boundary(end) && end > start {
                end -= 1;
            }

            let slice = &s[start..end];

            // Push slice safely
            let mut part: String<60> = String::new();
            let _ = part.push_str(slice);

            let _ = $crate::LOG_CHANNEL.try_send(part);

            start = end;
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
