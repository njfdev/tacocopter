#[macro_export]
macro_rules! concat_elrs_bytes {
    ( $( $x:expr ),* ) => {
        {
          let mut vec = heapless::Vec::<u8, 64>::new();
          $(
            vec.extend($x);
          )*
          vec
        }
    };
}
