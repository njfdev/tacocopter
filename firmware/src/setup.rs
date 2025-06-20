use embassy_rp::{
    clocks::{ClockConfig, SysClkSrc},
    config::Config,
    pac,
};
use nalgebra::ComplexField;

pub fn setup_clock_speeds(overclock_frequency: u64) -> Config {
    // // 1. Enable XOSC (12 MHz crystal)
    // pac::XOSC.ctrl().write(|w| unsafe { w.bits(0xaa0) });
    // pac::XOSC.startup().write(|w| unsafe { w.bits(0xc4) });
    // while pac::XOSC.status().read().stable().bit_is_clear() {}

    // // 2. Power down PLL_SYS and reset
    // pac::PLL_SYS.pwr().write(|w| w.pd().set_bit());
    // pac::PLL_SYS.cs().write(|w| w.bypass().set_bit());

    // // 3. Calculate PLL parameters for the requested frequency
    // // Fout = 12_000_000 * fbdiv / refdiv / post_div1 / post_div2
    // // We'll use refdiv=1, post_div2=1, and choose post_div1 and fbdiv to get close to the target
    // let refdiv = 1;
    // let post_div2 = 1;
    // // Try post_div1 from 1 to 7 and pick the best match
    // let mut best_post_div1 = 1;
    // let mut best_fbdiv = 1;
    // let mut best_freq = 0;
    // for post_div1 in 1..=7 {
    //     let fbdiv =
    //         (overclock_frequency as u64 * refdiv as u64 * post_div1 as u64 * post_div2 as u64)
    //             / 12_000_000;
    //     let freq = 12_000_000 * fbdiv / refdiv / post_div1 / post_div2;
    //     if freq <= overclock_frequency && freq > best_freq {
    //         best_post_div1 = post_div1 as u8;
    //         best_fbdiv = fbdiv as u16;
    //         best_freq = freq;
    //     }
    // }

    // // 4. Configure PLL_SYS
    // pac::PLL_SYS
    //     .fbdiv_int()
    //     .write(|w| unsafe { w.bits(best_fbdiv as u32) });
    // pac::PLL_SYS.prim().write(|w| unsafe {
    //     w.postdiv1()
    //         .bits(best_post_div1 as u8)
    //         .postdiv2()
    //         .bits(post_div2 as u8)
    // });
    // pac::PLL_SYS.pwr().write(|w| w.pd().clear_bit()); // Power up
    // pac::PLL_SYS.cs().write(|w| w.bypass().clear_bit());

    // // 5. Wait for PLL lock
    // while pac::PLL_SYS.cs().read().lock().bit_is_clear() {}

    // let mut clocks = ClockConfig::crystal(12_000_000);
    // clocks.sys_clk.div_int = 1;
    // clocks.sys_clk.div_frac = 0;
    // clocks.sys_clk.src = SysClkSrc::PllSys;
    // let conf = Config::new(clocks);
    // conf
    Default::default()
}
