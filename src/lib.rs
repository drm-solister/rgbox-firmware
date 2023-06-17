#![no_std]

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;

#[cfg(feature = "rt")]
pub use hal::entry;

#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

hal::bsp_pins!(
    Gpio7 {
        name: sp6,
    },
    Gpio8 {
        name: sp5,
    },
    Gpio9 {
        name: sp4,
    },
    Gpio10 {
        name: sp3,
    },
    Gpio11 {
        name: sp2,
    },
    Gpio12 {
        name: sp1,
    },
    Gpio13 {
        name: joy1,
    },
    Gpio14 {
        name: joy2,
    },
    Gpio15 {
        name: joy3,
    },
    Gpio16 {
        name: joy4,
    },
    Gpio17 {
        name: argb,
        aliases: {
            FunctionPio0: Gp17Pio0
        }
    },
    Gpio18 {
        name: row0,
    },
    Gpio19 {
        name: row1,
    },
    Gpio20 {
        name: row2,
    },
    Gpio21 {
        name: col0,
    },
    Gpio22 {
        name: col1,
    },
    Gpio23 {
        name: col2,
    },
    Gpio24 {
        name: col3,
    }

);