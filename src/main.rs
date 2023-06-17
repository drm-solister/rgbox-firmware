//! # Pico USB Serial (with Interrupts) Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the USB interrupt.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]
use rp2040_hal::gpio::DynPin;

// The macro for our start-up function
use rgbox_bsp::entry;

// The macro for marking our interrupt functions
use rgbox_bsp::hal::pac::interrupt;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rgbox_bsp::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rgbox_bsp::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rgbox_bsp::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Import useful traits to handle the ws2812 LEDs:
use smart_leds::{brightness, SmartLedsWrite, RGB8};

// Import the actual crate to handle the Ws2812 protocol:
use ws2812_pio::Ws2812;

use rgbox_bsp::hal::pio::PIOExt;

use rgbox_bsp::hal::timer::Timer;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

use hal::multicore::{Multicore, Stack};
static mut CORE1_STACK: Stack<4096> = Stack::new();

use embedded_hal::digital::v2::InputPin;

// USB device support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rgbox_bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let usb_hid = HIDClass::new(bus_ref, &RGBOX_REPORT_DESC, 1);
    unsafe {
        USB_HID = Some(usb_hid);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x2e8a, 0x000a))
        .manufacturer("Fake company")
        .product("RGBox")
        .serial_number("001")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // No more USB code after this point in main! We can do anything we want in
    // here since USB is handled in the interrupt - let's blink an LED!

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rgbox_bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    // let mut col0 = pins.col0.into_push_pull_output();



    let mut leds: [RGB8; 40] = [(255, 0, 0).into(); 40];
    let mut center = 0;

    let sin = hal::rom_data::float_funcs::fsin::ptr();

    // *leds.get_mut(center as usize).unwrap() = (5,0,0).into();
    // leds[0] = (255, 0, 0).into();

    let mut columns: [DynPin; 4] = [pins.col0.into_push_pull_output().into(), pins.col1.into_push_pull_output().into(), pins.col2.into_push_pull_output().into(), pins.col3.into_push_pull_output().into()];
    let rows: [DynPin; 3] = [pins.row0.into_pull_down_input().into(), pins.row1.into_pull_down_input().into(), pins.row2.into_pull_down_input().into()];
    let sp_buttons: [DynPin; 6] = [
        pins.sp1.into_pull_up_input().into(), pins.sp2.into_pull_up_input().into(), pins.sp3.into_pull_up_input().into(),
        pins.sp4.into_pull_up_input().into(), pins.sp5.into_pull_up_input().into(), pins.sp6.into_pull_up_input().into(),
    ];

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {

            // Create a count down timer for the Ws2812 instance:
            let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

            // Split the PIO state machine 0 into individual objects, so that
            // Ws2812 can use it:
            let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

            // Instanciate a Ws2812 LED strip:
            let mut ws = Ws2812::new(
                // Use pin 6 on the Raspberry Pi Pico (which is GPIO4 of the rp2040 chip)
                // for the LED data output:
                pins.argb.into_mode(),
                &mut pio,
                sm0,
                clocks.peripheral_clock.freq(),
                timer.count_down(),
            );

            // // Get the second core's copy of the `CorePeripherals`, which are per-core.
            // // Unfortunately, `cortex-m` doesn't support this properly right now,
            // // so we have to use `steal`.
            let core = unsafe { pac::CorePeripherals::steal() };
            // // Set up the delay for the second core.
            let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
            // // Blink the second LED.
            // loop {
            //     led2.toggle().unwrap();
            //     delay.delay_us(CORE1_DELAY)
            // }

            loop {
                ws.write(brightness(leds.iter().copied(), (8.0*(sin(center as f32*0.1))+1.0) as u8)).unwrap();
        
                delay.delay_ms(60);
                center+=1;
                
                center = center % 40;
            }
        })
        .unwrap();

    /*
                    [[L, D, R, U],
                     [punches   ],
                     [kicks     ]];
    */
    // note that 1<<x corresponds to button (9+x)%16 for x<=16
    // button 1 starts with x = 8
    // let button_map = [[1, 1, 1, 1],
    //                   [8, 11, 13, 12],
    //                   [9, 10, 15, 14]]; // doesnt work for some reason

    let button_map = [[1, 1, 1, 1],
                      [10, 11, 13, 12],
                      [8, 9, 15, 14]];

    let sp_map = [1, 0, 4, 2, 3, 5];

    let mut counter = 0;
    let one: u16 =   0b0000000100000000;
    let two: u16 =   0b0000001000000000;
    let three: u16 = 0b0000010000000000;
    let four: u16 =  0b0000100000000000;
    let five: u16 =  0b0001000000000000;
    let numbers: [u16; 4] = [one, two, three, four];
    let hat_bindings: [u8; 9] = [6,7,8,5,0,1,4,3,2];
    let mut hat_inputs: [bool; 4] = [false, false, false, false]; // left, down, right, up

    // Blink the LED at 1 Hz
    loop {
        hat_inputs = [false, false, false, false];

        let mut button_states: [[bool; 4]; 4] = [[false; 4]; 4];

        let mut report = RgboxReport {
            idk: [0,0,0,0,0,0],
            bitmap: [0, 0],
            hat: 0
        };

        let mut bitmap_bits = 0u16;

        for (c, column) in columns.iter_mut().enumerate() {
            column.set_high();
            delay.delay_us(1);
            for (r, row) in rows.iter().enumerate() {
                if row.is_high().unwrap() {
                    if r == 0 { // directional buttons
                        hat_inputs[c] = true;
                    } else {
                        bitmap_bits |= (1u16 << button_map[r][c]);
                    }
                }
            }
            column.set_low();
        }

        // could avoid branching by setting hat_inputs to numbers and adding tham all into hat_index
        let mut hat_index: usize = 4;
        if hat_inputs[3] {
            hat_index += 1;
        } else if hat_inputs[1] { // implements SOCD
            hat_index -= 1;
        }
        if hat_inputs[0] {
            hat_index -= 3;
        }
        if hat_inputs[2] {
            hat_index += 3;
        }
        report.hat = hat_bindings[hat_index];

        for (i, sp_button) in sp_buttons.iter().enumerate() {
            if sp_button.is_low().unwrap() {
                bitmap_bits |= (1u16 << sp_map[i]);
            }
        }

        report.bitmap = bitmap_bits.to_be_bytes();


        push_mouse_movement(report).ok();
        
        
        delay.delay_ms(1);
    }
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
// #[allow(non_snake_case)]
// #[interrupt]
// unsafe fn USBCTRL_IRQ() {
//     use core::sync::atomic::{AtomicBool, Ordering};

//     /// Note whether we've already printed the "hello" message.
//     static SAID_HELLO: AtomicBool = AtomicBool::new(false);

//     // Grab the global objects. This is OK as we only access them under interrupt.
//     let usb_dev = USB_DEVICE.as_mut().unwrap();
//     let serial = USB_SERIAL.as_mut().unwrap();

//     // Say hello exactly once on start-up
//     if !SAID_HELLO.load(Ordering::Relaxed) {
//         SAID_HELLO.store(true, Ordering::Relaxed);
//         let _ = serial.write(b"Hello, World!\r\n");
//     }

//     // Poll the USB driver with all of our supported USB Classes
//     if usb_dev.poll(&mut [serial]) {
//         let mut buf = [0u8; 64];
//         match serial.read(&mut buf) {
//             Err(_e) => {
//                 // Do nothing
//             }
//             Ok(0) => {
//                 // Do nothing
//             }
//             Ok(count) => {
//                 // Convert to upper case
//                 buf.iter_mut().take(count).for_each(|b| {
//                     b.make_ascii_uppercase();
//                 });

//                 // Send back to the host
//                 let mut wr_ptr = &buf[..count];
//                 while !wr_ptr.is_empty() {
//                     let _ = serial.write(wr_ptr).map(|len| {
//                         wr_ptr = &wr_ptr[len..];
//                     });
//                 }
//             }
//         }
//     }
// }

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_mouse_movement(report: RgboxReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = GAMEPAD) = {

        (usage_page = BUTTON, usage_min = 1, usage_max = 16) = {
            #[packed_bits 16] #[item_settings data,variable,absolute] bitmap=input;
        };
        (usage_page = GENERIC_DESKTOP, usage = 0x39) = { // 0x39 is hat switch
            #[item_settings data,variable,absolute] hat=input;
        };

    }
)]
struct RgboxReport {
    pub idk: [u8; 6],
    pub bitmap: [u8; 2],
    pub hat: u8,
}

// const RGBOX_REPORT_DESC: [u8; 38] = [
//     0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
//     0x09, 0x05,        // Usage (Game Pad)
//     0xA1, 0x01,        // Collection (Application)
//     0x05, 0x09,        //   Usage Page (Button)
//     0x19, 0x01,        //   Usage Minimum (0x01)
//     0x29, 0x10,        //   Usage Maximum (0x10)
//     0x15, 0x00,        //   Logical Minimum (0)
//     0x25, 0x01,        //   Logical Maximum (1)
//     0x75, 0x01,        //   Report Size (1)
//     0x95, 0x10,        //   Report Count (16)
//     0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
//     0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
//     0x09, 0x39,        //   Usage (Hat switch)
//     0x15, 0x01,        //   Logical Minimum (1)
//     0x26, 0x08, 0x00,  //   Logical Maximum (255)
//     0x75, 0x08,        //   Report Size (8)
//     0x95, 0x01,        //   Report Count (1)
//     0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
//     0xC0,              // End Collection
// ];

const RGBOX_REPORT_DESC: [u8; 66] = [
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x81,        //   Logical Minimum (-127)
    0x25, 0x7F,        //   Logical Maximum (127)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x01,        //   Logical Minimum (1)
    0x25, 0x08,        //   Logical Maximum (8)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x20,        //   Usage Maximum (0x20)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x95, 0x20,        //   Report Count (32)
    0x75, 0x01,        //   Report Size (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
];