#![no_std]
#![no_main]

pub mod key_code;
pub mod keyboard;
pub mod keys;
pub mod mybuf;

use cortex_m::prelude::_embedded_hal_blocking_i2c_WriteRead;
use key_code::*;
use keyboard::*;
// For string formatting.
use core::fmt::Write;
use core::iter::repeat;
// The macro for our start-up function
use cortex_m_rt::entry;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use embedded_hal::digital::v2::InputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use hal::{i2c::Error, pac, Clock, Sio};
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// The macro for marking our interrupt functions
// use hal::pac::interrupt;

// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    image::Image,
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::*,
};
use tinybmp::Bmp;

// use display_interface;
use display_interface_spi;
// The display driver:
use ssd1306::{prelude::*, Ssd1306};

// led drivers
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

// pio drivers used for ws2812
use hal::pio::PIOExt;

// usb drivers
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
};
// multiple usb classes
use usbd_hid::{
    descriptor::{KeyboardReport, MediaKey, MediaKeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};
use usbd_serial::SerialPort;

use crate::keys::Key;
#[derive(PartialEq)]
#[repr(u32)]
enum bongo {
    Up,
    Left,
    Right,
    Both,
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core: cortex_m::Peripherals = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::sio::Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(sys_freq)
    });
    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // Set the pins up according to their function on this particular board
    let pins = hal::gpio::pin::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    #[rustfmt::skip]
    let keys : [Key;12] = [
        Key::new_key(KeyCode::U,0), Key::new_key(KeyCode::I,0),Key::new_key(KeyCode::O,0),Key::new_key(KeyCode::P,0),
        Key::new_key(KeyCode::H,0), Key::new_key(KeyCode::J,0),Key::new_key(KeyCode::K,0),Key::new_key(KeyCode::L,0),
        Key::new_key(KeyCode::M,0), Key::new_key(KeyCode::N,0),Key::new_key(KeyCode::Comma,0),Key::new_key(KeyCode::Dot,0),
        // Key::new_key(KeyCode::M,0), Key::new_key(KeyCode::N,0),Key::new_key(KeyCode::O,0),Key::new_key(KeyCode::P,0),
        ];

    let layers = [
        Layer::new(b"KiCad", keys),
        Layer::new(b"FreeCad", keys),
        Layer::new(b"Vim", keys),
        Layer::new(b"VS Code", keys),
    ];
    #[rustfmt::skip]
    let keyboard_pins = [
        p(pins.gpio27),p(pins.gpio14),p(pins.gpio8),p(pins.gpio2),
        p(pins.gpio26),p(pins.gpio13),p(pins.gpio7),p(pins.gpio1),
        p(pins.gpio15),p(pins.gpio12),p(pins.gpio6),p(pins.gpio0),
    ];
    let mut k_board = keyboard::Keyboard::new(
        hal::Timer::new(unsafe { pac::Peripherals::steal() }.TIMER, &mut pac.RESETS),
        keyboard_pins,
        layers,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    // Set up the USB HID Class Device driver, providing Keyboard Reports
    let mut keyboard_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 10);
    // let mut media_hid = HIDClass::new(&usb_bus, KeyboardReport::desc(), 100);

    // Create a USB device with a fake VID and PID
    // Set up the USB Communications Class Device driver
    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27da))
        .manufacturer("Adam Mills")
        .product("(Cl|H)ack Pad")
        .serial_number("(Cl|H)ack Pad")
        .device_class(0x00)
        .composite_with_iads()
        .build();

    // Configure the addressable LEDs
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio3.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    let mut wheel_pos: u8 = 128;

    // rotary encoder pins
    let r_btn = pins.gpio20.into_pull_up_input();
    let r_cw = pins.gpio22.into_pull_up_input();
    let r_cc = pins.gpio21.into_pull_up_input();

    // counts of presses
    let mut _btn = 0;
    let mut _cc = 0;
    let mut _cw = 0;

    // booleans for detecting presses
    let mut _test_trigger: bool = false;
    let mut btn_trigger: bool = false;
    let mut cc_trigger: bool = false;
    let mut cw_trigger: bool = false;
    let mut rotary_turning: bool = false;
    let mut rotary_counted: bool = false;
    // true == cw  false == cc
    let mut dir: bool = false;

    let mut buf = mybuf::FmtBuf::new();
    let mut buf_write = mybuf::FmtBuf::new();
    // let mut debug = FmtBuf::new();

    let mut btn_toggle: bool = false;
    let mut said_hello: bool = false;
    let mut usec = timer.get_counter();

    let hid_poll_rate_usec = 10000; //10 ms
    let mut scan_counter = 0;
    let mut prior_scan_val = 0;

    // * init led
    let colours = repeat(wheel(wheel_pos)).take(12);
    let final_rgb = brightness(colours, 100);
    ws.write(final_rgb).unwrap();
    loop {
        if r_btn.is_low().unwrap() {
            hal::rom_data::reset_to_usb_boot(0x1 << 13, 0x0);
        }
        // reset display text buffer
        buf.reset();
        buf_write.reset();

        // rotary updating
        if !btn_trigger && r_btn.is_low().unwrap() {
            btn_trigger = true;
            // btn += 1;
            btn_toggle = !btn_toggle;
        } else if !r_btn.is_low().unwrap() {
            btn_trigger = false;
        }

        /* rotary encoder logic */
        if !cw_trigger && r_cw.is_low().unwrap() {
            cw_trigger = true;
            // if this is first signal of the rotary encoder set direction
            if !rotary_turning {
                dir = true;
            }
            rotary_turning = true;
        } else if !r_cw.is_low().unwrap() {
            cw_trigger = false;
        }

        if !cc_trigger && r_cc.is_low().unwrap() {
            cc_trigger = true;
            // if this is first signal of the rotary encoder set direction
            if !rotary_turning {
                dir = false;
            }
            rotary_turning = true;
        } else if !r_cc.is_low().unwrap() {
            cc_trigger = false;
        }

        // if the turning hasn't been counted yet, and both signals are low (ie triggered) we count
        if !rotary_counted && cc_trigger && cw_trigger {
            rotary_counted = true;
            // true == cw  false == cc
            // match dir {
            //     true => cw += 1,
            //     false => cc += 1,
            // }
            match dir {
                true => wheel_pos = wheel_pos.wrapping_add(1),
                false => wheel_pos = wheel_pos.wrapping_sub(1),
            }
        // if both signals are high (ie not triggered) we may reset
        } else if rotary_turning && !cc_trigger && !cw_trigger {
            rotary_turning = false;
            rotary_counted = false;
            dir = false;
        }

        // led rainbow control
        // let colours = repeat(wheel(wheel_pos)).take(12);
        // let final_rgb = brightness(colours, 100);
        // ws.write(final_rgb).unwrap();
        if btn_toggle {
            wheel_pos = wheel_pos.wrapping_add(1);
        }

        // * usb testing
        buf_write.reset();

        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            write!(buf_write, "(Cl|H)ack Pad connected!\n\n");
            let _ = serial.write(buf_write.as_bytes());
            buf_write.reset();
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial, &mut keyboard_hid /*  &mut media_hid */]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                // WouldBlock - No bytes available for reading.
                Err(usb_device::UsbError::WouldBlock) => {}
                Ok(0) => {}
                Err(e) => {
                    // write!(&mut buf_write, "{:?} \n\r", e);
                    // let mut wr_ptr = buf_write.as_bytes();
                    // while !wr_ptr.is_empty() {
                    //     match serial.write(wr_ptr) {
                    //         Ok(len) => wr_ptr = &wr_ptr[len..],
                    //         // On error, just drop unwritten data.
                    //         // One possible error is Err(WouldBlock), meaning the USB
                    //         // write buffer is full.
                    //         Err(_) => break,
                    //     };
                    //     // serial.write(&buf_write.as_bytes()).unwrap();
                    // }
                    // serial.flush();
                    // buf_write.reset();
                }
                Ok(count) => {
                    // // Convert to upper case
                    // buf.iter_mut().take(count).for_each(|b| {
                    //     b.make_ascii_uppercase();
                    // });
                    // // Send back to the host
                    // let mut wr_ptr = &buf[..count];
                    // while !wr_ptr.is_empty() {
                    //     match serial.write(wr_ptr) {
                    //         Ok(len) => wr_ptr = &wr_ptr[len..],
                    //         Err(_) => break,
                    //     };
                    // }
                }
            }
            // * usb testing

            // HID actions
            {
                let mut report = KeyboardReport {
                    modifier: 0,
                    reserved: 0,
                    leds: 0,
                    keycodes: [0, 0, 0, 0, 0, 0],
                };
                let mut i: usize = 0;
                let mut rep_index: usize = 0;
                let mut bongo_change = false;
                // * keyboard pins scanning
                let reports = k_board.read_layer();
                for r in reports {
                    match r {
                        keys::PressedReport::Toggled(m, c) => {
                            if m != KeyCode::No.byte() {
                                report.modifier = report.modifier | 1 << m - KeyCode::LCtrl.byte();
                            }
                            report.keycodes[rep_index] = c;
                            rep_index += 1;
                            bongo_change = true;
                        }
                        keys::PressedReport::Held(m, c) => {
                            if m != KeyCode::No.byte() {
                                report.modifier = report.modifier | 1 << m - KeyCode::LCtrl.byte();
                            }
                            report.keycodes[rep_index] = c;
                            rep_index += 1;
                        }
                        keys::PressedReport::None => break,
                    }
                }
                if bongo_change && sio.fifo.is_write_ready() {
                    write!(&mut buf_write, "{} UPDATE_BONGO\n\r", scan_counter);
                    serial.write(buf_write.as_bytes());
                    buf_write.reset();
                    sio.fifo.write(UPDATE_BONGO);
                }
                // * keyboard reporting
                scan_counter += 1;
                if hid_poll_rate_usec + usec < timer.get_counter() {
                    if scan_counter != prior_scan_val {
                        // writing every hid_poll_rate_usec which is 10ms
                        // write!(
                        //     &mut buf_write,
                        //     "scancount: {}\t SPS: {}\n\r",
                        //     scan_counter,
                        //     scan_counter * 100
                        // );
                        // serial.write(buf_write.as_bytes());
                        // buf_write.reset();
                    }
                    prior_scan_val = scan_counter;
                    scan_counter = 0;

                    usec = timer.get_counter();
                    match keyboard_hid.push_input(&report) {
                        Ok(s) => {}
                        Err(e) => {}
                    };
                }
                buf_write.reset();
            }
        }
    }
}

// Convert a number from `0..=255` to an RGB color triplet.
//
// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
fn w(mut wheel_pos: u8) -> RGB8 {
    wheel(wheel_pos)
}

use hal::multicore::{Multicore, Stack};
static mut CORE1_STACK: Stack<4096> = Stack::new();

const UPDATE_BONGO: u32 = 0xEE;

fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _sck = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _tx = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    let dc = pins.gpio16.into_push_pull_output();
    let cs = pins.gpio17.into_push_pull_output();

    use fugit::RateExtU32;
    // Create an SPI driver instance for the SPI0 device
    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        125_000_000u32.Hz(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );
    let interface = display_interface_spi::SPIInterface::new(spi, dc, cs);
    // Create a driver instance and initialize:
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Create a text style for drawing the font:
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let display_rect = embedded_graphics::primitives::rectangle::Rectangle::new(
        Point::new(1, 1),
        Size::new(126, 62),
    )
    .into_styled(
        PrimitiveStyleBuilder::new()
            .stroke_width(2)
            .stroke_color(BinaryColor::On)
            .build(),
    );

    // init display properties
    let up_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_up_inverted.bmp")).unwrap();
    let left_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_left_inverted.bmp"))
            .unwrap();
    let right_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_right_inverted.bmp"))
            .unwrap();
    let both_bmp =
        Bmp::<BinaryColor>::from_slice(include_bytes!("../assets/bongo_both_inverted.bmp"))
            .unwrap();

    let mut bongo_state: bongo = bongo::Up;
    let mut bongo_prev: bongo = bongo::Left;

    let bongo_up = Image::new(&up_bmp, Point::zero());
    let bongo_left = Image::new(&left_bmp, Point::zero());
    let bongo_right = Image::new(&right_bmp, Point::zero());
    let bongo_both = Image::new(&both_bmp, Point::zero());

    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);
    bongo_up.draw(&mut display).unwrap();
    display_rect.draw(&mut display).unwrap();
    display.flush();

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut usec = timer.get_counter();
    let hold_time = 100000; //100 ms
    let mut dusec = timer.get_counter();
    let debounce = 50000; //50 ms
    let mut hold = false;
    loop {
        let mut update_bongo = false;
        let input = sio.fifo.read();

        match input {
            Some(UPDATE_BONGO) => {
                // set the debounce to ignore any errant inputs. if we are already debouncing then eat input
                if debounce + dusec < timer.get_counter() {
                    dusec = timer.get_counter();
                } else {
                    continue;
                }
                if bongo_state == bongo::Up {
                    if bongo_prev == bongo::Right {
                        bongo_state = bongo::Left;
                        bongo_prev = bongo::Left;
                    } else {
                        bongo_state = bongo::Right;
                        bongo_prev = bongo::Right;
                    }
                } else if bongo_state == bongo::Right {
                    bongo_state = bongo::Left;
                } else if bongo_state == bongo::Left {
                    bongo_state = bongo::Right;
                }
                update_bongo = true;
                hold = true;
                usec = timer.get_counter();
            }
            Some(word) => {}
            None => {}
        }
        // hold keeps the animation frame in until timer is up
        if hold {
            if hold_time + usec < timer.get_counter() {
                // usec = timer.get_counter();
                bongo_state = bongo::Up;
                hold = false;
                update_bongo = true;
            }
            if update_bongo {
                usec = timer.get_counter();
                display.clear();
                match bongo_state {
                    bongo::Up => bongo_up.draw(&mut display).unwrap(),
                    bongo::Left => bongo_left.draw(&mut display).unwrap(),
                    bongo::Right => bongo_right.draw(&mut display).unwrap(),
                    bongo::Both => bongo_both.draw(&mut display).unwrap(),
                }
                display_rect.draw(&mut display).unwrap();
                display.flush().unwrap();
            }
        }
    }
}

// debug.reset();
// Empty the display:
// display.clear();

//* if (left.is_low().unwrap() && right.is_low().unwrap()) || center.is_low().unwrap() {
//*     if position != bongo::Both {
//*         position = bongo::Both;
//*         bongo_both.draw(&mut display).unwrap();
//*     }
//* } else if left.is_low().unwrap() {
//*     if position != bongo::Left {
//*         position = bongo::Left;
//*         bongo_left.draw(&mut display).unwrap();
//*     }
//* } else if right.is_low().unwrap() {
//*     if position != bongo::Right {
//*         position = bongo::Right;
//*         bongo_right.draw(&mut display).unwrap();
//*     }
//* } else {
//*     if position != bongo::Up {
//*         position = bongo::Up;
//*         bongo_up.draw(&mut display).unwrap();
//*     }
//* }
// //* rotary checks
//* } else if r_cc.is_low().unwrap() && r_cw.is_low().unwrap() {
//*     bongo_both.draw(&mut display).unwrap();
//* } else if r_cc.is_low().unwrap() {
//*     bongo_left.draw(&mut display).unwrap();
//* } else if r_cw.is_low().unwrap() {
//*     bongo_right.draw(&mut display).unwrap();
//* } else {
//*     bongo_up.draw(&mut display).unwrap();
//* }
// Display the image
//* display_rect.draw(&mut display).unwrap();
//* display.flush();
