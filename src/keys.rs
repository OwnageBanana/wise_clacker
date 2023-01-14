use crate::key_code::KeyCode;

use embedded_hal::digital::v2::InputPin;
use rp2040_hal::{
    gpio::DynPin, pac::usbctrl_regs::usbphy_direct_override::DM_PULLUP_HISEL_OVERRIDE_EN_R,
};
// led drivers
use smart_leds::{brightness, SmartLedsWrite, RGB, RGB8};
use ws2812_pio::Ws2812;

use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};

#[derive(Clone, Copy)]
pub enum PressedReport {
    Toggled(u8, u8),
    Held(u8, u8),
    None,
}

pub trait Pressable {
    fn press_report(&self) -> PressedReport;
}

/* pub struct Macro {
    code: u8,
    pin: Pin<T, Input<PullUp>>,
    toggled: bool, //pressed to is_pressed
    color: u8,
}

impl Pressable for Macro {}
 */
#[derive(Clone, Copy)]
pub struct Key {
    pub modifier: KeyCode,
    pub code: KeyCode,
    pub toggled: bool,
    pub color: RGB<u8>,
    pub report: PressedReport,
    pub debounce: u64,
}
impl Key {
    pub fn new(modifier: KeyCode, code: KeyCode, color: u8) -> Self {
        Key {
            toggled: false,
            modifier,
            code,
            color: wheel(color),
            report: PressedReport::None,
            debounce: 0,
        }
    }
    pub fn new_key(code: KeyCode, color: u8) -> Self {
        Key {
            toggled: false,
            modifier: KeyCode::No,
            code,
            color: wheel(color),
            report: PressedReport::None,
            debounce: 0,
        }
    }
    // updates the state of the key based on its last state, and pin information
    pub fn read(&mut self, pin: &DynPin) {
        if self.toggled {
            if pin.is_low().unwrap() {
                self.report = PressedReport::Held(self.modifier.byte(), self.code.byte());
            } else {
                self.report = PressedReport::None;
                self.toggled = false;
            }
        } else if pin.is_low().unwrap() {
            self.report = PressedReport::Toggled(self.modifier.byte(), self.code.byte());
            self.toggled = true;
        } else {
            self.toggled = false;
        }
    }
}
impl Pressable for Key {
    fn press_report(&self) -> PressedReport {
        self.report.clone()
        // if !self.toggled {
        //     return PressedReport::None;
        // }
        // PressedReport::Pressed(self.modifier.byte(), self.code.byte())
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
