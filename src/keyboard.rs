use core::result;
use core::slice::Iter;

use crate::key_code::KeyCode;
use crate::keys::{Key, Pressable, PressedReport};
use embedded_hal::digital::v2::InputPin;

use embedded_time::timer::param::None;
use hal::gpio::Pin;
use hal::gpio::{AnyPin, DynPin, PinId, PinMode, PullUpInput, ValidPinMode};
use rp2040_hal as hal;
use smart_leds::{brightness, Brightness, SmartLedsWrite, RGB, RGB8};
use ws2812_pio::Ws2812;

// brightness value for leds. Max of 255
pub const BRIGHTNESS: u8 = 100;

#[rustfmt::skip]
pub const REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x95, 0x08,        //   Report Count (8)
    0x75, 0x01,        //   Report Size (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0xFF,        //   Usage Maximum (0xFF)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (Num Lock)
    0x29, 0x05,        //   Usage Maximum (Kana)
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x03,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection
];

// usb_report: REPORT_DESCRIPTOR,

pub struct Encoder {
    pub btn_code: u8,
    pub cw_code: u8,
    pub cc_code: u8,
    pub mode: bool, // toggle for rotating led color or regular twisting mode to provide hid codes
}
// enum EncoderMode {
//     key
// }
impl Encoder {
    fn new() -> Self {
        Encoder {
            btn_code: todo!(),
            cw_code: todo!(),
            cc_code: todo!(),
            mode: todo!(),
        }
    }
}
pub struct Keyboard {
    // pub r_encoder: Encoder,
    pub layers: [Layer; 4], // colors: [u8; 12],
    pub layer_sel: usize,
    pub layer_count: usize,
    pub pins: [DynPin; 12],
    pub pc: hal::Timer,
}

// sets up pins for keyboard array
pub fn p<I, M>(pin: Pin<I, M>) -> DynPin
where
    I: PinId,
    M: PinMode + ValidPinMode<I>,
{
    let mut d = DynPin::from(pin);
    d.into_pull_up_input();
    return d;
}

// eventually I'll want some sort of update function to examine the pins for state changes, (using key.togggled) and push that change to the key board report. maybe the keyboard should hold a report and update it as pins change.
impl Keyboard {
    pub fn new(pc: hal::Timer, pins: [hal::gpio::DynPin; 12], layers: [Layer; 4]) -> Self {
        Keyboard {
            // r_encoder: Encoder::new(),
            layers: layers,
            layer_sel: 0,
            layer_count: 0,
            pins,
            pc,
        }
    }

    pub fn add_layer(&mut self, layer: Layer) -> bool {
        if self.layer_count == self.layers.len() {
            return false;
        }
        self.layers[self.layer_sel] = layer;
        self.layer_count += 1;
        return true;
    }
    pub fn rotate_layer(&mut self) {
        self.layer_sel += 1;
        if self.layer_sel > 3 {
            self.layer_sel = 0
        }
    }
    // pub fn layer_led(&self) -> Brightness<Iter<RGB<u8>>> {
    //     return brightness(self.layers[self.layer_sel].led_colors(), BRIGHTNESS);
    // }
    // pub fn get_active_pins(&self) -> vec<usize> {
    //     let v: vec<usize>;
    //     let i: usize = 0;
    //     for pin in self.pins {
    //         if pin.is_low().unwrap() {
    //             v.push(i.clone());
    //         }
    //         i += 1;
    //     }
    //     return v;
    // }
    pub fn curr_layer(&self) -> &Layer {
        &self.layers[self.layer_sel]
    }
    pub fn read_layer(&mut self) -> [PressedReport; 6] {
        let mut reports: [PressedReport; 6] = [PressedReport::None; 6];
        let mut curr: usize = 0;
        for i in 0..12 {
            if curr == 6 {
                break;
            }
            self.layers[self.layer_sel].keys[i].read(&self.pins[i]);
            match self.layers[self.layer_sel].keys[i].press_report() {
                PressedReport::Toggled(m, c) => {
                    reports[curr] = PressedReport::Toggled(m, c);
                    curr += 1;
                }
                PressedReport::Held(m, c) => {
                    reports[curr] = PressedReport::Held(m, c);
                    curr += 1;
                }
                PressedReport::None => {}
            }
        }
        return reports;
    }
}

pub struct Layer {
    pub name: [u8; 32],
    pub keys: [Key; 12],
}

impl Layer {
    pub fn new(name: &[u8], keys: [Key; 12]) -> Self {
        let mut new = Layer {
            name: [0; 32],
            keys: keys,
        };
        let range = if 32 < name.len() { 32 } else { name.len() };
        for n in 0..range {
            new.name[n] = name[n];
        }
        return new;
    }
    // pub fn led_colors(self) -> Iter<RGB<u8>> {
    //     return self.keys.map( |k| k.color.clone()).iter();
    // }
}
