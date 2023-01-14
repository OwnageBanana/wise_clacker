# Wise (Cl|H)acker

a simple 85 key split keyboard based on CY8C9540 io expanders with WS2812 LEDs.

**this repo is a really big mess and still a work in progress**

outside of a micro-python starter kit for embedded programming this is one of my first learning experiences with pcb design, embedded programming and rust programming, I've almost exclusively been learning rust for embedded.

So this is my excuse for all the messy code and choices that don't really make sense in this project.

I have [dev-logs](https://adammills.dev/projects/2) up on my website (may not always be available, its run off of my PI and my new ISP uses DHCP for non-commercial customers)



## plans for the Keyboard

- USB HID that has NKRO (currently a 6 key report)
- USB media device reports (Media key modifiers)
- macro keys
  - modifiers + key
  - sequence of keys to type out
- layers for different programs
- LED patterns for layers, per key, runtime adjustable

stretch goal:

- writing and reading to flash/EEPROMs for keyboard configurations
  - goal: to avoid compiling code to reconfigure, one could instead send config though serial usb or a menu on the device

## build steps

build with how you build!
... follow the [RP-Hal](https://github.com/rp-rs/rp-hal) documentation to get things up and running.

```
cargo build

cargo build --release

```

you can also run and the built binaries will be transfered to the micro controller when in USB Mass Storage Device mode

```
cargo run

cargo run --release
```

## device setup

some additional notes on the setup of the usb device

```rust

/// USB VIP for a generic keyboard from
/// https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
const VID: u16 = 0x16c0;

/// USB PID for a generic keyboard from
/// https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
const PID: u16 = 0x27db;

```
