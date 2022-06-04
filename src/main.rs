// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]

use panic_halt as _;

use nb::block;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use cortex_m_semihosting::debug;

use embedded_hal::spi::{Mode, Phase, Polarity};
use stm32f1xx_hal::{
    gpio::gpioa,
    gpio::gpiob::PB12,
    gpio::{Output, PushPull},
    pac::{Peripherals, SPI2},
    prelude::*,
    spi::{Pins, Spi, Spi2NoRemap, SpiReadWrite},
    timer::{Delay, SysDelay},
    usb::{Peripheral, UsbBus}
};

use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};


// TODO: Deal with results in spi_write
#[inline]
fn _draw_xy(spi: &mut (
    Spi<SPI2, Spi2NoRemap, impl Pins<Spi2NoRemap>, u16>,
    PB12<Output<PushPull>>), 
            x: u16,  
            y: u16,
            mut delay: SysDelay)
{
    let mut txbuffer: [u16; 2] = [0; 2];
    txbuffer[0] = ((0b0001u16) << 12) | (0x0fffu16 & x);
    txbuffer[1] = ((0b1001u16) << 12) | (0x0fffu16 & y);
    // We ignore LDAC for now, it will be tied low
    // We set CS to low to send the write to the DAC
    spi.1.set_low();
    spi.0.spi_write(&txbuffer[..0]);
    // Minimum time CS must stay high between 2 data points is 15ns
    // 10us to be safe
    spi.1.set_high();
    delay.delay_us(10u16);
    spi.1.set_low();
    spi.0.spi_write(&txbuffer[1..]);
    // Restores the state of the CS pin
    spi.1.set_high();
}

#[entry]
fn main() -> ! {
    // SETUP PHASE

    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIOC peripheral
    let mut gpiob = dp.GPIOB.split();
    let mut gpioa = dp.GPIOA.split();

    // Delay setup
    let systimer = cp.SYST.delay(&clocks);

    // USB Setup
    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: gpioa.pa12.into_floating_input(&mut gpioa.crh)
    };
    // Create the USB Bus from the peripheral
    let usb_bus = UsbBus::new(usb);
    let mut serial = SerialPort::new(&usb_bus);
    // Build the USB device from the bus
    // We will use the serial to send data through USB
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("AVTL")
        .product("Vectr")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();
    
    // Setup SPI output pins
    let pins = (
        gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
        gpiob.pb14.into_floating_input(&mut gpiob.crh),
        gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
    );
    
    // CS is high by default and pulled low when a write is needed
    let mut cs = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    cs.set_high();

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition
    };
    let spi = Spi::spi2(dp.SPI2, pins, spi_mode, 8.MHz(), clocks).frame_size_16bit();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        // TODO: Remove continue! implement dynamic speed drawing!!
    }
}

// TODO: Drawline method!