// Example for CST226 touch driver, adapted for the LilyGo T4-S3  display hardware.

#![no_std]
#![no_main]

// Replaced ft3x68_rs with the conceptual cst226_rs crate.
// Note: PowerMode and DriverError are not used in this specific example but are kept for completeness.
use cst226_rs::{Cst226Driver, ResetInterface, CST226_DEVICE_ADDRESS};

use esp_alloc as _;
use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    delay::Delay,
    gpio::{Io, Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, Error as I2cError, I2c},
    main,
    time::Rate,
};
use esp_println::{print, println};

esp_app_desc!();

// This reset implementation is specific to the LilyGo T4-S3 display,
// which uses GPIO17 on the ESP32-S3 to control the touch reset pin.
pub struct ResetDriver<OUT> {
    output: OUT,
}

impl<OUT> ResetDriver<OUT> {
    pub fn new(output: OUT) -> Self {
        ResetDriver { output }
    }
}

impl<OUT> ResetInterface for ResetDriver<OUT>
where
    OUT: embedded_hal::digital::OutputPin,
{
    type Error = OUT::Error;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();
        self.output.set_low()?;
        delay.delay_millis(20);
        self.output.set_high()?;
        delay.delay_millis(150);
        Ok(())
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut delay = Delay::new();

    // I2C Configuration for the Waveshare ESP32-S3 1.8inch AMOLED display.
    // Both the touch controller and the I/O expander for reset are on this bus.
    let touch_i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO6)
    .with_scl(peripherals.GPIO7);

    let output = Output::new(peripherals.GPIO17, Level::High, OutputConfig::default());

    // Initialize the reset driver.
    let reset = ResetDriver::new(output);

    println!("Initializing CST226 Touch Driver...");

    // Instantiate the CST226 driver.
    // Note: The CST226 driver's `new` function does not take a delay instance.
    let mut touch = Cst226Driver::new(touch_i2c, CST226_DEVICE_ADDRESS, reset, delay);

    // Initialize the driver. The `initialize` method requires a delay provider.
    touch
        .initialize()
        .expect("Failed to initialize touch driver");

    println!("Touch driver initialized. Reading points...");

    loop {
        // The primary method for the CST226 is `get_touches()`.
        // It reads all points in one transaction.
        match touch.get_touches() {
            Ok(touches) => {
                if !touches.is_empty() {
                    print!("Touches detected: {} | ", touches.len());
                    for (i, point) in touches.iter().enumerate() {
                        print!("P{}: ({}, {}) ", i, point.x, point.y);
                    }
                    println!();
                }
            }
            Err(e) => {
                println!("Error reading touches: {:?}", e);
            }
        }

        delay.delay_millis(100);
    }
}
