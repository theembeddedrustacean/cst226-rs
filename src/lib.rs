//! # CST226 Touch Controller Driver Crate
//!
//! A `no_std` driver for the CST226 touch controller, providing functionality to read touch points and manage power modes.
//!
//! This driver is based on publicly available C++ drivers, as official datasheets for the CST226 are not widely available.
//! It is `embedded-hal` compatible and provides a generic interface for the device's hardware reset functionality.
//! This allows the reset pin to be controlled by a direct GPIO pin or an I2C I/O expander.
//!
//! This driver reads the entire touch data block in a single I2C transaction to report all active touch points simultaneously.
//!
//! Some drivers include an INT pin indicating touch events, but this driver does not use it. If interrupts need to be supported, the user can attach the pin output to an interrupt and implement their own callback.
//!
//! ## Usage
//!
//! 1. Implement the `ResetInterface` trait for your specific reset mechanism.
//! 2. Create an instance of the `Cst226Driver`.
//! 3. Initialize the touch driver.
//! 4. In a loop, use the `get_touches()` method to read the current touch state.
//!
//! ```rust
//! // This is a conceptual example. `I2CInstance`, `PinInstance`, and `DelayInstance`
//! // would be your concrete implementations from a HAL crate.
//!
//! // 1. Implement the ResetInterface for your hardware.
//! struct GpioReset<P: OutputPin> {
//!     pin: P,
//! }
//!
//! impl<P: OutputPin> ResetInterface for GpioReset<P> {
//!     type Error = P::Error;
//!     fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Self::Error> {
//!         self.pin.set_high()?;
//!         delay.delay_ms(5);
//!         self.pin.set_low()?;
//!         delay.delay_ms(5);
//!         self.pin.set_high()?;
//!         delay.delay_ms(30);
//!         Ok(())
//!     }
//! }
//!
//! // 2. Create driver instance.
//! let reset_pin = GpioReset { pin: PinInstance };
//! let mut touch = Cst226Driver::new(
//!     I2CInstance,
//!     CST226_DEVICE_ADDRESS,
//!     reset_pin,
//! );
//!
//! // 3. Initialize the driver.
//! touch.initialize(&mut DelayInstance).expect("Failed to initialize touch driver");
//!
//! // 4. Read touches in a loop.
//! loop {
//!     let touches = touch.get_touches().unwrap();
//!
//!     for (i, touch) in touches.iter().enumerate() {
//!         println!("Touch {}: x={}, y={}", i + 1, touch.x, touch.y);
//!     }
//! }
//! ```
//!
//! Notes:
//! - If the reset pin is controlled via an I2C GPIO expander sharing the same bus with the touch driver, you should use a shared bus implementation like `embedded_hal_bus` to manage I2C access.

#![no_std]
use core::prelude::v1;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use heapless::Vec;

// Constants for CST226 device address and registers
pub const CST226_DEVICE_ADDRESS: u8 = 0x5A;
const CST226_REG_STATUS: u8 = 0x00;
const CST226_BUFFER_SIZE: usize = 28;
const MAX_TOUCH_POINTS: usize = 5;

// Swipe detection threshold
const SWIPE_MIN_DISTANCE: i32 = 50; // pixels

/// Power modes for the touch device.
#[derive(Clone, Copy, Debug)]
pub enum PowerMode {
    /// The device is actively scanning for touches.
    Active,
    /// The device is in a low-power sleep state. Wake-up requires a hardware reset.
    Sleep,
}

/// Represents a single touch point with X and Y coordinates.
#[derive(Debug, Default, Clone, Copy)]
pub struct TouchPoint {
    pub x: u16,
    pub y: u16,
}

/// Single-finger swipe gestures.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gesture {
    None,
    SwipeUp,
    SwipeDown,
    SwipeLeft,
    SwipeRight,
}

/// Internal state for gesture detection.
#[derive(Default)]
struct GestureState {
    is_touching: bool,
    start_point: TouchPoint,
    last_point: TouchPoint,
}

/// Trait for controlling the CST226 hardware reset pin.
pub trait ResetInterface {
    /// The specific error type for this reset implementation.
    type Error;

    /// Performs the hardware reset sequence for the CST226 device.
    /// This could be a GPIO port or an I2C expander-controlled pin.
    /// Recommended Implementation: HIGH -> delay -> LOW -> delay -> HIGH -> delay.
    fn reset(&mut self) -> Result<(), Self::Error>;
}

/// Driver Errors
#[derive(Debug)]
pub enum DriverError<ResetError, I2cError> {
    /// Error originating from the I2C bus.
    I2cError(I2cError),
    /// Error originating from the reset pin control.
    ResetError(ResetError),
}

/// CST226 touch driver
pub struct Cst226Driver<I2C, D, RST> {
    i2c: I2C,
    device_address: u8,
    delay: D,
    reset: RST,
    gesture_state: GestureState,
}

impl<I2C, D, RST> Cst226Driver<I2C, D, RST>
where
    I2C: I2c,
    D: DelayNs,
    RST: ResetInterface,
{
    /// Creates a new instance of the CST226 driver.
    pub fn new(i2c: I2C, device_address: u8, reset: RST, delay: D) -> Self {
        Cst226Driver {
            i2c,
            device_address,
            delay,
            reset,
            gesture_state: GestureState::default(),
        }
    }

    /// Initializes the device.
    /// This method performs a hardware reset to ensure the chip is in a known state.
    pub fn initialize(&mut self) -> Result<(), DriverError<RST::Error, I2C::Error>> {
        self.reset.reset().map_err(DriverError::ResetError)?;
        self.delay.delay_ms(10);
        self.gesture_state = GestureState::default();
        Ok(())
    }

    /// Sets the power mode of the device.
    /// To wake the device from `Sleep` mode, a hardware reset is required by calling `initialize()`.
    pub fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), I2C::Error> {
        match mode {
            PowerMode::Sleep => self.i2c.write(self.device_address, &[0xD1, 0x05]),
            PowerMode::Active => {
                // Waking up requires a hardware reset, which is handled by initialize().
                // We do nothing here, user must call initialize().
                Ok(())
            }
        }
    }

    /// Reads all active touch points' state in a single transaction.
    ///
    /// Returns a `Vec` containing `TouchPoint`s (up to 5).
    pub fn get_touches(&mut self) -> Result<Vec<TouchPoint, MAX_TOUCH_POINTS>, I2C::Error> {
        let mut buffer = [0u8; CST226_BUFFER_SIZE];
        self.i2c
            .write_read(self.device_address, &[CST226_REG_STATUS], &mut buffer)?;

        // Check for invalid data markers
        if buffer[6] != 0xAB || buffer[0] == 0xAB || buffer[5] == 0x80 {
            return Ok(Vec::new());
        }

        let num_touches = (buffer[5] & 0x7F) as usize;

        if num_touches == 0 {
            return Ok(Vec::new());
        }

        // If touch count is invalid, clear the status register and return
        if num_touches > MAX_TOUCH_POINTS {
            self.i2c.write(self.device_address, &[0x00, 0xAB])?;
            return Ok(Vec::new());
        }

        let mut touches = Vec::new();
        let mut index: usize = 0;

        for i in 0..num_touches {
            if index + 4 >= CST226_BUFFER_SIZE {
                break; // Avoid buffer overflow
            }

            let x = ((buffer[index + 1] as u16) << 4) | (((buffer[index + 3] >> 4) & 0x0F) as u16);
            let y = ((buffer[index + 2] as u16) << 4) | ((buffer[index + 3] & 0x0F) as u16);

            if touches.push(TouchPoint { x, y }).is_err() {
                break; // Stop if Vec is full
            }

            // The data format for the CST226 is unusual. The first touch point block
            // is 7 bytes long, while subsequent blocks are 5 bytes.
            index += if i == 0 { 7 } else { 5 };
        }

        Ok(touches)
    }

    /// Detects single-finger swipe gestures based on touch history.
    ///
    /// Call this method periodically with the current time in milliseconds.
    /// Returns `Gesture::None` if no gesture is detected or if multiple fingers are used.
    pub fn get_gesture(&mut self) -> Result<Gesture, I2C::Error> {
        let touches = self.get_touches()?;

        let state = &mut self.gesture_state;

        if touches.len() > 1 {
            // Only support single finger
            state.is_touching = false;
            return Ok(Gesture::None);
        }

        if touches.is_empty() {
            // Touch up
            if state.is_touching {
                state.is_touching = false;
                let dx = state.last_point.x as i32 - state.start_point.x as i32;
                let dy = state.last_point.y as i32 - state.start_point.y as i32;

                // Swipe detection
                if dx.abs() > dy.abs() && dx.abs() > SWIPE_MIN_DISTANCE {
                    return Ok(if dx > 0 {
                        Gesture::SwipeRight
                    } else {
                        Gesture::SwipeLeft
                    });
                } else if dy.abs() > dx.abs() && dy.abs() > SWIPE_MIN_DISTANCE {
                    return Ok(if dy > 0 {
                        Gesture::SwipeDown
                    } else {
                        Gesture::SwipeUp
                    });
                }
            }
            return Ok(Gesture::None);
        } else {
            // Touch down or move
            let point = touches[0];
            if !state.is_touching {
                state.is_touching = true;
                state.start_point = point;
                state.last_point = point;
            } else {
                state.last_point = point;
            }
            return Ok(Gesture::None);
        }
    }
}
