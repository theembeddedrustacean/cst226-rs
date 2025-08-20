# CST226 Touch Controller Driver Crate

A driver for the CST226 touch controller(s), providing functionality to read touch points and manage power modes.

> **Note:** There is limited public information available for CST226 touch controller. Much of the operational detail is not documented in datasheets.  
> This driver is largely based on the implementation in the [LilyGo AMOLED Series repository](https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series).

This driver is **`embedded-hal`** compatible and provides a generic interface for implementing the device reset functionality.  
The reset pin can be controlled by custom interface such as a GPIO port or an I2C I/O expander.

Some drivers include an `INT` pin to indicate touch events, but this driver **does not** use it.  
If interrupts are needed, users can connect the pin to an interrupt-capable input and implement their own callback.

## Usage

1. Implement the `ResetInterface` trait for your specific reset mechanism.
2. Create an instance of `Cst226Driver`.
3. Initialize the touch driver.
4. Use the provided methods to read touch points and/or gestures.

```rust
// Initialize GPIO Reset Pin or I2C-controlled Reset Pin
// ResetDriver is a type that implements the `ResetInterface` trait.
let reset = ResetDriver::new(PinInstance);

// Instantiate Driver
let mut touch = Cst226Driver::new(
    I2CInstance,
    CST226_DEVICE_ADDRESS,
    reset_pin,
);

// Initialize Touch device
touch.initialize(&mut DelayInstance).expect("Failed to initialize touch driver");

// Monitor Touch events
loop {
    let touches = touch.get_touches().unwrap();

    for (i, touch) in touches.iter().enumerate() {
        println!("Touch {}: x={}, y={}", i + 1, touch.x, touch.y);
    }
}
```

## Running the Example

To run the example demonstrating touch support:

```bash
cargo run --release --example touch --all-features
```

> **Notes:**
> - If the reset pin is controlled via an I2C GPIO expander sharing the same bus with the touch driver, you should use the `embedded_hal_bus` to manage multiple instances of I2C. Refer to the examples folder to see how that looks like.
