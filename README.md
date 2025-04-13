# `hc-sr04`

> A platform agnostic driver to interface with the HC-SR04 (ultrasonic distance)

## Features

- Platform agnostic using `embedded-hal` traits
- Support for both blocking and non-blocking measurements
- Distance measurements with accurate timing using embedded clocks

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
hc-sr04 = "0.2.0"
```

### Example

For a complete working example using ESP32, check out the [examples/esp32-example](examples/esp32-example) directory. Here's a basic demonstration of how to use the HC-SR04 ultrasonic distance sensor with the `esp-idf-hal` crate:

```rust
use esp_idf_hal::{delay::Delay, peripherals::Peripherals};
use hc_sr04::HcSr04;
use log::{error, info};

// Create clock implementation for ESP32
struct SystemClock;
impl embedded_timers::clock::Clock for SystemClock {
  type Instant = embedded_timers::instant::Instant32<1_000_000>;
  fn now(&self) -> Self::Instant {
    embedded_timers::instant::Instant32::new(unsafe {
      esp_idf_sys::esp_timer_get_time() as u32
    })
  }
}

// Initialize sensor with ESP32 pins
let peripherals = Peripherals::take().unwrap();
let echo_pin = esp_idf_hal::gpio::PinDriver::input(pins.gpio9).unwrap();
let trigger_pin = esp_idf_hal::gpio::PinDriver::output(pins.gpio10).unwrap();

let clock = SystemClock;
let delay = Delay::new(1000);
let mut sensor = HcSr04::new(trigger_pin, echo_pin, clock, delay);

// Take measurements
match sensor.measure() {
  Ok(distance) => info!("Distance: {} cm ({} mm)", distance.cm(), distance.mm()),
  Err(e) => error!("Error: {:?}", e),
}
```

## How it Works

The driver triggers the HC-SR04 sensor and measures the duration of the echo pulse to calculate distance. It supports:

- Trigger pulse generation
- Echo pulse timing measurement
- Distance calculation
- Error handling for timeout and pin failures

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
