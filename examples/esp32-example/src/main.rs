use esp_idf_sys::{self as _}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use esp_idf_hal::{delay::Delay, peripherals::Peripherals};
use hc_sr04::HcSr04;
use log::{error, info};

struct SystemClock;

impl embedded_timers::clock::Clock for SystemClock {
    type Instant = embedded_timers::instant::Instant32<1_000_000>;

    fn now(&self) -> Self::Instant {
        embedded_timers::instant::Instant32::new(unsafe {
            esp_idf_sys::esp_timer_get_time() as u32
        })
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let delay = Delay::new(1000);

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let echo_pin = esp_idf_hal::gpio::PinDriver::input(pins.gpio9).unwrap();
    let trigger_pin = esp_idf_hal::gpio::PinDriver::output(pins.gpio10).unwrap();

    let clock = SystemClock;
    let mut sensor = HcSr04::new(trigger_pin, echo_pin, clock, delay);

    loop {
        match sensor.measure() {
            Ok(distance) => info!("Distance: {} cm ({} mm)", distance.cm(), distance.mm()),
            Err(e) => error!("Failed to read distance from HC-SR04 sensor: {:?}", e),
        }

        delay.delay_ms(1000);
    }
}
