//! A platform agnostic driver to interface the [`HC-SR04`][2] (ultrasonic distance sensor)
//! using embedded timers.
//!
//! This driver uses a generic clock that implements the `embedded_timers::clock::Clock` trait
//! and a delay object that implements `embedded_hal::delay::DelayNs`. An external update or interrupt
//! is still required to advance the sensor’s state.
//!
//! [2]: http://www.micropik.com/PDF/HCSR04.pdf
#![no_std]
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_timers::clock::Clock;

/// Publicly re-export `nb::Error` for easier usage downstream
pub use embedded_hal_nb::nb::Error;

/// Wrapper for return value of sensor.
#[derive(Debug, Copy, Clone)]
pub struct Distance(u32);

impl Distance {
    /// Get distance as centimeters.
    pub fn cm(&self) -> u32 {
        self.0 / 10
    }

    /// Get distance as millimeters.
    pub fn mm(&self) -> u32 {
        self.0
    }
}

/// Possible error returned by sensor.
#[derive(Debug, Copy, Clone)]
pub enum SensorError {
    /// Wrong mode for the operation.
    WrongMode,
    /// Timer error.
    TimerError,
    /// Pin error.
    PinError,
}

/// Internal sensor mode. The MeasurePulse state now stores the instant from the embedded clock.
enum Mode<Instant> {
    /// Ready to start new measurement.
    Idle,
    /// Sensor has been triggered and the echo is awaited.
    Triggered,
    /// Echo pulse has started—contains the timestamp when the high pulse started.
    MeasurePulse(Instant),
    /// Measurement is ready.
    Measurement(Distance),
}

/// HC-SR04 device using an embedded timer clock.
pub struct HcSr04<TrigPin, EchoPin, Clk, D>
where
    Clk: Clock,
    D: DelayNs,
{
    /// Output pin used to trigger sensor.
    trig_pin: TrigPin,
    /// Input pin to read the echo signal.
    echo_pin: EchoPin,
    /// Clock to measure durations.
    clock: Clk,
    /// Delay implementation.
    delay: D,
    /// Internal sensor state.
    mode: Mode<Clk::Instant>,
}

impl<TrigPin, EchoPin, Clk, D> HcSr04<TrigPin, EchoPin, Clk, D>
where
    TrigPin: OutputPin,
    EchoPin: InputPin,
    Clk: Clock,
    D: DelayNs,
{
    /// Create a new driver.
    ///
    /// # Arguments
    /// - `trig_pin` is the `OutputPin` connected to the sensor used to trigger the sensor.
    /// - `echo_pin` is the `InputPin` connected to the sensor to read the echo signal.
    /// - `clock` is an implementation of `embedded_timers::clock::Clock`.
    /// - `delay` provides microsecond delays.
    pub fn new(trig_pin: TrigPin, echo_pin: EchoPin, clock: Clk, delay: D) -> Self {
        HcSr04 {
            trig_pin,
            echo_pin,
            clock,
            delay,
            mode: Mode::Idle,
        }
    }

    /// Trigger a sensor reading and return the resulting `Distance`.
    ///
    /// If no complete measurement is yet available this method returns
    /// `nb::Error::WouldBlock`.
    ///
    /// The sensor state is advanced only by calling the `update` method.
    pub fn distance(&mut self) -> embedded_hal_nb::nb::Result<Distance, SensorError> {
        match self.mode {
            // Idle state -> trigger measurement.
            Mode::Idle => {
                self.trigger()?;
                Err(Error::WouldBlock)
            }
            // In the process of taking a measurement.
            Mode::Triggered | Mode::MeasurePulse(_) => Err(Error::WouldBlock),
            // Measurement ready.
            Mode::Measurement(dist) => {
                self.mode = Mode::Idle;
                Ok(dist)
            }
        }
    }

    /// Update the internal state in response to external events (e.g., an interrupt).
    ///
    /// For the proper progression:
    /// 1. After calling `distance` (in Idle), the sensor is triggered and enters Triggered mode.
    /// 2. An external event should then call this update method.
    ///    - In Triggered mode, if the echo is now high, the state advances to MeasurePulse.
    ///    - In MeasurePulse mode, if the echo is now low, the measurement is computed.
    ///
    /// # Errors
    /// Returns `SensorError::WrongMode` if update is called in an unexpected state.
    pub fn update(&mut self) -> Result<(), SensorError> {
        // Read the echo pin once.
        let echo_high = self.echo_pin.is_high().map_err(|_| SensorError::PinError)?;

        match self.mode {
            Mode::Triggered => {
                if echo_high {
                    // Transition from Triggered to MeasurePulse.
                    self.mode = Mode::MeasurePulse(self.clock.now());
                }
                Ok(())
            }
            Mode::MeasurePulse(start) => {
                if !echo_high {
                    // Calculate elapsed time.
                    let elapsed = self.clock.now() - start;
                    let ticks = elapsed.as_micros() as u32;
                    // Calculate distance_mm using:
                    // distance_mm = elapsed_us * (171,500) / 1_000_000.
                    let distance_mm = (ticks * 171_500) / 1_000_000;
                    self.mode = Mode::Measurement(Distance(distance_mm));
                }
                Ok(())
            }
            _ => Err(SensorError::WrongMode),
        }
    }

    /// Trigger the sensor by pulsing the trigger pin.
    fn trigger(&mut self) -> Result<(), SensorError> {
        self.trig_pin
            .set_high()
            .map_err(|_| SensorError::PinError)?;
        // Wait for 10 microseconds.
        self.delay.delay_us(10);
        self.trig_pin.set_low().map_err(|_| SensorError::PinError)?;
        // Set internal state to waiting for echo.
        self.mode = Mode::Triggered;
        Ok(())
    }

    /// Blocking measurement function that repeatedly calls update
    /// until the sensor returns a valid distance.
    pub fn measure(&mut self) -> Result<Distance, SensorError> {
        // If in Idle mode, this call will trigger a measurement.
        // Otherwise, it might immediately return a valid reading.
        match self.distance() {
            Ok(distance) => return Ok(distance),
            Err(embedded_hal_nb::nb::Error::WouldBlock) => {}
            Err(embedded_hal_nb::nb::Error::Other(e)) => return Err(e),
        }
        loop {
            // Process any external events.
            self.update()?;
            match self.distance() {
                Ok(distance) => return Ok(distance),
                Err(embedded_hal_nb::nb::Error::WouldBlock) => {
                    // Use a short delay to poll the sensor.
                    self.delay.delay_us(50);
                }
                Err(embedded_hal_nb::nb::Error::Other(e)) => return Err(e),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;
    use embedded_hal::delay::DelayNs;
    use embedded_hal_mock::eh1::digital::{
        Mock as DigitalMock, State, Transaction as PinTransaction,
    };
    use embedded_timers::clock::Clock;
    use embedded_timers::instant::Instant32;

    /// A simple fake clock that allows manual time advancement.
    struct FakeClock {
        time: Cell<u32>,
    }

    impl FakeClock {
        fn new(start: u32) -> Self {
            FakeClock {
                time: Cell::new(start),
            }
        }

        /// Advance the clock by the given microseconds.
        fn advance(&self, us: u32) {
            self.time.set(self.time.get().wrapping_add(us));
        }
    }

    impl Clock for FakeClock {
        /// Use an Instant type that represents microseconds.
        type Instant = Instant32<1_000_000>;

        fn now(&self) -> Self::Instant {
            Instant32::new(self.time.get())
        }
    }

    impl<'a> Clock for &'a FakeClock {
        type Instant = Instant32<1_000_000>;

        fn now(&self) -> Self::Instant {
            Instant32::new(self.time.get())
        }
    }

    /// A fake delay that advances the fake clock.
    struct FakeDelay<'a> {
        clock: &'a FakeClock,
    }

    impl<'a> DelayNs for FakeDelay<'a> {
        fn delay_ns(&mut self, ns: u32) {
            self.clock.advance((ns + 999) / 1000);
        }
        fn delay_us(&mut self, us: u32) {
            self.clock.advance(us);
        }
    }

    #[test]
    fn test_distance_measurement_with_update() {
        let trig_expectations = [
            PinTransaction::set(State::High),
            PinTransaction::set(State::Low),
        ];
        let echo_expectations = [
            // First update: echo is high, then low.
            PinTransaction::get(State::High),
            PinTransaction::get(State::Low),
        ];

        let trig_pin = DigitalMock::new(&trig_expectations);
        let echo_pin = DigitalMock::new(&echo_expectations);
        let clock = FakeClock::new(0);
        let delay = FakeDelay { clock: &clock };

        let mut sensor = HcSr04::new(trig_pin, echo_pin, &clock, delay);

        // Step 1: Idle -> Trigger measurement.
        assert!(matches!(
            sensor.distance(),
            Err(embedded_hal_nb::nb::Error::WouldBlock)
        ));

        // Step 2: Call update — transition to MeasurePulse when echo is high.
        sensor.update().unwrap();

        // Simulate pulse duration.
        let pulse_us = 110_u32;
        // This delay will advance the fake clock.
        sensor.delay.delay_us(pulse_us);

        // Step 3: Call update — detect echo falling edge and compute measurement.
        sensor.update().unwrap();

        // Get measurement.
        let d = sensor.distance().unwrap();

        let expected_mm = (pulse_us * 171_500) / 1_000_000;
        let actual_mm = d.mm();
        let tolerance = 10;
        assert!(
            (expected_mm.saturating_sub(tolerance)..=expected_mm.saturating_add(tolerance))
                .contains(&actual_mm),
            "Distance {} not within ±{} of {}",
            actual_mm,
            tolerance,
            expected_mm
        );

        sensor.trig_pin.done();
        sensor.echo_pin.done();
    }

    #[test]
    fn test_blocking_measure() {
        let trig_expectations = [
            PinTransaction::set(State::High),
            PinTransaction::set(State::Low),
        ];
        let echo_expectations = [
            // For the blocking measure, the sensor first reads High for triggering then Low.
            PinTransaction::get(State::High),
            PinTransaction::get(State::Low),
        ];

        let trig_pin = DigitalMock::new(&trig_expectations);
        let echo_pin = DigitalMock::new(&echo_expectations);
        let clock = FakeClock::new(0);
        let delay = FakeDelay { clock: &clock };

        let mut sensor = HcSr04::new(trig_pin, echo_pin, &clock, delay);

        // The measure() method will trigger the sensor and loop until a measurement is obtained.
        let distance = sensor.measure().unwrap();
        assert!(distance.mm() > 0);

        sensor.trig_pin.done();
        sensor.echo_pin.done();
    }
}
