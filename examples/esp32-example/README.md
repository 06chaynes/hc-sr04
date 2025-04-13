# ESP32 Example for HC-SR04 Driver

This example demonstrates the usage of the HC-SR04 ultrasonic distance sensor driver on ESP32 platforms.

## Supported Hardware

- ESP32
- ESP32-S3 (default target)

## Building

1. Navigate to the example directory:

    ```bash
    cd examples/esp32-example
    ```

2. Build the project:

    ```bash
    cargo build --release
    ```

    To specifically target ESP32 or ESP32-S3, use the `--config` flag:

    ```bash
    # For ESP32-S3 (default)
    cargo build --release

    # For ESP32
    cargo build --release --config .cargo/esp32.config.toml
    ```

## Flashing

Flash and monitor the example using `espflash`:

For ESP32-S3:

```bash
espflash flash --monitor --partition-table=partitions.csv target/xtensa-esp32s3-espidf/release/esp32-example
```

For ESP32:

```bash
espflash flash --monitor --partition-table=partitions.csv target/xtensa-esp32-espidf/release/esp32-example
```
