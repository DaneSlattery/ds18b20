# DS18B20 temperature sensor

A Rust driver for the [DS18B20](https://www.analog.com/en/products/DS18B20.html)
temperature sensor using [embedded-hal](https://github.com/rust-embedded/embedded-hal) 1.0.

This is a fork of [fuchsnj/ds18b20](https://github.com/fuchsnj/ds18b20), the original
driver, which is no longer actively maintained. This fork uses the
[async-capable one-wire-bus fork](https://github.com/DaneSlattery/one-wire-bus)
and provides blocking and async APIs.

Add this fork to your project's dependencies.

```toml
[dependencies]
ds18b20 = { git = "https://github.com/DaneSlattery/ds18b20", branch = "master" }
```

The default `async` feature adds `ds18b20::asynch::Ds18b20Async` for use with
`OneWireAsync` and an `embedded_hal_async::delay::DelayNs` implementation. Set
`default-features = false` if you only use the blocking API.

## Quick Start

### Get Temperature
```rust
use core::fmt::{Debug, Write};
use ds18b20::{Ds18b20, Resolution};
use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin}};
use one_wire_bus::{OneWire, OneWireResult};

fn get_temperature<P, E>(
    delay: &mut impl DelayNs,
    tx: &mut impl Write,
    one_wire_bus: &mut OneWire<P>,
) -> OneWireResult<(), E>
    where
        P: OutputPin<Error=E> + InputPin<Error=E>,
        E: Debug
{
    // initiate a temperature measurement for all connected devices
    ds18b20::start_simultaneous_temp_measurement(one_wire_bus, delay)?;

    // wait until the measurement is done. This depends on the resolution you specified
    // If you don't know the resolution, you can obtain it from reading the sensor data,
    // or just wait the longest time, which is the 12-bit resolution (750ms)
    Resolution::Bits12.delay_for_measurement_time(delay);

    // iterate over all the devices, and report their temperature
    let mut search_state = None;
    loop {
        if let Some((device_address, state)) = one_wire_bus.device_search(search_state.as_ref(), false, delay)? {
            search_state = Some(state);
            if device_address.family_code() != ds18b20::FAMILY_CODE {
                // skip other devices
                continue;
            }
            // You will generally create the sensor once, and save it for later
            let sensor = Ds18b20::new(device_address)?;

            // contains the read temperature, as well as config info such as the resolution used
            let sensor_data = sensor.read_data(one_wire_bus, delay)?;
            let _ = writeln!(tx, "Device at {:?} is {}°C", device_address, sensor_data.temperature);
        } else {
            break;
        }
    }
    Ok(())
}
```

### Configuration
```rust
use core::fmt::{Debug, Write};
use ds18b20::Ds18b20;
use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin}};
use one_wire_bus::{OneWire, OneWireResult};

fn test_config<P, E>(
    delay: &mut impl DelayNs,
    tx: &mut impl Write,
    one_wire_bus: &mut OneWire<P>,
) -> OneWireResult<(), E>
    where
        P: OutputPin<Error=E> + InputPin<Error=E>,
        E: Debug
{

    // Find the first device on the bus (assuming they are all Ds18b20's)
    if let Some(device_address) = one_wire_bus.devices(false, delay).next() {
        let device_address = device_address?;
        let device = Ds18b20::new(device_address)?;

        // read the initial config values (read from EEPROM by the device when it was first powered)
        let initial_data = device.read_data(one_wire_bus, delay)?;
        let _ = writeln!(tx, "Initial data: {:?}", initial_data);

        let resolution = initial_data.resolution;

        // set new alarm values, but keep the resolution the same
        device.set_config(18, 24, resolution, one_wire_bus, delay)?;

        // confirm the new config is now in the scratchpad memory
        let new_data = device.read_data(one_wire_bus, delay)?;
        let _ = writeln!(tx, "New data: {:?}", new_data);

        // save the config to EEPROM to save it permanently
        device.save_to_eeprom(one_wire_bus, delay)?;

        // read the values from EEPROM back to the scratchpad to verify it was saved correctly
        device.recall_from_eeprom(one_wire_bus, delay)?;
        let eeprom_data = device.read_data(one_wire_bus, delay)?;
        let _ = writeln!(tx, "EEPROM data: {:?}", eeprom_data);
    }
    Ok(())
}
```
Example output
```
Initial data: SensorData { temperature: 85.0, resolution: Bits12, alarm_temp_low: 70, alarm_temp_high: 75 }
New data: SensorData { temperature: 85.0, resolution: Bits12, alarm_temp_low: 18, alarm_temp_high: 24 }
EEPROM data: SensorData { temperature: 85.0, resolution: Bits12, alarm_temp_low: 18, alarm_temp_high: 24 }
```
