#![cfg_attr(not(feature = "std"), no_std)]

//! DS18B20 temperature sensor driver for embedded-hal.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use one_wire_bus::{self, Address, OneWire, OneWireError, OneWireResult};

pub const FAMILY_CODE: u8 = 0x28;

#[cfg(feature = "async")]
pub mod asynch;
pub mod commands;
mod resolution;

use one_wire_bus::crc::check_crc8;
pub use resolution::Resolution;

/// All of the data that can be read from the sensor.
#[derive(Debug)]
pub struct SensorData {
    /// Temperature in degrees Celsius. Defaults to 85 on startup
    pub temperature: f32,

    /// The current resolution configuration
    pub resolution: Resolution,

    /// If the last recorded temperature is lower than this, the sensor is put in an alarm state
    pub alarm_temp_low: i8,

    /// If the last recorded temperature is higher than this, the sensor is put in an alarm state
    pub alarm_temp_high: i8,
}

pub struct Ds18b20 {
    address: Address,
}

impl Ds18b20 {
    /// Checks that the given address contains the correct family code, reads
    /// configuration data, then returns a device
    pub fn new<E>(address: Address) -> OneWireResult<Ds18b20, E> {
        if address.family_code() == FAMILY_CODE {
            Ok(Ds18b20 { address })
        } else {
            Err(OneWireError::FamilyCodeMismatch)
        }
    }

    /// Returns the device address
    pub fn address(&self) -> &Address {
        &self.address
    }

    /// Starts a temperature measurement for just this device
    /// You should wait for the measurement to finish before reading the measurement.
    /// The amount of time you need to wait depends on the current resolution configuration
    pub fn start_temp_measurement<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        onewire.send_command(commands::CONVERT_TEMP, Some(&self.address), delay)?;
        Ok(())
    }

    pub fn read_data<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<SensorData, E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        let data = read_data(&self.address, onewire, delay)?;
        Ok(data)
    }

    pub fn set_config<T, E>(
        &self,
        alarm_temp_low: i8,
        alarm_temp_high: i8,
        resolution: Resolution,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        onewire.send_command(commands::WRITE_SCRATCHPAD, Some(&self.address), delay)?;
        onewire.write_byte(alarm_temp_high.to_ne_bytes()[0], delay)?;
        onewire.write_byte(alarm_temp_low.to_ne_bytes()[0], delay)?;
        onewire.write_byte(resolution.to_config_register(), delay)?;
        Ok(())
    }

    pub fn save_to_eeprom<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        save_to_eeprom(Some(&self.address), onewire, delay)
    }

    pub fn recall_from_eeprom<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        recall_from_eeprom(Some(&self.address), onewire, delay)
    }
}

/// Starts a temperature measurement for all devices on this one-wire bus, simultaneously
pub fn start_simultaneous_temp_measurement<T, E>(
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.reset(delay)?;
    onewire.skip_address(delay)?;
    onewire.write_byte(commands::CONVERT_TEMP, delay)?;
    Ok(())
}

/// Read the contents of the EEPROM config to the scratchpad for all devices simultaneously.
pub fn simultaneous_recall_from_eeprom<T, E>(
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    recall_from_eeprom(None, onewire, delay)
}

/// Read the config contents of the scratchpad memory to the EEPROMfor all devices simultaneously.
pub fn simultaneous_save_to_eeprom<T, E>(
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    save_to_eeprom(None, onewire, delay)
}

pub fn read_scratchpad<T, E>(
    address: &Address,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<[u8; 9], E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.reset(delay)?;
    onewire.match_address(address, delay)?;
    onewire.write_byte(commands::READ_SCRATCHPAD, delay)?;
    let mut scratchpad = [0; 9];
    onewire.read_bytes(&mut scratchpad, delay)?;
    check_crc8(&scratchpad)?;
    Ok(scratchpad)
}

fn read_data<T, E>(
    address: &Address,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<SensorData, E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    let scratchpad = read_scratchpad(address, onewire, delay)?;

    let resolution = if let Some(resolution) = Resolution::from_config_register(scratchpad[4]) {
        resolution
    } else {
        return Err(OneWireError::CrcMismatch);
    };
    let temperature = decode_temperature([scratchpad[0], scratchpad[1]], resolution);
    Ok(SensorData {
        temperature,
        resolution,
        alarm_temp_high: i8::from_le_bytes([scratchpad[2]]),
        alarm_temp_low: i8::from_le_bytes([scratchpad[3]]),
    })
}

fn recall_from_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.send_command(commands::RECALL_EEPROM, address, delay)?;

    // wait for the recall to finish (up to 10ms)
    let max_retries = (10000 / one_wire_bus::READ_SLOT_DURATION_MICROS) + 1;
    for _ in 0..max_retries {
        if onewire.read_bit(delay)? == true {
            return Ok(());
        }
    }
    Err(OneWireError::Timeout)
}

fn save_to_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.send_command(commands::COPY_SCRATCHPAD, address, delay)?;
    delay.delay_us(10000); // delay 10ms for the write to complete
    Ok(())
}

fn decode_temperature(bytes: [u8; 2], resolution: Resolution) -> f32 {
    // The register always uses sixteenths of a degree. At lower resolutions
    // the least significant bits are undefined.
    let undefined_bits = match resolution {
        Resolution::Bits9 => 0b111,
        Resolution::Bits10 => 0b011,
        Resolution::Bits11 => 0b001,
        Resolution::Bits12 => 0,
    };
    let raw = i16::from_le_bytes(bytes) & !undefined_bits;
    f32::from(raw) / 16.0
}

#[cfg(test)]
mod tests {
    use super::{decode_temperature, Resolution};

    #[test]
    fn decodes_signed_temperatures_at_all_resolutions() {
        for (resolution, positive, negative, expected) in [
            (Resolution::Bits9, [0x1f, 0x00], [0xef, 0xff], 1.5),
            (Resolution::Bits10, [0x17, 0x00], [0xef, 0xff], 1.25),
            (Resolution::Bits11, [0x13, 0x00], [0xef, 0xff], 1.125),
            (Resolution::Bits12, [0x11, 0x00], [0xef, 0xff], 1.0625),
        ] {
            assert_eq!(decode_temperature(positive, resolution), expected);
            assert_eq!(decode_temperature(negative, resolution), -expected);
        }
    }
}
