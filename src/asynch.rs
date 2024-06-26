use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::{delay::DelayNs, digital::Wait};
use one_wire_bus::{asynch::OneWireAsync, crc::check_crc8, Address, OneWireError, OneWireResult};

use crate::{commands, FAMILY_CODE};

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum ResolutionAsync {
    Bits9 = 0b00011111,
    Bits10 = 0b00111111,
    Bits11 = 0b01011111,
    Bits12 = 0b01111111,
}

impl ResolutionAsync {
    pub fn max_measurement_time_millis(&self) -> u32 {
        match self {
            ResolutionAsync::Bits9 => 94,
            ResolutionAsync::Bits10 => 188,
            ResolutionAsync::Bits11 => 375,
            ResolutionAsync::Bits12 => 750,
        }
    }

    /// Blocks for the amount of time required to finished measuring temperature
    /// using this resolution
    pub async fn delay_for_measurement_time(&self, delay: &mut impl DelayNs) {
        delay.delay_ms(self.max_measurement_time_millis()).await;
    }

    pub(crate) fn from_config_register(config: u8) -> Option<ResolutionAsync> {
        match config {
            0b00011111 => Some(ResolutionAsync::Bits9),
            0b00111111 => Some(ResolutionAsync::Bits10),
            0b01011111 => Some(ResolutionAsync::Bits11),
            0b01111111 => Some(ResolutionAsync::Bits12),
            _ => None,
        }
    }

    pub(crate) fn to_config_register(self) -> u8 {
        self as u8
    }
}

/// All of the data that can be read from the sensor.
#[derive(Debug)]
pub struct SensorData {
    /// Temperature in degrees Celsius. Defaults to 85 on startup
    pub temperature: f32,

    /// The current resolution configuration
    pub resolution: ResolutionAsync,

    /// If the last recorded temperature is lower than this, the sensor is put in an alarm state
    pub alarm_temp_low: i8,

    /// If the last recorded temperature is higher than this, the sensor is put in an alarm state
    pub alarm_temp_high: i8,
}

pub struct Ds18b20Async {
    address: Address,
}

impl Ds18b20Async {
    /// Checks that the given address contains the correct family code, reads
    /// configuration data, then returns a device
    pub fn new<E>(address: Address) -> OneWireResult<Ds18b20Async, E> {
        if address.family_code() == FAMILY_CODE {
            Ok(Ds18b20Async { address })
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
    pub async fn start_temp_measurement<T, E>(
        &self,
        onewire: &mut OneWireAsync<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
        T: Wait<Error = E>,
    {
        onewire
            .send_command(commands::CONVERT_TEMP, Some(&self.address), delay)
            .await?;
        Ok(())
    }

    pub async fn read_data<T, E>(
        &self,
        onewire: &mut OneWireAsync<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<SensorData, E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
        T: Wait<Error = E>,
    {
        let data = read_data(&self.address, onewire, delay).await?;
        Ok(data)
    }

    pub async fn set_config<T, E>(
        &self,
        alarm_temp_low: i8,
        alarm_temp_high: i8,
        resolution: ResolutionAsync,
        onewire: &mut OneWireAsync<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
        T: Wait<Error = E>,
    {
        onewire
            .send_command(commands::WRITE_SCRATCHPAD, Some(&self.address), delay)
            .await?;
        onewire
            .write_byte(alarm_temp_high.to_ne_bytes()[0], delay)
            .await?;
        onewire
            .write_byte(alarm_temp_low.to_ne_bytes()[0], delay)
            .await?;
        onewire
            .write_byte(resolution.to_config_register(), delay)
            .await?;
        Ok(())
    }

    pub async fn save_to_eeprom<T, E>(
        &self,
        onewire: &mut OneWireAsync<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
        T: Wait<Error = E>,
    {
        save_to_eeprom(Some(&self.address), onewire, delay).await
    }

    pub async fn recall_from_eeprom<T, E>(
        &self,
        onewire: &mut OneWireAsync<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
        T: Wait<Error = E>,
    {
        recall_from_eeprom(Some(&self.address), onewire, delay).await
    }
}

/// Starts a temperature measurement for all devices on this one-wire bus, simultaneously
pub async fn start_simultaneous_temp_measurement<T, E>(
    onewire: &mut OneWireAsync<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
    T: Wait<Error = E>,
{
    onewire.reset(delay).await?;
    onewire.skip_address(delay).await?;
    onewire.write_byte(commands::CONVERT_TEMP, delay).await?;
    Ok(())
}

/// Read the contents of the EEPROM config to the scratchpad for all devices simultaneously.
pub async fn simultaneous_recall_from_eeprom<T, E>(
    onewire: &mut OneWireAsync<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
    T: Wait<Error = E>,
{
    recall_from_eeprom(None, onewire, delay).await
}

/// Read the config contents of the scratchpad memory to the EEPROMfor all devices simultaneously.
pub async fn simultaneous_save_to_eeprom<T, E>(
    onewire: &mut OneWireAsync<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
    T: Wait<Error = E>,
{
    save_to_eeprom(None, onewire, delay).await
}

pub async fn read_scratchpad<T, E>(
    address: &Address,
    onewire: &mut OneWireAsync<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<[u8; 9], E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
    T: Wait<Error = E>,
{
    onewire.reset(delay).await?;
    onewire.match_address(address, delay).await?;
    onewire.write_byte(commands::READ_SCRATCHPAD, delay).await?;
    let mut scratchpad = [0; 9];
    onewire.read_bytes(&mut scratchpad, delay).await?;
    check_crc8(&scratchpad)?;
    Ok(scratchpad)
}

async fn read_data<T, E>(
    address: &Address,
    onewire: &mut OneWireAsync<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<SensorData, E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
    T: Wait<Error = E>,
{
    let scratchpad = read_scratchpad(address, onewire, delay).await?;

    let resolution = if let Some(resolution) = ResolutionAsync::from_config_register(scratchpad[4])
    {
        resolution
    } else {
        return Err(OneWireError::CrcMismatch);
    };
    let raw_temp = u16::from_le_bytes([scratchpad[0], scratchpad[1]]);
    let temperature = match resolution {
        ResolutionAsync::Bits12 => (raw_temp as f32) / 16.0,
        ResolutionAsync::Bits11 => (raw_temp as f32) / 8.0,
        ResolutionAsync::Bits10 => (raw_temp as f32) / 4.0,
        ResolutionAsync::Bits9 => (raw_temp as f32) / 2.0,
    };
    Ok(SensorData {
        temperature,
        resolution,
        alarm_temp_high: i8::from_le_bytes([scratchpad[2]]),
        alarm_temp_low: i8::from_le_bytes([scratchpad[3]]),
    })
}

async fn recall_from_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWireAsync<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
    T: Wait<Error = E>,
{
    onewire
        .send_command(commands::RECALL_EEPROM, address, delay)
        .await?;

    // wait for the recall to finish (up to 10ms)
    let max_retries = (10000 / one_wire_bus::READ_SLOT_DURATION_MICROS) + 1;
    for _ in 0..max_retries {
        if onewire.read_bit(delay).await? {
            return Ok(());
        }
    }
    Err(OneWireError::Timeout)
}

async fn save_to_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWireAsync<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
    T: Wait<Error = E>,
{
    onewire
        .send_command(commands::COPY_SCRATCHPAD, address, delay)
        .await?;
    delay.delay_us(10000).await; // delay 10ms for the write to complete
    Ok(())
}
