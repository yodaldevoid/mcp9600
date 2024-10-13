#![deny(unsafe_code)]
#![no_std]

mod types;

use embedded_hal::i2c;

pub use types::*;

#[derive(Debug)]
pub struct MCP9600<I2C> {
    // The concrete I2C device implementation
    i2c: I2C,

    // Device address
    address: DeviceAddr,
}

impl<I2C: i2c::I2c> MCP9600<I2C> {
    /// Creates a new instance of the sensor, taking ownership of the i2c peripheral
    pub fn new(i2c: I2C, address: DeviceAddr) -> Result<Self, I2C::Error> {
        Ok(Self { i2c, address })
    }

    /// Returns the device's ID and revision.
    pub fn device_id(&mut self) -> Result<[u8; 2], I2C::Error> {
        let mut data = [0u8, 0u8];
        self.i2c
            .write_read(self.address as u8, &[Register::DeviceID as u8], &mut data)?;
        Ok(data)
    }

    /// Reads the `hot junction` or thermocouple side
    /// ! This will still succeed even if there is no thermocouple connected !
    pub fn read_hot_junction(&mut self) -> Result<RawTemperature, I2C::Error> {
        let mut data = [0u8, 0u8];
        self.i2c.write_read(
            self.address as u8,
            &[Register::HotJunction as u8],
            &mut data,
        )?;
        Ok(RawTemperature(u16::from_be_bytes(data)))
    }

    /// Reads the `cold junction` or internal temperature of the
    /// mcp960x chip
    pub fn read_cold_junction(&mut self) -> Result<RawTemperature, I2C::Error> {
        let mut data = [0u8, 0u8];
        self.i2c.write_read(
            self.address as u8,
            &[Register::ColdJunction as u8],
            &mut data,
        )?;
        Ok(RawTemperature(u16::from_be_bytes(data)))
    }

    /// Reads the raw ADC data. Does no extra processing of the returned data
    /// Note that the data is formatted LSB0
    pub fn read_adc_raw(&mut self) -> Result<[u8; 3], I2C::Error> {
        let mut data = [0u8, 0u8, 0u8];
        self.i2c
            .write_read(self.address as u8, &[Register::RawADCData as u8], &mut data)?;
        Ok(data)
    }

    /// Set the sensor configuration. Requires a thermocouple type, and filter coefficient to be
    /// specified
    pub fn set_sensor_configuration(
        &mut self,
        thermocoupletype: ThermocoupleType,
        filtercoefficient: FilterCoefficient,
    ) -> Result<(), I2C::Error> {
        let configuration = sensor_configuration(thermocoupletype, filtercoefficient);
        self.i2c.write(
            self.address as u8,
            &[Register::SensorConfiguration as u8, configuration],
        )
    }

    /// Sets the device configuration. Requires a cold junction resolution
    /// ADC resolution, burst mode samples (even if not using burst mode),
    /// and shutdown mode.
    pub fn set_device_configuration(
        &mut self,
        coldjunctionresolution: ColdJunctionResolution,
        adcresolution: ADCResolution,
        burstmodesamples: BurstModeSamples,
        shutdownmode: ShutdownMode,
    ) -> Result<(), I2C::Error> {
        let configuration = device_configuration(
            coldjunctionresolution,
            adcresolution,
            burstmodesamples,
            shutdownmode,
        );
        self.i2c.write(
            self.address as u8,
            &[Register::DeviceConfiguration as u8, configuration],
        )
    }
}

// Functions for testing
/// Generates a binary u8 word which contains the necessary sensor configuration
fn sensor_configuration(
    thermocoupletype: ThermocoupleType,
    filtercoefficient: FilterCoefficient,
) -> u8 {
    thermocoupletype as u8 | filtercoefficient as u8
}

/// Generates a binary u8 word which contains the necessary device configuration
fn device_configuration(
    coldjunctionresolution: ColdJunctionResolution,
    adcresolution: ADCResolution,
    burstmodesamples: BurstModeSamples,
    shutdownmode: ShutdownMode,
) -> u8 {
    coldjunctionresolution as u8 | adcresolution as u8 | burstmodesamples as u8 | shutdownmode as u8
}

#[allow(unused)]
#[derive(Clone, Copy)]
enum Register {
    HotJunction = 0b0000_0000,
    JunctionsTemperatureDelta = 0b0000_0001,
    ColdJunction = 0b0000_0010,
    RawADCData = 0b0000_0011,
    Status = 0b0000_0100,
    SensorConfiguration = 0b0000_0101,
    DeviceConfiguration = 0b0000_0110,
    Alert1Configuration = 0b0000_1000,
    Alert2Configuration = 0b0000_1001,
    Alert3Configuration = 0b0000_1010,
    Alert4Configuration = 0b0000_1011,
    Alert1Hysteresis = 0b0000_1100,
    Alert2Hysteresis = 0b0000_1101,
    Alert3Hysteresis = 0b0000_1110,
    Alert4Hysteresis = 0b0000_1111,
    Alert1Limit = 0b0001_0000,
    Alert2Limit = 0b0001_0001,
    Alert3Limit = 0b0001_0010,
    Alert4Limit = 0b0001_0011,
    DeviceID = 0b0010_0000, // Should contain the device ID
}

#[derive(Clone, Copy)]
pub enum ThermocoupleType {
    // Rather than mess around with constructing a bit vector, lets just make this a logical
    // operator
    TypeK = 0b0000_0000,
    TypeJ = 0b0001_0000,
    TypeT = 0b0010_0000,
    TypeN = 0b0011_0000,
    TypeS = 0b0100_0000,
    TypeE = 0b0101_0000,
    TypeB = 0b0110_0000,
    TypeR = 0b0111_0000,
}

#[derive(Clone, Copy)]
pub enum FilterCoefficient {
    FilterOff = 0b0000_0000,
    FilterMinimum = 0b0000_0001,
    Filter2 = 0b0000_0010,
    Filter3 = 0b0000_0011,
    FilterMedium = 0b0000_0100,
    Filter5 = 0b0000_0101,
    Filter6 = 0b0000_0110,
    FilterMaximum = 0b0000_0111,
}

pub enum ADCResolution {
    /// 320 ms update time
    Bit18 = 0b0000_0000,
    /// 80 ms update time
    Bit16 = 0b0010_0000,
    /// 20 ms update time
    Bit14 = 0b0100_0000,
    /// 5 ms update time
    Bit12 = 0b0110_0000,
}
pub enum BurstModeSamples {
    Sample1 = 0b0000_0000,
    Sample2 = 0b0000_0100,
    Sample4 = 0b0000_1000,
    Sample8 = 0b0000_1100,
    Sample16 = 0b0001_0000,
    Sample32 = 0b0001_0100,
    Sample64 = 0b0001_1000,
    Sample128 = 0b0001_1100,
}
pub enum ShutdownMode {
    NormalMode = 0b0000_0000,
    ShutdownMode = 0b0000_0001,
    BurstMode = 0b0000_0010,
}

pub enum ColdJunctionResolution {
    High = 0b0000_0000, //0.0625C
    Low = 0b1000_0000,  //0.25C
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeviceAddr {
    // The device address can be any of 8 values from 96-103 depending on the value of pin 16
    // If tied to GND then the value should be 0b110_0000
    // If tied to VDD then the value should be 0b110_0111

    // 96
    AD0 = 0b110_0000,
    // 97
    AD1 = 0b110_0001,
    // 98
    AD2 = 0b110_0010,
    // 99
    AD3 = 0b110_0011,
    // 100
    AD4 = 0b110_0100,
    // 101
    AD5 = 0b110_0101,
    // 102
    AD6 = 0b110_0110,
    // 103
    AD7 = 0b110_0111,
}
// Testing
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_sensor_configuration() {
        let config_byte =
            sensor_configuration(ThermocoupleType::TypeJ, FilterCoefficient::FilterMedium);
        assert_eq!(config_byte, 0b0001_0100)
    }
    #[test]
    fn test_device_configuration() {
        let config_byte = device_configuration(
            ColdJunctionResolution::Low,
            ADCResolution::Bit14,
            BurstModeSamples::Sample16,
            ShutdownMode::BurstMode,
        );
        assert_eq!(config_byte, 0b1101_0010)
    }
}
