#![deny(unsafe_code)]
#![no_std]

use core::slice;
use embedded_hal::i2c;

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

    /// Reads the thermocouple temperature with cold-junction correction applied.
    pub fn read_temperature(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature(Register::HotJunction)
    }

    /// Reads the thermocouple temperature without cold-junction correction applied.
    pub fn read_temperature_uncorrected(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature(Register::JunctionsTemperatureDelta)
    }

    /// Reads the `cold junction` or internal temperature of the MCP960x chip.
    pub fn read_temperature_internal(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature(Register::ColdJunction)
    }

    fn _read_temperature(&mut self, reg: Register) -> Result<RawTemperature, I2C::Error> {
        let mut data = [0u8, 0u8];
        self.i2c
            .write_read(self.address as u8, &[reg as u8], &mut data)?;
        Ok(RawTemperature(u16::from_be_bytes(data)))
    }

    pub fn read_adc(&mut self) -> Result<i32, I2C::Error> {
        let mut data = [0u8, 0u8, 0u8];
        self.i2c
            .write_read(self.address as u8, &[Register::RawADCData as u8], &mut data)?;
        let sign_extend = ((data[0] & 0x80 == 0) as u8).wrapping_sub(1);
        let raw = [sign_extend, data[0], data[1], data[2]];
        Ok(i32::from_be_bytes(raw))
    }

    pub fn sensor_configuration(
        &mut self,
    ) -> Result<(ThermocoupleType, FilterCoefficient), I2C::Error> {
        let mut data = 0;
        self.i2c.write_read(
            self.address as u8,
            &[Register::SensorConfiguration as u8],
            slice::from_mut(&mut data),
        )?;

        let thermocouple_type = ThermocoupleType::from_register(data);
        let filter_coefficient = FilterCoefficient::from_register(data);
        Ok((thermocouple_type, filter_coefficient))
    }

    pub fn set_sensor_configuration(
        &mut self,
        thermocouple_type: ThermocoupleType,
        filter_coefficient: FilterCoefficient,
    ) -> Result<(), I2C::Error> {
        let configuration = thermocouple_type.to_register() | filter_coefficient.to_register();
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

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Temperature(pub f32);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawTemperature(pub u16);

impl From<RawTemperature> for Temperature {
    fn from(raw: RawTemperature) -> Self {
        Self((raw.0 as i16) as f32 / 16.0)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ThermocoupleType {
    K = 0b000,
    J = 0b001,
    T = 0b010,
    N = 0b011,
    S = 0b100,
    E = 0b101,
    B = 0b110,
    R = 0b111,
}

impl ThermocoupleType {
    fn to_register(&self) -> u8 {
        (*self as u8) << 4
    }

    fn from_register(reg: u8) -> Self {
        match (reg & 0x70) >> 4 {
            0b001 => ThermocoupleType::J,
            0b010 => ThermocoupleType::T,
            0b011 => ThermocoupleType::N,
            0b100 => ThermocoupleType::S,
            0b101 => ThermocoupleType::E,
            0b110 => ThermocoupleType::B,
            0b111 => ThermocoupleType::R,
            _ => ThermocoupleType::K,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FilterCoefficient {
    Off = 0b000,
    /// Minimum
    Filter1 = 0b001,
    Filter2 = 0b010,
    Filter3 = 0b011,
    Filter4 = 0b100,
    Filter5 = 0b101,
    Filter6 = 0b110,
    /// Maximum
    Filter7 = 0b111,
}

impl FilterCoefficient {
    fn to_register(&self) -> u8 {
        *self as u8
    }

    fn from_register(reg: u8) -> Self {
        match reg & 0x07 {
            0b001 => FilterCoefficient::Filter1,
            0b010 => FilterCoefficient::Filter2,
            0b011 => FilterCoefficient::Filter3,
            0b100 => FilterCoefficient::Filter4,
            0b101 => FilterCoefficient::Filter5,
            0b110 => FilterCoefficient::Filter6,
            0b111 => FilterCoefficient::Filter7,
            _ => FilterCoefficient::Off,
        }
    }
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
    fn test_positive_convert_temperature() {
        assert_eq!(
            Temperature::from(RawTemperature(0b0000_1100_0101_0010u16)).0,
            197.125
        );
    }

    #[test]
    fn test_negative_convert_temperature() {
        assert_eq!(
            Temperature::from(RawTemperature(0b1111_0011_1010_1101u16)).0,
            -197.1875
        );
    }

    #[test]
    fn test_from_raw() {
        assert_eq!(
            Temperature::from(RawTemperature(0b1000_1011_1010_1110u16)).0,
            -1861.125
        );
    }

    #[test]
    fn test_sensor_configuration() {
        let config_byte =
            ThermocoupleType::J.to_register() | FilterCoefficient::Filter4.to_register();
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
