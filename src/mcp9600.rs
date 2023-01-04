#![deny(unsafe_code)]

use bitvec::prelude::*;
use embedded_hal::blocking::i2c;

#[derive(Debug)]
pub struct MCP9600<I2C> {
    // The concrete I2C device implementation
    i2c: I2C,

    // Device address
    address: DeviceAddr,
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

impl<I2C, E> MCP9600<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    /// Creates a new instance of the sensor, taking ownership of the i2c peripheral.
    pub fn new(i2c: I2C, address: DeviceAddr) -> Result<Self, E> {
        Ok(Self { i2c, address })
    }

    /// Returns the device's ID
    pub fn read_device_id_register(&mut self) -> Result<u8, E> {
        self.read_register(Register::DeviceID)
        // This should return 64 for MCP9600 and 65 for MCP9601
    }

    /// Writes into a register
    #[allow(unused)]
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), E> {
        let byte = value as u8;
        self.i2c
            .write(self.address as u8, &[register.address(), byte])
    }

    /// Reads a register using the `write_read` method
    fn read_register(&mut self, register: Register) -> Result<u8, E> {
        let mut data = [0];
        self.i2c
            .write_read(self.address as u8, &[register.address()], &mut data)?;
        Ok(u8::from_le_bytes(data)) // from_le_bytes converts from little endian
    }

    pub fn read_hot_junction(&mut self) -> Result<f32, E> {
        let mut data = [0u8, 2];
        self.i2c.write_read(
            self.address as u8,
            &[Register::HotJunction as u8],
            &mut data,
        )?;
        Self::temperature_conversion(&data)
    }

    pub fn temperature_conversion(buffer: &[u8]) -> Result<f32, E> {
        let sign = buffer[0].view_bits::<Lsb0>()[0];
        match sign {
            false => return Ok(buffer[0] as f32 * 16.0 + buffer[1] as f32 / 16.0),
            true => return Ok((buffer[0] as f32 * 16.0 + buffer[1] as f32 / 16.0) - 4096.0),
        }
    }

    pub fn set_sensor_configuration(
        &mut self,
        thermocoupletype: ThermocoupleType,
        filtercoefficient: FilterCoefficient,
    ) -> Result<(), E> {
        let configuration: u8 = thermocoupletype as u8 & filtercoefficient as u8;
        // for TypeK and Medium filter, this should produce the following LSB0 byte:
        // 0  0  0  0  0  1  0  0
        // 7  6  5  4  3  2  1  0
        // U  ThermoT  U  FilterT
        self.i2c.write(
            self.address as u8,
            &[Register::HotJunction as u8, configuration],
        )
    }
    // TODO: Device Configuration Register
    // TODO: Read Delta Temperature Register
    // TODO: Read Cold Junction
}

#[derive(Clone, Copy)]
pub enum Register {
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
    Bit18 = 0b00,
    Bit16 = 0b01,
    Bit14 = 0b10,
    Bit12 = 0b11,
}
pub enum BurstModeSamples {
    Sample1 = 0b000,
    Sample2 = 0b001,
    Sample4 = 0b010,
    Sample8 = 0b011,
    Sample16 = 0b100,
    Sample32 = 0b101,
    Sample64 = 0b110,
    Sample128 = 0b111,
}
pub enum ShutdownMode {
    NormalMode = 0b00,
    ShutdownMode = 0b01,
    BurstMode = 0b10,
}
impl Register {
    fn address(&self) -> u8 {
        *self as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_positive_temperature_conversion() {
        let data = [0b0000_1100u8, 0b0101_0010u8];
        let temperature = new_temperature_conversion(&data);
        assert_eq!(temperature, 197.125);
    }
    #[test]
    fn test_negative_temperature_conversion() {
        let data = [0b1111_0011u8, 0b1010_1101u8];
        let temperature = new_temperature_conversion(&data);
        assert_eq!(temperature, -197.1875);
    }
}

pub fn new_temperature_conversion(buffer: &[u8]) -> f32 {
    let sign = buffer[0].view_bits::<Lsb0>()[0];
    match sign {
        false => buffer[0] as f32 * 16.0 + buffer[1] as f32 / 16.0,
        true => (buffer[0] as f32 * 16.0 + buffer[1] as f32 / 16.0) - 4096.0,
    }
}
