#![deny(unsafe_code)]

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
        Ok(u8::from_le_bytes(data))
    }
}

#[derive(Clone, Copy)]
pub enum Register {
    DeviceID = 0b0010_0000, // Should contain the device ID. 0b00100000
}

impl Register {
    fn address(&self) -> u8 {
        *self as u8
    }
}
