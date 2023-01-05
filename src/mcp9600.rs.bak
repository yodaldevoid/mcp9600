#![deny(unsafe_code)]

use crate::encode::*;
use bitvec::prelude::*;
use embedded_hal::blocking::i2c;

#[derive(Debug)]
pub struct MCP9600<I2C> {
    // The concrete I2C device implementation
    i2c: I2C,

    // Device address
    address: DeviceAddr,
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
        Ok(temperature_conversion(&data))
    }

    pub fn set_sensor_configuration(
        &mut self,
        thermocoupletype: ThermocoupleType,
        filtercoefficient: FilterCoefficient,
    ) -> Result<(), E> {
        // for TypeK and Medium filter, this should produce the following LSB0 byte:
        // 0  0  0  0  0  1  0  0
        // 7  6  5  4  3  2  1  0
        // U  ThermoT  U  FilterT
        let configuration = sensor_configuration(thermocoupletype, filtercoefficient);
        self.i2c.write(
            self.address as u8,
            &[Register::HotJunction as u8, configuration],
        )
    }

    pub fn set_device_configuration(
        &mut self,
        coldjunctionresolution: ColdJunctionResolution,
        adcresolution: ADCResolution,
        burstmodesamples: BurstModeSamples,
        shutdownmode: ShutdownMode,
    ) -> Result<(), E> {
        // 7  | 6   5 |  4   3   2 |  1   0
        // CJ |ADC Res|Burst Mode  | Shutdown
        let configuration = device_configuration(
            coldjunctionresolution,
            adcresolution,
            burstmodesamples,
            shutdownmode,
        );
        // Example configuration for High CJ, 16bit ADC, 2 samples, Shutdown mode
        // 0 | 0 1 | 0 0 1 | 0 1 = 37 = 0x25
        self.i2c.write(
            self.address as u8,
            &[Register::DeviceConfiguration as u8, configuration],
        )
    }
    // TODO: Read Delta Temperature Register
    // TODO: Read Cold Junction
}

impl Register {
    fn address(&self) -> u8 {
        *self as u8
    }
}
