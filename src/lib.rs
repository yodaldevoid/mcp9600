#![deny(unsafe_code)]
#![no_std]

use core::slice;
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

#[derive(Debug)]
pub struct MCP9600<I2C> {
    i2c: I2C,
    address: DeviceAddress,
}

impl<I2C> MCP9600<I2C> {
    pub fn new(i2c: I2C, address: DeviceAddress) -> Self {
        Self { i2c, address }
    }
}

impl<I2C: I2c> MCP9600<I2C> {
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

    pub fn status(&mut self) -> Result<Status, I2C::Error> {
        let mut data = 0;
        self.i2c.write_read(
            self.address as u8,
            &[Register::Status as u8],
            slice::from_mut(&mut data),
        )?;
        Ok(Status::from_register(data))
    }

    pub fn clear_burst_complete(&mut self) -> Result<(), I2C::Error> {
        self.clear_status(true, false)
    }

    pub fn clear_temperature_update(&mut self) -> Result<(), I2C::Error> {
        self.clear_status(false, true)
    }

    pub fn clear_status(
        &mut self,
        burst_complete: bool,
        temperature_update: bool,
    ) -> Result<(), I2C::Error> {
        let data = ((burst_complete as u8) << 7) | ((temperature_update as u8) << 6);
        self.i2c
            .write(self.address as u8, &[Register::Status as u8, data])
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

    pub fn device_configuration(
        &mut self,
    ) -> Result<
        (
            ColdJunctionResolution,
            AdcResolution,
            BurstModeSamples,
            ShutdownMode,
        ),
        I2C::Error,
    > {
        let mut data = 0;
        self.i2c.write_read(
            self.address as u8,
            &[Register::SensorConfiguration as u8],
            slice::from_mut(&mut data),
        )?;

        let cold_resolution = ColdJunctionResolution::from_register(data);
        let adc_resolution = AdcResolution::from_register(data);
        let burst_samples = BurstModeSamples::from_register(data);
        let shutdown_mode = ShutdownMode::from_register(data);
        Ok((
            cold_resolution,
            adc_resolution,
            burst_samples,
            shutdown_mode,
        ))
    }

    pub fn set_device_configuration(
        &mut self,
        cold_resolution: ColdJunctionResolution,
        adc_resolution: AdcResolution,
        burst_samples: BurstModeSamples,
        shutdown_mode: ShutdownMode,
    ) -> Result<(), I2C::Error> {
        let configuration = cold_resolution.to_register()
            | adc_resolution.to_register()
            | burst_samples.to_register()
            | shutdown_mode.to_register();
        self.i2c.write(
            self.address as u8,
            &[Register::DeviceConfiguration as u8, configuration],
        )
    }

    pub fn alert1_configuration(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration(Register::Alert1Configuration)
    }

    pub fn alert2_configuration(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration(Register::Alert2Configuration)
    }

    pub fn alert3_configuration(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration(Register::Alert3Configuration)
    }

    pub fn alert4_configuration(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration(Register::Alert4Configuration)
    }

    fn alert_configuration(&mut self, reg: Register) -> Result<AlertConfig, I2C::Error> {
        let mut data = 0;
        self.i2c
            .write_read(self.address as u8, &[reg as u8], slice::from_mut(&mut data))?;
        Ok(AlertConfig::from_register(data))
    }

    pub fn set_alert1_configuration(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration(Register::Alert1Configuration, config, clear_interrupt)
    }

    pub fn set_alert2_configuration(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration(Register::Alert2Configuration, config, clear_interrupt)
    }

    pub fn set_alert3_configuration(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration(Register::Alert3Configuration, config, clear_interrupt)
    }

    pub fn set_alert4_configuration(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration(Register::Alert4Configuration, config, clear_interrupt)
    }

    fn set_alert_configuration(
        &mut self,
        reg: Register,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.i2c.write(
            self.address as u8,
            &[reg as u8, config.to_register(clear_interrupt)],
        )
    }

    pub fn alert1_hysteresis(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis(Register::Alert1Hysteresis)
    }

    pub fn alert2_hysteresis(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis(Register::Alert2Hysteresis)
    }

    pub fn alert3_hysteresis(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis(Register::Alert3Hysteresis)
    }

    pub fn alert4_hysteresis(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis(Register::Alert4Hysteresis)
    }

    fn alert_hysteresis(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data = 0;
        self.i2c
            .write_read(self.address as u8, &[reg as u8], slice::from_mut(&mut data))?;
        Ok(data)
    }

    pub fn set_alert1_hysteresis(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis(Register::Alert1Hysteresis, hysteresis)
    }

    pub fn set_alert2_hysteresis(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis(Register::Alert2Hysteresis, hysteresis)
    }

    pub fn set_alert3_hysteresis(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis(Register::Alert3Hysteresis, hysteresis)
    }

    pub fn set_alert4_hysteresis(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis(Register::Alert4Hysteresis, hysteresis)
    }

    fn set_alert_hysteresis(&mut self, reg: Register, hysteresis: u8) -> Result<(), I2C::Error> {
        self.i2c.write(self.address as u8, &[reg as u8, hysteresis])
    }

    pub fn alert1_limit(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature(Register::Alert1Limit)
    }

    pub fn alert2_limit(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature(Register::Alert2Limit)
    }

    pub fn alert3_limit(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature(Register::Alert3Limit)
    }

    pub fn alert4_limit(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature(Register::Alert4Limit)
    }

    pub fn set_alert1_limit(&mut self, limit: RawTemperature) -> Result<(), I2C::Error> {
        self.set_alert_limit(Register::Alert1Limit, limit)
    }

    pub fn set_alert2_limit(&mut self, limit: RawTemperature) -> Result<(), I2C::Error> {
        self.set_alert_limit(Register::Alert2Limit, limit)
    }

    pub fn set_alert3_limit(&mut self, limit: RawTemperature) -> Result<(), I2C::Error> {
        self.set_alert_limit(Register::Alert3Limit, limit)
    }

    pub fn set_alert4_limit(&mut self, limit: RawTemperature) -> Result<(), I2C::Error> {
        self.set_alert_limit(Register::Alert4Limit, limit)
    }

    fn set_alert_limit(&mut self, reg: Register, limit: RawTemperature) -> Result<(), I2C::Error> {
        let data = limit.0.to_be_bytes();
        self.i2c
            .write(self.address as u8, &[reg as u8, data[0], data[1]])
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> MCP9600<I2C> {
    /// Returns the device's ID and revision.
    pub async fn device_id_async(&mut self) -> Result<[u8; 2], I2C::Error> {
        let mut data = [0u8, 0u8];
        self.i2c
            .write_read(self.address as u8, &[Register::DeviceID as u8], &mut data)
            .await?;
        Ok(data)
    }

    /// Reads the thermocouple temperature with cold-junction correction applied.
    pub async fn read_temperature_async(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature_async(Register::HotJunction).await
    }

    /// Reads the thermocouple temperature without cold-junction correction applied.
    pub async fn read_temperature_uncorrected_async(
        &mut self,
    ) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature_async(Register::JunctionsTemperatureDelta)
            .await
    }

    /// Reads the `cold junction` or internal temperature of the MCP960x chip.
    pub async fn read_temperature_internal_async(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature_async(Register::ColdJunction).await
    }

    async fn _read_temperature_async(
        &mut self,
        reg: Register,
    ) -> Result<RawTemperature, I2C::Error> {
        let mut data = [0u8, 0u8];
        self.i2c
            .write_read(self.address as u8, &[reg as u8], &mut data)
            .await?;
        Ok(RawTemperature(u16::from_be_bytes(data)))
    }

    pub async fn read_adc_async(&mut self) -> Result<i32, I2C::Error> {
        let mut data = [0u8, 0u8, 0u8];
        self.i2c
            .write_read(self.address as u8, &[Register::RawADCData as u8], &mut data)
            .await?;
        let sign_extend = ((data[0] & 0x80 == 0) as u8).wrapping_sub(1);
        let raw = [sign_extend, data[0], data[1], data[2]];
        Ok(i32::from_be_bytes(raw))
    }

    pub async fn status_async(&mut self) -> Result<Status, I2C::Error> {
        let mut data = 0;
        self.i2c
            .write_read(
                self.address as u8,
                &[Register::Status as u8],
                slice::from_mut(&mut data),
            )
            .await?;
        Ok(Status::from_register(data))
    }

    pub async fn clear_burst_complete_async(&mut self) -> Result<(), I2C::Error> {
        self.clear_status_async(true, false).await
    }

    pub async fn clear_temperature_update_async(&mut self) -> Result<(), I2C::Error> {
        self.clear_status_async(false, true).await
    }

    pub async fn clear_status_async(
        &mut self,
        burst_complete: bool,
        temperature_update: bool,
    ) -> Result<(), I2C::Error> {
        let data = ((burst_complete as u8) << 7) | ((temperature_update as u8) << 6);
        self.i2c
            .write(self.address as u8, &[Register::Status as u8, data])
            .await
    }

    pub async fn sensor_configuration_async(
        &mut self,
    ) -> Result<(ThermocoupleType, FilterCoefficient), I2C::Error> {
        let mut data = 0;
        self.i2c
            .write_read(
                self.address as u8,
                &[Register::SensorConfiguration as u8],
                slice::from_mut(&mut data),
            )
            .await?;

        let thermocouple_type = ThermocoupleType::from_register(data);
        let filter_coefficient = FilterCoefficient::from_register(data);
        Ok((thermocouple_type, filter_coefficient))
    }

    pub async fn set_sensor_configuration_async(
        &mut self,
        thermocouple_type: ThermocoupleType,
        filter_coefficient: FilterCoefficient,
    ) -> Result<(), I2C::Error> {
        let configuration = thermocouple_type.to_register() | filter_coefficient.to_register();
        self.i2c
            .write(
                self.address as u8,
                &[Register::SensorConfiguration as u8, configuration],
            )
            .await
    }

    pub async fn device_configuration_async(
        &mut self,
    ) -> Result<
        (
            ColdJunctionResolution,
            AdcResolution,
            BurstModeSamples,
            ShutdownMode,
        ),
        I2C::Error,
    > {
        let mut data = 0;
        self.i2c
            .write_read(
                self.address as u8,
                &[Register::SensorConfiguration as u8],
                slice::from_mut(&mut data),
            )
            .await?;

        let cold_resolution = ColdJunctionResolution::from_register(data);
        let adc_resolution = AdcResolution::from_register(data);
        let burst_samples = BurstModeSamples::from_register(data);
        let shutdown_mode = ShutdownMode::from_register(data);
        Ok((
            cold_resolution,
            adc_resolution,
            burst_samples,
            shutdown_mode,
        ))
    }

    pub async fn set_device_configuration_async(
        &mut self,
        cold_resolution: ColdJunctionResolution,
        adc_resolution: AdcResolution,
        burst_samples: BurstModeSamples,
        shutdown_mode: ShutdownMode,
    ) -> Result<(), I2C::Error> {
        let configuration = cold_resolution.to_register()
            | adc_resolution.to_register()
            | burst_samples.to_register()
            | shutdown_mode.to_register();
        self.i2c
            .write(
                self.address as u8,
                &[Register::DeviceConfiguration as u8, configuration],
            )
            .await
    }

    pub async fn alert1_configuration_async(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration_async(Register::Alert1Configuration)
            .await
    }

    pub async fn alert2_configuration_async(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration_async(Register::Alert2Configuration)
            .await
    }

    pub async fn alert3_configuration_async(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration_async(Register::Alert3Configuration)
            .await
    }

    pub async fn alert4_configuration_async(&mut self) -> Result<AlertConfig, I2C::Error> {
        self.alert_configuration_async(Register::Alert4Configuration)
            .await
    }

    async fn alert_configuration_async(
        &mut self,
        reg: Register,
    ) -> Result<AlertConfig, I2C::Error> {
        let mut data = [0u8];
        self.i2c
            .write_read(self.address as u8, &[reg as u8], &mut data)
            .await?;
        Ok(AlertConfig::from_register(data[0]))
    }

    pub async fn set_alert1_configuration_async(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration_async(Register::Alert1Configuration, config, clear_interrupt)
            .await
    }

    pub async fn set_alert2_configuration_async(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration_async(Register::Alert2Configuration, config, clear_interrupt)
            .await
    }

    pub async fn set_alert3_configuration_async(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration_async(Register::Alert3Configuration, config, clear_interrupt)
            .await
    }

    pub async fn set_alert4_configuration_async(
        &mut self,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.set_alert_configuration_async(Register::Alert4Configuration, config, clear_interrupt)
            .await
    }

    async fn set_alert_configuration_async(
        &mut self,
        reg: Register,
        config: AlertConfig,
        clear_interrupt: bool,
    ) -> Result<(), I2C::Error> {
        self.i2c
            .write(
                self.address as u8,
                &[reg as u8, config.to_register(clear_interrupt)],
            )
            .await
    }

    pub async fn alert1_hysteresis_async(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis_async(Register::Alert1Configuration)
            .await
    }

    pub async fn alert2_hysteresis_async(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis_async(Register::Alert2Configuration)
            .await
    }

    pub async fn alert3_hysteresis_async(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis_async(Register::Alert3Configuration)
            .await
    }

    pub async fn alert4_hysteresis_async(&mut self) -> Result<u8, I2C::Error> {
        self.alert_hysteresis_async(Register::Alert4Configuration)
            .await
    }

    async fn alert_hysteresis_async(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data = 0;
        self.i2c
            .write_read(self.address as u8, &[reg as u8], slice::from_mut(&mut data))
            .await?;
        Ok(data)
    }

    pub async fn set_alert1_hysteresis_async(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis_async(Register::Alert1Configuration, hysteresis)
            .await
    }

    pub async fn set_alert2_hysteresis_async(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis_async(Register::Alert2Configuration, hysteresis)
            .await
    }

    pub async fn set_alert3_hysteresis_async(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis_async(Register::Alert3Configuration, hysteresis)
            .await
    }

    pub async fn set_alert4_hysteresis_async(&mut self, hysteresis: u8) -> Result<(), I2C::Error> {
        self.set_alert_hysteresis_async(Register::Alert4Configuration, hysteresis)
            .await
    }

    async fn set_alert_hysteresis_async(
        &mut self,
        reg: Register,
        hysteresis: u8,
    ) -> Result<(), I2C::Error> {
        self.i2c
            .write(self.address as u8, &[reg as u8, hysteresis])
            .await
    }

    pub async fn alert1_limit_async(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature_async(Register::Alert1Configuration)
            .await
    }

    pub async fn alert2_limit_async(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature_async(Register::Alert2Configuration)
            .await
    }

    pub async fn alert3_limit_async(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature_async(Register::Alert3Configuration)
            .await
    }

    pub async fn alert4_limit_async(&mut self) -> Result<RawTemperature, I2C::Error> {
        self._read_temperature_async(Register::Alert4Configuration)
            .await
    }

    pub async fn set_alert1_limit_async(
        &mut self,
        limit: RawTemperature,
    ) -> Result<(), I2C::Error> {
        self.set_alert_limit_async(Register::Alert1Configuration, limit)
            .await
    }

    pub async fn set_alert2_limit_async(
        &mut self,
        limit: RawTemperature,
    ) -> Result<(), I2C::Error> {
        self.set_alert_limit_async(Register::Alert2Configuration, limit)
            .await
    }

    pub async fn set_alert3_limit_async(
        &mut self,
        limit: RawTemperature,
    ) -> Result<(), I2C::Error> {
        self.set_alert_limit_async(Register::Alert3Configuration, limit)
            .await
    }

    pub async fn set_alert4_limit_async(
        &mut self,
        limit: RawTemperature,
    ) -> Result<(), I2C::Error> {
        self.set_alert_limit_async(Register::Alert4Configuration, limit)
            .await
    }

    async fn set_alert_limit_async(
        &mut self,
        reg: Register,
        limit: RawTemperature,
    ) -> Result<(), I2C::Error> {
        let data = limit.0.to_be_bytes();
        self.i2c
            .write(self.address as u8, &[reg as u8, data[0], data[1]])
            .await
    }
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
    DeviceID = 0b0010_0000,
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

impl From<Temperature> for RawTemperature {
    fn from(raw: Temperature) -> Self {
        Self(((raw.0 * 16.0) as i16) as u16)
    }
}

#[derive(Clone)]
pub struct Status {
    pub burst_complete: bool,
    pub temperature_update: bool,
    pub short_circuit: bool,
    pub input_range_exceeded: bool,
    pub alert: [bool; 4],
}

impl Status {
    fn from_register(reg: u8) -> Self {
        Self {
            burst_complete: reg & 0x80 != 0,
            temperature_update: reg & 0x40 != 0,
            short_circuit: reg & 0x20 != 0,
            input_range_exceeded: reg & 0x10 != 0,
            alert: [
                reg & 0x01 != 0,
                reg & 0x02 != 0,
                reg & 0x04 != 0,
                reg & 0x08 != 0,
            ],
        }
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

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ColdJunctionResolution {
    /// 0.0625 C per bit
    High = 0,
    /// 0.25 C per bit
    Low = 1,
}

impl ColdJunctionResolution {
    fn to_register(&self) -> u8 {
        (*self as u8) << 7
    }

    fn from_register(reg: u8) -> Self {
        if reg & 0x80 != 0 {
            ColdJunctionResolution::Low
        } else {
            ColdJunctionResolution::High
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AdcResolution {
    /// 320 ms update time
    Bit18 = 0b00,
    /// 80 ms update time
    Bit16 = 0b01,
    /// 20 ms update time
    Bit14 = 0b10,
    /// 5 ms update time
    Bit12 = 0b11,
}

impl AdcResolution {
    fn to_register(&self) -> u8 {
        (*self as u8) << 5
    }

    fn from_register(reg: u8) -> Self {
        match (reg & 0x60) >> 5 {
            0b01 => AdcResolution::Bit16,
            0b10 => AdcResolution::Bit14,
            0b11 => AdcResolution::Bit12,
            _ => AdcResolution::Bit18,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
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

impl BurstModeSamples {
    fn to_register(&self) -> u8 {
        (*self as u8) << 2
    }

    fn from_register(reg: u8) -> Self {
        match (reg & 0x1C) >> 2 {
            0b001 => BurstModeSamples::Sample2,
            0b010 => BurstModeSamples::Sample4,
            0b011 => BurstModeSamples::Sample8,
            0b100 => BurstModeSamples::Sample16,
            0b101 => BurstModeSamples::Sample32,
            0b110 => BurstModeSamples::Sample64,
            0b111 => BurstModeSamples::Sample128,
            _ => BurstModeSamples::Sample1,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ShutdownMode {
    NormalMode = 0b00,
    ShutdownMode = 0b01,
    BurstMode = 0b10,
}

impl ShutdownMode {
    fn to_register(&self) -> u8 {
        *self as u8
    }

    fn from_register(reg: u8) -> Self {
        match reg & 0x6 {
            0b01 => ShutdownMode::ShutdownMode,
            0b10 => ShutdownMode::BurstMode,
            _ => ShutdownMode::NormalMode,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AlertMonitor {
    HotJunction = 0,
    ColdJunction = 1,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AlertDirection {
    Heating = 0,
    Cooling = 1,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AlertMode {
    Comparator = 0,
    Interrupt = 1,
}

#[derive(Clone)]
pub struct AlertConfig {
    pub monitor: AlertMonitor,
    pub direction: AlertDirection,
    pub active_high: bool,
    pub mode: AlertMode,
    pub enable: bool,
}

impl AlertConfig {
    fn to_register(&self, clear_interrupt: bool) -> u8 {
        ((clear_interrupt as u8) << 7)
            | ((self.monitor as u8) << 4)
            | ((self.direction as u8) << 3)
            | ((self.active_high as u8) << 2)
            | ((self.mode as u8) << 1)
            | (self.enable as u8)
    }

    fn from_register(reg: u8) -> Self {
        let monitor = if reg & 0x10 != 0 {
            AlertMonitor::ColdJunction
        } else {
            AlertMonitor::HotJunction
        };
        let direction = if reg & 0x08 != 0 {
            AlertDirection::Cooling
        } else {
            AlertDirection::Heating
        };
        let mode = if reg & 0x02 != 0 {
            AlertMode::Interrupt
        } else {
            AlertMode::Comparator
        };

        Self {
            monitor,
            direction,
            active_high: reg & 0x04 != 0,
            mode,
            enable: reg & 0x01 != 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceAddress {
    AD0 = 0b110_0000,
    AD1 = 0b110_0001,
    AD2 = 0b110_0010,
    AD3 = 0b110_0011,
    AD4 = 0b110_0100,
    AD5 = 0b110_0101,
    AD6 = 0b110_0110,
    AD7 = 0b110_0111,
}

impl TryFrom<u8> for DeviceAddress {
    type Error = ();

    fn try_from(value: u8) -> Result<DeviceAddress, ()> {
        match value {
            0b110_0000 => Ok(DeviceAddress::AD0),
            0b110_0001 => Ok(DeviceAddress::AD1),
            0b110_0010 => Ok(DeviceAddress::AD2),
            0b110_0011 => Ok(DeviceAddress::AD3),
            0b110_0100 => Ok(DeviceAddress::AD4),
            0b110_0101 => Ok(DeviceAddress::AD5),
            0b110_0110 => Ok(DeviceAddress::AD6),
            0b110_0111 => Ok(DeviceAddress::AD7),
            _ => Err(()),
        }
    }
}

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
        let config_byte = ColdJunctionResolution::Low.to_register()
            | AdcResolution::Bit14.to_register()
            | BurstModeSamples::Sample16.to_register()
            | ShutdownMode::BurstMode.to_register();
        assert_eq!(config_byte, 0b1101_0010)
    }
}
