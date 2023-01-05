use bitvec::prelude::*;

pub fn temperature_conversion(buffer: &[u8]) -> f32 {
    let sign = buffer[0].view_bits::<Lsb0>()[0];
    match sign {
        false => buffer[0] as f32 * 16.0 + buffer[1] as f32 / 16.0,
        true => (buffer[0] as f32 * 16.0 + buffer[1] as f32 / 16.0) - 4096.0,
    }
}

pub fn sensor_configuration(
    thermocoupletype: ThermocoupleType,
    filtercoefficient: FilterCoefficient,
) -> u8 {
    let configuration: u8 = thermocoupletype as u8 | filtercoefficient as u8;
    return configuration;
}

pub fn device_configuration(
    coldjunctionresolution: ColdJunctionResolution,
    adcresolution: ADCResolution,
    burstmodesamples: BurstModeSamples,
    shutdownmode: ShutdownMode,
) -> u8 {
    let configuration = coldjunctionresolution as u8
        | adcresolution as u8
        | burstmodesamples as u8
        | shutdownmode as u8;
    return configuration;
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
    Bit18 = 0b0000_0000,
    Bit16 = 0b0010_0000,
    Bit14 = 0b0100_0000,
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
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_positive_temperature_conversion() {
        let data = [0b0000_1100u8, 0b0101_0010u8];
        let temperature = temperature_conversion(&data);
        assert_eq!(temperature, 197.125);
    }
    #[test]
    fn test_negative_temperature_conversion() {
        let data = [0b1111_0011u8, 0b1010_1101u8];
        let temperature = temperature_conversion(&data);
        assert_eq!(temperature, -197.1875);
    }
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
