use bitvec::prelude::*;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Temperature(pub f32);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawTemperature {
    pub msb: u8,
    pub lsb: u8,
}

impl From<RawTemperature> for Temperature {
    fn from(other: RawTemperature) -> Self {
        Temperature::from_raw(other)
    }
}

impl Temperature {
    /// Create a new `Temperature` from a raw measurement result.
    pub fn from_raw(raw: RawTemperature) -> Self {
        Self(convert_temperature(raw))
    }
}

#[inline]
fn convert_temperature(buffer: RawTemperature) -> f32 {
    let sign = buffer.msb.view_bits::<Lsb0>()[0];
    match sign {
        false => buffer.msb as f32 * 16.0 + buffer.lsb as f32 / 16.0,
        true => (buffer.msb as f32 * 16.0 + buffer.lsb as f32 / 16.0) - 4096.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_positive_convert_temperature() {
        let data = RawTemperature {
            msb: 0b0000_1100u8,
            lsb: 0b0101_0010u8,
        };
        let temperature = convert_temperature(data);
        assert_eq!(temperature, 197.125);
    }
    #[test]
    fn test_negative_convert_temperature() {
        let data = RawTemperature {
            msb: 0b1111_0011u8,
            lsb: 0b1010_1101u8,
        };
        let temperature = convert_temperature(data);
        assert_eq!(temperature, -197.1875);
    }
    #[test]
    fn test_from_raw() {
        let raw = RawTemperature {
            msb: 0b1000_1011u8,
            lsb: 0b1010_1110u8,
        };
        let temperature = Temperature::from(raw);
        assert_eq!(temperature, Temperature(-1861.125))
    }
}
