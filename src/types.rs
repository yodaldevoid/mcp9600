#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Temperature(pub f32);

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawTemperature(pub u16);

impl From<RawTemperature> for Temperature {
    fn from(raw: RawTemperature) -> Self {
        Self((raw.0 as i16) as f32 / 16.0)
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
}
