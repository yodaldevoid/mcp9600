use bitfield;

#[derive(Debug, Copy, Clone)]
pub struct Register {
    /// points to a specific register in the sensor
    ptr: u8,
    /// register contents, either 1 or 2 bytes
    buf: [u8; 2],
    /// actual register size in bytes, either 1 or 2
    len: u8,
}

impl Register {
    pub fn new(ptr: u8, len: u8) -> Self {
        if ptr == 0 || ptr > 0b100000u8 {
            panic!("invalid pointer value: reserved")
        }

        if len > 2 {
            panic!("length > 2")
        }
    }
}
