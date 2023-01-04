#![deny(unsafe_code)]
#![no_std]

mod encode;
mod types;

use bitvec::prelude::*;
use embedded_hal::blocking::i2c;
