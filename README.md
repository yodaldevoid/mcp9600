# mcp9600
Basic I2C driver for the Microchip Technology MCP960X Thermocouple Amplifier Chip
---
## ! This crate is a WIP and has minimal functionality !

Currently, the following features are implemented:
- Reading the device ID
- Configuring the Sensor portion of the device
- Configuring the measurement profile of the device
- Performing basic hot junction temperature measurements (results in an f32)

TODO:
- Read the status register and pass the result to the user
- Enable configuration of the Alert registers
