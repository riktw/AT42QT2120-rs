# AT42QT2120 Rust driver
Platform agnostic Rust driver for the AT42QT2120

## The AT42QT2120
The AT42QT2120 is a touch driver with 12 channels, of which 3 can be used as a slider or a wheel.
It has an I2C interface.
Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/doc9634.pdf


## Status
Basic support works and is tested on hardware:
- [x] Support reading keys
- [x] Support reading raw key values
- [x] Support reading slider
- [x] Configurable slider/wheel
- [x] Configurable key threshold
- [x] Enable/disable keys
- [x] Configurable detection integrator

Most advanced configuration is not supported:
- [ ] Configurable drift compensation
- [ ] Configurable touch recal delay
- [ ] Low power mode
- [ ] Using Change pin as input
- [ ] Configurable key oversampling

I am new to Rust, so the code quality is probably not great.

## License
Licensed under GNU Lesser General Public License v3.0.
All contributions intentionally submitted shall be licensed under the same license.
