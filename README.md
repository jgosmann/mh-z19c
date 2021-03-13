[![Build Status](https://travis-ci.com/jgosmann/mh-z19c.svg?branch=main)](https://travis-ci.com/jgosmann/mh-z19c)

# MH-Z19C crate

Pure rust implementation to read out the Winsen MH-Z19C CO2 sensor.

This crate provides an API to read-out the nondispersive infrared (NDIR)
CO₂ sensor MH-Z19C by Winsen via the serial (UART) interface.

The provided API supports non-blocking usage and is `no_std`.

## Example
```rust
use mh_z19c::MhZ19C;
use nb::block;

let mut co2sensor = MhZ19C::new(uart);
let co2 = block!(co2sensor.read_co2_ppm())?;
println!("CO₂ concenration: {}ppm", co2);
```