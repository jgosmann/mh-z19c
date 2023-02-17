# MH-Z19C crate

Pure rust implementation to read out the Winsen MH-Z19C CO2 sensor.

This crate provides an API to read-out the nondispersive infrared (NDIR)
CO₂ sensor MH-Z19C by Winsen via the serial (UART) interface.

The provided API supports non-blocking usage and is `no_std`.

## Examples
```rust
use mh_z19c::MhZ19C;
use nb::block;

let mut co2sensor = MhZ19C::new(uart);
let co2 = block!(co2sensor.read_co2_ppm())?;
println!("CO₂ concentration: {}ppm", co2);
```

To activate features of a sensor with a firmware of version 5:

```rust
let mut co2sensor = block!(co2sensor.upgrade_to_v5())?;
let co2_temp = block!(co2sensor.read_co2_and_temp())?;
println!("Temperature: {}°C", co2_temp.temp_celsius);
```

## Versioning

This crate uses [Semantic Versioning](https://semver.org/).

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.