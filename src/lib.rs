//! Crate to read out the Winsen MH-Z19C CO2 sensor.
//!
//! This crate provides an API to read-out the nondispersive infrared (NDIR)
//! CO₂ sensor MH-Z19C by Winsen via the serial (UART) interface.
//!
//! The provided API supports non-blocking usage and is `no_std`.
//!
//!
//! # Examples
//!
//! ```
//! use mh_z19c::MhZ19C;
//! use nb::block;
//!
//! # use test_support::{create_serial_mock_returning, READ_CO2_RESPONSE};
//! # fn main() -> Result<(), mh_z19c::Error<String>> {
//! # let uart = create_serial_mock_returning(&READ_CO2_RESPONSE);
//! let mut co2sensor = MhZ19C::new(uart);
//! let co2 = block!(co2sensor.read_co2_ppm())?;
//! println!("CO₂ concentration: {}ppm", co2);
//! # Ok(())
//! # }
//! ```
//!
//! To activate features of a sensor with a firmware of version 5:
//!
//! ```
//! # use mh_z19c::MhZ19C;
//! # use nb::block;
//! #
//! # use test_support::{
//! #     serial_mock::SerialMock,
//! #     FIRMWARE_0515_RESPONSE,
//! #     READ_CO2_AND_TEMPERATURE_RESPONSE
//! # };
//! # fn main() -> Result<(), mh_z19c::Error<String>> {
//! # let mut responses: Vec<nb::Result<u8, String>> = FIRMWARE_0515_RESPONSE
//! #     .iter()
//! #     .chain(READ_CO2_AND_TEMPERATURE_RESPONSE.iter())
//! #     .copied()
//! #     .map(Ok)
//! #     .collect();
//! # let uart = SerialMock::new(responses, vec![Ok(()); 27]);
//! # let mut co2sensor = MhZ19C::new(uart);
//! let mut co2sensor = block!(co2sensor.upgrade_to_v5())?;
//! let co2_temp = block!(co2sensor.read_co2_and_temp())?;
//! println!("Temperature: {}°C", co2_temp.temp_celsius);
//! # Ok(())
//! # }
//! ```
//!
//!
//! # no_std
//!
//! This crate is `no_std` by default, unless the `std` feature is activated.
//! Currently, the `std` feature will only add [`std::error::Error`] trait
//! implementations to the error types.
//!
//!
//! # Versioning
//!
//! This crate uses [Semantic Versioning](https://semver.org/).

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(doc)]
extern crate std;

#[macro_use]
extern crate lazy_static;

use crate::command::Command;
use crate::frame::{Frame, ValidateFrameError};
use crate::nb_comm::{NbFuture, WriteAll, WriteAndReadResponse};
use core::convert::TryInto;
use core::fmt::{self, Display};
use embedded_hal::serial::{Read, Write};

pub mod command;
pub mod frame;
mod nb_comm;

lazy_static! {
    static ref READ_CO2_AND_TEMPERATURE: Frame = Command::ReadCo2AndTemperature.into();
    static ref READ_CO2: Frame = Command::ReadCo2.into();
    static ref GET_FIRMWARE_VERSION: Frame = Command::GetFirmwareVersion.into();
}

/// Methods supported by all MH-Z19C sensors.
pub trait BaseApi<E> {
    /// Reads and returns the CO₂ concentration in parts-per-million (ppm).
    fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>>;

    /// Retrieves the firmware version of the sensor.
    fn get_firmware_version(&mut self) -> nb::Result<[u8; 4], Error<E>>;

    /// Activates or deactivates the sensor's self-calibration mode.
    ///
    /// See the sensor's data sheet for more information on self-calibration
    /// and hand-operated mode.
    fn set_self_calibrate(&mut self, enabled: bool) -> nb::Result<(), Error<E>>;
}

/// Data-transfer object for combined measurement of CO₂ and temperature.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Co2AndTemperature {
    /// CO₂ concentration in parts per million (ppm).
    pub co2_ppm: u16,
    /// Temperature in degrees Celsius (°C).
    pub temp_celsius: f32,
}

/// Methods supported by all MH-Z19C sensors with firmware 5.
pub trait Firmware5Api<E>: BaseApi<E> {
    /// Reads the CO₂ concentration and temperature.
    fn read_co2_and_temp(&mut self) -> nb::Result<Co2AndTemperature, Error<E>>;
}

/// Driver for the MH-Z19C sensor.
#[derive(Debug)]
pub struct MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    state: MhZ19CState<'a, U, E>,
    uart: Option<U>,
}

#[derive(Debug)]
enum MhZ19CState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Idle,
    ReadCo2AndTemperature(WriteAndReadResponse<U, E, &'a [u8], [u8; 9]>),
    ReadCo2(WriteAndReadResponse<U, E, &'a [u8], [u8; 9]>),
    GetFirmwareVersion(WriteAndReadResponse<U, E, &'a [u8], [u8; 9]>),
    SetSelfCalibrate(WriteAll<U, E, Frame>),
}

impl<'a, U, E> Default for MhZ19CState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    fn default() -> Self {
        Self::Idle
    }
}

impl<'a, U, E> MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    /// Create a new instance.
    ///
    /// * `uart`: Serial (UART) interface for communication with the sensor.
    pub fn new(uart: U) -> Self {
        Self {
            state: MhZ19CState::default(),
            uart: Some(uart),
        }
    }
}

impl<'a, U, E> MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    /// Reads and returns the CO₂ concentration in parts-per-million (ppm).
    pub fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        BaseApi::read_co2_ppm(self)
    }

    /// Retrieves the firmware version of the sensor.
    pub fn get_firmware_version(&mut self) -> nb::Result<[u8; 4], Error<E>> {
        BaseApi::get_firmware_version(self)
    }

    /// Activates or deactivates the sensor's self-calibration mode.
    ///
    /// See the sensor's data sheet for more information on self-calibration
    /// and hand-operated mode.
    pub fn set_self_calibrate(&mut self, enabled: bool) -> nb::Result<(), Error<E>> {
        BaseApi::set_self_calibrate(self, enabled)
    }

    /// Returns the owned UART interface.vec!
    ///
    /// Note that this might leave the interface with partially written or read
    /// bytes on the UART interface if not all MH-Z19C commands have been polled
    /// to completion (i.e. the last command call did not return
    /// [`nb::Error::WouldBlock`]).
    pub fn into_inner(mut self) -> U {
        use MhZ19CState::*;
        match self.state {
            Idle => self.uart.take().unwrap(),
            ReadCo2AndTemperature(future) => future.into_return_value().0,
            ReadCo2(future) => future.into_return_value().0,
            GetFirmwareVersion(future) => future.into_return_value().0,
            SetSelfCalibrate(future) => future.into_return_value(),
        }
    }

    /// Will make the [`Firmware5Api`] capabilities available.
    ///
    /// If the sensor firmware is not at least of version 5, an error will be
    /// returned.
    pub fn upgrade_to_v5<'b>(&'b mut self) -> nb::Result<MhZ19CFw5<'a, 'b, U, E>, Error<E>> {
        let fw_version = self.get_firmware_version()?;

        if fw_version[1] >= b'5' {
            Ok(MhZ19CFw5 { mh_z19c: self })
        } else {
            Err(nb::Error::Other(Error::NotSupportedByFirmware(fw_version)))
        }
    }

    fn poll(&mut self) -> nb::Result<(), Error<E>> {
        use MhZ19CState::*;
        match &mut self.state {
            Idle => Ok(()),
            ReadCo2AndTemperature(future) => future.poll(),
            ReadCo2(future) => future.poll(),
            GetFirmwareVersion(future) => future.poll(),
            SetSelfCalibrate(future) => future.poll(),
        }
        .map_err(|err| err.map(Error::UartError))
    }

    fn recover_uart(&mut self, state: MhZ19CState<U, E>) {
        use MhZ19CState::*;
        match state {
            Idle => (),
            ReadCo2AndTemperature(future) => self.uart = Some(future.into_return_value().0),
            ReadCo2(future) => self.uart = Some(future.into_return_value().0),
            GetFirmwareVersion(future) => self.uart = Some(future.into_return_value().0),
            SetSelfCalibrate(future) => self.uart = Some(future.into_return_value()),
        }
    }

    fn unpack_return_frame(command: Command, frame: &Frame) -> Result<&[u8], Error<E>> {
        frame.validate().map_err(Error::ValidateFrameError)?;
        if !frame.is_response() {
            Err(Error::NotAResponse)
        } else if frame.op_code() != command.op_code() {
            Err(Error::OpCodeMismatch {
                expected: command.op_code(),
                got: frame.op_code(),
            })
        } else {
            Ok(frame.data())
        }
    }
}

impl<'a, U, E> BaseApi<E> for MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        loop {
            if let MhZ19CState::Idle = &mut self.state {
                let uart = self.uart.take().unwrap();
                self.state = MhZ19CState::ReadCo2(WriteAndReadResponse::new(
                    uart,
                    READ_CO2.as_ref(),
                    [0u8; 9],
                    9,
                ));
            }

            self.poll()?;

            let state = core::mem::take(&mut self.state);
            if let MhZ19CState::ReadCo2(future) = state {
                let (uart, buf) = future.into_return_value();
                self.uart = Some(uart);
                let frame = Frame::new(buf);
                let data = Self::unpack_return_frame(Command::ReadCo2, &frame)
                    .map_err(nb::Error::Other)?;
                return Ok(u16::from_be_bytes(data[..2].try_into().unwrap()));
            } else {
                self.recover_uart(state);
            }
        }
    }

    fn get_firmware_version(&mut self) -> nb::Result<[u8; 4], Error<E>> {
        loop {
            if let MhZ19CState::Idle = &mut self.state {
                let uart = self.uart.take().unwrap();
                self.state = MhZ19CState::GetFirmwareVersion(WriteAndReadResponse::new(
                    uart,
                    GET_FIRMWARE_VERSION.as_ref(),
                    [0u8; 9],
                    9,
                ));
            }

            self.poll()?;

            let state = core::mem::take(&mut self.state);
            if let MhZ19CState::GetFirmwareVersion(future) = state {
                let (uart, buf) = future.into_return_value();
                self.uart = Some(uart);
                let frame = Frame::new(buf);
                let data = Self::unpack_return_frame(Command::GetFirmwareVersion, &frame)
                    .map_err(nb::Error::Other)?;
                let ret_data = [data[0], data[1], data[2], data[3]];
                return Ok(ret_data);
            } else {
                self.recover_uart(state);
            }
        }
    }

    fn set_self_calibrate(&mut self, enabled: bool) -> nb::Result<(), Error<E>> {
        loop {
            if let MhZ19CState::Idle = &mut self.state {
                let uart = self.uart.take().unwrap();
                let frame: Frame = Command::SetSelfCalibrate(enabled).into();
                self.state = MhZ19CState::SetSelfCalibrate(WriteAll::new(uart, frame));
            }

            self.poll()?;

            let state = core::mem::take(&mut self.state);
            if let MhZ19CState::SetSelfCalibrate(future) = state {
                self.uart = Some(future.into_return_value());
                return Ok(());
            } else {
                self.recover_uart(state);
            }
        }
    }
}

/// Driver for the MH-Z19C sensor with firmware 5 capabilities.
pub struct MhZ19CFw5<'a, 'b, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    mh_z19c: &'b mut MhZ19C<'a, U, E>,
}

impl<'a, 'b, U, E> MhZ19CFw5<'a, 'b, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    /// Reads and returns the CO₂ concentration in parts-per-million (ppm).
    pub fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        BaseApi::read_co2_ppm(self)
    }

    /// Retrieves the firmware version of the sensor.
    pub fn get_firmware_version(&mut self) -> nb::Result<[u8; 4], Error<E>> {
        BaseApi::get_firmware_version(self)
    }

    /// Activates or deactivates the sensor's self-calibration mode.
    ///
    /// See the sensor's data sheet for more information on self-calibration
    /// and hand-operated mode.
    pub fn set_self_calibrate(&mut self, enabled: bool) -> nb::Result<(), Error<E>> {
        BaseApi::set_self_calibrate(self, enabled)
    }

    /// Reads the CO₂ concentration and temperature.
    pub fn read_co2_and_temp(&mut self) -> nb::Result<Co2AndTemperature, Error<E>> {
        Firmware5Api::read_co2_and_temp(self)
    }
}

impl<'a, 'b, U, E> BaseApi<E> for MhZ19CFw5<'a, 'b, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        self.mh_z19c.read_co2_ppm()
    }

    fn get_firmware_version(&mut self) -> nb::Result<[u8; 4], Error<E>> {
        self.mh_z19c.get_firmware_version()
    }

    fn set_self_calibrate(&mut self, enabled: bool) -> nb::Result<(), Error<E>> {
        self.mh_z19c.set_self_calibrate(enabled)
    }
}

impl<'a, 'b, U, E> Firmware5Api<E> for MhZ19CFw5<'a, 'b, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    fn read_co2_and_temp(&mut self) -> nb::Result<Co2AndTemperature, Error<E>> {
        loop {
            if let MhZ19CState::Idle = &mut self.mh_z19c.state {
                let uart = self.mh_z19c.uart.take().unwrap();
                self.mh_z19c.state = MhZ19CState::ReadCo2AndTemperature(WriteAndReadResponse::new(
                    uart,
                    READ_CO2_AND_TEMPERATURE.as_ref(),
                    [0u8; 9],
                    9,
                ));
            }

            self.mh_z19c.poll()?;

            let state = core::mem::take(&mut self.mh_z19c.state);
            if let MhZ19CState::ReadCo2AndTemperature(future) = state {
                let (uart, buf) = future.into_return_value();
                self.mh_z19c.uart = Some(uart);
                let frame = Frame::new(buf);
                let data =
                    MhZ19C::<'a, U, E>::unpack_return_frame(Command::ReadCo2AndTemperature, &frame)
                        .map_err(nb::Error::Other)?;

                let co2_ppm = u16::from_be_bytes(data[2..4].try_into().unwrap());
                let temp_celsius =
                    f32::from(u16::from_be_bytes(data[..2].try_into().unwrap())) / 100.0;

                return Ok(Co2AndTemperature {
                    co2_ppm,
                    temp_celsius,
                });
            } else {
                self.mh_z19c.recover_uart(state);
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error<T> {
    /// The frame of a command or return value was invalid.
    ValidateFrameError(ValidateFrameError),
    /// The received data is not response.
    NotAResponse,
    /// Received a response for a different op code than expected.
    OpCodeMismatch { expected: u8, got: u8 },
    /// Communication error caused by the UART/serial interface.
    UartError(T),
    /// Cannot upgrade to requested firmware version.
    /// Firmware version reported by the sensor will be included.
    NotSupportedByFirmware([u8; 4]),
}

impl<T: Display> Display for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ValidateFrameError(err) => write!(f, "frame error: {err}"),
            Self::NotAResponse => write!(f, "expected response, but got command"),
            Self::OpCodeMismatch { expected, got } => write!(
                f,
                "expected response for op code 0x{expected:x}, but got op code 0x{got:x}"
            ),
            Self::UartError(err) => write!(f, "UART communication error: {err}"),
            Self::NotSupportedByFirmware(version) => {
                write!(
                    f,
                    "not supported by firmware version {}",
                    core::str::from_utf8(version).unwrap_or("<invalid version string>")
                )
            }
        }
    }
}

#[cfg(std)]
impl std::error::Error for Error {}

#[cfg(test)]
#[macro_use]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;

    use nb::block;
    use std::string::String;
    use std::vec::Vec;
    use test_support::serial_mock::SerialMock;
    use test_support::{
        create_serial_mock_returning, FIRMWARE_0400_RESPONSE, FIRMWARE_0515_RESPONSE,
        READ_CO2_AND_TEMPERATURE_RESPONSE, READ_CO2_RESPONSE, SELF_CALIBRATE_ON_COMMAND,
    };

    #[test]
    fn test_read_co2() {
        let uart = create_serial_mock_returning(&READ_CO2_RESPONSE);
        let mut co2sensor = MhZ19C::new(uart);
        let co2 = block!(co2sensor.read_co2_ppm());
        let uart = co2sensor.into_inner();
        assert_eq!(uart.write_buf, &*READ_CO2.as_ref());
        assert_eq!(co2, Ok(800));
    }

    #[test]
    fn test_read_co2_uart_error() {
        let uart = SerialMock::new(
            vec![Err(nb::Error::Other("No more data.".into()))],
            vec![Ok(()); 9],
        );
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.read_co2_ppm()),
            Err(Error::UartError("No more data.".into()))
        );
    }

    #[test]
    fn test_read_co2_invalid_start_byte() {
        let mut response = READ_CO2_RESPONSE.clone();
        response[0] = 0x00;
        let uart = create_serial_mock_returning(&response);
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.read_co2_ppm()),
            Err(Error::ValidateFrameError(
                ValidateFrameError::InvalidStartByte(0x00)
            ))
        );
    }

    #[test]
    fn test_read_co2_invalid_checksum() {
        let mut response = READ_CO2_RESPONSE.clone();
        response[8] = 0x00;
        let uart = create_serial_mock_returning(&response);
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.read_co2_ppm()),
            Err(Error::ValidateFrameError(
                ValidateFrameError::InvalidChecksum {
                    expected: READ_CO2_RESPONSE[8],
                    actual: 0x00
                }
            ))
        );
    }

    #[test]
    fn test_set_self_calibrate() -> Result<(), Error<String>> {
        let uart = create_serial_mock_returning(&[]);
        let mut co2sensor = MhZ19C::new(uart);
        block!(co2sensor.set_self_calibrate(true))?;
        let uart = co2sensor.into_inner();
        assert_eq!(uart.write_buf, &*SELF_CALIBRATE_ON_COMMAND.as_ref());
        Ok(())
    }

    #[test]
    fn test_set_self_calibrate_uart_error() {
        let uart = SerialMock::new(vec![], vec![Err(nb::Error::Other("No more data.".into()))]);
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.set_self_calibrate(true)),
            Err(Error::UartError("No more data.".into()))
        );
    }

    #[test]
    fn test_into_inner_during_read() {
        let uart = SerialMock::new(vec![], vec![Err(nb::Error::WouldBlock)]);
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(co2sensor.read_co2_ppm(), Err(nb::Error::WouldBlock));
        let _ = co2sensor.into_inner(); // Must not panic
    }

    #[test]
    fn test_ignore_read_co2_result_by_polling_set_self_calibrate() {
        let mut read_data = Vec::with_capacity(10);
        read_data.push(Err(nb::Error::WouldBlock));
        read_data.extend(READ_CO2_RESPONSE.iter().copied().map(Ok));
        let uart = SerialMock::new(read_data, vec![Ok(()); 2 * 9]);
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(co2sensor.read_co2_ppm(), Err(nb::Error::WouldBlock));
        assert_eq!(block!(co2sensor.set_self_calibrate(true)), Ok(()));
    }

    #[test]
    fn test_read_co2_without_waiting_for_set_self_calibrate() {
        let mut write_return_values = vec![Ok(()); 2 * 9 + 1];
        write_return_values[0] = Err(nb::Error::WouldBlock);
        let uart = SerialMock::new(
            READ_CO2_RESPONSE.iter().copied().map(Ok).collect(),
            write_return_values,
        );
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            co2sensor.set_self_calibrate(true),
            Err(nb::Error::WouldBlock)
        );
        assert_eq!(block!(co2sensor.read_co2_ppm()), Ok(800));
    }

    #[test]
    fn test_get_firmware_version() {
        let uart = create_serial_mock_returning(&FIRMWARE_0515_RESPONSE);
        let mut co2sensor = MhZ19C::new(uart);
        let firmware = block!(co2sensor.get_firmware_version());
        let uart = co2sensor.into_inner();
        assert_eq!(uart.write_buf, &*GET_FIRMWARE_VERSION.as_ref());
        assert_eq!(firmware, Ok(*b"0515"));
    }

    #[test]
    fn test_get_firmware_version_error() {
        let uart = SerialMock::new(
            vec![Err(nb::Error::Other("No more data.".into()))],
            vec![Ok(()); 9],
        );
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.get_firmware_version()),
            Err(Error::UartError("No more data.".into()))
        );
    }

    #[test]
    fn test_upgrade_to_v5() {
        let uart = create_serial_mock_returning(&FIRMWARE_0515_RESPONSE);
        let mut co2sensor = MhZ19C::new(uart);
        block!(co2sensor.upgrade_to_v5()).unwrap();
    }

    #[test]
    fn test_upgrade_to_v5_error() {
        let uart = create_serial_mock_returning(&FIRMWARE_0400_RESPONSE);
        let mut co2sensor = MhZ19C::new(uart);
        assert_eq!(
            block!(co2sensor.upgrade_to_v5()).err(),
            Some(Error::NotSupportedByFirmware(*b"0400"))
        );
    }

    #[test]
    fn test_read_co2_and_temperature() {
        let mut responses: Vec<nb::Result<u8, String>> = FIRMWARE_0515_RESPONSE
            .iter()
            .chain(READ_CO2_AND_TEMPERATURE_RESPONSE.iter())
            .copied()
            .map(Ok)
            .collect();
        responses.push(Err(nb::Error::Other("No more data.".into())));
        let uart = SerialMock::new(responses, vec![Ok(()); 27]);
        let mut co2sensor = MhZ19C::new(uart);
        let mut co2sensor = block!(co2sensor.upgrade_to_v5()).unwrap();

        assert_eq!(
            block!(co2sensor.read_co2_and_temp()),
            Ok(Co2AndTemperature {
                co2_ppm: 800,
                temp_celsius: 24.
            })
        );
        assert_eq!(
            block!(co2sensor.read_co2_and_temp()),
            Err(Error::UartError("No more data.".into()))
        );
    }
}
