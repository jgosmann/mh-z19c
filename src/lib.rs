//! Crate to read out the Winsen MH-Z19C CO2 sensor.
//!
//! This crate provides an API to read-out the nondispersive infrared (NDIR)
//! CO₂ sensor MH-Z19C by Winsen via the serial (UART) interface.
//!
//! The provided API supports non-blocking usage and is `no_std`.
//!
//! # Example
//! ```
//! use mh_z19c::MhZ19C;
//! use nb::block;
//!
//! # use test_support::{create_serial_mock_returning, READ_CO2_RESPONSE};
//! # fn main() -> Result<(), mh_z19c::Error<String>> {
//! # let uart = create_serial_mock_returning(&READ_CO2_RESPONSE);
//! let mut co2sensor = MhZ19C::new(uart);
//! let co2 = block!(co2sensor.read_co2_ppm())?;
//! println!("CO₂ concenration: {}ppm", co2);
//! # Ok(())
//! # }
//! ```

#![no_std]

#[macro_use]
extern crate lazy_static;

use crate::command::Command;
use crate::frame::Frame;
use crate::nb_comm::{NbFuture, WriteAll, WriteAndReadResponse};
use core::convert::TryInto;
use core::fmt::{self, Display};
use embedded_hal::serial::{Read, Write};

pub mod command;
pub mod frame;
mod nb_comm;

lazy_static! {
    static ref READ_CO2: Frame = Command::ReadCo2.into();
}

/// Driver for the MH-Z19C sensor.
pub struct MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    state: MhZ19CState<'a, U, E>,
    uart: Option<U>,
}

enum MhZ19CState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Idle,
    ReadCo2(WriteAndReadResponse<U, E, &'a [u8], [u8; 9]>),
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
            ReadCo2(future) => future.into_return_value().0,
            SetSelfCalibrate(future) => future.into_return_value(),
        }
    }

    /// Reads and returns the CO₂ concentration in parts-per-million (ppm).
    pub fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        loop {
            if let MhZ19CState::Idle = &mut self.state {
                let uart = self.uart.take().unwrap();
                self.state = MhZ19CState::ReadCo2(WriteAndReadResponse::new(
                    uart,
                    &*READ_CO2.as_ref(),
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

    /// Activates or deactivates the sensor's self-calibration mode.
    ///
    /// See the sensor's data sheet for more information on self-calibration
    /// and hand-operated mode.
    pub fn set_self_calibrate(&mut self, enabled: bool) -> nb::Result<(), Error<E>> {
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

    fn poll(&mut self) -> nb::Result<(), Error<E>> {
        use MhZ19CState::*;
        match &mut self.state {
            Idle => Ok(()),
            ReadCo2(future) => future.poll(),
            SetSelfCalibrate(future) => future.poll(),
        }
        .map_err(|err| err.map(Error::UartError))
    }

    fn recover_uart(&mut self, state: MhZ19CState<U, E>) {
        use MhZ19CState::*;
        match state {
            Idle => (),
            ReadCo2(future) => self.uart = Some(future.into_return_value().0),
            SetSelfCalibrate(future) => self.uart = Some(future.into_return_value()),
        }
    }

    fn unpack_return_frame(command: Command, frame: &Frame) -> Result<&[u8], Error<E>> {
        frame.validate().map_err(Error::FrameError)?;
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

#[derive(Debug, PartialEq, Eq)]
pub enum Error<T> {
    /// The frame of a command or return value was invalid.
    FrameError(frame::Error),
    /// The received data is not response.
    NotAResponse,
    /// Received a response for a different op code than expected.
    OpCodeMismatch { expected: u8, got: u8 },
    /// Communication error caused by the UART/serial interface.
    UartError(T),
}

impl<T: Display> Display for Error<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FrameError(err) => write!(f, "Frame error: {}", err),
            Self::NotAResponse => write!(f, "Expected response, but got command."),
            Self::OpCodeMismatch { expected, got } => write!(
                f,
                "Expected response for op code 0x{:x}, but got op code 0x{:x}.",
                expected, got
            ),
            Self::UartError(err) => write!(f, "UART communication error: {}", err),
        }
    }
}

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
        create_serial_mock_returning, READ_CO2_RESPONSE, SELF_CALIBRATE_ON_COMMAND,
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
            Err(Error::FrameError(frame::Error::InvalidStartByte(0x00)))
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
            Err(Error::FrameError(frame::Error::InvalidChecksum))
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
}
