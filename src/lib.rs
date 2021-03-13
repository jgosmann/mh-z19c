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

#[cfg(test)]
mod serial_mock;

enum MhZ19CState<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    Idle,
    ReadCo2(WriteAndReadResponse<U, E, &'a [u8], [u8; 9]>),
    SetSelfCalibrate(WriteAll<U, E, Frame>),
}

lazy_static! {
    static ref READ_CO2: Frame = Command::ReadCo2.into();
}

pub struct MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    state: MhZ19CState<'a, U, E>,
    uart: Option<U>,
}

impl<'a, U, E> MhZ19C<'a, U, E>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
{
    pub fn new(uart: U) -> Self {
        Self {
            state: MhZ19CState::Idle,
            uart: Some(uart),
        }
    }

    pub fn into_inner(mut self) -> U {
        use MhZ19CState::*;
        match self.state {
            Idle => self.uart.take().unwrap(),
            ReadCo2(future) => future.cancel().unwrap().0,
            SetSelfCalibrate(future) => future.cancel().unwrap(),
        }
    }

    pub fn read_co2_ppm(&mut self) -> nb::Result<u16, Error<E>> {
        if let MhZ19CState::Idle = &mut self.state {
            let uart = self.uart.take().unwrap();
            self.state = MhZ19CState::ReadCo2(WriteAndReadResponse::new(
                uart,
                &*READ_CO2.as_ref(),
                [0u8; 9],
                9,
            ));
        }
        if let MhZ19CState::ReadCo2(future) = &mut self.state {
            match future.poll() {
                Ok((uart, buf)) => {
                    let frame = Frame::new(buf);
                    let data = Self::unpack_return_frame(Command::ReadCo2, &frame)
                        .map_err(nb::Error::Other)?;
                    self.uart = Some(uart);
                    self.state = MhZ19CState::Idle;
                    Ok(u16::from_be_bytes(data[..2].try_into().unwrap()))
                }
                Err(err) => Err(err.map(|inner_err| match inner_err {
                    nb_comm::Error::Stopped => Error::InternalError,
                    nb_comm::Error::Execution(uart_err) => Error::UartError(uart_err),
                })),
            }
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn set_self_calibrate(&mut self, enabled: bool) -> nb::Result<(), Error<E>> {
        if let MhZ19CState::Idle = &mut self.state {
            let uart = self.uart.take().unwrap();
            let frame: Frame = Command::SetSelfCalibrate(enabled).into();
            self.state = MhZ19CState::SetSelfCalibrate(WriteAll::new(uart, frame));
        }
        if let MhZ19CState::SetSelfCalibrate(future) = &mut self.state {
            match future.poll() {
                Ok(uart) => {
                    self.uart = Some(uart);
                    self.state = MhZ19CState::Idle;
                    Ok(())
                }
                Err(err) => Err(err.map(|inner_err| match inner_err {
                    nb_comm::Error::Stopped => Error::InternalError,
                    nb_comm::Error::Execution(uart_err) => Error::UartError(uart_err),
                })),
            }
        } else {
            Err(nb::Error::WouldBlock)
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
    FrameError(frame::Error),
    NotAResponse,
    OpCodeMismatch { expected: u8, got: u8 },
    InternalError,
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
            Self::InternalError => {
                write!(f, "An internal error occured, please report this as a bug.")
            }
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

    use crate::serial_mock::SerialMock;
    use nb::block;
    use std::string::String;

    static READ_CO2_RESPONSE: [u8; 9] = [0xff, 0x86, 0x03, 0x20, 0x12, 0x34, 0x56, 0x78, 0x43];
    static SELF_CALIBRATE_ON_COMMAND: [u8; 9] =
        [0xff, 0x01, 0x79, 0xa0, 0x00, 0x00, 0x00, 0x00, 0xe6];

    fn create_serial_mock_returning(read_data: &[u8]) -> SerialMock {
        SerialMock::new(read_data.iter().copied().map(Ok).collect(), vec![Ok(()); 9])
    }

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
}
