//! Non-blocking communication state machines for the MH-Z19C UART interface.

use embedded_hal::serial::{Read, Write};

/// Trait for a future or state machine used for non-blocking communication.
///
/// These state machines depend on being actively polled (similar to Rust
/// futures). To cancel progress, just stop polling. However, this might leave
/// you within an undefined state of the UART communication protocol.
///
/// The future must not be polled after it has returned an
/// [`core::result::Result::Ok`] result.
///
/// * `R`: OK result type
/// * `E`: error type
pub trait NbFuture<R, E> {
    fn poll(&mut self) -> nb::Result<R, E>;
}

/// Write all bytes within a buffer.
///
/// * `'a`: buffer lifetime
/// * `W`: Concrete [`embedded_hal::serial::Write`] type with error type `E`
pub struct WriteAll<'a, W, E>
where
    W: Write<u8, Error = E>,
{
    uart: Option<W>,
    buf: &'a [u8],
    bytes_written: usize,
}

impl<'a, W, E> WriteAll<'a, W, E>
where
    W: Write<u8, Error = E>,
{
    /// Create future to write all bytes in `buf` to `uart`.
    ///
    /// The ownership of the `uart` will be returned from [`Self::poll`] on
    /// completion.
    pub fn new(uart: W, buf: &'a [u8]) -> Self {
        Self {
            uart: Some(uart),
            buf,
            bytes_written: 0,
        }
    }
}

impl<'a, W, E> NbFuture<W, E> for WriteAll<'a, W, E>
where
    W: Write<u8, Error = E>,
{
    fn poll(&mut self) -> nb::Result<W, E> {
        let uart = self.uart.as_mut().unwrap();
        loop {
            let c = self.buf[self.bytes_written];
            match uart.write(c) {
                Ok(()) => {
                    self.bytes_written += 1;
                    if self.bytes_written >= self.buf.len() {
                        return Ok(self.uart.take().unwrap());
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }
}

/// Read multiple bytes.
///
/// * `R`: Concrete [`embedded_hal::serial::Read`] type with error type `E`
/// * `B`: Type of buffer to write to
pub struct ReadMultiple<R, E, B>
where
    R: Read<u8, Error = E>,
    B: AsMut<[u8]>,
{
    uart: Option<R>,
    buf: Option<B>,
    bytes_read: usize,
    read_len: usize,
}

impl<R, E, B> ReadMultiple<R, E, B>
where
    R: Read<u8, Error = E>,
    B: AsMut<[u8]>,
{
    /// Create future to read `read_len` bytes from `uart`.
    ///
    /// The ownership of the `uart` will be returned together with the bytes
    /// read from [`Self::poll`] on completion.
    pub fn new(uart: R, buf: B, read_len: usize) -> Self {
        Self {
            uart: Some(uart),
            buf: Some(buf),
            bytes_read: 0,
            read_len,
        }
    }
}

impl<'a, R, E, B> NbFuture<(R, B), E> for ReadMultiple<R, E, B>
where
    R: Read<u8, Error = E>,
    B: AsMut<[u8]>,
{
    fn poll(&mut self) -> nb::Result<(R, B), E> {
        loop {
            let uart = self.uart.as_mut().unwrap();
            let buf = self.buf.as_mut().unwrap().as_mut();
            match uart.read() {
                Ok(c) => {
                    buf[self.bytes_read] = c;
                    self.bytes_read += 1;
                    if self.bytes_read >= self.read_len {
                        return Ok((self.uart.take().unwrap(), self.buf.take().unwrap()));
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }
}

/// Write all bytes within a buffer and read a fixed length response afterwards.
///
/// * `'a`: buffer lifetime
/// * `U`: Concrete [`embedded_hal::serial::Read`] and
///   [`embedded_hal::serial::Write`] type with error type `E`
/// * `B`: Type of buffer to write to
pub struct WriteAndReadResponse<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    state: Option<WriteAndReadResponseState<'a, U, E, B>>,
}

enum WriteAndReadResponseState<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    Write {
        future: WriteAll<'a, U, E>,
        read_buf: B,
        response_len: usize,
    },
    Flush {
        uart: U,
        read_buf: B,
        response_len: usize,
    },
    Read {
        future: ReadMultiple<U, E, B>,
    },
    Completed {
        uart: U,
        read_buf: B,
    },
}

impl<'a, U, E, B> WriteAndReadResponseState<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    fn advance(self) -> Result<Self, (Self, nb::Error<E>)> {
        use WriteAndReadResponseState::*;
        match self {
            Write {
                mut future,
                read_buf,
                response_len,
            } => match future.poll() {
                Ok(uart) => Ok(Flush {
                    uart,
                    read_buf,
                    response_len,
                }),
                Err(err) => Err((
                    Write {
                        future,
                        read_buf,
                        response_len,
                    },
                    err,
                )),
            },
            Flush {
                mut uart,
                read_buf,
                response_len,
            } => match uart.flush() {
                Ok(()) => Ok(Read {
                    future: ReadMultiple::new(uart, read_buf, response_len),
                }),
                Err(err) => Err((
                    Flush {
                        uart,
                        read_buf,
                        response_len,
                    },
                    err,
                )),
            },
            Read { mut future } => match future.poll() {
                Ok((uart, read_buf)) => Ok(Completed { uart, read_buf }),
                Err(err) => Err((Read { future }, err)),
            },
            other => Ok(other),
        }
    }
}

impl<'a, U, E, B> WriteAndReadResponse<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    /// Create future to write `buf` bytes to `uart` and read `read_len` bytes
    /// afterwards.
    ///
    /// The ownership of the `uart` will be returned together with the bytes
    /// read from [`Self::poll`] on completion.
    pub fn new(uart: U, write_buf: &'a [u8], read_buf: B, response_len: usize) -> Self {
        Self {
            state: Some(WriteAndReadResponseState::Write {
                future: WriteAll::new(uart, write_buf),
                read_buf,
                response_len,
            }),
        }
    }
}

impl<'a, U, E, B> NbFuture<(U, B), E> for WriteAndReadResponse<'a, U, E, B>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    B: AsMut<[u8]>,
{
    fn poll(&mut self) -> nb::Result<(U, B), E> {
        loop {
            let (result, new_state) = match self.state.take().unwrap().advance() {
                Ok(new_state) => (Ok(()), new_state),
                Err((new_state, err)) => (Err(err), new_state),
            };

            if let WriteAndReadResponseState::Completed { uart, read_buf } = new_state {
                return Ok((uart, read_buf));
            }

            self.state = Some(new_state);
            if let Err(err) = result {
                return Err(err);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::serial_mock::SerialMock;
    use nb::block;
    use std::boxed::Box;
    use std::string::String;

    #[test]
    fn test_write_all() -> Result<(), Box<dyn std::error::Error>> {
        let write_mock = SerialMock::new(
            vec![],
            vec![
                Ok(()),
                Ok(()),
                Err(nb::Error::WouldBlock),
                Err(nb::Error::WouldBlock),
                Ok(()),
            ],
        );
        let buf = ['f' as u8, 'o' as u8, 'o' as u8];

        let mut future = WriteAll::new(write_mock, &buf);
        let write_mock = block!(future.poll())?;
        assert_eq!(write_mock.write_buf, buf);
        Ok(())
    }

    #[test]
    fn test_write_all_error_propagation() {
        let write_mock =
            SerialMock::new(vec![], vec![Err(nb::Error::Other("expected error".into()))]);
        let buf = [0u8; 1];

        let mut future = WriteAll::new(write_mock, &buf);
        assert_eq!(
            block!(future.poll()).unwrap_err(),
            String::from("expected error")
        );
    }

    #[test]
    fn test_read_multiple() -> Result<(), Box<dyn std::error::Error>> {
        let read_mock = SerialMock::new(
            vec![
                Ok('f' as u8),
                Ok('o' as u8),
                Err(nb::Error::WouldBlock),
                Err(nb::Error::WouldBlock),
                Ok('o' as u8),
            ],
            vec![],
        );
        let buf = [0u8; 3];

        let mut future = ReadMultiple::new(read_mock, buf, 3);
        let (_, buf) = block!(future.poll())?;
        assert_eq!(buf, ['f' as u8, 'o' as u8, 'o' as u8]);
        Ok(())
    }

    #[test]
    fn test_read_multiple_error_propagation() {
        let read_mock =
            SerialMock::new(vec![Err(nb::Error::Other("expected error".into()))], vec![]);
        let buf = [0u8; 1];

        let mut future = ReadMultiple::new(read_mock, buf, 1);
        assert_eq!(
            block!(future.poll()).unwrap_err(),
            String::from("expected error")
        );
    }

    #[test]
    fn test_write_and_read_response() -> Result<(), Box<dyn std::error::Error>> {
        let serial_mock = SerialMock::new(
            vec![
                Ok('o' as u8),
                Err(nb::Error::WouldBlock),
                Ok('u' as u8),
                Ok('t' as u8),
            ],
            vec![Ok(()), Err(nb::Error::WouldBlock), Ok(())],
        );
        let write_buf = ['i' as u8, 'n' as u8];
        let read_buf = [0u8; 3];

        let mut future = WriteAndReadResponse::new(serial_mock, &write_buf, read_buf, 3);
        let (serial_mock, read_buf) = block!(future.poll())?;
        assert_eq!(serial_mock.write_buf, write_buf);
        assert_eq!(read_buf, ['o' as u8, 'u' as u8, 't' as u8]);
        assert_eq!(serial_mock.flushed_up_to, 2);
        Ok(())
    }
}
