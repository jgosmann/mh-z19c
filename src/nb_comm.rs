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
/// To retrieve the result use [`into_return_value`]. If called before
/// completing the future this can be used to retrieve owned values when
/// intending to cancel the future.
///
/// * `R`: return value type, wrap anything that is not available before the
///   completion of the future into an [`core::option::Option`].
/// * `E`: error type
pub trait NbFuture<R, E> {
    /// Poll the future and advance execution if possible.
    fn poll(&mut self) -> nb::Result<(), E>;
    /// Turn the future into the owned variable captured by the future and other
    /// return values.
    fn into_return_value(self) -> R;
}

/// Write all bytes within a buffer.
///
/// * `'a`: buffer lifetime
/// * `W`: Concrete [`embedded_hal::serial::Write`] type with error type `E`
/// * `B`: Type of buffer
pub struct WriteAll<W, E, B>
where
    W: Write<u8, Error = E>,
    B: AsRef<[u8]>,
{
    uart: W,
    buf: B,
    bytes_written: usize,
}

impl<W, E, B> WriteAll<W, E, B>
where
    W: Write<u8, Error = E>,
    B: AsRef<[u8]>,
{
    /// Create future to write all bytes in `buf` to `uart`.
    ///
    /// The ownership of the `uart` will be returned from [`Self::poll`] on
    /// completion.
    pub fn new(uart: W, buf: B) -> Self {
        Self {
            uart,
            buf,
            bytes_written: 0,
        }
    }
}

impl<W, E, B> NbFuture<W, E> for WriteAll<W, E, B>
where
    W: Write<u8, Error = E>,
    B: AsRef<[u8]>,
{
    fn poll(&mut self) -> nb::Result<(), E> {
        loop {
            let c = self.buf.as_ref()[self.bytes_written];
            match self.uart.write(c) {
                Ok(()) => {
                    self.bytes_written += 1;
                    if self.bytes_written >= self.buf.as_ref().len() {
                        return Ok(());
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }

    fn into_return_value(self) -> W {
        self.uart
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
    uart: R,
    buf: B,
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
            uart,
            buf,
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
    fn poll(&mut self) -> nb::Result<(), E> {
        loop {
            match self.uart.read() {
                Ok(c) => {
                    self.buf.as_mut()[self.bytes_read] = c;
                    self.bytes_read += 1;
                    if self.bytes_read >= self.read_len {
                        return Ok(());
                    }
                }
                Err(err) => return Err(err),
            }
        }
    }

    fn into_return_value(self) -> (R, B) {
        (self.uart, self.buf)
    }
}

/// Write all bytes within a buffer and read a fixed length response afterwards.
///
/// * `'a`: buffer lifetime
/// * `U`: Concrete [`embedded_hal::serial::Read`] and
///   [`embedded_hal::serial::Write`] type with error type `E`
/// * `BWrite`: Type of buffer to write
/// * `BRead`: Type of buffer to read into
pub struct WriteAndReadResponse<U, E, BWrite, BRead>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    BWrite: AsRef<[u8]>,
    BRead: AsMut<[u8]>,
{
    state: Option<WriteAndReadResponseState<U, E, BWrite, BRead>>,
}

enum WriteAndReadResponseState<U, E, BWrite, BRead>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    BWrite: AsRef<[u8]>,
    BRead: AsMut<[u8]>,
{
    Write {
        future: WriteAll<U, E, BWrite>,
        read_buf: BRead,
        response_len: usize,
    },
    Flush {
        uart: U,
        read_buf: BRead,
        response_len: usize,
    },
    Read {
        future: ReadMultiple<U, E, BRead>,
    },
    Completed {
        uart: U,
        read_buf: BRead,
    },
}

impl<U, E, BWrite, BRead> WriteAndReadResponseState<U, E, BWrite, BRead>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    BWrite: AsRef<[u8]>,
    BRead: AsMut<[u8]>,
{
    fn advance(self) -> Result<Self, (Self, nb::Error<E>)> {
        use WriteAndReadResponseState::*;
        match self {
            Write {
                mut future,
                read_buf,
                response_len,
            } => match future.poll() {
                Ok(()) => Ok(Flush {
                    uart: future.into_return_value(),
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
                Ok(()) => {
                    let (uart, read_buf) = future.into_return_value();
                    Ok(Completed { uart, read_buf })
                }
                Err(err) => Err((Read { future }, err)),
            },
            other => Ok(other),
        }
    }

    fn cancel(self) -> (U, BRead) {
        use WriteAndReadResponseState::*;
        match self {
            Write {
                future, read_buf, ..
            } => (future.into_return_value(), read_buf),
            Flush { uart, read_buf, .. } => (uart, read_buf),
            Read { future } => future.into_return_value(),
            Completed { uart, read_buf } => (uart, read_buf),
        }
    }
}

impl<'a, U, E, BWrite, BRead> WriteAndReadResponse<U, E, BWrite, BRead>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    BWrite: AsRef<[u8]>,
    BRead: AsMut<[u8]>,
{
    /// Create future to write `buf` bytes to `uart` and read `read_len` bytes
    /// afterwards.
    ///
    /// The ownership of the `uart` will be returned together with the bytes
    /// read from [`Self::poll`] on completion.
    pub fn new(uart: U, write_buf: BWrite, read_buf: BRead, response_len: usize) -> Self {
        Self {
            state: Some(WriteAndReadResponseState::Write {
                future: WriteAll::new(uart, write_buf),
                read_buf,
                response_len,
            }),
        }
    }
}

impl<'a, U, E, BWrite, BRead> NbFuture<(U, BRead), E> for WriteAndReadResponse<U, E, BWrite, BRead>
where
    U: Read<u8, Error = E> + Write<u8, Error = E>,
    BWrite: AsRef<[u8]>,
    BRead: AsMut<[u8]>,
{
    fn poll(&mut self) -> nb::Result<(), E> {
        loop {
            let (result, new_state) = match self.state.take().unwrap().advance() {
                Ok(new_state) => (Ok(()), new_state),
                Err((new_state, err)) => (Err(err), new_state),
            };

            self.state = Some(new_state);

            if let Some(WriteAndReadResponseState::Completed { .. }) = self.state {
                return Ok(());
            }

            if let Err(err) = result {
                return Err(err);
            }
        }
    }

    fn into_return_value(self) -> (U, BRead) {
        self.state.unwrap().cancel()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::serial_mock::SerialMock;
    use nb::block;
    use std::string::String;

    #[test]
    fn test_write_all() -> Result<(), String> {
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
        block!(future.poll())?;
        let write_mock = future.into_return_value();
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
    fn test_read_multiple() -> Result<(), String> {
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
        block!(future.poll())?;
        let (_, buf) = future.into_return_value();
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
    fn test_write_and_read_response() -> Result<(), String> {
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
        block!(future.poll())?;
        let (serial_mock, read_buf) = future.into_return_value();
        assert_eq!(serial_mock.write_buf, write_buf);
        assert_eq!(read_buf, ['o' as u8, 'u' as u8, 't' as u8]);
        assert_eq!(serial_mock.flushed_up_to, 2);
        Ok(())
    }

    #[test]
    fn test_write_and_read_response_error_propagation() {
        let serial_mock =
            SerialMock::new(vec![], vec![Err(nb::Error::Other("expected error".into()))]);
        let write_buf = ['i' as u8, 'n' as u8];
        let read_buf = [0u8; 3];

        let mut future = WriteAndReadResponse::new(serial_mock, &write_buf, read_buf, 3);
        assert_eq!(
            block!(future.poll()).unwrap_err(),
            String::from("expected error")
        );
    }
}
