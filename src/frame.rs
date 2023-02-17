//! Serial communication frame handling for the MH-Z19C sensor.

use crate::command::Command;
use core::convert::From;
use core::fmt::{self, Display, Formatter};

/// Calculates the checksum of `buf`.
pub fn checksum(buf: &[u8]) -> u8 {
    buf.iter()
        .fold(0x00, |acc: u8, &x: &u8| acc.overflowing_sub(x).0)
}

const START_BYTE: u8 = 0xff;
const COMMAND_MAGIC_BYTE: u8 = 0x01;

/// Represents a frame for the serial communication.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct Frame([u8; 9]);

impl From<Command> for Frame {
    fn from(command: Command) -> Self {
        let mut buf = [START_BYTE, COMMAND_MAGIC_BYTE, 0, 0, 0, 0, 0, 0, 0];
        buf[2..8].copy_from_slice(&command.serialize());
        buf[8] = checksum(&buf[1..8]);
        Self(buf)
    }
}

impl AsRef<[u8]> for Frame {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl Frame {
    /// Return a new initialized [`Frame`] struct.
    pub fn new(data: [u8; 9]) -> Self {
        Self(data)
    }

    /// Unwrap the frame data.
    pub fn into_inner(self) -> [u8; 9] {
        self.0
    }

    fn start_byte(&self) -> u8 {
        self.0[0]
    }

    /// Returns `true` if the frame has a valid start byte.
    pub fn has_valid_start_byte(&self) -> bool {
        self.start_byte() == START_BYTE
    }

    /// Returns `true` if the frame represents the response to a command.
    pub fn is_response(&self) -> bool {
        self.0[1] != COMMAND_MAGIC_BYTE
    }

    /// Returns the op code byte of the command of a request or response.
    pub fn op_code(&self) -> u8 {
        if self.is_response() {
            self.0[1]
        } else {
            self.0[2]
        }
    }

    fn checksum(&self) -> u8 {
        self.0[8]
    }

    /// Returns the command arguments or response data (without op code).
    pub fn data(&self) -> &[u8] {
        if self.is_response() {
            &self.0[2..8]
        } else {
            &self.0[3..8]
        }
    }

    /// Returns `true` if the frame's checksum is valid.
    pub fn has_valid_checksum(&self) -> bool {
        checksum(&self.0[1..8]) == self.checksum()
    }

    /// Validates the correctness of the frame.
    ///
    /// * Checks that the start byte is valid.
    /// * Checks that the checksum is valid.
    pub fn validate(&self) -> Result<(), ValidateFrameError> {
        if !self.has_valid_start_byte() {
            Err(ValidateFrameError::InvalidStartByte(self.start_byte()))
        } else if !self.has_valid_checksum() {
            Err(ValidateFrameError::InvalidChecksum {
                expected: checksum(&self.0[1..8]),
                actual: self.checksum(),
            })
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidateFrameError {
    /// Indicates that the start byte is invalid and provides that invalid byte.
    InvalidStartByte(u8),
    /// Indicates that the checksum is invalid.
    InvalidChecksum { expected: u8, actual: u8 },
}

impl Display for ValidateFrameError {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), fmt::Error> {
        use ValidateFrameError::*;
        match self {
            InvalidStartByte(got) => {
                write!(f, "expected start byte 0xff, but got 0x{:x}", got)
            }
            InvalidChecksum { expected, actual } => write!(
                f,
                "invalid checksum (got {} instead of {})",
                expected, actual
            ),
        }
    }
}

#[cfg(std)]
impl std::error::Error for Error {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::Command;

    #[test]
    fn test_frame_read_co2_command() {
        let frame = Frame::new([0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]);
        assert!(frame.has_valid_start_byte());
        assert!(!frame.is_response());
        assert_eq!(frame.op_code(), 0x86);
        assert_eq!(frame.data(), [0x00; 5]);
        assert!(frame.has_valid_checksum());
        assert!(frame.validate().is_ok());
    }

    #[test]
    fn test_frame_read_co2_response() {
        let frame = Frame::new([0xff, 0x86, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x77]);
        assert!(frame.has_valid_start_byte());
        assert!(frame.is_response());
        assert_eq!(frame.op_code(), 0x86);
        assert_eq!(frame.data(), [0x01, 0x02, 0x00, 0x00, 0x00, 0x00]);
        assert!(frame.has_valid_checksum());
        assert!(frame.validate().is_ok());
    }

    #[test]
    fn test_frame_invalid_start_byte() {
        let frame = Frame::new([0x00; 9]);
        assert!(!frame.has_valid_start_byte());
        assert_eq!(
            frame.validate(),
            Err(ValidateFrameError::InvalidStartByte(0x00))
        );
    }

    #[test]
    fn test_frame_invalid_checksum() {
        let frame = Frame::new([0xff; 9]);
        assert!(!frame.has_valid_checksum());
        assert_eq!(
            frame.validate(),
            Err(ValidateFrameError::InvalidChecksum {
                expected: 0x07,
                actual: 0xff,
            })
        );
    }

    #[test]
    fn test_frame_from_command() {
        let frame: Frame = Command::SetSelfCalibrate(true).into();
        assert!(frame.has_valid_start_byte());
        assert!(!frame.is_response());
        assert_eq!(frame.op_code(), Command::SetSelfCalibrate(true).op_code());
        assert_eq!(
            frame.data(),
            &Command::SetSelfCalibrate(true).serialize()[1..]
        );
        assert!(frame.has_valid_checksum());
        assert!(frame.validate().is_ok());
    }

    #[test]
    fn test_checksum() {
        assert_eq!(checksum(&[0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00]), 0x79);
    }
}
