use crate::command::Command;
use core::convert::From;
use core::fmt::{self, Display, Formatter};

fn checksum(buf: &[u8]) -> u8 {
    buf.iter()
        .fold(0x00, |acc: u8, &x: &u8| acc.overflowing_sub(x).0)
}

const START_BYTE: u8 = 0xff;
const COMMAND_MAGIC_BYTE: u8 = 0x01;

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
    pub fn new(data: [u8; 9]) -> Self {
        Self(data)
    }

    pub fn into_inner(self) -> [u8; 9] {
        self.0
    }

    fn start_byte(&self) -> u8 {
        self.0[0]
    }

    pub fn has_valid_start_byte(&self) -> bool {
        self.start_byte() == START_BYTE
    }

    pub fn is_response(&self) -> bool {
        self.0[1] != COMMAND_MAGIC_BYTE
    }

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
    pub fn data(&self) -> &[u8] {
        if self.is_response() {
            &self.0[2..8]
        } else {
            &self.0[3..8]
        }
    }

    pub fn has_valid_checksum(&self) -> bool {
        checksum(&self.0[1..8]) == self.checksum()
    }

    pub fn validate(&self) -> Result<(), Error> {
        if !self.has_valid_start_byte() {
            Err(Error::InvalidStartByte(self.start_byte()))
        } else if !self.has_valid_checksum() {
            Err(Error::InvalidChecksum)
        } else {
            Ok(())
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    InvalidStartByte(u8),
    InvalidChecksum,
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), fmt::Error> {
        use Error::*;
        match self {
            InvalidStartByte(got) => write!(f, "Expected start byte 0xff, but got 0x{:x}.", got),
            InvalidChecksum => write!(f, "Invalid checksum."),
        }
    }
}
