//! MH-Z19C command definitions.

/// Commands understood by the MH-Z19C sensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// Read CO2 concentration from sensor.
    ReadCo2,
    GetFirmwareVersion,
    /// Set self calibration enabled status.
    SetSelfCalibrate(bool),
}

impl Command {
    /// Op code used for the command in communication with the sensor.
    pub fn op_code(&self) -> u8 {
        match self {
            Self::ReadCo2 => 0x86,
            Self::GetFirmwareVersion => 0xA0,
            Self::SetSelfCalibrate(_) => 0x79,
        }
    }

    /// Serialize the command op code together with its arguments.
    pub fn serialize(&self) -> [u8; 6] {
        match self {
            Self::ReadCo2 => [self.op_code(), 0, 0, 0, 0, 0],
            Self::GetFirmwareVersion => [self.op_code(), 0, 0, 0, 0, 0],
            Self::SetSelfCalibrate(true) => [self.op_code(), 0xa0, 0, 0, 0, 0],
            Self::SetSelfCalibrate(false) => [self.op_code(), 0, 0, 0, 0, 0],
        }
    }
}
