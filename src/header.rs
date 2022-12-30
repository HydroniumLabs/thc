use crate::DecodingError;
use std::io::{self, Write};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Header {
    /// Compressed H3 Tree.
    ChtV1,
}

impl Header {
    /// Returns the header for a Compressed H3 Tree payload, version 1.
    pub const fn cht_v1() -> Self {
        Self::ChtV1
    }

    /// Writes the header into the specified writer.
    pub fn write<W: Write>(self, writer: &mut W) -> Result<(), io::Error> {
        match self {
            Self::ChtV1 => writer.write_all(&[1]),
        }
    }

    /// Reads a header from the given bytes.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, DecodingError> {
        if bytes.first() == Some(&1) {
            return Ok(Self::cht_v1());
        }

        Err(DecodingError::bad_header("cannot decode"))
    }

    /// Returns the header size, in bytes.
    pub const fn len(self) -> usize {
        match self {
            Self::ChtV1 => 1,
        }
    }
}
