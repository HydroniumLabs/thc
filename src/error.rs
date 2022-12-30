use std::{error::Error, fmt};

/// Errors related to the decoding process.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum DecodingError {
    /// Invalid header.
    InvalidHeader(&'static str),
    /// Missing tag bit at the given position.
    MissingTag(usize),
    /// Invalid cell index.
    InvalidCellIndex {
        /// Error message.
        reason: &'static str,
        /// Underlying validation error, if any.
        source: Option<h3o::error::InvalidCellIndex>,
    },
    /// Not enough data to decompress.
    NotEnoughData,
}

impl DecodingError {
    pub(crate) const fn bad_header(reason: &'static str) -> Self {
        Self::InvalidHeader(reason)
    }

    pub(crate) const fn missing_tag(position: usize) -> Self {
        Self::MissingTag(position)
    }

    pub(crate) const fn bad_index(
        reason: &'static str,
        source: Option<h3o::error::InvalidCellIndex>,
    ) -> Self {
        Self::InvalidCellIndex { reason, source }
    }

    pub(crate) const fn not_enough_data() -> Self {
        Self::NotEnoughData
    }
}

impl fmt::Display for DecodingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::InvalidHeader(reason) => {
                write!(f, "header error: {reason}")
            }
            Self::MissingTag(position) => {
                write!(f, "missing tag bit at {position}")
            }
            Self::InvalidCellIndex { reason, .. } => {
                write!(f, "invalid cell index: {reason}")
            }
            Self::NotEnoughData => write!(f, "truncated input"),
        }
    }
}

impl Error for DecodingError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        if let Self::InvalidCellIndex {
            source: Some(ref source),
            ..
        } = *self
        {
            return Some(source);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use h3o::CellIndex;

    // All error must have a non-empty display.
    #[test]
    fn display() {
        assert!(!DecodingError::bad_header("invalid header")
            .to_string()
            .is_empty());

        assert!(!DecodingError::missing_tag(42).to_string().is_empty());

        assert!(!DecodingError::bad_index("invalid cell index", None)
            .to_string()
            .is_empty());

        assert!(!DecodingError::not_enough_data().to_string().is_empty());
    }

    // Check that source if forwarded when relevant.
    #[test]
    fn source() {
        assert!(DecodingError::bad_header("invalid header")
            .source()
            .is_none());

        assert!(DecodingError::missing_tag(42).source().is_none());

        assert!(DecodingError::bad_index("invalid cell index", None)
            .source()
            .is_none());
        assert!(DecodingError::bad_index(
            "not a cell index",
            CellIndex::try_from(0).err()
        )
        .source()
        .is_some());

        assert!(DecodingError::not_enough_data().source().is_none());
    }
}
