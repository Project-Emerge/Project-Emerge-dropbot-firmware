#[derive(Debug)]
pub enum DisplayError<E: core::fmt::Debug> {
    Bus(E),
    NotInitialized,
    CoordinatesOutOfRange,
}
