//! Async embedded driver for the INA3221 current and power monitor
//!
//! Provides a platform agnostic driver for the [INA3221] triple-channel current and power monitor
//! that can be used with any [embedded_hal_async] v1.0.0 blocking I2C implementation.
//!
//! [INA3221]: https://www.ti.com/lit/ds/symlink/ina3221.pdf
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/1.0.0/embedded_hal_async/
#![no_std]

#[cfg(feature = "sync")]
extern crate embedded_hal as hal;
#[cfg(feature = "async")]
extern crate embedded_hal_async as hal;

mod driver;
mod flags;
mod helpers;
mod mode;
pub mod prelude;
mod registers;

#[cfg(feature = "sync")]
pub use driver::INA3221;
#[cfg(feature = "async")]
pub use driver::INA3221Async;

pub use flags::MaskEnableFlags;
pub use mode::OperatingMode;
pub use ohms::*;
