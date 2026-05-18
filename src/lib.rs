#![no_std]
#![doc = include_str!("../README.md")]

mod altitude;
mod types;

pub use altitude::AltitudeEstimator;
pub use types::AltitudeSettings;

/// Standard gravity in m/s². Convenience constant for converting
/// `fusion-ahrs`-style accelerations (units of g) to the m/s² expected by
/// [`AltitudeEstimator::update`].
pub const GRAVITY: f32 = 9.806_65;
