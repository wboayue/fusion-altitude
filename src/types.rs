/// Tuning parameters for [`crate::AltitudeEstimator`].
///
/// The three gains define a 3rd-order observer with characteristic
/// polynomial `s³ + position_gain·s² + velocity_gain·s + bias_gain`.
/// A clean cascaded design places a damped position/velocity pair at
/// bandwidth `ω` and a slow real bias pole at `ω_b ≪ ω`:
///
/// ```text
/// position_gain = 2ζω + ω_b      (1/s)
/// velocity_gain = ω² + 2ζω·ω_b   (1/s²)
/// bias_gain     = ω² · ω_b       (1/s³)
/// ```
///
/// Routh-Hurwitz stability requires `position_gain · velocity_gain > bias_gain`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct AltitudeSettings {
    /// Feedback gain (1/s) from baro residual into the altitude state.
    /// Larger → tighter tracking of the barometer, more noise visible.
    pub position_gain: f32,

    /// Feedback gain (1/s²) from baro residual into the velocity state.
    /// Damps integrated-acceleration drift in the velocity estimate.
    pub velocity_gain: f32,

    /// Feedback gain (1/s³) from baro residual into the accel-bias state.
    /// Bias-loop bandwidth `ω_b ≈ (bias_gain / velocity_gain)`. Set to
    /// `0.0` to disable bias estimation (recovers the original 2-state
    /// filter).
    pub bias_gain: f32,
}

impl Default for AltitudeSettings {
    /// Tuned for a VTOL multirotor: `ω = 1.5 rad/s`, `ζ = 0.7`,
    /// `ω_b = 0.3 rad/s` (position/velocity loop ~1 s, bias loop ~10 s
    /// to converge). Steady-state altitude error from a constant Z accel
    /// bias is **zero** with these gains — the bias is estimated as a
    /// state and subtracted from the measured acceleration.
    fn default() -> Self {
        Self {
            position_gain: 2.40,
            velocity_gain: 2.88,
            bias_gain: 0.675,
        }
    }
}
