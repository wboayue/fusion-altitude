/// Tuning parameters for [`crate::AltitudeEstimator`].
///
/// The two gains define the 2nd-order complementary filter's
/// characteristic polynomial `s² + position_gain·s + velocity_gain`. A
/// well-behaved response is obtained by choosing
/// `position_gain = 2ζω` and `velocity_gain = ω²`, with damping `ζ ≈ 0.7`
/// and bandwidth `ω` (rad/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltitudeSettings {
    /// Feedback gain (1/s) from baro residual into the altitude state.
    /// Larger → tighter tracking of the barometer, more noise visible.
    pub position_gain: f32,

    /// Feedback gain (1/s²) from baro residual into the velocity state.
    /// Damps integrated-acceleration drift in the velocity estimate.
    pub velocity_gain: f32,
}

impl Default for AltitudeSettings {
    /// Tuned for a VTOL multirotor: `ω ≈ 1.5 rad/s` bandwidth, `ζ ≈ 0.7`
    /// damping. Error-envelope time constant `τ = 1/(ζω) ≈ 1 s`
    /// (2% settling ≈ `4τ ≈ 4 s`). Fast enough for altitude-hold control
    /// loops while rejecting prop-wash baro noise. Retune for slower
    /// platforms (balloons, fixed-wing) or noisier sensors — see the
    /// Tuning Guide in `README.md`.
    fn default() -> Self {
        Self {
            position_gain: 2.1,
            velocity_gain: 2.25,
        }
    }
}
