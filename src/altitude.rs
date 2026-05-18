use crate::types::AltitudeSettings;

/// 3rd-order complementary observer fusing vertical acceleration with
/// barometric altitude to produce drift-corrected altitude, vertical
/// velocity, and a running estimate of the Z-axis accelerometer bias.
/// All three states are corrected by the baro residual through
/// independent gains; the estimated bias is subtracted from the measured
/// acceleration before integration, so a constant accel bias produces
/// **no** steady-state altitude error.
///
/// # Example
///
/// ```
/// use fusion_altitude::AltitudeEstimator;
///
/// let mut est = AltitudeEstimator::new();
/// let dt = 0.01_f32;
///
/// // Stationary at 100 m: integrating zero accel against a steady baro
/// // reading converges to the baro value.
/// for _ in 0..1000 {
///     est.update(0.0, 100.0, dt);
/// }
///
/// assert!((est.altitude() - 100.0).abs() < 0.01);
/// assert!(est.vertical_velocity().abs() < 0.01);
/// ```
#[derive(Debug, Clone)]
pub struct AltitudeEstimator {
    settings: AltitudeSettings,
    altitude: f32,
    velocity: f32,
    accel_bias: f32,
    reference_set: bool,
}

impl AltitudeEstimator {
    /// Construct with default settings. See [`AltitudeSettings::default`].
    pub fn new() -> Self {
        Self::with_settings(AltitudeSettings::default())
    }

    /// Construct with custom settings.
    pub fn with_settings(settings: AltitudeSettings) -> Self {
        Self {
            settings,
            altitude: 0.0,
            velocity: 0.0,
            accel_bias: 0.0,
            reference_set: false,
        }
    }

    /// Set the altitude reference explicitly to `baro_altitude` and zero
    /// the velocity estimate. The accel-bias estimate is **preserved** —
    /// it's a physical sensor property independent of the altitude
    /// reference frame, and a converged bias is worth keeping across a
    /// reference-zero event.
    ///
    /// If `reset` is not called before the first `update`, the first baro
    /// sample auto-zeroes the reference.
    pub fn reset(&mut self, baro_altitude: f32) {
        self.altitude = baro_altitude;
        self.velocity = 0.0;
        self.reference_set = true;
    }

    /// Advance the filter by `dt` seconds using the latest sensor inputs.
    ///
    /// * `vertical_accel` — gravity-compensated vertical acceleration in
    ///   m/s², positive = up. For `fusion-ahrs` users:
    ///   `ahrs.earth_acceleration().z * fusion_altitude::GRAVITY` under
    ///   NWU/ENU (negate for NED).
    /// * `baro_altitude` — barometric altitude in meters (any reference).
    /// * `dt` — time since the previous `update`, in seconds.
    pub fn update(&mut self, vertical_accel: f32, baro_altitude: f32, dt: f32) {
        if !self.reference_set {
            self.reset(baro_altitude);
            return;
        }

        let residual = baro_altitude - self.altitude;
        let corrected_accel = vertical_accel - self.accel_bias;

        // Semi-implicit Euler discretisation of the continuous-time
        // observer
        //   v̇ = (a - b) + K_v (z - h)
        //   ḣ = v       + K_h (z - h)
        //   ḃ = -K_b (z - h)
        // The bias-update sign comes from the error dynamics: if h_hat
        // overshoots truth, residual is negative, and b_hat must increase
        // to subtract more from the integrated accel.
        self.velocity += (corrected_accel + self.settings.velocity_gain * residual) * dt;
        self.altitude += (self.velocity + self.settings.position_gain * residual) * dt;
        self.accel_bias -= self.settings.bias_gain * residual * dt;
    }

    /// Current fused altitude estimate (m), in the reference frame
    /// established by the first baro sample or the most recent `reset`.
    pub fn altitude(&self) -> f32 {
        self.altitude
    }

    /// Current fused vertical velocity estimate (m/s, positive = up).
    pub fn vertical_velocity(&self) -> f32 {
        self.velocity
    }

    /// Current estimate of the additive Z-axis accelerometer bias
    /// (m/s², +up). Converges to the true sensor bias over roughly
    /// `3 / ω_b` seconds (~10 s with defaults).
    pub fn accel_bias(&self) -> f32 {
        self.accel_bias
    }
}

impl Default for AltitudeEstimator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "altitude_tests.rs"]
mod tests;
