use crate::types::AltitudeSettings;

/// 2nd-order complementary filter fusing vertical acceleration with
/// barometric altitude to produce drift-corrected altitude and vertical
/// velocity estimates. The two filter states are altitude and vertical
/// velocity; both are corrected by the baro residual through independent
/// gains.
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
            reference_set: false,
        }
    }

    /// Set the altitude reference explicitly to `baro_altitude` and zero
    /// the velocity estimate.
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

        // Semi-implicit Euler: update v first, then h with the new v.
        // Equivalent to the continuous-time 2nd-order complementary filter
        //   v̇ = a + K_v (z - h),  ḣ = v + K_h (z - h)
        // discretised symplectically.
        self.velocity += (vertical_accel + self.settings.velocity_gain * residual) * dt;
        self.altitude += (self.velocity + self.settings.position_gain * residual) * dt;
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
}

impl Default for AltitudeEstimator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "altitude_tests.rs"]
mod tests;
