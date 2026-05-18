use crate::types::AltitudeSettings;

/// Two-state complementary observer fusing vertical acceleration with
/// barometric altitude to produce drift-corrected altitude and vertical
/// velocity estimates.
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
        // Equivalent to the continuous-time observer
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
mod tests {
    use super::*;

    const DT: f32 = 0.01;

    #[test]
    fn first_update_auto_zeroes_reference() {
        let mut est = AltitudeEstimator::new();
        est.update(0.0, 123.4, DT);
        assert_eq!(est.altitude(), 123.4);
        assert_eq!(est.vertical_velocity(), 0.0);
    }

    #[test]
    fn explicit_reset_sets_reference() {
        let mut est = AltitudeEstimator::new();
        est.reset(50.0);
        assert_eq!(est.altitude(), 50.0);
        assert_eq!(est.vertical_velocity(), 0.0);
        // Subsequent first update does not re-zero.
        est.update(0.0, 200.0, DT);
        assert!(est.altitude() < 200.0);
    }

    #[test]
    fn stationary_converges_to_baro() {
        let mut est = AltitudeEstimator::new();
        for _ in 0..2000 {
            est.update(0.0, 100.0, DT);
        }
        assert!((est.altitude() - 100.0).abs() < 1e-3);
        assert!(est.vertical_velocity().abs() < 1e-3);
    }

    #[test]
    fn baro_drift_pulls_altitude() {
        let mut est = AltitudeEstimator::new();
        est.reset(0.0);
        // Constant baro reading of 10 m with no accel — observer must
        // track the baro reference over time.
        for _ in 0..5000 {
            est.update(0.0, 10.0, DT);
        }
        assert!((est.altitude() - 10.0).abs() < 1e-2);
    }

    #[test]
    fn constant_upward_accel_against_truthful_baro() {
        // Truth: h(t) = 0.5 * a * t², v(t) = a * t. Provide matching baro.
        let a = 0.2_f32; // m/s²
        let mut est = AltitudeEstimator::new();
        est.reset(0.0);

        let mut t = 0.0_f32;
        for _ in 0..1000 {
            t += DT;
            let truth_h = 0.5 * a * t * t;
            est.update(a, truth_h, DT);
        }

        let truth_h = 0.5 * a * t * t;
        let truth_v = a * t;
        assert!(
            (est.altitude() - truth_h).abs() < 0.05,
            "h={} truth={}",
            est.altitude(),
            truth_h
        );
        assert!(
            (est.vertical_velocity() - truth_v).abs() < 0.05,
            "v={} truth={}",
            est.vertical_velocity(),
            truth_v
        );
    }

    #[test]
    fn settings_default_is_well_damped() {
        let s = AltitudeSettings::default();
        // ζ = K_h / (2 sqrt(K_v))
        let zeta = s.position_gain / (2.0 * libm_sqrtf(s.velocity_gain));
        assert!(zeta > 0.5 && zeta < 1.0, "ζ={}", zeta);
    }

    // tiny no_std-friendly sqrt for the damping check
    fn libm_sqrtf(x: f32) -> f32 {
        // Newton's method, 8 iterations — fine for a test.
        let mut g = x;
        for _ in 0..8 {
            g = 0.5 * (g + x / g);
        }
        g
    }
}
