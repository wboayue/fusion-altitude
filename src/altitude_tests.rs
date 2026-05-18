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
    // Constant baro reading of 10 m with no accel — filter must
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
