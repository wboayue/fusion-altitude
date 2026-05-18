//! End-to-end demo of the `fusion-ahrs` → `fusion-altitude` pipeline.
//!
//! Synthesises a 30 s flight with the platform oscillating ±5 m at 0.1 Hz
//! while held level. Body-frame accelerometer readings are constructed so
//! the AHRS sees a stationary orientation; only the Z channel carries the
//! vertical motion. A noisy synthetic barometer provides the absolute
//! reference.
//!
//! Run: `cargo run --release --example with_fusion_ahrs`

use fusion_ahrs::Ahrs;
use fusion_altitude::{AltitudeEstimator, GRAVITY};
use nalgebra::Vector3;
use std::f32::consts::PI;

const SAMPLE_RATE_HZ: f32 = 100.0;
const DT: f32 = 1.0 / SAMPLE_RATE_HZ;
const DURATION_S: f32 = 30.0;

fn main() {
    let mut ahrs = Ahrs::new();
    let mut altitude = AltitudeEstimator::new();

    let omega = 2.0 * PI * 0.1; // 0.1 Hz vertical oscillation
    let amplitude = 5.0_f32; // metres

    println!(
        "{:>6}  {:>10}  {:>10}  {:>8}  {:>11}  {:>11}",
        "t(s)", "truth_h(m)", "est_h(m)", "err(m)", "truth_v(m/s)", "est_v(m/s)"
    );

    let n_samples = (DURATION_S * SAMPLE_RATE_HZ) as usize;
    let mut t = 0.0_f32;

    let mut max_err: f32 = 0.0;
    let mut steady_state_err: f32 = 0.0;

    for i in 0..n_samples {
        // Ground truth vertical motion (positive = up).
        let truth_h = amplitude * (omega * t).sin();
        let truth_v = amplitude * omega * (omega * t).cos();
        let truth_a = -amplitude * omega * omega * (omega * t).sin(); // m/s²

        // Body-frame sensors, NWU level orientation:
        //   gyro = 0 (no rotation)
        //   accel = gravity + motion, in units of g
        //   mag = constant unit vector pointing "north"
        // Under NWU the accelerometer reads +1 g in Z at rest; vertical
        // motion adds truth_a / GRAVITY (g) on top.
        let gyro = Vector3::zeros();
        let accel = Vector3::new(0.0, 0.0, 1.0 + truth_a / GRAVITY);
        let mag = Vector3::new(1.0, 0.0, 0.0);

        ahrs.update(gyro, accel, mag, DT);

        // earth_acceleration() returns g; convert to m/s².
        let vertical_accel = ahrs.earth_acceleration().z * GRAVITY;

        // Synthetic baro: truth + deterministic noise so the demo is
        // reproducible. 5 cm peak-to-peak is in the ballpark for a
        // decent MS5611-class sensor at 100 Hz.
        let baro_noise = 0.05 * (i as f32 * 0.41).sin();
        let baro_altitude = truth_h + baro_noise;

        altitude.update(vertical_accel, baro_altitude, DT);

        let err = altitude.altitude() - truth_h;
        max_err = max_err.max(err.abs());
        if t > 5.0 {
            // ignore the AHRS initialisation transient (~3 s) plus a
            // little settling time for the altitude observer.
            steady_state_err = steady_state_err.max(err.abs());
        }

        if i % 100 == 0 {
            println!(
                "{:6.2}  {:10.3}  {:10.3}  {:+8.3}  {:11.3}  {:11.3}",
                t,
                truth_h,
                altitude.altitude(),
                err,
                truth_v,
                altitude.vertical_velocity(),
            );
        }

        t += DT;
    }

    println!("\nMax altitude error: {:.3} m", max_err);
    println!(
        "Max steady-state error (t > 5 s): {:.3} m",
        steady_state_err
    );
}
