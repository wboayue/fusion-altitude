//! VTOL altitude fusion under realistic sensor noise.
//!
//! Same ±5 m, 0.1 Hz vertical oscillation as `with_fusion_ahrs`, but every
//! sensor carries representative MEMS-class noise + bias and the baro adds
//! a periodic prop-wash component on top of white noise.
//!
//! Sensor profile (1-sigma where Gaussian):
//!   gyro    bias 0.2 deg/s + noise 0.1 deg/s    (MPU/BMI-class)
//!   accel   bias up to 12 mg + noise 5 mg
//!   mag     noise 0.05 normalised units
//!   baro    noise 10 cm    + 30 cm prop-wash at 8 Hz
//!
//! Observed (seed 0xC0FFEE, 60 s):
//!   steady-state RMS error ≈ 6 cm
//!   steady-state max error ≈ 12 cm
//!   steady-state mean error ≈ +5 cm
//!
//! The +5 cm bias is a direct consequence of the Z-axis accel bias: in
//! steady-state the velocity loop holds `K_v · (z - h) = -a_bias`, so the
//! position residual is `-a_bias / K_v ≈ -0.118 / 2.25 ≈ -0.052 m`. Tighter
//! `velocity_gain` (or runtime accel-bias estimation, future work) reduces it.
//!
//! Reproducible: seeded PCG, fixed sample count.
//!
//! Run: `cargo run --release --example realistic_noise`

use fusion_ahrs::Ahrs;
use fusion_altitude::{AltitudeEstimator, GRAVITY};
use nalgebra::Vector3;
use rand::{RngExt, SeedableRng};
use rand_pcg::Pcg64;
use std::f32::consts::PI;

const SAMPLE_RATE_HZ: f32 = 100.0;
const DT: f32 = 1.0 / SAMPLE_RATE_HZ;
const DURATION_S: f32 = 60.0;
const SEED: u64 = 0xC0FFEE;
const TRANSIENT_S: f32 = 5.0;

const GYRO_BIAS_DPS: Vector3<f32> = Vector3::new(0.20, -0.15, 0.10);
const GYRO_NOISE_DPS: f32 = 0.10;
const ACCEL_BIAS_G: Vector3<f32> = Vector3::new(0.005, -0.008, 0.012);
const ACCEL_NOISE_G: f32 = 0.005;
const MAG_NOISE: f32 = 0.05;
const BARO_WHITE_NOISE_M: f32 = 0.10;
const BARO_PROPWASH_M: f32 = 0.30;
const PROPWASH_HZ: f32 = 8.0;

/// Standard normal sample via Box-Muller. Avoids pulling in `rand_distr`
/// just for one distribution.
fn gauss(rng: &mut Pcg64) -> f32 {
    let u1: f32 = rng.random_range(1e-7_f32..1.0);
    let u2: f32 = rng.random_range(0.0_f32..1.0);
    (-2.0_f32 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
}

fn main() {
    let mut rng = Pcg64::seed_from_u64(SEED);
    let mut ahrs = Ahrs::new();
    let mut altitude = AltitudeEstimator::new();

    let omega = 2.0 * PI * 0.1; // 0.1 Hz vertical oscillation
    let amplitude = 5.0_f32;

    let mut sum_err = 0.0_f64;
    let mut sum_err_sq = 0.0_f64;
    let mut max_err: f32 = 0.0;
    let mut steady_state_max: f32 = 0.0;
    let mut steady_state_n: u32 = 0;

    println!(
        "{:>6}  {:>10}  {:>10}  {:>8}  {:>11}  {:>11}  {:>10}",
        "t(s)", "truth_h(m)", "est_h(m)", "err(m)", "truth_v(m/s)", "est_v(m/s)", "baro(m)"
    );

    let n_samples = (DURATION_S * SAMPLE_RATE_HZ) as usize;
    let mut t = 0.0_f32;

    for i in 0..n_samples {
        let truth_h = amplitude * (omega * t).sin();
        let truth_v = amplitude * omega * (omega * t).cos();
        let truth_a = -amplitude * omega * omega * (omega * t).sin(); // m/s²

        // Body-frame measurements with bias + Gaussian noise.
        let gyro = GYRO_BIAS_DPS
            + Vector3::new(
                GYRO_NOISE_DPS * gauss(&mut rng),
                GYRO_NOISE_DPS * gauss(&mut rng),
                GYRO_NOISE_DPS * gauss(&mut rng),
            );

        // Level platform: only the Z accel sees motion + gravity.
        let accel_true = Vector3::new(0.0, 0.0, 1.0 + truth_a / GRAVITY);
        let accel = accel_true
            + ACCEL_BIAS_G
            + Vector3::new(
                ACCEL_NOISE_G * gauss(&mut rng),
                ACCEL_NOISE_G * gauss(&mut rng),
                ACCEL_NOISE_G * gauss(&mut rng),
            );

        let mag = Vector3::new(1.0, 0.0, 0.0)
            + Vector3::new(
                MAG_NOISE * gauss(&mut rng),
                MAG_NOISE * gauss(&mut rng),
                MAG_NOISE * gauss(&mut rng),
            );

        ahrs.update(gyro, accel, mag, DT);

        let vertical_accel = ahrs.earth_acceleration().z * GRAVITY;

        // Baro: white noise + prop-wash periodic component.
        let propwash = BARO_PROPWASH_M * (2.0 * PI * PROPWASH_HZ * t).sin();
        let baro_altitude = truth_h + propwash + BARO_WHITE_NOISE_M * gauss(&mut rng);

        altitude.update(vertical_accel, baro_altitude, DT);

        let err = altitude.altitude() - truth_h;
        max_err = max_err.max(err.abs());

        if t > TRANSIENT_S {
            sum_err += err as f64;
            sum_err_sq += (err as f64) * (err as f64);
            steady_state_max = steady_state_max.max(err.abs());
            steady_state_n += 1;
        }

        if i % 100 == 0 {
            println!(
                "{:6.2}  {:10.3}  {:10.3}  {:+8.3}  {:11.3}  {:11.3}  {:10.3}",
                t,
                truth_h,
                altitude.altitude(),
                err,
                truth_v,
                altitude.vertical_velocity(),
                baro_altitude,
            );
        }

        t += DT;
    }

    let n = steady_state_n as f64;
    let mean_err = (sum_err / n) as f32;
    let rms_err = ((sum_err_sq / n).sqrt()) as f32;

    println!(
        "\nSteady-state (t > {:.0} s, n = {}):",
        TRANSIENT_S, steady_state_n
    );
    println!("  mean error  = {:+.3} m", mean_err);
    println!("  RMS error   = {:.3} m", rms_err);
    println!("  max |error| = {:.3} m", steady_state_max);
    println!(
        "Overall max |error| = {:.3} m (includes init transient)",
        max_err
    );
}
