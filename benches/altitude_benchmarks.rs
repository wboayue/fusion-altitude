use criterion::{Criterion, criterion_group, criterion_main};
use fusion_altitude::AltitudeEstimator;
use std::f32::consts::PI;
use std::hint::black_box;

const SAMPLE_RATE_HZ: f32 = 100.0;
const DT: f32 = 1.0 / SAMPLE_RATE_HZ;

/// Pre-generated `(vertical_accel, baro_altitude)` samples representing a
/// realistic vertical-motion profile. Eliminates per-iteration arithmetic
/// from the hot path so the benchmark measures `update()` cost alone.
fn generate_samples(count: usize) -> Vec<(f32, f32)> {
    let mut samples = Vec::with_capacity(count);
    for i in 0..count {
        let t = i as f32 * DT;

        // Oscillating climb/descent: 5 m amplitude at 0.1 Hz
        let omega = 2.0 * PI * 0.1;
        let truth_h = 5.0 * (omega * t).sin();
        let truth_a = -5.0 * omega * omega * (omega * t).sin();

        // Mild measurement noise via deterministic offsets so the bench
        // result is reproducible. The filter is unconditional on input
        // values so realism is not load-bearing.
        let baro_noise = 0.02 * ((i as f32 * 0.37).sin());
        let accel_noise = 0.05 * ((i as f32 * 0.91).cos());

        samples.push((truth_a + accel_noise, truth_h + baro_noise));
    }
    samples
}

fn bench_update_steady_state(c: &mut Criterion) {
    let samples = generate_samples(1024);
    let mut est = AltitudeEstimator::new();
    est.reset(0.0); // skip the auto-zero branch

    let mut i = 0usize;
    c.bench_function("update_steady_state", |b| {
        b.iter(|| {
            let (a, h) = samples[i & 1023];
            est.update(black_box(a), black_box(h), black_box(DT));
            i = i.wrapping_add(1);
            black_box(est.altitude())
        })
    });
}

fn bench_update_cold(c: &mut Criterion) {
    // Includes the first-call auto-zero branch on every iteration by
    // constructing a fresh estimator each time.
    let samples = generate_samples(1024);
    let mut i = 0usize;
    c.bench_function("update_cold_first_sample", |b| {
        b.iter(|| {
            let mut est = AltitudeEstimator::new();
            let (a, h) = samples[i & 1023];
            est.update(black_box(a), black_box(h), black_box(DT));
            i = i.wrapping_add(1);
            black_box(est.altitude())
        })
    });
}

fn bench_loop_100hz_10s(c: &mut Criterion) {
    // 1000 update calls — representative of a 10-second control loop
    // running at 100 Hz. Amortises any per-iteration harness overhead and
    // matches the kind of total work an embedded MCU does per flight second.
    let samples = generate_samples(1000);
    c.bench_function("loop_100hz_10s", |b| {
        b.iter(|| {
            let mut est = AltitudeEstimator::new();
            for &(a, h) in &samples {
                est.update(black_box(a), black_box(h), black_box(DT));
            }
            black_box((est.altitude(), est.vertical_velocity()))
        })
    });
}

fn bench_construction(c: &mut Criterion) {
    c.bench_function("new_default", |b| {
        b.iter(|| black_box(AltitudeEstimator::new()))
    });
}

criterion_group!(
    benches,
    bench_update_steady_state,
    bench_update_cold,
    bench_loop_100hz_10s,
    bench_construction,
);
criterion_main!(benches);
