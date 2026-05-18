## Objective

A `no_std`-friendly altitude and vertical-velocity estimator that fuses barometric pressure with gravity-compensated vertical acceleration from an AHRS via a 3rd-order complementary observer (estimating altitude, vertical velocity, and Z-axis accel bias), yielding drift-corrected altitude with zero steady-state error from constant accel bias.

Companion crate to [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs). Designed to consume the Earth-frame acceleration output from that library and combine it with a barometric altitude source.

Tracks issue: https://github.com/wboayue/fusion-ahrs/issues/27

## Quick Reference

```bash
cargo test                          # run all tests
cargo build --no-default-features   # verify no_std / embedded compatibility
cargo fmt                           # format (required before commit)
cargo clippy                        # lint
cargo bench                         # criterion benchmarks
cargo run --release --example with_fusion_ahrs   # clean-signal AHRS → altitude demo
cargo run --release --example realistic_noise    # demo with MEMS-class noise + baro prop wash
```

## Architecture

- **Input**: vertical acceleration (m/s², gravity-compensated, Earth frame) + barometric altitude (m) + dt (s)
- **Output**: fused altitude (m), vertical velocity (m/s), Z-axis accel bias estimate (m/s²)
- **Compatibility**: `#![no_std]` (edition 2024)

### Source Layout

```
src/
  lib.rs                – public API re-exports, GRAVITY const
  altitude.rs           – AltitudeEstimator: new(), with_settings(), reset(),
                          update(), altitude(), vertical_velocity(), accel_bias()
  altitude_tests.rs     – unit tests for altitude.rs
  types.rs              – AltitudeSettings
  types_tests.rs        – unit tests for types.rs (when added)
```

### Algorithm

3rd-order complementary observer, three states (altitude + vertical velocity + Z accel bias). Semi-implicit Euler discretisation of `v̇ = (a - b) + K_v·(z - h)`, `ḣ = v + K_h·(z - h)`, `ḃ = -K_b·(z - h)`:

```text
residual        = baro_altitude - h
corrected_accel = a - b
v += (corrected_accel + velocity_gain * residual) * dt
h += (v + position_gain * residual) * dt    # uses the just-updated v
b -= bias_gain * residual * dt
```

- The `dt` factor on the residual terms is required for dimensional correctness: `K_v` has units 1/s², residual is m, so `K_v * residual * dt` is m/s — the units of `v`. Same chain for `K_b` (1/s³ × m × s = m/s²).
- Accel integrated twice (a → v → h); baro residual corrects **all three** states.
- Three independent gains: `position_gain` (= 2ζω + ω_b, units 1/s), `velocity_gain` (= ω² + 2ζω·ω_b, units 1/s²), `bias_gain` (= ω²·ω_b, units 1/s³). Cascaded pole placement: damped position/velocity pair at `ω`, slow real bias pole at `ω_b ≪ ω`.
- Sign of the bias update: if `h_hat` overshoots truth, residual is negative, and `b_hat` must increase to subtract more from the integrated accel. Hence `b -= K_b · residual · dt` (with residual = baro - h).
- Routh-Hurwitz stability for the characteristic polynomial `s³ + K_h s² + K_v s + K_b`: `K_h · K_v > K_b`. Asserted by `settings_default_satisfies_routh_hurwitz`.
- One gain (single `alpha`) is **wrong** for this output shape — velocity needs its own correction path. The 2-state form is wrong too — it parks at a constant altitude bias of `+a_bias/K_v` when the accel has any DC offset (the 3-state observer is the principled fix).

State: `altitude`, `velocity`, `accel_bias`, `reference_set` flag (auto-zero on first sample if `reset()` not called). `reset()` preserves `accel_bias` — it's a physical sensor property independent of the altitude reference frame.

### Tuning notes

- **Steady-state altitude error from constant accel bias is zero** with `bias_gain > 0`. Derivation: at steady state `ḃ = 0` forces `residual = 0`, so `h_hat = baro = h_true`; then `v̇ = 0` forces `b_hat = a_bias`. Verified by `estimates_constant_accel_bias_and_zeroes_altitude_error` test and by `examples/realistic_noise.rs` (steady-state mean error +0.9 cm with a +12 mg accel bias, down from +5 cm in the 2-state version).
- **Response time** for default VTOL gains (`ω = 1.5, ζ = 0.7, ω_b = 0.3`): position/velocity error-envelope time constant `τ = 1/(ζω) ≈ 1 s` (2% settling ≈ `4τ ≈ 4 s`). Bias-loop time constant `1/ω_b ≈ 3 s` (95% converged ≈ `3/ω_b ≈ 10 s`).
- **Loop separation**: keep `ω_b ≲ ω/5` so the bias loop doesn't fight position/velocity transients. Faster `ω_b` means quicker bias convergence at the cost of more baro-noise injection into the bias estimate.
- **Initial transient** is larger than the 2-state filter because the bias loop is spinning up at startup. Hold position during the first ~`3/ω_b` seconds, or set `bias_gain = 0.0` to recover the 2-state filter.

### API Shape (matches `fusion-ahrs` conventions)

- `AltitudeEstimator::new()` — defaults
- `AltitudeEstimator::with_settings(s)` — custom settings
- `reset(baro_altitude)` — explicit reference zero (preserves accel_bias)
- `update(vertical_accel, baro_altitude, dt)` — returns `()`
- `altitude()`, `vertical_velocity()`, `accel_bias()` — accessors (mirrors sibling's `quaternion()`, `linear_acceleration()` style)

### Integration with `fusion-ahrs`

Sibling-crate I/O units (verified against `fusion-ahrs/src/ahrs.rs` and `testdata/sensor_data.csv`, *not* the README which is misleading):

| API | Units |
|---|---|
| `Ahrs::update(gyro, …)` | gyro in **deg/s** (CSV: `Gyroscope X (deg/s)`) — not rad/s as the README zero-vector example comment suggests |
| `Ahrs::update(_, accel, …)` | accel in **g** (stationary level = `(0, 0, 1)`) |
| `Ahrs::update(_, _, mag, …)` | mag is a normalised direction |
| `linear_acceleration()`, `earth_acceleration()` | output in **g**, gravity removed |

For our pipeline: `vertical_accel = ahrs.earth_acceleration().z * fusion_altitude::GRAVITY` under NWU/ENU; negate for NED.

```rust,ignore
ahrs.update(gyro, accel, mag, dt);
let vertical_accel = ahrs.earth_acceleration().z * fusion_altitude::GRAVITY;
altitude.update(vertical_accel, baro_altitude_m, dt);
let h = altitude.altitude();
let vz = altitude.vertical_velocity();
```

### Dependencies
- **None** in the core crate. Algorithm is pure scalar `f32` arithmetic (`+`, `-`, `*`, `/`) — no transcendentals, no vector ops, no allocator. Keeps the dependency tree empty and the `no_std` story trivial.
- Add `nalgebra` (with `libm` feature) only if a future public API takes/returns `Vector3<f32>`; otherwise resist.
- Dev-deps: `criterion` (benches), `fusion-ahrs` + `nalgebra` (examples), `rand` + `rand_pcg` (reproducible noise in examples). None propagated to crate consumers.

### Benchmarks

`benches/altitude_benchmarks.rs` covers:

- `update_steady_state` — hot-path `update()` cost after the auto-zero branch is settled.
- `update_cold_first_sample` — first-call path (auto-zero branch).
- `loop_100hz_10s` — 1000 sequential updates, representative of a 10 s control loop at 100 Hz.
- `new_default` — construction overhead.

Run with `cargo bench`. Use `cargo bench --bench altitude_benchmarks -- --quick` for fast sanity-check runs without the full criterion warmup. Reference numbers (Apple Silicon, release build): steady-state update ~10 ns, 1000-update loop ~7 µs.

### Code Quality Standards
- Single-responsibility modules, minimal public API
- Zero-cost abstractions, no heap allocation on the hot path
- **Test file layout**: unit tests live in a **sibling file** next to the implementation (e.g. `altitude.rs` ↔ `altitude_tests.rs`), wired in via:
  ```rust
  #[cfg(test)]
  #[path = "altitude_tests.rs"]
  mod tests;
  ```
  Tests use `use super::*;` to reach the implementation. Integration tests still go in `tests/`. Do **not** nest `#[cfg(test)] mod tests { ... }` inline in the implementation file.
- Rustdoc on all public APIs with examples

## Development Guidelines
- Maintain embedded compatibility (`cargo build --no-default-features`)
- Use `nalgebra` types only where vectors are genuinely needed; scalars stay `f32`. The core crate currently has zero deps — preserve that.
- Keep `README.md` in sync with public API in the same PR
- Commit messages: conventional-commit style — `feat(altitude): …`, `fix: …`, `docs: …`, `chore(deps): …`, `fmt: …`

### Doc-tests and the README

`src/lib.rs` ingests the README via `#![doc = include_str!("../README.md")]`, so README fenced blocks are compiled as doc-tests by default. When editing README code blocks:

- **Pseudocode / equations** → tag the block ` ```text ` (otherwise Rust tries to compile it).
- **Sketches that reference external symbols** (e.g. `fusion_ahrs::Ahrs`, `read_barometer()`) → tag ` ```rust,ignore ` — `no_run` is not enough because they'd still fail to compile.
- **Real, runnable examples** → leave untagged or ` ```rust ` so they get type-checked.

After any README edit run `cargo test --doc` before committing.

## Design Decisions (locked unless reopened)

- **Separate crate, not a feature of `fusion-ahrs`.** `fusion-ahrs` deliberately tracks the upstream xioTechnologies C library (orientation only). Bundling baro/altitude in — even behind a cargo feature — would muddy that parity boundary and force altitude churn through the AHRS release cycle. Embedded users who only need orientation also keep a smaller surface.
- **3rd-order complementary observer with three gains, not 2-state or 1st-order.** Output is altitude, velocity, *and* accel bias (three states). A 1st-order filter corrects only one state; velocity drifts with accel bias. A 2-state filter corrects altitude and velocity but parks at `+a_bias / K_v` steady-state altitude error. The 3-state observer identifies the bias as a state and drives steady-state altitude error to zero. Cascaded pole placement (position/velocity loop at `ω`, bias loop at `ω_b ≪ ω`) keeps the position-loop tuning intuition unchanged and the bias loop decoupled. Characteristic polynomial `s³ + K_h s² + K_v s + K_b`; Routh-Hurwitz `K_h · K_v > K_b`. Don't collapse to fewer states.
- **API mirrors `fusion-ahrs`.** Constructor: `new()` + `with_settings(s)`. Updates return `()`; results via `altitude()` / `vertical_velocity()` / `accel_bias()` accessors — not a return-by-value snapshot. Consistency with the sibling crate beats minor ergonomic wins.
- **Scalar `vertical_accel: f32` input, +up convention.** Caller extracts `.z` from `earth_acceleration()` and negates if using NED. Keeps this crate independent of a `Convention` enum.
- **Reference handling: auto-zero on first sample, with explicit `reset(baro_altitude)` override.** Absolute altitude is meaningless without a reference; users get something working out of the box, with an escape hatch for known-ground starts. `reset()` preserves `accel_bias` — re-zeroing the altitude reference is unrelated to the physical sensor bias.

## Open Design Questions
- Whether to expose a `set_accel_bias(b)` API for warm-starting from a stationary pre-flight calibration (currently the bias has to converge from 0 over ~10 s).
- Filter form: keep 3rd-order complementary observer (current) vs. 3-state Kalman with explicit accel/baro noise models (later, if tuning needs become sensor-specific).
- Whether to expose intermediate state (baro residual, ground reference) for diagnostics.
- Whether to accept `Vector3<f32>` + a `Convention` instead of a scalar `vertical_accel` (cleaner call site, adds `Convention` coupling).

## Success Criteria
- Drift-corrected altitude tracks truth within bounded error on synthetic + recorded data
- Passes `no_std` build
- Composes cleanly with `fusion-ahrs` `earth_acceleration()`
- Clear examples and rustdoc
