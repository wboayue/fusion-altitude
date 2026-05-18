## Objective

A `no_std`-friendly altitude and vertical-velocity estimator that fuses barometric pressure with gravity-compensated vertical acceleration from an AHRS via a 2nd-order complementary filter, yielding drift-corrected altitude.

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
- **Output**: fused altitude (m) and vertical velocity (m/s)
- **Compatibility**: `#![no_std]` (edition 2024)

### Source Layout

```
src/
  lib.rs                – public API re-exports, GRAVITY const
  altitude.rs           – AltitudeEstimator: new(), with_settings(), reset(),
                          update(), altitude(), vertical_velocity()
  altitude_tests.rs     – unit tests for altitude.rs
  types.rs              – AltitudeSettings
  types_tests.rs        – unit tests for types.rs (when added)
```

### Algorithm

2nd-order complementary filter, two states (altitude + vertical velocity). Semi-implicit Euler discretisation of `v̇ = a + K_v·(z - h)`, `ḣ = v + K_h·(z - h)`:

```text
residual = baro_altitude - h
v += (a + velocity_gain * residual) * dt
h += (v + position_gain * residual) * dt    # uses the just-updated v
```

- The `dt` factor on the residual terms is required for dimensional correctness: `K_v` has units 1/s², residual is m, so `K_v * residual * dt` is m/s — the units of `v`.
- Accel integrated twice (a → v → h); baro residual corrects **both** states, not just altitude
- Two independent gains: `position_gain` (≈ 2ζω, units 1/s) and `velocity_gain` (≈ ω², units 1/s²) define the 2nd-order complementary filter
- One gain (single `alpha`) is **wrong** for this output shape — velocity needs its own correction path or it drifts with accel bias

State: `altitude`, `velocity`, `reference_set` flag (auto-zero on first sample if `reset()` not called).

### API Shape (matches `fusion-ahrs` conventions)

- `AltitudeEstimator::new()` — defaults
- `AltitudeEstimator::with_settings(s)` — custom settings
- `reset(baro_altitude)` — explicit reference zero
- `update(vertical_accel, baro_altitude, dt)` — returns `()`
- `altitude()`, `vertical_velocity()` — accessors (mirrors sibling's `quaternion()`, `linear_acceleration()` style)

### Integration with `fusion-ahrs`

`fusion-ahrs::Ahrs::earth_acceleration()` returns **g**, not m/s². Multiply by `fusion_altitude::GRAVITY` (= 9.80665) to convert. Sign: positive Z is up under NWU/ENU; negate for NED.

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
- **2nd-order complementary filter with two gains, not single-alpha 1st-order.** Output is altitude *and* velocity (two states). A 1st-order complementary filter only corrects one state; velocity then drifts with accel bias. Two gains (`position_gain`, `velocity_gain`) are physically independent — they set the position and velocity time constants of the 2nd-order filter (`K_h = 2ζω`, `K_v = ω²`). The frequency-domain check passes: baro path `(K_h·s + K_v) / (s² + K_h·s + K_v)` and accel path `1 / (s² + K_h·s + K_v)` are complementary on the true altitude signal. Don't collapse to one parameter.
- **API mirrors `fusion-ahrs`.** Constructor: `new()` + `with_settings(s)`. Updates return `()`; results via `altitude()` / `vertical_velocity()` accessors — not a return-by-value `AltitudeEstimate` snapshot. Consistency with the sibling crate beats minor ergonomic wins.
- **Scalar `vertical_accel: f32` input, +up convention.** Caller extracts `.z` from `earth_acceleration()` and negates if using NED. Keeps this crate independent of a `Convention` enum.
- **Reference handling: auto-zero on first sample, with explicit `reset(baro_altitude)` override.** Absolute altitude is meaningless without a reference; users get something working out of the box, with an escape hatch for known-ground starts.

## Open Design Questions
- Filter form: 2nd-order complementary filter (start here) vs. 2-state Kalman with explicit accel/baro noise models (later, if tuning needs are sensor-specific)
- Whether to expose intermediate state (baro residual, ground reference) for diagnostics
- Whether to accept `Vector3<f32>` + a `Convention` instead of a scalar `vertical_accel` (cleaner call site, adds `Convention` coupling)

## Success Criteria
- Drift-corrected altitude tracks truth within bounded error on synthetic + recorded data
- Passes `no_std` build
- Composes cleanly with `fusion-ahrs` `earth_acceleration()`
- Clear examples and rustdoc
