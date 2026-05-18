## Objective

A `no_std`-friendly altitude and vertical-velocity estimator that fuses barometric pressure with gravity-compensated vertical acceleration from an AHRS via a complementary filter, yielding drift-corrected altitude.

Companion crate to [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs). Designed to consume the Earth-frame acceleration output from that library and combine it with a barometric altitude source.

Tracks issue: https://github.com/wboayue/fusion-ahrs/issues/27

## Quick Reference

```bash
cargo test                          # run all tests
cargo build --no-default-features   # verify no_std / embedded compatibility
cargo fmt                           # format (required before commit)
cargo clippy                        # lint
cargo bench                         # criterion benchmarks
cargo run --example simple          # basic altitude fusion demo
```

## Architecture

- **Input**: vertical acceleration (m/s², gravity-compensated, Earth frame) + barometric altitude (m) + dt (s)
- **Output**: fused altitude (m) and vertical velocity (m/s)
- **Compatibility**: `#![no_std]` (edition 2024)

### Source Layout (planned)

```
src/
  lib.rs          – public API re-exports
  altitude.rs     – AltitudeEstimator: new(), with_settings(), reset(),
                    update(), altitude(), vertical_velocity()
  types.rs        – AltitudeSettings
```

### Algorithm

Two-state complementary observer (altitude + vertical velocity):

```
residual = baro_altitude - h
v += a*dt + velocity_gain * residual
h += v*dt + position_gain * residual
```

- Accel integrated twice (a → v → h); baro residual corrects **both** states, not just altitude
- Two independent gains: `position_gain` (≈ 2ζω) and `velocity_gain` (≈ ω²) for a 2nd-order observer
- One gain (single `alpha`) is **wrong** for this output shape — velocity needs its own correction path or it drifts with accel bias

State: `altitude`, `velocity`, `reference_set` flag (auto-zero on first sample if `reset()` not called).

### API Shape (matches `fusion-ahrs` conventions)

- `AltitudeEstimator::new()` — defaults
- `AltitudeEstimator::with_settings(s)` — custom settings
- `reset(baro_altitude)` — explicit reference zero
- `update(vertical_accel, baro_altitude, dt)` — returns `()`
- `altitude()`, `vertical_velocity()` — accessors (mirrors sibling's `quaternion()`, `linear_acceleration()` style)

### Integration with `fusion-ahrs`

```rust
ahrs.update(gyro, accel, mag, dt);
let vertical_accel = ahrs.earth_acceleration().z; // +up under NWU/ENU; negate for NED
altitude.update(vertical_accel, baro_altitude_m, dt);
let h = altitude.altitude();
let vz = altitude.vertical_velocity();
```

### Dependencies
- `nalgebra` (no-std, `libm` feature) — match `fusion-ahrs` choice
- Dev only: `criterion`, `plotters`, `csv`, `serde` for tests/examples

### Code Quality Standards
- Single-responsibility modules, minimal public API
- Zero-cost abstractions, no heap allocation on the hot path
- Inline unit tests in each module (`#[cfg(test)] mod tests`); integration tests in `tests/`
- Rustdoc on all public APIs with examples

## Development Guidelines
- Maintain embedded compatibility (`cargo build --no-default-features`)
- Use `nalgebra` types where vectors are needed; scalars stay `f32`
- Keep `README.md` in sync with public API in the same PR
- Commit messages: conventional-commit style — `feat(altitude): …`, `fix: …`, `docs: …`, `chore(deps): …`, `fmt: …`

## Open Design Questions
- Filter form: 2-state complementary observer (start here) vs. 2-state Kalman with explicit accel/baro noise models (later, if tuning needs are sensor-specific)
- Whether to expose intermediate state (baro residual, ground reference) for diagnostics
- Whether to accept `Vector3<f32>` + a `Convention` instead of a scalar `vertical_accel` (cleaner call site, adds `Convention` coupling)

## Success Criteria
- Drift-corrected altitude tracks truth within bounded error on synthetic + recorded data
- Passes `no_std` build
- Composes cleanly with `fusion-ahrs` `earth_acceleration()`
- Clear examples and rustdoc
