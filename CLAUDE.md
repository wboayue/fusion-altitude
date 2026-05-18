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
  altitude.rs     – AltitudeEstimator: update(), state accessors
  types.rs        – AltitudeSettings, AltitudeEstimate
  math.rs         – filter helpers (high-pass / low-pass primitives)
```

### Algorithm

Complementary filter:
- **High-pass** integrated vertical acceleration — short-term accuracy, no baro lag
- **Low-pass** barometric altitude — long-term drift correction
- Single tunable blend weight `alpha` via `AltitudeSettings` (accel path gets `alpha`, baro path gets `1 - alpha`)

State: `velocity`, `altitude`, `baro_reference`, previous accel/baro samples.

### Integration with `fusion-ahrs`

```rust
ahrs.update(gyro, accel, mag, dt);
let vertical_accel = ahrs.earth_acceleration().z;
let est = altitude.update(vertical_accel, baro_altitude_m, dt);
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
- Filter form: simple complementary (start here) vs. 2-state Kalman (later, if drift demands it)
- Baro reference handling: auto-zero on first sample vs. explicit `set_reference()`
- Whether to expose intermediate state (raw integrated velocity, baro residual) for diagnostics

## Success Criteria
- Drift-corrected altitude tracks truth within bounded error on synthetic + recorded data
- Passes `no_std` build
- Composes cleanly with `fusion-ahrs` `earth_acceleration()`
- Clear examples and rustdoc
