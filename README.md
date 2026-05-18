[![License:MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

# fusion-altitude

A `no_std`-friendly altitude and vertical-velocity estimator that fuses barometric pressure with gravity-compensated vertical acceleration from an AHRS via a complementary filter, yielding drift-corrected altitude.

Companion crate to [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs). Implements the altitude estimator proposed in [fusion-ahrs#27](https://github.com/wboayue/fusion-ahrs/issues/27).

> Status: early scaffolding. API is not yet stable.

## Why

`fusion-ahrs` (and the upstream xioTechnologies C library) is scoped to orientation — gyro + accel + mag. It does not estimate altitude. Barometers drift slowly and are noisy on short timescales; integrated vertical acceleration is accurate short-term but drifts without bound. Fusing the two gives a stable altitude signal suitable for drones, balloons, and other airborne platforms.

## Algorithm

Two-state complementary observer over altitude and vertical velocity:

```
residual = baro_altitude - h
v += a*dt + velocity_gain * residual
h += v*dt + position_gain * residual
```

- Accel is integrated twice (a → v → h); the baro residual feeds back into **both** states, so velocity also gets drift-corrected — not just altitude.
- Two independent gains tune the position and velocity time constants separately. Larger gains trust the baro more (faster correction, more noise/lag in the output); smaller gains trust the inertial integration more (smoother, slower to recover from drift).

The vertical acceleration input is expected to be gravity-compensated and in the Earth frame, positive = up. When paired with `fusion-ahrs`, this is the Z component of `ahrs.earth_acceleration()` under NWU/ENU conventions (negate for NED).

## Usage (planned API)

```rust
use fusion_ahrs::Ahrs;
use fusion_altitude::AltitudeEstimator;

let mut ahrs = Ahrs::new();
let mut altitude = AltitudeEstimator::new();

// Optionally: altitude.with_settings(custom_settings)
// Optionally: altitude.reset(initial_baro_altitude);

loop {
    let dt = 0.01; // 100 Hz

    ahrs.update(gyro, accel, mag, dt);
    let vertical_accel = ahrs.earth_acceleration().z; // m/s², Earth frame, +up
    let baro_altitude = read_barometer();              // m

    altitude.update(vertical_accel, baro_altitude, dt);

    println!(
        "alt = {:.2} m, vz = {:.2} m/s",
        altitude.altitude(),
        altitude.vertical_velocity(),
    );
}
```

`AltitudeEstimator::new()` constructs with defaults; `AltitudeEstimator::with_settings(s)` overrides. `reset(baro_altitude)` zeroes the reference explicitly; otherwise the first `update()` call auto-zeroes against the first baro sample.

## Settings (planned)

| Setting          | Type  | Range     | Typical | Effect |
|------------------|-------|-----------|---------|--------|
| `position_gain`  | `f32` | `> 0`     | `~0.3`  | Feedback gain from baro residual into the altitude state. Sets the altitude correction time constant `τ_h ≈ 1 / position_gain`. Larger → tighter tracking of baro, more baro noise visible. |
| `velocity_gain`  | `f32` | `> 0`     | `~0.05` | Feedback gain from baro residual into the velocity state. Damps integrated-acceleration drift in `v`. Larger → faster velocity correction, more sensitivity to baro noise; smaller → cleaner velocity but slower to recover from accel bias. |

The two gains together set a 2nd-order observer; choosing them as `K_h = 2ζω` and `K_v = ω²` (with damping `ζ ≈ 0.7` and bandwidth `ω`) gives a well-behaved critically-near-damped response.

## Installation

Not yet published.

```bash
# once released:
cargo add fusion-altitude
```

## Development

```bash
cargo test                         # all tests
cargo build --no-default-features  # verify no_std build
cargo fmt
cargo clippy
```

## License

MIT — see [LICENSE](LICENSE).
