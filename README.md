[![License:MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

# fusion-altitude

A `no_std`-friendly altitude and vertical-velocity estimator that fuses barometric pressure with gravity-compensated vertical acceleration from an AHRS via a 2nd-order complementary filter, yielding drift-corrected altitude.

Companion crate to [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs). Implements the altitude estimator proposed in [fusion-ahrs#27](https://github.com/wboayue/fusion-ahrs/issues/27).

> Status: early scaffolding. API is not yet stable.

## Why

`fusion-ahrs` (and the upstream xioTechnologies C library) is scoped to orientation — gyro + accel + mag. It does not estimate altitude. Barometers drift slowly and are noisy on short timescales; integrated vertical acceleration is accurate short-term but drifts without bound. Fusing the two gives a stable altitude signal suitable for drones, balloons, and other airborne platforms.

## Algorithm

2nd-order complementary filter with two states — altitude `h` and vertical velocity `v`. Semi-implicit Euler discretisation of the continuous-time form `v̇ = a + K_v·(z - h)`, `ḣ = v + K_h·(z - h)`:

```text
residual = baro_altitude - h
v += (a + velocity_gain * residual) * dt
h += (v + position_gain * residual) * dt    # uses the just-updated v
```

- Accel is integrated twice (a → v → h); the baro residual feeds back into **both** states, so velocity is drift-corrected too — not just altitude.
- Two independent gains tune the position and velocity time constants separately. Larger gains trust the baro more (faster correction, more noise/lag in the output); smaller gains trust the inertial integration more (smoother, slower to recover from drift).

The vertical acceleration input is expected to be gravity-compensated, Earth frame, in **m/s²**, positive = up. When paired with `fusion-ahrs`, `earth_acceleration()` returns units of g — multiply by `fusion_altitude::GRAVITY` (and negate for NED).

## Usage

```rust,ignore
use fusion_ahrs::Ahrs;
use fusion_altitude::AltitudeEstimator;

let mut ahrs = Ahrs::new();
let mut altitude = AltitudeEstimator::new();

// Optionally: altitude.with_settings(custom_settings)
// Optionally: altitude.reset(initial_baro_altitude);

loop {
    let dt = 0.01; // 100 Hz

    ahrs.update(gyro, accel, mag, dt);
    // earth_acceleration() returns g; convert to m/s². Negate for NED.
    let vertical_accel = ahrs.earth_acceleration().z * fusion_altitude::GRAVITY;
    let baro_altitude = read_barometer(); // m

    altitude.update(vertical_accel, baro_altitude, dt);

    println!(
        "alt = {:.2} m, vz = {:.2} m/s",
        altitude.altitude(),
        altitude.vertical_velocity(),
    );
}
```

`AltitudeEstimator::new()` constructs with defaults; `AltitudeEstimator::with_settings(s)` overrides. `reset(baro_altitude)` zeroes the reference explicitly; otherwise the first `update()` call auto-zeroes against the first baro sample.

## Settings

| Setting          | Type  | Units  | Default (VTOL) | Effect |
|------------------|-------|--------|----------------|--------|
| `position_gain`  | `f32` | `1/s`  | `2.1`          | Feedback gain from baro residual into the altitude state. Larger → tighter tracking of baro, more baro noise visible in altitude. |
| `velocity_gain`  | `f32` | `1/s²` | `2.25`         | Feedback gain from baro residual into the velocity state. Damps integrated-acceleration drift in `v`. Larger → faster velocity correction, more sensitivity to baro noise. |

The two gains define the 2nd-order complementary filter's characteristic polynomial `s² + position_gain·s + velocity_gain`. Pick `position_gain = 2ζω`, `velocity_gain = ω²` with damping `ζ ≈ 0.7` and bandwidth `ω` (rad/s). The defaults give `ω ≈ 1.5 rad/s` (~1 s settling) — fast enough for VTOL multirotor altitude-hold loops while rejecting prop-wash baro noise. Retune for slower platforms (balloons, fixed-wing) or noisier sensors.

## Installation

Not yet published.

```bash
# once released:
cargo add fusion-altitude
```

## Examples

```bash
cargo run --release --example with_fusion_ahrs    # clean signal, ~3 cm steady-state error
cargo run --release --example realistic_noise     # MEMS-class noise + prop wash, ~6 cm RMS
```

`with_fusion_ahrs` shows the full `fusion-ahrs` → `fusion-altitude` pipeline on a synthetic ±5 m, 0.1 Hz vertical oscillation. `realistic_noise` adds representative gyro/accel biases, Gaussian sensor noise, and a periodic prop-wash component on the barometer — see the file header for the sensor budget and the predicted/observed error.

## Development

```bash
cargo test                         # all tests
cargo build --no-default-features  # verify no_std build
cargo bench                        # criterion benches
cargo fmt
cargo clippy
```

## License

MIT — see [LICENSE](LICENSE).
