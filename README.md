[![License:MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

# fusion-altitude

A `no_std`-friendly altitude and vertical-velocity estimator that fuses barometric pressure with gravity-compensated vertical acceleration from an AHRS via a complementary filter, yielding drift-corrected altitude.

Companion crate to [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs). Implements the altitude estimator proposed in [fusion-ahrs#27](https://github.com/wboayue/fusion-ahrs/issues/27).

> Status: early scaffolding. API is not yet stable.

## Why

`fusion-ahrs` (and the upstream xioTechnologies C library) is scoped to orientation — gyro + accel + mag. It does not estimate altitude. Barometers drift slowly and are noisy on short timescales; integrated vertical acceleration is accurate short-term but drifts without bound. Fusing the two gives a stable altitude signal suitable for drones, balloons, and other airborne platforms.

## Algorithm

Complementary filter:

- **High-pass** filtered integrated vertical acceleration — captures fast changes without baro lag
- **Low-pass** filtered barometric altitude — anchors the estimate against long-term drift
- Tunable corner frequency via filter coefficients

The vertical acceleration input is expected to be gravity-compensated and in the Earth frame. When paired with `fusion-ahrs`, this is exactly the Z component of `ahrs.earth_acceleration()`.

## Usage (planned API)

```rust
use fusion_ahrs::Ahrs;
use fusion_altitude::{AltitudeEstimator, AltitudeSettings};

let mut ahrs = Ahrs::new();
let mut altitude = AltitudeEstimator::new(AltitudeSettings::default());

loop {
    let dt = 0.01; // 100 Hz

    ahrs.update(gyro, accel, mag, dt);
    let vertical_accel = ahrs.earth_acceleration().z; // m/s², Earth frame
    let baro_altitude = read_barometer();              // m

    let est = altitude.update(vertical_accel, baro_altitude, dt);
    println!("alt = {:.2} m, vz = {:.2} m/s", est.altitude, est.velocity);
}
```

## Settings (planned)

| Setting       | Type  | Description |
|---------------|-------|-------------|
| `accel_alpha` | `f32` | High-pass coefficient for acceleration integration |
| `baro_alpha`  | `f32` | Low-pass coefficient for barometric altitude |

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
