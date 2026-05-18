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

The two gains define the 2nd-order complementary filter's characteristic polynomial `s² + position_gain·s + velocity_gain`. The defaults target a VTOL multirotor (~1 s settling). See the next section to retune.

## Tuning Guide

Unlike a 1st-order complementary filter's single `α ∈ [0, 1]`, these gains carry physical units (`1/s` and `1/s²`) and their "natural range" depends on platform dynamics and sample rate. Don't tune the gains directly — pick a bandwidth and a damping ratio, then compute them:

```text
position_gain = 2 · ζ · ω        (1/s)
velocity_gain = ω²               (1/s²)
```

| Symbol | Meaning | Typical |
|---|---|---|
| `ω` | Filter bandwidth, rad/s. Sets how fast estimates respond to true motion. | platform-dependent (table below) |
| `ζ` | Damping ratio. Sets overshoot vs. sluggishness. | `0.7` (mild overshoot) to `1.0` (no overshoot) |

### Bandwidth by platform

| Platform | `ω` (rad/s) | `position_gain` | `velocity_gain` | ~Settling |
|---|---|---|---|---|
| Balloon / very slow | `0.2` | `0.28` | `0.04` | ~15 s |
| Fixed-wing | `0.5` | `0.70` | `0.25` | ~4 s |
| **VTOL multirotor (default)** | **`1.5`** | **`2.10`** | **`2.25`** | **~1 s** |
| Aggressive racing | `5.0` | `7.00` | `25.0` | ~0.3 s |

### Practical bounds

- **Lower bound** — `ω ≳ 0.1 rad/s`. Below that, accel bias drift dominates between barometer corrections.
- **Discretisation bound** — keep `ω · dt < 0.1` for the semi-implicit Euler step to faithfully track the continuous-time response. At 100 Hz that means `ω < 10 rad/s`. Beyond, the discrete dynamics deviate.
- **Noise bound** — the baro path is low-pass with corner `ω`. Higher `ω` admits more baro noise into altitude. Pick the lowest `ω` that meets your controller's bandwidth requirement.

### Bias tradeoff

A constant Z-axis accel bias `a_bias` (m/s²) produces a steady-state altitude bias

```text
e_h ≈ -a_bias / velocity_gain
```

So doubling `velocity_gain` halves accel-bias-driven altitude error — but admits more baro noise. There is no free lunch; this is why `ζ` is held near `0.7–1.0` and `ω` is the real knob. If accel bias dominates your error budget, a runtime accel-bias estimator (or a 3-state filter that estimates bias as a state) is the principled fix, not chasing `velocity_gain` upward.

### Quick recipe

1. Estimate the bandwidth your controller needs (rule of thumb: 3–5× your altitude-hold loop bandwidth).
2. Pick `ζ` — `0.7` if you can tolerate ~5% overshoot for snappier response, `1.0` for no overshoot.
3. Compute `position_gain = 2ζω`, `velocity_gain = ω²`.
4. Verify `ω · dt < 0.1` at your sample rate.
5. Run the `realistic_noise` example with your gains plugged in and confirm the predicted bias / RMS error match what you observe.

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
