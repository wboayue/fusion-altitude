[![License:MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Crates.io](https://img.shields.io/crates/v/fusion-altitude.svg)](https://crates.io/crates/fusion-altitude)
[![Coverage Status](https://coveralls.io/repos/github/wboayue/fusion-altitude/badge.svg?branch=main)](https://coveralls.io/github/wboayue/fusion-altitude?branch=main)

# fusion-altitude

A `no_std`-friendly altitude and vertical-velocity estimator that fuses barometric pressure with gravity-compensated vertical acceleration from an AHRS via a 3rd-order complementary observer, yielding drift-corrected altitude and online accel-bias estimation.

Companion crate to [`fusion-ahrs`](https://github.com/wboayue/fusion-ahrs). Implements the altitude estimator proposed in [fusion-ahrs#27](https://github.com/wboayue/fusion-ahrs/issues/27).

> Status: early scaffolding. API is not yet stable.

## Why

`fusion-ahrs` (and the upstream xioTechnologies C library) is scoped to orientation — gyro + accel + mag. It does not estimate altitude. Barometers drift slowly and are noisy on short timescales; integrated vertical acceleration is accurate short-term but drifts without bound. Fusing the two gives a stable altitude signal suitable for drones, balloons, and other airborne platforms.

## Algorithm

3rd-order complementary observer with three states — altitude `h`, vertical velocity `v`, and additive Z-axis accel bias `b`. Semi-implicit Euler discretisation of the continuous-time form `v̇ = (a - b) + K_v·(z - h)`, `ḣ = v + K_h·(z - h)`, `ḃ = -K_b·(z - h)`:

```text
residual        = baro_altitude - h
corrected_accel = a - b
v += (corrected_accel + velocity_gain * residual) * dt
h += (v + position_gain * residual) * dt    # uses the just-updated v
b -= bias_gain * residual * dt
```

- Accel is integrated twice (a → v → h); the baro residual feeds back into **all three** states, so velocity is drift-corrected and a constant accel bias produces **no** steady-state altitude error.
- Three independent gains tune the position, velocity, and bias time constants. Larger gains trust the baro more (faster correction, more noise/lag in the output); smaller gains trust the inertial integration more (smoother, slower to recover from drift).
- The bias loop is intentionally slow (~10 s default settling) — fast enough to converge within a typical flight, slow enough to avoid chasing transients or short-term baro noise.

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

### Diagnostics

`baro_residual()` returns the baro innovation `baro_altitude - altitude()` from the most recent `update`, sign matching the residual that drove the correction. Useful at the outer loop / autopilot for fault detection (sustained large residual ⇒ baro stuck, prop-wash, or filter divergence), confidence weighting, or innovation-gated handoff to a nav EKF. Zero before the first `update` and immediately after `reset`.

## Settings

| Setting          | Type  | Units   | Default (VTOL) | Effect |
|------------------|-------|---------|----------------|--------|
| `position_gain`  | `f32` | `1/s`   | `2.40`         | Feedback gain from baro residual into the altitude state. Larger → tighter tracking of baro, more baro noise visible in altitude. |
| `velocity_gain`  | `f32` | `1/s²`  | `2.88`         | Feedback gain from baro residual into the velocity state. Damps integrated-acceleration drift in `v`. |
| `bias_gain`      | `f32` | `1/s³`  | `0.675`        | Feedback gain from baro residual into the accel-bias state. Sets the bias-loop bandwidth (`ω_b ≈ bias_gain / velocity_gain`). Set to `0.0` to disable bias estimation (recovers a 2-state filter). |

The three gains define the observer's characteristic polynomial `s³ + position_gain·s² + velocity_gain·s + bias_gain`. Stability (Routh-Hurwitz) requires `position_gain · velocity_gain > bias_gain`. The defaults target a VTOL multirotor with ~1 s position settling and ~10 s bias settling. See the next section to retune.

## Tuning Guide

Don't tune the gains directly — they carry physical units (`1/s`, `1/s²`, `1/s³`) and are coupled. Pick a position-loop bandwidth `ω`, a damping ratio `ζ`, and a separate (slower) bias-loop bandwidth `ω_b`, then compute the gains:

```text
position_gain = 2·ζ·ω + ω_b           (1/s)
velocity_gain = ω² + 2·ζ·ω·ω_b        (1/s²)
bias_gain     = ω² · ω_b              (1/s³)
```

| Symbol | Meaning | Typical |
|---|---|---|
| `ω`   | Position-loop bandwidth (rad/s). Sets how fast `h, v` respond to true motion. | platform-dependent (table below) |
| `ζ`   | Damping ratio of the position/velocity loop. | `0.7` (mild overshoot) to `1.0` (no overshoot) |
| `ω_b` | Bias-loop bandwidth (rad/s). Sets how fast `b` converges. Must be ≪ `ω` so the loops are decoupled. | `ω / 5` (10× slower) |

### Bandwidth by platform

Values below use `ζ = 0.7` and `ω_b = ω / 5`.

| Platform | `ω` (rad/s) | `ω_b` (rad/s) | `position_gain` | `velocity_gain` | `bias_gain` | Position `τ = 1/(ζω)` | Bias `1/ω_b` |
|---|---|---|---|---|---|---|---|
| Balloon / very slow         | `0.2` | `0.04` | `0.32` | `0.06`  | `0.008` | ~7 s   | 25 s |
| Fixed-wing                  | `0.5` | `0.10` | `0.80` | `0.35`  | `0.025` | ~3 s   | 10 s |
| **VTOL multirotor (default)** | **`1.5`** | **`0.30`** | **`2.40`** | **`2.88`** | **`0.675`** | **~1 s** | **~3 s** |
| Aggressive racing           | `5.0` | `1.00` | `8.00` | `32.00` | `25.00` | ~0.3 s | 1 s  |

Position `τ` is the error-envelope decay constant; 2% settling ≈ `4τ`. Bias `1/ω_b` is the bias-loop time constant; bias estimate is converged within ~`3/ω_b` seconds.

### Practical bounds

- **Lower bound on `ω`** — `ω ≳ 0.1 rad/s`. Below that, accel noise and baro drift dominate the bias-loop input.
- **Discretisation bound** — keep `ω · dt < 0.1` for the semi-implicit Euler step to faithfully track the continuous-time response. At 100 Hz that means `ω < 10 rad/s`. Beyond, the discrete dynamics deviate.
- **Loop separation** — keep `ω_b ≲ ω / 5` so the bias loop doesn't fight the position/velocity transient.
- **Noise bound** — the baro path is low-pass with corner ≈ `ω`. Higher `ω` admits more baro noise into altitude. Pick the lowest `ω` that meets your controller's bandwidth requirement.

### What changed: bias is now estimated, not parked

The previous 2-state filter parked at a steady-state altitude error of `+a_bias / velocity_gain` (≈ +5 cm for a 12 mg Z accel bias with the old `velocity_gain = 2.25`). The 3-state observer drives that error to **zero** by identifying the bias as a state.

The cost is a slower initial transient: the bias loop takes ~`3/ω_b` seconds to converge. With defaults that's ~10 s. **Motion does not slow convergence** — the error dynamics are autonomous (the true trajectory cancels between truth and filter), so bias converges normally during any flight regime, hover or otherwise. The "transient" is the cold-filter startup, not a stationary-only requirement. If you cannot tolerate a startup transient at all, set `bias_gain = 0.0` to keep the original 2-state behavior. A future API for warm-starting `accel_bias` from a stationary pre-flight calibration is tracked in `CLAUDE.md` under Open Design Questions — not currently exposed.

Caveat: AHRS attitude error during sustained pitch/roll couples into the bias estimate (rotation-induced error in `earth_acceleration().z` is indistinguishable from real accel bias to this filter). On a level platform this is zero; on aggressive maneuvers the estimate absorbs a few mg of effective bias. Usually a feature — you want the lumped DC offset gone — but worth knowing.

### Quick recipe

1. Estimate the bandwidth your controller needs (rule of thumb: 3–5× your altitude-hold loop bandwidth) — that's `ω`.
2. Pick `ζ` — `0.7` if you can tolerate ~5% overshoot, `1.0` for no overshoot.
3. Pick `ω_b ≈ ω / 5` (or slower if you want bias to be conservative).
4. Compute `position_gain`, `velocity_gain`, `bias_gain` from the formulas above.
5. Verify `ω · dt < 0.1` at your sample rate, and `position_gain · velocity_gain > bias_gain` (Routh-Hurwitz).
6. Run the `realistic_noise` example with your gains plugged in and confirm the steady-state bias / RMS error match what you observe.

## Installation

```bash
cargo add fusion-altitude
```

## Examples

```bash
cargo run --release --example with_fusion_ahrs    # clean signal, ~3 cm typical / ~10 cm peak steady-state error
cargo run --release --example realistic_noise     # MEMS-class noise + prop wash, ~4 cm RMS, ~1 cm mean
```

`with_fusion_ahrs` shows the full `fusion-ahrs` → `fusion-altitude` pipeline on a synthetic ±5 m, 0.1 Hz vertical oscillation. `realistic_noise` adds representative gyro/accel biases, Gaussian sensor noise, and a periodic prop-wash component on the barometer — see the file header for the sensor budget and the observed error.

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
