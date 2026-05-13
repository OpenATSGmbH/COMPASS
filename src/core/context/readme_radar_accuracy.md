# Radar Accuracy Model

This document describes the radar error model used in COMPASS for position accuracy estimation, bias correction, and error ellipse rendering. The model follows standard ATC surveillance conventions as used by EUROCONTROL.

## Radar measurement model

A radar measures target position in polar coordinates relative to its antenna:

- **Azimuth** θ [deg] - horizontal bearing from north
- **Slant range** ρ [m] - distance from antenna to target

These measurements are subject to two types of errors:

1. **Random errors** (noise) - zero-mean Gaussian, characterized by standard deviations
2. **Systematic errors** (biases) - constant offsets that can be estimated and corrected

## Parameters stored per data source

### Random errors (`radar_accuracy` in `info_`)

Per detection channel (PSR, SSR, Mode S):

| Parameter | Key | Unit | Typical value | Description |
|-----------|-----|------|---------------|-------------|
| Azimuth stddev | `primary_azimuth_stddev` | deg | 0.05 | PSR azimuth measurement noise |
| Range stddev | `primary_range_stddev` | m | 120 | PSR range measurement noise |
| Azimuth stddev | `secondary_azimuth_stddev` | deg | 0.025 | SSR azimuth measurement noise |
| Range stddev | `secondary_range_stddev` | m | 70 | SSR range measurement noise |
| Azimuth stddev | `mode_s_azimuth_stddev` | deg | 0.02 | Mode S azimuth measurement noise |
| Range stddev | `mode_s_range_stddev` | m | 50 | Mode S range measurement noise |

Azimuth stddev is angular - it naturally produces larger position errors at greater range. Range stddev is a fixed distance - radar range accuracy depends on pulse width, not distance.

Global defaults (used when no per-source values are configured) are in `SensorConfig` in `DBContextManager`.

### Systematic errors (`radar_bias` in `info_`)

| Parameter | Key | Unit | Default | Description |
|-----------|-----|------|---------|-------------|
| Range bias | `range_bias` | m | 0 | Additive range offset |
| Range bias stddev | `range_bias_stddev` | m | 0 | Uncertainty of range bias estimate |
| Range gain | `range_gain` | unitless | 0 | Multiplicative range scale error |
| Range gain stddev | `range_gain_stddev` | unitless | 0 | Uncertainty of range gain estimate |
| Azimuth bias | `azimuth_bias` | deg | 0 | Additive azimuth offset |
| Azimuth bias stddev | `azimuth_bias_stddev` | deg | 0 | Uncertainty of azimuth bias estimate |

## Bias correction

Given measured azimuth θ_meas and ground range r_meas, the corrected values are:

```
r_corrected = (r_meas - range_bias) / (1 + range_gain)
θ_corrected = θ_meas - azimuth_bias
```

This is applied in `RS2GCoordinateSystem::radarSlant2LocalCart()` before converting to Cartesian coordinates. The corrected polar coordinates are then projected to WGS-84 via:

```
x_local = r_corrected * sin(θ_corrected)
y_local = r_corrected * cos(θ_corrected)
```

## Bias estimation

### Azimuth bias estimation

Computed in `Number::estimateAzimuthBias()`. For each target report with a known reference position:

1. Compute the azimuth from radar to the reference position (θ_ref) and from radar to the measured position (θ_meas).
2. Compute the angular difference: `diff = θ_ref - θ_meas` (wrapped to [-180°, 180°]).
3. Reject differences exceeding `max_diff_deg` (default 30°).
4. The azimuth bias is the **median** of all angular differences.
5. Reject if the median exceeds `max_bias_deg` (default 30°).

### Range bias and gain estimation

Computed in `Number::estimateRangeBiasGain()`. For each target report:

1. Compute the ground range from radar to the reference position (r_ref) and from radar to the measured position (r_meas).
2. Filter out pairs where `r_meas / r_ref` deviates more than `max_range_ratio_diff` (default 0.1 = 10%) from 1.0.
3. Fit a linear regression via QR decomposition: `r_ref = a * r_meas + b`.
4. Extract bias and gain: `gain = 1/a - 1`, `bias = -b/a`.
5. Reject if `|gain| > max_gain` (default 0.05) or `|bias| > max_bias_m` (default 2000 m).

### Blending across slices

Both bias estimates use exponential smoothing across processing slices to prevent feedback loops (bias correction shifts positions, which changes references, which changes the measured bias):

```
bias_new = 0.5 * bias_raw + 0.5 * bias_previous
```

First slice uses the raw estimate directly.

### Safety check

After bias correction, the median position error (corrected vs reference) is compared to the original (uncorrected) error. If the corrected error is worse by more than 20% relative and more than 5 m absolute, all biases are reset to zero.

## Error ellipse rendering

The geographic view renders error ellipses for each radar target report using the per-channel azimuth and range standard deviations.

### Polar-to-Cartesian covariance conversion

Given azimuth stddev σ_θ [deg] and range stddev σ_r [m] for the active detection channel:

1. Convert azimuth stddev to meters at the target's range: `σ_θ_m = σ_θ * 2π * d / 360`, where d is the distance from radar to target.

2. Form the polar covariance matrix:
```
C_polar = | σ_r²      0      |
          |  0      σ_θ_m²   |
```

3. Rotate to the local Cartesian frame using the bearing angle β from radar to target:
```
A = | sin(β)   cos(β)  |
    | cos(β)  -sin(β)  |

C_cart = A * C_polar * A^T
```

4. Extract Cartesian stddevs: `σ_x = √C_cart(0,0)`, `σ_y = √C_cart(1,1)`, `cov_xy = C_cart(0,1)`.

### Ellipse from covariance

The Cartesian covariance is decomposed via SVD to obtain ellipse parameters:

```
C = | σ_x²    cov_xy |
    | cov_xy  σ_y²   |

U, S = SVD(C)

semi_major = √S₀
semi_minor = √S₁
rotation   = atan2(U₁₀, U₀₀)
```

The ellipse is sampled at regular angular intervals and projected to geodetic coordinates for rendering.

### Detection type handling

For combined detections (e.g. SSR+PSR, Mode S+PSR), the minimum stddev of the two channels is used (configurable to maximum).

## EUROCONTROL quality requirements

For reference, EUROCONTROL specifies these quality thresholds for en-route radar surveillance:

| Parameter | Requirement |
|-----------|-------------|
| Range bias | < 100 m |
| Azimuth bias | < 0.1° |
| Range gain | < 1 m/NM (~0.00054) |
| Range stddev | < 70 m |
| Azimuth stddev | < 0.08° |
| Timestamp error | < 100 ms |

Positional errors exceeding 1° in azimuth or 700 m in range are classified as "jumps" (required ratio < 0.05%).

## Usage in reconstruction

Both reconstructors load per-data-source radar parameters from the context system at initialization and apply them during processing.

### Bias correction (both reconstructors)

Both `SimpleAccuracyEstimator` and `RadarAccuracyEstimator` (ProbIMM) load `radar_bias` from each radar data source via `DataSource::radarBiasInfo()` and apply bias correction through the shared `radar_bias::correctPosition()` utility. The correction converts measured azimuth/range to Cartesian, applies the bias formula (see "Bias correction" above), and reprojects to WGS-84.

### Position accuracy from data source parameters

When a target report has no reported position accuracy and the data source is a radar with configured per-channel stddevs (`radar_accuracy`), both reconstructors compute accuracy using the polar-to-Cartesian covariance model (see "Polar-to-Cartesian covariance conversion" above):

- **SimpleAccuracyEstimator**: Loads per-channel accuracies (PSR/SSR/Mode S azimuth + range stddevs) at init. In `positionAccuracy()`, reads the TR's radar range and azimuth, bias-corrects them if available, selects the best channel for the detection type, and converts to Cartesian stddevs. Falls back to unspecific accuracy if no channel info or no range/azimuth available.

- **RadarAccuracyEstimator** (ProbIMM): Uses the same polar model as tier 2 (when no accuracy grid cell exists). Additionally estimates and blends per-channel accuracies from observed errors across slices, and provides a spatial accuracy grid (tier 1) when rescaling is enabled.

For combined detection types (SSR+PSR, Mode S+PSR), both select the channel with the smaller azimuth stddev.

## Key source files

| File | Purpose |
|------|---------|
| `src/core/context/data_source.h/.cpp` | `radar_accuracy` and `radar_bias` storage in `info_` JSON |
| `src/core/source/datasourcebase.h/.cpp` | Key constants and accessors (`PSRAzmSDKey`, `RangeBiasKey`, etc.) |
| `src/core/projection/radarbiasinfo.h` | `RadarBiasInfo` struct used during projection |
| `src/core/projection/rs2g/rs2gcoordinatesystem.cpp` | Bias correction in `radarSlant2LocalCart()` |
| `src/core/util/number.h/.cpp` | `estimateAzimuthBias()`, `estimateRangeBiasGain()` |
| `src/task/reconstructor/simpleaccuracyestimator.h/.cpp` | Simple reconstructor: loads per-source biases + accuracies, polar-to-Cartesian accuracy model |
| `src/task/reconstructor/radarbiascorrection.h/.cpp` | Shared bias correction utility used by both reconstructors |
| `experimental_src/reconstruction/complex/radaraccuracyestimator.h/.cpp` | ProbIMM: per-radar accuracy estimation, bias estimation, and correction |
| `experimental_src/view/geographicview/geometry/geometryitemgrouppositionaccuracyellipses.cpp` | Error ellipse rendering |

## References

- [EUROCONTROL Surveillance Standard Document for Radar Surveillance in En-Route Airspace and Major Terminal Areas](https://www.eurocontrol.int/sites/default/files/publication/files/surveillance-standard-document-for-radar-surveillance-in-en-route-airspace-and-major-terminal-areas199703.pdf)
- [Complete Systematic Error Model of SSR for Sensor Registration in ATC Surveillance Networks (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC5677430/)
- [Radartutorial - Accuracy of Measurement](https://www.radartutorial.eu/01.basics/Radars%20Accuracy.en.html)
- [EUROCONTROL ASTERIX CAT048 Specification](https://www.eurocontrol.int/sites/default/files/2019-05/cat048pt4ed122.pdf)
