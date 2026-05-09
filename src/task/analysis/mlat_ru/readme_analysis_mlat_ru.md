# MLAT Remote Unit Analysis

## Summary

MLAT RU Analysis is a standalone COMPASS task that analyzes the **Probability of Detection (PD)** and (in a later iteration) the **position accuracy** of MLAT data sources, with a per-Remote-Unit breakdown for CAT020. It accepts CAT020 and CAT010 MLAT data sources, runs as a background job, and produces an Analysis Report with PD heat-maps, optional accuracy heat-maps, and CAT020 RU contribution maps.

Entry point: Task Manager - MLAT RU Analysis

Prerequisites:
- Professional license present
- CAT020 or CAT010 MLAT data imported
- Reconstruction run (reference trajectories required)

Reference data: **only the Reference Trajectory (RefTraj) is used as ground truth.** Tracker- and ADS-B-based reference selection is not part of this task.

Iteration scope:
- **Feature A** (PD per cell, both categories) - implemented in this iteration.
- **Feature 1** (per-RU coverage, CAT020 only) - implemented in this iteration.
- **Feature B** (per-cell position accuracy) - listed below; **design TBD, to be implemented later.**
- **Feature 2** (per-RU offset and RU combination analysis, CAT020 only) - listed below; **design TBD, to be implemented later.**

---

## User Workflow

1. Import CAT020 or CAT010 MLAT data
2. Run reconstruction (provides reference trajectories)
3. Open MLAT RU Analysis task - configure via dialog and click Run
4. Browse results in the Analysis Report

---

## Features

### Feature A: Sensor Coverage Analysis (PD) - this iteration

Always calculated. Computes per-cell PD across a configurable 3D grid, using RefTraj as ground truth.

**What is shown:** for each 3D grid cell (lat, lon, baro_alt), the operational PD - the probability that the MLAT system delivered a target report when one was expected, scoped to the time the reference trajectory was inside that cell. Aggregation is sum-of-counters across targets, not a mean of per-target PDs.

**Cadence sources** (selectable in the dialog):
- *Time-difference* (default) - configured nominal Update Interval, identical to the existing COMPASS detection requirement (see [eval/requirement/detection/readme_detection.md](../../../eval/requirement/detection/readme_detection.md), §2.2).
- *Period-based via CAT019* - update intervals derived from MLAT system-status messages. Falls back to time-difference if CAT019 is missing or partial.

**Display (3 projections):**
- Horizontal 2D: PD heat-map over lat/lon (counters aggregated over altitude bins)
- Altitude/Longitude: PD over lon/alt (counters aggregated over latitude)
- Altitude/Latitude: PD over lat/alt (counters aggregated over longitude)
- Sector outlines overlaid on all projections

**Color scale (configurable):** transparent (no #EUI) - red (low PD) - green (high PD); thresholds default to ED-117 / ESASSP guidance.

**Report output:**
- Three projection images
- Summary table: total #EUI, total #MUI, sector-aggregate PD, worst cell location/PD

Algorithmic details are in the [Design](#design) section.

---

### Feature B: Position Accuracy Analysis - to be implemented later, design TBD

The horizontal position error for each TR is the Cartesian distance between the MLAT reported position and the interpolated RefTraj position at the same timestamp.

**What is analyzed:**
- For each MLAT TR associated with a UTN: compute horizontal error in meters
- Bucket (TR, error) into the 3D grid
- Cell value: mean (or median) horizontal error of all TRs in that cell

**Display (3 projections):**
- Horizontal 2D: mean error heat-map over lat/lon
- Altitude/Longitude: mean error over lon/alt
- Altitude/Latitude: mean error over lat/alt
- Sector outlines overlaid on all projections

**Ideas/TBD:**
- Color scale: green (<12m) - yellow - orange - red (>60m); configurable thresholds
- Alternative cell values: median error, P95 error, stddev, sample count
- Worst-cell highlighting
- "Honesty of reported σ" map using I020/500 / REF PA (CAT020) and I010/500 (CAT010)
- Summary statistics: overall mean error, P95 error, worst cell location + value

**Report output:**
- Horizontal error map image
- Altitude/longitude and altitude/latitude error projection images
- Table: overall mean error, P95 error, worst-area cell

> **Status:** design pending - to be specified once Feature A is in place. Algorithm detail will follow the same "per-target compute, cell-cut" structure as Feature A.

---

### Feature 1: RU Coverage Analysis (CAT020 only) - this iteration

Optional. Built on the same 3D grid as Feature A, broken down by **contributing RU (I020/400)**.

**Availability:** CAT020 only. CAT010 has no Contributing Receivers item, so this feature is hidden when the configured data source is CAT010.

No reference trajectory required - this is a contribution-count analysis, not PD per RU.

**1a. Coverage Map per RU**

For each RU listed in the data source's RemoteUnitDefinition: a 2D lat/lon heatmap showing how often that RU contributed to MLAT positions in each cell. Color scale: transparent (0) - red (few) - green (many). User selects which RU to display from the GridView layer panel or a dropdown.

Report output:
- One coverage map image per RU
- Table: RU name, total contribution count, estimated coverage area, max cell count

**1b. Dominant RU Map**

A single horizontal map where each cell is colored by the RU that contributed most often there. Optional opacity encodes dominance ratio. Clicking a cell shows: dominant RU + count, runner-up RU + count.

Variant: "contested areas" map - cells where the top-2 RUs are within 10% of each other.

Report output:
- Single dominant-RU map image
- Table: RU name, number of dominant cells, percentage of total coverage area

---

### Feature 2: RU Offset and Combination Analysis (CAT020 only) - to be implemented later, design TBD

Built on the same 3D grid as Feature B, broken down by contributing RU.

**2a. Average Position Error Heat-Map per RU**

For each RU: a 2D heatmap (horizontal projection) where each cell value is the mean (or median) position error of all TRs in that cell where that RU contributed.

Useful derived views:
- Overall error map: all RUs combined - baseline reference
- Delta map: error difference between two selected RUs

Report output:
- Per-RU error heatmap images
- Overall system error map image
- Table: RU name, overall mean error, 95th percentile error, worst-area cell location

**2b. RU Combination Analysis**

Answers: which specific sets of contributing RUs produce the best and worst position accuracy?

Algorithm sketch:
1. Group TRs by contributing RU subset (exact match or fixed-size subsets - pairs, triples - to avoid combinatorial explosion with 20+ RUs)
2. Compute statistics per group: count, mean/median error, stddev, P5/P25/P75/P95
3. Discard groups below minimum sample count (default: 50)
4. Rank groups by effect size vs. global baseline (median delta or Cohen's d)
5. Report top-N best (lowest error) and worst (highest error) combinations
6. RU pair matrix: cell (i,j) = average error when both RU i and RU j contributed - reveals geometric weaknesses (high GDOP pairs)

Report output:
- Top/bottom 10 combinations table (RU set, count, mean error, median, P95)
- RU pair matrix image (green = low error pairs, red = high error pairs)
- Minimum RU count analysis: "reports with >=N RUs have avg error X" for N = 2, 3, 4, 5

> **Status:** design pending - depends on Feature B. Available only for CAT020 (uses I020/400).

---

## 3D Grid Infrastructure

All grid-based analyses share a common 3D grid populated once on load.

**Cell dimensions (configurable):**
- Horizontal: single cell size in meters (square cells at reference latitude)
- Vertical: cell size in feet (**barometric flight level** by default - see [Design / Altitude axis](#altitude-axis))

**Cell indexing:** (lat_bin, lon_bin, alt_bin)

Geographic bounds are auto-derived from the data extent by default, or user-specified.

**Projection modes** - any 3D grid can be projected to 2D for display:

| Projection | Axes | How |
|---|---|---|
| Horizontal | lat / lon | aggregated over alt_bins |
| Altitude/Longitude | lon / alt | aggregated over lat_bins |
| Altitude/Latitude | lat / alt | aggregated over lon_bins |

For PD layers the aggregation across the dropped axis is sum-of-#EUI / sum-of-#MUI; for count layers (Feature 1) it is sum-of-counts; for error layers (Feature B, when implemented) the aggregation rule will be specified with the Feature B design.

**Sector overlay:** sectors defined in the active data context are drawn as outlines on top of all three projections. Display only - no filtering or grouping by sector.

**Visualization:** all projections are displayed via the existing GridView. Horizontal projections also appear as overlays in the Geographic View.

---

## Data Inputs

| Use | CAT020 | CAT010 |
|---|---|---|
| Time of applicability | I020/140 | I010/140 |
| Position (WGS-84) | I020/041 | I010/041 |
| Barometric altitude | I020/090 (Mode-S binary FL) | I010/090 (FL) |
| Contributing receivers (Feature 1, Feature 2) | I020/400 | **not available** |
| System status / cadence (period-based PD) | CAT019 (associated MLAT) | CAT019 / CAT023 if associated |
| Reference trajectory | RefTraj (post-reconstruction) | RefTraj |

I020/110 and I010/091 (locally-referenced measured heights) are **not used** - their reference frames differ from RefTraj's altitude and would require an MLT-system-point or airport-frame transform that is out of scope here.

---

## Design

This section covers Feature A only. Design for Feature B and Feature 2 will be added in a later iteration.

### PD computation (Feature A)

Feature A reuses the existing COMPASS detection requirement's per-target machinery, then "cuts up" the per-target #EUI / #MUI integers into 3D cells so that summing cell counters reproduces the per-target totals. Per-target PD is therefore identical to what the detection requirement would produce on the same data; per-cell PD is a strict refinement of it.

**Per-target slot-walk** (counters live on cells; reused from `detection.cpp`):

```
For each reference period [t_begin, t_end] of the target:
    n_slots = floor((t_end - t_begin) / UI)
    For k = 0 .. n_slots - 1:
        t_slot = t_begin + k * UI
        cell  = cell_of( RefTraj.interpolatedPos(target, t_slot) )
        cell.eui += 1                                     # one expected slot

For each gap (period start, between consecutive in-period test reports,
period end - same gap construction as detection.cpp):
    if gap passes miss test (miss_tolerance, min_gap, max_gap):
        adj_gap  = gap - miss_tolerance        (if enabled)
        n_misses = floor(adj_gap / UI)
        For m = 0 .. n_misses - 1:
            t_miss = gap_start + (m + 1) * UI
            cell   = cell_of( RefTraj.interpolatedPos(target, t_miss) )
            cell.mui += 1                                 # one missed slot
```

**Per-cell PD, aggregated across all targets:**

```
PD_cell = ( Σ_targets cell.eui − Σ_targets cell.mui ) / Σ_targets cell.eui
```

**Identity:** for each target, `Σ_cells cell.eui` equals the sum-over-its-periods of `floor(period_duration / UI)`, and `Σ_cells cell.mui` equals the target's #MUI from a plain detection-requirement run on the same inputs. The detection requirement uses `floor(total_period_duration / UI)` for #EUI in §3.3; the per-period floor used here can differ by at most (n_periods - 1) slots per target. The per-period floor is the more meaningful quantization for cell attribution, since a slot cannot span a period boundary.

**Reused machinery from [detection.cpp](../../../eval/requirement/detection/detection.cpp):**
- Reference-period construction (max-ref-time-diff, min-period-length, sector / extent inside test)
- Gap construction (period start to first in-period test, between consecutive in-period tests, last in-period test to period end)
- Miss test parameters: `Miss Tolerance`, `Use Minimum Gap Length`, `Use Maximum Gap Length`
- `target.interpolatedRefPosForTime()` for the per-slot reference position lookup

### CAT019 period-based variant

When the user selects "CAT019 cycle" as cadence source:

- Slot timestamps come from CAT019 start-of-cycle / system-status messages instead of `t_begin + k * UI`.
- Each CAT019-defined cycle inside a reference period contributes one #EUI to the cell of the reference position at the cycle timestamp; the cycle is a #MUI for that cell if no test report falls in it.
- Fallback: if CAT019 is missing or partial, the analyzer falls back to time-difference and logs a warning. Without this, #EUI is silently under-counted in the affected windows and PD is biased upward (see [readme_detection.md](../../../eval/requirement/detection/readme_detection.md), §2.4).

### Altitude axis

The grid altitude axis is **barometric flight level**, by default and exclusively in this iteration:

- Test side: I020/090 (CAT020) or I010/090 (CAT010) - both barometric, both routinely populated.
- Reference side: barometric channel of RefTraj. If RefTraj does not carry a barometric altitude for the target's slot timestamps, the altitude axis is disabled for that target's contributions and the analysis runs in 2D (lat / lon) for those targets - the alt/lon and alt/lat projections then reflect only the targets with usable baro reference altitude. The dialog reports how many targets fall back to 2D-only.

I020/110 (local Cartesian) and I010/091 (local airport frame) are deliberately not used: their reference frames would require an MLT-system-point or airport-2D transform to align with RefTraj altitude, and they are often absent from recordings.

---

## Architecture

### What Is Reused

| Component | File | Used for |
|---|---|---|
| `AccuracyGridBase<AccInfo, CellInfo>` | `experimental_src/reconstruction/complex/accuracygridestimator.h` | 2D grid foundation (spatial R-tree, `updateGrid()`) |
| `Grid2DLayers` + named layers | `src/view/gridview/grid2dlayer.h` | One named layer per RU or projection |
| `GridView` | `src/view/gridview/gridview.h` | Renders all 2D projections |
| `Grid2DLayerRenderer` | `src/view/gridview/grid2dlayerrenderer.h` | `render()` produces `QImage` from a layer |
| `ResultReport::Section` | `src/task/result/report/section.h` | Report tree |
| `TaskResult` | `src/task/result/taskresult.h` | Base for `MLATRUAnalysisResult` |
| Detection requirement gap/miss machinery | `src/eval/requirement/detection/detection.cpp` | Reference-period construction, gap walking, miss test |
| Contributing receivers column | `var_cat020_contrib_recv_` | I020/400 - **CAT020 only** |
| `RemoteUnitDefinition` / `DataSourceRemoteUnit` | `src/core/source/datasourceremoteunit.h` | RU index → name, lat, lon, alt |
| `DBContextManager` | context system | RU resolution; sector definitions |
| Reference position interpolation | reconstruction targets | `target.interpolatedRefPosForTime()` |
| `JobManager` | `src/core/job/jobmanager.h` | Async background job execution |

### New Classes

| Class | File | Purpose |
|---|---|---|
| `MLAT3DGrid` | `mlat3dgrid.h/.cpp` | Core 3D grid; #EUI/#MUI counters per cell + count-only counters; produces 2D projections |
| `MLATRUAnalysisSettings` | `mlatruanalysissettings.h` | Config: data sources, grid resolution, bounds, cadence source, miss-test parameters, enabled features |
| `MLATRUAnalysisTask` | `mlatruanalysistask.h/.cpp` | Task entry point - owns settings, triggers dialog, launches job, holds result |
| `MLATRUAnalysisDialog` | `mlatruanalysisdialog.h/.cpp` | Pre-run configuration dialog |
| `MLATRUAnalysisJob` | `mlatruanalysisjob.h/.cpp` | Background job - drives analyzers, assembles result |
| `MLATRUAnalysisResult` | `mlatruanalysisresult.h/.cpp` | Result container + report section generator |
| `MLATPDAnalyzer` | `mlatpdanalyzer.h/.cpp` | Feature A: per-target slot-walk, cell attribution, 3 projections |
| `MLATRUCoverageAnalyzer` | `mlatrucoverageanalyzer.h/.cpp` | Feature 1: per-RU contribution layers + dominant-RU layer (CAT020 only) |
| `MLATAccuracyAnalyzer` *(later)* | `mlataccuracyanalyzer.h/.cpp` | Feature B - design TBD |
| `MLATRUOffsetAnalyzer` *(later)* | `mlatruoffsetanalyzer.h/.cpp` | Feature 2a - design TBD |
| `MLATCombinationAnalyzer` *(later)* | `mlatcombinationanalyzer.h/.cpp` | Feature 2b - design TBD |

### Data Flow

```
CAT020 / CAT010 TRs                RefTraj from reconstruction
   (time + WGS-84 pos +              (per-target reference positions
    baro alt + I020/400               + baro alt)
    on CAT020)
       \                             /
        \                           /
         v                         v
         MLATPDAnalyzer  [Feature A, both cats]
            per target: walk reference periods in UI steps
                        cell-cut #EUI / #MUI by ref pos at slot t
            project to 3x Grid2DLayer (PD per cell)

         MLATRUCoverageAnalyzer  [Feature 1, CAT020 only]
            per TR: increment cell.count for each contributing RU index
            project to per-RU horizontal layers + dominant-RU layer

         MLATAccuracyAnalyzer      [Feature B, later]
         MLATRUOffsetAnalyzer      [Feature 2a, later]
         MLATCombinationAnalyzer   [Feature 2b, later]
```

---

## Implementation Plan

### Phase 1 - Framework Skeleton

Goal: runnable (but empty) task visible in Task Manager.

- [ ] Create `src/task/analysis/` and `src/task/analysis/mlat_ru/` with `CMakeLists.txt` files
- [ ] Implement `MLATRUAnalysisSettings` - all config params with defaults, Configurable JSON persistence
- [ ] Implement `MLATRUAnalysisTask` - inherits `Task` + `Configurable`, registers with `TaskManager`
- [ ] Implement `MLATRUAnalysisDialog` - shows settings, OK/Cancel
- [ ] Implement `MLATRUAnalysisJob` skeleton - loads CAT020/CAT010 TRs + RU definitions, logs counts
- [ ] Implement `MLATRUAnalysisResult` skeleton - empty report with placeholder sections
- [ ] Add `include("${CMAKE_CURRENT_LIST_DIR}/analysis/CMakeLists.txt")` to `src/task/CMakeLists.txt`
- [ ] Register task in `TaskManager` constructor
- [ ] Verify: task appears in UI, dialog opens, job runs and produces an empty report

### Phase 2 - 3D Grid Infrastructure

Goal: the shared data structure that all analyzers build on.

- [ ] Implement `MLAT3DGrid`:
  - Constructor takes horizontal cell size (m) and vertical cell size (ft) + geo bounds
  - `addCount(lat, lon, baro_alt_ft, layer_id)` - bucket a TR into its cell on a named layer
  - `addEUI(lat, lon, baro_alt_ft)` / `addMUI(lat, lon, baro_alt_ft)` - PD counters
  - `projectHorizontal(layer)`, `projectAltLon(layer)`, `projectAltLat(layer)` → `Eigen::MatrixXd`
  - PD-layer aggregation: sum-of-EUI / sum-of-MUI across the dropped axis, then `(Σ EUI - Σ MUI) / Σ EUI`
  - Sector overlay: `addSectorLayer(sector)` returns sector outline as `Grid2DLayer`
- [ ] Unit test: bucket known TRs, verify projection counts and PD aggregation identity

### Phase 3 - Feature A: PD Analysis

- [ ] Implement `MLATPDAnalyzer::run()`:
  - Per target, build reference periods using detection.cpp's parameters and rules
  - Walk each period in UI steps, attribute #EUI to `cell_of(RefTraj at t_slot)`
  - For each gap that passes the miss test, attribute `floor(adj_gap/UI)` #MUI by walking the gap in UI steps
  - Add sector outline layers
- [ ] Implement PD report sections in `MLATRUAnalysisResult`
- [ ] Optional: CAT019 cadence source (otherwise leave behind a TODO + UI option disabled)
- [ ] Verify per-target totals match a plain detection requirement run on the same data

### Phase 4 - Feature 1: RU Coverage Analysis (CAT020)

- [ ] Implement `MLATRUCoverageAnalyzer::run()` (skipped if data source is CAT010):
  - Per TR, parse I020/400, increment per-RU count layer at `cell_of(TR pos)`
  - Build dominant-RU horizontal layer + optional contested-areas variant
- [ ] Implement RU coverage report sections
- [ ] Verify per-RU coverage maps appear in report; verify CAT010 sources skip Feature 1 cleanly

### Phase 5 - Polish (covers Feature A + Feature 1)

- [ ] Configurable color scales for PD and count layers
- [ ] CSV export for cell counters and per-RU summary tables
- [ ] Progress reporting per phase
- [ ] Performance profiling on large datasets (>1M TRs)

### Phase 6 - Feature B: Position Accuracy Analysis (later)

Design TBD. To be specified once Phases 1-5 are in place. Expected to follow the same "per-target compute, cell-cut" structure as Feature A.

### Phase 7 - Feature 2: RU Offset and Combination Analysis (later)

Design TBD. Depends on Feature B. CAT020 only (uses I020/400).
