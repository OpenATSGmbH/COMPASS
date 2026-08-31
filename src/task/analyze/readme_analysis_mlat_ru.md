# Analyze Data Source

**Status (2026-05-19): Phases 1-7 shipped; Phase 8 shipped in a revised form (single `MLATRUEffectInspector`); Phase B/C deferred.** This document has been reconciled against the actual implementation in `src/task/analyze/` and `experimental_src/analysis/`; the original design is preserved where it still matches the code, and divergences are called out inline.

## Summary

Analyze Data Source is a standalone COMPASS task for examining a chosen data source from multiple angles. Analyzes are organized by **DSType** (currently only **MLAT** = CAT020 / CAT010) and within each DSType by a set of **Inspectors** - focused components that each produce one section of the result report.

Two inspectors are **always present** (free, in the main source tree); the remaining ones live in `experimental_src/analysis/` and require the **Professional license** to run. Pro / experimental inspectors are **always visible in the dialog** when their source is compiled in (`USE_EXPERIMENTAL_SRC` set, which is the case for the AppImage distribution); without a Professional license their row is greyed out with the licensing reason in a tooltip, so users can see at a glance which inspectors a license unlocks. Source builds without `USE_EXPERIMENTAL_SRC` (developer-only) do not see the pro inspectors at all.

Entry point: Task Manager - Analyze Data Source

**Task-wide prerequisites** (always required, regardless of which inspectors are run):
- Imported data of the bound DSType.
- Reconstruction has been run, so the **Reference Trajectory (RefTraj) is available** for every loaded target. The Reference Trajectory is the **only** ground truth used by this task; tracker- and ADS-B-based reference selection is not part of it.

Beyond those, inspectors carry only their own additional prerequisites (Professional license for the pro / experimental ones; at least one CAT020 source selected for CAT020-only inspectors). Per-inspector prerequisites are listed in the [DSType / Inspector overview](#dstype--inspector-overview) table below.

## DSType / Inspector overview

| Inspector | DSType | Tier | Source location | Prerequisites |
|---|---|---|---|---|
(All inspectors share the task-wide prerequisites; the column below lists only the *additional* per-inspector requirements.)

| Data Item Inspection (Feature 1) | MLAT | free, always | `src/task/analyze/mlat/` | none |
| Sensor Coverage / PD (Feature 2) | MLAT | free, always | `src/task/analyze/mlat/` | none |
| Position Accuracy (Feature 3) | MLAT | pro + experimental | `experimental_src/analysis/mlat/` | Professional license |
| RU Coverage (Feature 4) | MLAT | pro + experimental | `experimental_src/analysis/mlat/` | Professional license + at least one CAT020 source selected |
| RU Offset / Combination (Feature 5) | MLAT | pro + experimental | `experimental_src/analysis/mlat/` | Professional license + at least one CAT020 source selected |

(Pro + experimental inspectors require `USE_EXPERIMENTAL_SRC` at build time to be compiled in; this is the case for the AppImage distribution. With the source compiled in, pro inspectors are always visible in the dialog and only get greyed out by the license check.)

## User Workflow

1. Import data of a supported DSType (currently CAT020 / CAT010 for MLAT).
2. Run reconstruction to produce the Reference Trajectory (always required - the task does not launch otherwise).
3. From the main menu, open the task entry for the DSType to be analyzed (for Main Menu -> Analyze -> MLAT). The dialog opens with the DSType pre-bound; the user selects one or more data sources of that DSType and ticks the inspectors to run. Inspectors whose prerequisites are not met (no professional license; CAT020-only inspector with no CAT020 source selected; etc.) are greyed out with a tooltip giving the reason. Each inspector's settings widget appears on the right when its row is highlighted; the data source selection has its own configuration page on the right too.
4. Click Run. **All selected data sources are folded into one combined dataset**, like in the Evaluation for test sources, before any inspector runs - there is no per-data-source analysis. Each ticked inspector consumes the combined dataset and contributes one section to the result report.
5. Browse results in the result report.

---

## Configuration Dialog

The configuration dialog is opened from the main menu entry corresponding to the DSType ("Tasks - Analyze MLAT Data Source" for the MLAT entry). The DSType is therefore implicit and is not selected inside the dialog.

The dialog is a two-pane layout: a tree on the left holds the Data Source node and one node per registered inspector; the right pane is a stacked widget showing the configuration view for the currently highlighted node. A Run button and a Cancel button sit at the bottom. The pattern follows `ReconstructorTaskDialog` plus the per-reconstructor widget tree (`SimpleReconstructorWidget`, [src/task/reconstructor/simplereconstructorwidget.cpp:49-103](src/task/reconstructor/simplereconstructorwidget.cpp#L49-L103)).

```
+----------------------------------------------------------------------------+
| Analyze MLAT Data Source                                              [x]  |
+----------------------------------------------------------------------------+
|  Tree                                | Configuration                       |
|                                      |                                     |
|  Data Source                         | (selected node's configuration      |
|     [x] Data Item Inspection         |  widget shown here; e.g. for the    |
|     [x] Sensor Coverage              |  Data Source node, the data source  |
|     [x] Position Accuracy   [pro]    |  selection widget; for an inspector |
|     [x] RU Coverage         [pro]    |  node, that inspector's settings    |
|     [ ] RU Offset           [pro]    |  widget drawn from its              |
|     [ ] RU Combination      [pro]    |  InspectorSettingsBase sub-config)  |
|                                      |                                     |
+----------------------------------------------------------------------------+
|                                              [ Cancel ]   [ Run ]          |
+----------------------------------------------------------------------------+
```

Tree behavior:
- The Data Source node is always present and always visible. Selecting it shows the data source selection widget on the right (reuses the existing `DataSourcesUseWidget` filtered to data sources of the bound DSType, with per-line selection consistent with the reconstructor task).
- Each inspector appears as a child node of the Data Source node. The first column shows an active checkbox per inspector; the second column shows the inspector name; an optional badge such as `[pro]` marks pro / experimental inspectors. Selecting an inspector node shows that inspector's configuration widget on the right (drawn from its `InspectorSettingsBase`-derived sub-config).
- Inspectors whose `prerequisitesMet(...)` returns false are greyed out with the failure reason as the row tooltip. Failure reasons include: missing professional license (for pro inspectors), Reference Trajectory not present (for inspectors that need it), no data source selected of the required CAT (for example RU Coverage requires CAT020).

Run validation: clicking Run is rejected if no data source is selected, if no inspector is ticked, or if any ticked inspector has unmet prerequisites. The dialog reports the offending row and reason.

**Scope filter** (Data Source page, shared by all DSTypes): restricts which target reports enter the combined dataset, so every grid inspector (PD, accuracy, RU) sees only the in-scope reports. Three independent, AND-combined constraints:

- **Use Ground Only** - keep only on-ground reports (ground bit set, or reconstructed target type is ground-only).
- **Minimum / Maximum Flight Level** - keep reports whose barometric altitude (FL = feet / 100) is within the band; the FL check is skipped when altitude is absent (as `Sector::isInside` does).
- **Limit by Sectors** - keep only reports inside the selected sector layers. The sector list mirrors the reconstruction dialog's widget (a checkbox per sector layer); when the box is ticked with no layer selected the Run button is disabled. A report is in scope if it is inside **any** selected layer (union).

It is applied once when the dataset is built (`AnalysisDataset`), as a per-report position + ground-bit inside test modeled on the evaluation's `EvaluationTargetData::computeSectorInsideInfo`. The sector test is the evaluation's exact call - `SectorLayer::isInside(pos, has_gb, gb_set)` with `has_gb = ground_bit.has_value()`, `gb_set = ground_bit.value_or(false)` - so sector altitude bands and ground handling behave identically to the evaluation. Both the reference (RefTraj) and the test reports are filtered, so reference periods and observed reports stay consistent. Because the filter needs the ground bit (and position) on every report, the dataset load uses an explicit read set (like `EvaluationManager`) that includes them; the "Ground Bit" meta-variable resolves to `ground_bit` for CAT010/020/021 and to `surface_target` for RefTraj/CAT062. The sector constraint above is the *scope* use of sectors (limit the dataset to selected areas). Independently, each grid inspector also emits a **per-sector results breakdown**: a "PD by Sector" (coverage) / "Accuracy by Sector" (accuracy) overview table with one row per selected sector plus an "All (in scope)" reference row. The breakdown ranges over the selected sector layers whether or not the Limit toggle is on (Limit restricts the dataset; the table splits whatever is in it). A report is attributed to every sector it lies in (same `SectorLayer::isInside` test), so - because sectors can overlap - the per-sector rows need not sum to the "All" row. Columns are the headline metrics only (coverage: targets, #EUI, #MUI, PD; accuracy: samples, targets, median/P95 offset, median reported, median consistency, no-accuracy count); the per-transponder / per-QI tables are not multiplied per sector.

---

## Result Report Structure

All selected data sources are folded into one combined dataset before any inspector runs; the report does not split by data source.

```
Analyze MLAT Data Source                          << result root
|
+-- Overview
|     Run configuration recap (DSType, selected data sources,
|     ticked inspectors, time range covered, build / license info).
|
+-- Data Item Inspection                          << Feature 1
|     One table per CAT seen in the combined dataset:
|       Item   Count   Min   Max   Description
|     Items defined in the active edition but never seen are listed
|     with count 0.
|
+-- Sensor Coverage                               << Feature 2
|     Settings recap (cadence source, nominal update interval,
|       miss-test parameters, grid resolution, color thresholds).
|     Summary table: total expected updates, total missed updates,
|       overall Probability of Detection, worst cell.
|     Figure: Probability of Detection - horizontal projection.
|     Figure: Probability of Detection - altitude / longitude projection.
|     Figure: Probability of Detection - altitude / latitude projection.
|
+-- Position Accuracy                             << Feature 3
      Settings recap (grid resolution, color thresholds,
        included reported-accuracy sources for CAT020).
      Reported Position Accuracy
        Summary table.
        Figure: horizontal projection.
        Figure: altitude / longitude projection.
        Figure: altitude / latitude projection.
        (For CAT020 with both legacy I020/500 and REF Position Accuracy:
         the above repeated once per source.)
      Horizontal Position Offset
        Summary table.
        Figure: horizontal projection.
        Figure: altitude / longitude projection.
        Figure: altitude / latitude projection.
      Reported Accuracy Consistency
        Summary table.
        Figure: horizontal projection.
        Figure: altitude / longitude projection.
        Figure: altitude / latitude projection.
        (For CAT020 with both reported sources: the above repeated once
         per source, plus a comparison sub-table with the median ratio
         per source.)
```

The framework owns the result root and the Overview section; each inspector contributes its own sub-tree below the root. Section heading text comes from the inspector's `name()`; the framework does not prefix the heading.

---

## MLAT inspectors - features

The following five Feature sections describe the MLAT-DSType inspectors. As more DSTypes are added, each will get its own parallel "Features" section. The infrastructure ([3D Grid](#mlat-inspectors---3d-grid-infrastructure), [Data Inputs](#mlat-inspectors---data-inputs), [Design](#mlat-inspectors---design)) sections below are MLAT-scope.

### Feature 1: Data Item Inspection (MLAT, free)

Always calculated. Report-only inspector drawn from the cumulative ASTERIX probing information; produces no grid output.

**Source:** the persistent per-(DS, CAT, item) summary held by `DBContextManager::asterixInfo()` (db_info key `"asterix_info"`, see [readme_context.md](../../../core/context/readme_context.md)). That store is refined on every import via `DBContextManager::mergeAsterixInfo(...)` and reflects the cumulative state across all imports into the currently open DB, not just the most recent file.

**Scope:** restricted to the data sources configured for this analysis - the CAT020 / CAT010 MLAT sources, plus the associated CAT019 source when the period-based PD cadence (Feature 2) is enabled.

**What is shown:** for each configured data source, one table per CAT in scope listing every data item defined in the active edition with cumulative `Count`, `Min`, `Max`, `Description`. Items defined but never seen are listed with count 0 (same convention as the per-file ASTERIX Import report).

**Difference from the ASTERIX Import report:** that report renders per-file probe results; this section renders the cumulative aggregate across all imports into the open DB, scoped to the data sources in this analysis. Useful for spotting items that appear only in some recordings, or that are present but rarely populated.

**Display:** report-only - no map, no GridView layer.

**Report output:**
- One sub-section per configured data source under "Data Item Analysis"
- One table per CAT (`Item`, `Count`, `Min`, `Max`, `Description`)
- Header row per (DS, CAT): total record count

**Not in scope:** temporal breakdown (per-day / per-hour), cross-source diffs, item-appearance flags ("seen by source X but not by source Y").

---

### Feature 2: Sensor Coverage Analysis / PD (MLAT, free)

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
- Summary table: total #EUI, total #MUI, overall PD, worst cell location/PD

Algorithmic details are in the [Design](#design) section.

---

### Feature 3: Position Accuracy Analysis (MLAT, pro + experimental)

The inspector reports MLAT position accuracy from two complementary perspectives - the accuracy claimed by the sensor and the offset measured against the Reference Trajectory - plus the ratio between them. The data model mirrors what `ScaledAccuracyEstimator` ([scaledaccuracyestimator.h:16-41](experimental_src/reconstruction/complex/scaledaccuracyestimator.h#L16-L41)) collects inside the ProbIMM reconstructor (the per-source accuracy estimator MLAT routes to, [complexaccuracyestimator.cpp:376-377](experimental_src/reconstruction/complex/complexaccuracyestimator.cpp#L376-L377)); the inspector re-derives the same quantities offline from stored target reports plus the stored Reference Trajectory.

**Per-target-report quantities collected** (matches `ScaledAccuracyEstimator::AccuracyInfo`):
- `distance_m` - horizontal Cartesian distance between the MLAT reported position and the interpolated reference position at the same timestamp.
- `tr_std_dev` - the reported position accuracy from the target report itself: x and y standard deviations from I020/500, from REF Position Accuracy (if present), or from I010/500 for CAT010. Reported as Cartesian magnitude `sqrt(std_dev_x^2 + std_dev_y^2)` for the per-cell heat-map; per-component values listed in the summary table only. For CAT020 with both legacy I020/500 and REF Position Accuracy present, both are tracked as separate reported quantities so the inspector can show whether they agree.
- `ref_std_dev` - the reference accuracy at that time (Cartesian magnitude of the RefTraj x/y standard deviations; the worse of the two bracketing reference updates), used to gate the comparison.

**Reference accuracy gate** (`ref_gate_factor`, default 2.0, 0 disables): the offset is only measured where the reference is sufficiently more accurate than the reported accuracy being assessed - `ref_gate_factor * ref_std_dev < tr_std_dev`. A sample failing the gate (including a reference update without accuracy) keeps contributing its `tr_std_dev` to the Reported Position Accuracy view, but produces no offset and no consistency ratio anywhere (aggregate, per-cell, per-sector), and is counted in the summary as "Test reports w/o accurate reference". Samples without a usable `tr_std_dev` are not gated (offset-only, as before) - there is no claim to gate against.

Bucketing: `cell_of(MLAT target report position)` on `TargetReport3DGrid`. Per cell, the inspector keeps mean / median / P95 of `distance_m`, mean of `tr_std_dev`, and mean of `distance_m / tr_std_dev`.

**Three views rendered every run:**

| View | Cell value | What it tells the operator |
|---|---|---|
| **Reported Position Accuracy** | per-cell mean of the reported `tr_std_dev` | What the sensor claims as its position accuracy, where reports are dense, where the reported value is missing. |
| **Horizontal Position Offset** | per-cell mean (and median + P95) of `distance_m` | The position offset of the MLAT report against the reference trajectory - the operationally observed accuracy. |
| **Reported Accuracy Consistency** | per-cell mean of `distance_m / tr_std_dev` | Diagnostic view: how well the reported accuracy matches the observed offset. Same metric the online rescaler in `ScaledAccuracyEstimator` uses internally. Values near 1.0 mean the reported accuracy is consistent; values much greater than 1.0 indicate the sensor is over-confident; values much less than 1.0 indicate the sensor is over-cautious. |

For CAT020 with both legacy I020/500 and REF Position Accuracy reported, the Reported Position Accuracy and Reported Accuracy Consistency views are rendered once per source so the user can see at a glance which source is consistent with the observed offsets.

**Display (3 projections per view):**
- Horizontal: lat / lon
- Altitude / Longitude: lon / alt
- Altitude / Latitude: lat / alt
- Sector outlines overlaid on all projections.

Cell aggregation across the dropped axis: weighted average by per-cell sample count.

**Color scales (configurable):**
- Reported Position Accuracy and Horizontal Position Offset: green below 12 m, yellow, orange, red above 60 m.
- Reported Accuracy Consistency: green near 1.0; red diverging in either direction (below 0.5 over-cautious, above 2.0 over-confident).

**Report output:**
- For each view: three projection images (horizontal plus two altitude cuts).
- Summary table: overall median offset, P95 offset, mean reported accuracy, mean reported-accuracy-consistency ratio, worst-area cell location for each view.
- For CAT020: a small reported-source comparison sub-table (legacy I020/500 vs. REF Position Accuracy) with median consistency ratio per source.

---

### Feature 4: RU Coverage Analysis (MLAT, CAT020 only, pro + experimental)

Optional. Built on the same 3D grid as Feature 2, broken down by **contributing RU (I020/400)**.

**Availability:** CAT020 only. CAT010 has no Contributing Receivers item, so this feature is hidden when the configured data source is CAT010.

No reference trajectory required - this is a contribution-count analysis, not PD per RU.

**4a. Coverage Map per RU**

For each RU listed in the data source's RemoteUnitDefinition: a 2D lat/lon heatmap showing how often that RU contributed to MLAT positions in each cell. Color scale: transparent (0) - red (few) - green (many). User selects which RU to display from the GridView layer panel or a dropdown.

Report output:
- One coverage map image per RU
- Table: RU name, total contribution count, estimated coverage area, max cell count

**4b. Dominant RU Map**

A single horizontal map where each cell is colored by the RU that contributed most often there. Optional opacity encodes dominance ratio. Clicking a cell shows: dominant RU + count, runner-up RU + count.

Variant: "contested areas" map - cells where the top-2 RUs are within 10% of each other.

Report output:
- Single dominant-RU map image
- Table: RU name, number of dominant cells, percentage of total coverage area

---

### Feature 5: RU Effect Analysis (MLAT, CAT020 only, pro + experimental)

Implemented as a single inspector `MLATRUEffectInspector` (the originally planned Feature 5a / 5b split was merged during implementation). Built on the same 3D grid as Feature 3, broken down by contributing RU.

**Shipped sub-analyses:**

1. **Marginal effect table** - per RU, the error statistics of all reports **with** that RU contributing vs. **without** it. Surfaces RUs whose presence systematically improves (or worsens) accuracy.
2. **Redundancy curve** - error statistics aggregated by **number of contributing RUs**. Tells the operator how much accuracy is gained by going from N to N+1 contributors, and when adding further contributors stops paying off.
3. **Per-RU drill-down** - for any RU selected from the table, a dual-histogram view (with vs. without that RU) plus a per-cell error heat-map of the reports where the RU contributed.

Report output:
- Marginal effect table (RU name, count with, count without, mean / median error with, mean / median error without, delta, p-value).
- Redundancy curve image (error vs. number of contributors).
- Per-RU drill-down images on demand.

**Deferred to a later release (Phase B/C, tracked in `mlatrueffectinspector.h`):**
- Exact-subset RU grouping with top/bottom combinations table.
- RU-pair matrix (cell (i,j) = average error when RUs i and j both contributed).
- Minimum-RU-count analysis ("reports with >= N RUs have avg error X" for N = 2, 3, 4, 5).

CAT020 only - uses I020/400 contributing receivers.

---

## MLAT inspectors - 3D Grid Infrastructure

All grid-based analyzes share a common 3D grid populated once on load.

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

For PD layers the aggregation across the dropped axis is sum-of-#EUI / sum-of-#MUI; for count layers (Feature 4) it is sum-of-counts; for error layers (Feature 3, when implemented) the aggregation rule will be specified with the Feature 3 design.

**Sector overlay:** sectors defined in the active data context are drawn as outlines on top of all three projections. Display only - no filtering or grouping by sector.

**Visualization:** all projections are displayed via the existing GridView. Horizontal projections also appear as overlays in the Geographic View.

---

## MLAT inspectors - Data Inputs

| Use | CAT020 | CAT010 |
|---|---|---|
| Time of applicability | I020/140 | I010/140 |
| Position (WGS-84) | I020/041 | I010/041 |
| Barometric altitude | I020/090 (Mode-S binary FL) | I010/090 (FL) |
| Contributing receivers (Feature 4, Feature 5) | I020/400 | **not available** |
| System status / cadence (period-based PD) | CAT019 (associated MLAT) | CAT019 / CAT023 if associated |
| Reference trajectory | RefTraj (post-reconstruction) | RefTraj |

I020/110 and I010/091 (locally-referenced measured heights) are **not used** - their reference frames differ from RefTraj's altitude and would require an MLT-system-point or airport-frame transform that is out of scope here.

---

## MLAT inspectors - Design

This section covers cross-cutting design notes for the PD computation (Feature 2). Per-feature design specifics live in each Feature's section above.

### PD computation (Feature 2)

Feature 2 reuses the existing COMPASS detection requirement's per-target machinery, then "cuts up" the per-target #EUI / #MUI integers into 3D cells so that summing cell counters reproduces the per-target totals. Per-target PD is therefore identical to what the detection requirement would produce on the same data; per-cell PD is a strict refinement of it.

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
- **Standing targets:** the same motion-adaptive logic as the time-difference method applies. A CAT019 cycle is system-wide; a standing target legitimately produces a position less often than the cycle rate (it emits fewer replies). So when the reference/test ground speed at a cycle is below `standing_speed_max_mps_`, that target is expected only once per `update_interval_standing_s_` rather than every cycle: the cycle opens an expected opportunity whose window spans the standing interval, and the cycles inside it are skipped (neither #EUI nor #MUI). The number of skipped cycles follows from the cycle period (e.g. a 1 s cycle and a 5 s standing interval excuse ~4 of every 5 cycles). Moving targets keep the per-cycle expectation. Implemented in `evaluateCyclesInPeriod` (see `mlatcoveragehelpers.h`), shared movement classifier in [movementui.h](movementui.h).
- Fallback: if CAT019 is missing or partial, the inspector falls back to time-difference and logs a warning. Without this, #EUI is silently under-counted in the affected windows and PD is biased upward (see [readme_detection.md](../../../eval/requirement/detection/readme_detection.md), §2.4).

### Altitude axis

The grid altitude axis is **barometric flight level**, by default and exclusively:

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
| `TaskResult` | `src/task/result/taskresult.h` | Base for `DataSourceAnalysisTaskResult` |
| Detection requirement gap/miss machinery | `src/eval/requirement/detection/detection.cpp` | Reference-period construction, gap walking, miss test |
| Contributing receivers column | `var_cat020_contrib_recv_` | I020/400 - **CAT020 only** |
| `RemoteUnitDefinition` / `DataSourceRemoteUnit` | `src/core/source/datasourceremoteunit.h` | RU index → name, lat, lon, alt |
| `DBContextManager` | context system | data source resolution; RU resolution; sector definitions; `asterixInfo()` for Feature 1 |
| Reference position interpolation | reconstruction targets | `target.interpolatedRefPosForTime()` |
| `JobManager` | `src/core/job/jobmanager.h` | Async background job execution |

### New Classes

Inspector components use the **`...Inspector`** suffix - distinct from the existing `AccuracyEstimator` family in `src/task/reconstructor/`. All inspectors derive from `DataSourceInspectorBase`. Free / always-present source lives under `src/task/analyze/`; pro + experimental source lives under `experimental_src/analysis/` (compiled in only when `USE_EXPERIMENTAL_SRC` is set). When the source is compiled in, pro inspectors are **always registered into the dispatcher** so they appear in the dialog regardless of license; the license check happens at the dialog level and only controls whether the row is enabled or greyed out.

**Task / framework (all free, in `src/task/analyze/`):**

| Class | Planned path | Purpose |
|---|---|---|
| `DataSourceInspectorBase` | `src/task/analyze/datasourceinspectorbase.h/.cpp` | Abstract base for all inspectors. Pure virtuals: `dsType()`, `name()`, `prerequisitesMet(...)`, `run(...)`, `generateReportSections(...)`. Holds a reference to its `InspectorSettingsBase`-derived settings sub-config. |
| `InspectorSettingsBase` | `src/task/analyze/inspectorsettingsbase.h/.cpp` | Abstract base `Configurable` for per-inspector settings. Each concrete inspector defines a `<Name>InspectorSettings : InspectorSettingsBase` sub-config; the task owns these as Configurable sub-configs and looks them up by inspector name. |
| `TargetReport3DGrid` | `src/task/analyze/targetreport3dgrid.h/.cpp` | Reusable 3D grid for any target-report-based inspector (any DSType). #EUI/#MUI counters per cell + count-only counters; produces 2D projections. Used by `MLATCoverageInspector` and the pro MLAT inspectors; intended to be reused by future DSType inspectors. |
| `AnalyzeDataSourceTask` | `src/task/analyze/analyzedatasourcetask.h/.cpp` | Task entry point - bound to a DSType at construction (one Task subclass / instance per DSType menu entry). Owns settings, the registered inspector list, and the `InspectorSettingsBase` sub-configs. Triggers dialog, runs the analysis inline (no separate Job class), holds result. Settings are handled inline in this class (no separate `AnalyzeDataSourceSettings` class). |
| `AnalyzeDataSourceDialog` | `src/task/analyze/analyzedatasourcedialog.h/.cpp` | Pre-run dialog. Tree on the left listing the Data Source node and one node per registered inspector (active checkbox per inspector); right pane is a `QStackedWidget` of configuration widgets, one per tree node. Inspectors whose `prerequisitesMet(...)` returns false (missing license, missing RefTraj, DSType / CAT mismatch with selected sources) are greyed out with a tooltip giving the reason. |
| `DataSourceAnalysisTaskResult` | `src/task/analyze/datasourceanalysistaskresult.h/.cpp` | Result container; holds an Overview section plus one section per executed inspector. (Renamed during implementation from the originally planned `AnalyzeDataSourceResult`.) |

**MLAT free inspectors (in `src/task/analyze/mlat/`):**

| Class | Planned path | Purpose |
|---|---|---|
| `MLATDataItemInspector` (+ `MLATDataItemInspectorSettings`) | `src/task/analyze/mlat/mlatdataiteminspector.h/.cpp` | Feature 1: read per-DS `asterixInfo`, emit per-CAT data-item tables |
| `MLATCoverageInspector` (+ `MLATCoverageInspectorSettings`) | `src/task/analyze/mlat/mlatcoverageinspector.h/.cpp` | Feature 2: per-target slot-walk on `TargetReport3DGrid`, cell attribution, 3 projections |

**MLAT pro + experimental inspectors (in `experimental_src/analysis/mlat/`):**

| Class | Planned path | Purpose |
|---|---|---|
| `MLATRUCoverageInspector` (+ `...Settings`) | `experimental_src/analysis/mlat/mlatrucoverageinspector.h/.cpp` | Feature 4: per-RU contribution layers + dominant-RU layer (CAT020 only) |
| `MLATAccuracyInspector` (+ `...Settings`) | `experimental_src/analysis/mlat/mlataccuracyinspector.h/.cpp` | Feature 3: three per-cell views on `TargetReport3DGrid` (Reported Position Accuracy, Horizontal Position Offset, Reported Accuracy Consistency) |
| `MLATRUEffectInspector` (+ `...Settings`) | `experimental_src/analysis/mlat/mlatrueffectinspector.h/.cpp` | Feature 5: per-RU marginal effect table, redundancy curve, per-RU drill-down (CAT020 only). Replaces the planned `MLATRUOffsetInspector` + `MLATCombinationInspector` split; exact-subset combination grouping and RU-pair matrix deferred to Phase B/C |

### Data Flow

Exactly **one** inspector runs per task invocation (selected in the dialog). The diagram below shows all available MLAT inspectors and their inputs - the alternatives the user can pick from, not a parallel pipeline.

```
DBContextManager::asterixInfo()
   (cumulative per-(DS, CAT, item) summary,
    refined on every import)
       |
       v
       MLATDataItemInspector  [Feature 1, both cats]
          per configured DS: emit per-CAT item table to report
          (no map, no grid)

CAT020 / CAT010 TRs                RefTraj from reconstruction
   (time + WGS-84 pos +              (per-target reference positions
    baro alt + I020/400               + baro alt)
    on CAT020)
       \                             /
        \                           /
         v                         v
         MLATCoverageInspector  [Feature 2, both cats]
            per target: walk reference periods in UI steps
                        cell-cut #EUI / #MUI by ref pos at slot t
            project to 3x Grid2DLayer (PD per cell)

         MLATRUCoverageInspector  [Feature 4, CAT020 only]
            per TR: increment cell.count for each contributing RU index
            project to per-RU horizontal layers + dominant-RU layer

         MLATAccuracyInspector      [Feature 3]
         MLATRUEffectInspector      [Feature 5, CAT020 only - replaces the planned 5a + 5b split]
```

---

## Implementation Plan

### Phase 1 - Framework Skeleton

Goal: runnable (but empty) task visible in Task Manager, with a working inspector dispatch.

- [ ] Create `src/task/analyze/` and `src/task/analyze/mlat/` with `CMakeLists.txt` files
- [ ] Create `experimental_src/analysis/` and `experimental_src/analysis/mlat/` with `CMakeLists.txt` files; gate inclusion on `USE_EXPERIMENTAL_SRC`
- [ ] Implement `InspectorSettingsBase` - abstract `Configurable` base for per-inspector settings sub-configs.
- [ ] Implement `DataSourceInspectorBase` - abstract base, pure virtuals (`dsType()`, `name()`, `prerequisitesMet(...)`, `run(...)`, `generateReportSections(...)`); holds a reference to its `InspectorSettingsBase`-derived settings sub-config.
- [ ] Implement `AnalyzeDataSourceTask` - inherits `Task` + `Configurable`, registers with `TaskManager`; one task instance per DSType (separate menu entry per DSType). Owns the inspector registry and the per-inspector `InspectorSettingsBase` sub-configs. Top-level settings (`selected_data_source_ids` (multi), per-inspector enabled flag, shared cross-inspector params such as `TargetReport3DGrid` resolution / bounds) are handled inline on this class - there is no separate `AnalyzeDataSourceSettings` class. Always registers the free MLAT inspectors. Pro / experimental inspectors are also registered whenever `USE_EXPERIMENTAL_SRC` is compiled in (regardless of license); the license check is consulted by the dialog to decide whether to gray the row, not by the registry to decide whether to add it. The task runs the analysis inline in `run()` (no separate `AnalyzeDataSourceJob` class): it folds all selected data sources into one combined dataset, then iterates over the ticked inspectors and calls `run()` on each.
- [ ] Implement `AnalyzeDataSourceDialog` - tree on the left (Data Source node, then one node per registered inspector, each with active checkbox), `QStackedWidget` configuration pane on the right showing the configuration widget of the currently highlighted tree node. Greyed inspector rows show their failure reason in a tooltip.
- [ ] Implement `DataSourceAnalysisTaskResult` skeleton - empty report with the Overview section + one placeholder section per ticked inspector. (Renamed from `AnalyzeDataSourceResult` during implementation.)
- [ ] Add `include("${CMAKE_CURRENT_LIST_DIR}/analyze/CMakeLists.txt")` to `src/task/CMakeLists.txt`
- [ ] Register task in `TaskManager` constructor
- [ ] Verify: task appears in UI; dialog opens with all five inspectors visible (free ones enabled, pro ones greyed out without a license, enabled with one); job runs and produces an empty report.

### Phase 2 - Feature 1: Data Item Analysis

Goal: report-only inspector populated from `DBContextManager::asterixInfo()`. No grid output.

- [ ] Implement `MLATDataItemInspector::run()`:
  - For each configured data source, read its per-CAT item summary from `DBContextManager::asterixInfo()`
  - Restrict to MLAT-relevant CATs (CAT020 / CAT010 and, when CAT019 cadence is enabled, the associated CAT019)
  - Emit one sub-section per data source, one table per CAT (Item, Count, Min, Max, Description; count-0 rows for unseen items)
- [ ] Implement Data Item Analysis report sections in `DataSourceAnalysisTaskResult`
- [ ] Verify: report appears with cumulative item counts; tables match what the ASTERIX Import report shows when a single file is the only import

### Phase 3 - 3D Grid Infrastructure

Goal: the shared data structure that all inspectors build on.

- [ ] Implement `TargetReport3DGrid`:
  - Constructor takes horizontal cell size (m) and vertical cell size (ft) + geo bounds
  - `addCount(lat, lon, baro_alt_ft, layer_id)` - bucket a TR into its cell on a named layer
  - `addEUI(lat, lon, baro_alt_ft)` / `addMUI(lat, lon, baro_alt_ft)` - PD counters
  - `projectHorizontal(layer)`, `projectAltLon(layer)`, `projectAltLat(layer)` → `Eigen::MatrixXd`
  - PD-layer aggregation: sum-of-EUI / sum-of-MUI across the dropped axis, then `(Σ EUI - Σ MUI) / Σ EUI`
  - Sector overlay: `addSectorLayer(sector)` returns sector outline as `Grid2DLayer`
- [ ] Unit test: bucket known TRs, verify projection counts and PD aggregation identity

### Phase 4 - Feature 2: PD Analysis

- [ ] Implement `MLATCoverageInspector::run()`:
  - Per target, build reference periods using detection.cpp's parameters and rules
  - Walk each period in UI steps, attribute #EUI to `cell_of(RefTraj at t_slot)`
  - For each gap that passes the miss test, attribute `floor(adj_gap/UI)` #MUI by walking the gap in UI steps
  - Add sector outline layers
- [ ] Implement PD report sections in `DataSourceAnalysisTaskResult`
- [ ] Optional: CAT019 cadence source (otherwise leave behind a TODO + UI option disabled)
- [ ] Verify per-target totals match a plain detection requirement run on the same data

### Phase 5 - Feature 4: RU Coverage Analysis (CAT020)

- [ ] Implement `MLATRUCoverageInspector::run()` (skipped if data source is CAT010):
  - Per TR, parse I020/400, increment per-RU count layer at `cell_of(TR pos)`
  - Build dominant-RU horizontal layer + optional contested-areas variant
- [ ] Implement RU coverage report sections
- [ ] Verify per-RU coverage maps appear in report; verify CAT010 sources skip Feature 4 cleanly

### Phase 6 - Polish (covers Feature 1 + Feature 2 + Feature 4)

- [ ] Configurable color scales for PD and count layers
- [ ] CSV export for cell counters and per-RU summary tables
- [ ] Progress reporting per phase
- [ ] Performance profiling on large datasets (>1M TRs)

### Phase 7 - Feature 3: Position Accuracy Analysis

- [ ] Implement `MLATAccuracyInspector::run()` (in `experimental_src/analysis/mlat/`):
  - Per associated MLAT target report: compute `distance_m` to the interpolated reference position; read `tr_std_dev` from I020/500 (x/y std dev), REF Position Accuracy (x/y std dev), or I010/500 (x/y std dev); track legacy I020/500 and REF Position Accuracy as separate quantities for CAT020.
  - Bucket per cell on `TargetReport3DGrid`; per cell keep mean / median / P95 of `distance_m`, mean of `tr_std_dev`, mean of `distance_m / tr_std_dev`.
  - Render three views (Reported Position Accuracy, Horizontal Position Offset, Reported Accuracy Consistency) each as three projections; for CAT020 with both reported sources, render Reported Position Accuracy and Reported Accuracy Consistency once per source.
- [ ] Implement Position Accuracy report sections in `DataSourceAnalysisTaskResult`.
- [ ] Verify the folded-across-data-sources overall median offset / P95 offset match a baseline scripted computation on the same data.

### Phase 8 - Feature 5: RU Effect Analysis (as shipped)

CAT020 only - uses I020/400 contributing receivers. Implemented as a single inspector `MLATRUEffectInspector` (the originally planned `MLATRUOffsetInspector` + `MLATCombinationInspector` split was merged during implementation).

Shipped:

- [x] Implement `MLATRUEffectInspector::run()` in `experimental_src/analysis/mlat/`:
  - Per CAT020 TR: parse I020/400; for each contributing RU, attribute the TR's `distance_m` to the RU's per-cell error layer.
  - Marginal effect table: for each RU, compute error statistics of reports with vs. without that RU.
  - Redundancy curve: error statistics aggregated by the number of contributing RUs.
  - Per-RU drill-down: dual-histogram (with vs. without) + per-cell error heatmap for a user-selected RU.
- [x] RU Effect report sections in `DataSourceAnalysisTaskResult`.

### Phase B/C - Deferred RU combination work

Tracked separately in `mlatrueffectinspector.h:78`. **Not in the current release.**

- [ ] Exact-subset RU error grouping (count, mean/median, stddev, P5/P25/P75/P95 per subset).
- [ ] Top/bottom combinations ranking by effect size vs. global baseline.
- [ ] RU-pair matrix (cell `(i, j)` = mean error when RUs `i` and `j` both contributed).
- [ ] Minimum-RU-count analysis ("reports with >= N RUs have avg error X").
