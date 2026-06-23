# Analyze ADS-B Data Source

**Status (2026-06-22): design - not yet implemented.** This document specifies the ADS-B DSType for the existing Analyze Data Source task. It is a sibling of [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md), which is the canonical description of the shared task framework (dialog, result report, 3D grid, inspector base classes). The framework sections here summarize only what differs for ADS-B; refer to the MLAT readme for the full framework design. Checkboxes in the [Implementation Plan](#implementation-plan) are unchecked because no ADS-B code exists yet.

## Summary

Analyze Data Source is a standalone COMPASS task for examining a chosen data source from multiple angles. Analyzes are organized by **DSType** and within each DSType by a set of **Inspectors** - focused components that each produce one section of the result report. The MLAT DSType (CAT020 / CAT010) shipped first; this document adds the **ADSB** DSType (**CAT021**).

The ADS-B DSType ships three inspectors, mirroring MLAT Features 1-3:

1. **Data Item Inspection** (free) - cumulative per-CAT data-item tables.
2. **Sensor Coverage / PD** (free) - operational Probability of Detection, **broken down per transponder**.
3. **Position Accuracy** (pro + experimental) - reported vs. observed position accuracy from the ADS-B quality indicators, **broken down per transponder**.

The distinguishing feature of the ADS-B DSType is that **Features 2 and 3 are computed per transponder**. A *transponder* is identified by the 24-bit ICAO aircraft address (ACAD, DB column `aircraft_address`, from I021/080). This is the ADS-B analog of MLAT's per-RU breakdown: where MLAT accuracy and coverage vary by ground receiver (Remote Unit), ADS-B accuracy and coverage vary by the airborne transmitter and its GNSS equipage (MOPS version, NACp / NUCp / NIC quality), so the natural breakdown dimension is the transponder rather than a ground sensor.

Entry point: Task Manager - Analyze ADS-B Data Source (a separate menu entry / task instance from the MLAT one, bound to DSType `"ADSB"`).

**Task-wide prerequisites** (always required, regardless of which inspectors are run):
- Imported CAT021 data.
- Reconstruction has been run, so the **Reference Trajectory (RefTraj) is available** for every loaded target. The Reference Trajectory is the **only** ground truth used by this task; tracker- and other-sensor-based reference selection is not part of it.

Beyond those, inspectors carry only their own additional prerequisites (Professional license for the pro / experimental Position Accuracy inspector). Per-inspector prerequisites are listed in the [DSType / Inspector overview](#dstype--inspector-overview) table below.

## DSType / Inspector overview

(All inspectors share the task-wide prerequisites; the column below lists only the *additional* per-inspector requirements.)

| Inspector | DSType | Tier | Source location | Prerequisites |
|---|---|---|---|---|
| Data Item Inspection (Feature 1) | ADSB | free, always | `src/task/analyze/adsb/` | none |
| Sensor Coverage / PD (Feature 2) | ADSB | free, always | `src/task/analyze/adsb/` | none |
| Position Accuracy (Feature 3) | ADSB | pro + experimental | `experimental_src/analysis/adsb/` | Professional license |

(The pro + experimental inspector requires `USE_EXPERIMENTAL_SRC` at build time to be compiled in; this is the case for the AppImage distribution. With the source compiled in, the pro inspector is always visible in the dialog and only gets greyed out by the license check. Source builds without `USE_EXPERIMENTAL_SRC` (developer-only) do not see the pro inspector at all.)

Unlike MLAT, the ADS-B DSType has no CAT-conditional inspectors: ADS-B is a single category (CAT021), so there is no equivalent of MLAT's "CAT020 only" RU inspectors.

## User Workflow

1. Import CAT021 data.
2. Run reconstruction to produce the Reference Trajectory (always required - the task does not launch otherwise).
3. From the main menu, open Tasks - Analyze ADS-B Data Source. The dialog opens with the DSType pre-bound to ADSB; the user selects one or more CAT021 data sources and ticks the inspectors to run. Inspectors whose prerequisites are not met (no professional license; RefTraj not present) are greyed out with a tooltip giving the reason. Each inspector's settings widget appears on the right when its row is highlighted; the data source selection has its own configuration page on the right too.
4. Click Run. **All selected data sources are folded into one combined dataset**, like in the Evaluation for test sources, before any inspector runs - there is no per-data-source analysis. The per-transponder breakdown of Features 2 and 3 happens *within* that combined dataset, keyed by aircraft address, not per data source.
5. Browse results in the result report.

---

## Configuration Dialog

The configuration dialog is the shared `AnalyzeDataSourceDialog`, opened from the "Analyze ADS-B Data Source" menu entry. The DSType is implicit (ADSB) and is not selected inside the dialog. Layout, tree behavior, and Run validation are identical to the MLAT dialog - see [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md#configuration-dialog). The only difference is the inspector list:

```
+----------------------------------------------------------------------------+
| Analyze ADS-B Data Source                                            [x]  |
+----------------------------------------------------------------------------+
|  Tree                                | Configuration                       |
|                                      |                                     |
|  Data Source                         | (selected node's configuration      |
|     [x] Data Item Inspection         |  widget shown here)                 |
|     [x] Sensor Coverage              |                                     |
|     [x] Position Accuracy   [pro]    |                                     |
|                                      |                                     |
+----------------------------------------------------------------------------+
|                                              [ Cancel ]   [ Run ]          |
+----------------------------------------------------------------------------+
```

The Data Source node reuses `DataSourcesUseWidget` filtered to CAT021 data sources.

---

## Result Report Structure

All selected data sources are folded into one combined dataset before any inspector runs; the report does not split by data source. It does, however, surface per-transponder breakdowns inside Features 2 and 3.

```
Analyze ADS-B Data Source                         << result root
|
+-- Overview
|     Run configuration recap (DSType, selected data sources,
|     ticked inspectors, time range covered, transponder count,
|     MOPS-version distribution, build / license info).
|
+-- Data Item Inspection                          << Feature 1
|     One table per CAT seen in the combined dataset (CAT021):
|       Item   Count   Min   Max   Description
|     Items defined in the active edition but never seen are listed
|     with count 0.
|
+-- Sensor Coverage                               << Feature 2
|     Settings recap (nominal update interval, miss-test parameters,
|       grid resolution, color thresholds).
|     Aggregate summary table: total expected updates, total missed
|       updates, overall Probability of Detection, worst cell.
|     Per-transponder table: aircraft address, callsign, ECAT,
|       MOPS version, expected updates, missed updates, PD
|       (sortable; ascending-PD highlights weak transponders).
|     Figure: Probability of Detection - horizontal projection (aggregate).
|     Figure: Probability of Detection - altitude / longitude projection.
|     Figure: Probability of Detection - altitude / latitude projection.
|     Per-transponder drill-down (on demand): PD heat-map for one
|       selected aircraft address.
|
+-- Position Accuracy                             << Feature 3
      Settings recap (grid resolution, color thresholds,
        risky-ADS-B inclusion, SIL handling).
      Aggregate views (each: horizontal + two altitude projections):
        Reported Position Accuracy
        Horizontal Position Offset
        Reported Accuracy Consistency
      Per-transponder table: aircraft address, callsign, MOPS version,
        dominant QI type/value, reported accuracy, median offset,
        P95 offset, consistency factor, sample count.
      Dubious transponders sub-table: aircraft addresses whose
        consistency factor exceeds the threshold (default 3.0),
        with per-QI factor detail (mirrors the online
        ADSBAccuracyEstimator "ACADs of interest" report).
      Per-transponder drill-down (on demand): dual offset histogram
        and per-cell offset heat-map for one selected aircraft address.
```

The framework owns the result root and the Overview section; each inspector contributes its own sub-tree below the root. Section heading text comes from the inspector's `name()`.

---

## ADS-B inspectors - features

### Feature 1: Data Item Inspection (ADSB, free)

Always calculated. Report-only inspector drawn from the cumulative ASTERIX probing information; produces no grid output. Behaves exactly like the MLAT Feature 1 inspector, scoped to CAT021.

**Source:** the persistent per-(DS, CAT, item) summary held by `DBContextManager::asterixInfo()` (db_info key `"asterix_info"`, see [readme_context.md](../../core/context/readme_context.md)). Refined on every import via `DBContextManager::mergeAsterixInfo(...)`; reflects the cumulative state across all imports into the open DB, not just the most recent file.

**Scope:** restricted to the CAT021 data sources configured for this analysis.

**What is shown:** for each configured data source, one table listing every CAT021 data item defined in the active edition with cumulative `Count`, `Min`, `Max`, `Description`. Items defined but never seen are listed with count 0 (same convention as the per-file ASTERIX Import report). Useful for spotting items that appear only in some recordings (e.g. I021/210 MOPS Version, I021/090 Quality Indicators, I021/140 Geometric Height) or that are present but rarely populated.

**Display:** report-only - no map, no GridView layer.

**Not in scope:** temporal breakdown (per-day / per-hour), cross-source diffs, per-transponder item presence.

---

### Feature 2: Sensor Coverage Analysis / PD (ADSB, free, per transponder)

Always calculated. Computes operational PD across a configurable 3D grid, using RefTraj as ground truth, and keeps **per-transponder** counters so PD can be reported per aircraft address as well as per cell.

**What is shown:**
- *Aggregate spatial PD*: for each 3D grid cell (lat, lon, baro_alt), the probability that the ADS-B system delivered a target report when one was expected, scoped to the time the reference trajectory was inside that cell. Aggregation is sum-of-counters across transponders, identical to MLAT Feature 2.
- *Per-transponder PD*: for each aircraft address, the total expected / missed updates and resulting PD. This is the ADS-B-specific breakdown - it identifies individual transponders with coverage gaps (e.g. low-power or intermittent transmitters), which an aggregate map averages away.

**Cadence source:** time-difference only - a configured nominal Update Interval, identical to the existing COMPASS detection requirement (see [eval/requirement/detection/readme_detection.md](../../eval/requirement/detection/readme_detection.md), §2.2). ADS-B has no Remote Units and no CAT019-style cycle messages, so the MLAT "period-based via CAT019" cadence variant does not apply here.

**Per-transponder grouping:** the PD slot-walk (see [Design](#pd-computation-feature-2)) already runs per target. For ADS-B each reconstructed target corresponds to one transponder, so the per-target #EUI / #MUI totals are exactly the per-transponder totals; the inspector keeps them in a `map<aircraft_address, counters>` for the per-transponder table in addition to cell-attributing them for the grid. Reports with no `aircraft_address` are accumulated under a single "unknown" bucket and noted in the summary.

**Display:**
- Three aggregate projections (horizontal lat/lon, altitude/longitude, altitude/latitude), sector outlines overlaid - same as MLAT.
- Per-transponder summary table (sortable).
- On-demand per-transponder drill-down: a PD heat-map for one selected aircraft address. Materialized only when requested, because one grid layer per transponder is infeasible for the hundreds-to-thousands of distinct addresses in a typical recording (this is the key scaling difference from MLAT's per-RU layers, where RUs number in the tens).

**Color scale (configurable):** transparent (no #EUI) - red (low PD) - green (high PD); thresholds default to ED-117 / ESASSP guidance.

**Report output:**
- Three aggregate projection images.
- Aggregate summary table: total #EUI, total #MUI, overall PD, worst cell.
- Per-transponder table: aircraft address, callsign, ECAT, MOPS version, #EUI, #MUI, PD.

Algorithmic details are in the [Design](#pd-computation-feature-2) section.

---

### Feature 3: Position Accuracy Analysis (ADSB, pro + experimental, per transponder)

The inspector reports ADS-B position accuracy from two complementary perspectives - the accuracy claimed by the transponder's quality indicators and the offset measured against the Reference Trajectory - plus the ratio between them, **grouped per transponder**. This is the offline re-derivation of what `ADSBAccuracyEstimator` ([adsbaccuracyestimator.h](../../../experimental_src/reconstruction/complex/adsbaccuracyestimator.h)) computes online inside the ProbIMM reconstructor (the per-source accuracy estimator ADS-B routes to), exactly as MLAT Feature 3 re-derives `ScaledAccuracyEstimator`. The grouping key matches the estimator's: `(aircraft_address, qi_key)`.

**Reported accuracy from quality indicators.** ADS-B does not carry a Cartesian standard deviation in the target report (unlike MLAT's I020/500). Instead, `tr_std_dev` is derived from the reported quality indicator via the same path the reconstructor uses (`TargetReportAccessor::positionAccuracy()`, see the [adsb_accuracy skill](../../../.claude/skills/adsb_accuracy/SKILL.md)):

- **MOPS v0**: NUCp -> Rc (95% containment), table `adsb_v0_accuracies`.
- **MOPS v1/v2, NACp present** (primary): NACp -> EPU (95% accuracy), table `adsb_v12_accuracies`.
- **MOPS v1/v2, NACp missing** (fallback): NIC -> Rc, table `adsb_v12_nic_accuracies`, using `Rc / 2.0` as approximate EPU.
- All branches then divide the 95% bound by **2.45** to get a 1-sigma stddev.
- **SIL penalty** applied to the resulting stddev: SIL=0 -> x2.0, SIL=1 -> x1.5, SIL>=2 -> none (same as `applySILPenalty()`).

The `qi_key` separates QI types that share a numeric range but mean different accuracies: `nacp` (0-11), `nucp` (0-9, V0), or `nic + 100` (100-111, V1/V2 NIC fallback). Risky ADS-B (missing/zero QI, MOPS v0) is excluded by default and, when included, keyed `qi_key = 0`. Keeping the `qi_key` in the grouping means a transponder that changes equipage state mid-recording is split into separate rows rather than averaged into a meaningless blend.

**Per-target-report quantities collected:**
- `distance_m` - horizontal Cartesian distance between the ADS-B reported position and the interpolated reference position at the same timestamp.
- `tr_std_dev` - reported position accuracy, derived from the QI as above.
- `ref_std_dev` - reference accuracy at that time, used to gate / weight the comparison.

Bucketing for the spatial views: `cell_of(ADS-B reported position)` on `TargetReport3DGrid`. Per cell, the inspector keeps mean / median / P95 of `distance_m`, mean of `tr_std_dev`, and mean of `distance_m / tr_std_dev`. Per transponder (per `(aircraft_address, qi_key)`), it keeps the same statistics plus the consistency factor `median(distance_m) / median(tr_std_dev)` - the same quantity the online rescaler clamps to [0.2, 5.0].

**Three aggregate views rendered every run:**

| View | Cell value | What it tells the operator |
|---|---|---|
| **Reported Position Accuracy** | per-cell mean of the reported `tr_std_dev` | What the transponders in that area claim as their position accuracy, where reports are dense, where the reported value is missing. |
| **Horizontal Position Offset** | per-cell mean (and median + P95) of `distance_m` | The position offset of the ADS-B report against the reference trajectory - the operationally observed accuracy. |
| **Reported Accuracy Consistency** | per-cell mean of `distance_m / tr_std_dev` | Diagnostic view: how well the reported QI-derived accuracy matches the observed offset. Values near 1.0 mean consistent; much greater than 1.0 means the transponders are over-confident; much less than 1.0 means over-cautious. |

**Per-transponder output (the ADS-B-specific breakdown):**
- A per-transponder table keyed by aircraft address (with `qi_key` detail): MOPS version, dominant QI type/value, mean reported accuracy, median / P95 offset, consistency factor, sample count.
- A **dubious transponders** sub-table: aircraft addresses whose consistency factor exceeds `min_factor_of_interest` (default 3.0) - transponders whose reported accuracy is off by 3x or more vs. observed offsets. This is the report-side equivalent of `ADSBAccuracyEstimator::doACADsOfInterest()`, surfaced as analysis output instead of a debug viewpoint.
- On-demand per-transponder drill-down: dual histogram (offset distribution) and a per-cell offset heat-map for one selected aircraft address.

**Display (3 projections per aggregate view):** horizontal (lat/lon), altitude/longitude (lon/alt), altitude/latitude (lat/alt), sector outlines overlaid. Cell aggregation across the dropped axis: weighted average by per-cell sample count.

**Color scales (configurable):**
- Reported Position Accuracy and Horizontal Position Offset: green below 12 m, yellow, orange, red above 60 m.
- Reported Accuracy Consistency: green near 1.0; red diverging in either direction (below 0.5 over-cautious, above 2.0 over-confident).

**Report output:**
- For each aggregate view: three projection images (horizontal plus two altitude cuts).
- Aggregate summary table: overall median offset, P95 offset, mean reported accuracy, mean consistency ratio, worst-area cell location for each view.
- Per-transponder table and dubious-transponders sub-table as above.

---

## ADS-B inspectors - 3D Grid Infrastructure

ADS-B reuses the shared `TargetReport3DGrid` and projection model unchanged - see [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md#mlat-inspectors---3d-grid-infrastructure) for cell dimensions, indexing, projection modes, sector overlay, and GridView display. Two ADS-B-specific notes:

- **Per-transponder layers are not pre-materialized.** MLAT creates one named grid layer per RU (tens of layers). ADS-B has hundreds-to-thousands of transponders, so per-transponder grids are kept as lightweight counter maps and a single grid layer is materialized on demand for the one transponder the user drills into.
- **Altitude axis** is barometric flight level, by default and exclusively: test side I021/145 (DB column `mode_c_code`); reference side the barometric channel of RefTraj. If RefTraj carries no barometric altitude for a target's slot timestamps, that target's contributions fall back to 2D (lat / lon) and the dialog reports how many targets fall back. I021/140 Geometric Height (WGS-84 ellipsoid) is **not** used for the grid axis - its reference frame differs from RefTraj's barometric altitude. (Geometric height may still be reported descriptively in Feature 1.)

---

## ADS-B inspectors - Data Inputs

| Use | CAT021 |
|---|---|
| Time of applicability | I021/071 (Time of Applicability for Position) |
| Position (WGS-84) | I021/130 / I021/131 (high-res) -> `latitude`, `longitude` |
| Barometric altitude | I021/145 (Flight Level) -> `mode_c_code` |
| Transponder identity | I021/080 (Target Address, 24-bit ICAO) -> `aircraft_address` |
| Callsign | I021/170 -> `aircraft_identification` |
| Emitter category | I021/020 -> `emitter_category` |
| MOPS version | I021/210 (VN) -> `mops_version` |
| Quality indicators | I021/090 -> `nacp`, `nucp_or_nic`, `sil`, `sda`, `geometric_altitude_accuracy` |
| Reference trajectory | RefTraj (post-reconstruction) |

I021/140 Geometric Height is not used as the grid altitude axis (see [3D Grid](#ads-b-inspectors---3d-grid-infrastructure)). I021/132 (MAM) and other items are out of scope for these three inspectors.

---

## ADS-B inspectors - Design

Cross-cutting design notes. The PD computation and altitude-axis handling are shared with MLAT; only the per-transponder accumulation and the QI-derived reported accuracy are ADS-B-specific.

### PD computation (Feature 2)

Identical per-target slot-walk to MLAT Feature 2 (reference-period construction, gap construction, miss test, #EUI / #MUI cell attribution) - see [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md#pd-computation-feature-2) for the full algorithm. The only addition is that the per-target #EUI / #MUI totals are also accumulated into a `map<aircraft_address, {eui, mui}>` for the per-transponder table. Per-target PD is therefore identical to what the detection requirement would produce; per-cell PD is a strict spatial refinement; per-transponder PD is the per-target total surfaced by aircraft address.

There is no CAT019 cadence variant for ADS-B.

### Reported accuracy derivation (Feature 3)

The reported `tr_std_dev` is computed from the target report's quality indicators via the same lookup path the reconstructor uses, so the offline analysis and the online estimator agree by construction. The branch order (NACp primary, NIC fallback, NUCp for v0), the `/2.45` conversion, and the SIL penalty are described in the [adsb_accuracy skill](../../../.claude/skills/adsb_accuracy/SKILL.md). Grouping by `(aircraft_address, qi_key)` (with the `nic + 100` offset) prevents conflating NACp and NIC values at the same numeric level and isolates equipage changes within a single transponder.

### Risky ADS-B handling

By default risky ADS-B reports (missing MOPS version, MOPS v0, missing both NACp and NUCp/NIC, or NACp/NUCp/NIC = 0) are excluded from the accuracy statistics, matching `ReconstructorInfo::isRiskyADSB()` and the estimator default `use_risky_adsb_=false`. A settings checkbox can include them (keyed `qi_key = 0`) so the operator can see their observed offsets without a meaningful reported accuracy. Feature 2 PD is unaffected by risk classification - a risky report is still a delivered report.

---

## Architecture

### What Is Reused

Everything the MLAT inspectors reuse (see [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md#what-is-reused)), in particular:

| Component | File | Used for |
|---|---|---|
| `DataSourceInspectorBase` / `InspectorSettingsBase` | `src/task/analyze/` | Inspector base classes + settings sub-configs |
| `TargetReport3DGrid` | `src/task/analyze/targetreport3dgrid.h/.cpp` | 3D grid, #EUI/#MUI + count layers, projections |
| `AnalyzeDataSourceTask` | `src/task/analyze/analyzedatasourcetask.h/.cpp` | Task entry point (one instance per DSType) |
| `AnalyzeDataSourceDialog` | `src/task/analyze/analyzedatasourcedialog.h/.cpp` | Pre-run dialog |
| `DataSourceAnalysisTaskResult` | `src/task/analyze/datasourceanalysistaskresult.h/.cpp` | Result container |
| Detection requirement gap/miss machinery | `src/eval/requirement/detection/detection.cpp` | Reference-period construction, gap walking, miss test |
| `TargetReportAccessor::positionAccuracy()` + `AccuracyTables` | `src/db/dbcontent/target/` | QI -> stddev (NUCp / NACp / NIC, /2.45, SIL penalty) |
| `DBContextManager` | context system | data source resolution; sector definitions; `asterixInfo()` for Feature 1 |
| Reference position interpolation | reconstruction targets | `target.interpolatedRefPosForTime()` |

### New Classes

Inspector components use the `...Inspector` suffix and derive from `DataSourceInspectorBase`. Free inspectors live under `src/task/analyze/adsb/`; the pro + experimental inspector lives under `experimental_src/analysis/adsb/` (compiled in only when `USE_EXPERIMENTAL_SRC` is set).

**ADSB free inspectors (in `src/task/analyze/adsb/`):**

| Class | Planned path | Purpose |
|---|---|---|
| `ADSBDataItemInspector` (+ `ADSBDataItemInspectorSettings`) | `src/task/analyze/adsb/adsbdataiteminspector.h/.cpp` | Feature 1: read per-DS `asterixInfo`, emit CAT021 data-item table |
| `ADSBCoverageInspector` (+ `ADSBCoverageInspectorSettings`) | `src/task/analyze/adsb/adsbcoverageinspector.h/.cpp` | Feature 2: per-target slot-walk on `TargetReport3DGrid` + per-transponder counter map, 3 projections + per-transponder table |

**ADSB pro + experimental inspector (in `experimental_src/analysis/adsb/`):**

| Class | Planned path | Purpose |
|---|---|---|
| `ADSBAccuracyInspector` (+ `ADSBAccuracyInspectorSettings`) | `experimental_src/analysis/adsb/adsbaccuracyinspector.h/.cpp` | Feature 3: QI-derived reported accuracy vs. RefTraj offset, grouped per `(aircraft_address, qi_key)`; three aggregate views + per-transponder and dubious-transponder tables |

### DSType binding (framework change required)

`AnalyzeDataSourceTask` is currently MLAT-specific: `registerParameter("ds_type", &ds_type_, std::string("MLAT"))` and `registerInspectors()` constructs the MLAT inspectors unconditionally ([analyzedatasourcetask.cpp:73](analyzedatasourcetask.cpp#L73), [:216](analyzedatasourcetask.cpp#L216)). Adding ADS-B requires making inspector registration and the per-inspector settings sub-configs **DSType-driven** rather than hardcoded. Two viable shapes:

1. **Branch on `ds_type_`** inside `registerInspectors()` / `generateSubConfigurable()` / `checkSubConfigurables()`: when `"MLAT"`, build the MLAT inspector set (as today); when `"ADSB"`, build the ADS-B set. The TaskManager registers two task instances with different `ds_type` defaults and two menu entries.
2. **A DSType registry** mapping a DSType string to a factory that produces its inspector list and settings classes. Cleaner for adding further DSTypes later, but more refactoring up front.

Either way, the dialog, result container, and 3D grid are unchanged. Recommend shape 1 for this feature (smallest diff, only two DSTypes) and revisiting a registry when a third DSType appears.

---

## Implementation Plan

### Phase 1 - DSType framework support + ADS-B skeleton

- [ ] Make `AnalyzeDataSourceTask` inspector registration DSType-driven (branch on `ds_type_`); keep MLAT behavior unchanged when `ds_type_ == "MLAT"`.
- [ ] Register a second `AnalyzeDataSourceTask` instance bound to `"ADSB"` in `TaskManager`, with its own menu entry "Analyze ADS-B Data Source".
- [ ] Create `src/task/analyze/adsb/` and `experimental_src/analysis/adsb/` with `CMakeLists.txt` files; gate the experimental dir on `USE_EXPERIMENTAL_SRC`.
- [ ] Stub `ADSBDataItemInspector`, `ADSBCoverageInspector`, `ADSBAccuracyInspector` (+ settings) returning empty report sections.
- [ ] Verify: ADS-B task appears in Task Manager; dialog opens with the three inspectors (free enabled, Position Accuracy greyed without a license); run produces an empty report scoped to CAT021 sources.

### Phase 2 - Feature 1: Data Item Inspection

- [ ] Implement `ADSBDataItemInspector::run()`: per configured CAT021 data source, read its per-CAT item summary from `DBContextManager::asterixInfo()`, emit one table (Item, Count, Min, Max, Description; count-0 rows for unseen items).
- [ ] Verify table matches the ASTERIX Import report when a single CAT021 file is the only import.

### Phase 3 - Feature 2: Sensor Coverage / PD (per transponder)

- [ ] Implement `ADSBCoverageInspector::run()` on `TargetReport3DGrid`: per target, build reference periods and walk them in UI steps (reuse detection.cpp machinery), attribute #EUI / #MUI to `cell_of(RefTraj at t_slot)`, and accumulate per-target totals into a `map<aircraft_address, {eui, mui}>`.
- [ ] Emit three aggregate projections + aggregate summary table + per-transponder table.
- [ ] On-demand per-transponder PD heat-map (single materialized layer for the selected address).
- [ ] Verify per-target totals match a plain detection-requirement run; verify per-transponder PD sums back to the aggregate.

### Phase 4 - Feature 3: Position Accuracy (per transponder, pro + experimental)

- [ ] Implement `ADSBAccuracyInspector::run()` in `experimental_src/analysis/adsb/`:
  - Per associated CAT021 target report: compute `distance_m` to the interpolated reference position; derive `tr_std_dev` from the QI via `TargetReportAccessor::positionAccuracy()` (NACp primary / NIC fallback / NUCp v0, /2.45, SIL penalty); compute `qi_key` (`nacp`, `nucp`, or `nic + 100`).
  - Bucket per cell on `TargetReport3DGrid` and per `(aircraft_address, qi_key)`; keep mean / median / P95 of `distance_m`, mean of `tr_std_dev`, mean of `distance_m / tr_std_dev`, and per-transponder consistency factor.
  - Apply risky-ADS-B exclusion by default; settings checkbox to include (qi_key = 0).
  - Render three aggregate views x three projections; emit per-transponder table + dubious-transponders sub-table (factor > 3.0).
- [ ] On-demand per-transponder drill-down (offset histogram + per-cell offset heat-map).
- [ ] Verify folded overall median / P95 offset match a baseline scripted computation; verify dubious-transponder list matches the online estimator's "ACADs of interest" on the same data.

### Phase 5 - Polish

- [ ] Configurable color scales for PD, accuracy, and consistency layers.
- [ ] CSV export for per-transponder coverage and accuracy tables.
- [ ] Progress reporting per phase.
- [ ] Performance profiling on large datasets (many transponders, >1M TRs); confirm the per-transponder maps and on-demand drill-down stay within budget.

### Deferred / out of scope (this release)

- [ ] Velocity accuracy (NACv / NUCr) analysis - this feature set covers position only.
- [ ] Geometric-height accuracy (GVA) analysis and a geometric-vs-barometric altitude comparison.
- [ ] Per-ground-station breakdown (would require a receiver identifier ADS-B target reports do not generally carry).
</content>
</invoke>
