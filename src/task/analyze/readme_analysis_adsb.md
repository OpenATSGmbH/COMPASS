# Analyze ADS-B Data Source

**Status (2026-06-23): implemented; all three inspectors shipped.** This document has been reconciled against the actual implementation in `src/task/analyze/adsb/` and `experimental_src/analysis/adsb/`. It is a sibling of [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md), which is the canonical description of the shared task framework (dialog, result report, 3D grid, inspector base classes). The framework sections here summarize only what differs for ADS-B; refer to the MLAT readme for the full framework design.

The per-transponder breakdowns are surfaced as **tables** (one row per aircraft address). Spatial breakdowns are surfaced as **summary heat-maps grouped by MOPS version and by target type** - not per transponder, since a typical recording carries hundreds-to-thousands of transponders and one figure per address is infeasible. The "target type" group is the reconstructed target category (`DBContentManager::emitterCategory(utn)`), not the raw I021/020 emitter category of a single report.

## Summary

Analyze Data Source is a standalone COMPASS task for examining a chosen data source from multiple angles. Analyzes are organized by **DSType** and within each DSType by a set of **Inspectors** - focused components that each produce one section of the result report. The MLAT DSType (CAT020 / CAT010) shipped first; this document covers the **ADSB** DSType (**CAT021**).

The ADS-B DSType ships three inspectors, mirroring MLAT Features 1-3:

1. **Data Item Analysis** (free) - cumulative per-CAT data-item tables.
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
| Data Item Analysis (Feature 1) | ADSB | free, always | `src/task/analyze/adsb/` | none |
| Sensor Coverage / PD (Feature 2) | ADSB | free, always | `src/task/analyze/adsb/` | none |
| Position Accuracy (Feature 3) | ADSB | pro + experimental | `experimental_src/analysis/adsb/` | Professional license |

(The pro + experimental inspector requires `USE_EXPERIMENTAL_SOURCE` at build time to be compiled in; this is the case for the AppImage distribution. With the source compiled in, the pro inspector is always visible in the dialog and only gets greyed out by the license check. Source builds without `USE_EXPERIMENTAL_SOURCE` (developer-only) do not see the pro inspector at all.)

Unlike MLAT, the ADS-B DSType has no CAT-conditional inspectors: ADS-B is a single category (CAT021), so there is no equivalent of MLAT's "CAT020 only" RU inspectors.

## User Workflow

1. Import CAT021 data.
2. Run reconstruction to produce the Reference Trajectory (always required - the task does not launch otherwise).
3. From the main menu, open Tasks - Analyze ADS-B Data Source. The dialog opens with the DSType pre-bound to ADSB; the user selects one or more CAT021 data sources and ticks the inspectors to run. Inspectors whose prerequisites are not met (no professional license; RefTraj not present) are greyed out with a tooltip giving the reason. Each inspector's settings widget appears on the right when its row is highlighted; the data source selection has its own configuration page on the right too.
4. Click Run. **All selected data sources are folded into one combined dataset** (the `AnalysisDataset`, loaded once before any inspector runs), like in the Evaluation for test sources - there is no per-data-source analysis. The per-transponder breakdown of Features 2 and 3 happens *within* that combined dataset, keyed by aircraft address, not per data source.
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
|     [x] Data Item Analysis           |  widget shown here)                 |
|     [x] Sensor Coverage              |                                     |
|     [x] Position Accuracy   [pro]    |                                     |
|                                      |                                     |
+----------------------------------------------------------------------------+
|                                              [ Cancel ]   [ Run ]          |
+----------------------------------------------------------------------------+
```

The Data Source node reuses `DataSourcesUseWidget` filtered to CAT021 data sources. It also carries the shared **Scope Filter** (Use Ground Only, Minimum / Maximum Flight Level) - see [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md#configuration-dialog). For ground ADS-B work, "Use Ground Only" keeps only on-ground reports (ground bit set, or a ground-only target type); ground state on the reference comes from RefTraj's `surface_target` via the "Ground Bit" meta-variable.

---

## Result Report Structure

All selected data sources are folded into one combined dataset before any inspector runs; the report does not split by data source. It does, however, surface per-transponder breakdowns inside Features 2 and 3.

```
Analyze ADS-B Data Source                         << result root
|
+-- Overview                                       << framework-owned
|     Run Configuration table: DSType, selected data sources (count
|     + names), test line, selected reference data sources, reference
|     line, professional-license state, scope filter (use ground only,
|     flight-level band).
|
+-- Data Item Analysis                            << Feature 1
|     About text.
|     Summary table (Data Source, CATs, Total records).
|     One sub-section per data source, one table per CAT seen
|       (Item, Count, Min, Max, Description); items defined in the
|       active edition but never seen are listed with count 0 (red).
|
+-- Sensor Coverage                               << Feature 2
|     About text.
|     Settings recap (nominal UI, miss tolerance, reference-period
|       split threshold, grid resolution, PD color thresholds).
|     Summary table: targets walked / without RefTraj / without test
|       data, transponder count, total #EUI, total #MUI, overall PD,
|       cells with EUI, median + P5 per-cell PD, worst cell PD.
|     Calculated Update Interval table: measured inter-report cadence
|       (median / P10 / P90 / mean, sample count).
|     Figure: Update Interval Histogram (measured cadence).
|     Per-Transponder PD table: aircraft address, callsign, #EUI,
|       #MUI, PD (sorted ascending PD - weakest transponders on top).
|     Figure: PD - Horizontal.
|     Figure: PD - Altitude/Longitude.
|     Figure: PD - Altitude/Latitude.
|     PD by MOPS Version: overview table + one horizontal PD map per
|       MOPS version.
|     PD by Target Type: overview table + one horizontal PD map per
|       reconstructed target type (busiest 12, rest noted).
|
+-- Position Accuracy                             << Feature 3
      About text.
      Settings recap (grid resolution, position-accuracy color,
        consistency color, dubious factor threshold; reports w/o QI
        assessed for offset only).
      Summary table: test reports walked / ignored (no position) /
        ignored (no RefTraj) / w/o quality indicator (offset only),
        transponder count, median/mean/P95 offset, mean/median reported
        std-dev, mean/median consistency ratio, cells with samples.
      Per-Transponder Accuracy table: aircraft address, callsign, MOPS
        version, dominant QI, samples, no-QI count, median offset, P95
        offset, mean reported, consistency factor (sorted by factor desc).
      Transponder Consistency (worst first) table: one row per (aircraft
        address, QI) with a computable factor, sorted by consistency factor
        descending, with a Dubious flag for rows >= threshold (default 3.0).
      Accuracy Grids index table (Quantity, Projection, Min/Mean/Max),
        linking to each figure.
      One sub-section per view (Horizontal Position Offset, Reported
        Position Accuracy, Reported Accuracy Consistency), each with an
        Overview table and three projection figures (horizontal +
        altitude/longitude + altitude/latitude).
      Accuracy by MOPS Version: overview table + per MOPS version a
        horizontal map per view (HPO / RPA / RAC).
      Accuracy by Target Type: same, per reconstructed target type
        (busiest 12, rest noted).
```

The framework owns the result root and the Overview section; each inspector contributes its own sub-tree below the root. Section heading text comes from the inspector's `name()`.

---

## ADS-B inspectors - features

### Feature 1: Data Item Analysis (ADSB, free)

Always calculated. Report-only inspector (`ADSBDataItemInspector`) drawn from the cumulative ASTERIX probing information; produces no grid output. Behaves exactly like the MLAT Feature 1 inspector, scoped to CAT021. `requiresLoadedDataset()` is false - it works from metadata only.

**Source:** the persistent per-(DS, CAT, item) summary held by `DBContextManager::asterixInfo()` (db_info key `"asterix_info"`, see [readme_context.md](../../core/context/readme_context.md)). Refined on every import via `DBContextManager::mergeAsterixInfo(...)`; reflects the cumulative state across all imports into the open DB, not just the most recent file.

**Scope:** restricted to the CAT021 data sources configured for this analysis. A per-CAT include flag (`included_cats_` in `ADSBDataItemInspectorSettings`) lets the user exclude individual categories; CATs not present in the map default to enabled.

**What is shown:** a Summary table (Data Source, CATs, total records), then one sub-section per data source containing one table per CAT in scope listing every data item defined in the active edition with cumulative `Count`, `Min`, `Max`, `Description`. The per-DS tables are rendered by `ASTERIXReportHelpers::renderDataItemTablesForDS(...)`, the same helper the per-file ASTERIX Import report uses, so items defined but never seen appear with count 0 (red). Useful for spotting items that appear only in some recordings (e.g. I021/210 MOPS Version, I021/090 Quality Indicators, I021/140 Geometric Height) or that are present but rarely populated.

**Display:** report-only - no map, no GridView layer.

**Not in scope:** temporal breakdown (per-day / per-hour), cross-source diffs, per-transponder item presence.

---

### Feature 2: Sensor Coverage Analysis / PD (ADSB, free, per transponder)

Always calculated (`ADSBCoverageInspector`). Computes operational PD across a configurable 3D grid, using RefTraj as ground truth, and keeps **per-transponder** counters so PD can be reported per aircraft address as well as per cell.

**What is shown:**
- *Aggregate spatial PD*: for each 3D grid cell (lat, lon, baro_alt), the probability that the ADS-B system delivered a target report when one was expected, scoped to the time the reference trajectory was inside that cell. Aggregation is sum-of-counters across transponders, identical to MLAT Feature 2.
- *Per-transponder PD*: for each aircraft address, the total expected / missed updates and resulting PD. This is the ADS-B-specific breakdown - it identifies individual transponders with coverage gaps (e.g. low-power or intermittent transmitters), which an aggregate map averages away.
- *Measured update cadence*: a "Calculated Update Interval" table and an "Update Interval Histogram" figure summarizing the actual per-transponder inter-report interval (median / P10 / P90 / mean). This is an analysis aid - not in the original design - so the operator can pick a nominal Update Interval that matches the feed (set UI at or above the measured median, near P90, to avoid counting the feed's own cadence as missed updates). Inter-report intervals above 300 s are treated as coverage gaps and excluded from this cadence statistic.

**Cadence source:** time-difference only - a configured nominal Update Interval (`update_interval_s_`, default 1.0 s), identical to the existing COMPASS detection requirement (see [eval/requirement/detection/readme_detection.md](../../eval/requirement/detection/readme_detection.md), §2.2). ADS-B has no Remote Units and no CAT019-style cycle messages, so the MLAT "period-based via CAT019" cadence variant does not apply here.

**Per-transponder grouping:** the PD slot-walk (see [Design](#pd-computation-feature-2)) runs per target. For ADS-B each reconstructed target corresponds to one transponder; `transponderIdOf(...)` resolves the aircraft address by majority vote across the target's CAT021 reports (guarding the rare mixed-association case) and the first non-empty callsign. The per-target #EUI / #MUI totals are accumulated into a `map<aircraft_address, {eui, mui}>` for the per-transponder table in addition to cell-attributing them for the grid. Reports with no `aircraft_address` are accumulated under a single "unknown" bucket and listed as `(unknown)`.

**Display:**
- Three aggregate projections (horizontal lat/lon, altitude/longitude, altitude/latitude). (Sector outlines are part of the shared grid infrastructure; see the MLAT readme.)
- Per-transponder summary table, sorted ascending by PD so the weakest transponders sort to the top.
- **Summary heat-maps grouped by MOPS version and by target type** (see [Group breakdowns](#group-breakdowns-features-2-and-3)). Per-transponder PD maps are not rendered (one figure per aircraft address would be thousands of figures); the per-transponder table carries the per-address numbers, and the spatial story is told per MOPS version and per reconstructed target type instead. During the walk each target's slots are attributed to the aggregate grid plus its MOPS grid (dominant reported MOPS version) and its target-type grid (the reconstructed `DBContentManager::emitterCategory(utn)`).

**Color scale (configurable):** Red2Green color map across the PD range; thresholds default to `pd_unacceptable_below_ = 0.70` (red) and `pd_acceptable_above_ = 0.95` (green), orange in between.

**Report output:**
- Three aggregate projection images.
- Summary table: total #EUI, total #MUI, overall PD, median / P5 per-cell PD, worst cell (computed over cells with >= 5 EUI).
- Calculated Update Interval table + Update Interval Histogram figure.
- Per-transponder table: aircraft address, callsign, #EUI, #MUI, PD.
- "PD by MOPS Version" and "PD by Target Type" sub-sections: an overview table (group, #EUI, #MUI, PD) and one horizontal PD map per group.

Algorithmic details are in the [Design](#pd-computation-feature-2) section.

---

### Feature 3: Position Accuracy Analysis (ADSB, pro + experimental, per transponder)

`ADSBAccuracyInspector` reports ADS-B position accuracy from three complementary perspectives - the accuracy claimed by the transponder's quality indicators, the offset measured against the Reference Trajectory, and the ratio between them - **grouped per transponder**. This is the offline counterpart to what `ADSBAccuracyEstimator` ([adsbaccuracyestimator.h](../../../experimental_src/reconstruction/complex/adsbaccuracyestimator.h)) computes online inside the ProbIMM reconstructor (the per-source accuracy estimator ADS-B routes to), in the same spirit as MLAT Feature 3 re-deriving `ScaledAccuracyEstimator`.

**Reported accuracy from quality indicators.** ADS-B does not carry a Cartesian standard deviation in the target report (unlike MLAT's I020/500). The reported `tr_std_dev` is obtained from the target report chain's position-accuracy accessor (`tst_chain.posAccuracy(id)` -> `x_stddev_`, `y_stddev_`), as the Cartesian magnitude `sqrt(sx^2 + sy^2)`. The QI-to-stddev derivation itself (NACp primary for MOPS v1/v2, NIC fallback, NUCp for v0, the `/2.45` 95%-to-1-sigma conversion, and the SIL penalty) lives **inside** that accessor / `AccuracyTables` (see the [adsb_accuracy skill](../../../.claude/skills/adsb_accuracy/SKILL.md)), not in the inspector, so the offline analysis and the online estimator agree by construction. A report whose accessor yields no usable, finite, positive std-dev is treated as having no reported accuracy.

**Grouping is per (aircraft address, quality-indicator).** Each target report's `(distance, reported std-dev)` is bucketed per cell of the shared grid and, per transponder, under a `(acad, qi_key)` sub-bucket (the "unknown" acad bucket is keyed -1). The `qi_key` is `(type, value)` with type one of NACp / NUCp / NIC / no-QI, derived from the chain's `mopsVersion`, `nacp` and `nucpNic` accessors the same way `ADSBAccuracyEstimator::qiKey` derives it (NACp when present and non-zero; otherwise NIC for MOPS v1/v2 or NUCp for v0; otherwise no-QI). Keeping NACp and NIC apart even at the same numeric level, and splitting a transponder that changes equipage mid-recording into separate QI states, is what feeds the per-QI dubious detail below. The per-transponder table row aggregates the QI sub-buckets back to one row per aircraft address, showing the dominant MOPS version and dominant QI label.

**Per-target-report quantities collected:**
- `distance_m` - horizontal Cartesian distance (via `Transformation::distanceL2Cart`) between the ADS-B reported position and the mapped reference position at the same timestamp (max reference time difference 2 s).
- `tr_std_dev` - reported position accuracy from `posAccuracy()` as above.

Bucketing for the spatial views: `cell_of(ADS-B reported position)` on `TargetReport3DGrid` via `addAccuracySample(lat, lon, alt_ft, distance, tr_std_dev)`. Each cell keeps running sums and counts for distance, reported std-dev, and the distance/std-dev ratio; **the per-cell value rendered is the mean** of each quantity. Per transponder, the inspector keeps the distance and reported samples and derives `median(distance) / median(reported)` as the consistency factor - the same quantity the online rescaler works with.

**Three aggregate views rendered every run:**

| View | Cell value | What it tells the operator |
|---|---|---|
| **Horizontal Position Offset** (HPO) | per-cell mean of `distance_m` | The position offset of the ADS-B report against the reference trajectory - the operationally observed accuracy. |
| **Reported Position Accuracy** (RPA) | per-cell mean of the reported `tr_std_dev` | What the transponders in that area claim as their position accuracy, where reports are dense, where the reported value is missing. |
| **Reported Accuracy Consistency** (RAC) | per-cell mean of `distance_m / tr_std_dev` | Diagnostic view: how well the reported QI-derived accuracy matches the observed offset. Values near 1.0 mean consistent; much greater than 1.0 means over-confident; much less than 1.0 means over-cautious. |

**Per-transponder output (the ADS-B-specific breakdown):**
- A Per-Transponder Accuracy table keyed by aircraft address: callsign, dominant MOPS version, dominant QI label, sample count, median offset, P95 offset, mean reported std-dev, consistency factor. Sorted by consistency factor descending (most dubious first; rows without a factor sort to the end).
- A **Transponder Consistency (worst first)** table: one row per `(aircraft address, qi_key)` with a computable per-QI consistency factor (`median(distance) / median(reported)` within that QI state), sorted by factor descending - the full ranked list of contributing targets. A `Dubious` column flags the rows at or above `min_factor_of_interest_` (default 3.0); those flagged rows are the report-side equivalent of `ADSBAccuracyEstimator::doACADsOfInterest()`. Keying by QI isolates equipage changes - a transponder that misreports in one state is not averaged away. The Summary reports the total number of `(transponder, QI)` states and how many are dubious.

**Spatial breakdowns:** rendered as summary heat-maps grouped by MOPS version and by reconstructed target type (see [Group breakdowns](#group-breakdowns-features-2-and-3)), not per transponder - one figure per aircraft address would be thousands of figures.

**Display (3 projections per aggregate view):** horizontal (lat/lon), altitude/longitude (lon/alt), altitude/latitude (lat/alt). Each view also gets an Overview table (cells with value, total samples, and the min / median / mean / P95 / max of the cell values). An "Accuracy Grids" index table links every (view, projection) figure with its min/mean/max. The MOPS-version and target-type group sub-sections render the horizontal map for each of the three views per group.

**Color scales (configurable):**
- Reported Position Accuracy and Horizontal Position Offset: Green2Red, green below `pos_acc_acceptable_below_m_` (12 m), red above `pos_acc_unacceptable_above_m_` (60 m).
- Reported Accuracy Consistency: a green-centered ramp in 10 discrete bands - blue (over-cautious) at `consistency_lower_unacceptable_below_` (default 0.2 = 1/5x), green at the ideal 1.0, red (over-confident) at `consistency_upper_unacceptable_above_` (default 5.0 = 5x). 1.0 stays green regardless of the endpoints (blue->green is interpolated below 1.0, green->red above), so the scale extends well past 2.0 without pushing the ideal off-center.

**Report output:**
- For each aggregate view: three projection images (horizontal plus two altitude cuts) under a per-view sub-section.
- Summary table: median / mean / P95 offset, mean / median reported std-dev, mean / median consistency ratio.
- Per-transponder table and dubious-transponders sub-table as above.

---

### Reports without a usable quality indicator

A report whose `posAccuracy()` accessor returns no finite positive std-dev (the practical signature of a missing or zero quality indicator, e.g. NACp 0) is **still assessed as far as possible**: its position offset against RefTraj is computable, so it contributes to the **offset** statistics (HPO and per-transponder median / P95 offset). It carries no reported accuracy, so it adds no Reported Position Accuracy value and no consistency ratio, and its `qi_key` type is `no QI` (which yields no consistency row). These reports are not silently dropped - they are counted separately:

- Summary: `Reports w/o quality indicator (offset only)` (alongside the fully-ignored `no position` and `no RefTraj match` counts).
- Per-transponder accuracy table: a `No QI` column gives the per-transponder count.

Feature 2 PD is unaffected - a delivered report is delivered regardless of its quality indicator.

---

## ADS-B inspectors - 3D Grid Infrastructure

ADS-B reuses the shared `TargetReport3DGrid` and projection model unchanged - see [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md#mlat-inspectors---3d-grid-infrastructure) for cell dimensions, indexing, projection modes, sector overlay, and GridView display. Cell sizes are clamped to the task's max-cells-per-axis budget via `AnalyzeDataSourceTask::clampedCellSizes(...)`. Two ADS-B-specific notes:

- **Per-transponder grids are never materialized.** Per-transponder results are kept as lightweight counter / sample maps and surfaced as tables; spatial breakdowns are rendered grouped by MOPS version and by reconstructed target type, each a separate sparse grid built during the same walk. This is the key scaling difference from MLAT's per-RU layers (RUs number in the tens, transponders in the hundreds-to-thousands): MOPS versions are at most three and target types are capped at the busiest 12, so the figure count stays bounded regardless of transponder count.
- **Altitude axis** is the altitude carried by the target-report position accessor (barometric flight level in normal CAT021 feeds, I021/145 -> `mode_c_code`), and on the reference side the altitude of the mapped reference position. When a report or reference position carries no altitude, the contribution falls back to altitude 0 for that sample. I021/140 Geometric Height (WGS-84 ellipsoid) is **not** used for the grid axis - its reference frame differs from RefTraj's barometric altitude. (Geometric height may still be reported descriptively in Feature 1.)

---

## ADS-B inspectors - Data Inputs

| Use | CAT021 |
|---|---|
| Time of applicability | I021/071 (Time of Applicability for Position) |
| Position (WGS-84) | I021/130 / I021/131 (high-res) -> `latitude`, `longitude` |
| Barometric altitude | I021/145 (Flight Level) -> `mode_c_code` |
| Transponder identity | I021/080 (Target Address, 24-bit ICAO) -> `aircraft_address` |
| Callsign | I021/170 -> `aircraft_identification` |
| Reported accuracy (via `posAccuracy()`) | I021/090 -> NACp / NUCp / NIC / SIL -> reported std-dev |
| Quality-indicator grouping | I021/210 (`Chain::mopsVersion`), I021/090 (`Chain::nacp`, `Chain::nucpNic`) -> `qi_key` + dominant MOPS column |
| Target type | reconstructed target category, `DBContentManager::emitterCategory(utn)` |
| Reference trajectory | RefTraj (post-reconstruction) |

The reported position accuracy is consumed through the chain's `posAccuracy()` accessor (the QI-to-stddev derivation lives inside it); the raw QI values are read separately through the `Chain::mopsVersion` / `nacp` / `nucpNic` accessors only to form the `qi_key` and the dominant-MOPS column. The spatial group dimension "target type" is the reconstructed target category, not the raw I021/020 emitter category of an individual report. I021/140 Geometric Height is not used as the grid altitude axis (see [3D Grid](#ads-b-inspectors---3d-grid-infrastructure)). I021/132 (MAM) and other items are out of scope for these three inspectors.

---

## ADS-B inspectors - Design

Cross-cutting design notes. The PD computation and altitude-axis handling are shared with MLAT; only the per-transponder accumulation and the QI-derived reported accuracy are ADS-B-specific.

### PD computation (Feature 2)

The same per-target slot-walk as MLAT Feature 2, built on the shared helpers in [detection_pd_helpers.h](../../eval/requirement/detection/detection_pd_helpers.h) (`MissTestParams`, `RefPeriod`, `buildReferencePeriods`, `isMiss`, `numMisses`): reference periods are built from the loaded reference chain (a gap larger than `ref_max_time_diff_s_`, default 4 s, starts a new period; minimum period length 1 s), each period is walked in nominal-UI steps attributing #EUI to `cell_of(RefTraj at t_slot)`, and each gap that passes the miss test contributes #MUI at the reference position of each missed slot. The only addition over a plain detection run is that the per-target #EUI / #MUI totals are also accumulated into a `map<aircraft_address, {eui, mui}>` for the per-transponder table. Per-target PD is therefore identical to what the detection requirement would produce; per-cell PD is a strict spatial refinement; per-transponder PD is the per-target total surfaced by aircraft address.

There is no CAT019 cadence variant for ADS-B.

### Reported accuracy derivation (Feature 3)

The reported `tr_std_dev` is obtained from the target report's `posAccuracy()` accessor, which performs the QI-to-stddev lookup the reconstructor uses (NACp primary, NIC fallback, NUCp for v0; `/2.45`; SIL penalty), so the offline analysis and the online estimator agree by construction. The branch order and conversion are described in the [adsb_accuracy skill](../../../.claude/skills/adsb_accuracy/SKILL.md). The `qi_key` used for the per-transponder sub-buckets and the dubious detail is derived from the raw `mopsVersion` / `nacp` / `nucpNic` values, mirroring `ADSBAccuracyEstimator::qiKey`.

### Group breakdowns (Features 2 and 3)

Both grid inspectors render their spatial output not only as one aggregate grid but also grouped by **MOPS version** and by **target type**, because a per-transponder grid would mean one figure per aircraft address (hundreds-to-thousands). During the single walk, each sample / slot is attributed to the aggregate grid plus the grid for its MOPS version and the grid for its target type, and group-level sample stats are accumulated alongside. MOPS versions are at most three; target types are capped at the busiest `kMaxTypeGroups` (12) by sample / #EUI count, and the number dropped is noted in the group overview table. The target type is the reconstructed target category (`DBContentManager::emitterCategory(utn)`), constant per transponder; MOPS version for Feature 2 is the transponder's dominant reported value, and for Feature 3 is per report.

---

## Architecture

### What Is Reused

Everything the MLAT inspectors reuse (see [readme_analysis_mlat_ru.md](readme_analysis_mlat_ru.md#what-is-reused)), in particular:

| Component | File | Used for |
|---|---|---|
| `DataSourceInspectorBase` / `InspectorSettingsBase` | `src/task/analyze/` | Inspector base classes + settings sub-configs |
| `AnalysisDataset` | `src/task/analyze/analysisdataset.h/.cpp` | Combined dataset (test buffers + per-UTN ref/test chains), loaded once |
| `TargetReport3DGrid` | `src/task/analyze/targetreport3dgrid.h/.cpp` | 3D grid, #EUI/#MUI + accuracy-sample layers, projections |
| `AnalyzeDataSourceTask` | `src/task/analyze/analyzedatasourcetask.h/.cpp` | Task entry point (one instance per DSType) |
| `AnalyzeDataSourceDialog` | `src/task/analyze/analyzedatasourcedialog.h/.cpp` | Pre-run dialog |
| `DataSourceAnalysisTaskResult` | `src/task/analyze/datasourceanalysistaskresult.h/.cpp` | Result container |
| Detection PD helpers | `src/eval/requirement/detection/detection_pd_helpers.h` | Reference-period construction, gap walking, miss test |
| `posAccuracy()` accessor + `AccuracyTables` | `src/db/dbcontent/target/` | QI -> stddev (NUCp / NACp / NIC, /2.45, SIL penalty) |
| `ASTERIXReportHelpers` | `src/task/import/asterix/` | per-DS data-item table rendering (Feature 1) |
| `DBContextManager` | context system | data source resolution; sector definitions; `asterixInfo()` for Feature 1 |
| Reference position interpolation | `AnalysisDataset::mappedRefPos(...)` | per-slot / per-report reference position lookup |

### New Classes

Inspector components use the `...Inspector` suffix and derive from `DataSourceInspectorBase`. The base class splits work into `compute(AnalysisDataset*)` (heavy data-side walk, may run off the main thread, stores results on the instance) and `writeReport(ResultReport::Section&)` (main thread, emits the stored results). Inspectors also expose `requiresLoadedDataset()`, `requiresReferenceTrajectory()`, `requiresProfessionalLicense()`, `testDBContentNames()`, and `prerequisitesMet()`. (This is the implemented interface; the original design's `run()` / `generateReportSections()` names were superseded.)

Free inspectors live under `src/task/analyze/adsb/`; the pro + experimental inspector lives under `experimental_src/analysis/adsb/` (compiled in only when `USE_EXPERIMENTAL_SOURCE` is set).

**ADSB free inspectors (in `src/task/analyze/adsb/`):**

| Class | Path | Purpose |
|---|---|---|
| `ADSBDataItemInspector` (+ `ADSBDataItemInspectorSettings`) | `src/task/analyze/adsb/adsbdataiteminspector.h/.cpp` | Feature 1: read per-DS `asterixInfo`, emit CAT021 data-item tables (report-only, no dataset load) |
| `ADSBCoverageInspector` (+ `ADSBCoverageInspectorSettings`) | `src/task/analyze/adsb/adsbcoverageinspector.h/.cpp` | Feature 2: per-target slot-walk on `TargetReport3DGrid` + per-transponder counter map; 3 projections, per-transponder table, measured-cadence table + histogram, PD heat-maps grouped by MOPS version and target type |

**ADSB pro + experimental inspector (in `experimental_src/analysis/adsb/`):**

| Class | Path | Purpose |
|---|---|---|
| `ADSBAccuracyInspector` (+ `ADSBAccuracyInspectorSettings`) | `experimental_src/analysis/adsb/adsbaccuracyinspector.h/.cpp` | Feature 3: QI-derived reported accuracy vs. RefTraj offset, grouped per `(aircraft address, qi_key)`; three aggregate views + per-transponder and dubious-transponder tables + accuracy heat-maps grouped by MOPS version and target type |

### DSType binding

The originally proposed "branch on `ds_type_`" (shape 1) was implemented. `AnalyzeDataSourceTask` takes a DSType default at construction; `registerInspectors()`, `generateSubConfigurable()`, and `checkSubConfigurables()` branch on `ds_type_` to build either the MLAT or the ADS-B inspector set ([analyzedatasourcetask.cpp](analyzedatasourcetask.cpp), `registerInspectors()` and the `generateSubConfigurableFromConfig` calls). `TaskManager` registers a second instance under the configurable class name `AnalyzeADSBDataSourceTask` with `ds_type` `"ADSB"` ([taskmanager.cpp](../taskmanager.cpp)), exposed via `TaskManager::analyzeADSBDataSourceTask()`, alongside the MLAT `AnalyzeDataSourceTask`. The dialog, result container, and 3D grid are shared and unchanged. A DSType registry was not introduced; revisit when a third DSType appears.

---

## Implementation Plan

### Phase 1 - DSType framework support + ADS-B skeleton

- [x] Make `AnalyzeDataSourceTask` inspector registration DSType-driven (branch on `ds_type_`); MLAT behavior unchanged when `ds_type_ == "MLAT"`.
- [x] Register a second `AnalyzeDataSourceTask` instance bound to `"ADSB"` in `TaskManager` (`AnalyzeADSBDataSourceTask`), with its own menu entry "Analyze ADS-B Data Source".
- [x] Create `src/task/analyze/adsb/` and `experimental_src/analysis/adsb/` with `CMakeLists.txt` files; gate the experimental dir on `USE_EXPERIMENTAL_SOURCE`.
- [x] Implement `ADSBDataItemInspector`, `ADSBCoverageInspector`, `ADSBAccuracyInspector` (+ settings).

### Phase 2 - Feature 1: Data Item Analysis

- [x] `ADSBDataItemInspector::writeReport()`: per configured CAT021 data source, read its per-CAT item summary from `DBContextManager::asterixInfo()`, emit a Summary table + per-DS tables via `ASTERIXReportHelpers::renderDataItemTablesForDS` (count-0 rows for unseen items).

### Phase 3 - Feature 2: Sensor Coverage / PD (per transponder)

- [x] `ADSBCoverageInspector::compute()` on `TargetReport3DGrid`: per target, build reference periods and walk them in UI steps (reuse `detection_pd_helpers`), attribute #EUI / #MUI to `cell_of(RefTraj at t_slot)`, accumulate per-target totals into a `map<aircraft_address, {eui, mui}>`.
- [x] Emit three aggregate projections + summary table + per-transponder table.
- [x] Measured update-interval statistics table + histogram figure (analysis aid, beyond original design).
- [x] PD summary heat-maps grouped by MOPS version and by target type (replaces infeasible per-transponder figures).

### Phase 4 - Feature 3: Position Accuracy (per transponder, pro + experimental)

- [x] `ADSBAccuracyInspector::compute()` in `experimental_src/analysis/adsb/`:
  - Per associated CAT021 target report: compute `distance_m` to the mapped reference position; read `tr_std_dev` from `posAccuracy()`.
  - Bucket per cell on `TargetReport3DGrid` (means) and per `(aircraft address, qi_key)` (median offset, P95 offset, median/mean reported, consistency factor).
  - Reports without a usable quality indicator are still assessed for offset (no toggle); they contribute to offset statistics under the `no-QI` qi_key and are counted separately in the Summary and per-transponder table.
  - Render three aggregate views x three projections; emit per-transponder table + dubious-transponders sub-table (factor > 3.0).
- [x] Group per `(aircraft_address, qi_key)` (NACp / NUCp / NIC kept apart) via the `Chain::mopsVersion` / `nacp` / `nucpNic` accessors.
- [x] MOPS-version / dominant-QI columns in the per-transponder table and per-QI detail in the dubious sub-table.
- [x] Accuracy summary heat-maps grouped by MOPS version and by target type (replaces infeasible per-transponder figures).

### Deferred / out of scope (this release)

- [ ] Finer breakdown of the "no quality indicator" count by cause (NACp 0 vs missing MOPS vs no QI item at all). The shipped predicate is simply "the position-accuracy accessor yields no usable std-dev"; the reconstruction's `isRiskyADSB()` classification is intentionally not replicated in the analysis.
- [ ] Velocity accuracy (NACv / NUCr) analysis - this feature set covers position only.
- [ ] Geometric-height accuracy (GVA) analysis and a geometric-vs-barometric altitude comparison.
- [ ] Per-ground-station breakdown (would require a receiver identifier ADS-B target reports do not generally carry).
</content>
</invoke>
