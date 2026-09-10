# COMPASS Evaluation Framework

Compares test data sources against reference data sources within defined airspace sectors. Pre-requisites: UTN associations, at least 1 sector, usable reference + test data.

**Supported standards**: EUROCAE ED-116, ED-117/A, ED-129C, ED-87C/D/E, ED-142, EUROCONTROL Radar Surveillance Standard

**Requirement types**: Detection (PD), Position (distance, along/across, RMS, radar azm/rng), Latency (position latency, ADS-B latency), Identification (correct/false/change delay), Mode 3/A (present/false), Mode C (correct/false/present), Speed, Track angle, Dubious targets/tracks, Extra data/tracks, MoM (longitudinal/transversal/vertical), Acceleration, ROCD, Track coasting

**Results**: Per-sector averages + per-target statistics, drillable to per-target-report level. Exportable as PDF, LaTeX, or JSON reports. Optional splits by ADS-B MOPS version or Mode A/C vs Mode S.

## Standard source documents

The EUROCAE standard documents (ED-116, ED-117/A, ED-87 series, ED-142, and others) are located in `/home/sk/Nextcloud/documents/standards/EuroCAE`. Some files are outdated editions. Read them only when needed for a specific requirement definition or threshold value - the documents are quite substantial.

## Main classes

| Class | Location | Role |
|---|---|---|
| `EvaluationManager` | `src/eval/evaluationmanager.h` | Top-level manager, owns the calculator and target filter |
| `EvaluationCalculator` | `src/eval/evaluationcalculator.h` | Owns all standards, data source selection, sector usage, settings; runs `evaluate()` |
| `EvaluationSettings` | `src/eval/evaluationsettings.h` | Evaluation parameters (report splits, filters, thresholds) |
| `EvaluationStandard` | `src/eval/standard/evaluationstandard.h` | One named standard; holds requirement groups; general settings `reference_max_time_diff`, `ignore_primary_only_targets`, `ignore_non_adsb_targets`; `targetIgnoreReason()` |
| `Group` | `src/eval/requirement/group.h` | Named requirement group inside a standard; holds requirement configs |
| `EvaluationRequirement::BaseConfig` | `src/eval/requirement/base/baseconfig.h` | Configurable per-requirement config (name, short_name, comment, use, thresholds); factory `createRequirement()` |
| `EvaluationRequirement::Base` | `src/eval/requirement/base/base.h` | Runtime requirement; `evaluate(target_data, instance, sector_layer)` returns a per-target result |
| `EvaluationData` / `EvaluationTargetData` | `src/eval/data/` | Per-UTN reference + test data used during evaluation |
| `EvaluationResultsGenerator` | `src/eval/results/evaluationresultsgenerator.cpp` | Drives the evaluation loops and result collection |
| `EvaluationRequirementResult::Single` / `Joined` | `src/eval/results/base/single.h`, `joined.h` | Per-target result / per-sector sum result |
| `EvaluationTaskResult` | `src/eval/results/evaluationtaskresult.h` | Stores the finished evaluation as a task result with report |
| `ResultReport::Report` | `src/task/result/report/report.h` | Generic report structure (sections, tables, figures, viewables) |

Intermediate base classes exist for common requirement kinds: `ProbabilityBase(Config)`, `IntervalBase(Config)`, `PositionBase(Config)` in `src/eval/requirement/base/`. Each config class has a matching `*ConfigWidget` for the GUI.

The Detection requirement selects its period source with `pd_calculation_method`. With `time_difference`, the default, it derives the missed intervals from the gaps between test reports and the configured update interval. With `status_message` it takes the periods from the update cycles the test data source reports, that is CAT019 message type 001 or CAT010 message type 002, and falls back to the time difference method when the source reports none. On top of that it supports 3 counting modes: counted update intervals (default, #MUI over #EUI), with `use_time_ratio` missed time over reference duration per EUROCAE ED-129C Appendix C, and with `use_gap_count` the number of gaps over the number of test reports per EUROCAE ED-117A Section 6.4.8 and ED-87E Section 5.3.14. In the gap count mode every gap counts once, independent of its length, and the requirement always walks the time differences. The minimum and maximum gap length select on the measured gap, the miss tolerance applies only to the update interval test, so a tolerance never shifts a gap length threshold. `use_stationary_ui` selects the update interval per gap from the reference ground speed, for surface targets that transmit slower when stationary. Details and the derivation: [readme_detection.md](src/eval/requirement/detection/readme_detection.md), [readme_ed129c.md](experimental_src/readme_ed129c.md).

The Position Distance requirement compares one distance per test report by default. With `use_averaging` it instead compares one mean position error per `averaging_window_s` window, for requirements stated on an averaged position such as EUROCAE ED-117 Section 3.3.3 for stands.

Probability results are shown in percent. The number of decimals follows the requirement threshold, one digit finer, so a value that fails a fine threshold such as 1e-6 is not rounded to zero in the report.

## How a standard is formed

Standards are pure configuration - no code change is needed to create or modify one.

- Everything lives in the `Configurable` tree persisted in `conf/default/eval.json` (per-user copy in `~/.compass/<version>/conf/default/eval.json`):
  - `EvaluationManager` > `EvaluationCalculator` > `EvaluationStandard` (one per standard) > `EvaluationRequirementGroup` (one per group, e.g. "Common", "Manoeuvring Area") > requirement config objects.
- Each requirement config object is one instance of a `*Config` class (e.g. `EvaluationRequirementDetectionConfig`) with parameters such as `name`, `short_name`, `comment` (cite the standard section here, e.g. "From EUROCAE ED-117 Section 3.3.5"), `use`, `prob`, `prob_check_type`, and type-specific thresholds.
- The map `Group::requirement_type_mapping_` in `src/eval/requirement/group.cpp` lists all available requirement config class names and their display names. Only classes in this map can be added to a group.
- At runtime, standards are managed via `EvaluationCalculator` (`addStandard()`, `copyCurrentStandard()`, `renameCurrentStandard()`, `deleteCurrentStandard()`) and edited in the standard tree GUI (`src/eval/standard/evaluationstandardtreemodel.h`).
- Which requirement group applies to which sector layer is selected per sector layer (`EvaluationCalculator::useGroupInSectorLayer()`).

To create a new standard: copy the closest existing one in the GUI, adjust the groups and requirement parameters, and cite the standard document sections in the `comment` fields. Alternatively add a new `EvaluationStandard` block in `eval.json` following the existing structure.

Requirement sources are grouped by topic: `detection/`, `position/`, `latency/` (position latency and ADS-B latency), `identification/`, `mode_a/`, `mode_c/`, `speed/`, `trackangle/`, `mom/`, `dubious/`, `extra/`, `status/`, `generic/`, each with a matching folder in `src/eval/results/`. A worked example of a full standard, from requirement mapping to configuration and verification, is [readme_ed129c.md](experimental_src/readme_ed129c.md).

To add a new requirement type (code change): create a `*Config` class + widget in `src/eval/requirement/<topic>/`, a requirement class derived from `EvaluationRequirement::Base` (or `ProbabilityBase` / `IntervalBase` / `PositionBase`), result classes derived from `Single` and `Joined` in `src/eval/results/<topic>/`, and register the config class in `Group::requirement_type_mapping_` and the group's `generateSubConfigurable()`.

## How results are calculated

After loading, `EvaluationTargetData::finalize()` asks the current standard via `EvaluationStandard::targetIgnoreReason()` whether the target is relevant for it: `ignore_primary_only_targets` skips targets without any secondary attribute, `ignore_non_adsb_targets` skips targets never detected in CAT021 (`Target::dbContentCount`), and `ignore_mode_ac_only_targets` skips targets with a Mode 3/A or Mode C code but no Mode S attribute, for standards that state their requirements for Mode S targets only. The resulting `ignored_by_std_` flag plus reason lives on the target data only, is never persisted, and therefore does not touch the user-set `use_in_eval_` (manual selection, Filter UTNs). `Single::updateUseFromTarget()` turns it into an ignored result with that reason, so ignored targets appear as unusable results and do not enter the sector sums.

`EvaluationCalculator::evaluate()` checks pre-conditions, then calls `EvaluationResultsGenerator::evaluate(standard, utns, requirements, update_report)`:

1. Loop over all sector layers, then over all used requirement groups (if enabled for that sector layer), then over all used requirement configs.
2. For each requirement config, `createRequirement()` builds the runtime requirement object.
3. The requirement is evaluated per UTN via `tbb::parallel_for`: `req->evaluate(target_data, req, sector_layer)` returns one `Single` result per target, containing `EvaluationDetail` entries (per-update events with positions and comments).
4. All `Single` results are accumulated into one `Joined` sector sum per requirement (`accumulateSingleResult()`), plus optional extra sums when splitting by ADS-B MOPS version or Mode A/C vs Mode S.
5. `computeResult_impl()` in each result class produces the final value (probability, RMS, etc.), which is checked against the requirement condition (`prob_check_type` / threshold comparison) to give passed/failed.

Results can be recalculated partially: `updateResultsToChanges()` reacts to target usage changes (excluded UTNs, excluded time windows, excluded requirements) without a full re-evaluation.

## How the report is written

- The generator writes results into the task result system: `TaskManager::beginTaskResultWriting(name, TaskResultType::Evaluation)` creates an `EvaluationTaskResult` with a `ResultReport::Report`; `endTaskResultWriting()` stores it in the database.
- All `Joined` results are added to the report first, then all `Single` results (`addToReport()` on each). Section IDs come from `EvalSectionID` (`src/eval/results/evalsectionid.h`).
- `Joined::addToReport()` fills the per-sector overview tables (sector infos, condition, result value, passed/failed). `Single::addToReport()` fills per-target tables and per-target detail sections (headers/values via `targetTableHeaders*()` / `targetTableValues*()`, the per-target-report details table comes from the Report Tables, see below).
- Results also provide viewables and annotations (grids, histograms, scatter series via `FeatureDefinition` classes in `src/eval/results/base/`) that render in Views when a result row is selected.
- Large content is loaded on demand (`loadOnDemandTable_impl()` etc. in `EvaluationTaskResult`) to keep the stored report small. The on-demand content is read from the Report Tables, single result details are purged after the joined results are updated and are never recomputed.
- Export to PDF, LaTeX, or JSON goes through the generic report export (`--export_report ... --export_report_mode JSON`), not through evaluation-specific code.

## Report Tables

Every evaluation writes its per-target-report values into the database, one **Report Table** per requirement and sector layer. The tables persist what the purged `EvaluationDetail` objects held, and their columns are offered as **Report Variables** in the Views. Design and the decisions of 2026-09-09: [readme_dynamic_dbcontent.md](experimental_src/readme_dynamic_dbcontent.md), the load path: [readme_loading.md](src/db/dbcontent/readme_loading.md) section "Report Table join".

- **Definition.** `Single::reportTableDefinition()` builds the table: key `identifierFrom("<layer>_<group>_<requirement>")`, display name `"<layer> - <short name>"`, host content the test DBContent (the reference DBContent for the gap families). Every table carries `rec_num` (the key) and `utn`. Test report tables add `dt_prev_s`, the test and reference positions and the two bracketing reference record numbers. Gap tables (Detection, the interval families) hold one row per gap keyed by the first reference sample inside it, with `gap_begin`, `gap_end`, `duration_s`, both reference positions and the test record numbers before and after. Each family adds its own columns in `addReportTableColumns()` and fills them in `fillReportTableRow()`, the position families through `common_addReportTableColumns()` with a value column named by the result type (`distance_m`, `range_offset_m`, `azimuth_offset_deg`, `along_m`, `across_m`, `latency_s`). Column names must not be SQL keywords, `ReportTableDefinition::validate()` rejects them (`end` was one).
- **Writing.** `EvaluationResultsGenerator::evaluate()` begins the task result writing at its start so the result id exists, and calls `writeReportTable(results)` per requirement and sector layer before the detail purge. `Single::addReportTableRows()` writes one row per detail, for nested details (dubious families) one row per child detail with `period_begin` and `period_end`. `ReportTableRows` drops a row whose key is taken or missing with a warning, which keeps the one-to-one relation the join needs (two detection gaps of one target can overlap). Rows are written for every evaluated target, used or not, but only for details: with `report_skip_no_data_details` (default true) an update outside the sector, without reference or with an inaccurate reference has no detail and therefore no row, the row count is "#CP" plus "#CF", not "#Pos". The catalog (`ReportTableInfo` with `num_rows`) is stored in `TaskResultHeader::tables` when `endTaskResultWriting()` finalizes the tables.
- **Report content.** `EvaluationTaskResult::loadOnDemandTable_impl()`, `loadOnDemandFigure_impl()` and `loadOnDemandViewable_impl()` find the table through `reportTableFor()` (the `report_table` content property, else the key from layer, group and requirement), read the rows of the target with `loadReportTableRows()` and render them with the `ReportTableContent` namespace (`src/eval/results/reporttablecontent.h`): the "Target Report Details" table shows every column except `rec_num` and `utn` plus a "Comment" built from the flag columns, the "Target Errors Overview" figure and the row highlight draw the same annotations the result objects draw. No result object is needed for this, so a restarted client shows the content without evaluation or data load.
- **Usage changes.** A target usage change recomputes the sector sums from the in-memory results as before, the tables stay. The "Sector Overview" figures of the joined result are rebuilt from the rows of the used targets through `Single::loadDetailsFromReportTable()`, the only path that still builds `EvaluationDetail` objects, with the per-family inverse `fillDetailFromReportTableRow()`.
- **Lifecycle.** `DBInterface::saveResult()` keeps the tables of the saved result and drops stale ones under its id, `deleteResult()` drops them, `TaskManager::loadResults()` drops every `result_*` table no report header references. A Locked report keeps its tables.
- **Views.** `TaskManager::updateReportContents()` fills the registry `DBContentManager::reportContents()` at database open and after a report is saved or deleted. The variable selection dialog offers every report with record tables as its own entry, the Views, the Geographic View labels and the filter conditions load a Report Variable through a LEFT JOIN on the record number, see the Variable model in [readme_dbcontent.md](src/db/dbcontent/readme_dbcontent.md).
- **Tests.** `experimental_src/py/tests/eval/eval_report_tables.py` exports the report as JSON, reads the catalog with `get_existing_reports`, loads the Report Variables joined onto CAT048 with `get_dbcontent_data`, and checks the joined values against the catalog, the passed and failed rows per UTN against the "Targets" tables and the totals against the "Sector Overview" tables of the export. `eval_report_variables_ui.py` selects a Report Variable in a Scatter Plot View and a Table View.
