# COMPASS Evaluation Framework

Compares test data sources against reference data sources within defined airspace sectors. Pre-requisites: UTN associations, at least 1 sector, usable reference + test data.

**Supported standards**: EUROCAE ED-116, ED-117/A, ED-87C/D/E, ED-142, EUROCONTROL Radar Surveillance Standard

**Requirement types**: Detection (PD), Position (distance, along/across, latency, RMS, radar azm/rng), Identification (correct/false), Mode 3/A (present/false), Mode C (correct/false/present), Speed, Track angle, Dubious targets/tracks, Extra data/tracks, MoM (longitudinal/transversal/vertical), Acceleration, ROCD, Track coasting

**Results**: Per-sector averages + per-target statistics, drillable to per-target-report level. Exportable as PDF, LaTeX, or JSON reports. Optional splits by ADS-B MOPS version or Mode A/C vs Mode S.

## Standard source documents

The EUROCAE standard documents (ED-116, ED-117/A, ED-87 series, ED-142, and others) are located in `/home/sk/Nextcloud/documents/standards/EuroCAE`. Some files are outdated editions. Read them only when needed for a specific requirement definition or threshold value - the documents are quite substantial.

## Main classes

| Class | Location | Role |
|---|---|---|
| `EvaluationManager` | `src/eval/evaluationmanager.h` | Top-level manager, owns the calculator and target filter |
| `EvaluationCalculator` | `src/eval/evaluationcalculator.h` | Owns all standards, data source selection, sector usage, settings; runs `evaluate()` |
| `EvaluationSettings` | `src/eval/evaluationsettings.h` | Evaluation parameters (report splits, filters, thresholds) |
| `EvaluationStandard` | `src/eval/standard/evaluationstandard.h` | One named standard; holds requirement groups; `reference_max_time_diff` parameter |
| `Group` | `src/eval/requirement/group.h` | Named requirement group inside a standard; holds requirement configs |
| `EvaluationRequirement::BaseConfig` | `src/eval/requirement/base/baseconfig.h` | Configurable per-requirement config (name, short_name, comment, use, thresholds); factory `createRequirement()` |
| `EvaluationRequirement::Base` | `src/eval/requirement/base/base.h` | Runtime requirement; `evaluate(target_data, instance, sector_layer)` returns a per-target result |
| `EvaluationData` / `EvaluationTargetData` | `src/eval/data/` | Per-UTN reference + test data used during evaluation |
| `EvaluationResultsGenerator` | `src/eval/results/evaluationresultsgenerator.cpp` | Drives the evaluation loops and result collection |
| `EvaluationRequirementResult::Single` / `Joined` | `src/eval/results/base/single.h`, `joined.h` | Per-target result / per-sector sum result |
| `EvaluationTaskResult` | `src/eval/results/evaluationtaskresult.h` | Stores the finished evaluation as a task result with report |
| `ResultReport::Report` | `src/task/result/report/report.h` | Generic report structure (sections, tables, figures, viewables) |

Intermediate base classes exist for common requirement kinds: `ProbabilityBase(Config)`, `IntervalBase(Config)`, `PositionBase(Config)` in `src/eval/requirement/base/`. Each config class has a matching `*ConfigWidget` for the GUI.

## How a standard is formed

Standards are pure configuration - no code change is needed to create or modify one.

- Everything lives in the `Configurable` tree persisted in `conf/default/eval.json` (per-user copy in `~/.compass/<version>/conf/default/eval.json`):
  - `EvaluationManager` > `EvaluationCalculator` > `EvaluationStandard` (one per standard) > `EvaluationRequirementGroup` (one per group, e.g. "Common", "Manoeuvring Area") > requirement config objects.
- Each requirement config object is one instance of a `*Config` class (e.g. `EvaluationRequirementDetectionConfig`) with parameters such as `name`, `short_name`, `comment` (cite the standard section here, e.g. "From EUROCAE ED-117 Section 3.3.5"), `use`, `prob`, `prob_check_type`, and type-specific thresholds.
- The map `Group::requirement_type_mapping_` in `src/eval/requirement/group.cpp` lists all available requirement config class names and their display names. Only classes in this map can be added to a group.
- At runtime, standards are managed via `EvaluationCalculator` (`addStandard()`, `copyCurrentStandard()`, `renameCurrentStandard()`, `deleteCurrentStandard()`) and edited in the standard tree GUI (`src/eval/standard/evaluationstandardtreemodel.h`).
- Which requirement group applies to which sector layer is selected per sector layer (`EvaluationCalculator::useGroupInSectorLayer()`).

To create a new standard: copy the closest existing one in the GUI, adjust the groups and requirement parameters, and cite the standard document sections in the `comment` fields. Alternatively add a new `EvaluationStandard` block in `eval.json` following the existing structure.

To add a new requirement type (code change): create a `*Config` class + widget in `src/eval/requirement/<topic>/`, a requirement class derived from `EvaluationRequirement::Base` (or `ProbabilityBase` / `IntervalBase` / `PositionBase`), result classes derived from `Single` and `Joined` in `src/eval/results/<topic>/`, and register the config class in `Group::requirement_type_mapping_` and the group's `generateSubConfigurable()`.

## How results are calculated

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
- `Joined::addToReport()` fills the per-sector overview tables (sector infos, condition, result value, passed/failed). `Single::addToReport()` fills per-target tables and per-target detail sections (headers/values via `targetTableHeaders*()` / `targetTableValues*()`, details via `detailHeaders()` / `detailValues()`).
- Results also provide viewables and annotations (grids, histograms, scatter series via `FeatureDefinition` classes in `src/eval/results/base/`) that render in Views when a result row is selected.
- Large content is loaded on demand (`loadOnDemandTable_impl()` etc. in `EvaluationTaskResult`) to keep the stored report small; single result details are purged after the joined results are updated.
- Export to PDF, LaTeX, or JSON goes through the generic report export (`--export_report ... --export_report_mode JSON`), not through evaluation-specific code.
