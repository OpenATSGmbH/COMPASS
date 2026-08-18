# DBContent and database representation

This document describes how ASTERIX surveillance data lives inside COMPASS *after* it has been decoded - i.e. what the in-memory and on-disk representation looks like, how variables are organized, and what the JSON configuration in `conf/default/` actually configures. The upstream side (wire format, jASTERIX, JSON-to-Variable mapping, the import pipeline) is documented separately in [readme_asterix.md](../../task/import/asterix/readme_asterix.md); the read path (DB → Buffer for views and tasks) is in [readme_loading.md](readme_loading.md).

## End-to-end flow

```
    bytes ──► jASTERIX ──► JSON ──► ASTERIXJSONParser ──► Buffer ──► DuckDB table
                                       (JSONDataMapping)   (NullableVector<T>)   (data_cat<NNN>)
                                              │
                                              ▼
                                       DBContent variables
                                              │
                                              ▼
                                       MetaVariables  ◄── views, filters, eval, reconstructor
```

Two independent JSON layers in `conf/default/` drive this:

- The **import-side** mappings in `task_import_asterix*.json` say "this jASTERIX JSON key fills that DBContent variable" (one parser per category, many `JSONDataMapping` entries per parser).
- The **DB-side** schema in `db_content*.json` says "this DBContent has these variables, stored in this DuckDB column with this type and unit", plus the cross-category MetaVariable definitions.

The two layers meet on a *string*: the `db_content_variable_name` field of a JSONDataMapping must match the `name` of a Variable on the target DBContent. There is no compile-time check; a typo silently drops the column.

## DBContent

[`dbcontent.h`](dbcontent.h) / [`dbcontent.cpp`](dbcontent.cpp). One `DBContent` represents one surveillance category, e.g. `CAT048`, `CAT062`, `CAT021`, plus the synthetic `RefTraj` for reconstructed reference trajectories used in evaluation.

Identity and role:
- `name()`, `id()`, `dbTableName()` - string name (`"CAT048"`), numeric id (48), DuckDB table (`data_cat048`).
- `containsTargetReports()` - true for the categories that carry per-target plots (010, 020, 021, 048, 062, RefTraj). False for status / service categories (019, 023, 025, 034, 063, 065).
- `containsStatusContent()` - the inverse role for service messages.
- `isReferenceContent()` - true only for `RefTraj`.

State flags:
- `loadable()` - the configured DBContent's table is present in the open database. False before a DB is opened, false for newly-defined contents on an old DB. Gates whether the manager will attempt to read from it.
- `loadingWanted()` - runtime opt-in flag, toggled by the UI / by `LoadRequest`. A content can be loadable but not currently wanted (user hid it).
- `existsInDB()` - true if the table actually contains rows.
- `hasData()` / `count()` / `loadedCount()` - row counts in the DB versus in the currently loaded buffer.

Lifecycle methods:
- `prepareInsert`, `updateDataSourcesBeforeInsert`, `finalizeInsert` - three-phase insert path. The middle one auto-creates `DataSource` rows for any (SAC, SIC) seen for the first time.
- `deleteDBContentData(...)` - overloaded by SAC/SIC and optional line.
- `quitLoading()` - cancels an in-flight read, used during shutdown and before issuing a new load.

DBContents are constructed from `Configurable` subconfigs at startup (see "Configuration" below); the `DBContentManager` is the only owner.

## DBContentManager

[`dbcontentmanager.h`](dbcontentmanager.h) / [`dbcontentmanager.cpp`](dbcontentmanager.cpp). Owns the collection of `DBContent`s, the `DBContentDataStore`, the `LabelGenerator`, and the cross-cutting MetaVariable registry.

Key signals (Qt):
- `loadingStartedSignal()`, `loadingDoneSignal()` - bracket every load.
- `loadedDataSignal(map<name, Buffer>, requires_reset)` - fired per-content as each `DBContentReadDBJob` completes; this is what views consume.
- `insertDoneSignal()`, `dataDeletedSignal()` - mirror inserts and deletions.
- `dbObjectsChangedSignal()` - DBContents added/removed/reconfigured.
- `dataSourcesChangedSignal()` - coalesced after an insert that created new sources.

The read path is in [readme_loading.md](readme_loading.md). The insert path is the inverse: each `DBContent::prepareInsert` validates the buffer against its variables, `updateDataSourcesBeforeInsert` registers new (SAC, SIC) tuples in the active context, then `finalizeInsert` hands the buffer to the DB worker.

## Variable model

[`variable/`](variable/). Two layers:

- **Variable** (`variable/variable.h`) - one column on one DBContent. Holds:
  - `name` - human/UI name (e.g. `"Time of Day"`).
  - `db_column_name` - DuckDB column (e.g. `"tod"`).
  - `data_type_str` - `BOOL`, `CHAR`, `UCHAR`, `INT`, `UINT`, `LONGINT`, `ULONGINT`, `FLOAT`, `DOUBLE`, `STRING`, `JSON`, `TIMESTAMP`. Maps directly to `PropertyDataType` and to the DuckDB column type.
  - `dimension` + `unit` - physical units (`Time`/`Second`, `Length`/`Meter`, `Angle`/`Degree`, ...). Used for unit conversion when crossing variables and for display.
  - `representation_str` - display formatter (`STANDARD`, `SECONDS_TO_TIME`, `DEC_TO_OCTAL`, `FLOAT_PREC2`, ...).
  - `description` - free-form semantic description of what the value means. Should *not* repeat the origin (that goes in `source`).
  - `source` - documents where the value comes from. One of these shapes (multi-line shapes use a literal `\n` between alternatives):
    - **ASTERIX item** in EUROCONTROL notation: `"I048/140.Time-of-Day"`.
    - **ASTERIX item with rewrite note** when the buffer value is not the raw decoded number: `"I020/500.SDP.rho-xy (rescaled to covariance: rho * sigma_x * sigma_y)"`. See "Post-import derivations" below.
    - **Multi-source / fallback chain** in priority order, `\n`-separated: `"I021/071.Time of Applicability for Position\nI021/073.Time of Message Reception for Position\nI021/077.Time of Report Transmission\nI021/030.Time of Day (deprecated)"`.
    - **Calculated from other variables**: `"Calculated from Vx, Vy"`, `"Calculated from Range, Azimuth, 3D Height (radar polar->WGS-84 projection)"`, `"Calculated from Time of Day, calendar date"`. Names other DBContent variables on the same record, since the computation reads the buffer columns by then.
    - **Internal**: `"Internal: ASTERIX category number"`, `"Internal: import line/source identifier"`, `"Calculated from I048/010.SAC, I048/010.SIC"` (ds_id).
    - **Empty** for variables with no canonical source (e.g. `Unique Target Number`, `Ascending record number`).
  - `db_expression` - optional SQL expression for *computed* columns (the column doesn't physically exist; SQL evaluates it on read).
  - `is_key` - primary-key flag.
  - `group` - semantic group used by the variable selection dialog's "Group" mode. One of: `Origin`, `Type`, `Time`, `Position`, `Accuracy`, `Movement`, `Secondary Identification`, `Altitude`, `Track`, `Ages`, `Warning/Alert`; empty falls into the `Other` bucket. Initially assigned by [`scripts/assign_variable_groups.py`](../../../scripts/assign_variable_groups.py) (name heuristics + per-item defaults + explicit overrides); hand-editable per variable. `MetaVariable` carries the same parameter.
- **MetaVariable** (`variable/metavariable.h`) - the unified concept across DBContents. Maps a single conceptual name (`"Time of Day"`, `"Aircraft Address"`, `"Position Latitude"`) to per-DBContent Variable names. Example from [`db_content.json`](../../../conf/default/db_content.json):
  ```json
  {
    "class_name": "MetaVariable",
    "parameters": {
      "name": "Time of Day",
      "dbcont_variables": {
        "CAT010": "Time of Day", "CAT020": "Time of Day",
        "CAT021": "Time of Applicability for Position",
        "CAT048": "Time of Day", "CAT062": "Time of Track Information",
        "RefTraj": "Time of Day"
      }
    }
  }
  ```
  Views, filters, evaluation requirements and the reconstructor work on MetaVariables - they never name a per-category variable directly. This is what the closing paragraph of `readme_asterix.md`'s "Information hierarchy" section is referring to: the layered ASTERIX item numbers vanish here; consumers see one unified surveillance variable regardless of which category supplied the report.

`VariableSet` is the container used for read-set construction and for filter intersection; see [readme_loading.md](readme_loading.md).

## Configuration in `conf/default/`

All schema is JSON; nothing is hard-coded. There are two parallel families of files:

### DB-side (DBContent definitions)

- [`db_content.json`](../../../conf/default/db_content.json) - `DBContentManager` config. Lists `sub_config_files` (one per DBContent) and carries all MetaVariable definitions as `sub_configs`. Adding a new DBContent means adding a file here and to that list.
- `db_content_cat<NNN>.json` - one per category. Top-level `class_name = "DBContent"` with parameters `name`, `id`, `db_table_name`, `contains_target_reports`, `contains_status_content`. `sub_configs` is a list of `Variable` blocks, e.g.
  ```json
  {
    "class_name": "Variable",
    "parameters": {
      "name": "Time of Day",
      "db_column_name": "tod",
      "data_type_str": "DOUBLE",
      "dimension": "Time", "unit": "Second",
      "representation_str": "SECONDS_TO_TIME",
      "description": "Absolute time stamping expressed as Co-ordinated Universal Time (UTC)",
      "source": "I048/140.Time-of-Day",
      "is_key": false,
      "db_expression": ""
    }
  }
  ```
- [`db_content_reftraj.json`](../../../conf/default/db_content_reftraj.json) - the synthetic reference trajectory DBContent used by evaluation. Schema-wise it looks like CAT062 (positions, velocities, accelerations) but is filled by the reconstructor, not by an importer. Besides the contributing-source ages (`Contributing Radar/Tracker/RefTraj/Other Age`, `Contributing Sources`/`Number`, the content-based `Primary`/`Mode A/C Update Age`), it carries CAT062-tracker-style analysis columns filled per reference update: I062/290-style technology update ages (`Track Age`, `PSR Update Age`, `SSR Update Age`, `Mode S Update Age` - zeroed by radar reports whose detection contains the respective component: the "Type" detection type when transmitted, content-based otherwise; a combined plot refreshes PSR and SSR alike - plus `ADS-B ES Update Age` and `MLAT Update Age` zeroed by their source types), I062/295-style data item ages (`Mode 3/A Age`, `Flight Level Measured Age`, `Aircraft Identification Age`, `Aircraft Address Age`), and `Associated Record Numbers` (flat JSON list of associated target report record numbers, the RefTraj counterpart of CAT062's `TRI Record Numbers`). The reconstructor also fills `Vx StdDev`/`Vy StdDev` from the Kalman smoother covariance, `Coasting` (set when a reference update had no associated target reports), and `Num Contributing Sensors` (distinct data sources among the associated reports, bridged to CAT062's `Sum Number Contributing Sensors` via the `Num Contributing Sensors` MetaVariable). The CAT062-analog variables deliberately use the CAT062 names and columns, and the shared MetaVariables (`Track Age`, `PSR/SSR/Mode S/MLAT/ADS-B ES Update Age`, `Mode 3/A Age`, `Flight Level Measured Age`, `Associated Record Numbers`) line them up so Views show operational tracker and reconstruction in the same column.

The DuckDB table for each DBContent is created on demand from its Variable list: column name = `db_column_name`, column type = the DuckDB type for `data_type_str`. Adding/removing a Variable in the JSON changes the table schema on the next DB open.

### Import-side (ASTERIX → DBContent mappings)

- [`task_import_asterix.json`](../../../conf/default/task_import_asterix.json) - `ASTERIXImportTask` master config:
  - `current_file_framing` (`ioss` / `ioss_seq` / `rff` / empty) - see "Framings" in `readme_asterix.md`.
  - `chunk_size_jasterix`, `chunk_size_insert` - per-stage batching (records per chunk).
  - `num_packets_overload`, `max_packets_in_processing`, `limit_ram` - backpressure for large files / sustained live feeds.
  - `filter_tod_min/max`, `filter_modec_min/max`, `filter_latitude_*` / `filter_longitude_*`, `filter_circ_*`, `filter_rec_*` - wire-time filters that drop records before insertion.
  - `override_sac_org/new`, `override_sic_org/new` - remap one (SAC, SIC) to another at import (see SAC/SIC below).
  - `override_tod_offset`, `reset_date_between_files` - see Time of Day below.
  - `sub_configs[ASTERIXCategoryConfig]` - one per category, picks the active edition / REF / SPF for jASTERIX.
- `task_import_asterix_cat<NNN>.json` - one `ASTERIXJSONParser` per category. Top-level parameters bind it to a DBContent (`db_content_name`, `category`); `sub_configs` is the list of `JSONDataMapping` entries:
  ```json
  {
    "class_name": "JSONDataMapping",
    "parameters": {
      "json_key": "140.Time-of-Day",
      "db_content_name": "CAT048",
      "db_content_variable_name": "Time of Day",
      "dimension": "Time", "unit": "Second",
      "format_data_type": "",
      "active": true,
      "mandatory": false,
      "in_array": false,
      "append_value": false
    }
  }
  ```
  - `json_key` - dotted path into the jASTERIX JSON record (`"140.Time-of-Day"`, `"010.SAC"`, `"170.Target Identification.TId"`, ...).
  - `db_content_variable_name` - must match a `Variable.name` on the target DBContent.
  - `dimension` / `unit` - input units; if they differ from the Variable's units, an automatic conversion runs.
  - `format_data_type` - optional source-side type cast.
  - `active`, `mandatory`, `in_array`, `append_value` - control whether the mapping fires, whether nulls are allowed, and how multi-valued JSON keys (lists) are flattened.

## Buffer and DuckDB tables

[`src/core/buffer/`](../../core/buffer/). A `Buffer` is the in-memory columnar representation of a slice of a DBContent: one `NullableVector<T>` per column, each row addressable across columns by index. Columns are added by `addProperty(id, type)` and accessed by `get<T>(id)`.

`PropertyDataType` (`property.h`) → DuckDB column type:

| `data_type_str` | `PropertyDataType` | DuckDB column type |
|---|---|---|
| `BOOL` | BOOL | `BOOLEAN` |
| `CHAR` / `UCHAR` | CHAR / UCHAR | `TINYINT` / `UTINYINT` |
| `INT` / `UINT` | INT / UINT | `INTEGER` / `UINTEGER` |
| `LONGINT` / `ULONGINT` | LONGINT / ULONGINT | `BIGINT` / `UBIGINT` |
| `FLOAT` / `DOUBLE` | FLOAT / DOUBLE | `FLOAT` / `DOUBLE` |
| `STRING` | STRING | `VARCHAR` |
| `JSON` | JSON | `VARCHAR` (JSON-as-text) |
| `TIMESTAMP` | TIMESTAMP | `TIMESTAMP` |

Per-DBContent target-report tables (`data_cat010`, `data_cat020`, `data_cat021`, `data_cat048`, `data_cat062`, `reftraj`, ...) carry one column per Variable. Implicit always-present columns: a primary-key record number, plus `ds_id` and `line_id` (and a timestamp / `tod` column for target-report contents). Status-content tables (e.g. `data_cat034`) follow the same pattern with their own variable lists.

Other DB tables (no DBContent backing them):
- `targets` - one row per unified track (UTN), with the Target's metadata as JSON.
- `sectors`, `viewpoints` - airspace and annotation persistence.
- `properties` - generic key/value store used by misc. managers.
- Plus jASTERIX/import bookkeeping tables.

## Post-import derivations

Most DBContent Variables are filled by direct `JSONDataMapping` from a single jASTERIX JSON key. The cases that are *not* a direct mapping fall into four groups; all of them are documented in the per-Variable `source` field (see "Variable model" above).

### Synthesized columns (added to every record)

[`asterixpostprocess.cpp`](../../import/asterix/asterixpostprocess.cpp) `postProcessFlat`, before the `JSONDataMapping` step. Three columns are appended to every record's JSON dictionary:

| Column | Source |
|---|---|
| `category` | The ASTERIX category number, constant per buffer. |
| `line_id` | The import line/source identifier passed to the decoder, constant per buffer. |
| `ds_id` | Bit-packed `(SAC, SIC)` from `010.SAC` / `010.SIC` (`Number::dsIdFrom`). Falls back to `(0, 255)` with a warning when SAC or SIC is absent. |

### In-place wire-value rewrites

The Variable still maps from a single ASTERIX item, but the value written into the buffer is no longer the raw decoded number. The `source` field includes a parenthetical note explaining the rewrite.

| DBContent | Variable | Wire item | Rewrite |
|---|---|---|---|
| CAT020 | `X/Y StdDev Cov` | `I020/500.SDP.rho-xy` | correlation `rho` -> covariance `rho * sigma_x * sigma_y` |
| CAT020 | `X/Y StdDev Cov` (REF variant) | `I020/REF.PA.SDC.COV-XY` | sign-square encoded -> `sign(v) * v^2` |
| CAT020 | `RUs` | `I020/400.Contributing Receivers` | per-record `[{RUx: byte}, ...]` 8-RU bitmaps -> array of 1-based receiver indices, written under `400.Contributing Receivers.RUx` |
| CAT062 | `XY StdDev Cov` | `I062/500.COV.COV (XY Covariance Component)` | sign-square encoded -> `sign(v) * v^2` |

### Fallback chains (variable filled from one of several alternatives)

Listed in priority order; the first non-null source wins.

| DBContent | Variable | Where | Sources tried |
|---|---|---|---|
| CAT021 | `Time of Day` | [`asterixtimestampcalculator.cpp`](../../task/import/asterix/asterixtimestampcalculator.cpp) `doADSBTimeProcessing` | `I021/071.Time of Applicability for Position`, then `I021/073.Time of Message Reception for Position`, then `I021/077.Time of Report Transmission`, then `I021/030.Time of Day (deprecated)` |
| CAT021 | `Latitude` / `Longitude` | [`asterixpostprocessjob.cpp`](../../task/import/asterix/asterixpostprocessjob.cpp) `doADSBPositionProcessing` | normal `I021/130` low-resolution position; if null, the `Latitude HR` / `Longitude HR` from `I021/131` high-resolution position is copied in |
| CAT020 / CAT021 | various position-accuracy variables | `task_import_asterix_cat020.json`, `task_import_asterix_cat021.json` | multiple `JSONDataMapping` entries with the same `db_content_variable_name` (legacy `I020/500` *or* `I020/REF.PA`; DO-260 `I021/090.NUCp or NIC` *or* DO-260A `I021/090.PA`) |

### Computed variables (not from any single ASTERIX item)

[`asterixpostprocessjob.cpp`](../../task/import/asterix/asterixpostprocessjob.cpp) `run_impl` runs the projection and velocity calculations once the buffer is built; [`asterixtimestampcalculator.cpp`](../../task/import/asterix/asterixtimestampcalculator.cpp) `doTimeStampCalculation` computes the wall-clock `Timestamp`.

| Variable | DBContent(s) | Inputs | Computation |
|---|---|---|---|
| `Latitude` / `Longitude` | CAT001, CAT010, CAT048 | `Range`, `Azimuth`, `3D Height` (or FFT lookup when missing), sensor coordinate system from `ds_id` | `ProjectionManager::calculateRadarPlotPositions` -> `Projection::polarSlantToWGS84` (RS2G / OGR / ...) |
| `Latitude` / `Longitude` | CAT010, CAT020, CAT062 | `X`, `Y`, sensor coordinate system from `ds_id` | `ProjectionManager::doXYPositionCalculations` -> `Projection::localXYToWGS84` |
| `Ground Speed` | any DBContent with vx/vy | `Vx`, `Vy` | `sqrt(vx^2 + vy^2)`, then m/s -> knots |
| `Track Angle` | any DBContent with vx/vy | `Vx`, `Vy` | `atan2(vx, vy)` -> radians -> degrees, normalised to [0, 360) |
| `Ground Speed` | CAT021 (when no vx/vy) | `SGV GSS` (I021/160), or `0.0` when `SGV STP` stopped bit is set | direct copy / hard zero |
| `Track Angle` | CAT021 (when no vx/vy) | `SGV HGT`, `SGV HTT`, `SGV HRD`, plus `Latitude` / `Longitude` / `Mode C` (for declination) | when `HTT`=track and `HRD`=magnetic, add `ProjectionManager::declination(year, lat, lon, alt)`; otherwise pass through |
| `Time of Day` | CAT063 | `Time of Day` from CAT063 record, plus first ToD seen in CAT062 and CAT063 | `tod += tod0_cat062 - tod0_cat063` to align CAT063 service messages with the corresponding CAT062 stream |
| `Time of Day` | all categories | `Time of Day` and `override_tod_offset` setting | additive shift when `override_tod_active` is set |
| `Timestamp` (`ptime`) | all target-report DBContents | `Time of Day`, resolved calendar date | `doTimeStampCalculation` adds the per-file resolved date, handling midnight wrap (`had_late_time_`) and time-jumps from previous-day offsets |

### Filtering and obfuscation

[`asterixpostprocessjob.cpp`](../../task/import/asterix/asterixpostprocessjob.cpp) also runs `doFilters` (drops records by ToD / position rectangle / circular range / Mode C window) and `doObfuscate` (rewrites Mode 3/A, ACAD, ACID with deterministic obfuscated values, drops military-reserved 3/A codes). These do not produce new variables; they remove rows or rewrite values of existing variables.

## DBContentDataStore

[`dbcontentdatastore.h`](dbcontentdatastore.h). The cache of *currently loaded* buffers, plus the indexing structures needed by views.

- `buffers_` - current `BufferMap` (DBContent id → `Buffer`).
- `accessor_` - typed-field access helper (`DBContentAccessor`).
- `indices_` - three-level index: `dbc_id → ds_id → line_id → row indices`. Lets a view say "give me all CAT048 rows from radar X line 2" without scanning the buffer.
- `update(changed_dbc_names)` - incremental refresh after `addLoadedData`, an insert, or a delete; only rebuilds for the changed DBContents.

The store is rebuilt rather than mutated on every load - there is exactly one logical "current dataset" at any time across all views.

## Targets

[`target/`](target/). A `Target` (UTN - Unique Target Number) is the unified track object built by aggregating target-report rows from across DBContents. The `Target` lives in the DB (`targets` table) plus an in-memory cache; the per-target rows it points into stay in their original `data_cat<NNN>` tables, addressed by `(dbc_id, rec_num)`.

- `Target` / `TargetBase` (`target/target.h`, `target/targetbase.h`) - UTN identity, evaluation-time metadata (`useInEval`, `evalExcludedTimeWindows`, `evalExcludedRequirements`, `comment`), per-DBContent counts, ADS-B specifics (`TargetADSBInfo`).
- `TargetReportAccessor` - typed read-only access to one target's rows in a specific DBContent's buffer.
- `TargetListWidget` - GUI list, owned by `DBContentManager`.

Targets are *created and updated by the reconstructor*, not by the importer (see [`reconstruction`](../../reconstruction/) and the `reconstruction` skill).

## Labels

[`label/`](label/). `LabelGenerator`, owned by `DBContentManager`, formats the on-screen text for a given `(DBContent, buffer row)`. Uses each Variable's `representation_str` to format values (`SECONDS_TO_TIME` for `tod`, `DEC_TO_OCTAL` for Mode 3/A, ...). Knobs: `autoLabel()`, `currentLOD()`, `labelDistance()`.

## Time of Day handling

ToD is the most error-prone field in real recordings. ASTERIX encodes it as a 24 h truncated value (3 octets, LSB 1/128 s in CAT048/020/010/021, and `Time of Track Information` in CAT062 with the same wrap behavior); recordings routinely cross midnight; sensor clocks drift; CAT021 splits time into per-item *applicabilities* (positions, velocities, reception, transmission).

- [`asterixtimestampcalculator.h`](../../task/import/asterix/asterixtimestampcalculator.h) / `.cpp` - per-file pass that tracks first/last ToD, detects midnight wrap (`had_late_time_`), assigns the calendar date, and increments it across wraps. Multiple wraps within one file are handled.
- `override_tod_offset` (in `task_import_asterix.json`) - additive offset applied before wrap accounting. Used to correct sensor clock skew that has been characterised in QA.
- `reset_date_between_files` - whether wrap accounting starts fresh per file or carries over (matters for chunked recordings of a single recording session).
- `filter_tod_min/max` - wire-time drop filter (default `0` to `86399`).
- CAT021 fine-grained time items (`I021/071..077`) are mapped to separate Variables - `Time of Applicability for Position`, `Time of Applicability for Velocity`, `Time of Message Reception for Position`, `Time of ASTERIX Report Transmission` - because for ADS-B they are not the same instant.
- The unified MetaVariable `Time of Day` resolves to the appropriate per-category variable (CAT021 uses the position-applicability time).

If `tod` is wrong, *everything* downstream - association, evaluation requirements, reconstruction, view selection - is wrong. This is the first thing to check when diagnosing weird import behavior.

## SAC/SIC and DataSources

A (SAC, SIC) tuple uniquely names the sensor that produced a record. COMPASS treats it as the primary key of a `DataSource`.

- [`data_source.h`](../../core/context/data_source.h) - `DataSource` carries `sac`, `sic`, derived `id()`, name and short name, `dsType` (`PSR`, `SSR`, `Mode S`, `MLAT`, `ADSB`, `Tracker`, ...), reference position (lat/lon/alt), update interval, line list, and a free-form info JSON.
- The owner is now the **`DBContextManager`** (`src/core/context/db_context_manager.h`) - it absorbed the responsibilities of the former `DataSourceManager` (and `FFTManager`). Lookup by `dataSource(ds_id)` or `dataSource(name)`.
- Every loaded buffer row carries a `ds_id` column, which is the bit-packed SAC/SIC. The `DBContentDataStore` indexes by `ds_id` (and `line_id`, for multi-line radars) so that a view can slice by source without re-querying the DB.
- Auto-registration: when an import sees a (SAC, SIC) that is not yet in the active context, `DBContent::updateDataSourcesBeforeInsert` creates the DataSource. The first record from a new sensor therefore "names" it; the user can then rename / classify it in the data-source UI.
- **Override at import** (`task_import_asterix.json`):
  - `override_sac_org` + `override_sic_org` - the (SAC, SIC) to remap.
  - `override_sac_new` + `override_sic_new` - what to remap it to.
  Used to merge legacy/test recordings whose IDs collide with production sources, or to rebrand a sensor that changed identity. Applied in `ASTERIXPostProcessJob` before the buffer is handed to the DB.

See the [`data_context`](../../core/context/readme_context.md) skill / readme for the full DBContext model.

## See also

- [readme_asterix.md](../../task/import/asterix/readme_asterix.md) - the upstream side: ASTERIX wire format, jASTERIX, framings/REFs/SPFs, the import pipeline.
- [readme_loading.md](readme_loading.md) - the read path in detail: `LoadRequest`, fan-out, per-content jobs, the two parallel observers.
- [readme_context.md](../../core/context/readme_context.md) - DBContext, data sources, FFTs, sectors.
