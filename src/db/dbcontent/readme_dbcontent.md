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
  - `description` - free-form semantic description of what the value means. A rewrite of the wire value, "Rescaled to covariance: rho * sigma_x * sigma_y.", is one sentence at its end.
  - `origin` - where the value comes from, one of:
    - `asterix` - decoded from one ASTERIX item. `data_item` names it in EUROCONTROL notation, `"I048/140.Time-of-Day"`, REF and SPF items as `"I021/REF.SGV.GSS"`. The item is the mapping key of the Variable in `task_import_asterix_cat<NNN>.json` with the category prefix, an edition variant with two mapping keys under one item, CAT021 `NUCp or NIC`, names the first. `scripts/check_asterix_mapping_coverage.py` fails on a `data_item` that is no mapping key of its Variable.
    - `calculated` - computed by COMPASS from other Variables of the same record, listed in `inputs`: `["Vx", "Vy"]`, `["Range", "Azimuth", "3D Height"]`, `["Time of Day*"]` for `Timestamp`. The `calculation` parameter names how, see "Calculated variables with an operator" under "Post-import derivations".
    - `internal` - bookkeeping, not from the data: `Line ID`, `Record Number`, `ARTAS Hash`, `Record Data`, the recording columns, `UTN`.
    - `selection` - chosen per record from an ordered candidate list, see "Selection variables" under "Post-import derivations". `selection` holds the candidates, `source_variable` names the companion UCHAR Variable with the 1-based index of the candidate that supplied the value.
    - empty on the 77 RefTraj Variables: the DBContent carries `"origin": "calculated"` for all of them, `Variable::origin()` falls back to it. A Report Variable answers `report`.
  - A derived Variable with a decoded namesake in the same content carries an asterisk in its stored name: `Latitude` is I062/105, `Latitude*` is the position COMPASS uses. A derived Variable without such a namesake keeps its plain name, `Timestamp`, `DS ID`, CAT062 `Mode C Code`. `Variable::originInfo()` gives the one-line summary the dialogs show, `info()` the tooltip, the Table View header and the variable selection button carry it.
  - `Variable::displayName(max_chars)` is the name shown where space is tight: the longest of item plus name (`I062/136 Flight Level Measured`), item plus short name (`I062/136 Meas FL`), name and short name that fits into `max_chars`, the shortest form when none fits, 0 means no limit, never truncated. Variables without a data item and MetaVariables have the forms name and short name only, `MetaVariable::shortName()` is the short name of its members when they agree. The stored name never changes, configurations reference it. Call sites: the Table View header shows `displayName(16)` (`BaseBufferTableModel::HeaderMaxChars`), its `VariableNameRole` carries the variable name for the clipboard copy, the Timestamp sort column and the UI state export. The `VariableSelectionWidget` button shows `displayName(24)` by default and keeps the selected names as members, `uiGet()` and `selectionAsString()` return the names, not the shown text; `setLabelMaxChars()` lowers the limit, the 3x3 label fields of the `LabelContentDialog` use 12 (`LabelFieldMaxChars`). Axis titles and the chart legends carry no variable names, the axis titles come from `ViewVariable::description()` without a limit.
  - `short_name` - at most 10 characters, a hard limit, none for a name of 12 characters or fewer, a soft threshold. Readable before cryptic (`Gnd Speed`, `Meas FL`, `Rec Num`), a mnemonic only when the specification and the users use it (`ACID`, `ACAD`, `MSSC`, `BPS`). One scheme across the contents, the shared abbreviation list is in section 3.4 of `readme_calc_dbcontent.md`: mode codes `MC`, `M2`, `M3A`, `M5`, the garbled, valid and smoothed flags as a suffix letter (`MC G`, `M3A V`), families by a suffix (`BPS ADD`, `BPS Age`, `M3A LM`, `WTC FPL`, `TI TOA`), the `Track` prefix of a flag dropped (`Confirmed`, `Ghost Tgt`). Unique within a DBContent, `check_asterix_mapping_coverage.py` fails on a duplicate or a longer one, `scripts/apply_short_names.py` applies a name to short name mapping to the repo and the user config copies. The `VariableEditDialog` refuses a longer one.
  - The `VariableEditDialog` edits `origin`, `data_item`, `inputs`, the candidate list of a selection (up, down, remove, add a Variable of the content) and `source_variable` in expert mode. A change of the candidates applies to the next import.
  - `calculation` - the operator of a calculated Variable, `{"op": "max"}`, an empty object when none. Every Variable has a real column, the import writes the value, so a buffer from the import (live mode) and a buffer read from the DB carry the same columns. There is no SQL expression on read (`db_expression` was removed, see `readme_calc_operators.md` in the repo root).

    All other selection Variables have a column, the import fills it.
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

- **ReportVariable / ReportContent** (`variable/reportvariable.h`) - one column of a Report Table written by the Evaluation or the Analyze Data Source task, offered in the variable selection under the report's own entry. Like the Meta content it has no rows of its own: `ReportVariable::getFor(dbcontent_name)` returns a runtime `Variable` for a host data content of the table, built outside the configuration tree (`Variable::createReportVariable()`, never written to `db_content_*.json`). The runtime Variable carries the join information (`reportTableName()`, `reportColumnName()`, `reportJoinAlias()`, `isReportVariable()`), its `name()` is `"<report name>: <column display name>"` and its `db_column_name` the SELECT alias. `DBContentManager::reportContents()` is the registry, filled by `TaskManager::updateReportContents()` at database open and after a report is saved or deleted. A Report Variable of a report the current database does not hold resolves to null, the name stays in the View configuration. Design and decisions: `experimental_src/readme_dynamic_dbcontent.md`.

`VariableSet` is the container used for read-set construction and for filter intersection; see [readme_loading.md](readme_loading.md). The Report Table join of the load path is described there as well.

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
      "origin": "asterix",
      "data_item": "I048/140.Time-of-Day",
      "inputs": [], "selection": [], "source_variable": "",
      "is_key": false
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

Most DBContent Variables are filled by direct `JSONDataMapping` from a single jASTERIX JSON key, origin `asterix`. The cases that are *not* a direct mapping fall into the groups below; the `origin`, `inputs` and `selection` parameters of the Variable say which one (see "Variable model" above).

### Synthesized columns (added to every record)

[`asterixpostprocess.cpp`](../../import/asterix/asterixpostprocess.cpp) `postProcessFlat`, before the `JSONDataMapping` step. Two columns are appended to every record's JSON dictionary:

| Column | Source |
|---|---|
| `category` | The ASTERIX category number, constant per buffer. |
| `line_id` | The import line/source identifier passed to the decoder, constant per buffer. |

### Decoder-provided columns

jASTERIX can add further flat top-level keys next to the decoded item keys. They are mapped like any other key, through a `JSONDataMapping` with the bare key name as `json_key`.

| Column | Variable | DBContents | Source |
|---|---|---|---|
| `artas_md5` | `ARTAS Hash` (STRING, group Origin) | CAT001, CAT010, CAT020, CAT021, CAT048 | jASTERIX `add_artas_md5_hash`, set in `ASTERIXImportTask`. MD5 over the record bytes, used for ARTAS association. |
| `record_data` | `Record Data` (STRING, group Origin), MetaVariable `Record Data` | CAT001, CAT010, CAT020, CAT021, CAT048, CAT062 | jASTERIX `add_record_data`, set in `ASTERIXImportTask`. The original ASTERIX record bytes as lowercase hex. |
| `recording_time` | `Recording Time` (DOUBLE, seconds since midnight, group Time), MetaVariable `Recording Time` | all 14 ASTERIX DBContents | jASTERIX recording time of the frame or packet: IOSS/RFF frame header, PCAP capture time (jASTERIX `recording_*` keys, see its readme). Null for raw/netto and network input, and for the COMPASS PCAP import, which passes plain buffers. |
| `recording_day` | `Recording Day` (UINT, group Time), MetaVariable `Recording Day` | all 14 ASTERIX DBContents | Day counter of the recording, 0 at its start. IOSS frame header, RFF computed. Null for PCAP. |
| `recording_date` | `Recording Date` (UINT `YYYYMMDD`, group Time), MetaVariable `Recording Date` | all 14 ASTERIX DBContents | UTC date of the frame or packet. RFF file header, PCAP capture time. Null for IOSS. |

`Record Data` costs about 45 bytes per record on disk. DuckDB compresses the hex text well, so it is only about a fifth larger than a raw binary column would be. RefTraj has no source record and does not carry the variable.

The two synthesized and the five decoder-provided keys are listed in `SYNTHESIZED_KEYS` of [`check_asterix_mapping_coverage.py`](../../../scripts/check_asterix_mapping_coverage.py), so the coverage checker does not treat them as fields outside the definition key space.

### In-place wire-value rewrites

The Variable still maps from a single ASTERIX item, but the value written into the buffer is no longer the raw decoded number. The last sentence of the `description` explains the rewrite.

| DBContent | Variable | Wire item | Rewrite |
|---|---|---|---|
| CAT020 | `X/Y StdDev Cov SDP` | `I020/500.SDP.rho-xy` | correlation `rho` -> covariance `rho * sigma_x * sigma_y` |
| CAT020 | `X/Y StdDev Cov PA` | `I020/REF.PA.SDC.COV-XY` | sign-square encoded -> `sign(v) * v^2` |
| CAT020 | `RUs` | `I020/400.Contributing Receivers` | per-record `[{RUx: byte}, ...]` 8-RU bitmaps -> array of 1-based receiver indices, written under `400.Contributing Receivers.RUx` |
| CAT062 | `XY StdDev Cov` | `I062/500.COV.COV (XY Covariance Component)` | sign-square encoded -> `sign(v) * v^2` |

### Selection variables (variable filled from one of several alternatives)

A Variable with origin `selection` carries an ordered candidate list in `selection`. A candidate is a Variable of the same DBContent, `{"variable": "Latitude HR"}`, or a named calculation, `{"calculation": "xy_projection_rs2g", "label": "X/Y projection RS2G"}`. A Variable candidate can require boolean Variables to be true or false, `"require_true": ["Mode C Valid LM"]`, `"require_false": ["Mode C Garbled LM"]`, a missing or null flag does not block it. The companion `source_variable`, an UCHAR Variable named `<name> Source`, holds the 1-based position of the candidate that supplied the value, null when none did. Its representation `SELECTION_SOURCE` prints the candidate label, the data item part of a Variable candidate (`I021/073`) or the explicit label of a calculation, so a Table View column reads `I021/073` and a filter accepts the label (`Variable::selectionSourceLabel()` and `selectionSourceIndex()`, resolved through `Variable::selectionOf()`).

[`selectionfiller.cpp`](variable/selectionfiller.cpp) `dbContent::SelectionFiller` fills them, called by `ASTERIXImportTask` on every decoded buffer before the timestamp calculation and the post-processing job: per row the first candidate with a value wins, its value is copied and its index written. Selections sharing a `source_variable` are chosen together, a candidate position counts only when every member has a value there, so `Latitude*` and `Longitude*` always come from the same item. A calculation candidate ends the search for a row, the code running that calculation fills the rows still null and sets their index, `ProjectionManager::doXYPositionCalculations` for the X/Y projection. A candidate of another data type or dimension than the selection is skipped with an error, a candidate with another unit of the same dimension is converted on copy with the factor of the `UnitManager` (the three CAT062 Mode C candidates are all feet, the conversion matters for future definitions). A string selection is copied like any other, `Aircraft Identification*` has the column `aircraft_identification_used`.

| DBContent | Variable | Candidates in order | Source variable |
|---|---|---|---|
| all 14 | `Time of Day*` (`Time of Day` in CAT004, CAT063 and CAT065, which decode `Time of Message` and have no namesake) | the decoded `Time of Day`; CAT021: `ToA Position` I021/071, `ToMR Position` I021/073, `ToRT` I021/077, `Time of Day` I021/030 | `Time of Day Source` |
| CAT021 | `Latitude*` / `Longitude*` | `Latitude` I021/130, `Latitude HR` I021/131 | `Position Source` |
| CAT010, CAT020, CAT062 | `Latitude*` / `Longitude*` | `Latitude` I0xx/041 or I062/105, calculation `xy_projection_rs2g` | `Position Source` |
| CAT062 | `Aircraft Identification*` | `Target Identification` I062/245, `Aircraft Identification` I062/380.ID, `Callsign FPL` I062/390.CSN, `Callsign Pre-Em FPL` I062/390.PEC | `Aircraft Identification Source` |
| CAT062 | `Mode C Code` | `Mode C Code LM` I062/340 when `Mode C Valid LM` and not `Mode C Garbled LM`, `Barometric Altitude Calculated` I062/135, `Flight Level Measured` I062/136 | `Mode C Code Source` |
| CAT020 | `X StdDev`, `Y StdDev`, `X/Y StdDev Cov`, `X DoP`, `Y DoP`, `X/Y DoP Cov`, `Geometric Height StdDev` | `<name> PA` from I020/REF.PA first, `<name> SDP`, `<name> DOP` or `<name> SDH` from the outdated I020/500 last | `<name> Source` |

The MetaVariables `Latitude`, `Longitude` and `Time of Day` point at the starred Variables, `Track Groundspeed` and `Track Angle` of CAT021 at `Ground Speed*` and `Track Angle*`, `Aircraft Identification` and `Mode C Code` of CAT062 at `Aircraft Identification*` and `Mode C Code`. The MetaVariables `Position Source` and `Time of Day Source` reach the source columns across contents.

Consumers of a selection, through the MetaVariable:

- The label generator, `TargetReportAccessor`, `TargetReport::Chain`, the evaluation and the reconstructors read the chosen value and carry no fallback chain of their own. The CAT062 altitude of all of them is the `Mode C Code` selection, the reconstructor's `trackedBarometricAltitude()` keeps reading I062/135 for the dubious track check.
- The flags of one decoded item, the MetaVariables `Mode C Valid` and `Mode C Garbled` on I062/340, do not describe a value the selection took from another candidate. A selection applied the flag conditions of its candidates, so a chosen value counts as valid and not garbled: `DBContentVariableLookup::metaVarIsSelection()`, reached through `BufferAccessor` and `DBContentAccessor`, tells the accessor, the chain and the label generator (`Variable::isSelection()`) to skip the flags. The evaluation `Mode C` requirements take this path through `Chain::modeC()`.
- A filter is a search: the ACID and Mode C filters match every variable candidate of the selection, `Variable::selectionCandidateVariables()` through `IDBVariableResolver::variableSelectionCandidateVariables()`, any candidate in range or matching the text, null when no candidate has a value. A content without a selection keeps the single column of the MetaVariable. The live mode `filterBuffer()` reads the candidate columns.

An edition variant is not a selection: CAT021 `NUCp or NIC` keeps two mappings, DO-260 `I021/090.NUCp or NIC` and DO-260A `I021/090.PA`, under one `data_item`.

### Calculated variables with an operator

[`calculationplan.cpp`](variable/calculationplan.cpp) `dbContent::CalculationPlan`, the design is in `readme_calc_operators.md` in the repo root. A Variable declares its calculation in `calculation`, `inputs` lists the input Variables in operator order. `"mode": "fill_null"` writes only rows where the output is null, the default `"overwrite"` writes all rows.

- `CalculationPlan::create()` reads the variable model on the main thread: it checks the operator, the inputs and their data types, resolves the unit factors from the `UnitManager` (`max`, `add`, `hypot` only) and sorts the calculations of a DBContent by their inputs. An error is logged, the Variable is skipped.
- `run()` touches buffer columns only. Every operator works on whole columns: the inputs are read once into value vectors through `NullableVector::ensureFullSize()`, `rawData()` and `rawNullFlags()`, NaN for null, the result is written in one loop. A missing input column means all rows null, an operator that then has no result adds no column (`ds_id` excepted).
- Call sites, each writer once before `DBContentManager::insertData()`: `ASTERIXPostprocessJob::run_impl()` first (ASTERIX and JSON import, the plan is built in `ASTERIXImportTask::run()` once per import run), `ReconstructorTask::writeDataSlice()` for RefTraj, `GPSTrailImportTask::run()`.

| Operator | Variable | DBContent(s) | Inputs | Null rule |
|---|---|---|---|---|
| `max` | `X/Y StdDev Max` | CAT010, CAT020, CAT062, RefTraj | `X StdDev`, `Y StdDev` | max of the inputs not null, as DuckDB `greatest` |
| `lookup` | `X/Y StdDev Max` | CAT021 | `NACp`, `NUCp or NIC`, `MOPS Version` | NACp table, else the NUCp table when MOPS Version is null or 0, else the NIC table, divided by 2.45 |
| `add` | `Sum Number Contributing Sensors` | CAT062 | `Num Contributing Sensors`, `Num Contributing Sensors Track Number` | null when one input is null |
| `array_length` | `Num Contributing Receivers` | CAT020 | `Contributing Receivers` | null when not an array |
| `ds_id` | `DS ID` | all 14 ASTERIX | `SAC`, `SIC` | `(0, 255)` with a warning when SAC or SIC is null |
| `hypot`, `fill_null` | `Ground Speed` | CAT010, CAT020, CAT062 | `Vx`, `Vy` | m/s to knots by the unit factor, the decoded CAT010 value is kept |
| `bearing_deg`, `fill_null` | `Track Angle` | CAT010, CAT020, CAT062 | `Vx`, `Vy` | `atan2(vx, vy)` in degrees, [0, 360) |

The `function` operator, `{"op": "function", "name": "timestamp"}`, only documents a hardcoded calculation, the plan skips it: `timestamp` (`Timestamp`), `radar_projection` (CAT001, CAT048 `Latitude*` / `Longitude*`), `adsb_sgv_speed_track` (CAT021 `Ground Speed*` / `Track Angle*`), `artas_tri_record_numbers` (CAT062 `TRI Record Numbers`, written later by the ARTAS association task). The RefTraj writers set `DS ID`, `Ground Speed` and `Track Angle` themselves, these have no `calculation`.

### Hardcoded calculations (not from any single ASTERIX item)

[`asterixpostprocessjob.cpp`](../../task/import/asterix/asterixpostprocessjob.cpp) `run_impl` runs the projection and CAT021 speed and track calculations once the buffer is built; [`asterixtimestampcalculator.cpp`](../../task/import/asterix/asterixtimestampcalculator.cpp) `doTimeStampCalculation` computes the wall-clock `Timestamp`.

| Variable | DBContent(s) | Inputs | Computation |
|---|---|---|---|
| `Latitude*` / `Longitude*` | CAT001, CAT048 (origin `calculated`, no decoded position exists) | `Range`, `Azimuth`, `3D Height` (or FFT lookup when missing), sensor coordinate system from `ds_id` | `ProjectionManager::calculateRadarPlotPositions` -> `Projection::polarSlantToWGS84` (RS2G / OGR / ...) |
| `Latitude*` / `Longitude*` | CAT010, CAT020, CAT062 (the `xy_projection_rs2g` candidate of the selection) | `X`, `Y`, sensor coordinate system from `ds_id` | `ProjectionManager::doXYPositionCalculations` -> `Projection::localXYToWGS84` for the rows the decoded item left null, `Position Source` set to the candidate index |
| `Ground Speed*` | CAT021 | `Ground Speed` I021/160, `SGV GSS`, `SGV STP` | the decoded value, else the SGV ground speed, else `0.0` when the SGV stopped bit is set |
| `Track Angle*` | CAT021 | `Track Angle` I021/160, `SGV HGT`, `SGV HTT`, `SGV HRD`, plus `Latitude*` / `Longitude*` / `Mode C Code` (for declination) | the decoded value, else the SGV ground track: when `HTT`=track and `HRD`=magnetic, add `ProjectionManager::declination(year, lat, lon, alt)`; otherwise pass through |
| `Time of Day` (the selection) | CAT063 | the selected value, plus first ToD seen in CAT062 and CAT063 | `tod += tod0_cat062 - tod0_cat063` to align CAT063 service messages with the corresponding CAT062 stream |
| `Time of Day*` | all categories | the selected value and the `override_tod_offset` setting | additive shift when `override_tod_active` is set, the decoded `Time of Day` stays unshifted |
| `Timestamp` (`ptime`) | all target-report DBContents | `Time of Day*`, resolved calendar date | `doTimeStampCalculation` adds the per-file resolved date, handling midnight wrap (`had_late_time_`) and time-jumps from previous-day offsets |

### Filtering and obfuscation

[`asterixpostprocessjob.cpp`](../../task/import/asterix/asterixpostprocessjob.cpp) also runs `doFilters` (drops records by ToD / position rectangle / circular range / Mode C window) and `doObfuscate` (rewrites Mode 3/A, ACAD, ACID with deterministic obfuscated values, drops military-reserved 3/A codes). These do not produce new variables; they remove rows or rewrite values of existing variables. The ACID obfuscation rewrites the column of the `Aircraft Identification` MetaVariable and every decoded candidate of a string selection over it, in CAT062 `Target Identification`, `Aircraft Identification`, `Callsign FPL` and `Callsign Pre-Em FPL`, and the copy in `Aircraft Identification*`.

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
- CAT021 fine-grained time items (`I021/071..077`) are mapped to separate Variables - `ToA Position`, `Time of Applicability for Velocity`, `ToMR Position`, `ToRT` - because for ADS-B they are not the same instant. `Time of Day` of CAT021 is the decoded I021/030, outdated and mostly absent.
- Every content has the decoded, unshifted `Time of Day` and the selection `Time of Day*` COMPASS uses (plain `Time of Day` in CAT004, CAT063 and CAT065, see "Selection variables"). The unified MetaVariable `Time of Day` resolves to the selection, `Time of Day Source` says which item supplied it. The override offset and the CAT063 alignment shift the selection only.

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
