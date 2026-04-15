# DBContext System

The context system groups sensor definitions, FFTs, ASTERIX decoding configs, and sectors into named configuration contexts stored at `~/.compass/data_contexts/`. It replaces the old `DataSourceManager` and `FFTManager`.

All classes live in `namespace context`.

## File overview

| File | Purpose |
|------|---------|
| `db_context.h/.cpp` | `DBContext` data container |
| `db_context_manager.h/.cpp` | `DBContextManager` orchestrator (owned by COMPASS) |
| `db_context_serializer.h/.cpp` | File I/O, schema versioning |
| `db_context_diff.h/.cpp` | Field-level context comparison |
| `data_source.h/.cpp` | `DataSource` sensor definition |
| `fft.h/.cpp` | `FFT` fixed frequency transmitter |
| `fft_edit_widget.h/.cpp` | Widget to edit a single FFT |
| `asterix_decoding_config.h/.cpp` | `ASTERIXDecodingConfig` ASTERIX category settings |
| `idatasourceprovider.h` | `IDataSourceProvider` lightweight query interface |
| `db_context_edit_dialog.h/.cpp` | Main edit dialog (tree + detail panels) |
| `db_context_edit_tree_item.h` | Tree item classes for the edit dialog |
| `db_context_edit_tree_model.h/.cpp` | `QAbstractItemModel` for edit tree |
| `db_context_create_dialog.h/.cpp` | Create new context |
| `db_context_select_dialog.h/.cpp` | Select existing context |
| `db_context_rename_dialog.h/.cpp` | Rename context |
| `db_context_delete_dialog.h/.cpp` | Delete contexts (multi-select) |
| `db_context_copy_dialog.h/.cpp` | Duplicate a context |

Related files outside this directory:
- `src/core/sector/sector_edit_widget.h/.cpp` -- widget to edit a single sector
- `src/core/sector/sector_import_utils.h/.cpp` -- GDAL polygon import utility

## Architecture

- **NOT Configurable** -- uses own file-based persistence, not the `Configurable` JSON system.
- **Owned by COMPASS** -- created in `COMPASS` constructor, accessed via `compass.dbContextManager()`.
- **One active context at a time** -- all queries (data sources, FFTs, sectors) operate on the active context.
- **Qt signals** for change notification: `activeContextChangedSignal()`, `contextsChangedSignal()`, `sectorsChangedSignal()`.

## DBContext (data container)

Groups four data sections plus metadata:

```cpp
class DBContext {
    std::string name_, description_, created_, modified_;  // metadata
    std::vector<DataSource> data_sources_;
    std::vector<FFT> ffts_;
    std::vector<ASTERIXDecodingConfig> asterix_decoding_;
    std::vector<std::shared_ptr<Sector>> sectors_;
};
```

**Name invariants:** Context names must be non-empty and unique within the manager. Empty names are never valid — `DBContextManager` asserts `!name.empty()` on `createContext`, `renameContext`, `duplicateContext`, `setActiveContext`, and `importContext`. The `db_context` table is only created in the DB when first writing a context (not eagerly on DB creation), so `databaseOpenedSlot` can distinguish a new DB (no table) from an existing one.

- `toJSON()` / `fromJSON()` serialize the full context as one JSON object.
- `operator==` compares all fields.
- `currentTimestamp()` returns ISO 8601 string for created/modified fields.

## DataSource (sensor)

Identified by SAC/SIC. ID computed via `Utils::Number::dsIdFrom(sac, sic)`.

**SAC/SIC uniqueness invariant:** Each data source within a context must have a unique SAC/SIC pair. Same SAC/SIC always means the same data source. This is enforced at all entry points:
- **Import/commands:** `DBContext::addOrReplaceDataSource()` overwrites an existing entry with the same SAC/SIC instead of creating a duplicate.
- **File/DB loading:** `DBContextSerializer::load()` and `DBContextManager::readContextFromDB()` detect duplicates, log an error, and keep only the first occurrence.

Never use `dataSources().push_back()` directly when adding from external input — always use `addOrReplaceDataSource()`.

```cpp
class DataSource {
    std::string ds_type_;     // "Radar", "MLAT", "ADSB", "Tracker", "RefTraj", "Other"
    unsigned int sac_, sic_;
    std::string name_;
    bool has_short_name_; std::string short_name_;
    nlohmann::json info_;     // flexible metadata (position, detection_type, radar_range, etc.)
};
```

Key `info_` sub-fields accessed via convenience methods:
- **Position**: `latitude()`, `longitude()`, `altitude()`, `hasPosition()`
- **Detection**: `groundOnly()`, `detectionTypeInt()`, `probabilityOfDetection()`, `clutterRate()`
- **Radar-specific**: `radarRanges()`, `radarAccuracies()`, `ignoreRadarAzmRange()`
- **Network**: `hasNetworkLines()`, `addNetworkLinesIfMissing()`
- **Remote units**: `hasRemoteUnits()`, `remoteUnitName(int)`
- **Update timing**: `updateInterval()`, `hasUpdateInterval()`

Serialized via `toJSON()` / `fromJSON()`.

## FFT (fixed frequency transmitter)

```cpp
class FFT {
    std::string name_;
    nlohmann::json info_;  // latitude, longitude, altitude, mode_s_address, mode_3a_code, mode_c_code
};
```

Position via `latitude()`, `longitude()`, `altitude()`. Codes stored directly in `info_`.

FFT detection (`DBContextManager::isFromFFT`): checks Mode S address, Mode 3/A code, Mode C altitude, and position proximity (< 5000 m). Mode 3/A code 7777 (octal 4095) always treated as FFT.

## ASTERIXDecodingConfig

```cpp
class ASTERIXDecodingConfig {
    unsigned int category_;  // ASTERIX category (e.g. 21, 48)
    std::string edition_;    // e.g. "1.31"
    std::string ref_;        // REF version
    std::string spf_;        // SPF version
};
```

## Sectors

Sector objects live in `src/core/sector/`. Stored as `shared_ptr<Sector>` in DBContext. Each sector has: `id`, `name`, `layer_name`, polygon points, `exclude` flag, `color`, optional altitude bounds.

`DBContextManager` caches sectors as `SectorLayer` objects (grouped by `layer_name`), rebuilt on context change via `rebuildSectorLayers()`.

## File persistence

### Storage layout

```
~/.compass/data_contexts/
  active_context.json              # {"active_context": "name"}
  context_name/
    context_meta.json              # name, description, created, modified
    data_sources.json              # {"version":"1.0","content_type":"data_sources","data":[...]}
    ffts.json                      # {"version":"1.0","content_type":"ffts","data":[...]}
    asterix_decoding.json          # {"version":"1.0","content_type":"asterix_decoding","data":[...]}
    sectors.json                   # {"version":"1.0","content_type":"sectors","data":[...]}
```

### DBContextSerializer

Static methods only:
- `save(ctx, base_path)` -- write all section files to `base_path/<name>/`
- `load(context_dir)` -- read all sections from a context directory
- `listContexts(base_path)` -- list context directory names
- `contextExists()`, `deleteContext()`, `renameContext()` -- filesystem ops

Version field (`"1.0"`) allows future schema upgrades.

## Import / export

`DBContextManager` provides both per-section and full-context import/export:

```cpp
// Per-section (active context, legacy JSON format)
void importSensors(const std::string& filepath);
void importFFTs(const std::string& filepath);
void importSectors(const std::string& filepath);
void exportSensors(const std::string& filepath);
void exportFFTs(const std::string& filepath);
void exportSectors(const std::string& filepath);

// Full context
void exportContext(const std::string& name, const std::string& filepath);
void importContext(const std::string& filepath);  // creates new context from file
```

GDAL-based sector import: `sector_utils::parseGDALFile(filepath)` returns `vector<ImportedSector>` with name and lat/lon polygon points.

## Database sync

On DB open (`databaseOpenedSlot`):
1. If no `db_context` table: write active context to DB.
2. If table exists: compare file vs DB context via `DBContextDiff`. File is source of truth; overwrites DB on mismatch.

On save: `writeContextToDB()` writes to DB if open.

## Runtime state (NOT persisted in context files)

These are per-session, managed by `DBContextManager`:

- **Loading filters**: `ds_type_loading_wanted_`, `ds_loading_wanted_`, `line_loading_wanted_` -- which data sources to load from DB.
- **Counts**: `inserted_counts_`, `loaded_counts_` (ds_id -> dbcontent_name -> line_id -> count). Persisted in `db_info` on DB close, not in context files.
- **Max timestamps**: `max_timestamps_` (ds_id -> line_id -> ptime).
- **Sensor config**: `SensorConfig` struct with radar stddevs, status widget config, display options.

## DBContextDiff

Compares two `DBContext` objects field-by-field:

```cpp
struct DBContextDiff {
    std::vector<ItemDiff> sensor_diffs, fft_diffs, asterix_diffs, sector_diffs;
    bool hasDifferences() const;
    std::string summary() const;
    static DBContextDiff compute(const DBContext& a, const DBContext& b);
};
```

`ItemDiff` types: `Added`, `Removed`, `Modified`. Modified items include per-field diffs. Keys: sac/sic for sensors, name for FFTs, category for ASTERIX, id for sectors.

## Edit dialog

`DBContextEditDialog` is the main UI for editing contexts:

```
[Context Combo] [Copy] [Rename] [Delete]
[Tree View]  |  [Detail Widget Stack]
```

Tree structure (via `DBContextEditTreeItem` hierarchy):
- **Data Sources** (GroupItem) -> DSTypeGroupItem ("Radar", "ADSB", ...) -> DataSourceItem
- **Sector Layers** (GroupItem) -> SectorLayerItem -> SectorItem
- **FFTs** (GroupItem) -> FFTItem
- **ASTERIX Configuration** (ASTERIXConfigLeafItem)

Detail widgets: `SectorEditWidget`, `FFTEditWidget` (and DataSource editing in the dialog itself).

Context management dialogs: `Create`, `Select`, `Rename`, `Delete`, `Copy`.

## Accessing from code

```cpp
// From any manager that has COMPASS&
auto& ctx_mgr = compass.dbContextManager();

// Guard before using
if (ctx_mgr.hasActiveContext()) {
    auto& ctx = ctx_mgr.activeContext();
    // query data sources, FFTs, sectors...
}

// Connect to changes
connect(&ctx_mgr, &DBContextManager::activeContextChangedSignal, this, &MyClass::onContextChanged);
```

## Key patterns

- **Guard with `hasActiveContext()`** before calling `activeContext()`.
- **File is source of truth** -- DB copy is synced from file, not vice versa.
- **Sectors are cached** as `SectorLayer` objects, rebuilt via `rebuildSectorLayers()`.
- **`info_` JSON field** in `DataSource` and `FFT` provides extensible metadata without schema changes.
- **IDataSourceProvider** decouples widgets from the full manager.
