# Filter Unit Test Implementation Instructions

## Goal
Write Catch2 unit tests for all DBFilter subclasses in `src/filter/`. Filters have been decoupled from COMPASS/DBContentManager/DataSourceManager via `IDBVariableResolver` interface, so they can be instantiated standalone with `parent=nullptr` and a mock resolver.

## File Structure

```
src/filter/unit_tests/
  CMakeLists.txt                    # included by src/unit_tests/CMakeLists.txt
  mock_variable_resolver.h          # MockVariableResolver + config helpers
  test_timestamp_filter.cpp
  test_mode3a_filter.cpp
  test_modec_filter.cpp
  test_acad_filter.cpp
  test_acid_filter.cpp
  test_utn_filter.cpp
  test_primaryonly_filter.cpp
  test_adsbquality_filter.cpp
  test_mlatru_filter.cpp
  test_tracker_tracknum_filter.cpp
  test_reftraj_accuracy_filter.cpp
  test_excluded_timewindows_filter.cpp
  test_dbfilter_condition.cpp       # generic DBFilter with DBFilterCondition sub-configs
  test_filter_buffer.cpp            # filterBuffer() tests for Mode3A, ModeC, ACAD, ACID, PrimaryOnly
```

## CMakeLists.txt

Follow the pattern from `src/config/unit_tests/CMakeLists.txt`:
```cmake
target_sources(compass_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/test_timestamp_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test_mode3a_filter.cpp
    # ... all test files
)
```

Then add to `src/unit_tests/CMakeLists.txt`:
```cmake
include("${CMAKE_SOURCE_DIR}/src/filter/unit_tests/CMakeLists.txt")
```

## MockVariableResolver (mock_variable_resolver.h)

### Design

A header-only mock implementing `IDBVariableResolver`. It uses lookup tables populated by the test. No JSON config files needed — the mock is configured programmatically per test.

### Key Data Structures

```cpp
#include "idbvariableresolver.h"
#include "property.h"
#include "dbcontent/variable/variableset.h"
#include <map>
#include <set>
#include <string>

class MockVariableResolver : public IDBVariableResolver
{
public:
    // --- Registration API (used by tests to configure the mock) ---

    // Register a meta variable: which DBContent types it exists in, and what DB column it maps to
    void addMetaVariable(const Property& prop,
                         const std::map<std::string, std::string>& dbcontent_to_column,
                         const std::map<std::string, std::string>& dbcontent_to_varname = {});

    // Register a direct (non-meta) variable for a specific DBContent
    void addDirectVariable(const std::string& dbcontent_name, const Property& prop,
                           const std::string& db_column,
                           const std::string& var_name = "");

    // --- Implementation of IDBVariableResolver ---
    // (all methods delegate to the lookup tables)
};
```

### Internal Storage

```cpp
private:
    // meta variables: Property key -> { dbcontent_name -> db_column_name }
    struct MetaVarInfo {
        std::map<std::string, std::string> dbcontent_to_column;  // dbcontent -> db column
        std::map<std::string, std::string> dbcontent_to_varname; // dbcontent -> variable name
    };
    std::map<std::string, MetaVarInfo> meta_vars_; // keyed by Property::name()

    // direct variables: (dbcontent_name, Property::name()) -> { db_column, var_name }
    struct DirectVarInfo {
        std::string db_column;
        std::string var_name;
        PropertyDataType data_type;
    };
    std::map<std::pair<std::string, std::string>, DirectVarInfo> direct_vars_;

    std::set<std::string> known_dbcontents_; // auto-populated from addMetaVariable/addDirectVariable
```

### Method Implementations

Property-based methods:
- `metaCanGetVariable(dbcontent, prop)` → check if `meta_vars_[prop.name()].dbcontent_to_column` contains `dbcontent`
- `metaGetVariableDBColumn(dbcontent, prop)` → return `meta_vars_[prop.name()].dbcontent_to_column[dbcontent]`
- `metaGetVariableName(dbcontent, prop)` → return the var_name mapping, or fall back to prop.name()
- `canGetVariable(dbcontent, prop)` → check `direct_vars_` has `{dbcontent, prop.name()}`
- `getVariableDBColumn(dbcontent, prop)` → return from `direct_vars_`
- `getVariableName(dbcontent, prop)` → return from `direct_vars_`, fall back to prop.name()
- `variableHasDBContent(dbcontent, prop)` → same as `canGetVariable`

Name-based methods (for DBFilterCondition):
- `existsMetaVariable(var_name)` → check `meta_vars_` has key `var_name`
- `metaVariableExistsIn(var_name, dbcontent)` → check `meta_vars_[var_name].dbcontent_to_column` has `dbcontent`
- `existsDBContent(dbcontent)` → check `known_dbcontents_`
- `dbContentHasVariable(dbcontent, var_name)` → check direct_vars_ or meta_vars_ for presence
- `metaVariableDBContentNames(var_name)` → return keys of `meta_vars_[var_name].dbcontent_to_column`
- `variableDBColumnName(dbcontent, var_name, var_dbcontent_name)` → if `var_dbcontent_name == "Meta"`, resolve via meta_vars_, else via direct_vars_
- `variableDBTableName(...)` → return `dbcontent` (table name = DBContent name in this mock)
- `variableDBExpression(...)` → return the column name (same as variableDBColumnName for the mock)
- `variableDataType(...)` → return from the Property or stored data_type
- `variableHasNonStandardRepresentation(...)` → return false (mock always standard)
- `variableValueFromRepresentation(...)` → return the value_str unchanged
- `readSetHasVariable(...)` → return false (mock: not in read set)
- `addVariableToReadSet(...)` → no-op (mock doesn't track read sets)

### Helper: Pre-configured Mock for Common Tests

```cpp
// Creates a MockVariableResolver pre-populated with all meta variables and direct
// variables needed by the standard filter set, using realistic column names.
static MockVariableResolver createStandardMock()
{
    MockVariableResolver mock;

    // Timestamp — all DBContent types
    mock.addMetaVariable(DBContent::meta_var_timestamp_,
        {{"CAT001", "timestamp"}, {"CAT010", "timestamp"}, {"CAT020", "timestamp"},
         {"CAT021", "timestamp"}, {"CAT048", "timestamp"}, {"CAT062", "timestamp"},
         {"RefTraj", "timestamp"}});

    // Mode 3/A — CAT001, CAT010, CAT020, CAT021, CAT048, CAT062, RefTraj
    mock.addMetaVariable(DBContent::meta_var_m3a_,
        {{"CAT001", "mode3a_code"}, {"CAT010", "mode3a_code"}, {"CAT020", "mode3a_code"},
         {"CAT021", "mode3a_code"}, {"CAT048", "mode3a_code"}, {"CAT062", "mode3a_code"},
         {"RefTraj", "mode3a_code"}},
        {{"CAT001", "Mode 3/A Code"}, {"CAT010", "Mode 3/A Code"}, {"CAT020", "Mode 3/A Code"},
         {"CAT021", "Mode 3/A Code"}, {"CAT048", "Mode 3/A Code"}, {"CAT062", "Mode 3/A Code"},
         {"RefTraj", "Mode 3/A Code"}});

    // Mode C
    mock.addMetaVariable(DBContent::meta_var_mc_,
        {{"CAT001", "modec_code_ft"}, {"CAT010", "modec_code_ft"}, {"CAT020", "modec_code_ft"},
         {"CAT021", "modec_code_ft"}, {"CAT048", "modec_code_ft"}, {"CAT062", "modec_code_ft"},
         {"RefTraj", "modec_code_ft"}},
        {{"CAT001", "Mode C Code"}, {"CAT010", "Mode C Code"}, {"CAT020", "Mode C Code"},
         {"CAT021", "Mode C Code"}, {"CAT048", "Mode C Code"}, {"CAT062", "Mode C Code"},
         {"RefTraj", "Mode C Code"}});

    // Aircraft Address (ACAD)
    mock.addMetaVariable(DBContent::meta_var_acad_,
        {{"CAT010", "target_addr"}, {"CAT020", "target_addr"}, {"CAT021", "target_addr"},
         {"CAT048", "target_addr"}, {"CAT062", "target_addr"}, {"RefTraj", "target_addr"}},
        {{"CAT010", "Aircraft Address"}, {"CAT020", "Aircraft Address"}, {"CAT021", "Aircraft Address"},
         {"CAT048", "Aircraft Address"}, {"CAT062", "Aircraft Address"}, {"RefTraj", "Aircraft Address"}});

    // Aircraft Identification (ACID)
    mock.addMetaVariable(DBContent::meta_var_acid_,
        {{"CAT010", "target_id"}, {"CAT020", "target_id"}, {"CAT021", "target_id"},
         {"CAT048", "target_id"}, {"CAT062", "target_id"}, {"RefTraj", "target_id"}},
        {{"CAT010", "Aircraft Identification"}, {"CAT020", "Aircraft Identification"},
         {"CAT021", "Aircraft Identification"}, {"CAT048", "Aircraft Identification"},
         {"CAT062", "Aircraft Identification"}, {"RefTraj", "Aircraft Identification"}});

    // UTN
    mock.addMetaVariable(DBContent::meta_var_utn_,
        {{"CAT001", "utn"}, {"CAT010", "utn"}, {"CAT020", "utn"}, {"CAT021", "utn"},
         {"CAT048", "utn"}, {"CAT062", "utn"}, {"RefTraj", "utn"}});

    // DS ID
    mock.addMetaVariable(DBContent::meta_var_ds_id_,
        {{"CAT001", "ds_id"}, {"CAT010", "ds_id"}, {"CAT020", "ds_id"}, {"CAT021", "ds_id"},
         {"CAT048", "ds_id"}, {"CAT062", "ds_id"}, {"RefTraj", "ds_id"}});

    // Line ID
    mock.addMetaVariable(DBContent::meta_var_line_id_,
        {{"CAT001", "line_id"}, {"CAT010", "line_id"}, {"CAT020", "line_id"}, {"CAT021", "line_id"},
         {"CAT048", "line_id"}, {"CAT062", "line_id"}, {"RefTraj", "line_id"}});

    // Track Number
    mock.addMetaVariable(DBContent::meta_var_track_num_,
        {{"CAT001", "track_num"}, {"CAT010", "track_num"}, {"CAT020", "track_num"},
         {"CAT021", "track_num"}, {"CAT048", "track_num"}, {"CAT062", "track_num"},
         {"RefTraj", "track_num"}},
        {{"CAT001", "Track Number"}, {"CAT010", "Track Number"}, {"CAT020", "Track Number"},
         {"CAT021", "Track Number"}, {"CAT048", "Track Number"}, {"CAT062", "Track Number"},
         {"RefTraj", "Track Number"}});

    // Detection Type
    mock.addMetaVariable(DBContent::meta_var_detection_type_,
        {{"CAT001", "detection_type"}, {"CAT048", "detection_type"}, {"CAT062", "detection_type"}},
        {{"CAT001", "Type"}, {"CAT048", "Type"}, {"CAT062", "Type"}});

    // Ground Bit
    mock.addMetaVariable(DBContent::meta_var_ground_bit_,
        {{"CAT010", "ground_bit"}, {"CAT020", "ground_bit"}, {"CAT021", "ground_bit"},
         {"CAT062", "ground_bit"}, {"RefTraj", "ground_bit"}});

    // X/Y StdDev (for RefTrajAccuracyFilter)
    mock.addMetaVariable(DBContent::meta_var_x_stddev_,
        {{"RefTraj", "x_stddev"}, {"CAT062", "x_stddev"}});
    mock.addMetaVariable(DBContent::meta_var_y_stddev_,
        {{"RefTraj", "y_stddev"}, {"CAT062", "y_stddev"}});

    // Latitude / Longitude (for Position filter condition)
    mock.addMetaVariable(DBContent::meta_var_latitude_,
        {{"CAT001", "latitude"}, {"CAT010", "latitude"}, {"CAT020", "latitude"},
         {"CAT021", "latitude"}, {"CAT048", "latitude"}, {"CAT062", "latitude"},
         {"RefTraj", "latitude"}});
    mock.addMetaVariable(DBContent::meta_var_longitude_,
        {{"CAT001", "longitude"}, {"CAT010", "longitude"}, {"CAT020", "longitude"},
         {"CAT021", "longitude"}, {"CAT048", "longitude"}, {"CAT062", "longitude"},
         {"RefTraj", "longitude"}});

    // Time of Day (for Time of Day filter condition)
    mock.addMetaVariable(DBContent::meta_var_time_of_day_,
        {{"CAT001", "tod"}, {"CAT010", "tod"}, {"CAT020", "tod"}, {"CAT021", "tod"},
         {"CAT048", "tod"}, {"CAT062", "tod"}, {"RefTraj", "tod"}});

    // --- Direct variables ---

    // CAT021 ADS-B quality variables
    mock.addDirectVariable("CAT021", DBContent::var_cat021_mops_version_, "mops_version");
    mock.addDirectVariable("CAT021", DBContent::var_cat021_nacp_, "nacp");
    mock.addDirectVariable("CAT021", DBContent::var_cat021_nucp_nic_, "nucp_nic");
    mock.addDirectVariable("CAT021", DBContent::var_cat021_sil_, "sil");

    // CAT020 MLAT contrib receivers
    mock.addDirectVariable("CAT020", DBContent::var_cat020_contrib_recv_, "contrib_receivers");

    // CAT062 direct variables
    mock.addDirectVariable("CAT062", DBContent::var_cat062_baro_alt_, "baro_alt");
    mock.addDirectVariable("CAT062", DBContent::var_cat062_fl_measured_, "fl_measured");
    mock.addDirectVariable("CAT062", DBContent::var_cat062_callsign_fpl_, "callsign_fpl",
                           "Callsign FPL");

    return mock;
}
```

### Config Helper Function

Reuse the pattern from `src/config/unit_tests/test_configurable.cpp`:

```cpp
#include "configuration.h"
#include "json.hpp"

inline nlohmann::json makeFilterConfig(const std::string& class_name,
                                       const std::string& instance_name,
                                       nlohmann::json params = nlohmann::json::object())
{
    nlohmann::json cfg;
    cfg[Configuration::CLASS_NAME_KEY]    = class_name;
    cfg[Configuration::INSTANCE_NAME_KEY] = instance_name;
    if (!params.empty())
        cfg["parameters"] = params;
    return cfg;
}
```

## Default Filter Configs

Copy from `conf/default/filter.json` — each sub_config entry is the JSON that goes into `makeFilterConfig`. The `filter.json` file has all defaults. Key configs:

| Filter Class | class_name in JSON | instance_name | Key parameters |
|---|---|---|---|
| `TimestampFilter` | `TimestampFilter` | `TimestampFilter0` | `min_value`, `max_value` (string timestamps) |
| `Mode3AFilter` | `Mode3AFilter` | `Mode 3/A Codes` | `values_str` (octal, comma-sep) |
| `ModeCFilter` | `ModeCFilter` | `Mode C Codes` | `min_value`, `max_value` (float), `null_wanted` |
| `ACADFilter` | `ACADFilter` | `Aircraft Address` | `values_str` (hex, comma-sep) |
| `ACIDFilter` | `ACIDFilter` | `Aircraft Identification` | `values_str` (string, comma-sep) |
| `UTNFilter` | `UTNFilter` | `UTNFilter0` | `utns_str` (decimal, comma-sep) |
| `PrimaryOnlyFilter` | `PrimaryOnlyFilter` | `Primary Only` | (no filter-specific params) |
| `ADSBQualityFilter` | `ADSBQualityFilter` | `ADSBQualityFilter` | `use_v0..v2`, `min/max_nucp/nic/nacp/sil_v1/sil_v2` |
| `MLATRUFilter` | `MLATRUFilter` | `MLATRUFilter0` | `rus_str`, `match_all` |
| `TrackerTrackNumberFilter` | `TrackerTrackNumberFilter` | `TrackerTrackNumberFilter0` | `tracker_track_nums` (json object) |
| `RefTrajAccuracyFilter` | `RefTrajAccuracyFilter` | `RefTrajAccuracyFilter0` | `min_value` (float) |
| `ExcludedTimeWindowsFilter` | `ExcludedTimeWindowsFilter` | `ExcludedTimeWindowsFilter0` | `time_windows_json` (json array) |

Generic DBFilter with conditions:
| Filter | instance_name | sub_configs |
|---|---|---|
| Position | `Position` | 4 DBFilterCondition sub_configs (Lat Min/Max, Lon Min/Max) |
| Ground Bit | `DBFilter1` | 1 DBFilterCondition: meta "Ground Bit" = 1 |
| ADSBMOPS | `ADSBMOPS` | 1 DBFilterCondition: CAT021 "MOPS Version" IN 0 |
| Detection Type | `Detection Type` | 1 DBFilterCondition: meta "Type" IN 1 |
| Track Number | `Track Number` | 1 DBFilterCondition: meta "Track Number" IN 4227 |
| Time of Day | `Time of Day` | 2 DBFilterCondition: meta "Time of Day" >= and <= |

## Test Pattern (per filter file)

```cpp
#include "catch.hpp"
#include "mock_variable_resolver.h"  // MockVariableResolver + makeFilterConfig + createStandardMock
#include "timestampfilter.h"         // the specific filter header
#include "dbcontent/dbcontent.h"     // for DBContent::meta_var_* static properties
#include "dbcontent/variable/variableset.h"

TEST_CASE("TimestampFilter construction", "[filter][timestamp]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });

    TimestampFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Timestamp");
    CHECK_FALSE(filter.getActive());  // default active=false not in params, so default
}
```

## What to Test Per Filter

### A. Construction (all filters)
- Construct from default JSON + mock resolver with `parent=nullptr`
- Verify `getName()`, `getActive()`, filter-specific getters

### B. `filters()` — DBContent applicability (all filters)
- Call `filters("CAT048")`, `filters("CAT062")`, `filters("CAT021")`, `filters("CAT020")`, `filters("RefTraj")`, `filters("CAT002")` (should be false for most)
- Verify each filter returns true only for expected DBContent types

Which filters apply to which DBContent types:
- **TimestampFilter**: all that have timestamp meta var (CAT001-062, RefTraj)
- **ExcludedTimeWindowsFilter**: same as TimestampFilter
- **Mode3AFilter**: those with m3a meta var (CAT001, 010, 020, 021, 048, 062, RefTraj)
- **ModeCFilter**: those with mc meta var (same as Mode3A)
- **ACADFilter**: those with acad meta var (CAT010, 020, 021, 048, 062, RefTraj)
- **ACIDFilter**: those with acid meta var + always CAT062 (special case: line 46-47)
- **UTNFilter**: always returns true for ALL DBContent types (line 48)
- **PrimaryOnlyFilter**: those with at least one of m3a/mc/acad/acid meta vars
- **ADSBQualityFilter**: only CAT021
- **MLATRUFilter**: only CAT020
- **TrackerTrackNumberFilter**: only CAT062
- **RefTrajAccuracyFilter**: only RefTraj

### C. `getConditionString()` — SQL generation (all filters)

For each filter: set `active_` to true, configure values, create a `VariableSet`, call `getConditionString()`, verify the SQL output string.

**IMPORTANT**: The `first` parameter (bool&) controls whether `" AND"` is prepended. First call should pass `first=true`, which the filter sets to `false` after appending. Test both cases.

Specific SQL patterns to verify (use `REQUIRE(sql.find("expected_substring") != std::string::npos)` or exact match where feasible):

- **TimestampFilter**: `(timestamp >= LONG AND timestamp <= LONG)` — uses `Time::toLong()` for the values
- **Mode3AFilter**: `mode3a_code IN (2041)` for octal "3771" → decimal 2041. Note: values_str is parsed as octal (base 8). So "7700" → decimal 4032.
- **ModeCFilter**: `(modec_code_ft BETWEEN -1000 AND 3000)`. For CAT062 also adds baro_alt and fl_measured conditions.
- **ACADFilter**: `target_addr IN (5013616)` for hex "4C8070" → decimal 5013616
- **ACIDFilter**: `(target_id LIKE '%AEE%')`. For CAT062 also checks `callsign_fpl`.
- **UTNFilter**: `utn IN (0)` for utns_str "0". Also test null_wanted behavior and non-associated DBContent (`" false"` when no UTN var and null not wanted).
- **PrimaryOnlyFilter**: multiple `col IS NULL` conditions + detection_type IN check
- **ADSBQualityFilter**: `mops_version IN (0,1,2)` + conditional min/max checks. Only for CAT021.
- **MLATRUFilter**: Needs pushed MLAT data via `updateMLATDataSources()`. Build a test lookup, set `rus_str`, verify SQL with `json_contains` patterns. Only for CAT020.
- **TrackerTrackNumberFilter**: Needs pushed data via `updateTrackerDataSources()`. Set track nums, verify SQL with ds_id/line_id/track_num conditions. Only for CAT062.
- **RefTrajAccuracyFilter**: `sqrt(pow(x_stddev,2) + (pow(y_stddev,2))) <= 30`. Only for RefTraj. Note: guards with `metaCanGetVariable(dbcontent_name, meta_var_mc_)` — so the mock must have mc for RefTraj.
- **ExcludedTimeWindowsFilter**: `NOT (timestamp BETWEEN X AND Y)` for each time window

**Inactive filter**: When `active_` is false, all filters must return empty string from `getConditionString()`. Set active via config param `"active": false` or by not calling `setActive(true)`.

### D. `filterBuffer()` (test_filter_buffer.cpp)

For filters that override `filterBuffer()`: Mode3AFilter, ModeCFilter, ACADFilter, ACIDFilter, PrimaryOnlyFilter.

Create a `Buffer` with appropriate `NullableVector<T>` columns, populate with test data, call `filterBuffer()`, verify returned index vector.

```cpp
#include "buffer/buffer.h"

// Example for Mode3AFilter:
auto buffer = std::make_shared<Buffer>();
// The variable name must match what metaGetVariableName returns for the DBContent
// For CAT048 + meta_var_m3a_, the mock returns "Mode 3/A Code"
auto& vec = buffer->addNewColumn<unsigned int>("Mode 3/A Code");  // or however Buffer::addNewColumn works
vec.set(0, 2041u);  // matches octal 3771
vec.set(1, 100u);   // does not match
vec.setNull(2);     // null

auto removed = filter.filterBuffer("CAT048", buffer);
// removed should contain index 1 (non-matching) and 2 (null, if null_wanted_=false)
```

**Check Buffer API**: Read `src/core/buffer/buffer.h` and `src/core/buffer/nullablevector.h` to understand `addNewColumn`, `set`, `setNull`, `size`. The existing tests in `src/core/unit_tests/test_buffer.cpp` show usage patterns.

### E. Pushed data (specific filters)

- **TrackerTrackNumberFilter**: call `updateTrackerDataSources({...}, {...})`, verify `getActiveTrackerTrackNums()`, `dataSourceName()`, `hasDataSourceName()`
- **MLATRUFilter**: call `updateMLATDataSources({...})` and `updateMLATKnownRUNames({...})`, then verify `getConditionString()` and `checkRUs()`
- **ExcludedTimeWindowsFilter**: call `updateMinMaxTimestamp(...)`, verify `hasMinMaxTimestamp()` and `minMaxTimestamp()`
- **TimestampFilter**: call `reset(min, max)`, verify `minValue()` and `maxValue()`

### F. ViewPoint save/load round-trip (all filters)

```cpp
nlohmann::json vp_filters;
filter.saveViewPointConditions(vp_filters);

// Create a fresh filter from same config
auto cfg2 = makeFilterConfig(...);
FilterType filter2(cfg2, nullptr, mock);
filter2.setActive(true);
filter2.loadViewPointConditions(vp_filters);

// Verify getConditionString produces same output
```

**Note**: `loadViewPointConditions` calls `widget()` on some filters. Since `parent=nullptr`, `createWidget()` will create a real Qt widget. This may require a `QApplication` instance. If so, create one in the test:
```cpp
// In test_main.cpp or at top of test file:
// static int argc = 0;
// static QApplication app(argc, nullptr);
```
**OR** — avoid calling functions that trigger widget creation. The `loadViewPointConditions` methods call `widget()` only if `widget_` is already set (checking `if (widget())`). Since we never call `widget()` first, `widget_` stays nullptr. Check each filter — some call `widget()` unconditionally vs `if (widget_)`. The ones that call `widget()` (which creates the widget) will need the QApplication. The ones that check `if (widget_)` or `if (widget())` are safe without it.

Filters that unconditionally call `widget()` in loadViewPointConditions: ACIDFilter (line 157: `if (widget())`), Mode3AFilter (line 129: `if (widget())`), UTNFilter (line 149: `if (widget())`), ACADFilter (line 127: `if (widget())`).

These call `widget()` which creates the widget via `createWidget()`. **Skip viewpoint round-trip tests for these** or add a `QApplication` to the test binary. The `TimestampFilter`, `ModeCFilter`, `ExcludedTimeWindowsFilter`, `TrackerTrackNumberFilter`, `ADSBQualityFilter`, `MLATRUFilter`, `RefTrajAccuracyFilter` are safe because they check `if (widget_)` or `if (widget())` only after `widget_` was already set by a prior call.

**Actually**: `widget()` in DBFilter creates the widget on first call (line 205-212 of dbfilter.cpp). So ANY filter calling `widget()` or `if (widget())` will trigger creation. Filters using `if (widget_)` (checking the raw pointer) are safe. Check each filter's `loadViewPointConditions` carefully:
- `TimestampFilter`: calls `if (widget()) widget()->update()` → UNSAFE (creates widget)
- `ModeCFilter`: calls `if (widget()) widget()->update()` → UNSAFE
- Safe ones check `if (widget_)` — the raw member pointer.

**Recommendation**: Either add QApplication to the test binary, or skip viewpoint round-trip tests entirely for now. The core value is in testing `getConditionString()` anyway.

### G. DBFilterCondition tests (test_dbfilter_condition.cpp)

Test the generic `DBFilter` class with `DBFilterCondition` sub-configs. Use configs from `filter.json`:

```cpp
// Position filter with 4 conditions (Lat min/max, Lon min/max)
auto cfg = makeFilterConfig("DBFilter", "Position", {
    {"active", true}, {"is_custom", false}, {"name", "Position"}
});
cfg["sub_configs"] = nlohmann::json::array({
    {
        {"class_name", "DBFilterCondition"},
        {"instance_name", "Latitude Maximum"},
        {"parameters", {
            {"absolute_value", false}, {"op_and", true},
            {"operator", "<="}, {"reset_value", "MAX"},
            {"value", "45.958201"},
            {"variable_dbcontent_name", "Meta"},
            {"variable_name", "Latitude"}
        }}
    },
    // ... more conditions
});

auto mock = createStandardMock();
DBFilter filter(cfg, true, nullptr, mock);  // is_generic=true for condition-based filters

// Check conditions were created
CHECK(filter.getNumConditions() == 4);

// Check SQL generation
VariableSet read_set;
bool first = true;
std::string sql = filter.getConditionString("CAT048", read_set, first);
CHECK(sql.find("latitude") != std::string::npos);
CHECK(sql.find("<= 45.958201") != std::string::npos);
```

Test different operators: `=`, `!=`, `>`, `>=`, `<`, `<=`, `IN`, `LIKE`.
Test meta vs direct variable resolution (`variable_dbcontent_name: "Meta"` vs `"CAT021"`).
Test `absolute_value: true` → wraps column in `ABS()`.

### H. Edge cases

- Inactive filter → empty condition string
- Empty values_str → no crash, empty or no condition
- Unknown DBContent name → `filters()` returns false, `getConditionString()` returns empty
- `first=false` → condition string starts with ` AND`

## Important Implementation Notes

1. **All filters take `nlohmann::json&` (non-const ref)** for their config. The json object must outlive the filter.

2. **`createSubConfigurables()`** is called in every filter constructor. For filters without sub_configs, it's a no-op. For `DBFilter` (generic, with conditions), it creates `DBFilterCondition` children.

3. **`DBFilter` constructor** takes `is_generic` bool as second param. Specific filter subclasses pass `false`. The generic `DBFilter` class itself (used for condition-based filters) should be constructed directly with `is_generic=true`.

4. **Don't test widgets**. The widget code depends on Qt event loop and would require QApplication. Focus on the data/logic layer.

5. **TrackerTrackNumberFilter is a QObject** — inherits from both QObject and DBFilter. Construction works fine without QApplication, but if you need signal/slot testing, you'd need one.

6. **String::compress()** used in SQL generation joins set elements with a separator. The output ordering depends on `std::set` ordering (ascending).

7. **Mode3A values are octal**: `"3771"` → decimal `2041`. `"7700"` → decimal `4032`. Parsed via `QString::toInt(&ok, 8)`.

8. **ACAD values are hex**: `"4C8070"` → decimal `5013616`. Parsed via `QString::toInt(&ok, 16)`.

9. **Naming**: follow project convention. Test files: `test_<topic>.cpp`. Test tags: `[filter][<filtername>]`.

10. **GPL header**: All source files need the GPL-3.0 header (copy from any existing filter .cpp).

11. **Logging**: Tests will produce log output via `loginf`/`logdbg` macros — this is expected and harmless.

## Build & Run

```bash
cmake --build build -j$(nproc)
./build/bin/compass_tests "[filter]"           # run all filter tests
./build/bin/compass_tests "[timestamp]"        # run just timestamp filter tests
```
