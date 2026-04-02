# View Points: Error Handling for Annotation Consumption

## User Manual References

For view point and annotation format specifications, see the user manual:
- **View Points UI**: `doc/user_manual/flightdeck/viewpoints/view_points.tex` (Section "View Points")
- **View Point JSON Format**: `doc/user_manual/appendix/appendix_view_points.tex` (Appendix: View Points) — defines the JSON structure, version, annotation features, and format requirements
- **Annotation Layers**: `doc/user_manual/geographicview/geo_layers_annotation_ops.tex` — Geographic View annotation layer operations

## Overview

When a view point is set via the `set_view_point` runtime command, its JSON payload is consumed
by multiple components (views, filters, etc.). Annotations in particular are parsed and rendered
by the Geographic View. Bad or malformed JSON must be handled gracefully — never crashing the
application — and all errors must be reported back through the command's reply.

## Architecture

### Error Accumulator (ViewManager)

`ViewManager` maintains a list of errors encountered while consuming a view point:

```cpp
void reportViewPointError(const std::string& component_name, const std::string& error);
void clearViewPointErrors();
const std::vector<std::pair<std::string, std::string>>& viewPointErrors() const;
```

- **Cleared** automatically at the start of `setCurrentViewPoint()`.
- **Populated** by any component that fails during view point consumption.
- **Read** by `RTCommandSetViewPoint::checkResult_impl()` after data loading completes.

### Command Reply

If any errors were reported, `set_view_point` returns:

```json
{
    "ok": false,
    "error": "view point consumption failed in 2 component(s)",
    "reply": {
        "view_point_errors": [
            {"component": "GeographicView0", "error": "annotation error: unknown drawable type 'foo'"},
            {"component": "GeographicView0", "error": "annotation error: annotation without name"}
        ]
    }
}
```

If no errors, the command returns `"ok": true` as usual.

### Timing

All errors are collected before `checkResult_impl()` runs:

```
run_impl()
  -> clearViewPointErrors()
  -> emit showViewPointSignal   -> views consume VP (phase 1 errors reported here)
  -> load()
  ... loading ...
  -> doViewPointAfterLoad()     -> views process loaded data (phase 2 errors reported here)
  -> loadingDoneSignal
checkResult_impl()
  -> read viewPointErrors()     <- all errors available
```

## Annotation JSON Parsing: Error Handling Rules

### Principle: throw on bad JSON, catch at the boundary

All JSON validation in annotation and drawable code uses **exceptions** (`std::runtime_error`).
The caller (viewpoint or internal) catches them and decides the policy:

- **Viewpoint annotations**: catch, report via `reportViewPointError()`, continue with next annotation.
- **Internal annotations**: catch, `traced_assert(false)` — internal annotations are our own code,
  so bad data is a programming error.

### What throws

**Drawable classes** (`DrawableFactory::fromJSON`, `DrawableFeature::fromJSON`, etc.):
- Missing or unknown `type` field
- Type mismatch (expected vs actual)
- Missing `geometry` field
- `sizes` not an array (ellipses)
- `fromJSON()` parse failure
- `update()` init failure

**Annotation tree items** (`OSGAnnotationsTreeItemViewPoint::build`, `OSGAnnotationsRootTreeItemViewPoint::update`):
- `annotations` field is not an array
- Annotation missing `name` field
- Children count mismatch between scan result and JSON
- `features` field is not an array
- Feature missing `type` field

**Scan function** (`scanAnnotationForFeaturesRecursive`):
- `features` field present but not an array
- Feature missing `type` field
- `annotations` field present but not an array

### What does NOT throw

Internal invariants that indicate programming errors remain as `traced_assert`:
- `map_node_.valid()` — the map node must exist before any annotation work starts
- Pointer validity checks in non-JSON code paths

### Catch sites

**Viewpoint path** — `OSGAnnotationsRootTreeItemViewPoint::update()`:
- Outer try/catch around the entire annotation array processing (catches structural errors
  like "annotations is not an array").
- Inner try/catch per annotation (catches per-annotation errors, allowing other annotations
  to still be processed).
- Both report via `viewManager().reportViewPointError(getName(), ...)`.

**Internal path** — `OSGAnnotationsRootTreeItemInternal::addAnnotation()`:
- Single try/catch around the entire method body.
- On catch: logs the error and calls `traced_assert(false)` since internal annotation data
  is always produced by our own code.

## Adding Error Reporting to Other Views

Any component consuming a view point can report errors the same way:

```cpp
viewManager().reportViewPointError(getName(), "description of what failed");
```

This works from any `View` subclass. For non-view components, pass the component name directly.
All reported errors will appear in the command's reply JSON.

## Data Source Selection (`data_sources`)

View points can restrict which data sources are loaded via the `"data_sources"` key. The value is a JSON array of `[ds_id, [line_ids]]` pairs (how nlohmann/json serializes `map<unsigned int, set<unsigned int>>`). An empty line array means **no lines** — `disableAllLines()` is called first, then only listed lines are enabled. To load all lines, list them explicitly (e.g. `[0, 1, 2, 3]`).

```json
{
    "data_sources": [
        [12750, [0, 1]],
        [65025, []]
    ]
}
```

- When `data_sources` is present, only the listed sources are loaded (`DataSourceManager::setLoadOnlyDataSources`).
- When absent, all data sources are loaded.
- `data_source_types` (e.g. `["ADSB", "RefTraj"]`) is applied independently — it restricts which DS types are enabled.
- The read side uses `get<map<unsigned int, set<unsigned int>>>()` which also accepts a JSON object with string keys (`{"12750": [0, 1]}`), but the write side always produces array-of-pairs.

**Key source files**:
- Write: `FilterManager::getFilterState()` in `filtermanager.cpp` — serializes via `ViewPoint::VP_DS_KEY`
- Read: `FilterManager::showViewPointSlot()` in `filtermanager.cpp` — deserializes and calls `setLoadOnlyDataSources`
- Constant: `ViewPoint::VP_DS_KEY` = `"data_sources"` in `viewpoint.h`

**Key source files**:
- Write: `FilterManager::getFilterState()` in `filtermanager.cpp` — serializes via `ViewPoint::VP_DS_KEY`
- Read: `FilterManager::showViewPointSlot()` in `filtermanager.cpp` — deserializes and calls `setLoadOnlyDataSources`
- Constant: `ViewPoint::VP_DS_KEY` = `"data_sources"` in `viewpoint.h`

## Files

| File | Role |
|------|------|
| `src/view/viewmanager.h/.cpp` | Error accumulator: `reportViewPointError`, `clearViewPointErrors`, `viewPointErrors` |
| `src/view/points/viewpoint_commands.cpp` | `checkResult_impl()` reads errors, populates command reply |
| `experimental_src/.../annotations/osgannotationsroottreeitem_viewpoint.cpp` | Viewpoint catch site: per-annotation try/catch, reports errors |
| `experimental_src/.../annotations/osgannotationstreeitem_viewpoint.cpp` | Viewpoint annotation `build()`: throws on bad JSON |
| `experimental_src/.../annotations/osgannotationsroottreeitem_internal.cpp` | Internal catch site: try/catch + `traced_assert(false)` |
| `experimental_src/.../annotations/osgannotationstreeitem_internal.cpp` | Internal annotation `build()`: throws on bad JSON |
| `experimental_src/.../drawable/drawablefactory.cpp` | Drawable creation: throws on bad JSON |
| `experimental_src/.../drawable/drawablefeature.cpp` | Feature base `fromJSON`: throws on type errors |
| `experimental_src/.../drawable/drawablearray.cpp` | Array drawable `fromJSON`: throws on missing geometry |
| `experimental_src/.../drawable/drawableellipses.cpp` | Ellipses `fromJSON`: throws on bad sizes |
