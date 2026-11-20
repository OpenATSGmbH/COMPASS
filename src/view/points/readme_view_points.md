# View Points: Error Handling for Annotation Consumption

## User Manual References

For view point and annotation format specifications, see the user manual:
- **View Points UI**: `doc/user_manual/flightdeck/viewpoints/view_points.tex` (Section "View Points")
- **View Point JSON Format**: `doc/user_manual/appendix/appendix_view_points.tex` (Appendix: View Points) - defines the JSON structure, version, annotation features, and format requirements
- **Annotation Layers**: `doc/user_manual/geographicview/geo_layers_annotation_ops.tex` - Geographic View annotation layer operations

## Overview

When a view point is set via the `set_view_point` runtime command, its JSON payload is consumed
by multiple components (views, filters, etc.). Annotations in particular are parsed and rendered
by the Geographic View. Bad or malformed JSON must be handled gracefully - never crashing the
application - and all errors must be reported back through the command's reply.

## Architecture

### Error Accumulator (ViewableDataConfig)

The view point object itself carries the errors encountered while it is consumed. `ViewableDataConfig` in `src/view/points/viewabledataconfig.h` holds the list:

```cpp
void reportError(const std::string& component_name, const std::string& error);
bool hasErrors() const;
const std::vector<std::pair<std::string, std::string>>& errors() const;
```

- **Empty at the start**, because `set_view_point` builds a new `ViewableDataConfig` for every call. There is no clear method.
- **Populated** by any component that fails during view point consumption.
- **Read** by `RTCommandSetViewPoint::checkResult_impl()` after data loading completes.

A view reaches its own view point through the `viewPoint()` accessor of `VariableView` or `GeographicView`, which returns the `ViewableDataConfig&` the view currently shows. The Geographic View also gets the object as the argument of `showViewPointSlot()` and `unshowViewPointSlot()`.

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
  -> new ViewableDataConfig      (error list starts empty)
  -> setCurrentViewPoint()
  -> emit showViewPointSignal   -> views consume VP (phase 1 errors reported here)
  -> load()
  ... loading ...
  -> doViewPointAfterLoad()     -> views process loaded data (phase 2 errors reported here)
  -> loadingDoneSignal
checkResult_impl()
  -> read viewable_data_cfg_->errors()   <- all errors available
```

`run_impl()` hands the view point to `setCurrentViewPoint()` as a `shared_ptr`. The command object is destroyed before the load finishes, so shared ownership keeps the error list alive until `checkResult_impl()` reads it.

## Annotation JSON Parsing: Error Handling Rules

### Principle: throw on bad JSON, catch at the boundary

All JSON validation in annotation and drawable code uses **exceptions** (`std::runtime_error`).
The caller (viewpoint or internal) catches them and decides the policy:

- **Viewpoint annotations**: catch, report via `reportError()` on the view point, continue with next annotation.
- **Internal annotations**: catch, `traced_assert(false)` - internal annotations are our own code,
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
- `map_node_.valid()` - the map node must exist before any annotation work starts
- Pointer validity checks in non-JSON code paths

### Catch sites

**Viewpoint path** - `OSGAnnotationsRootTreeItemViewPoint::update()`:
- Outer try/catch around the entire annotation array processing (catches structural errors
  like "annotations is not an array").
- Inner try/catch per annotation (catches per-annotation errors, allowing other annotations
  to still be processed).
- Both report via `osg_view_.viewPoint().reportError(osg_view_.getName(), ...)`.

**Internal path** - `OSGAnnotationsRootTreeItemInternal::addAnnotation()`:
- Single try/catch around the entire method body.
- On catch: logs the error and calls `traced_assert(false)` since internal annotation data
  is always produced by our own code.

## Adding Error Reporting to Other Views

Any component consuming a view point reports errors on the view point object:

```cpp
viewPoint().reportError(getName(), "description of what failed");
```

`viewPoint()` is available in `VariableView` and in `GeographicView`. Guard it with `hasViewPoint()` where the code also runs outside a view point, because the accessor asserts. A component that is not a view calls `reportError()` on the `ViewableDataConfig` it was given and passes its own component name. The Grid View, Histogram View and Scatter Plot View data widgets use this form. All reported errors appear in the command's reply JSON.

## Load Statements (`load_data`)

A view point loads data only if it has a `"load_data"` object. Missing or `null`: the view point loads no data and shows its annotations only. `view_point_load::loadData()` / `loadsData()` in `src/core/viewpointkeys.h` decide this for all readers (filter, view, report figures).

Inside `load_data`, `data_source_types` and `data_sources` are required, each a list or the string `"all"` (`view_point_load::checkLoadData()`). A missing key: `FilterManager::showViewPointSlot()` logs an error and loads no data. `filters`, `selected_rec_nums`, `labels`, `timestamp` and `time_window` are optional keys of `load_data`.

```json
{
    "load_data": {
        "data_source_types": "all",
        "data_sources": "all",
        "filters": { "UTNs": { "utns": "4" } }
    }
}
```

Generated view points: `ViewPointGenVP` writes no `load_data` by default, `ViewPointGenVP::enableDataLoading()` writes it with `"all"` and the generator filters.

## Data Source Selection (`load_data.data_sources`)

View points can restrict which data sources are loaded via the `"data_sources"` key of `load_data`. The value is a JSON array of `[ds_id, [line_ids]]` pairs (how nlohmann/json serializes `map<unsigned int, set<unsigned int>>`). An empty line array means **no lines** - `disableAllLines()` is called first, then only listed lines are enabled. To load all lines, list them explicitly (e.g. `[0, 1, 2, 3]`).

```json
{
    "load_data": {
        "data_source_types": "all",
        "data_sources": [
            [12750, [0, 1]],
            [65025, []]
        ]
    }
}
```

- A list loads only the listed sources (`DBContextManager::setLoadOnlyDataSources`).
- `"all"` loads all data sources and lines.
- `data_source_types` (e.g. `["ADSB", "RefTraj"]` or `"all"`) is applied independently - it restricts which DS types are enabled.
- The read side uses `get<map<unsigned int, set<unsigned int>>>()` which also accepts a JSON object with string keys (`{"12750": [0, 1]}`), but the write side always produces array-of-pairs.

**Key source files**:
- Write: `FilterManager::getFilterState()` in `filtermanager.cpp` - serializes via `ViewPoint::VP_DS_KEY`
- Read: `FilterManager::showViewPointSlot()` in `filtermanager.cpp` - deserializes and calls `setLoadOnlyDataSources`
- Constant: `ViewPoint::VP_DS_KEY` = `"data_sources"` in `viewpoint.h`

**Key source files**:
- Write: `FilterManager::getFilterState()` in `filtermanager.cpp` - serializes via `ViewPoint::VP_DS_KEY`
- Read: `FilterManager::showViewPointSlot()` in `filtermanager.cpp` - deserializes and calls `setLoadOnlyDataSources`
- Constant: `ViewPoint::VP_DS_KEY` = `"data_sources"` in `viewpoint.h`

## Labels (`labels`)

A view point can pin a label on individual target reports with the `"labels"` key of `load_data`. The value is an array of `[rec_num, level]` pairs, where `rec_num` is the record number of the target report and `level` is its level of detail, 1 to 3 (how nlohmann/json serializes `map<unsigned long, unsigned int>`). The level is set per label, so one view point can mix levels.

```json
{
    "labels": [
        [529386773, 3],
        [572269077, 3]
    ]
}
```

- Level 1 draws a 1x1 matrix, level 2 a 2x2 matrix, level 3 a 3x3 matrix. The section "Automatic Labeling" in `doc/user_manual/geographicview/geo_labels_tab.tex` lists the content of each cell.
- Record numbers are the `record_number` values of the DBContent tables. They are unique across the database, so the DBContent and the data source do not have to be named.
- A record number that is not part of the loaded data is skipped without an error.
- The read side accepts a JSON object with string keys as well (`{"529386773": 3}`).

**Independent of automatic labeling.** The pinned labels are drawn whether the Geographic View labels targets automatically or not. Automatic labeling puts a label on every loaded target, at the level of detail configured in the view. That is unusable for a figure with many targets. For report figures, switch automatic labeling off and pin the few labels that carry information with this key.

**Errors.** `GeographicView::updateViewPointLabels()` throws if `labels` is neither an array nor an object, or if a level lies outside 1 to 3. It catches the error, reports it as `label error: <what>` through `ViewableDataConfig::reportError()`, and clears all pinned labels of that view point. The rest of the view point is still shown.

**Lifetime.** The labels live in the `LabelGenerator` while the view point is shown. The Geographic View applies them to the geometry layers after every rebuild. It removes them when the view point is unshown.

**Key source files**:
- Constant: `ViewPoint::VP_LABELS_KEY` = `"labels"` in `viewpoint.cpp`
- Read: `GeographicView::updateViewPointLabels()` in `geographicview.cpp`
- Storage: `LabelGenerator::setViewPointLabels()` / `clearViewPointLabels()` in `labelgenerator.h/.cpp`
- Apply: `GeometryItemProvider::finalizeContent()` calls `applyViewPointLabels()` on every layer
- Draw: `GeometryItemGroupLabels::applyViewPointLabels()` pins the level with `setCustomLOD()`

## Sectors (`show_sectors`)

A view point can restrict the sector layers that the Geographic View shows with the `"show_sectors"` key. The value is an array of sector layer names, as defined in the active Data Context. The Geographic View hides all sector layers, then shows the listed ones. Without the key the sector layers keep their current visibility.

```json
{
    "show_sectors": ["TMA", "CTR"]
}
```

**Errors.** `GeographicViewDataWidget::loadingDone_impl()` throws if `show_sectors` is not an array of strings. It catches the error, reports it as `sector error: <what>` through `ViewableDataConfig::reportError()`, and leaves the sector layers untouched. A name that matches no sector layer of the active Data Context is reported as `sector error: unknown sector layer '<name>'` and skipped. The other listed layers are still shown. Both checks run in phase 2, after the load.

**Key source files**:
- Read: `GeographicViewDataWidget::loadingDone_impl()` in `geographicviewdatawidget.cpp`
- Apply: `OSGSectorRootTreeItem::showSectors()` in `sector/osgsectorroottreeitem.cpp`, returns the unknown names

## Grid Background (`grid_background`)

A view point with a longitude / latitude grid can carry the background of its grids with the `"grid_background"` key (`ViewPointGenFeatureGrid::ViewPointFieldNameBackground`). The key is in the view point, next to `annotations`, not in the grid feature. The value is the JSON form of `GridBackgroundSettings` (`src/view/gridview/gridbackgroundsettings.h`). A missing field keeps its default.

```json
{
    "grid_background": {
        "show": true,
        "color": "#000000",
        "map_show": false,
        "map_file": "arcgis_2d.earth",
        "map_opacity": 50,
        "borders_show": true,
        "sectors_show": true,
        "hidden_sector_layers": ["TMA"],
        "hidden_sectors": []
    }
}
```

- Writers: the evaluation (`Base::createViewable()`, the evaluated sector layer of the result shown) and the data source analysis (`AnalyzeDataSourceTask`, no sector shown). They write the key only into view points with a longitude / latitude grid (`report_figure::hasGeographicGrid()`).
- Report figures: the grid render widget draws the background of the key. A figure without the key uses `GridBackgroundSettings::reportDefaults()` (national borders on black, map off, all sectors).
- Grid View: showing the view point copies the background into the settings of the view (`GridView::showViewPoint()`), so the layer panel shows it and the user can change it.
- Text features (`text`) of a view point are the labels of each longitude / latitude grid of that view point, in the report figure and in the Grid View. Grids over other axes get no labels.
- An unreadable value is ignored with a warning, the defaults apply.

## Collection Content Version

A view point collection file carries `"content_type": "view_points"` and `"content_version"`. `ViewPoint::isValidJSON()` compares the version against `ViewPoint::VP_COLLECTION_CONTENT_VERSION` in `viewpoint.cpp` and accepts that value only. The current value is **0.4**. An older file fails the import with `current data content version is not supported`, so a generator script has to be updated when the constant changes.

## Annotation-Only View Points (e.g. Grids)

A view point that only displays annotations - e.g. 'grid' features shown in a Grid View, or geometry overlays - has no `load_data`. Setting it loads no DBContent data (`FilterManager::showViewPointSlot()` enables no DS type and no data source and disables the filters), so no target reports are displayed on top of the annotation content.

## Files

| File | Role |
|------|------|
| `src/view/points/viewabledataconfig.h` | Error accumulator: `reportError`, `hasErrors`, `errors` |
| `src/view/points/viewpoint_commands.cpp` | `checkResult_impl()` reads errors, populates command reply |
| `src/view/points/viewpoint.cpp` | JSON keys and `isValidJSON()`, including the collection content version |
| `experimental_src/.../geographicview.cpp` | `updateViewPointLabels()`: parses the `labels` key, reports label errors |
| `experimental_src/.../geographicviewdatawidget.cpp` | `loadingDone_impl()`: parses the `show_sectors` key, reports sector errors |
| `experimental_src/.../sector/osgsectorroottreeitem.cpp` | `showSectors()`: shows the listed sector layers, returns the unknown names |
| `src/db/dbcontent/label/labelgenerator.h/.cpp` | Holds the pinned view point labels while the view point is shown |
| `experimental_src/.../annotations/osgannotationsroottreeitem_viewpoint.cpp` | Viewpoint catch site: per-annotation try/catch, reports errors |
| `experimental_src/.../annotations/osgannotationstreeitem_viewpoint.cpp` | Viewpoint annotation `build()`: throws on bad JSON |
| `experimental_src/.../annotations/osgannotationsroottreeitem_internal.cpp` | Internal catch site: try/catch + `traced_assert(false)` |
| `experimental_src/.../annotations/osgannotationstreeitem_internal.cpp` | Internal annotation `build()`: throws on bad JSON |
| `experimental_src/.../drawable/drawablefactory.cpp` | Drawable creation: throws on bad JSON |
| `experimental_src/.../drawable/drawablefeature.cpp` | Feature base `fromJSON`: throws on type errors |
| `experimental_src/.../drawable/drawablearray.cpp` | Array drawable `fromJSON`: throws on missing geometry |
| `experimental_src/.../drawable/drawableellipses.cpp` | Ellipses `fromJSON`: throws on bad sizes |
