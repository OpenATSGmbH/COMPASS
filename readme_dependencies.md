# Resolving the circular dependencies between the COMPASS libraries

*Design document / plan. Status: Workstreams A–D implemented (2026-11-03), E and F (2026-11-04), no include violations left in `src/` (from 270), the allowlist is empty. G (structural guards, 2026-11-04): link graph and include paths follow the layering, an upward include is a compile error. H (2026-11-04): `COMPASS` offers only the core managers and the provider interfaces. I (sub-layers inside core): planned. Written 2026-11-03 against the then-current code (line numbers refer to that state).*

## Context

COMPASS is built from ten module libraries in `src/` (`compass_app`, `config`, `core`, `eval`, `filter`, `gui`, `reconstruction`, `report`, `task`, `view`) plus six in `experimental_src/`. All of them are static archives, and they depend on each other in cycles. Static archives tolerate this. The repeated archives in [src/app/client/CMakeLists.txt:40-48](src/app/client/CMakeLists.txt#L40) exist only to satisfy the link order.

Shared libraries do not tolerate cycles. CMake allows cyclic dependencies only between static libraries, and on Windows a DLL cannot even be linked before the import libraries of all its dependencies exist. The plugin plan ([readme_plugins.md](readme_plugins.md)) needs shared libraries.

**This plan removes the cycles between the `src/` module libraries, as a preparation for the plugin plan.** It ends with static libraries whose dependencies form a directed acyclic graph, kept that way by the link and include paths of the libraries (an upward include does not compile) and a layering check that is run manually.

**Out of scope:**
- turning the libraries into shared libraries, and the host's references to ProbIMM and the Geographic View in `experimental_src` (plugin plan);
- separating compute code from UI code (compute core plan, [readme_compute_core.md](readme_compute_core.md)).

### Execution order (decided 2026-11-03)

1. **This plan**: now.
2. **Plugin plan**: next.
3. **Compute core plan**: in the near future.

## Findings

### Measured include dependencies

Number of files in the row module that include at least one header of the column module. `src/` only, unit tests and the watchdog excluded. `src/db` is counted as `core` and `src/import` as `task`, as in the CMake files. Includes were resolved by file name, so the numbers are approximate.

| from \ to | app | config | core | eval | filter | gui | reconstr. | report | task | view |
|---|---|---|---|---|---|---|---|---|---|---|
| **app** | | 3 | 13 | 3 | 2 | 5 | | | 6 | 3 |
| **config** | | | 7 | | | | | | | |
| **core** | | 17 | | 4 | 5 | 8 | | | 7 | 8 |
| **eval** | 1 | 6 | 136 | | 3 | 6 | | | 42 | 25 |
| **filter** | | 4 | 42 | | | 2 | | | | 1 |
| **gui** | | | 19 | | | | | | | |
| **reconstr.** | | | 13 | | | | | | | |
| **report** | | | 6 | | | | | | 2 | 1 |
| **task** | 3 | 23 | 142 | 11 | 2 | 20 | 10 | 1 | | 17 |
| **view** | 1 | 8 | 106 | 3 | 2 | 9 | | 6 | | |

The matrix misses the most important edge: `compass.h` is counted under core, but `COMPASS` is **defined** in [src/app/compass.cpp](src/app/compass.cpp). About 190 files include `compass.h` (core/db 58, task 54, view 46, eval 17, filter 4, report 2), so at link level almost every library depends on `compass_app`.

### The cycles, grouped by cause

| # | Edge | Files | Cause |
|---|---|---|---|
| 1 | everyone → app | ~190 | `COMPASS` is a service locator defined in `compass_app`. It owns and constructs every manager, includes every manager header and creates the `MainWindow`. |
| 2 | eval ↔ task | 42 / 11 | Mostly the **report model** (`section.h` 35, `sectioncontenttable.h` 36, `report.h` 32), which lives in [src/task/result/report/](src/task/result/report/). In addition, evaluation results are `TaskResult`s stored in `TaskManager` ([evaluationtaskresult.h:38](src/eval/results/evaluationtaskresult.h#L38), [evaluationresultsgenerator.cpp:145](src/eval/results/evaluationresultsgenerator.cpp#L145), [single.cpp:269](src/eval/results/base/single.cpp#L269)), and `EvaluationManager` connects to a `TaskManager` signal ([evaluationmanager.cpp:169-171](src/eval/evaluationmanager.cpp#L169)). |
| 3 | report model → task | – | The report model calls back into its owner: `Report::taskManager()` for navigation and loading ([report.cpp:330-355](src/task/result/report/report.cpp#L330)). `TaskResult` and the sections include each other. |
| 4 | view ↔ report | 6 / 1 | Double dispatch: views call `LatexVisitor` (`latexvisitor.h` in 5 views), and the visitor includes the view headers. `viewpointsreportgenerator.cpp` uses `latexdocument.h` directly. |
| 5 | core/db → view, task, eval, filter | ~20 | **Model code** calling upward through `COMPASS`: `DBContentManager` → `ViewManager` ([dbcontentmanager.cpp:228,247,491,936-953,1290](src/db/dbcontent/dbcontentmanager.cpp#L228)), `TargetModel` → `EvaluationManager` ([targetmodel.cpp:112,145,172,569](src/db/dbcontent/target/targetmodel.cpp#L112)), `LabelGenerator` → `ViewManager::currentBuffers()` ([labelgenerator.cpp:1603](src/db/dbcontent/label/labelgenerator.cpp#L1603)), `DBContentDataEngine` → `FilterManager::viewClause()` ([dbcontentdataengine.cpp:726](src/db/dbcontent/dbcontentdataengine.cpp#L726)), `DBInterface` → `TaskManager` ([dbinterface.cpp:2936](src/db/interface/dbinterface.cpp#L2936)), `viewsBusy()` → `ViewManager` ([rtcommand_runner_stash.cpp:57](src/core/rtcommand/rtcommand_runner_stash.cpp#L57)), `dbcontent_commands.cpp` → `ViewManager`. Found by the layering check (A): **task result persistence** in `DBInterface` and `SQLGenerator` (`TaskResult`, report model, `EvaluationTaskResult`, `AsyncTask`), `reportvariable.h` → `reporttable.h`, `DBContentAccessor` → `ReconstructorBase`, `TargetModel` → `ReconstructorTarget`. **Widgets** in core/db calling upward: `datasourceswidget.cpp`, `datasourcesstatuswidget.cpp`, `targetlistwidget.cpp`, `asterixconfigwidget.cpp`. |
| 6 | core ↔ gui | 8 / 19 | Core/db files use a handful of **generic widgets** from `src/gui`: `questiondialog.h` (3 files), `textfieldhexvalidator.h` / `textfieldoctvalidator.h`, `waitspinner.h`, `rangeedit.h`, `popupmenu.h`, `dialogs.h` (in `async.cpp`). In the other direction, `src/gui` uses util headers and a few core headers (`property.h`, `colormap.h`, `ui_test_*.h`, `toolboxwidget.h`, `idatasourceprovider.h`). |
| 7 | config → core | 7 | Utilities only: `traced_assert.h`, `logger.h`, `stringconv.h`, `result.h`. |
| 8 | small leftovers | 1–5 each | view → eval (`evaluationmanager.h` in `histogramview.cpp`, `histogramviewdatawidget.cpp`, `variableview.cpp`), filter → view (`ViewPoint::VP_*_KEY` constants, [filtermanager.cpp:28](src/filter/filtermanager.cpp#L28)), `mainwindow.h` in five files (`evaluation_commands.cpp`, `asteriximporttask.cpp`, `analyzedatasourcetask.cpp`, `taskmanager.cpp`, `viewpointswidget.cpp`). |

Only #1 and #2/#3 are structural. The others are local call sites.

**Not part of this plan:** the cycles between the host and `experimental_src`. `compass_reconst_adv` links `compass_task`, which creates `ProbIMMReconstructor`; `compass_geo_view` links `compass_view`, `compass_report` and `compass_task`, which name `GeographicView`. These are the host sites the plugin plan resolves with its registries.

## Target layering

Bottom to top. Each library may only depend on libraries below it.

```
compass_app          composition root, MainWindow
compass_task         tasks, TaskManager (incl. import)
compass_eval         evaluation
compass_result       TaskResult, result store, report model, report export (LaTeX, JSON, DocX)
compass_view         views, ViewManager, view point model
compass_filter       compass_reconstruction
compass_gui          shared widgets
compass_core         base, buffer, property, projection, rtcommand, job, license, source, context, db,
                     thin COMPASS, the generic widgets core uses
compass_config
compass_util         logger, traced_assert, message handler, string/time conversion, Result, file paths,
                     Property / PropertyList
```

- **`compass_result` is new.** It combines today's [src/report/](src/report/) with `TaskResult` and the report model from [src/task/result/](src/task/result/). These parts include each other and belong together.
- **`compass_util` is new as well.** It is [src/core/util/](src/core/util/) and holds the utilities `compass_config` and `compass_gui` need.
- **Directory = library**, as everywhere else in the project. A file that belongs to another library moves (`git mv`) to that library's directory, and each directory's CMakeLists lists only its own files. Includes by bare file name keep working; includes with a directory part (e.g. `"util/async.h"`) are adjusted.

## Workstreams

Order: A → B → C → D, E, F (any order). Every step leaves `compass_client` building and all tests green.

### A: Measurement + guardrail (~2 d) — implemented

- **[scripts/check_layering.py](scripts/check_layering.py)** maps every `src/` file to its library in the target layering and checks every include against it.
  - The mapping (`LIBRARY_RULES`) is the target assignment of this plan: path patterns, first match wins.
  - A header whose implementation lives in another directory can be attributed to the implementation's library (`IMPLEMENTED_IN`). This was the case for `compass.h` (implemented in `src/app/`) until Workstream C; the map is empty now.
  - Includes are resolved by path suffix against the headers in `src/`. Include names are ambiguous only within `eval`, which does not matter here.
- **[scripts/layering_allowlist.txt](scripts/layering_allowlist.txt)** lists the known violations as `<including file> -> <included header>`. The initial list has 270 entries.
  - The check fails on any violation not in the list and on any entry that no longer occurs, so the list can only shrink.
  - After fixing violations, regenerate it with `--write-allowlist`.
- **Run manually**, not as a CI stage (decided 2026-11-04; the Jenkins stage `Layering` was removed again). With G1/G2 the compiler and the linker already reject upward dependencies in every build.
- **Further options:** `--matrix` prints the library include matrix, `--experimental` lists the 34 includes into `experimental_src/`, which are not checked (they belong to the plugin plan), and `--graph FILE` writes the library dependency graph of the current mode (`.dot`, or `.svg`/`.png`/`.pdf` rendered with Graphviz; violations in red, `experimental_src` dashed). With `--graph-reduced` the graph omits edges implied by other paths (transitive reduction; violations are always kept).
- **When a workstream moves files** to their library's directory, the directory rules in `LIBRARY_RULES` cover them automatically. The special rules for single files (now only the result files) are removed once those files have moved.

### B: `compass_util` (~3 d) — implemented

Removes cycle #7 and the util part of #6.

- **New library `compass_util` = [src/core/util/](src/core/util/)**, added before `src/config` in the top-level CMakeLists. The file set was computed with the layering check as the largest self-contained one; it has no upward includes.
- **File moves:**
  - `property.{h,cpp}` and `propertylist.{h,cpp}` from `src/core/property/` into `src/core/util/`. They are needed through `traced_assert` → `msghandler` → `logmodel`. `property_templates.h` needs `nullablevector.h` and stays in `src/core/property/`;
  - `config_paths.h` from `src/config/` into `src/core/util/`, needed by `files.h`;
  - `geotiff.{h,cpp}` and `geotiff_defs.{h,cpp}` from `src/core/util/` into `src/core/projection/`, since they need projection;
  - `async.{h,cpp}` from `src/core/util/` into `src/core/job/`, since it needs a gui dialog. Its nine `"util/async.h"` includes become `"async.h"`.
- **`compass_config` links `compass_util`** instead of exposing `src/core/util` as an include directory.
- **Include fix:** `event_log.cpp` and `stringconv.cpp` included `"util/timeconv.h"`, which only resolved through `compass_core`'s include directories. They now include `"timeconv.h"`.

### C: Thin `COMPASS` + composition root (~2 wk) — implemented

This is the central workstream. It removes cycle #1.

**As implemented** (decisions 2026-11-03):
- **`COMPASS`** moved to [src/core/compass.cpp](src/core/compass.cpp).
  - It is a pure access point: it owns no manager and holds pointers to all of them in one struct, `COMPASS::Managers`, filled by the root.
  - It stays the configuration parent of the managers, so the config tree is unchanged.
  - It keeps `SimpleConfig`, `LogStore`, the DB lifecycle, app mode and settings. All accessors are unchanged.
- **[CompassRoot](src/app/compassroot.h)** in `src/app/`, created and owned by `Client`. It owns `COMPASS`, all managers (the eleven sub-configurable ones, `DBContextManager`, `RTCommandRunner`, the variable resolver) and the `MainWindow`.
- **Startup:**
  1. the `COMPASS` constructor registers the parameters only;
  2. the root registers a creator per manager (`registerManagerCreator()`), in the old creation order of missing managers (License, DBInterface, DBContent, Filter, Task, View, Eval, Job, Unit, Projection, RTCommand);
  3. `COMPASS::createManagers()` runs `createSubConfigurables()`;
  4. `CompassRoot::startManagers()` wires the variable resolver, starts jobs and rtcommands, and creates runner and context manager;
  5. the root wires all managers (`connectManagers()`, same connection order as before);
  6. `CompassRoot::init()` runs the task manager's `init()`;
  7. `CompassRoot::mainWindow()` creates the main window on first use.
- **Shutdown:**
  - `COMPASS::shutdown()` sets the state and runs `CompassRoot::shutdown()`, the old sequence in one function. It nulls each `Managers` pointer before resetting the manager.
  - The root's destructor then destroys the remaining managers in the order of the former `COMPASS` member destruction (runner, resolver, rtcommand, projection, unit, job), and `COMPASS` last. The managers are its sub-configurables and unregister from it.
- **App mode:** the ASTERIX importer is connected to the new `COMPASS::appModeAboutToSwitchSignal`, emitted right before `appModeSwitchSignal`, so it still runs before all other slots.
- **Interfaces:**
  - `liveImportBacklogSize()` is answered by a function the root registers;
  - `COMPASS::mainWindow()` returns [IMainWindow](src/core/imainwindow.h) (`showResult()`, `importViewPointsSlot()`, `liveStopSlot()`, `updateMenus()`);
  - the view lookup facades forward to [IViewWidgetProvider](src/core/iviewwidgetprovider.h), implemented by `ViewManager`.
- **Removed:** the unused `TaskManager::getMainWindow()` and the unused `mainwindow.h` include in `evaluation_commands.cpp`. `mainwindow_commands_import.cpp` and the test lab reach the main window via `rtcommand::mainWindow()`.
- **Additional move:** `DBContentManagerVariableResolver` from `src/filter/` into `src/db/dbcontent/`. It only depends on dbcontent and is used by `COMPASS`.
- **Verified:** build, unit tests, client start → `create_db` → quit with the full shutdown sequence. The app mode switch (live mode) has not been exercised yet.

The original design notes follow.

- **`COMPASS` stays the access point, but stops owning and constructing the managers.** It moves into `compass_core` and keeps all accessors (`viewManager()`, `taskManager()`, …). The managers are only forward-declared there; the accessors return references to pointers that are set at startup. None of the ~190 call sites change.
- **Ownership and construction move into a composition root in `compass_app`.**
  - Today `COMPASS::generateSubConfigurable()` creates the managers by class name. Instead, it delegates to a creator map registered by the root.
  - The config tree is unchanged: the managers remain sub-configurables of `COMPASS`.
- **The root runs explicit, ordered startup steps:**
  1. create `COMPASS`;
  2. register the manager creators;
  3. instantiate the config tree;
  4. `init()` of the managers;
  5. create the `MainWindow`.

  Shutdown runs in a fixed order as well.
- **Lifecycle methods** (`openDBFile*`, `closeDB*`, `appMode()`, `init()`, `shutdown()`) only call managers below `COMPASS` directly. Calls into upper managers become signals the managers subscribe to, or move into the root.
  - Some of these signals already exist (`databaseOpenedSignal`, `databaseClosedSignal`, `appModeSwitchSignal`).
  - The direct calls to upper managers are few: `task_manager_->init()/shutdown()`, `view_manager_->close()`, `eval_manager_->close()`, `hasPendingViewProcessing()` ([compass.cpp:1023-1073](src/app/compass.cpp#L1023)), and the `sectorsChanged` connects ([compass.cpp:225-228](src/app/compass.cpp#L225)).
- **Order matters.** Today the shutdown order is explicit. With signals it would be the connection order, so `init()`/`shutdown()` stay explicit, ordered calls in the root rather than signals.
- **`mainWindow()` moves to the root.** `COMPASS::mainWindow()` ([compass.cpp:1102](src/app/compass.cpp#L1102)) constructs the `MainWindow` today. Its callers, including the five `mainwindow.h` includes of #8 and `test_lab_phil.cpp`, get a narrow interface in core that `MainWindow` implements, containing only the functions these call sites use.
- **The `QWidget*` view facades** ([compass.cpp:840-852](src/app/compass.cpp#L840)) are used by the rtcommand helpers in core to navigate the view tree. They become a narrow interface in core, implemented by `ViewManager` and registered by the root.

### D: core/db upward calls + generic widgets (~1.5 wk) — implemented

**As implemented** (decisions 2026-11-03):
- **Unused includes removed:**
  - `filtermanager.h` in `dbcontent.cpp` and `dbcontentmanager.cpp`;
  - `view/viewmanager.h` in `dbcontent_commands.cpp`;
  - `reconstructorbase.h` in `dbcontentaccessor.cpp`;
  - `taskmanager.h` and `evaluationcalculator.h` in `targetlistwidget.cpp`.
- **Types moved to the library that uses them:**
  - `filterclause.h` and `viewabledataconfig.h` into `src/db/dbcontent/`;
  - `reporttable.{h,cpp}` into `src/db/interface/`;
  - `asterix{,ref,spf}editioncombobox.h` into `src/core/context/`;
  - the generic widgets into `src/core/gui/` (before C).
- **Interfaces in core,** implemented by the upper managers and reached via `COMPASS`; the root sets the pointers in `COMPASS::Managers`:
  - [IViewProvider](src/core/iviewprovider.h) (`ViewManager`, `viewProvider()`) replaces `IViewWidgetProvider`. It covers pending/processing state, `clearDataInViews()`, `getReadSet()`, view point, selection, latency, `currentBuffers()` and the widget lookups;
  - [IFilterProvider](src/core/ifilterprovider.h) (`FilterManager`, `filterProvider()`): `useFilters()`, `viewClause()`, `filterBuffers()`;
  - [IEvalProvider](src/core/ievalprovider.h) (`EvaluationManager`, `evalProvider()`): read set variables, timestamp and target filter, requirement names, and the four evaluation dialogs of the target list. `TargetListWidget` stays in db and uses only this interface.
- **Relayed view signals:** `ViewManager::dataDistributedSignal` and `loadingDoneSignal` are relayed via `COMPASS::viewDataDistributedSignal` and `viewLoadingDoneSignal` (connected by the root). They are used by `DataSourcesStatusWidget` and `TargetListWidget`.
- **`TargetModel::createNewTargets()`** takes `dbContent::Target`s. `ReconstructorBase::saveTargets()` fills them from the `ReconstructorTarget`s, with the code moved unchanged.
- **Left for later:**
  - task result persistence in `DBInterface`/`SQLGenerator` → Workstream E;
  - the `ViewPoint::VP_*_KEY` constants used by `DBContentManager` and `DBInterface` → Workstream F.
- **Verified:** build, unit tests, client start → `create_db` → quit. Not yet exercised: data loading with filters, live mode, the target list dialogs, saving the targets of a reconstruction.

The original design notes follow.

Removes cycles #5 and #6.

- **Model code: invert the direction** at the ~10 call sites of #5:
  - `DBContentManager` → `ViewManager` (clear views, view point, selected rec nums, latency, pending processing): signals from `DBContentManager` that `ViewManager` handles, plus a small `ViewStateProvider` interface in core for the queries (`hasPendingViewProcessing()`, `currentBuffers()`, `maxLatency()`), implemented by `ViewManager` and registered by the root.
  - `DBContentDataEngine` → `FilterManager::viewClause()`: a `FilterClauseProvider` interface in core, implemented by `FilterManager`.
  - `TargetModel` → `EvaluationManager` (timestamp filter, target filter): the target model receives the filter state from outside, or the eval-specific parts move into eval.
  - `DBInterface` → `TaskManager` ([dbinterface.cpp:2936](src/db/interface/dbinterface.cpp#L2936)) and `viewsBusy()`: decide per site.
  - **Task result persistence** in `DBInterface` and `SQLGenerator` (`TaskResult`, report model, `EvaluationTaskResult`, `AsyncTask`), and `reportvariable.h` → `reporttable.h`. Either the result-specific part moves into `compass_result` on top of a generic `DBInterface` API, or `TaskResult` serializes itself and `DBInterface` only stores the data. Decide when D starts.
  - `DBContentAccessor` → `ReconstructorBase` and `TargetModel` → `ReconstructorTarget`: decide per site.
- **Widgets in core/db calling upward** (`datasourceswidget`, `datasourcesstatuswidget`, `targetlistwidget`, `asterixconfigwidget`): decide per file.
  - If nothing in core constructs the widget, assign it to the lowest module whose code it uses.
  - Otherwise, its owner keeps a construction hook that the root fills, or the widget gets the needed state passed in.
- **Generic widgets used by core** (#6) — done before C (`compass.cpp` uses `dialogs.h`): `questiondialog`, `textfieldhexvalidator`, `textfieldoctvalidator`, `waitspinner`, `rangeedit`, `rangeslider`, `popupmenu` and `dialogs` moved from `src/gui/` into [src/core/gui/](src/core/gui/). `compass_gui` then sits above core, and its remaining core dependencies point downward.

### E: `compass_result` (~1.5 wk) — implemented

Removes cycles #2, #3 and #4.

As implemented:

- **[src/result/](src/result/)** is the directory of `compass_result`. It holds `TaskResult`, `taskdefs.h` and the report model, exporters and widgets ([src/result/report/](src/result/report/)). `src/report/` is gone. `#include "task/result/..."` became `#include "result/..."`.
- **Moved down:** `asynctask` into [src/core/job/](src/core/job/); `taskresultswidget` into [src/task/](src/task/) (it is the task manager's UI).
- **`ITaskProvider`** ([itaskprovider.h](src/result/itaskprovider.h)) replaces the planned `ResultHost` and `TaskResultStore`: one interface with the result API that the report model, `TaskResult` and eval use. `TaskManager` implements it, eval reaches it via `COMPASS::taskProvider()`.
- **LaTeX code, instead of the planned `ViewVisitor`:** the LaTeX code only needs core and the views, not the result model, so it stays below result and no visitor interface is needed.
  - The LaTeX core (`latexdocument`, `latexsection`, `latextable`, ...) is in [src/core/latex/](src/core/latex/), part of `compass_core`.
  - `ResultReport::Colors` (report colors plus their LaTeX color defines, used by `LatexDocument`) moved unchanged from `reportdefs.h` into [src/core/latex/reportcolors.h](src/core/latex/reportcolors.h). `reportdefs.h` includes it, so its users are unchanged.
  - `LatexVisitor`, `viewpointsreportgenerator` and its dialog are in [src/view/points/report/](src/view/points/report/), part of `compass_view`. `ViewManager` keeps owning the generator, so the config tree is unchanged.
- **Task result persistence:** moved from `DBInterface`/`SQLGenerator` into [TaskResultDB](src/result/taskresultdb.h) in `compass_result`, owned by `TaskManager`. The logic and the table schema are unchanged.
  - `DBInterface` got generic table operations for it: `createTable(table, properties, primary_key)`, `deleteRows()`, `updateCell()`, `getMaxUIntValue()`. Reads use the existing `select()`.
  - The result-specific create statements left `SQLGenerator`.
  - `TaskResultDB` creates the loaded results via `ITaskProvider::createResult()`.
  - The report content ID (`Section::setCurrentContentID()`) is set in `TaskManager::databaseOpenedSlot()` before the results are loaded, instead of in `DBInterface` on open. No earlier `databaseOpenedSignal` slot creates report contents.
  - The report-table functions (`createReportTable()`, `reportTableNames()`, ...) stay in `DBInterface`, they only use `ReportTableDefinition` from db.

Originally planned:

- **Move** `TaskResult`, the report model (`report`, `section*`, `sectionid`, `reportitem`, `reportdefs`, `treeitem`, `treemodel`), the report exporters and the report widgets from `src/task/result/` into the directory of `compass_result`, together with [src/report/](src/report/). Which directory that is (today's `src/report/`, or a new one) is decided when E starts. Path-qualified includes such as `"task/result/report/section.h"` are adjusted.
- **`ResultHost` interface** in `compass_result` for what the report model and `TaskResult` need from their owner today: `setViewableDataConfig()`, `unsetViewableDataConfig()`, `selectID()`, `loadContent()` ([report.cpp:330-355](src/task/result/report/report.cpp#L330)). `TaskManager` implements it.
- **Result store.** The result part of the `TaskManager` API that eval uses (`beginTaskResultWriting()`, `hasCurrentResult()`, `hasResult()`, `result()`) becomes a `TaskResultStore` in `compass_result`. `TaskManager` owns it and eval reaches it through `COMPASS`. The `taskRadarPlotPositionsDoneSignal` connect in `EvaluationManager` moves into the root.
- **View visitor.**
  - An abstract `ViewVisitor` in the view base, with a virtual `visit()` per host view type. Views call it, and `LatexVisitor` in `compass_result` implements it.
  - `viewpointsreportgenerator` and its dialog move to `compass_result`.
  - The `GeographicView` branch of `LatexVisitor` stays as it is, under its existing `USE_EXPERIMENTAL_SOURCE` guard.

### F: Small leftovers (~3 d) — implemented

As implemented:

- **View point keys:** the `VP_*` constants moved unchanged from `ViewPoint` into `ViewPointKeys` ([src/core/viewpointkeys.h](src/core/viewpointkeys.h)). All uses changed from `ViewPoint::VP_*` to `ViewPointKeys::VP_*`; db and filter include `viewpointkeys.h` instead of `viewpoint.h`.
- **view → eval:** the `evaluationmanager.h` includes in `histogramview.cpp`, `histogramviewdatawidget.cpp` and `variableview.cpp` were unused leftovers and are removed, so no interface was needed.
- `check_layering.py` reports no violations for `src/`, the allowlist is empty.

Originally planned:

- `ViewPoint::VP_*_KEY` constants into a header below filter (e.g. `viewpointkeys.h` in core).
- view → eval in the histogram and variable views: an interface in view that eval implements, or move the eval-specific parts into eval.
- Whatever else `check_layering.py` still reports for `src/`.

### G: Structural guards (~1 wk) — implemented

After A–F the includes are acyclic, but the build setup still allows new cycles: the CMake link graph is cyclic and include paths reach into higher layers. G makes the compiler and the linker enforce the layering instead of only the layering check.

As implemented:

- **G1:** `compass_view` links core, gui, filter; `compass_result` core, gui, view; `compass_eval` core, reconstruction, gui, filter, view, result; `compass_task` additionally view and eval. The link repetitions in `compass_client` and `compass_tests` ("static archive link order") are gone. Without them two hidden link dependencies showed up:
  - `APP_FILENAME`, declared in `global.h` (util) but defined in `client.cpp` (app) and used by eval: now defined in [src/util/global.cpp](src/util/global.cpp), `Client` still sets it.
  - `compass_reconstruction` (`kalman_estimator.cpp`) uses the Kalman interfaces of `compass_reconst_adv`: declared at its origin as link-only dependency (`$<LINK_ONLY:compass_reconst_adv>`, no include paths for consumers), a known host ↔ experimental cycle for the plugin plan.
- **G2:** the upward PRIVATE include paths of core, eval, task, filter and view are removed, as are the global include paths `src/` ([src/CMakeLists.txt](src/CMakeLists.txt) deleted) and the repository root. 987 includes in 377 files lost their module prefix (`eval/` 569, `result/` 237, `util/` 158, `view/` 20, `task/` 3), each verified to resolve to exactly one header. An upward include in a host library is now a compile error. Unchanged: the include paths of the host libraries into `experimental_src/` and of the experimental libraries into `src/` (plugin plan).
- **G3:** `check_layering.py --build-dir <dir>` reads `ninja -t deps` or `compiler_depend.make`/`depend.make`, attributes each object to the library of its CMake target and checks every included header, also transitive ones. The CMake check (link libraries, include directories, global include paths) runs in both modes. The check is run manually (`--build-dir build` locally, `--build-dir build_deb10` on a Docker build), it is not a CI stage.
- **G6:** `src/core/util` → `src/util`, `src/db` → `src/core/db`, `src/import` merged into `src/task/import`.
- **G7:** [CLAUDE.md](CLAUDE.md) has a section "Library layering" (layer table, rules, check, exceptions), an updated project structure and the composition root.

Decided:

- **G1: link graph follows the layering.** Each library links only libraries below it. Today `compass_view` links `compass_eval`, `compass_eval` links `compass_task`, `compass_result` links `compass_task` and `compass_eval`; with `compass_task` → `compass_result` that is still a link cycle (tolerated by static libraries, not by the shared libraries of the plugin plan). PUBLIC links also pass on include paths, so through the cycle every library sees the headers of all others.
- **G2: no include paths into higher layers.**
  - Remove the PRIVATE include paths into higher modules: 57 in `compass_core` (eval 29, task 10, view 8, …), view in `compass_filter`, `src/app/client` in `compass_view`, `compass_eval` and `compass_task`.
  - Remove `src/` as global include path ([src/CMakeLists.txt](src/CMakeLists.txt)): it makes every header reachable as `"module/.../x.h"`. The 833 includes of that form (eval 572, result 237, view 20, task 3, gui 1) and the `"util/x.h"` includes become relative to the include directories of their library (announced bulk edit).
  - A library then only sees its own headers and those below it, an upward include is a compile error.
- **G3: exact layering check.**
  - `check_layering.py` reads the resolved headers of every compiled file from the build directory (`ninja -t deps`, or `compiler_depend.make`/`depend.make` of the Makefile generator) instead of resolving includes by file name. Without a build directory it falls back to the include text analysis.
  - It also checks the CMake files: `target_link_libraries` and `target_include_directories` of the `compass_*` libraries must not point to higher layers, no global include path into `src/`.
  - Run manually, not as a CI stage.
- **G6: directory = library.** One directory per library, the layer is visible in every path:
  - `src/core/util` → `src/util` (`compass_util`);
  - `src/db` → `src/core/db` (part of `compass_core`);
  - `src/import/{asterix,json}` merged into `src/task/import/{asterix,json}` (part of `compass_task`).
  - Not changed: `experimental_src/analysis` stays compiled into `compass_task` (separate closed-source repository; as an own library it would be a host ↔ experimental cycle, which is the plugin plan's scope). `src/core/gui` (part of core) and `src/gui` (`compass_gui`) are already consistent.
- **G7: rules where they are read.** [CLAUDE.md](CLAUDE.md) gets the layer order, the pattern for upward calls (interface in the lower layer, implementation in the upper one, wiring in `CompassRoot`), "move files, no cross-directory CMake references" and the layering check; its outdated parts (`COMPASS` owning the managers, `src/report/`) are corrected.

### H: `COMPASS` without the upper managers (~1.5–2.5 wk) — implemented

Formerly future improvement "`COMPASS` as service locator for upper layers" (decided 2026-11-04 to implement). Core's API still names the upper managers: `viewManager()`, `taskManager()`, `evaluationManager()`, `filterManager()`, about 150 calls on `COMPASS` (about 35 in `experimental_src`). After G lower layers cannot use them, but the easiest path in the code still leads to any manager.

Goal: `COMPASS` offers only the core managers and the provider interfaces. Each upper manager declares the peers it uses.

Decided:

- **Peers by one setter per manager.** The managers are created in the order of the existing configuration file, so a peer created later cannot be passed to a constructor. `CompassRoot` calls `setManagers(...)` (references to all peers of that manager) right after `createManagers()`, before anything else runs:
  - `ViewManager::setManagers(FilterManager&)`;
  - `EvaluationManager::setManagers(ViewManager&, FilterManager&)`;
  - `TaskManager::setManagers(ViewManager&, EvaluationManager&, FilterManager&)`.
  
  Manager-to-manager connections in constructors that relied on the config order (`ViewManager` → `FilterManager` view point signals) move to `CompassRoot::connectManagers()`.
- **Code inside a library** reaches its own manager through the parent chain (`view_->viewManager()`, `task().manager()`), and the peers through its manager (`manager().viewManager()`).
- **`compass_result`** reaches the views through `ITaskProvider::viewManager()` (new, view is below result).
- **Runtime command modules** with a static `COMPASS*` (`evaluation_commands`, `analyze_commands`, `viewpoint_commands`, `geographicview_commands`) get a static pointer to their manager instead, set in their `init_*_commands()`.
- **App layer** (`MainWindow`, `Client`, `mainwindow_commands*`) reaches the managers via `CompassRoot` and its manager accessors. The composition root lives in the app layer, so this locator is only available at the top.
- **`experimental_src`:** the geo view uses `View::viewManager()` and its peer `filterManager()`; ProbIMM uses `task().manager()`; `TestLab` gets the task and view manager from `TestLabCollection::appendTestLabs()`. The geo view's check for ARTAS associations (`GeometryItemGroupLayer::canShowAssociations()`, an upward call to `CreateARTASAssociationsTask::wasRun()`) reads the DB property through a new `DBContentManager::artasAssociationsCreated()` in core, which `wasRun()` uses too.
- **Removed:** `COMPASS::viewManager()`, `taskManager()`, `evaluationManager()`, `filterManager()` (incl. const) and the four fields in `COMPASS::Managers`.

As implemented (2026-11-04):

- `setManagers(...)` is called at the start of `CompassRoot::startManagers()`. The accessors (`viewManager()`, `evaluationManager()`, `filterManager()`) assert that it ran. The peers are not reset during shutdown; the shutdown order is unchanged.
- `LoadController` gets the `ViewManager&` in its constructor, `EvaluationTaskResult` the `EvaluationManager&` (created by `TaskManager::createResult()`).
- App layer: `MainWindow(CompassRoot&)`, `main_window::init_commands(CompassRoot&)` with `main_window::root()` for the runtime commands, `Client` via its `root_`.
- `CreateARTASAssociationsTask::DONE_PROPERTY_NAME` is replaced by `DBContentManager::ARTASAssociationsCreatedProperty` (same value `artas_associations_created`), so the key exists only once.
- Verified: build, `compass_tests` (697 cases), smoke test (create DB, quit), `check_layering.py --build-dir build` 0 violations.

### I: Sub-layers inside `compass_core` (~3–5 wk) — planned

Formerly future improvement "Cycles inside `compass_core`" (decided 2026-11-04 to implement). 16 of its 22 subdirectories form one include cycle group (e.g. `db/dbcontent ↔ db/interface`, `db/dbcontent ↔ db/job`, `core/context ↔ core/source`, `compass.h`/`compass.cpp` as hub). The check cannot see cycles inside a library, and neither can the compiler.

Decided:

- **Guard: sub-layers in the check.** `compass_core` stays one library. `check_layering.py` gets sub-layers for core and checks them like the libraries (exact and text mode). No CMake restructuring; a split into libraries stays possible later.
- **App state out of `COMPASS`.** The application state and global settings (DB opened, app mode/state, dark mode, color mode, expert mode, UI styles, live overload, ...) move into a small object low in core. `COMPASS` keeps registering the config parameters, bound to that object's fields, so the config files stay identical.
- **Direct dependencies instead of `compass_.x()`.** Core subsystems get the managers they use (`DBInterface&`, `DBContentManager&`, `DBContextManager&`, `JobManager&`, the provider interfaces, ...) passed in by `CompassRoot`, instead of reaching them through `compass.h`.
- **Mutual pairs** are resolved one by one as in D/E (move files, small interfaces, signals): `db/dbcontent ↔ db/interface`, `db/dbcontent ↔ db/job`, `context ↔ source`, `db/dbcontent ↔ context`, `db/dbcontent ↔ source`, `rtcommand ↔ test`, plus the single upward edges (`net → source`, `projection → context`, `gui → test`, `job → gui`, `latex → license`).

The exact sub-layer order is fixed when I starts, from the code after H.

## Verification

1. **No behaviour change.** Every workstream is a refactoring. `compass_tests` and the CI integration tests pass after each one.
2. **Order-sensitive paths** after C and D: open/close DB, app mode switches (offline ↔ live), shutdown with pending view processing, config written at shutdown identical to before (diff `~/.compass/<version>/`).
3. **Layering.** `check_layering.py` passes with an empty allowlist for `src/`.

## What the follow-up plans get

- **Plugin plan:**
  - the `src/` module libraries can become shared libraries, since there are no cycles left between them;
  - what remains for it are the host ↔ experimental cycles;
  - the composition root's startup steps give it a place to load plugins before the config tree is instantiated.
- **Compute core plan:** not blocked. It finds `COMPASS` already in `compass_core` and no `COMPASS::mainWindow()`. It also finds `compass_util` and `compass_result`, and a few generic widgets moved into core.

## Risks

- **Merge conflicts.** C touches `compass.cpp` and every manager's construction; D and E change call paths in core/db and task and move files. Long-running branches (overlay rework, layer tree rework) need rebasing; git follows pure renames, but files that were both moved and edited need care. Mitigation: small steps that each build and pass the tests, landed quickly.
- **Changed call order.** Direct calls replaced by signals run in connection order. Keep order-sensitive steps (init, shutdown, DB open/close) explicit in the root, and use direct connections.
- **Shutdown and destruction order.** The managers move from `COMPASS`'s `unique_ptr`s to the root. Destruction order and the config write at shutdown must stay as they are (Verification 2).
- **Approximate measurement.** The include analysis resolves by file name. Link-level dependencies without includes (e.g. calls through `compass.h`) are not visible to it. With static libraries they remain hidden and only show up when the plugin plan links shared libraries.

## Effort

For one senior developer who knows this codebase, working with Claude Code.

| Workstream | Realistic | Pessimistic |
|---|---|---|
| A: measurement + guardrail | 2 d | 3 d |
| B: `compass_util` | 3 d | 1 wk |
| C: thin `COMPASS` + composition root | 2 wk | 3 wk |
| D: core/db upward calls + generic widgets | 1.5 wk | 2 wk |
| E: `compass_result` | 1.5 wk | 2.5 wk |
| F: small leftovers | 3 d | 1 wk |
| G: structural guards | 1 wk | 1.5 wk |
| **Total** | **~7.5 wk** | **~11.5 wk** |

## Open decisions

- **Directory of `compass_result` (E):** decided, [src/result/](src/result/).
- **Task result persistence (E):** implemented (`TaskResultDB`), decided to move the result-specific part of `DBInterface`/`SQLGenerator` into `compass_result`, on top of the generic `DBInterface` API.
