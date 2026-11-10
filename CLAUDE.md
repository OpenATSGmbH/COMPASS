# COMPASS - Compliance Assessment

OpenATS COMPASS is a C++ application for air traffic surveillance data inspection, analysis, and evaluation. It imports EUROCONTROL ASTERIX recordings into a database for visualization, compliance assessment (e.g. EUROCAE ED-116, ED-117), and report generation.

## Platform & distribution

- **OS**: Linux 64-bit (x86_64) only - no Windows or macOS support
- **Tested distributions**: Debian 10+, Ubuntu 18.04+, Linux Mint 18.3+
- **Distribution format**: AppImage - single self-contained executable, no installation required. Downloaded from GitHub releases. The AppImage is built on Debian 10 (Buster) via Docker to maximize glibc compatibility across distributions.
- **No plugin system**: Extensions require source code modification. The Geographic View lives in a separate closed-source repository (`experimental_src/`) and is only published in the AppImage binary, not in source builds.
- **Licensing**: Source code is GPL-3.0; AppImage binary is CC BY 4.0; Geographic View is closed-source (AppImage only). Free for all use including commercial.
- **Hardware requirements**: Minimum 2+ physical CPU cores, dedicated NVidia or ATI GPU (native drivers, OpenGL 3.0+), 8 GB RAM. Recommended: Intel i5+, 16 GB+ RAM. Large datasets (>1M reports/hour): 32 GB RAM.

## Application architecture

COMPASS is a **monolithic Qt5 desktop application** with a **DuckDB** embedded database backend. There is no client-server split - the GUI, database, and processing engine run in a single process (`compass_client`), with a separate `compass_handler` watchdog process for long-running/live deployments.

**Key architectural layers:**
- **GUI layer**: Qt5 Widgets with signals/slots. Main thread runs the Qt event loop; views render from a single shared in-memory dataset.
- **Data layer**: Columnar `Buffer`/`NullableVector<T>` containers loaded from DuckDB. Only one unified dataset exists at a time; all views share it.
- **Processing layer**: Async job management (`JobManager`), parallelized via Intel TBB. Reconstruction and evaluation run in background jobs on worker threads.
- **3D rendering**: OpenSceneGraph + osgEarth for the Geographic View. Runs on the main thread with 1-second update intervals and overload detection (skip rendering at >3s latency, skip ASTERIX decoding at >60s).
- **Configuration**: `Configurable` base class - most components inherit from it. Parameters are auto-serialized to/from JSON files in `~/.compass/<version>/`. Read at startup, written at shutdown.
- **External interfaces**: CLI for batch processing (options executed in order), TCP runtime command interface (`localhost:27960`) for controlling a running instance from Python or other tools.

**Design patterns**: Dependency injection (not singletons) - the composition root `CompassRoot` (`src/app/compassroot.h`, owned by `Client`) creates `COMPASS` and all managers, wires them and runs the startup and shutdown sequence. `COMPASS` (`src/app/compass.h`) is the application facade of the app layer (DB session, app state, config parameters): it holds pointers to the core managers and the provider interfaces of the upper ones, it does not own them. The managers get their dependencies injected (constructor and `setManagers(...)` from `CompassRoot`) and expose them as accessors (e.g. `task_.manager().dbContentManager()`). Only `Logger` and `RTCommandRegistry` are true singletons (via `Singleton` base class). Qt signals/slots throughout GUI, `Configurable` inheritance for JSON persistence, smart pointers (`unique_ptr`/`shared_ptr`).

## Library layering

The code is split into static libraries with acyclic dependencies. Each library is one directory, and a library may only depend on the libraries below it (bottom to top):

| Layer | Library | Directory |
|---|---|---|
| 1 | `compass_util` | `src/util/` |
| 2 | `compass_config` | `src/config/` |
| 3 | `compass_core` | `src/core/` (incl. `db/`, `gui/`, `latex/`, `context/`, ...) |
| 4 | `compass_gui` | `src/gui/` |
| 5 | `compass_filter`, `compass_reconstruction` | `src/filter/`, `src/reconstruction/` |
| 6 | `compass_view` | `src/view/` |
| 7 | `compass_result` | `src/result/` |
| 8 | `compass_eval` | `src/eval/` |
| 9 | `compass_task` | `src/task/` |
| 10 | `compass_app`, `compass_client` | `src/app/` |

Rules:
- **Only downward dependencies**, in includes and in `target_link_libraries`. A library's include paths only reach its own directory and, through its links, the libraries below it, so an upward `#include` does not compile. Never add include paths into other module directories or global include paths into `src/`.
- **Upward calls** go through an interface in the lower layer, implemented by the upper one and wired in `CompassRoot` (e.g. `IViewProvider`, `IFilterProvider`, `IEvalProvider` in core, `ITaskProvider` in result, injected via `setManagers(...)`, e.g. `dbcont_man.viewProvider()`). Upward notifications are signals, connected in `CompassRoot`.
- **Directory = library.** Code that belongs to another library is moved there (`git mv`), never compiled into a target from another directory or referenced with `../` paths in CMake.
- **Includes are relative to the library's include directories** (e.g. `"report/section.h"`, `"dbcontent/dbcontentmanager.h"`), not to `src/`.
- **Check:** `python3 scripts/check_layering.py --build-dir build` checks the headers each compiled file actually includes and the CMake link and include paths (without `--build-dir` it analyzes the include lines). It is run manually (not part of the build or CI), e.g. after larger refactorings; the allowlist `scripts/layering_allowlist.txt` is empty and may not grow.
- **Sub-layers inside `compass_core`** (bottom to top): `base` → `rtcommand` (incl. `test/`) → `gui` → `services` (job, unit, sector, license) → `dbinterface` → `context` → `projection` → `dbcontent` (incl. `db/job`, `i*provider.h`) → `ui` (`source/`). The check verifies them too; the allowlist `scripts/layering_core_allowlist.txt` is empty and may not grow (workstream I in `readme_dependencies.md`). `DBInterface` is generic database access; DB content specific access (DB content tables, targets table) goes through `DBContentDB` (`dbcont_man.contentDB()`).
- **`compass.h` only at app level:** it lives in `src/app/` (so an include from a lower `src/` library does not compile); besides `src/app/` only the app-level parts of `experimental_src/` (`rtcommand/`, `test/`, linked to `compass_app`) may include it. The check enforces this for `src/` and `experimental_src/`; the allowlist `scripts/layering_compass_allowlist.txt` is empty and may not grow (workstream J). Code above core gets its managers directly (e.g. `filter_manager.dbContextManager()`, `view_->viewManager().appState()`).
- **Exceptions:** host sites that use `experimental_src/` (geo view, ProbIMM, Kalman interfaces) are known host ↔ experimental cycles, resolved by the plugin plan (`readme_plugins.md`). Background and history: `readme_dependencies.md`.

## Build

```bash
cmake -B build -S .
make -C build -j$(nproc)
```

Build output goes to `build/bin/` (executables) and `build/lib/` (libraries).

**C++ standard**: The project uses **C++17** (`-std=c++17`). The AppImage is built inside a Debian 10 Docker container (see `docker/Dockerfile_deb10`) using GCC 8.3. C++17 features such as structured bindings (`auto [x, y] = ...`), `std::optional`, `std::variant`, `if constexpr`, `std::string_view`, and `std::any` are available and may be used freely.

**Targets:**
- `compass_util` ... `compass_task`, `compass_app` - the libraries, see Library layering
- `compass_client` - GUI application
- `compass_handler` - watchdog process
- `compass_tests` - unit tests

## Testing

Framework: **Catch2** (header-only, in `lib/catch.hpp`)

```bash
./build/bin/compass_tests
```

Unit tests live inside each module's `unit_tests/` subdirectory (e.g. `src/core/unit_tests/`, `src/filter/unit_tests/`, `src/config/unit_tests/`, `src/gui/unit_tests/`, `experimental_src/unit_tests/`). Test files are named `test_<topic>.cpp`. A dedicated `test_main.cpp` in `src/unit_tests/` defines `CATCH_CONFIG_MAIN`. Tests use `TEST_CASE`, `SECTION`, `REQUIRE`, and `Approx()`.

When adding tests, add new `test_<topic>.cpp` files in the appropriate module's `unit_tests/` directory and register them in that module's `unit_tests/CMakeLists.txt`. The top-level test CMakeLists is `src/unit_tests/CMakeLists.txt`.

The unit tests take their resources from the repository, never from `~/.compass` (CI containers have none): `test_main.cpp` points `Utils::Files::setDataDirectory()` to the repository's `data/` (used by all `Utils::Files::dataDirectory()` lookups: icons, images, fonts, maps, ...), and tests read other files via `COMPASS_SOURCE_DIR` (e.g. `conf/default/`). Application code reads data resources via `Utils::Files::dataDirectory()`, not `HOME_DATA_DIRECTORY`.

## Project structure

```
src/                    Main source code, one directory per library (see Library layering)
  util/                 compass_util: utilities (logger.h, singleton.h, files, strings, time, global.h)
  config/               compass_config: configuration framework (Configurable, ConfigurationManager)
  core/                 compass_core
    appstate.h          App state and global settings (AppState)
    i*provider.h        Interfaces for upward calls (view, filter, eval), implemented by upper managers
    buffer/             Core data container (Buffer, NullableVector<T>)
    db/                 Database layer
      dbcontent/        Database content types, variables, targets, labels
      interface/        Database backend (DuckDB)
    context/            DB context, data source and sector context
    gui/                Generic widgets used by core (dialogs, popup menu, range edit, ...)
    job/                Async job management (JobManager, async tasks)
    latex/              LaTeX document generation
    net/                Networking
    projection/         Geographic projections (OGR, RS2G, geo)
    rtcommand/          Runtime command interface
    sector/             Airspace sector definitions
    source/             Data source management
    unit/               Physical unit management
    unit_tests/         Core unit tests (buffer, projection, number)
  gui/                  compass_gui: shared GUI widgets and helpers
  filter/               compass_filter: data filtering system
  reconstruction/       compass_reconstruction: trajectory reconstruction (simple reconstructor, associator)
  view/                 compass_view: view system
    viewbase/           Base view classes
    gridview/           2D grid visualization
    histogramview/      Histogram visualization
    points/             View points (report/: view points LaTeX report)
    scatterplotview/    Scatter plot visualization
    tableview/          Text table visualization
  result/               compass_result: task results (TaskResult, TaskResultDB), report model and exporters
  eval/                 compass_eval: evaluation framework
    requirement/        Requirement definitions (detection, position, speed, ...)
    results/            Evaluation results
  task/                 compass_task: task system (TaskManager, import, reconstruction, analysis, calc)
    import/             Import tasks and parsing (asterix/, json/, gps/, view_points/)
  app/                  compass_app, compass_client, compass_handler
    compass.h           COMPASS application facade (DB session, app state, config parameters, command host)
    compassroot.h       Composition root: creates COMPASS and the managers, wiring, startup/shutdown
    client/             GUI application (Client, MainWindow, main.cpp)
    watchdog/           Watchdog process (compass_handler)
  unit_tests/           Test main (CATCH_CONFIG_MAIN) and top-level test CMakeLists
experimental_src/       Experimental features (separate git repo)
  reconstruction/       Advanced reconstruction (Kalman, JPDA, ProbIMM)
  view/geographicview/  OSG/osgEarth geographic view
  unit_tests/           Experimental unit tests
lib/                    Third-party headers (catch.hpp, json.hpp, spline.h)
conf/                   Runtime configuration (JSON)
data/                   Runtime data (icons, textures, maps, ASTERIX definitions)
cmake_modules/          CMake find-scripts for dependencies
doc/                    Documentation and user manual (LaTeX)
```

## Code conventions

### Naming
- **Files:** lowercase with underscores: `main_window.cpp`, `buffer.h`
- **Headers:** `.h` (not `.hpp`), use `#pragma once`
- **Classes:** PascalCase: `Buffer`, `DBContent`, `ViewManager`
- **Functions:** camelCase: `dbOpened()`, `addProperty()`, `lastUsedPath()`
- **Member variables:** snake_case with trailing underscore: `db_opened_`, `app_mode_`
- **Local variables:** snake_case without trailing underscore
- **Qt signals:** suffixed with `Signal`: `databaseOpenedSignal()`
- **Qt slots:** suffixed with `Slot`: `databaseOpenedSlot()`

### Patterns
- **COMPASS access:** `COMPASS` is not a singleton - it is created by `CompassRoot` and used only by the app level (`CompassRoot`, `Client`, `MainWindow`, the app-level runtime commands, the test lab). Other code reaches its dependencies via the parent chain, e.g. `task_.manager().dbContentManager()`, `view_->viewManager().dbContextManager()`. `COMPASS` offers the core managers it needs for the DB session and the command host (DB interface, DB content, DB context, license, projection, runtime commands) and `viewProvider()` for the view lookups of the command host.
- **Inside `compass_core`** nothing uses `COMPASS`. Core managers take `Configurable` as parent and get their dependencies directly (`AppState&`, peers via `setManagers(...)` from `CompassRoot`, e.g. `dbcont_man.dbInterface()`, `ctx_man.appState()`). App state and global settings live in `AppState` (`src/core/appstate.h`), reached via `appState()` (`COMPASS` has no forwards for them). Runtime command execution reaches the application through `rtcommand::IRTCommandHost` (implemented by `COMPASS`).
- **Upper managers** (`FilterManager`, `ViewManager`, `EvaluationManager`, `TaskManager`) are reached through their own library's parent chain (`view_->viewManager()`, `task().manager()`). Their peers are set by `CompassRoot` via `setManagers(...)` and exposed as accessors (e.g. `task().manager().viewManager()`). Every manager with peers set after construction implements `IManager` (`src/core/imanager.h`): `checkWired()` returns whether all of them are present, and `CompassRoot::startManagers()` asserts it for all such managers at its end (a new manager with `setManagers(...)` must be added there). The app layer uses `CompassRoot`'s accessors (`MainWindow::root_`, `main_window::root()`).
- **Runtime commands** get their managers in the constructor (no static manager pointers) and are registered by the `init_*_commands(registry, managers...)` function of their module via `registry.registerCommand<Cmd>(managers...)`; the `DECLARE_RTCOMMAND` macro only declares name and description. App-level commands take `CompassRoot&`. The registry is cleared at the start of `CompassRoot::shutdown()`.
- **True singletons:** Only `Logger::getInstance()` and `RTCommandRegistry::instance()` (both inherit from `Singleton` base class in `src/util/singleton.h`).
- **Configurable base class:** most components inherit from `Configurable` for JSON-based config persistence
- **Qt signals/slots** throughout the GUI layer; `Q_OBJECT` macro required
- **No widgets or dialogs cached in non-UI classes:** a widget or dialog is created by its UI host (e.g. `MainWindow`, a parent widget) and reads or modifies the data on demand when shown. State that must live on without the widget belongs in a model class (e.g. `SensorStatusTracker` for the Sensor Status tool).
- **Smart pointers:** prefer `std::unique_ptr` and `std::shared_ptr`
- **Buffer/NullableVector<T>:** columnar data storage - the central data structure for surveillance records. Supports bool, char, uchar, int, uint, long, ulong, float, double, string, json, ptime.
- **PropertyList / PropertyDataType:** type-safe schema for Buffer columns

### Logging
Use the LOG4CPP-based stream macros defined in `logger.h`. They auto-prepend the function name and are used like C++ output streams:
- `logerr` - errors (always printed)
- `logwrn` - warnings
- `loginf` - informational messages
- `logdbg` - debug (compiled in, but filtered by runtime log level)

Usage: `loginf << "loaded " << count << " records";`

Do **not** use `std::cout`, `std::cerr`, `printf`, or `qDebug()` for application logging.

### License header
All source files must include the GPL-3.0 header (see any existing `.h`/`.cpp` file).

## Key dependencies

- **Qt5** (Widgets, Core, OpenGL, Charts, Test)
- **Boost** (regex, system, program_options, filesystem, iostreams, thread, stacktrace_backtrace)
- **Eigen3** - linear algebra
- **DuckDB** - embedded database backend
- **GDAL** - geographic data
- **OpenSceneGraph** - 3D rendering
- **osgEarth** - geographic visualization
- **GeographicLib** - geographic calculations
- **jASTERIX** - ASTERIX data decoding
- **LOG4CPP** - logging
- **TBB** - threading
- **nlohmann/json** - JSON (header-only in `lib/`)
