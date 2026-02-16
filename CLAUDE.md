# COMPASS — Compliance Assessment

OpenATS COMPASS is a C++ application for air traffic surveillance data inspection, analysis, and evaluation. It imports EUROCONTROL ASTERIX recordings into a database for visualization, compliance assessment (e.g. EUROCAE ED-116, ED-117), and report generation.

## Build

```bash
cmake -B build -S .
make -C build -j$(nproc)
```

Build output goes to `build/bin/` (executables) and `build/lib/` (libraries).

**Targets:**
- `compass` — main library
- `compass_client` — GUI application
- `compass_handler` — watchdog process
- `compass_tests` — unit tests

## Testing

Framework: **Catch2** (header-only, in `lib/catch.hpp`)

```bash
./build/bin/compass_tests
```

Unit tests live in `src/unit_tests/` and `experimental_src/unit_tests/`. Test files are named `test_<topic>.cpp`. A dedicated `test_main.cpp` defines `CATCH_CONFIG_MAIN`. Tests use `TEST_CASE`, `SECTION`, `REQUIRE`, and `Approx()`.

When adding tests, add new `test_<topic>.cpp` files and register them in `src/test/CMakeLists.txt`.

## Project structure

```
src/                    Main source code
  asterix/              ASTERIX data format parsing
  buffer/               Core data container (Buffer, NullableVector<T>)
  client/               Main GUI application (MainWindow, entry point)
  config/               Configuration framework (Configurable base class)
  command/              Command infrastructure
  dbcontent/            Database content types, variables, targets, labels
  eval/                 Evaluation framework
    requirement/        Requirement definitions (detection, position, speed, mode_a, mode_c, ...)
    results/            Evaluation results and reports
    data/               Evaluation data handling
  filter/               Data filtering system
  interface/            Database backends (SQLite, DuckDB)
  job/                  Async job management
  net/                  Networking
  projection/           Geographic projections (OGR, RS2G, geo)
  reconstruction/       Trajectory reconstruction
  rtcommand/            Runtime command interface
  sector/               Airspace sector definitions
  source/               Data source management
  task/                 Task system (import, association, reconstruction, calc)
  unit/                 Physical unit management
  unit_tests/           Catch2 unit tests
  util/                 Utilities
  view/                 View system
    viewbase/           Base view classes
    gridview/           2D grid visualization
    histogramview/      Histogram visualization
    points/             View points and reports
    scatterplotview/    Scatter plot visualization
    tableview/          Text table visualization
  watchdog/             Watchdog process
experimental_src/       Experimental features
  reconstruction/       Advanced reconstruction (Kalman, JPDA)
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
- **Singleton managers:** `COMPASS::instance()`, `JobManager::instance()`, etc.
- **Configurable base class:** most components inherit from `Configurable` for JSON-based config persistence
- **Qt signals/slots** throughout the GUI layer; `Q_OBJECT` macro required
- **Smart pointers:** prefer `std::unique_ptr` and `std::shared_ptr`
- **Buffer/NullableVector<T>:** columnar data storage — the central data structure for surveillance records. Supports bool, char, uchar, int, uint, long, ulong, float, double, string, json, ptime.
- **PropertyList / PropertyDataType:** type-safe schema for Buffer columns

### License header
All source files must include the GPL-3.0 header (see any existing `.h`/`.cpp` file).

## Key dependencies

- **Qt5** (Widgets, Core, OpenGL, Charts, Test)
- **Boost** (regex, system, program_options, filesystem, iostreams, thread, stacktrace_backtrace)
- **Eigen3** — linear algebra
- **DuckDB** — primary database backend
- **SQLite3** — legacy database backend
- **GDAL** — geographic data
- **OpenSceneGraph** — 3D rendering
- **osgEarth** — geographic visualization
- **GeographicLib** — geographic calculations
- **jASTERIX** — ASTERIX data decoding
- **LOG4CPP** — logging
- **TBB** — threading
- **nlohmann/json** — JSON (header-only in `lib/`)

## Domain concepts

- **DBContent**: a type of surveillance data (e.g. CAT048 = radar, CAT062 = tracker). Each has typed variables (time, lat, lon, Mode 3/A, Mode C, etc.)
- **Data Source**: a specific sensor identified by name, SAC/SIC, and DSType (Radar, MLAT, ADS-B, Tracker, RefTraj)
- **Line ID**: L1–L4, distinguishes network lines or separate recordings of same source
- **Meta Variable**: groups equivalent DBContent variables across different DBContent types
- **UTN (Unique Target Number)**: groups target reports belonging to the same physical target
- **Buffer**: columnar in-memory data container holding NullableVector<T> columns
- **View**: visualization component (TableView, HistogramView, GeographicView, ScatterPlotView, GridView)
- **View Point**: saved filter + view configuration + annotations for revisiting a specific data view
- **Evaluation**: compliance assessment of test data vs reference data against configurable standards/requirements
- **Reconstructor**: calculates reference trajectories from Traffic of Opportunity (Basic: Scoring+UMKalman, Advanced: Probabilistic+IMM)
- **Application modes**: Offline (recorded data analysis), Live Running (real-time network import), Live Paused
