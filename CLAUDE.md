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

**Design patterns**: Dependency injection (not singletons) - `COMPASS` is created by `Client` in `main()` and passed by reference through the manager tree (e.g. `task_.manager().compass()`). Managers store a `COMPASS&` and expose it via `compass()` accessors. Only `Logger` and `RTCommandRegistry` are true singletons (via `Singleton` base class). Qt signals/slots throughout GUI, `Configurable` inheritance for JSON persistence, smart pointers (`unique_ptr`/`shared_ptr`).

## Build

```bash
cmake -B build -S .
make -C build -j$(nproc)
```

Build output goes to `build/bin/` (executables) and `build/lib/` (libraries).

**C++ standard**: The project uses **C++17** (`-std=c++17`). The AppImage is built inside a Debian 10 Docker container (see `docker/Dockerfile_deb10`) using GCC 8.3. C++17 features such as structured bindings (`auto [x, y] = ...`), `std::optional`, `std::variant`, `if constexpr`, `std::string_view`, and `std::any` are available and may be used freely.

**Targets:**
- `compass` - main library
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

## Project structure

```
src/                    Main source code
  app/                  Application executables
    client/             GUI application (Client, MainWindow, main.cpp)
    watchdog/           Watchdog process (compass_handler)
  config/               Configuration framework (Configurable, ConfigurationManager)
  core/                 Core libraries
    buffer/             Core data container (Buffer, NullableVector<T>)
    compass.h           Main COMPASS class (owns all managers)
    job/                Async job management (JobManager)
    net/                Networking
    projection/         Geographic projections (OGR, RS2G, geo)
    rtcommand/          Runtime command interface
    sector/             Airspace sector definitions
    source/             Data source management
    unit/               Physical unit management
    util/               Utilities (logger.h, singleton.h, files, strings, time)
    unit_tests/         Core unit tests (buffer, projection, number)
  db/                   Database layer
    dbcontent/          Database content types, variables, targets, labels
    interface/          Database backend (DuckDB)
  eval/                 Evaluation framework
    requirement/        Requirement definitions (detection, position, speed, ...)
    results/            Evaluation results and reports
  filter/               Data filtering system
    unit_tests/         Filter unit tests
  gui/                  Shared GUI widgets and helpers
    unit_tests/         GUI unit tests
  import/               Data import
    asterix/            ASTERIX data format parsing and import
    fft/                FFT import
    json/               JSON import
  reconstruction/       Trajectory reconstruction (simple reconstructor, associator)
  report/               Report generation
  task/                 Task system (import, reconstruction, calc)
  unit_tests/           Test main (CATCH_CONFIG_MAIN) and top-level test CMakeLists
  view/                 View system
    viewbase/           Base view classes
    gridview/           2D grid visualization
    histogramview/      Histogram visualization
    points/             View points and reports
    scatterplotview/    Scatter plot visualization
    tableview/          Text table visualization
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
- **COMPASS access:** `COMPASS` is not a singleton - it is created by `Client` in `main()` and passed by reference. Access via parent chain, e.g. `task_.manager().compass().dataSourceManager()`. Managers store `COMPASS&` and expose `compass()`.
- **True singletons:** Only `Logger::getInstance()` and `RTCommandRegistry::instance()` (both inherit from `Singleton` base class in `src/core/util/singleton.h`).
- **Configurable base class:** most components inherit from `Configurable` for JSON-based config persistence
- **Qt signals/slots** throughout the GUI layer; `Q_OBJECT` macro required
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
