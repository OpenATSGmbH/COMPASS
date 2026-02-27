# COMPASS — Compliance Assessment

OpenATS COMPASS is a C++ application for air traffic surveillance data inspection, analysis, and evaluation. It imports EUROCONTROL ASTERIX recordings into a database for visualization, compliance assessment (e.g. EUROCAE ED-116, ED-117), and report generation.

## Platform & distribution

- **OS**: Linux 64-bit (x86_64) only — no Windows or macOS support
- **Tested distributions**: Debian 10+, Ubuntu 18.04+, Linux Mint 18.3+
- **Distribution format**: AppImage — single self-contained executable, no installation required. Downloaded from GitHub releases. The AppImage is built on Debian 10 (Buster) via Docker to maximize glibc compatibility across distributions.
- **No plugin system**: Extensions require source code modification. The Geographic View is a proprietary closed-source module included only in the AppImage binary, not in source builds.
- **Licensing**: Source code is GPL-3.0; AppImage binary is CC BY 4.0; Geographic View is proprietary (AppImage only). Free for all use including commercial.
- **Hardware requirements**: Minimum 2+ physical CPU cores, dedicated NVidia or ATI GPU (native drivers, OpenGL 3.0+), 8 GB RAM. Recommended: Intel i5+, 16 GB+ RAM. Large datasets (>1M reports/hour): 32 GB RAM.

## Application architecture

COMPASS is a **monolithic Qt5 desktop application** with a **DuckDB** embedded database backend. There is no client-server split — the GUI, database, and processing engine run in a single process (`compass_client`), with a separate `compass_handler` watchdog process for long-running/live deployments.

**Key architectural layers:**
- **GUI layer**: Qt5 Widgets with signals/slots. Main thread runs the Qt event loop; views render from a single shared in-memory dataset.
- **Data layer**: Columnar `Buffer`/`NullableVector<T>` containers loaded from DuckDB (primary) or SQLite3 (legacy). Only one unified dataset exists at a time; all views share it.
- **Processing layer**: Async job management (`JobManager` singleton), parallelized via Intel TBB. Reconstruction and evaluation run in background jobs on worker threads.
- **3D rendering**: OpenSceneGraph + osgEarth for the Geographic View. Runs on the main thread with 1-second update intervals and overload detection (skip rendering at >3s latency, skip ASTERIX decoding at >60s).
- **Configuration**: `Configurable` base class — most components inherit from it. Parameters are auto-serialized to/from JSON files in `~/.compass/<version>/`. Read at startup, written at shutdown.
- **External interfaces**: CLI for batch processing (options executed in order), TCP runtime command interface (`localhost:27960`) for controlling a running instance from Python or other tools.

**Design patterns**: Singleton managers (`COMPASS::instance()`, `JobManager::instance()`), Qt signals/slots throughout GUI, `Configurable` inheritance for JSON persistence, smart pointers (`unique_ptr`/`shared_ptr`).

## Build

```bash
cmake -B build -S .
make -C build -j$(nproc)
```

Build output goes to `build/bin/` (executables) and `build/lib/` (libraries).

**C++ standard**: The project uses **C++17** (`-std=c++17`). The AppImage is built inside a Debian 10 Docker container (see `docker/Dockerfile_deb10`) using GCC 8.3. C++17 features such as structured bindings (`auto [x, y] = ...`), `std::optional`, `std::variant`, `if constexpr`, `std::string_view`, and `std::any` are available and may be used freely.

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

### Logging
Use the LOG4CPP-based stream macros defined in `logger.h`. They auto-prepend the function name and are used like C++ output streams:
- `logerr` — errors (always printed)
- `logwrn` — warnings
- `loginf` — informational messages
- `logdbg` — debug (compiled in, but filtered by runtime log level)

Usage: `loginf << "loaded " << count << " records";`

Do **not** use `std::cout`, `std::cerr`, `printf`, or `qDebug()` for application logging.

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
- **View Point**: saved filter + view configuration + annotations for revisiting a specific data view (JSON format, stores data source selection, filters, view configs, and annotations)
- **Evaluation**: compliance assessment of test data vs reference data against configurable standards/requirements
- **Reconstructor**: calculates reference trajectories from Traffic of Opportunity (Basic: Scoring+UMKalman, Advanced: Probabilistic+IMM)
- **Application modes**: Offline (recorded data analysis), Live Running (real-time network import), Live Paused
- **FFT (Fixed Field Transponder)**: stationary transponders used as reference points for calibration
- **ASTERIX**: EUROCONTROL standard binary format for surveillance data exchange (decoded by jASTERIX library)

## Application workflow

Typical processing pipeline: **Create/Open DB → Import data → Reconstruct references → Evaluate → Generate reports**

1. **Database**: DuckDB single-file container (`.db`). Create or open via File menu. Data sources, filters, and views all operate on this database.
2. **Import**: ASTERIX files/PCAP/network streams, JSON, GPS trails (NMEA). During import, data is mapped to DBContent variables and attributed to data sources + line IDs.
3. **Process**: Radar plot position calculation, ARTAS track association, reference trajectory reconstruction (time-sliced in ~15 min intervals).
4. **Load & Filter**: Data is loaded into RAM as a single unified dataset, restricted by active filters and data source selection. Only one dataset exists at a time; all Views share it.
5. **Visualize**: Five view types display the loaded dataset. Cross-view selection highlights data across all views.
6. **Evaluate**: Compare test data vs reference data against configurable standards with per-sector and per-target results.

## Data import formats

- **ASTERIX files**: Raw binary recordings (framings: none, IOSS, IOSS_seq, RFF)
- **ASTERIX PCAP**: Network packet captures containing ASTERIX data
- **ASTERIX network**: Live UDP streams (switches to Live mode)
- **JSON**: jASTERIX JSON format
- **GPS trails**: NMEA files for (D)GPS reference trajectories
- **Sectors**: GML, KML, ESRI Shapefiles, GeoJSON polygon imports

## Reconstruction

Associates target reports to UTNs and calculates reference trajectories. Processes data in time slices (~15 min).

**Association order**: Mode S address → Mode S identification → Track number → Mode A/C + position → Position only

**Two reconstructors**:
- **Basic (Scoring + UMKalman)**: Free. Distance-based scoring, Linear Uniform Motion Kalman filter. Key params: Maximum Acceptable Distance (Air: 1852m/1nm, Ground: 40m), Maximum Altitude Difference (300ft).
- **Advanced (Probabilistic + IMM)**: Licensed. Mahalanobis distance scoring, accuracy rescaling (2D maps), radar bias correction, ADS-B geometric altitude for slant-range correction, JPDA for PSR/SMR plots, IMM filter + RTS smoother. Multiple calculation passes per slice.

## Evaluation

Compares test data sources against reference data sources within defined airspace sectors. Pre-requisites: UTN associations, at least 1 sector, usable reference + test data.

**Supported standards**: EUROCAE ED-116, ED-117/A, ED-87C/D/E, ED-142, EUROCONTROL Radar Surveillance Standard

**Requirement types**: Detection (PD), Position (distance, along/across, latency, RMS, radar azm/rng), Identification (correct/false), Mode 3/A (present/false), Mode C (correct/false/present), Speed, Track angle, Dubious targets/tracks, Extra data/tracks, MoM (longitudinal/transversal/vertical), Acceleration, ROCD, Track coasting

**Results**: Per-sector averages + per-target statistics, drillable to per-target-report level. Exportable as PDF, LaTeX, or JSON reports. Optional splits by ADS-B MOPS version or Mode A/C vs Mode S.

## Filtering system

Filters restrict which data is loaded from the database. All active filters are combined with logical AND.

**Built-in filters**: Time of Day, Timestamp, Position (lat/lon bounds), Mode 3/A codes, Mode C altitude, Aircraft Address (Mode S hex), Aircraft Identification, Detection Type, ADS-B Quality, ADS-B MOPS version, Ground Bit, MLAT RUs, Track Number, UTNs, Primary Only, RefTraj Accuracy, Excluded Time Windows, Tracker Track Number

**Custom filters**: Users can create filters on any DBContent variable using SQL operators (=, !=, >, >=, <, <=, IN, LIKE, IS, IS NOT). Supports ABS() for absolute value conditions.

## Views

- **Table View**: Textual data as sortable tables, configurable columns, CSV export
- **Histogram View**: Any numeric variable or eval result as histogram (linear/log axis), supports annotations
- **Geographic View**: osgEarth map with OpenSceneGraph rendering, automatic labeling, time filtering, data selection, distance measurements, layer/style system, 3D support, configurable data grouping (by DBContent, data source, Mode A, variable). Live mode: shows most recent 5 min with 1s updates
- **Scatter Plot View**: Any two numeric variables as X/Y plot, supports annotations
- **Grid View**: Numeric variable over 2D grid of two other variables, configurable color maps

All views support **cross-selection** (selecting data in one view highlights it in all others) and **view presets** (saved/restorable configurations).

## Live mode

Three application modes: Offline → Live:Running → Live:Paused (and back)

- **Live:Running**: Reads ASTERIX from UDP network streams, imports to DB, displays in Geographic View with 1s updates. Most recent 5 min kept in RAM cache. Data older than 60 min removed from DB.
- **Live:Paused**: Caches incoming network data, allows offline-like inspection, auto-resumes after configurable timeout (default 60 min).
- **Overload handling**: Display latency >3s skips rendering until caught up; >60s skips ASTERIX decoding entirely.
- **Sensor status**: CAT063-based sensor health monitoring (Operational/Degraded/Initializing/Not Connected).

## Command-line interface

Supports semi-automated batch processing. Key options:
```
--create_db / --open_db              Database management
--import_asterix_file(s)             ASTERIX file import
--import_asterix_pcap_file(s)        PCAP import
--import_asterix_network             Live network import
--import_json / --import_gps_trail   Other formats
--import_sectors_json                Sector import
--reconstruct_references             Run reconstructor
--evaluate                           Run evaluation
--export_report                      Export report (PDF/LaTeX/JSON)
--quit                               Quit after tasks complete
--no_cfg_save                        Don't save config on exit
```
Options are executed in order, enabling pipelines like: `--open_db X --reconstruct_references --evaluate --export_report Y --quit`

## Runtime command interface

TCP socket interface for controlling a running COMPASS instance from external tools (Python, netcat, etc.). Enabled with `--open_rt_cmd_port`.

- **Connection**: `127.0.0.1:27960` (localhost)
- **Protocol**: Send command as string, receive two JSON replies (issued + completed)
- **Command format**: `command_name --arg1=VALUE --arg2=VALUE` or short form `command_name VALUE1 VALUE2`

**Reply structure** (completion reply):
```json
{
    "ok": true,
    "error": "",
    "error_additional_info": "",
    "reply": { },
    "execution_time": "00:00:01.234"
}
```

**Available commands**:
- **Database**: `create_db`, `open_db`, `open_recent_db`, `close_db`, `quit`
- **Import**: `import_asterix_file(s)`, `import_asterix_pcap_file`, `import_asterix_network`, `import_asterix_network_stop`, `import_json`, `import_gps_trail`, `import_view_points`, `import_sectors_json`, `import_data_sources`
- **Processing**: `reconstruct_references`, `calculate_artas_tr_usage`, `calculate_radar_plot_positions`
- **Evaluation**: `evaluate` (with `--config`, `--result`, `--run_filter`)
- **Data retrieval**: `get_utns`, `get_target`, `get_target_stats`, `get_dbcontent_data`, `get_db_data_sources`, `get_cfg_data_sources`
- **Reports**: `get_existing_reports`, `get_report` (with `--section` for hierarchical drill-down), `export_report`, `export_view_points_report`
- **Configuration**: `set_data_sources`, `set_view_point`, `delete_all_data_sources`, `client_info`, `help`

**Python integration**: A `COMPASSInstance` class exists for starting COMPASS, connecting, sending commands, and receiving replies programmatically.

## Configuration system

- Most components inherit from `Configurable` (in `src/core/config/`), which registers typed parameters and auto-serializes them to/from JSON. `Configuration` groups parameters belonging to one component instance; `ConfigurationManager` is the singleton that owns all `Configuration` objects and handles file I/O.
- **Default configuration** lives in `conf/default/` in the source tree — do not modify these files unless explicitly asked. At runtime, the deployed configuration is stored in `~/.compass/1.0.0/` (1.0.0 = current version), read at startup and written at shutdown (not on crash).
- JSON files include `compass.json`, `db_content.json`, `views.json`, `task_import_asterix.json`, etc.
