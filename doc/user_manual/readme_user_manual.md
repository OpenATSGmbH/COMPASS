# User Manual - Structure & Extension Reference

Source: `doc/user_manual/` - LaTeX memoir-class document.  
Root file: `user_manual.tex` (includes all chapters via `\subfile{}`).  
Current version: **0.9.2** ("Charismatic Capybara").  
Authors: Helmut Puhr & Philipp Wagner.

---

## Document Structure

```
user_manual.tex               root, preamble, chapter includes
intro/
  introduction.tex            Chapter: Introduction (chapter file + subfiles)
  intro_feature_highlights.tex
  intro_general_aspects.tex
  intro_acknowledgements.tex
  intro_key_concepts.tex
install/
  installation.tex            Chapter: Installation
  install_prerequisites.tex
  install_appimage.tex
  install_build.tex
  install_run.tex
  install_upgrade_config.tex
ui/
  ui_overview.tex             Chapter: UI Overview (menus inline + subfiles)
  ui_main_window.tex
  ui_ui.tex
  ui_views.tex
  ui_filters.tex
  import/
    ui_import.tex
    ui_import_asterix.tex
    ui_import_asterix_network.tex
    ui_import_pcap.tex
    ui_import_json.tex
    ui_import_gps.tex
    ui_import_view_points.tex
  config/
    ui_configuration.tex
    ui_configuration_data_sources.tex
    ui_configuration_ffts.tex
    ui_configuration_metavars.tex
    ui_configuration_sectors.tex
    ui_configuration_licenses.tex
  process/
    ui_process.tex
    ui_process_radar_positions.tex
    ui_process_associations_artas.tex
    ui_process_references.tex
    ui_process_evaluate.tex
flightdeck/
  flightdeck.tex              Chapter: Flight Deck
  datasources/data_sources.tex
  filters/filters.tex
  targets/targets.tex
  sensorstatus/sensor_status.tex
  reports/reports.tex
  viewpoints/view_points.tex
  tasklog/task_log.tex
reconst/
  reconstructor.tex           Chapter: Reconstructor
  reconstructor_config.tex
eval/
  evaluation.tex              Chapter: Evaluation
  eval_inspect.tex
  eval_excluded_data.tex
  eval_targets.tex
  eval_requirements.tex
  eval_req_det.tex            (+ eval_req_*.tex per requirement type)
tableview/view.tex            Chapter: Table View
histogramview/view.tex        Chapter: Histogram View
geographicview/
  view.tex                    Chapter: Geographic View
  geo_layout.tex
  geo_toolbar.tex
  geo_data_widget.tex
  geo_config_panel.tex
  geo_layers_tab.tex  geo_layers_*.tex
  geo_style_tab.tex  geo_labels_tab.tex  geo_others_tab.tex
scatterplotview/view.tex      Chapter: Scatter Plot View
gridview/view.tex             Chapter: Grid View
live/live.tex                 Chapter: Live Mode
scripting/
  scripting.tex               Chapter: Scripting
  commandline/command_line.tex
  rtcommands/rtcommands.tex
issues/issues.tex             Chapter: Troubleshooting
appendix/
  appendix.tex                Chapter: Appendix (subfiles only)
  appendix_configuration.tex
  appendix_data_sources.tex
  appendix_view_points.tex
  appendix_maps.tex
  appendix_algorithms.tex
  appendix_logging.tex
  appendix_latex.tex
  appendix_utils.tex
  appendix_remote_gpu.tex
  appendix_licensing.tex
```

---

## Section Summaries

### Introduction
Overview of COMPASS purpose (ATC surveillance data inspection, analysis, evaluation). Explains the manual layout section by section. Licensing summary (GPL-3.0 source, CC BY 4.0 AppImage + manual).

**intro_feature_highlights**: Bullet list of feature highlights grouped by category - Application modes, Import, Analysis, Visualization, MLAT/WAM Contributing Receivers, Reference Trajectory Calculation, Evaluation, Semi-automated Processing.

**intro_general_aspects**: High-level architecture description - database (DuckDB), Offline vs. Live modes, data loading concept (filter → query → result set → views).

**intro_acknowledgements**: Credits to contributors and third-party libraries.

**intro_key_concepts**: Explains the core vocabulary before the rest of the manual. Key subsections:
- Database Systems (DuckDB, single-file container)
- Configuration (JSON files in `~/.compass/<version>/`, read at startup, written at shutdown)
- Data Sources & Database Content (DS name/SAC-SIC/DSType, line IDs L1-L4, DBContent types, DBContent variables)
- ASTERIX Data Import (jASTERIX library, JSON mapping to DBContent variables)
- JSON Data Import (same mapping mechanism as ASTERIX)
- Meta Variables (group same-content variables across DBContent types)
- Unique Target Numbers (UTN - assigned per unique target during association)
- Data Loading (single unified dataset, all views share it, reload needed when variables change)
- Live Mode (three states: Offline, Live: Running, Live: Paused)
- Calculation of Reference Trajectories (Traffic of Opportunity, time-sliced, used as eval reference)
- Free / Pro License (Basic reconstructor free; Probabilistic + IMM requires commercial license)
- Evaluation (reference vs. test data, per-sector averages, per-target statistics, PDF reports)
- View Presets (per-view configurable presets)
- View Points (filter settings + view config + annotations in JSON, version 0.3)

---

### Installation
Intro: AppImage (recommended) vs. build from source (no Geographic View).

**install_prerequisites**: Hardware and OS requirements.

**install_appimage**: Download and run AppImage.

**install_build**: cmake + make build instructions, optional dependencies.

**install_run**: How to start the application.

**install_upgrade_config**: What to do when upgrading versions (config migration).

---

### UI Overview
Describes the main window layout and all top-level menus. Menus are documented inline (not subfiles) with figure + itemize per menu item:

- **File Menu**: New / Open / Open Recent / Export / Close / Save Config / Quit Without Saving Config / Quit
- **Import Menu**: ASTERIX Recording / PCAP Network Recording / ASTERIX From Network / JSON Recording / GPS Trail / View Points
- **Configuration Menu**: Data Sources / FFTs / Meta Variables / Sectors / Licenses / Dark Mode / Refresh Views Automatically
- **Process Menu**: Radar Plot Positions / ARTAS TR Usage / Reconstruct References / Evaluate
- **UI Menu**: Reset Views

Then includes subfiles for each section in detail. Dark mode appendix at end of chapter.

**ui_main_window**: Main window layout, status bar, loading indicator.

**ui_filters** (`sec:filters`): Filter system - built-in filter types, custom filters, filter configuration panel. Covered in detail in Flight Deck chapter.

**ui_views**: How views are opened/closed, view presets.

**ui_import/\***: Per-import-type dialogs - ASTERIX (framing, edition, decoder config, time offset, date handling), PCAP, network UDP streams, JSON, GPS NMEA, View Points JSON.

**ui_configuration/\***: Per-configuration dialogs - Data Sources (name, SAC/SIC, DSType, network lines), FFTs (Fixed Field Transponders), Meta Variables (read-only display), Sectors (import from GML/KML/SHP/GeoJSON, altitude limits, sector layers), Licenses (enter/verify pro license key).

**ui_process/\***: Per-process-task dialogs - Radar plot position calculation, ARTAS TRI association, Reconstruct References (links to Reconstructor chapter), Evaluate (links to Evaluation chapter).

---

### Flight Deck
Central panel for interactive data control. Contains tabs for Data Sources, Filters, Targets, Sensor Status, Reports, View Points, Task Log.

**datasources/data_sources**: Enable/disable data sources per DBContent and line. Live mode adds network status per source.

**filters/filters**: Full filter system - standard filters (time, Mode A/C, Mode C, target address, callsign, data sources, sector, UTN), custom filters, filter enable/disable, save/load filter configs.

**targets/targets**: List of all UTNs - filter by type, show/hide in views, flag targets for exclusion from evaluation.

**sensorstatus/sensor_status**: Live mode only - per-sensor update rate, last update time, status indicators.

**reports/reports**: Manage generated evaluation reports - open, export, delete.

**viewpoints/view_points** (`sec:view_points`): Import/export view point files, navigate through view point list, apply view points to current views.

**tasklog/task_log**: Log of completed import/processing tasks with timestamps.

---

### Reconstructor (`sec:reconst`)
Associates all target reports to UTNs and calculates reference trajectories.

Two reconstructors:
- **Basic (Scoring + UMKalman)**: Free. Secondary attribute matching + distance scoring. Linear Uniform Motion Kalman filter.
- **Advanced (Probabilistic + IMM)**: Pro license required. JPDA, Mahalanobis distances, bias estimation, IMM + RTS smoother.

Processing is time-sliced (typically 15-minute intervals).

**Workflow** per slice: load TRs → remove previous slice → associate by secondary attributes (Mode S addr/ID, track number) → associate by Mode A/C + position → associate by position only → self-associate new targets → retry-associate → compute references → write.

**reconstructor_config**: Configuration panel - reconstructor selection, time slice duration, association thresholds, per-DSType enable/disable.

**Details sections**: Position matching, new target creation, self-association, target types (from ECAT, Mode C, FFT lists), discussion of limitations.

**Probabilistic + IMM features**: Accuracy rescaling (2D maps per DSType), radar bias correction (azimuth + range), JPDA for PSR/SMR, slant-range correction using ADS-B geometric altitude, Mahalanobis distance matching, outlier detection, spline resampling of tracker data.

---

### Evaluation (`sec:eval`)
Requirement-based compliance assessment.

Pre-requisites: associations set, sector defined, reference data and test data exist.

**4 tabs in the dialog**:
- Main: data selection (reference DBContent + sources + line, test DBContent + sources + line), standard selection, minimum height filter (sector layer as complex altitude precondition), sector layer/requirement group mapping.
- Filter: time range filter, ADS-B quality indicator filters.
- Standard: manage standards (add/rename/copy/delete), add/delete requirement groups and requirements, set reference max time difference.
- Results Config: skip-no-data details, split by MOPS version, split by Mode A/C only vs Mode S, show OK in sector results, Geographic View zoom factor, grid cell resolution.

**Supported standards** (partially): EUROCAE ED-116, ED-117/A, ED-87C/E, ED-142, EUROCONTROL Radar Standard.

**Requirement types** (table in evaluation.tex): ~30 types covering detection, identification, Mode 3/A, Mode C, position (distance, across, along, latency, azimuth, range, RMS), speed, track angle, ROCD, MoM, coasting, dubious targets/tracks, extra data/tracks, acceleration.

**eval_inspect**: Results tree - per-standard, per-sector, per-requirement, per-target drill-down. Integration with Geographic View for spatial visualization.

**eval_excluded_data**: Table of target reports excluded due to sector altitude filtering.

**eval_targets**: Per-UTN result table - pass/fail per requirement, manual remove from evaluation.

**eval_requirements**: Per-requirement-type detailed documentation (separate files: eval_req_det, _dub, _extra, _id, _m3a, _mc, _mom, _pos, _speed, _trk_cst).

---

### Table View (`sec:table_view`)
Displays DBContent as text tables. Tabs per DBContent type + 'All'. Configuration panel on the right for column selection. CSV export. Reload button.

---

### Histogram View (`sec:histo_view`)
Histogram of any numeric DBContent variable or evaluation result. Linear or logarithmic axes. Configurable bin count. Export.

---

### Geographic View (`sec:geo_view`)
3D/2D geographic display using OpenSceneGraph + osgEarth. Map widget left, config panel right.

Sub-sections: Layout, Toolbar (mouse modes, navigation), Data Widget (OSG render area), Config Panel tabs:
- Layers tab: enable/disable data layers, annotation layers, sector layers, radar layers, map layers, measurement ops (`geo_layers_*.tex`)
- Style tab: per-DSType symbol size, color, line width
- Labels tab: automatic labeling configuration
- Others tab: misc display options

GeoTIFF import via drag and drop.

**geo_config_panel**: Right panel with all config tabs.

---

### Scatter Plot View (`sec:scatter_view`)
X/Y scatter plot of any two numeric DBContent variables. Configurable axes, log scale, cross-view selection.

---

### Grid View (`sec:grid_view`)
2D grid display: one numeric variable (Z) over two others (X/Y). Configurable color map, data range clamping, resolution.

---

### Live Mode (`sec:live_mode`)
Three modes: Offline (default), Live: Running, Live: Paused.

Triggered by network ASTERIX import. Main status bar indicates live state. Pause button → Live: Paused (cache continues). Stop button → Offline.

Live-mode-specific UI differences: Data Sources tab shows network status per source; Sensor Status tab enabled; most process menus disabled.

---

### Scripting (`sec:scripting`)

**commandline/command_line** (`sec:command_line`): CLI options for semi-automated processing. Options execute in order. GUI config must be pre-set. Key options:
- `--create_db` / `--open_db`
- `--import_asterix_file[s]` / `--import_asterix_pcap_file[s]`
- `--import_asterix_network`
- `--import_json` / `--import_gps_trail`
- `--import_sectors_json` / `--import_data_sources_file`
- `--set_context`
- `--calculate_radar_plot_positions` / `--calculate_artas_tr_usage`
- `--reconstruct_references` / `--reconstruct_references_cfg`
- `--evaluate` / `--evaluation_parameters` / `--evaluate_run_filter`
- `--export_report` / `--export_report_directory`
- `--export_view_points_report`
- `--load_data`
- `--quit`

**rtcommands/rtcommands** (`sec:rt_commands`): TCP interface on `localhost:27960`. JSON command/response. Allows controlling a running COMPASS instance from Python or other tools.

---

### Troubleshooting (`sec:troubleshooting`)
Known issues: AppImage FUSE permissions on CentOS, missing glibc library versions, other OS-specific issues. Instructions for reporting new issues (GitHub).

---

### Appendix (`sec:appendix`)

**appendix_configuration**: Configuration file locations, structure, how to reset.

**appendix_data_sources**: Data source JSON format for `--import_data_sources_file`.

**appendix_view_points** (`sec:appendix_view_points`): View point JSON format specification (version 0.3). Context block + view point array. Fields: content_type, version, view_points[]. Per view point: id, type, text, time, position, ds_types, data_sources, filters, context_variables, annotations. Annotation types: points, lines, linestring, ellipses, text, grid, histogram, scatter, colorvec, nested.

**appendix_maps**: Custom map setup for Geographic View - OSM via QGIS, dark mode maps, GeoTIFF.

**appendix_algorithms**: Position accuracy ellipses (68-95-99.7 rule, 95% = 2σ), ADS-B accuracy variable mapping (MOPS version, NACp, NUCp/NIC).

**appendix_logging**: Log file location, log level configuration.

**appendix_latex**: Instructions for building the PDF from source.

**appendix_utils**: Manual data import utilities (command-line tools for importing data outside the GUI).

**appendix_remote_gpu**: Remote GPU / headless rendering setup for Geographic View.

**appendix_licensing**: GPL-3.0 / CC BY 4.0 conditions, full list of third-party libraries with their licenses.

---

## Writing Style & Conventions

### LaTeX Setup
- Document class: `\documentclass[oneside,a4paper]{memoir}`
- TOC depth: subsection (overview) and subsubsection (detailed)
- Numbering depth: subsubsection
- Custom style packages: `memsty`, `memlays`, `titlepages` (all in `doc/user_manual/`)
- Code blocks: `lstlisting` with gray background (`lbcolor = {0.9,0.9,0.9}`)
- Tables: `tabularx` with `\textwidth`, column types `| l | X |` (fixed + fill)

### File / Module Organization
- One `\chapter{}` per top-level `.tex` file
- Sections and subsections either inline or split into `\subfile{}` calls
- Each subfile begins directly with `\section{}` or `\subsection{}` - no preamble
- Labels: `\label{sec:snake_case_name}` immediately after `\chapter/\section/\subsection`
- Cross-references: `\nameref{sec:label}` for human-readable names, `\ref{sec:label}` for numbers

### Text Style
- Professional technical tone - aimed at ATC surveillance domain experts
- No first-person singular; no marketing language
- Short opening paragraph per section, then structured content
- Paragraph break: `\\` or `\  \\` (explicit line spacing, no blank lines between items)
- Key terms bolded on first introduction: `\textbf{UTN}`, `\textbf{DSType}`, etc.
- Abbreviations introduced inline: `\textbf{U}nique \textbf{T}arget \textbf{N}umber (\textbf{UTN})`
- No prose lists - always `\begin{itemize}...\end{itemize}`
- Nested itemize for sub-details (up to 3 levels)
- Workflow steps as itemize (not enumerate) unless strict order matters

### Figures
```latex
\begin{figure}[H]
  \center
    \includegraphics[width=12cm,frame]{figures/filename.png}
  \caption{Caption text}
\end{figure}
```
- Wide figures (full-page): `\hspace*{-2.5cm}` + `width=19cm`
- `frame` option on most UI screenshots
- Figure path relative to containing `.tex` file
- `figures/` subdirectory per chapter folder
- Captions: sentence case, concise, no trailing period on short captions

### Notes / Hints
```latex
\includegraphics[width=0.5cm]{../../data/icons/hint.png} \textbf{Please note that ...}
```
- Icon path is relative (`../../data/icons/hint.png` from most subfiles)
- Strong warnings use `\textbf{Please note that ...}` inline without icon

### Code / CLI
```latex
\begin{lstlisting}
./COMPASS-release_x86_64.AppImage --help
\end{lstlisting}
```
- All code, file paths, CLI options, JSON in `lstlisting`
- Inline code/paths: `\texttt{value}` or plain monospace via lstlisting

### Tables
```latex
\begin{center}
 \begin{table}[H]
  \begin{tabularx}{\textwidth}{ | l | X | }
    \hline
    \textbf{Column A} & \textbf{Column B} \\ \hline
    value & description \\ \hline
  \end{tabularx}
  \caption{Table caption}
 \end{table}
\end{center}
```

### Links
```latex
\href{https://example.com}{Link Text}
\url{https://example.com}
```

### Section Label Naming
Pattern: `sec:<module>_<topic>` - e.g.:
- `sec:eval_req_detection`
- `sec:ui_import_asterix`
- `sec:configure_datasources_network`
- `sec:data_sources_live_mode`
- `sec:appendix_view_points`
- `chap:flight_deck` (chapters use `chap:` prefix)

### Adding New Content
1. Create `<folder>/<topic>.tex` - start with `\section{Title}\label{sec:name}`
2. Add `\subfile{<topic>}` in the parent chapter file
3. Add a `figures/` subfolder if the section has screenshots
4. Register label in the parent chapter's `\nameref{}` links if cross-referenced
5. For new chapters: add `\subfile{<folder>/view}` in `user_manual.tex` between existing chapter includes

### What Belongs Where
- New UI dialog/panel → subfile under relevant `ui/` sub-folder + reference from `ui_overview.tex` menu list
- New Flight Deck tab → new subfile under `flightdeck/`, registered in `flightdeck.tex`
- New view type → new chapter folder with `view.tex`, registered in `user_manual.tex`
- New evaluation requirement → new `eval/eval_req_<name>.tex` + row in requirements table in `evaluation.tex`
- New CLI option → add to `lstlisting` block in `scripting/commandline/command_line.tex`
- New appendix topic → new `appendix/appendix_<topic>.tex` + `\subfile{}` in `appendix/appendix.tex`

### Building the PDF
```bash
cd doc/user_manual
pdflatex user_manual.tex
pdflatex user_manual.tex   # second pass for TOC/refs
```
See `appendix/appendix_latex.tex` for full build instructions.
