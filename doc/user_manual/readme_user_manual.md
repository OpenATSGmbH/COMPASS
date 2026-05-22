# User Manual - Structure & Extension Reference

Source: `doc/user_manual/` - LaTeX memoir-class document.  
Root file: `user_manual.tex` (includes all chapters via `\subfile{}`).  
Current version: **1.0.0** ("Defiant Dodo").  
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
    ui_configuration_data_context.tex
    ui_configuration_data_context_data_sources.tex
    ui_configuration_data_context_asterix.tex
    ui_configuration_data_context_sectors.tex
    ui_configuration_data_context_ffts.tex
    ui_configuration_data_context_colors.tex
    ui_configuration_show_content.tex
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
portal/portal.tex             Chapter: COMPASS Portal
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

**ui_configuration/\***: Data Context dialog and program-wide options. The Data Context dialog (opened from the 'Context' menu, not 'Configuration') folds the per-entity editors into one place: Data Sources (name, SAC/SIC, DSType, network lines, radar ranges/accuracies, MLAT Remote Units), Sector Layers (import from SHP/GeoJSON/GML, altitude limits, exclude sectors), FFTs (Fixed Field Transponders), ASTERIX Configuration (per-CAT edition/REF/SPF), Colors (per-DSType, per-DBContent, per-data-source palettes). The 'Configuration' menu itself only carries DBContent (Meta Variables / DBContent read-only display), Licenses (enter/verify pro license key), Dark Mode, Fullscreen and Refresh Views Automatically.

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

### Voice, Tense, Person
- **Voice**: Passive voice dominates. "Data can be imported", "the file is opened", "a status indication will be shown". Active voice only when an explicit user action is described step-by-step ("Choose a filename, and press 'Save'.").
- **Person**: No first-person singular. No second-person "you" / "your". Use:
  - "the user" when agency matters - "a user can have a specific program configuration", "the user is always aware of occurring issues"
  - "one" for instructional voice - "First, one has to give the filter a new (unique) name"
  - Passive otherwise (most common)
- **Tense**: Present tense throughout. "When started, data is incrementally read". Past tense only for completed sequence steps ("After successful import, a confirmation message is shown") or for design-history rationale ("the 'Reset Date Between Files' option was added").
- **Author voice**: Singular "the author" is accepted for caveats and support pointers - "contact the author for support", "on the author's hardware". Do not substitute "we" / "I".
- **Hedging is house style**: prefer cautious phrasing - "broadly speaking", "somewhat", "may", "might", "is recommended", "no guarantees can be made", "for now", "out of scope for this document". Do not over-promise.

### Tone & Audience
- Professional technical tone, aimed at ATC surveillance domain experts.
- Domain vocabulary used freely without re-explanation: target reports, sectors, MOPS, NACp, NUCp, NIC, NACv, SIL, SDA, GVA, CAT048, CAT062, Mode 3/A, ED-116, ED-117/A, ToD, ARTAS TRI, etc.
- No marketing language: "powerful", "seamless", "enterprise-grade", "best-in-class", "easy-to-use".
- No exclamation marks in body text.

#### ATC / EUROCAE terminology
The manual is written for the same audience that reads EUROCAE / EUROCONTROL / ICAO surveillance documents. Use the wording from those standards rather than software-engineering paraphrases.
- Match the source documents for requirement and performance terms: Probability of Detection (PD), Probability of False Detection (PFD), Update Interval, Reference Trajectory, Sector Layer, Sensor, System Tracker, Target Report, Track, Plot, Coasted Track, Tentative Track, Mode 3/A, Mode C, Mode S, MOPS, Geometric Altitude, Barometric Altitude, Ground Bit, Sensor Status, RU (Remote Unit), Contributing Receiver.
- Reference the standard by its identifier when relevant: "EUROCAE ED-116", "EUROCAE ED-117/A", "EUROCAE ED-87C/E", "EUROCAE ED-142", "EUROCONTROL Specification for Surveillance Data Exchange (ASTERIX) Part N Category NNN", DO-260 / DO-260A / DO-260B for ADS-B MOPS versions.
- Keep canonical abbreviations and casing as printed in the standards: ASTERIX, ADS-B (not "ADSB"), MLAT, WAM, SMR, PSR, SSR, Mode S, ARTAS, JPDA, IMM, RTS, NACp / NUCp / NIC / NACv / SIL / SDA / GVA, RCu, EPU / HFOM, VEPU / VFOM, HPL, MOPS.
- Units follow ATC convention: feet for altitude (FL where applicable), nautical miles (NM) for range, knots for ground speed, degrees / seconds for azimuth-rate and update intervals. Mix only where the source document does (e.g. metres for accuracy standard deviations, seconds for time differences).
- Quote ASTERIX data item references in the standard "CAT.NNN/IIc" / "I048/170" / "REF.PA.SDW.SDW" form, not paraphrased.
- When introducing a domain abbreviation, expand it using the bold pattern (see "Abbreviations") and afterwards use the form printed in the standard - do not anglicise or invent new short forms.

### Highlighting Modes (pick the right one)
- **`\textbf{Term}`**: first introduction of a domain concept (`\textbf{UTN}`, `\textbf{DSType}`, `\textbf{DBContent}`, `\textbf{Meta variables}`). Subsequent uses are plain text.
- **`'Label'` (single quotes)**: every UI string - buttons (`'Reload'`), menus (`'File'`), tabs (`'Main' tab`), dialog options (`'Reset Date Between Files'`), application modes (`'Live: Running'`), variable names (`'Time of Day'`), and literal user-supplied values (`'7000,7777'`, `'L1'`, `'CAT048'`, `'NULL'`). If it appears in the UI as text, single-quote it.
- **`\textit{Path}` + math arrow**: menu navigation paths and data-flow chains - `\textit{Process $\rightarrow$ Evaluate}`, `\textit{Database} $\rightarrow$ \textit{Data Sources} $\rightarrow$ \textit{Filters} $\rightarrow$ \textit{Views}`.
- **`\texttt{value}`**: inline paths / code where single-quoting would be ambiguous. In practice rare - most inline values use single quotes instead.

### Abbreviations
- Pattern: bold each first letter of the expansion **and** the abbreviation itself.
- `\textbf{U}nique \textbf{T}arget \textbf{N}umber (\textbf{UTN})`
- Subsequent uses: plain `UTN`, no bold.

### Capitalization of Domain Concepts
Once a domain noun is introduced as a COMPASS concept, keep it capitalized:
- **Capitalized**: Views, Data Sources, DSType, DBContent, UTN, Meta Variables, Flight Deck, Main Menubar, Main Window, View Points, View Presets, Data Context, Live mode names ('Offline', 'Live: Running', 'Live: Paused').
- **Lowercase**: when used generically rather than as the COMPASS concept - "multiple data sources contribute" vs. "the 'Data Sources' tool".
- **Specific casing - do not normalize**: Mode 3/A, Mode A, Mode C, Mode S, DS ID, ToD, CAT048, CAT062, ED-116, ED-117/A, JPDA, IMM, RTS, NUCp, NACp, NIC.

### Data Context vs. Configuration
**Data Context** and **Configuration** are two distinct concepts. Never call a Data Context a "configuration", a "configuration container", or similar.
- **Configuration**: the program-wide state of COMPASS components stored under `~/.compass/<version>/` (Views that exist, filter definitions, per-widget options, etc.), read at startup and written at shutdown. Covers what the program looks like and which components exist.
- **Data Context**: a named *surveillance context* that groups the entities describing the recorded surveillance environment - data sources, sectors, FFTs, ASTERIX decoding settings, and coloring. Stored under `~/.compass/data_contexts/<name>/`. At most one is active at a time.

When describing Data Context in prose, prefer phrases like "named surveillance context grouping ...", "active Data Context", "stored in the Data Context". Do **not** use "configuration container", "configuration set", "configuration grouping".

### Section Opening Convention
Every `\section`, `\subsection`, `\subsubsection` opens with a single short sentence (10-25 words) describing what the surface is for, ends with `\\`, then jumps to the big-picture figure (if any), then itemize / further detail.

Examples (verbatim opening lines):
- "In this Flight Deck tool, filters can be specified in order to determine which data is loaded into the current dataset and distributed to the Views. \\\\"
- "This task allows importing ASTERIX data recording files into the opened database. \\\\"
- "This dialog allows management of sectors (as 2.5D polygons) stored in the database. \\\\"
- "A Table View displays DBContent data as text in tables to allow textual data inspection."

Avoid: marketing intros, long expository paragraphs before the first list, bullet lists before the intro sentence.

### Big-Picture Figure First
For dialogs / views / Flight Deck tools, place the full-surface screenshot at the top of the section, right after the opening sentence. Component-by-component description follows. Smaller "detail" figures are interleaved with the prose that references them.

### Recurring Scaffolding Phrases (house idiom)
Prefer these over paraphrases - they make the manual read as one voice:
- "In this section, ..." / "In this chapter, ..."
- "At the top, ..." / "Below, ..." / "At the bottom, ..." / "On the left side, ..." / "On the right side, ..."
- "The following X exist:" / "The following items exist:" / "The following columns exist:"
- "There exist N tabs:" / "There exist N buttons:"
- "Using the 'X' button, ..." (instrumental opener for describing a control)
- "When active, this filter ..." (filter-doc opener)
- "Please refer to \nameref{...}" / "see \nameref{...}" (inside list items)
- "as described in \nameref{...}"
- "Therefore, ..." / "However, ..." / "Also, ..." / "Further, ..." / "By default, ..." / "Commonly, ..."
- "e.g." (preferred over "for example"), "i.e." (preferred over "that is")
- "is/are of paramount importance", "of interest", "of use"
- Numbers as digits even when small: "4 tabs", "2 different reconstructors", "3 buttons".

### Item List Format
Most itemize items describe a single GUI element or parameter as `Label: short description`:
- `\item Reload: Trigger reload of view data`
- `\item Line ID: Line into which all data should be written`
- `\item DBContent Variable: Target variable to which this data is mapped`

Use itemize (never enumerate) for workflow steps unless ordering is strictly mandatory.

Item punctuation is intentionally inconsistent - very short label-description items omit a trailing period; full-sentence items keep one. Match the surrounding section.

Spacing after a list: close with `\end{itemize}` then `\ \\` (or `\  \\`) on its own line to leave breathing room before the next paragraph.

When adding content, respect the level of detail and approximate length of existing text in the same section. Items within one list must stay at a similar level of detail - do not write one long bullet next to short sibling bullets. If an item genuinely has more to say, push the extra information into nested sub-items (`\begin{itemize}` inside the item) when feasible, otherwise shorten the text.

### Lists, Subsections, Paragraphs
- `\subsection{Title}` - numbered, appears in TOC.
- `\subsection*{Title}` - unnumbered heading; used in `intro_key_concepts.tex` and similar light-weight dividers inside a chapter where TOC entries would be noise.
- `\paragraph{Title}` - mini-heading inside a `\subsubsection`, e.g. 'Top Elements', 'Parser GUI Elements', 'Comments', 'Usage'. No `\label{}` on `\paragraph` unless cross-referenced.

### Cross-References
- `\nameref{sec:foo}` for inline human-readable references - **default choice**, always within a sentence, never bare.
- `\ref{sec:foo}` only when the **number** matters (rare; appears as parenthetical `(\ref{sec:appendix_view_points})` for "see appendix N.N" style).
- Common forms:
  - "described in \nameref{sec:filters}"
  - "see \nameref{sec:reconst}"
  - "(using \nameref{sec:ui_configure_sectors})"

### Inline UI-Element Icons
When documenting a UI button by its icon rather than a text label, embed the icon inline at body-text size:
- In running text: `The \includegraphics[width=0.5cm,frame]{../../data/icons/edit.png} button in the top right corner ...`
- Inside an itemize item: `\item Edition Edit Button \includegraphics[width=0.5cm]{../../data/icons/edit.png}: Opens the current edition definition in a text editor`

Inline UI-button icons use `frame`, `width=0.5cm`. The hint icon (notes/warnings) uses `width=0.5cm` without `frame`.

### Figures
```latex
\begin{figure}[H]
  \center
    \includegraphics[width=12cm,frame]{figures/filename.png}
  \caption{Caption text}
\end{figure}
```
- Always wrap in `\begin{figure}[H] ... \end{figure}`.
- `\center` (or `\centering`) inside the float, before `\includegraphics`.
- `frame` on UI screenshots; omit `frame` only for full-bleed wide figures or for plain icons in body text.
- Figure path is relative to the containing `.tex` file. Each chapter folder has its own `figures/` subdirectory.
- Caption is a noun phrase / sentence fragment, no trailing period. Both sentence case ("Filters Overview") and Title Case for proper dialog names ("Evaluation Main tab") occur in the existing manual - match the surrounding section.

#### Figure-size cheat sheet
| Use case | width | hspace prefix | frame |
| - | - | - | - |
| Full-page wide screenshot | `19cm` | `\hspace*{-2.5cm}` | optional |
| Almost-page wide | `18cm` | `\hspace*{-2cm}` | `frame` |
| Slightly wide | `17cm` | `\hspace*{-0.5cm}` | `frame` |
| Standard dialog / view | `14cm` to `16cm` | none | `frame` |
| Smaller dialog | `12cm` | none | `frame` |
| Menu / small UI | `5-7cm` | none | `frame` |
| Inline button icon | `0.5cm` | none | `frame` |
| Inline hint icon | `0.5cm` | none | (no frame) |

### Notes / Hints
Three tiers:

- **Routine note (no icon)**: bold the word `note` only:
  `Please \textbf{note} that the filter configuration will be saved at program shutdown.`
- **Highlighted note (hint icon)**: bold `note` only, prefix the icon:
  `\includegraphics[width=0.5cm]{../../data/icons/hint.png} Please \textbf{note} that ...`
- **Hard warning (hint icon + bold full statement)**:
  `\includegraphics[width=0.5cm]{../../data/icons/hint.png} \textbf{Please note that the evaluation feature should not be used as a sole basis for decision making - especially not without manually verifying the evaluation results.}`

Rules:
- Always `Please \textbf{note} that ...` (bold only the word `note`) - except in the hard-warning tier where the whole statement is bold.
- Follow-up note in the same context: `Please also \textbf{note} that ...`.
- Hint icon path is relative to the containing `.tex` file: typically `../../data/icons/hint.png` from chapter folders (e.g. `eval/`, `flightdeck/...`), and from nested folders too (the depth stays the same because `data/` is two levels above `doc/user_manual/`).
- Do not use admonition boxes, framed colored notes, or `\fbox{}` warnings - the manual has no such style.

### Code / CLI
- Shell / CLI: plain `\begin{lstlisting} ... \end{lstlisting}`.
- JSON: small typewriter font and optional string-coloring set immediately before the block:
  ```latex
  \lstset{
      string=[s]{"}{"},
      stringstyle=\color{blue},
      showstringspaces=false,
  }
  \begin{lstlisting}[basicstyle=\small\ttfamily]
  { "content_type": "view_points", ... }
  \end{lstlisting}
  ```
- The global `\lstset` (in `user_manual.tex`) already provides `basicstyle=\ttfamily`, gray background, line breaks, and the `$\hookrightarrow$` overflow marker. Do not redefine these.
- Inline CLI options inside paragraphs: refer to them by name in single quotes ("the '--quit' option" or simply "'--quit'"). Reserve `\texttt{}` for inline filesystem paths and code-like values when single-quoting would be ambiguous.

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

### Subfile Boilerplate
- A subfile begins directly with `\section{...}\label{sec:...}` (or `\subsection{...}\label{sec:...}` for files included one level deeper). No `\documentclass`, no preamble, no `\begin{document}`, no `\end{document}`.
- The root `user_manual.tex` is responsible for the `subfiles` package and the document environment.
- The opening sentence (see "Section Opening Convention") follows the `\label{}` line directly.

### Hard "Do Nots"
- **No em-dash (U+2014)**. Use the ASCII hyphen `-`. En-dash (U+2013) is allowed only in numeric/letter ranges (e.g. "L1-L4" is written with a plain hyphen here in practice; reserve en-dash for ranges that need it visually).
- **No contractions**: write "it is", "do not", "cannot", "will not", "is not". Never "it's", "don't", "can't".
- **No second-person**: no "you", "your", "you can", "you will".
- **No first-person plural/singular**: no "we", "us", "our", "I", "me", "my".
- **No marketing adjectives**: "powerful", "seamless", "world-class", "best-in-class", "easy-to-use".
- **No admonition boxes** or colored note frames. Use the `Please \textbf{note} that ...` pattern.
- **No bare hyperlinks** in body text. Wrap with `\href{URL}{Link Text}` or, for the URL itself as the link, `\url{URL}`.
- **No exclamation marks** in body text.
- **No undefined abbreviations**: introduce with the bold pattern (`\textbf{U}nique \textbf{T}arget \textbf{N}umber (\textbf{UTN})`) before any plain use.
- **No blank lines inside item lists** to create spacing; use `\\` or `\ \\` after the list instead.

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
