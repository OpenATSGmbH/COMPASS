# COMPASS Command Line Interface

This readme summarizes the COMPASS command line options. The authoritative,
customer-facing documentation is the user manual chapter "Scripting":

- `doc/user_manual/scripting/commandline/command_line.tex` - Command Line Options

For the runtime command (TCP) interface see `readme_rtcmd.md`.

## Command line interface

COMPASS supports semi-automated batch processing via command line options. Options are
executed **in order**, enabling pipelines such as:

```bash
./COMPASS-release_x86_64.AppImage --open_db /data/file.db \
  --reconstruct_references --evaluate \
  --export_report "EUROCAE ED-87E Evaluation" --quit
```

Notes:
- Configuration of the application still has to be performed using the GUI; the
  application must be set up correctly before command line options can be used.
- Error or warning messages (and related confirmations) halt automatic task running.
- `--help` prints the full option list.

### General options

| Option | Description |
|---|---|
| `--help` | Produce help message |
| `-r`, `--reset` | Reset user configuration and data (factory config copied into `~/.compass/<version>/`) |
| `--override_cfg_path arg` | Use `~/.compass/<version>/<arg>/` instead of `.../default/` (separate config sets) |
| `--expert_mode` | Enable expert mode for the session (persistent via `"expert_mode": true` in `~/.compass/<version>/task.json`) |
| `--no_cfg_save` | Do not save configuration upon quitting |
| `--open_rt_cmd_port` | Open runtime command TCP port (default 27960), see `readme_rtcmd.md` |
| `--enable_event_log` | Collect warnings/errors in the event log (query via `get_events` runtime command) |
| `--no_highdpi` | Disable Qt high-DPI scaling (fixes double-sized UI on some systems) |
| `--quit` | Quit after finishing all previous steps |

### Database and data context

| Option | Description |
|---|---|
| `--create_db arg` | Create and open new DuckDB database, e.g. `/data/file1.db` |
| `--open_db arg` | Open existing DuckDB database |
| `--list_contexts` | List all available data contexts (active one marked) and exit |
| `--set_context arg` | Set the active data context before any DB/import operations, e.g. `Test` |

### Import

| Option | Description |
|---|---|
| `--import_data_sources_file arg` | Import data sources JSON file |
| `--import_view_points arg` | Import view points JSON file |
| `--import_asterix_file arg` | Import ASTERIX file |
| `--import_asterix_files arg` | Import multiple ASTERIX files, `;`-separated |
| `--import_asterix_pcap_file arg` | Import ASTERIX PCAP file |
| `--import_asterix_pcap_files arg` | Import multiple ASTERIX PCAP files, `;`-separated |
| `--import_asterix_file_line arg` | Line identifier for file import, `L1`..`L4` |
| `--import_asterix_date arg` | Import date, `YYYY-MM-DD` |
| `--import_asterix_file_time_offset arg` | Time of Day override offset, `HH:MM:SS.ZZZ` |
| `--import_asterix_ignore_time_jumps` | Ignore 24h Time of Day wrap-arounds (default: day rollover) |
| `--import_asterix_network` | Import ASTERIX from defined network UDP streams |
| `--import_asterix_network_time_offset arg` | Additive time offset for network import, `HH:MM:SS.ZZZ` |
| `--import_asterix_network_max_lines arg` | Max lines per data source during network import, 1..4 |
| `--import_asterix_network_ignore_future_ts` | Silently drop target reports with future timestamps |
| `--asterix_framing arg` | Framing: `none` (raw/netto), `ioss`, `ioss_seq`, `rff` |
| `--asterix_decoder_cfg arg` | Decoder config JSON string (see below) |
| `--import_asterix_parameters arg` | Import parameter overrides JSON string, same parameter set as the runtime commands' `config` argument (see "ASTERIX import configuration parameters" in `readme_rtcmd.md`) |
| `--import_json arg` | Import JSON file |
| `--import_gps_trail arg` | Import GPS trail NMEA file |
| `--import_gps_parameters arg` | GPS import parameters JSON string (see below) |
| `--import_sectors_json arg` | Import previously exported sectors JSON |

`--asterix_decoder_cfg` sets editions/mappings per category, e.g. (including one pair of
single quotes):

```bash
--asterix_decoder_cfg '{"10":{"edition":"0.31"}}'
```

Per category the keys `"edition"`, `"ref_edition"`, `"spf_edition"`, and `"mapping"`
(e.g. `"CAT010 to Radar"`) can be set. Naming must be exactly as in the GUI, otherwise
the application quits with an error message. Note the category key `"10"` for CAT010.

`--import_gps_parameters` example. The keys are the GPS trail import task's
configuration parameters (unlike the runtime command `import_gps_trail`, which
also offers convenience arguments `mode3a` octal / `address` hex): here
`mode_3a_code` and `target_address` are decimal, and the `set_*` / `use_*`
flags must be enabled for the corresponding value to apply. NMEA files without
date information need the override date:

```json
{
  "ds_name": "GPS Trail",
  "ds_sac": 255,
  "ds_sic": 0,
  "use_override_date": true,
  "override_date_str": "2026-06-09",
  "set_callsign": true,
  "callsign": "ENTRPRSE",
  "set_mode_3a_code": true,
  "mode_3a_code": 961,
  "set_target_address": true,
  "target_address": 16702992,
  "use_tod_offset": false,
  "tod_offset": 0.0
}
```

### Processing, evaluation, and reports

| Option | Description |
|---|---|
| `--calculate_radar_plot_positions` | Calculate radar plot positions |
| `--calculate_artas_tr_usage` | Associate target reports based on ARTAS usage |
| `--reconstruct_references` | Reconstruct references from sensor and tracker data |
| `--reconstruct_references_cfg arg` | Reconstructor config override JSON, e.g. `'{"current_reconstructor_str": "Scoring + UMKalman"}'` |
| `--load_data` | Load data after start |
| `--evaluate` | Run pre-configured evaluation |
| `--evaluation_parameters arg` | Evaluation parameters JSON string (see below) |
| `--evaluate_run_filter` | Run evaluation filter ('Filter UTNs') before evaluation |
| `--analyze_data_source arg` | Run Analyze Data Source task for the given DSType: `MLAT` or `ADSB` |
| `--analyze_parameters arg` | Analyze task parameters JSON string, analogous to `--evaluation_parameters` |
| `--export_view_points_report arg` | Export View Points as PDF, argument is the report filename |
| `--export_report arg` | Export existing report by name, e.g. `EUROCAE ED-87E Evaluation`, PDF by default |
| `--export_report_directory arg` | Export directory, e.g. `/data/report2/` |
| `--export_report_mode arg` | Export mode: `DocX`, `JSON`, `Latex`, `PDF` |

`--evaluation_parameters` example (excerpt; parameters mirror the Evaluation
configuration):

```json
{
  "active_sources_ref": {"CAT062": {"1234": true}},
  "active_sources_tst": {"CAT021": {"2345": true}},
  "current_standard": "Dubious Targets",
  "dbcontent_name_ref": "CAT062",
  "dbcontent_name_tst": "CAT021",
  "use_grp_in_sector": {
    "Dubious Targets": {
      "SectorName1": {"Optional": false},
      "SectorName2": {"Optional": true}
    }
  }
}
```

Note: `dbcontent_name_ref` cannot be `CAT001` or `CAT048`; these categories do not
provide the maximum X/Y standard deviation required for reference accuracy filtering
and are rejected as reference data.

## Examples

Each invocation runs its options in order; multi-step workflows with different
per-import options (e.g. different line identifiers) are split into one
invocation per step, re-opening the database.

Create a database and import IOSS recordings into line L1, restricted to a
10 NM circle around an airport (`filter_circ_range` in NM; the parameter keys
follow the ASTERIX import configuration, see `readme_rtcmd.md`):

```bash
./COMPASS-release_x86_64.AppImage --set_context LOWW_26 \
  --create_db /data/loww/20260609.db \
  --import_asterix_files '/data/loww/rec-astos.ff;/data/loww/rec-mlw.ff' \
  --asterix_framing ioss --import_asterix_file_line L1 \
  --import_asterix_parameters '{"date_str": "2026-06-09",
    "filter_position_circ_active": true, "filter_circ_latitude": 48.110278,
    "filter_circ_longitude": 16.569722, "filter_circ_range": 10.0}' \
  --quit
```

Import a further recording into line L2 of the same database:

```bash
./COMPASS-release_x86_64.AppImage --open_db /data/loww/20260609.db \
  --import_asterix_file /data/loww/rec-ets.ff \
  --asterix_framing ioss --import_asterix_file_line L2 \
  --import_asterix_parameters '{"date_str": "2026-06-09"}' \
  --quit
```

Import a GPS trail NMEA file with secondary attributes (see the
`--import_gps_parameters` description above):

```bash
./COMPASS-release_x86_64.AppImage --open_db /data/loww/20260609.db \
  --import_gps_trail /data/loww/solution.NMEA.txt \
  --import_gps_parameters '{"ds_name": "GPS Trail", "ds_sac": 255, "ds_sic": 0,
    "use_override_date": true, "override_date_str": "2026-06-09",
    "set_callsign": true, "callsign": "FLUSI60",
    "set_mode_3a_code": true, "mode_3a_code": 1,
    "set_target_address": true, "target_address": 4487920}' \
  --quit
```

Reconstruct references and export the resulting task report as JSON:

```bash
./COMPASS-release_x86_64.AppImage --open_db /data/loww/20260609.db \
  --reconstruct_references \
  --export_report 'Reconstruct References L1' \
  --export_report_directory /data/loww/reconstruction_report/ \
  --export_report_mode JSON --quit
```

Run an ADS-B data source analysis (ground traffic only) and export its report:

```bash
./COMPASS-release_x86_64.AppImage --open_db /data/loww/20260609.db \
  --analyze_data_source ADSB \
  --analyze_parameters '{"use_ground_only": true}' \
  --export_report 'Analyze ADSB Data Source' \
  --export_report_directory /data/loww/adsb_analysis_report/ \
  --export_report_mode JSON --quit
```

Report names follow the generating task (shown in the result view); existing
names can be queried via the `get_existing_reports` runtime command or the
`task_results` database table.
