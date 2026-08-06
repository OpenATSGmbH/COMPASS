# COMPASS Runtime Command Interface

This readme summarizes the COMPASS runtime command (TCP) interface. The authoritative,
customer-facing documentation is the user manual chapter "Scripting":

- `doc/user_manual/scripting/rtcommands/rtcommands.tex` - Runtime Command Interface

For the command line options see `readme_cli.md`.

## Overview

TCP socket interface for controlling a running COMPASS instance from external tools
(Python, netcat, etc.). Enabled with the `--open_rt_cmd_port` command line option.

- **Connection**: `127.0.0.1:27960` (localhost)
- **Command format**: `command_name --arg1=VALUE --arg2=VALUE`, or the short form
  `command_name VALUE1 VALUE2` for commands that support positional arguments

Two JSON replies are sent per command. First, an "issued" reply:

```json
{
  "ok": true,
  "error": "",
  "error_additional_info": ""
}
```

Then a completion reply:

```json
{
  "ok": true,
  "error": "",
  "error_additional_info": "",
  "reply": {},
  "execution_time": "00:00:01.234"
}
```

The structure of `reply` depends on the command; some commands send no reply data
(`reply` is `null`).

Quick manual test with netcat:

```bash
netcat 127.0.0.1 27960
open_db "/home/user/data/hello_world2.db"
```

## Quoting and escaping rules

Command arguments are parsed with POSIX shell-style tokenization; `"`, `'`, and `\`
have special meaning:

- Arguments containing spaces must be wrapped in double quotes.
- Inner double quotes must be escaped as `\"`.
- Braces `{}`, brackets `[]`, colons, semicolons, pipes, and angle brackets are safe.
- String argument **values** cannot contain literal double quotes, single quotes, or
  backslashes (use forward slashes in paths).

For JSON arguments (e.g. `set_view_point`, `set_data_sources`, `evaluate`,
`reconstruct_references`): wrap the entire JSON in outer double quotes and escape all
inner double quotes:

```
set_view_point "{\"id\": 1, \"name\": \"All\", \"status\": \"open\"}"
```

In Python the escaping can be done programmatically:

```python
import json
vp = {"id": 1, "name": "All", "status": "open"}
vp_escaped = '"' + json.dumps(vp).replace('"', '\\"') + '"'
compass.interface.sendCommandAndUnpack('set_view_point ' + vp_escaped)
```

## Available commands

**Database**: `create_db`, `open_db` (with `assure_open`), `open_recent_db` (by
`filename` or `index`), `close_db` (with `strict`), `quit`

**Import**: `import_asterix_file`, `import_asterix_files`, `import_asterix_pcap_file`,
`import_asterix_pcap_files` (all with `framing`, `line`, `date`, `time_offset`,
`ignore_time_jumps`, `config`), `import_asterix_network` (with `time_offset`,
`max_lines`, `ignore_future_ts`, `config`), `import_asterix_network_stop`,
`import_json`, `import_gps_trail` (with `name`, `sac`, `sic`, `tod_offset`, `date`,
`mode3a` octal, `address` hex, `id`, `config`), `import_view_points`,
`import_sectors_json`, `import_data_sources`

**Processing**: `reconstruct_references` (with `config`, `disable_sensors`),
`calculate_artas_tr_usage`, `calculate_radar_plot_positions`, `load_data`, `delete_data`
(with `delete_info` JSON array)

**Evaluation**: `evaluate` (with `config`, `result`, `run_filter`),
`get_eval_standards` (standards with requirement groups/requirements, no DB needed)

**Data retrieval**: `get_utns` (with `nodesc`), `get_target`, `get_target_stats`,
`get_dbcontent_data` (with `dbcontent`, `variables` `|`-separated, `utn`, `max_size`),
`get_data_sources`, `get_data_source_counts` (per data source / dbcontent / line
inserted record counts)

**Reports**: `get_existing_reports`, `get_report` (with `section` for hierarchical
drill-down, see below), `export_report` (with `report`, `dir`, `mode`),
`export_view_points_report`

**Data context**: `create_context`, `set_context`, `delete_context`, `list_contexts`,
`get_context_info`, `get_context`, `import_ffts_json`, `delete_all_ffts`,
`import_sectors_gdal` (with `layer`, `color`, `exclude`), `export_sectors_json`,
`delete_all_sectors`, `export_data_sources_json`

**Analyze**: `analyze_data_source` (with `ds_type`, `config`), `get_analyze_inspectors`

**Geographic View**: `set_map`, `get_maps`

**Configuration / misc**: `set_data_sources`, `set_view_point`,
`delete_all_data_sources`, `client_info`, `help` (with `command`, `details`)

Note: `evaluate`'s `config` argument cannot use `CAT001` or `CAT048` as
`dbcontent_name_ref`; these categories do not provide the maximum X/Y standard
deviation required for reference accuracy filtering and are rejected as reference data.

## ASTERIX import configuration parameters

The `config` argument of the ASTERIX import commands (and the CLI option
`--import_asterix_parameters`) accepts a JSON string of import parameter overrides.
Parameter naming must be exactly as in `task_import_asterix.json`, plus the following
non-configuration parameters: `override_tod_active`, `filter_tod_active`,
`filter_position_rec_active`, `filter_position_circ_active`, `filter_modec_active`,
`file_line_id` (0..3), `date_str` (`YYYY-mm-DD`), `network_ignore_future_ts`,
`obfuscate_secondary_info`.

Examples:

Rectangular position filter (degrees) plus Mode C filter (feet):
```json
{
  "filter_position_rec_active": true,
  "filter_rec_latitude_min": 49.686,
  "filter_rec_latitude_max": 50.3898,
  "filter_rec_longitude_min": 8.0115,
  "filter_rec_longitude_max": 9.1129,
  "filter_modec_active": true,
  "filter_modec_max": 500,
  "filter_modec_min": -10000
}
```

Circular position filter (center in degrees, range in nautical miles):
```json
{
  "filter_position_circ_active": true,
  "filter_circ_latitude": 47.26,
  "filter_circ_longitude": 11.34,
  "filter_circ_range": 40.0
}
```

Time of Day override (offset in seconds, may be negative):
```json
{
  "override_tod_active": true,
  "override_tod_offset": 3600.0
}
```

Time of Day filter (seconds since midnight):
```json
{
  "filter_tod_active": true,
  "filter_tod_min": 36000.0,
  "filter_tod_max": 43200.0
}
```

## Fetching reports via get_report

Called without `--section`, `get_report` returns the list of all section IDs in the
report:

```
get_report "test Evaluation"
```

With `--section=SECTION_ID` it returns the serialized section; its `sub_sections`
entries contain only `name` and `id` of each subsection, which can be used for further
`get_report` calls (hierarchical drill-down):

```
get_report "test Evaluation" --section="Report:Results:Sectors:Mandatory DOI:Sum"
```

## Python integration

The Python class `COMPASSInstance` (module `compass_interface`) starts a COMPASS
instance, connects, sends commands, and unpacks replies:

```python
from compass_interface import COMPASSInstance

compass_instance = COMPASSInstance()
r = compass_instance.runCOMPASS(binary=binary,
                                wait_for_commands=True,  # wait until commands can be received
                                no_cfg_save=True)        # do not save config on close

r = compass_instance.interface.sendCommandAndUnpack('open_db "' + filename + '"')
# r.value holds the command's reply data as a Python dictionary; check r / r.error

compass_instance.closeCOMPASS()
```
