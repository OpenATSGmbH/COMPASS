# GUI Conventions

Rules for building task / configuration dialogs and the result reports they produce in COMPASS. Distilled from the existing reconstructor, evaluation, and analyse-data-source dialogs and their reports. Apply these rules to every new dialog and every new section of a result report so the look, behaviour, and wording stay consistent.

These rules supersede ad-hoc styling. When existing code violates them and you are touching it, fix the violation while you are there; do not introduce new violations.

The text below talks about "the dialog", "the action button", "the underlying task", etc. as generic concepts. Examples in parentheses are illustrations, not the scope of the rule.

---

## 0. Where new GUI fits

Before you add a widget, decide which surface of the app it lives on. The Main Window has four established surfaces (described in detail in [doc/user_manual/ui/ui_main_window.tex](../../doc/user_manual/ui/ui_main_window.tex) and [doc/user_manual/flightdeck/flightdeck.tex](../../doc/user_manual/flightdeck/flightdeck.tex)):

- **Main Menubar** at the top. The entry point for actions: opening / closing the database, importing data, configuring application-wide settings, launching processing tasks, resetting the UI. Actions that produce a result (a dialog, a task run, an info window) belong here.
- **Flight Deck** on the left. A collapsible sidebar holding tools the user inspects or interacts with throughout a session (Data Sources, Filters, Targets, Sensor Status, Reports, View Points, Task Log). Things the user "lives with" go here; transient configuration does not.
- **Main Viewbar** in the centre. A tab bar of Views (TableView, GeographicView, ...). New visualisations are Views; not GUI scaffolding.
- **Main Statusbar** at the bottom. Database indicator, application mode, and the global Load button. Reserved for status; do not pile widgets onto it.

Modal configuration dialogs (Reconstruct, Evaluate, Analyse, Import) are triggered from menu actions and disappear once the task is submitted. Use this pattern for "configure a one-shot run" surfaces.

If your widget does not fit any of these surfaces, ask before introducing a new one.

## 0.1 Main Menu

The Main Menubar has five top-level menus, in this order:

| Menu              | Purpose                                                                                  |
|-------------------|------------------------------------------------------------------------------------------|
| `&File`           | Database lifecycle (New, Open, Open Recent, Export, Close), saving config, quit.         |
| `&Import`         | Data import dialogs (ASTERIX, PCAP, network ASTERIX, JSON, NMEA, View Points).           |
| `&Configuration`  | Application-wide settings: DBContent, Licenses, Dark Mode, Fullscreen, view auto-refresh. |
| `&Process`        | Processing tasks operating on the imported data (radar plot positions, ARTAS association, reconstruction, analysis, evaluation). |
| `&UI`             | UI state (Reset Views).                                                                  |

Add new actions to the menu they fit, not to a new top-level menu. If your action triggers a one-shot processing run, it is `Process`; if it opens an import dialog, it is `Import`; if it changes a global setting, it is `Configuration`.

### Action wording

- **Title Case**, no trailing punctuation. Examples: "Calculate Radar Plot Positions", "Reconstruct References", "Reset Views".
- Use the **standard term**: "Reconstruct", "Evaluate", "Analyse" (British spelling, matching the existing menu). Never invent project synonyms.
- **Accelerator marker `&`** on a distinct letter of the action: `&New`, `&Open`, `&ASTERIX Recording`. Pick a letter not already used inside that menu.
- **Every action has a `setToolTip(...)`**. Sentence-cased, no trailing period, describes the effect ("Open an existing database"). The menu sets `setToolTipsVisible(true)` so tooltips render.

### Shortcuts

- Use Qt's platform-standard sequences when one exists: `QKeySequence::New`, `QKeySequence::Open`, `QKeySequence::Quit`.
- Otherwise use `Ctrl+<letter>` matching the action's first distinctive letter (`Ctrl+S` Save Config, `Ctrl+A` ASTERIX, `Ctrl+J` JSON, `Ctrl+G` GPS, `Ctrl+V` View Points, `Ctrl+D` Dark Mode, `F11` Fullscreen).
- Don't reuse a shortcut that's already taken by a Configuration / View action.

### Grouping

- Use `addSeparator()` between operationally distinct groups within a menu (e.g. database operations vs. config operations vs. quit operations in `File`).
- Use a sub-menu when several actions share a kind: `Process > Analyze > MLAT`. Pluralise the parent only when the verb is shared.
- Conditionally visible items: actions that require a database (Import, most of Process) are disabled until a DB is open. Wire that to the existing menu enable/disable logic; do not add ad-hoc visibility checks inside the action handler.

## 0.2 Flight Deck

The Flight Deck is the collapsible left-hand sidebar that hosts session-long tools. As of writing, the seven tools are listed in [ui_main_window.tex](../../doc/user_manual/ui/ui_main_window.tex). Tools share a contract:

- A single icon visible on the collapsed deck, picked from `data/icons/`.
- A **number-key hotkey** (1, 2, 3, ...). Adding a new tool means picking the next free number.
- A **decoration bar** at the top of the expanded tool: Decrease Width (`-`), Increase Width (`+`), Expand / Collapse Flight Deck (`#`), and a tool-specific Configuration button (`edit.png`). Existing tools use the established icons; reuse them.
- Support for **Expansion Mode**, in which the tool overlaps all Views. Tools must lay out reasonably at both narrow (sidebar) and wide (expansion) widths.
- A **Tool Configuration menu** containing at least the Screen Ratio entry plus tool-specific options. Wire it to the bar's `edit.png` button.

When to add a tool to the Flight Deck:

- The user interacts with it across many task / view operations during one session (filtering, inspecting unique targets, browsing reports).
- It does **not** belong as a Flight Deck tool when:
  - It is a one-shot configuration dialog (use a menu action instead).
  - It is a visualisation of the loaded dataset (use a View instead).
  - It only matters in one specific mode (Live, Evaluation, ...) — embed it in that surface instead of the global deck.

If you need to register a new tool, follow the existing widget classes in `src/core/source/datasourcestoolwidget.*`, `src/view/points/viewpointstoolwidget.*`, etc. — they show the exact pattern.

---

## 1. Dialog shell

- **Window title**: short, in Title Case, no trailing punctuation. Names what the dialog configures or what action it submits (e.g. "Reconstruct Reference Trajectories", "Evaluate", "Analyse MLAT Data Source").
- **Minimum size**: at least `900 x 720`. Reports, threshold tables, and lists with long names need both the height and the width.
- **Multi-page dialogs use a two-pane layout**: navigation on the left (`QTreeWidget`), `QStackedWidget` on the right, separated by a horizontal `QSplitter`.
  - `tree_->setHeaderHidden(true)` unless a header label adds information.
  - `tree_->setMinimumWidth(180)`.
  - `splitter->setChildrenCollapsible(false)`.
  - Equal stretch factors (`1, 1`); set initial sizes around `{400, 600}`.
  - Wrap every page added to the stack in a `QScrollArea` (`setWidgetResizable(true)`, `setFrameShape(QFrame::NoFrame)`). Without this, a single page with a wide form can lock the splitter handle in place.
- **Single-page dialogs** can drop the splitter and put the form directly in the dialog body.

## 2. Navigation tree (left pane)

- One top-level item per page. Avoid hierarchical nesting unless the structure is genuinely tree-shaped.
- Order: inputs first, configurable steps next, outputs last.
- Every tree item is selectable and shows a real configuration page. **Do not add tree items that exist only as headings or comments.**
- Items that toggle whether a step runs use a tree-row checkbox (`Qt::ItemIsUserCheckable`). The initial state comes from the persisted config, never a hard-coded default.
- Disabled rows: `item->setDisabled(true)` with the reason as the column-0 tooltip. **Never auto-uncheck a row** when its prerequisites become temporarily unmet (because of an unrelated change elsewhere in the dialog). The user's selection state is preserved; the action button is the gatekeeper for execution.
- Right-aligned italic badges (e.g. `[pro]`, `[beta]`, `[experimental]`) live in **column 1**:
  ```cpp
  tree_->setColumnCount(2);
  tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  // per item:
  item->setText(1, "[pro]");
  item->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
  QFont f = item->font(1); f.setItalic(true); item->setFont(1, f);
  ```

## 3. Configuration pane (right pane)

- **No paragraph-style help text on configuration pages.** No `QLabel` containing instructions like "Select the X to do Y." The form labels and tooltips already describe what each control does.
- Use `QFormLayout` for label/widget pairs. The label column describes the control; the field column is the editor.
- Group related controls in a `QGroupBox` with a Title-Cased title.
- **Selectable-list pattern** (a checkbox list backed by a parameter shared across all entries, e.g. an item ID, a line, an axis):
  ```
  [QListWidget with checkboxes]
  [Select All] -- stretch -- [Select Nothing]
  Shared Parameter: [QComboBox / QSpinBox]
  ```
  - Both selection buttons are mandatory. Place them on a row directly below the list, with `addStretch(1)` between them so they sit on opposite ends.
  - The shared parameter, if any, is its own row below the buttons. (Use the project's wording for the shared parameter; do not invent new abbreviations.)

## 4. Form widgets

### Checkboxes

- One row per checkbox in the form layout.
- The label sits in the form-layout's label column; the checkbox is a **bare `QCheckBox` with no inline text**. The label column carries the description; the checkbox is just the tick.
- Default state: **checked**. Defaults are applied so the user sees a working configuration on first open. Default a checkbox unchecked only when the feature is dangerous, expensive, or requires explicit acknowledgement.

### Spinboxes

- Always set `setRange`, `setSingleStep`, `setDecimals` (for `QDoubleSpinBox`), and `setSuffix` when the value has a unit (`" m"`, `" ft"`, `" s"`).
- Defaults match the operationally sensible value (cite the relevant standard / ED document if there is one).

### Dropdowns

- Title-Cased option text. No abbreviations the user has to decode.
- Use `combo->setCurrentIndex(combo->findData(...))` rather than `findText` when option text could change.

### Line edits

- Wire to `QLineEdit::textEdited` (not `textChanged`) when the change should only fire on user input. Programmatic `setText()` does not emit `textEdited`.
- For an editable auto-suggested value, set a `<thing>_user_edited_` flag the first time `textEdited` fires, and stop auto-refreshing once it is set.

### Lists

- Items carry their identifier in `Qt::UserRole`.
- The empty state is a single `QListWidgetItem` with `Qt::NoItemFlags` and a sentence describing what is missing ("No data sources of type X present.").
- Population logic that recurs across more than one list is extracted into a helper.

## 5. Action buttons

- **Bottom button row**: `[Cancel] -- stretch -- [<action>]`. Cancel on the left; the action button (`Run`, `OK`, `Apply`, ...) on the right.
- Every `QPushButton` calls `setIcon(QIcon())` (clears the platform default icon) and `setToolTip(...)` immediately after construction. The tooltip describes what the button does, not what it is.
- Title Case for button text.
- **No colored buttons anywhere.** No green confirmation, no red reject, no yellow warning. The semantics of a button live in its label and its position (Cancel left, action right); coloring on top of that is noise, breaks dark mode, and is harder to scan once a dialog has more than two buttons. This applies to dialog footers, in-form buttons, and confirmations.
- Avoid `QMessageBox::question` — it injects a colored icon and re-introduces the green/red coloring above. Use `QuestionDialog::ask()` (project-wide question dialog) instead so confirmations match the rest of the UI.
- Same rule for `QMessageBox::Yes/No/Save/Discard` standard buttons: they pick up platform-themed colors. Build the buttons explicitly as plain `QPushButton`s when you need that wording.

### Action button gating

- The action button is enabled only when the dialog's current configuration would actually let the underlying task run. **The dialog must not let the user submit an invalid configuration.**
- Every precondition the task validates at runtime (required input present, parameters within accepted range, dependencies satisfied) is mirrored by a check in the dialog's `updateActionEnabled()` (or whatever you name it). The user must not see a "rejected because X" pop-up for something the dialog could have known.
- Re-evaluate the action button after every change that could affect any precondition. Wire it to whichever signal fires: list `itemChanged`, combo `currentIndexChanged`, etc.
- **Do not force-uncheck rows or clear values to fake the gate.** Preserve the user's selections; the gate is purely dialog-level.

## 6. Wording and terminology

- **Use the standard ATC / EUROCAE / ICAO term** where one exists. Do not invent project-only synonyms.
- **No abbreviations in user-visible text.** Spell things out ("Position Accuracy", not "Pos Acc"; "Reference Data", not "Ref Data"). Identifier-level abbreviations (`pos_acc_`) are fine in code.
- **No comparison operators in labels.** Use semantic names ("Acceptable (green)" / "Unacceptable (red)", not "Acceptable (green >=)"). The numeric semantics live in the docs and the report's settings recap.
- **No em-dash (U+2014) anywhere.** Use the ASCII hyphen `-`. The en-dash is allowed only in numeric/letter ranges (`L1-L4`).
- **Title Case** for window titles, page titles, group-box titles, button text, action links, and headings. ("All Days", "Scan Now", "Reference Data".)
- **No editorial / explanatory comments embedded in the UI.** If the user needs more context than a label and a tooltip provide, document it in the user manual or in a tooltip — not as inline copy in the form.
- **No colored buttons.** See [Section 5 - Action buttons](#5-action-buttons). Buttons are plain text; colors do not encode meaning.
- **Be consistent with the entry point.** A dialog launched from `Process > Analyze > MLAT` is titled "Analyse MLAT Data Source", and the verb "Analyse" appears in its Run-button effect, its result name, and its user-manual section. Do not switch to a synonym between the menu action, the dialog, the report, and the docs.

## 7. Threshold and color settings

- **Two-band model** for monotonic metrics (where one direction is "good" and the other is "bad"):
  - One spinbox **Acceptable (green)** — default at the operationally good end.
  - One spinbox **Unacceptable (red)** — default at the operationally bad end.
  - Everything in between is rendered orange. No "yellow" intermediate tier.
- For two-sided "near-target" metrics (e.g. ratios where deviation in either direction is bad): **two pairs**, lower band and upper band, each with its own Acceptable / Unacceptable spinbox.
- Thresholds default to the value from the relevant standard (cite ED-117 / ED-116 / ESASSP / etc. in the source comment).
- **Auto-pick deterministic choices instead of asking the user.** When the data picks itself ("prefer X if present, fall back to Y"), do that silently and document the choice in the report's settings recap row.

## 8. Persistence and configuration model

- Every dialog-controlled value is a `Configurable`-registered parameter. The dialog reads from the live struct on open and writes back on every change; **`Cancel` does not undo**, because the dialog and the live config share state. Document this if it is not obvious from context.
- Enum-typed selectors are stored as `int` parameters with an accessor that casts to the enum. JSON serialisation treats them as integers.
- **Default-true keyed maps** for "all enabled unless explicitly disabled" toggles (per CAT, per data source, per axis, ...):
  ```cpp
  bool useX(unsigned int id) const {
      auto k = std::to_string(id);
      if (use_x_.contains(k)) return use_x_.at(k).get<bool>();
      return true;          // default: enabled
  }
  ```
  Storage type: `nlohmann::json` object keyed by stringified ID.

## 9. Result reports

- Call `tm.beginTaskResultWriting(report_name, type)`. The argument becomes the displayed result name.
- The report's root section is already named `"Results"`. Use `*report->rootSection()` (or `report->getSection("Overview")` for the canonical default-selected page). Do **not** call `report->getSection("Results")` — that creates a redundant `Results:Results` nesting and breaks the default-section logic in `taskresultswidget.cpp`.
- Always add an `Overview` sub-section under the root. It contains:
  - a `Run Configuration` table (`Property`, `Value` columns) recapping the dialog's settings, and
  - any high-level dataset stats (record counts, UTNs, time range, ...).
- Each feature contributes one sub-section under the root. The framework owns the root and Overview; features own their sub-trees and never modify the framework's.

### Tables

- Use the canonical column sets for recurring shapes (e.g. `Item, Count, Min, Max, Description` for ASTERIX item tables, `Property, Value` for key/value recaps). Don't invent new column orders for the same content.
- Identifier-bearing entries are rendered with their human-readable name first and the structured ID in parentheses (e.g. `<name> (<sac>/<sic>)` for data sources). Never just the raw numeric ID.
- **Long cell values are line-broken with `\n`.** Comma-separated lists, multi-clause threshold descriptions, and any composite text running past ~40 characters: split into one item per line.
  ```
  green >= 0.95
  red <= 0.7
  orange in between
  ```
- Counts inside per-item summary tables use the project's count formatter (e.g. `formatCountWithPercent(count, total)`) so the format stays uniform.
- Items that are defined but not seen are rendered with count 0 and `CellStyleTextColorRed` row style.

### Default selected section

- The widget that displays a result selects `Report:Results:Overview` by default. Make sure that section exists. If it does not, the result widget falls back to the root and the user lands on an empty page.

## 10. Views

A View is a tab in the Main Viewbar showing one visualisation of the loaded dataset. Existing views: TableView, ScatterPlotView, HistogramView, GridView, ViewPoints, GeographicView (closed-source). The framework lives in [src/view/viewbase/](../view/viewbase/); the concrete views live in `src/view/<name>/` and `experimental_src/view/geographicview/`.

A View is the right surface when:
- The output is a visualisation of the **loaded dataset** (the in-memory cross-DBContent buffers), refreshed on Load.
- The user wants the visualisation alongside other visualisations, switchable via tabs and replicable in additional windows.
- The visualisation is interactive (cursor readout, selection, zoom) or has cross-View linkage (selecting in one View highlights in another).

It is **not** the right surface for a one-shot configuration dialog (use a modal task dialog), a persistent inspection panel (use a Flight Deck tool), or a report figure (the report system already has its own renderers).

### 10.1 Standard layout

Every View widget extends `ViewWidget` ([viewwidget.h:40-74](../view/viewbase/viewwidget.h#L40-L74)) and follows the same four-region layout:

```
+----------------------------------------------------------+
| ViewPresetWidget (optional) | ViewToolWidget (toolbar)   |
+--------------------------------------------+-------------+
|                                            |             |
|              ViewDataWidget                |  ViewConfig |
|         (the visualisation itself)         |   Widget    |
|                                            |  (settings) |
|                                            |             |
+--------------------------------------------+-------------+
| ViewInfoWidget (cursor / stats)  | ViewLoadStateWidget   |
+----------------------------------------------------------+
```

- The data widget is the central, dominant region. It claims the remaining space.
- The config widget docks on the right. Min width 400 px ([viewconfigwidget.h:70-85](../view/viewbase/viewconfigwidget.h#L70-L85)). It is collapsible via the toolbar's Toggle Configuration Panel action.
- The toolbar is **optional**: a View that is entirely passive (e.g. TableView) may omit it. Once a View has any interactive tool (Select, Zoom, ...), the toolbar is required.
- The status row ([viewloadstatewidget.h:37-99](../view/viewbase/viewloadstatewidget.h#L37-L99)) is mandatory; it is the only legitimate channel for "no data", "loading", "reload required" feedback.

Use the existing `ViewWidget::setDataWidget(...)` / `setConfigWidget(...)` calls in the View widget's constructor; do not re-invent the layout (see [tableviewwidget.cpp:30-37](../view/tableview/tableviewwidget.cpp#L30-L37) and [scatterplotviewwidget.cpp](../view/scatterplotview/scatterplotviewwidget.cpp) for the pattern).

### 10.2 Toolbar (`ViewToolWidget`)

The toolbar is a `QToolBar` populated from the View widget's constructor ([viewtoolwidget.h:44](../view/viewbase/viewtoolwidget.h#L44)). The conventional ordering is:

1. **Tool selectors** (radio-like exclusive group): one per interaction mode. `addTool(id, label, icon, shortcut)`.
2. **Spacer** between tool selectors and action buttons (`addSpacer()`).
3. **Action buttons**: invert / delete selection, zoom home, refresh. `addActionCallback(name, callback, updateCB, icon, shortcut)`.
4. **Toggle Configuration Panel** (added automatically by the framework, shortcut `C`).

Conventions:

- **Label wording**: Title Case, **verb-first**. "Zoom to Rectangle", not "Rect Zoom"; "Invert Selection", not "Invert"; "Zoom to Home", not "Home Zoom".
- **Icons**: descriptive PNG basenames in `data/icons/` (e.g. `select_action.png`, `zoom_select_action.png`, `select_invert.png`, `select_delete.png`, `zoom_home.png`, `refresh.png`). Reuse existing icons before adding new ones.
- **Shortcuts**: single keys for primary tools (`S` Select, `R` Zoom to Rectangle, `Z` Navigate). `Space` is reserved for "Zoom to Home". `Escape` cancels the active tool ([viewtoolswitcher.cpp](../view/viewbase/viewtoolswitcher.cpp)). `C` is reserved for Toggle Configuration Panel.
- **Tooltips**: every action has a tooltip describing its effect.
- **No colored buttons** in the toolbar (Section 5 rule applies here too).

A View that adds a new interaction mode picks a free letter for its shortcut and reuses an icon if the metaphor matches.

### 10.3 Selection vocabulary

Selection actions across plot views use a fixed set of verbs. Use these names; do not invent synonyms.

| Action | Effect | Icon |
|---|---|---|
| **Select** (tool mode) | Rectangular lasso, marks points as selected. | `select_action.png` |
| **Zoom to Rectangle** (tool mode) | Drag a rectangle to zoom in. | `zoom_select_action.png` |
| **Navigate** (tool mode) | Pan / default cursor. | (per-view) |
| **Invert Selection** | Toggle selected/unselected for every point. | `select_invert.png` |
| **Delete Selection** | Clear the selection. | `select_delete.png` |
| **Zoom to Home** | Reset zoom to fit data. | `zoom_home.png` |

`TableView` uses Qt's native row selection through its model; the verbs above do not apply there. Don't emulate them on widgets that already have a working native model.

### 10.4 Config widget (`ViewConfigWidget`)

Config widgets extend `TabStyleViewConfigWidget` ([viewconfigwidget.h:70-85](../view/viewbase/viewconfigwidget.h#L70-L85)), which is a `QTabWidget` inside a `QVBoxLayout`. Tabs separate concerns ("Display", "Data", "Export", ...); the framework does not impose tab names, but reuse what other Views already use.

Recurring components inside the config widget:

- **Variable / axis picker**: `VariableSelectionWidget` ([variableselectionwidget.h:37-113](../db/dbcontent/variable/variableselectionwidget.h#L37-L113)). Two `QLabel`s + a "Select" button that opens a DBContent-aware dropdown. One per axis. For an *ordered set* of variables (TableView columns), use `VariableOrderedSetWidget` instead.
- **Layer panel**: `ViewLayerPanelWidget`, hierarchical checkbox tree showing data sources and their fields. Used to hide / show series.
- **Display toggles**: bare `QCheckBox` rows for "Show Only Selected", "Use Presentation", "Log Scale", etc.
- **Color scale picker**: `ColorScaleSelection` (used by GridView, [gridviewconfigwidget.h:98](../view/gridview/gridviewconfigwidget.h#L98)).
- **Annotation switching** (if the View supports view-point annotations) lives in the layer panel's `Annotations` subtree, not in the config widget. See Section 10.7.
- **Export button**: a single `QPushButton "Export"` triggering an async export; do not put export under a sub-menu.

**Behaviour**:

- Changes apply **immediately** through Qt signals. No Apply / OK / Cancel buttons. The config widget shares state with the live `View` (a `Configurable`); persistence is automatic on shutdown.
- Disable controls that don't apply to the current state instead of hiding them; only hide widgets that are conditional on a feature flag or DBContent shape.
- Min width of 400 px is hardcoded; if your config exceeds the visible area, add tabs or grouping rather than expanding the panel.

### 10.5 Status feedback

`ViewLoadStateWidget` is the **only** channel for inline View status. It owns one `QLabel` (italic, state-dependent text) and one Refresh `QPushButton` (icon `refresh.png`). States ([viewloadstatewidget.h:40-48](../view/viewbase/viewloadstatewidget.h#L40-L48)):

- `NoData` -> "No data loaded"
- `Loading` -> "Loading..."
- `Drawing` -> "Drawing..."
- `Loaded` -> "Loaded"
- `ReloadRequired` -> "Reload Required" (refresh button changes label)
- `RedrawRequired` -> "Redraw Required"

Rules:

- **No popups for routine feedback.** "No data", "loading", "selection empty", "redraw required" all live inline. A `QMessageBox` only appears for an irreversible failure (export write error, file system error).
- **Disabled-not-popup.** Toolbar actions that are not currently applicable are disabled (their `update` callback returns false). Don't pop a message saying "select something first".
- **Selection-empty is not a state.** It is reflected in `ViewInfoWidget` (cursor / counter region) and via disabled toolbar actions, not in the load-state widget.

### 10.6 Variable pickers and naming

`VariableSelectionWidget` is the canonical picker. Conventions when wiring it into a View config:

- One picker per axis / column. Place them in a tab named after the axis kind ("Data", "X / Y Variables", "Columns").
- The picker's surrounding label uses Title Case ("Variable", "X Axis", "Color By").
- No abbreviated axis names ("X" alone is acceptable as an axis label inside the plot; the config widget uses the full "X Axis").

For columns / ordered sets in a tabular view, use `VariableOrderedSetWidget`.

### 10.7 Annotations / View Points

A View opts into annotations by extending `VariableView` and constructing its `ViewLayerPanelWidget` with the `show_annotations` flag set (already true by default for views whose `canShowAnnotations()` returns true). The annotation switch UI then appears in the layer panel as a separate `Annotations` subtree alongside the `DBContent` subtree.

- `VariableView::currentAnnotation()` returns the user-selected annotation; the data widget reads it and overlays it (time markers, regions, ...).
- The annotation subtree is radio-style single-select: when annotations are present, exactly one is always shown. Clicking another leaf swaps the active annotation; the active leaf cannot be unchecked. There is no "show variables instead" escape hatch in the UI - on view-point unshow, the framework reverts to variable display automatically.
- The flat 2-level structure (group -> annotation) is built by `AnnotationsRootItem::update(...)` ([annotationsrootitem.h](../view/viewbase/layerpanel/annotationsrootitem.h)) from `VariableView::annotations()`. Data widgets connect to `VariableView::annotationsChangedSignal` and call `annotationsRootItem()->update(...)` on every change. See [scatterplotviewdatawidget.cpp](../view/scatterplotview/scatterplotviewdatawidget.cpp), [histogramviewdatawidget.cpp](../view/histogramview/histogramviewdatawidget.cpp), [gridviewdatawidget.cpp](../view/gridview/gridviewdatawidget.cpp) for the wiring.
- A View that does not support annotations returns `false` from `canShowAnnotations()`. The `Annotations` subtree is then never created. Do not silently accept annotation calls and ignore them.
- **Saving** the current visualisation as a view point is not a per-View action. The Flight Deck's `View Points` tool ([viewpointswidget.h:34-101](../view/points/viewpointswidget.h#L34-L101)) handles export / import / management.

### 10.8 Wording inside Views

- **Title Case** for tool labels, action names, tab titles, axis labels, checkbox text. "Zoom to Home", not "zoom to home"; "Show Only Selected", not "show only selected".
- **Verb-first** for actions, **noun** for toggles. "Invert Selection" (verb), "Log Scale" (noun toggle).
- **Units in axis labels** when the variable carries one; otherwise just the variable name. The View pulls the unit from the DBContent variable, it does not invent units.
- No abbreviations the user has to decode in user-visible labels.

### 10.9 Color usage

Color is per-DBContent (data-source colors set in Configuration > Data Sources) or per-data-value (grid heat maps, color-scale selectors). The View framework does **not** define a global palette. Use:

- DBContent-supplied colors for series and points (TableView row colors, ScatterPlotView point colors).
- `ColorScaleSelection` for value-scaled palettes (GridView).
- The dedicated selection-highlight color (`ColorLegendWidget` selection state) for selected points.

Do not hard-code colors per view type. Do not use color to encode role on toolbar / config buttons (Section 5 rule: no colored buttons).

### 10.10 Persistence

Views inherit `Configurable` ([view.h:58-80](../view/viewbase/view.h#L58-L80)). The View model class receives a `nlohmann::json& config` in its constructor and registers all parameters; the config widget edits the same parameters directly. There is no Apply / Cancel; everything is auto-persisted on shutdown.

When adding a new View:

- All persistent settings are registered as `Configurable` parameters in the View class, not in the data / config widgets.
- The data widget reads from the View; the config widget writes to it through the View's setters.
- Sub-configurables (e.g. one config block per axis) follow the standard `generateSubConfigurable` / `checkSubConfigurables` pattern.

## 11. Reusable helpers

- The moment the same rendering, the same population logic, or the same form pattern appears in two places, extract a helper. Keep the helper framework-agnostic so each caller can adapt its native data structure to the helper's view type.
- Existing examples worth following:
  - `ASTERIXReportHelpers::renderDataItemTablesForDS(...)` ([asterixreporthelpers.h](../task/import/asterix/asterixreporthelpers.h)) — shared between the ASTERIX Import report and the Analyse-Data-Source data-item inspector. Both build a small `CategoryView` map and hand it to the helper.
  - `populateDSList(...)` in `analysedatasourcedialog.cpp` — used by the Reference and Test panes to populate their checkbox lists from a set of IDs and a "checked?" predicate.

---

## Quick checklist

### Picking the surface

- [ ] One-shot configuration → modal dialog launched from a menu action.
- [ ] Session-long inspection / filtering tool → Flight Deck tool.
- [ ] New visualisation → View in the Main Viewbar.
- [ ] Application-wide setting → action under `&Configuration`.
- [ ] None of the above? Ask before adding a new top-level surface.

### Adding a menu action

- [ ] Goes under the appropriate top-level menu (File / Import / Configuration / Process / UI). No new top-level menus.
- [ ] Title Case action text, no trailing punctuation.
- [ ] `&` accelerator on a letter not already used in that menu.
- [ ] `setToolTip(...)` set, sentence-cased, no terminal period.
- [ ] Standard `QKeySequence` if one applies, otherwise a non-conflicting `Ctrl+<letter>`.
- [ ] Disabled until its prerequisite (database open, etc.) is met.

### Adding a View

- [ ] Extend `ViewWidget` and use `setDataWidget` / `setConfigWidget`. Do not invent your own layout.
- [ ] Toolbar (`ViewToolWidget`) only if the View has interactive tools; selectors first, spacer, then action buttons. Title Case verb-first labels, descriptive icons reused from `data/icons/`, single-letter shortcuts; `Space` reserved for Zoom to Home, `C` for Toggle Configuration Panel, `Escape` cancels the active tool.
- [ ] Selection vocabulary: Select / Zoom to Rectangle / Navigate / Invert Selection / Delete Selection / Zoom to Home. Don't introduce synonyms.
- [ ] Config widget extends `TabStyleViewConfigWidget`. Variables picked via `VariableSelectionWidget` (one per axis) or `VariableOrderedSetWidget` (column sets). Changes apply immediately; no Apply / Cancel.
- [ ] Status feedback through `ViewLoadStateWidget` only. No popups for routine state. Disable inapplicable toolbar actions instead of warning the user.
- [ ] Annotations / view points via `VariableView` + the layer panel's `Annotations` subtree (radio-style single-select). Views that don't support annotations return `false` from `canShowAnnotations()`.
- [ ] All persistent settings are `Configurable` parameters on the View class; widgets read/write through setters. No per-widget JSON.
- [ ] Colors come from DBContent / `ColorScaleSelection`. No hard-coded per-view palette, no colored buttons.

### Building a dialog

- [ ] Title Case window title; minimum size at least 900 x 720.
- [ ] Splitter with tree on the left (header hidden, min width 180, two columns when badges are needed) and stack on the right wrapped in `QScrollArea`.
- [ ] Top-level tree items: inputs, configurable steps, outputs. No comment-only items.
- [ ] No paragraph-style help text on pages. Tooltips and form labels carry the description.
- [ ] Bare checkboxes in `QFormLayout` rows; description in the label column.
- [ ] Spinboxes have ranges, units, and sensible defaults (cite the standard).
- [ ] List panes follow the Selectable-list pattern: list, Select All / Select Nothing with stretch between them, optional shared parameter row.
- [ ] Bottom row: `[Cancel] -- stretch -- [<action>]`. Both have `setIcon(QIcon())` + `setToolTip(...)`. No colored buttons; no `QMessageBox::question`.
- [ ] The action button is disabled whenever the underlying task could not actually run; never force-uncheck rows to fake the gate.
- [ ] Two-band Acceptable / Unacceptable thresholds, no comparison operators in labels.
- [ ] `<name> (<id>)` for identifier-bearing entries; `\n` to wrap long table cells.
- [ ] Report uses `report->rootSection()`, adds `Overview` with `Run Configuration`, and has one sub-section per feature.
- [ ] Reusable rendering or population logic is pulled into a helper as soon as a second caller appears.
