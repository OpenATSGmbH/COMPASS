# View Models and View Render Widgets

Status: all six steps planned and agreed (2026-11-17), sections 7.1 to 7.5. No code written. All planning questions P1 to P13 are answered (section 9). Open: the items marked "not checked" or "not known", they are checks at implementation time.
Audience: the agent that continues the planning or implements this. Read `CLAUDE.md` first (library layering, code conventions).

Related documents:
- `handover_view_render_widgets_20261116.md`: task, agreed report behavior, open planning questions P1 to P13.
- `readme_report_figures.md`: earlier draft for the report figures (figure kinds, figure sources, tests). Parts are out of date, see the handover, section 5.
- `handover_grid_display_20261116.md`: the grid feature work that is done.

## 1. Goal

- Split each chart view (Histogram View, Scatterplot View, Grid View) into a model and a render widget.
- The same model and render widget classes serve three users:
  - The view: with tools, configuration and layer panel.
  - The report GUI: the figure without interaction, with a "View" button below it.
  - The report export: renders into a QImage at the export size. No view, no render delay.
- The drawing code and the data processing code exist only once.

## 2. Agreed Decisions

1. The design is a model / render widget framework.
2. `ViewModel` and `ViewRenderWidget` are base classes. Each view derives its own classes from them.
3. The view specific render widget is a light class. It talks to its view specific model directly.
4. The view specific model holds most of the data processing that the view data widgets do today.
5. A render widget can have a toolbar. The toolbar can be empty (report).
6. Tools are registered with one generalized function, `registerTool`, with `std::function` callbacks (section 5).
7. A tool has a kind with three values: `Trigger`, `Toggle`, `Mode`.
8. The tool description holds nothing about the mouse. The mouse handling is a service of the render widget.
9. Table View and Geographic View are special cases:
   - They need no render widget for the report.
   - They are not ported in the first iteration.
10. First iteration: an adapter class keeps `View`, `ViewWidget` and `ViewLoadStateWidget` unchanged (section 6).

## 3. Assembly Today

```
View (QObject, Configurable)            settings, load and update control
+- ViewWidget                           layout, owns the parts below
   +- ViewToolSwitcher, ViewToolWidget  tools and toolbar
   +- ViewDataWidget                    buffers + processing + layer data + drawing
   +- ViewConfigWidget
   +- ViewPresetWidget, ViewInfoWidget, ViewLoadStateWidget
```

- `ViewDataWidget` is model and render widget in one class.
- Class chain: `ViewDataWidget` -> `VariableViewDataWidget` -> `VariableViewStashDataWidget`.
  - `HistogramViewDataWidget` derives from `VariableViewDataWidget`.
  - `ScatterPlotViewDataWidget` and `GridViewDataWidget` derive from `VariableViewStashDataWidget`.
  - No other class uses `VariableViewDataWidget` or `VariableViewStashDataWidget`.
- `TableViewDataWidget` and `GeographicViewDataWidget` (`experimental_src`) derive from `ViewDataWidget` directly.
- `VariableViewDataWidget` already has separate hooks for the two phases:
  - Compute: `updateVariableData`, `processStash`.
  - Display: `updateVariableDisplay`, `commitStashDisplayData`.
- Tool handling, the same in all three views. A tool id is handled in three places:
  1. The view widget registers it (`ViewToolSwitcher::addTool`, `ViewToolWidget::addTool`).
  2. The chart view (`HistogramViewChartView`, `ScatterPlotViewChartView`, `GridViewChart`) reads `data_widget_->selectedTool()` in each mouse handler.
  3. `rectangleSelectedSlot` in the data widget checks the tool again, then zooms or selects, then calls `endTool()`.

## 4. New Base Classes

Location: `src/view/viewbase/` (library `compass_view`). The view specific classes go next to their view.

| Class | Base | Content | Taken from |
|---|---|---|---|
| `ViewModel` | QObject | Buffers, asynchronous processing, layer aggregates and snapshots, null/NaN counts, selection. Signals: display data changed, processing started, processing finished. | The data part of `ViewDataWidget` |
| `VariableViewModel` | `ViewModel` | Variable states, compute steps | `VariableViewDataWidget` |
| `VariableViewStashModel` | `VariableViewModel` | Stash | `VariableViewStashDataWidget` |
| `ViewRenderWidget` | QWidget | `setModel()`, `registerTool()`, `render(QImage&, QSize)`, draw state | The drawing part of `ViewDataWidget` |
| `ChartRenderWidget` | `ViewRenderWidget` | Holds the `ChartView`. Mouse services: drag, `requestRectangle()`. Delayed chart release. | Code that is three times in the data widgets today |
| `ViewTool` | struct | Section 5 | New |

Rules:
- A model is not a widget. Then the report export can use model and render widget without a view.
- A model has two inputs and one output:
  - Input in a view: buffers, processed as today.
  - Input in a report: annotation data, no processing.
  - Output for both: the display data, plus render options.
- A render widget reads only the display data and the render options. It does not read buffers or view settings.
- Signals go from the model to the widgets. Tool callbacks go from the render widget to the model.

Classes per view:

| View | Model | Model base | Render widget | Display data |
|---|---|---|---|---|
| Histogram | `HistogramViewModel` | `VariableViewModel` | `HistogramViewRenderWidget` | `RawHistogramCollection` |
| Scatterplot | `ScatterPlotViewModel` | `VariableViewStashModel` | `ScatterPlotViewRenderWidget` | `ScatterSeriesCollection` |
| Grid | `GridViewModel` | `VariableViewStashModel` | `GridViewRenderWidget` | `Grid2DLayer` + `Grid2DRenderSettings` |

Each view also gets a render options struct. The options structs have no base class.

Split of today's data widget content:

| View | Model | Render widget | Render options |
|---|---|---|---|
| Histogram | `HistogramGenerator`, row layer index, bin zoom (`saved_zoom_range_`), selection by bins, colors, hidden layers, title, axis name | Chart from the display data, rectangle to bin indices | Log scale |
| Scatterplot | Stash processing to the series, bounds, selection by rectangle | Chart, axis ticks, date/time format, axis zoom | Connection lines, axis data type and representation, hidden layers, title, axis names |
| Grid | Stash to `Grid2D` / `Grid2DLayers`, value range | Rendering, legend, chart, background, user view and zoom limits | `Grid2DRenderSettings`, grid opacity, background settings, sectors |

This table comes from the class declarations. The Histogram row is checked and agreed (section 4.1). The complete list for the Scatterplot View and the Grid View is open (section 9, P4).

### 4.1 Histogram: Display Data and Render Options (Agreed 2026-11-16)

Checked against `HistogramViewDataWidget::updateChart` (`src/view/histogramview/histogramviewdatawidget.cpp`).

| Value | Source today | New place |
|---|---|---|
| Series: name, color, bins, bin labels | `histogram_raw_` | Display data |
| Title | Annotation metadata, empty for variables | Display data |
| X axis label | Variable description, or annotation metadata | Display data |
| "Has data" flag | `histogram_raw_.hasData()` and (variables ok or annotation shown) | Display data |
| Log scale | View setting `use_log_scale` | Render option |
| Y axis label "Count", label angle 85, legend limit 8 (`MaxLegendEntries`) | Constants | Constants in the render widget |

```cpp
struct HistogramDisplayData      // output of HistogramViewModel
{
    RawHistogramCollection histogram;
    std::string            title;
    std::string            x_axis_label;
    bool                   valid = false;
};

struct HistogramRenderOptions
{
    bool use_log_scale = false;
    // legend mode for the report, added in step 2 (section 7.4)
};
```

- Colors and hidden layers are not render options. The color mode and the hidden layers go into the generator and into `toRaw()`. The colors are inside the display data. Both stay in the model.
- Log scale rule:
  - An annotation can carry its own log scale (`RawHistogramCollection::useLogScale()`). Today the data widget writes it into the view setting.
  - New: the render widget uses the log scale of the display data if it is set, else the render option.
  - The view takes the value into its setting when the display data changes. The checkbox then shows the same state as today.
  - The report needs no extra code.
- Not checked yet: `ChartView::applyPaletteTheme()`. The chart follows the application palette. Check for step 2 if an exported image depends on the theme of the running application.

### 4.2 Base Class Signatures (Agreed 2026-11-17)

```cpp
class ViewModel : public QObject
{
public:
    // inputs
    void loadingStarted();
    void updateFromSource(const DBContentDataSet& source, const std::vector<std::string>& names, bool reset, bool last);
    void loadingDone();
    void clearData();
    void recompute();                 // from the loaded buffers, can run asynchronously
    virtual bool setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata) = 0;
    void attachLayerPanel(ViewLayerTreeModel* model);

    // state
    bool hasData() const;
    bool hasAnnotation() const;
    bool hasPendingProcessing() const;

    // selection
    void invertSelection();
    void clearSelection();

signals:
    void displayDataChangedSignal();
    void selectionChangedSignal();
    void processingStartedSignal();
    void processingFinishedSignal();
};

class ViewRenderWidget : public QWidget
{
public:
    void registerTool(const ViewTool& tool);
    ViewToolSwitcher& toolSwitcher();

    virtual void redraw() = 0;                    // builds the display from the model's display data
    virtual bool render(QImage& image, const QSize& size, double scale = 1.0) = 0; // scale: section 7.2
    bool isDrawn() const;
    bool hasContent() const;
};
```

- `HistogramViewModel` adds: `displayData()`, `zoomToBins(first, last)`, `resetZoom()`, `selectBins(first, last)`, `viewInfo()`.
- `HistogramViewRenderWidget` adds: `setModel(HistogramViewModel*)`, `setOptions(const HistogramRenderOptions&)`.
- The model gets `DBContentManager&`, `DBContextManager&` and `AppState&` in its constructor.
- In the report the model gets only an annotation. The buffer functions are never called there.

### 4.3 Histogram: Calls Into the Data Widget (Checked 2026-11-17)

Group 1: calls through the base class `ViewDataWidget`. The callers do not change. The adapter forwards them.

| Caller | Functions | Adapter forwards to |
|---|---|---|
| `View` | `updateFromSource`, `beginDataReload`, `liveReload`, `hasPendingProcessing` | Model |
| `View` | `renderData`, `prepareForRender`, `hasScreenshotContent`, `isExporting` | Render widget |
| `View`, `VariableView`, `HistogramView` | `redrawData(recompute)` | Model (if recompute), then render widget |
| `ViewWidget` | `loadingStarted`, `loadingDone`, `clearData`, `databaseOpened`, `databaseClosed`, `isVariableSetLoaded`, `appModeSwitch` | Model |
| `ViewWidget` | `configChanged`, `onInit`, `viewInfoJSON` | Model and render widget |
| `ViewWidget` | `setToolSwitcher` | Goes away (A1) |
| `ViewLoadStateWidget` | `hasData` | Model |
| `ViewConfigWidget` | `attachLayerPanel` | Model |

Group 2: calls that are specific to the histogram.

| Caller | Call | New target |
|---|---|---|
| `HistogramViewWidget` (toolbar) | `isDrawn()` for the tool state | Render widget |
| `HistogramViewWidget` (toolbar) | `invertSelectionSlot`, `clearSelectionSlot` | Model |
| `HistogramViewWidget` (toolbar) | `resetZoomSlot` | Model (bin zoom) |
| `HistogramView::viewInfos_impl` | `getViewInfo()` (minimum, maximum, out of range, zoom active) | Model |
| `HistogramView::useLogScale` | Cast and assert only, no call | Remove |
| `HistogramViewChartView` | `selectedTool()` (3 places) | Goes away (`registerTool`) |
| `HistogramViewChartView` | `numBins()` | Display data |
| `LatexVisitor::visit(HistogramView*)` (`src/view/points/report/latexvisitor.cpp`) | `hasVisibleContent`, `hasData`, `hasAnnotations`, `isDrawn`, `renderPixmap` | Model (data state), render widget (image). Keeps the screenshot in step 1. |

Group 3: what the data widget takes from outside. The model needs these as inputs.

| Input | Use |
|---|---|
| `DBContentManager`, `DBContextManager` | `HistogramGeneratorBuffer` |
| `AppState` | Application mode (asynchronous or not), color mode |
| View variable 0 | Data to bin, axis label |
| Current annotation of `VariableView` | Annotation display data, title, axis label |
| Hidden layer ids of the layer panel | Row filter |
| `view_->selectionChangedSignal()` | Emitted after a selection |

Notes:
- The histogram config widget does not call the data widget.
- `exportDataSlot`, `exportDoneSlot` and `exportDoneSignal` exist in the data widget. Their connections in `histogramview.cpp` are commented out. Other users are not checked.

## 5. Tools

```cpp
struct ViewTool
{
    int          id;
    QString      name;
    QKeySequence key;
    QIcon        icon;
    QCursor      cursor;

    enum class Kind
    {
        Trigger, // not checkable, runs once (Zoom to Home, Invert Selection)
        Toggle,  // checkable, independent (configuration panel)
        Mode     // checkable, exclusive, one is always active (Navigate, Select, Zoom)
    };
    Kind kind;

    std::function<void(bool)>     callback; // Trigger: called once. Toggle / Mode: on and off.
    std::function<void(QAction*)> update;   // enabled / checked state, as ViewToolWidget::UpdateCallback today
};

void ViewRenderWidget::registerTool(const ViewTool& tool);
```

- The three kinds replace today's `addTool` (mode), `addActionCallback` with `Callback` (trigger) and `addActionCallback` with `ToggleCallback` (toggle).
- Mouse services of `ChartRenderWidget`:
  - `setDragEnabled(bool)`.
  - `requestRectangle(std::function<void(const QRectF&)>)`: the next rectangle (chart coordinates) goes to this callback. Then the default mode tool becomes active again, as `endTool()` does today.
- The base calls the rectangle callback queued. Today `rectangleSelectedSignal` uses a queued connection, because the callback can build the chart again during its own mouse event. Keep this.
- Who registers:
  - The render widget registers its display tools (navigate, zoom to rectangle, zoom to home).
  - The view registers the tools that need the model, with callbacks into the model.
  - The report registers nothing. The toolbar is empty.

Tools today:

| Tool | Histogram | Scatterplot | Grid | Needs the model |
|---|---|---|---|---|
| Navigate / default | yes | yes | yes | no |
| Zoom to rectangle | yes | yes | yes | Histogram only (fills the bins again, `HistogramGenerator::zoom`) |
| Zoom to Home | yes | yes | yes | Histogram only |
| Select | yes | yes | commented out | yes |
| Invert Selection, Delete Selection | yes | yes | commented out | yes |

Example, the Select tool of the Scatterplot View:
- `kind = Mode`.
- `callback`: on activation it calls `requestRectangle(...)` with a callback into the selection function of the model.

## 6. Changes to the Existing Classes, First Iteration

| Class | Change |
|---|---|
| `View`, `VariableView` | Stay. The view creates its model and fills the render options from its settings. `VariableView` keeps the annotation list and gives the current annotation to the model. |
| `ViewWidget` | Stays, same layout. |
| `ViewDataWidget` | Stays. Table View and Geographic View use it without change. |
| New: `ViewDataWidgetAdapter : ViewDataWidget` | Adapter. Holds model and render widget. Forwards the base functions (`updateFromSource`, `redrawData`, `clearData`, ...) to them. `View`, `ViewWidget` and `ViewLoadStateWidget` need no second code path. |
| `HistogramViewDataWidget`, `ScatterPlotViewDataWidget`, `GridViewDataWidget` | Removed. Each is split into model and render widget. |
| `VariableViewDataWidget`, `VariableViewStashDataWidget` | Removed when the third view is ported. |
| `ChartView` and its three subclasses | Stay. They get the render widget in place of the data widget. The tool checks in the mouse handlers go away. |
| `ViewConfigWidget` and subclasses | Stay. They read the layer data from the model in place of the data widget. |
| `SimpleGridViewChart` | Removed. It has no users. |

Assembly of a ported view:

```
View
+- <View>Model                          created by the view
+- ViewWidget
   +- ViewDataWidgetAdapter             adapter
   |  +- <View>RenderWidget             reads the model
   +- ViewConfigWidget, ...             as today
```

In the report:
- `SectionContentFigure` gets model and render widget from one factory function in `compass_view`.
- No `View`, no `ViewWidget`, no adapter.

Later iteration:
- Port Table View and Geographic View.
- Then `ViewWidget` talks to model and render widget directly. The adapter and `ViewDataWidget` go away.

## 7. Agreed After the First Draft

A3, class names (agreed 2026-11-16):
- The names in section 4 and section 6.
- `ChartView` (`src/view/viewbase/chartview.h`) and its three subclasses keep their names. They become internal parts of the render widgets.

A1, owner of the tools and the toolbar (agreed 2026-11-16):
- The render widget owns the `ViewToolSwitcher`. `registerTool` stores the tools there.
- The host decides if it shows a toolbar:
  - View: `ViewWidget` builds its toolbar (`ViewToolWidget`) from the switcher of the render widget, at the same place as today.
  - Report: no toolbar.
  - Table View and Geographic View: `ViewWidget` creates its own switcher, as today.
- `ViewToolWidget` loses its `ViewWidget*` argument. It uses it only for the screenshot button and the configuration panel toggle (added in `viewwidget.cpp`, `addScreenshotButton`, `addConfigWidgetToggle`). `ViewWidget` registers these two as normal tools.

A2, order of the work (agreed 2026-11-16). Do the steps one at a time. The existing view UI tests must pass after each step.
1. Base classes, adapter, and the Histogram View.
2. Histogram figures in the report GUI and in the export.
3. Scatterplot View, then its report figures.
4. The complete Grid View, and report figures for grids with other axes (content agreed later, section 7.5).
5. Report figures for geographic grids: background set from the view point, sectors, text labels (section 7.5).
6. Report figure background in the task dialogs of the evaluation and the data source analysis (section 7.5).

Reasons for this order:
- The histogram is the smallest data widget and needs no stash model.
- Its zoom tool needs the model. It tests `registerTool` completely.
- Step 2 tests the annotation input of the model, the factory function and `render(QImage&, QSize)` early.
- The grid background is the hardest part. It comes when the base classes are stable.

## 7.1 Step 1: Base Classes, Adapter, Histogram View (Agreed 2026-11-17)

Scope:
- New: `ViewModel`, `VariableViewModel`, `ViewRenderWidget`, `ChartRenderWidget`, `ViewTool`, `ViewDataWidgetAdapter` (section 4, section 5, section 6).
- New: `HistogramViewModel`, `HistogramViewRenderWidget`, `HistogramDisplayData`, `HistogramRenderOptions` (section 4.1, section 4.2).
- Toolbar change A1 (section 7).
- `HistogramViewDataWidget` is removed. Its callers go to the targets in section 4.3.
- `VariableViewStashModel` is not part of step 1. It comes with the Scatterplot View.
- `VariableViewDataWidget` stays until the third view is ported.

Constraints from the existing integration tests:
- The tests address the chart as `last_view.datawidget.chart_view` and inject the mouse there (for example `ui/histogramview/ui_histogramview_selection.py`).
  - The lookup searches each path element in all children below the last one (`findObjectNoPath`, `src/core/test/ui_test_find.cpp`, `Qt::FindChildrenRecursively`).
  - So the path stays valid if the adapter keeps the object name `datawidget` (set in the `ViewDataWidget` constructor) and the chart view keeps the object name `chart_view`. The render widget between them does not break the path.
- The tests compare the view info JSON with stored reference results. The view info JSON of the Histogram View must stay the same.

New unit tests (Catch2, registered in the `unit_tests/CMakeLists.txt` of the directory):

| File | Checks |
|---|---|
| `src/view/viewbase/unit_tests/test_viewtool.cpp` | `registerTool` for the three kinds. `Trigger`: callback once. `Toggle`: on and off. `Mode`: exclusive, callback on and off, default tool. The `update` callback sets the enabled state. |
| `src/view/viewbase/unit_tests/test_chartrenderwidget.cpp` | `requestRectangle`: one injected rectangle calls the callback once, queued, in chart coordinates. Then the default mode tool is active. No callback without a request. |
| `src/view/histogramview/unit_tests/test_histogram_render.cpp` | From a `HistogramDisplayData`: number of bar sets, bin labels, colors, title, x axis label. Log axis from the option. The log scale of the display data wins over the option. Legend shown up to 8 series, hidden above. Invalid data gives the empty chart. `render()`: requested size, not blank, same input gives the same image twice. |
| `src/view/histogramview/unit_tests/test_histogram_model.cpp` | Annotation input: a histogram feature JSON gives the expected display data (series, title, axis label, log scale). Malformed JSON returns false and gives invalid display data. |

- Patterns: `src/view/viewbase/unit_tests/test_chartselection.cpp`, `test_chartmargins.cpp` (real charts in unit tests).
- Buffer input of the model: the generator needs `DBContentManager` and `DBContextManager`. It is not known if they can be built in a unit test. The existing UI tests cover this path. Check at implementation time if a unit test is possible.

Existing integration tests that must pass, with unchanged reference results (`experimental_src/py/tests/`):
- `ui/histogramview/`: all 10 tests.
- `view_points/histogramview/`: all 6 tests.
- `ui/views/ui_open_views.py`, `analyze/analyze_report_variable_histogram.py`, `sensors/live_mode_transitions_check.py`.
- No new integration test in step 1. The report tests come with step 2.
- Read `experimental_src/py/readme_test_writing.md` and the `test` skill first. Never run the suite while a COMPASS instance that the user uses holds port 27960.

Done criteria:
- The build has no errors. `compass_tests` passes, with the four new test files.
- The listed integration tests pass on the local build, references unchanged.
- `python3 scripts/check_layering.py --build-dir build` reports no new violations.
- `HistogramViewDataWidget` is removed.
- Scatterplot View, Grid View, Table View and Geographic View are not changed, except for the toolbar change (A1).
- Changes reported per repository. No commit.

## 7.2 Step 2: Histogram Figures in the Report GUI and in the Export (Agreed 2026-11-17)

Code state (checked 2026-11-17):

| Topic | Finding |
|---|---|
| Report GUI | `Section::createContentWidget` (`src/result/report/section.cpp`) builds the content of one section when the user opens that section. Figures are created per section, not for the whole report. |
| Report GUI | `SectionContentFigure::addContentUI` adds a label and the "View" button. A locked figure gets a placeholder. |
| Figure content | A figure that is not flagged on demand stores its view point with the annotations in the report (`toJSON_impl`). A report read from the database returns the stored content. Only a figure flagged on demand builds its content at the first use (`viewableContent()`). The only one is the Target Errors Overview of a single evaluation result. |
| Export | `TaskResult::renderFigure` calls `figure.view()`, waits, and takes one image per view with content. The image name is the view instance name. `SectionContentFigure::obtainImages` uses it as the prefix of the image file. |
| Export size | There is no size setting. The image has the size of the view widget on the screen. |
| Palette | `ChartView::applyPaletteTheme` uses the application palette. A dark application gives dark report images. This is the same today. |
| Library layering | `compass_result` (layer 7) can use `compass_view` (layer 6) directly. The selection function and the factory go into `compass_view`. |

Not checked: what the export gives today when no Histogram View is open.

Figure selection rule for step 2 (agreed 2026-11-17):
- One function decides per figure: render widgets, or today's label and "View" button.
- A figure gets histogram render widgets if all are true:
  - The figure is not locked and not on demand. The report does not build content only to draw it.
  - The view point loads no data.
  - The view point holds histogram annotations only.
- All other figures: as today.
- A view point with more than one histogram annotation: one render widget and one image per annotation, each with its title.
  - Order: the same as in the view. `VariableView::scanViewPointForAnnotations` sorts the annotations into groups by the plot group name (alphabetical), and keeps the found order inside a group. The report uses the same function or the same rule.
- The same rule is the base for the later figure kinds (for example a figure with a grid and histograms).

Images per figure and image names (agreed 2026-11-17):
- The number of images is the number of annotations. It does not depend on the open views.
- Image name: the figure kind plus the annotation index, for example `histogram_0`, `histogram_1`. The names are unique inside one figure, so the image files do not overwrite each other.
- The annotation title goes into the image as chart title, as in the view.
- Figures on the view path keep the view instance name, as today.
- Tests that read image names from the JSON export of histogram figures can need new reference values. Not checked which tests do that.

Image size at export (agreed 2026-11-17):
- Facts: LaTeX places each image with a width of 18 cm (`src/core/latex/lateximage.cpp`). DOCX limits the width to 16 cm (`src/result/report/reportexporterdocx.cpp`).
- Qt Charts draws text with a fixed font size. More pixels alone make the text smaller relative to the chart. So the size has two parts:
  - Layout size: 1200 x 800.
  - Scale factor for the pixels: 2. The image has 2400 x 1600 pixels with the same layout.
- Both are constants in step 2. They become export settings in step 6.
- Signature: `render(QImage& image, const QSize& size, double scale = 1.0)`.
- Requirement: the exported image does not depend on the display scaling of the operating system. The same report gives the same image on a screen with 100 % and with 200 % scaling.
  - Approach: `render()` creates the QImage itself and sets its device pixel ratio (the scale factor) and its resolution (dots per meter for 96 dpi) explicitly. It does not use `QWidget::grab()`, which takes the factor of the screen.
  - Risk, not checked: Qt Charts can compute the text layout with the font metrics of the screen and paint with the metrics of the image. Then labels can be cut off or misplaced when the two differ.
  - Check at implementation time: export the same figure with `QT_SCALE_FACTOR=1` and `QT_SCALE_FACTOR=2`, and with `QT_FONT_DPI=96` and `QT_FONT_DPI=144`. The images must be equal. If they are not, fix it in `render()` before the step is done.
  - Known case: on the CI machine crunch the AppImage scales by 2 because of the monitor data (see the `test_ci` skill).
- Cost: larger image files than today. The JSON export holds them as PNG data, so its size grows too.

Palette (agreed 2026-11-17):
- Exported images always use the light palette. A report then looks the same for every user.
- The render widget in the report GUI and in the view follows the application palette, as today.
- The palette is an input of the drawing: `ChartView::applyPaletteTheme` takes the palette in place of `QApplication::palette()`. Unit tests can set it.
- Not checked: where the application defines its light palette. Find it at implementation time and use the same one.

Report GUI layout (agreed 2026-11-17):
- The content of a section is built when the user opens the section. No extra rule for the creation time is necessary.
- Per figure, top to bottom:
  - The label "Figure: <name>", as today.
  - One render widget per annotation: full width of the content area. The height follows the width (see below).
  - The "View" button, right aligned, as today. One button per figure, not per annotation.
- The render widget ignores the mouse wheel. The section scrolls, the chart does not zoom.
- Height of a render widget:
  - The widget keeps the ratio of the export image (1200 x 800, 3:2): height = width x 2/3.
  - Minimum height: 400 pixels. In a narrow window the chart stays readable.
  - Maximum height: the visible height of the section area. The user sees the full figure without scrolling.
  - At a limit the widget keeps the full width, and the chart fills it.
  - Implementation: the widget reports its height for a given width (`heightForWidth`). No own resize code.
- No user setting for the height.
- Each render widget gets an object name, so that UI tests can address it.

New unit tests for step 2 (agreed 2026-11-17):

| File | Checks |
|---|---|
| `src/view/unit_tests/test_figure_selection.cpp` | The selection function, one `SECTION` per case: histogram only (render widget), data load (view path), other feature (view path), histogram plus other feature (view path), three histogram annotations (three entries, names `histogram_0` to `histogram_2`, stored order). |
| `src/view/histogramview/unit_tests/test_histogram_render.cpp` (additions) | `render()` with size 1200 x 800 and scale 2 gives 2400 x 1600 pixels. With the light palette, a background pixel has the light color while the application palette is dark. |
| `src/view/unit_tests/test_figure_factory.cpp` | The factory builds model and render widget from a view point JSON with a histogram annotation. The display data has the expected series and title. |
| Height rule (in the test file of the render widget) | Widths that give a height below the minimum, inside the range, and above the maximum. |

- The locked and on-demand cases of the selection rule sit in `SectionContentFigure`. Not checked if `compass_result` has unit tests where this fits. If not, the report GUI test covers them.

New integration tests for step 2 (Python, `experimental_src/py/tests/`):

| Test | Checks |
|---|---|
| Report GUI | Open a report section with a histogram figure. The render widget exists. Its height is width x 2/3 inside the limits. The "View" button opens the Histogram View with the annotation. A figure on the view path has no render widget. |
| Report export | JSON export of a report with histogram figures, with no Histogram View open. Each image exists, has 2400 x 1600 pixels, is not blank, and has the name `histogram_<n>`. The export opened no view. |
| Display scaling | The same export with `QT_SCALE_FACTOR=1` and `QT_SCALE_FACTOR=2`. The images are equal. |

- Source of the histogram figures: proposal is the report of `analyze/analyze_report_variable_histogram.py`. This test is not read yet. Check if its report fits.
- The runtime command `get_report_figure` returns the stored view point of a report figure. The tests can use it.
- Read `experimental_src/py/readme_test_writing.md` first. Register each test, run it targeted, export reports into the DB folder.

Done criteria for step 2:
- The build has no errors. `compass_tests` passes with the new tests.
- The three new integration tests pass on the local build.
- All existing integration tests of step 1 pass. Reference values change only for image names of histogram figures.
- The display scaling check gives equal images.
- `python3 scripts/check_layering.py --build-dir build` reports no new violations.
- Changes reported per repository. No commit.

## 7.3 Step 3: Scatterplot View and Its Report Figures (Agreed 2026-11-17)

Checked 2026-11-17 against `ScatterPlotViewDataWidget` (`updateChart`, `updateDataSeries`, `updateFromAnnotations`, `updateVariableDisplay`).

Display data and options:

| Value | Source today | New place |
|---|---|---|
| Series: points, name, color, marker size, visible flag, selection series | `scatter_series_` (`ScatterSeriesCollection`) | Display data |
| Title, x and y axis label | Variables or annotation metadata | Display data |
| Axis is date/time, axis data type and representation (for the tick labels) | Variables or annotation | Display data |
| Data bounds (zoom to home) | `bounds_` | Display data |
| "Has data" flag | Series points and (variables ok or annotation shown) | Display data |
| Connection lines | View setting `use_connection_lines` | Render option |
| OpenGL for the series | Always on (`setUseOpenGL(true)`) | Render option `use_opengl` |
| Legend | Hidden (`setMaximumSize(0,0)`), the layer panel is the legend | Render option, section 7.4 |
| Current axis ranges (zoom) | Read from the old chart, set on the new chart | State of the render widget |

- `VariableViewStashModel` is created in this step.
- Calls from outside: the same pattern as the histogram (toolbar, tool checks in the chart view, `LatexVisitor`, `redrawData`).
- Zoom to rectangle and zoom to home change only the axis ranges. They are display tools of the render widget. Select needs the model.

OpenGL (agreed 2026-11-17):
- Render option `use_opengl`.
  - View: on, as today. The views can show millions of points.
  - Report GUI and export: off. The report draws with the CPU only, and a section with several figures creates no OpenGL surfaces.
- The drawing order differs. With OpenGL, Qt Charts orders the series by pointer value, and the code sorts for that. Without OpenGL the order is the order of adding. The render widget handles both. The selection series stays on top.
- Not known: the draw time without OpenGL for large series, and the point counts of the report scatterplots (reconstruction debug figures).
- Check at implementation time: the draw time of the largest report scatterplot without OpenGL. If it is too slow, a point limit or a reduction for the report is necessary. Ask the user then.

Existing tests:
- `experimental_src/py/tests/ui/scatterplotview/`: 11 tests (open, close, presets, connection lines, selection, zoom, zoom with variable change, axis labels, variable selection).
- `experimental_src/py/tests/view_points/scatterplotview/`: 7 tests.
- `src/view/viewbase/unit_tests/test_chartselection.cpp`: the selection series with OpenGL.

Connection lines (agreed 2026-11-17):
- Today two places in the data widget write the view setting: the annotation, and a limit (more than 100000 loaded values, `ConnectLinesDataCountMax`, turns the lines off).
- Annotation: the same rule as the log scale of the histogram (section 4.1). The render widget uses the value of the display data if it is set, else the render option. The view takes the value into its setting when the display data changes.
- Limit: it stays in the view. After a load, the view asks the model for the number of loaded values and turns its setting off above the limit. The model and the render widget do not know the limit.
- The behavior for the user does not change.

Zoom over a redraw (agreed 2026-11-17):
- Today: after a redraw (for example after a selection) the user keeps the zoom. When the annotation or an axis variable changes, the zoom goes back to the full data range. The data widget compares annotation indices and variable names for that (`last_drawn_anno_group_idx_`, `last_drawn_anno_idx_`, `last_drawn_var_`, `prior_draw_had_content_`).
- New: the display data has a value `domain_id` (a text). The model builds it from the annotation indices or from the two variable names.
- The render widget keeps its axis ranges while the `domain_id` stays the same. It goes to the data bounds when the `domain_id` changes or when the last draw had no content.
- The render widget needs no knowledge of annotations or variables.
- The Grid View has the same need (`user_view_`). It uses the same rule in step 4.

```cpp
struct ScatterPlotDisplayData    // output of ScatterPlotViewModel
{
    ScatterSeriesCollection   series;
    std::string               title;
    std::string               x_axis_label;
    std::string               y_axis_label;
    bool                      x_axis_is_datetime = false;
    bool                      y_axis_is_datetime = false;
    PropertyDataType          axis_data_type[ 2 ];
    dbContent::Representation axis_repr[ 2 ];
    boost::optional<QRectF>   bounds;
    std::string               domain_id;
    bool                      valid = false;
};

struct ScatterPlotRenderOptions
{
    bool use_connection_lines = false;
    bool use_opengl           = true;
    // legend mode for the report (section 7.4)
};
```

New unit tests for step 3 (agreed 2026-11-17):

| File | Checks |
|---|---|
| `src/view/scatterplotview/unit_tests/test_scatter_render.cpp` | From a `ScatterPlotDisplayData`: number of series, colors, marker sizes, hidden series not drawn, title, axis labels. Date/time axis when the flag is set. Connection lines from the option, and the value of the display data wins. With and without OpenGL: the selection series is on top. Invalid data gives the empty chart. |
| Same file | Zoom rule: the same `domain_id` keeps the axis ranges over a redraw. Another `domain_id` gives the data bounds. The first draw with content gives the data bounds. |
| Same file | `render()` without OpenGL: requested size, not blank, same input gives the same image twice. Legend: few series, many series ("... (n more)"), no entry for "Selected". |
| `src/view/scatterplotview/unit_tests/test_scatter_model.cpp` | Annotation input: a scatter series feature JSON gives the expected display data (series, labels, date/time flags, bounds, connection lines, `domain_id`). Malformed JSON returns false. |
| `src/view/unit_tests/test_figure_selection.cpp` (additions) | Scatter series only (render widget), several scatter annotations (names `scatterplot_0`, ...), scatter plus other feature (view path). |

- `src/view/scatterplotview/unit_tests/` does not exist yet. Create it with its `CMakeLists.txt` and register it as the other view test directories.

Integration tests for step 3 (`experimental_src/py/tests/`):
- Existing, must pass with unchanged reference results: `ui/scatterplotview/` (11 tests), `view_points/scatterplotview/` (7 tests), `ui/views/ui_open_views.py`, `eval/eval_report_variables_ui.py`.
- New, report: the complex reconstruction with the three debug settings on (`analyze_bias_correction_`, `analyze_association_`, `analyze_accuracy_estimation_`). The three scatterplot figures show a render widget in the report GUI. The JSON export holds their images (2400 x 1600 pixels, not blank, names `scatterplot_<n>`). `readme_report_figures.md` proposes this test as `calculate/calculate_report_scatter.py`, pattern `calculate/calculate_references.py`.
- Extend the display scaling test of step 2 with a scatterplot figure.

Done criteria for step 3:
- The build has no errors. `compass_tests` passes with the new tests.
- The existing and the new integration tests pass on the local build.
- The draw time of the largest report scatterplot without OpenGL is measured and reported to the user.
- `ScatterPlotViewDataWidget` is removed. Grid View, Table View and Geographic View are not changed.
- `python3 scripts/check_layering.py --build-dir build` reports no new violations.
- Changes reported per repository (the reconstruction figures are in `experimental_src`). No commit.

## 7.4 Legend in Report Figures (Agreed 2026-11-17)

- Requirement: report figures of all three kinds (histogram, scatterplot, grid) show a legend, in the report GUI and in the exported image.
- State today in the views:
  - Histogram View: chart legend for up to 8 series, hidden above. The layer panel shows the layers.
  - Scatterplot View: chart legend hidden. The layer panel is the legend.
  - Grid View: a separate legend widget (`ColorLegendWidget`) next to the chart, from `Grid2DRenderSettings::legend()`.
- The views keep their behavior. The legend is a render option that the report sets.
- Histogram and scatterplot in the report:
  - One legend entry per series: color and name.
  - Position: right of the chart, as a vertical list. More entries fit there than below the chart, and the grid legend is at the same side.
  - Many series: the legend shows as many entries as fit in the height. The last entry is then "... (n more)".
  - The scatterplot selection series ("Selected") gets no entry. A report figure has no selection.
  - Not known: how the Qt Charts legend behaves with too many entries. Check at implementation time. If the chart legend cannot do this, draw an own legend.
- Grid in the report:
  - The color legend (colors and values from `Grid2DRenderSettings::legend()`) at the right side.
  - The legend is a part of the grid render widget. `render()` draws it into the exported image. The Grid View uses the same legend.
- Histogram View with more than 8 series: the view stays as today (legend hidden). The report uses the rule above.
- Step assignment: the histogram legend option belongs to step 2, the scatterplot legend to step 3, the grid legend to step 4.
- Tests: for each kind, a unit test with few series (all entries shown) and with many series (last entry "... (n more)"), and that the rendered image contains the legend area.

## 7.5 Steps 4, 5 and 6: Grid View and Grid Figures (Agreed 2026-11-17)

Checked 2026-11-17 against `GridViewDataWidget`, `GridViewChart`, `GridBackgroundRenderer`, `gridbackgrounditems.h`, `GridView::Settings`, `Joined` (`src/eval/results/base/joined.cpp`).

Values that the grid drawing reads today:

| Value | Source today | Proposed place |
|---|---|---|
| Grid values and reference (`Grid2D`, `Grid2DLayers`) | Built from the stash, or from the annotation | Display data |
| Value type, grid resolution, maximum grid resolution | `GridView::Settings` | Input of the model (they change the grid, not the drawing) |
| Hidden layers | Layer panel | Input of the model |
| Title, x and y axis label, axis data type and representation | Variables or annotation | Display data |
| Data type and representation of the legend values | `GridView::currentLegendDataType()`, `currentLegendRepresentation()` | Display data |
| Render settings of the annotation (color map, minimum, maximum, legend fields) | `annotation_render_settings_` | Display data (optional), wins over the render options |
| Geographic grid or not | `annotationHasLonLatGrid()`, `GridView::hasLonLatAxes()` | Display data |
| Position extent of the database (empty chart, zoom limits) | `databasePositionExtent()` (data engine) | Display data |
| Color scale, number of steps, log scale, minimum, maximum | `GridView::Settings` (`render_color_*`) | Render options (`Grid2DRenderSettings`) |
| Grid opacity | `GridView::Settings::grid_opacity` | Render option |
| Background: show, color, map show, map file, map opacity, borders, sectors, hidden sector layers, hidden sectors | `GridView::Settings` (`background_*`) | Render options (one background settings struct) |
| Zoom and position of the user (`user_view_`), zoom limits | Data widget and `GridViewChart` | State of the render widget, with the `domain_id` rule of section 7.3 |
| Rendered image, legend, color map | Data widget | Render widget |

Background:
- `GridBackgroundRenderer` uses its `GridView&` for three things only: the background settings, the hidden sector lists, and the `DBContextManager` (sectors).
- New: it takes a background settings struct and `DBContextManager&`. No view.
- `prepareExport()` exists: it updates the map image for a viewport and waits for the tiles. `render()` of the grid render widget uses it.
- The background items of the layer panel (`GridBackgroundItem` and subclasses) keep their checked states in the `GridView` settings. They stay with the view. The view gives the changed settings to the render widget as render options (today: `backgroundSettingsChanged()`).
- In the report the factory needs the `DBContextManager` for the sectors.

Calls into `GridViewDataWidget` from outside:

| Caller | Calls | New target |
|---|---|---|
| `GridViewChart` | `background()`, `backgroundActive()`, `backgroundExtent()`, `gridBounds()`, `gridRendering()`, `gridIsNorthUp()`, `gridCellWidth()`, `setUserView()`, `clearUserView()`, `getView()` (grid opacity) | Render widget (same object family) |
| `GridViewChart` | `selectedTool()` | Goes away (`registerTool`) |
| `GridViewConfigWidget` | `hasValidGrid()`, `customRangeInvalid()`, `currentGrid()`, `currentGeoImage()` | Model (grid), render widget (current render settings, image) |
| `GridView` | `backgroundSettingsChanged()`, `clearUserView()`, `redrawData()` | Render options, render widget, adapter |
| `GridViewWidget` (toolbar) | `isDrawn()`, `resetZoomSlot()`, `invertSelectionSlot()`, `clearSelectionSlot()` (the selection tools are commented out) | Render widget, model |
| `LatexVisitor` | Screenshot | As for the histogram |

P11, evaluation sector overview (checked):
- It loads no target reports. Its base view point comes from `EvaluationCalculator::getBaseViewableNoDataConfig()`.
- The figure is not flagged on demand. So its view point with the annotations is stored in the report (`SectionContentFigure::toJSON_impl`). A report that is read from the database returns the stored content (`fromJSON_impl`). Nothing is built then.
- During the evaluation run, the sector result builds the content at once, while the details of the single results are in memory (`Joined::updateToChanges`, `hasStoredDetails()`).
- Fallback only: `Joined::getOrCreateCachedViewable` builds the content from the report table when the cached content was dropped (`updateToChanges(reset_viewable = true)`) and the details are not in memory any more. Not checked which user action causes that.
- So the sector overview figure gets its render widget from the stored annotations, as every other figure.
- Check in step 5: find the case that drops the cached content, and make sure that opening a section does not start a long build there.
- The only figure that is flagged on demand is the Target Errors Overview of a single result (`src/eval/results/base/single.cpp`). It stays on the view path, as agreed.

Annotations:
- The Grid View accepts only grid features (`GridView::acceptedAnnotationFeatureTypes`). Points, error ellipses and text are not read there. Only the Geographic View draws them.
- `VariableView::scanViewPointForAnnotations` makes a flat list of the accepted features, in groups by the plot group name. The relation between a grid and its overlay features in the annotation tree is lost there. The overlays (P6) need this relation. Not checked yet how the producers place the overlay features in the tree.

Existing tests:
- `experimental_src/py/tests/ui/gridview/`: 11 tests.
- `experimental_src/py/tests/view_points/gridview/`: 10 tests (background, legend, render settings, large grid, malformed data, ...).
- Unit tests in `src/view/unit_tests/`: `test_grid_*.cpp`, `test_gridbackground.cpp`.

Content of the two steps (agreed 2026-11-17):
- The Grid View has the background already. Its port cannot leave the background out.
- Step 4: port the complete Grid View.
  - `GridViewModel` (base `VariableViewStashModel`), `GridViewRenderWidget`.
  - `GridBackgroundRenderer` without `GridView&`.
  - The legend inside the render widget (section 7.4).
  - Report figures for grids with other axes than longitude / latitude.
  - `GridViewDataWidget`, `VariableViewDataWidget`, `VariableViewStashDataWidget` and `SimpleGridViewChart` are removed.
- Step 5: report figures for geographic grids. Report work only.
  - Background and sectors in the report.
  - Text labels.
  - The evaluation sector overview.
- Step 4 is the largest step.

Overlays (P6, decided by the user 2026-11-17):
- The grid render widget shows text labels as the only additional annotation content. No points, no circles, no lines.
- Facts: the producers put overlay features into own annotations next to the grid annotation, in the same view point (for example "RU position" in `experimental_src/analysis/mlat/mlatrucoverageinspector.cpp`). There is no explicit link between a grid and its overlays. They only share the view point.
- Rule: all text features (`ViewPointGenFeatureText`) of a view point are labels of each geographic grid in that view point. Grids with other axes get no labels.
- Drawing order: background, grid, national borders and sectors, text labels, legend.
- The text labels get no legend entry.
- The grid render widget is the same in the Grid View and in the report. So the Grid View shows the text labels too.
  - The feature types that a view accepts select the views that open for a figure (`acceptedAnnotationFeatureTypes`, `ViewManager::activateCompatibleViewTabs`). This set stays "grid only" for the Grid View. Else the Grid View opens for every figure with a text feature.
  - The text features are read in a second scan, only when a grid annotation is shown.
- Consequences:
  - Points and circles (for example the yellow circle at the RU position, the RU points of the MLAT RU effect figure) are not in the grid render widget. A figure with a grid and such features still gets the render widget, without them. The "View" button opens the views, and the Geographic View shows all features.
  - A figure with points only and no grid (SMR "Unassociated Reports") gets no render widget. It stays on the view path.
- This replaces decision 4.1, item 4 ("Overlays: points, circles, text") and the "overlays only" figure kind of `readme_report_figures.md`.

Background and sectors of geographic grids in the report (decided by the user 2026-11-17):
- One set of background values for all geographic figures of a report. The report GUI and the report export use the same set.
- The set is configurable in the report generation. These are the tasks that create the report: the evaluation and the data source analysis (Analyze Data Source task). It is not a setting of the report export.
- So the user sets the background before the task runs. The task stores the set with the report. The report GUI and every export of this report use the stored set.
- The background settings are one struct after step 4 (color, map on/off, map file, map opacity, borders, sectors, hidden sectors). The report set uses this struct.
- This replaces the handover item "Geographic grid background, set in the report export" (section 3, item 5) and proposal P8 for the background. The image size stays an export topic.
- Not checked: where the two tasks have their settings GUI, and where a report stores task wide values.
- Not checked: if the tile source of the map needs an internet connection or reads a local cache.

Storage of the background set (agreed 2026-11-17):
- The task writes the background set into each geographic figure, inside its view point next to the annotations. It is not stored once per report.
- Reasons:
  - The figure is complete in itself: the view point holds the grid, the render settings and the background.
  - The "View" button can give the same background to the Grid View. The view and the report figure then look the same.
  - A task can give another sector selection to single figures. For example the evaluation draws the evaluated sector layer in each sector overview.
  - The grid annotations already carry their render settings in this way (`Grid2DRenderSettings`).
- The task GUI holds one set. The task copies it into the figures and can change the sector selection per figure.
- A figure without a background set (for example from an older report) uses the defaults.

Fields of the background set (agreed 2026-11-17). One group "Report Figure Background" with the same fields in both task dialogs:

| Field | Type | Default |
|---|---|---|
| Background Color | Color | Black |
| Show Map | Checkbox | Off |
| Map | Selection of the map files, as in the Grid View | First map file |
| Map Opacity | Percent | 50 |
| Show National Borders | Checkbox | On |
| Show Sectors | Checkbox | On |
| Sectors | Checkbox per sector layer and per sector, as in the Grid View layer panel | Evaluation: the evaluated sector layers. Analysis: none. |

- These are the same values that the Grid View has in its layer panel (`GridView::Settings`, `background_*`).
- The defaults give the minimal map (national borders on black). "Show Map" on gives the map with transparency.
- Place:
  - Evaluation: the evaluation dialog, tab "Results Config" (`EvaluationReportTabWidget`, `src/eval/evaluationreporttabwidget.cpp`).
  - Data source analysis: `AnalyzeDataSourceDialog` (`src/task/analyze/analyzedatasourcedialog.cpp`). Its layout is not read yet. Propose the exact place to the user before the implementation.
- Both tasks store the set in their configuration (`Configurable`). The values stay for the next run.
- The CLI options and the runtime commands of both tasks can set the values, so that tests can use them. Check the existing parameter interfaces first (`--evaluate`, `--analyze_parameters`, see the `cli_rtcmd` skill).
- GUI conventions: load the `gui` skill before the dialog work.

Content of step 5 and step 6 (agreed 2026-11-17, replaces the first content of step 5 above and "export settings" as step 6):
- Step 5: report figures for geographic grids.
  - The render widget reads the background set from the view point. A figure without a set uses the defaults.
  - The evaluation and the data source analysis write the set into their geographic figures, with the defaults and the sector selection of the task.
  - Text labels.
  - The "View" button gives the background set to the Grid View.
  - Check: the case that drops the cached sector overview content (see P11 above).
- Step 6: the group "Report Figure Background" in both task dialogs, the task configuration, and the CLI and runtime command parameters.
- Image size as an export setting: not planned. The constants of step 2 stay (1200 x 800, scale 2). It can be added later if a user needs it.
- After step 5 all report figures work with fixed defaults. Step 6 makes them configurable.

Grid structs (agreed 2026-11-17):

```cpp
struct GridDisplayData           // output of GridViewModel
{
    boost::optional<Grid2DLayer>          layer;            // values and reference
    boost::optional<Grid2DRenderSettings> render_settings;  // of the annotation, wins over the options
    bool                                  is_geographic = false;
    std::string                           title;
    std::string                           x_axis_label;
    std::string                           y_axis_label;
    PropertyDataType                      axis_data_type[ 2 ];
    dbContent::Representation             axis_repr[ 2 ];
    PropertyDataType                      legend_data_type;
    dbContent::Representation             legend_repr;
    QRectF                                database_extent;  // empty chart and zoom limits
    std::vector<GridTextLabel>            labels;           // text labels, step 5
    boost::optional<GridBackgroundSettings> background;     // from the view point, step 5
    std::string                           domain_id;
    bool                                  valid = false;
};

struct GridRenderOptions
{
    Grid2DRenderSettings   render_settings;   // color scale, steps, log scale, minimum, maximum
    unsigned int           grid_opacity = 100; // percent
    GridBackgroundSettings background;        // color, map, map opacity, borders, sectors, hidden sectors
    // legend mode for the report (section 7.4)
};
```

- Rule for the two optional values in the display data (`render_settings`, `background`): the value of the display data wins over the render option. This is the same rule as for the log scale and the connection lines.
- Value type, grid resolution and hidden layers are inputs of the model, not render options. They change the grid, not the drawing.
- `GridBackgroundSettings` is the new struct for `GridBackgroundRenderer` (in place of `GridView&`). It has a JSON form (`toJSON` / `fromJSON`). This is also the form of the background set in the view point. The field name in the view point is defined at implementation time, next to the grid feature fields (`ViewPointGenFeatureGrid`).
- `GridTextLabel`: position (longitude, latitude), text, color. Read from `ViewPointGenFeatureText`. Check its fields at implementation time.

Tests for step 4 (agreed 2026-11-17):

| Kind | Tests |
|---|---|
| Unit, new | `src/view/unit_tests/test_grid_render_widget.cpp`: pixel color per cell from a small grid, null cells, minimum and maximum, the render settings of the display data win over the options, grid opacity, axis labels, color legend drawn into the image, zoom rule with `domain_id`, `render()` size and equal images. |
| Unit, new | `src/view/unit_tests/test_grid_model.cpp`: a grid feature JSON gives the expected display data (layer, render settings, geographic flag). Malformed JSON returns false. |
| Unit, changed | `src/view/unit_tests/test_gridbackground.cpp`: the renderer works from a `GridBackgroundSettings` without a view. JSON round trip of the settings. |
| Unit, additions | `src/view/unit_tests/test_figure_selection.cpp`: grid with other axes (render widget, name `grid_<n>`), geographic grid (view path until step 5). |
| Integration, existing | `ui/gridview/` (11 tests), `view_points/gridview/` (10 tests), `ui/views/ui_open_views.py`, unchanged references. |
| Integration, new | Analysis report with grids with other axes: render widget in the report GUI, images in the JSON export. |

Tests for step 5 (agreed 2026-11-17):

| Kind | Tests |
|---|---|
| Unit, additions | `test_grid_render_widget.cpp`: background color, national border pixels at a known position, a selected sector draws its outline and an unselected sector does not, a text label changes the pixels at its position, the background of the display data wins over the options, defaults without a background set. |
| Unit, additions | `test_grid_model.cpp`: text features of the view point give the labels. |
| Unit, additions | `test_figure_selection.cpp`: geographic grid (render widget), grid plus text (render widget with labels), grid plus points (render widget without the points), points only (view path). |
| Integration, new | Evaluation report: the sector overview figures have a render widget and exported images, with no view open. The stored view point holds the background set with the evaluated sector layer (runtime command `get_report_figure`). |
| Integration, new | Analysis report (MLAT): geographic grids exported. The RU coverage figure has no circle in the image. |
| Integration, new | The "View" button of a geographic figure: the Grid View shows the background of the figure. |

Tests for step 6 (agreed 2026-11-17):

| Kind | Tests |
|---|---|
| Integration, new | Evaluation: set the fields through the runtime command or CLI parameters. The stored view points hold the values. Two different values (for example national borders on and off) give different exported images. |
| Integration, new | Data source analysis: the same. |
| Integration, new | UI test: the group "Report Figure Background" exists in both dialogs. A changed value is in the task configuration after the dialog closes. |

- Use `src/view/unit_tests/test_gridbackground.cpp` as the pattern for the background tests. Map tiles in unit tests come from the repository, never from the network.
- Integration tests: read `experimental_src/py/readme_test_writing.md` and the skills `test`, `test_writing` first. Verify through the JSON report export into the DB folder. Use runtime commands, not DuckDB.

Done criteria for steps 4, 5 and 6, each step:
- The build has no errors. `compass_tests` passes with the new tests.
- The existing and the new integration tests pass on the local build.
- `python3 scripts/check_layering.py --build-dir build` reports no new violations.
- Changes reported per repository. No commit.

In addition:
- Step 4: `GridViewDataWidget`, `VariableViewDataWidget`, `VariableViewStashDataWidget` and `SimpleGridViewChart` are removed. Table View and Geographic View are not changed.
- Step 5: the check of the dropped sector overview cache is reported to the user.
- Step 6: the user manual sections of the two tasks describe the new fields (load the `documentation` skill first).

## 8. Risks

- `GridBackgroundRenderer` takes a `GridView&` today. It must work from render options and the DB context manager, without a view.
- The layer panel items of the grid background live in the data widget today.
- The layer panel tree items point into the layer snapshots. The snapshots move to the model. The update rules stay (main thread only, see `ViewDataWidget::updateLayerTree`).
- The asynchronous processing moves to the model. The rule for the destructor stays (`shutdownAsyncProcessor()` before the derived part is destroyed).
- `View` and the config widgets call the data widgets directly, and the data widgets have friends. These calls are not counted yet.
- The views must look and behave the same after each step.

## 9. Open Planning Questions

From `handover_view_render_widgets_20261116.md`, section 7.

| # | Question | State |
|---|---|---|
| P1 | Name and location of the classes | Agreed, section 4 and section 7 (A3) |
| P2 | Interface of a render widget | Agreed, section 4.2 |
| P3 | How the views add interaction | Agreed: `registerTool` and the mouse services (section 5) |
| P4 | Values that move from the view into the render options | Agreed for all three views: histogram (section 4.1), scatterplot (section 7.3), grid (section 7.5). |
| P5 | Number of images per figure at export | Agreed, section 7.2 |
| P6 | Overlays on geographic grids | Decided: text labels only (section 7.5) |
| P7 | Figure selection rule | Agreed for histogram figures, section 7.2. Other kinds open. |
| P8 | Export settings | Background and sectors: decided, they are settings of the report generation tasks and stored in each figure (section 7.5). Image size: constants in step 2, export setting open. |
| P9 | Report GUI: widget size, creation time | Agreed, section 7.2 |
| P10 | Legend with many series | Agreed, section 7.4 |
| P11 | Does the evaluation sector overview load target reports? | Checked: no. The content is stored as annotations in the report (section 7.5). |
| P12 | Order of the work | Agreed, section 7 (A2) |
| P13 | Export without main window | Decided 2026-11-17: out of scope for now. Later item: the render widgets make an export without the main window possible, the application always opens the main window today. |

## 10. Tests

- See `handover_view_render_widgets_20261116.md`, section 8, and `readme_report_figures.md`, section 8.
- New with this concept, to plan with the steps:
  - Unit tests per model: known buffers or a known annotation give the expected display data. No widget needed.
  - Unit tests per render widget: known display data and options give the expected chart content and image.
  - Unit tests for `registerTool`: kinds, exclusive mode tools, return to the default tool after a rectangle.

## 11. Rules

- Do not commit. The user commits.
- `experimental_src/` is a separate git repository. Report changes there separately.
- Write all text in ASD-STE100 (short sentences, active voice, lists), American English, no em-dash, no semicolons.
- After the work: `python3 scripts/check_layering.py --build-dir build` must report no new violations.
