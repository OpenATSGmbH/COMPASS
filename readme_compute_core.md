# Separating the COMPASS compute core from the Qt UI

*Design document / migration plan. Status: proposed, not started. Revised 2026-11-03: execution order and starting point after the dependency plan and the plugin plan added.*

## Context

COMPASS today is a single Qt Widgets desktop application. Everything — DB access, ASTERIX
import, reconstruction, evaluation, report generation — is reachable only by starting
`compass_client`, which builds a `QSplashScreen`, a `MainWindow`, and a full widget tree
before any work can begin. The documented "CLI batch mode" is not headless: it starts the
complete GUI and translates CLI flags into runtime commands
([client.cpp:545-762](src/app/client/client.cpp#L545-L762)).

The goal is a **headless computation core** that can be linked and run without a main window,
shipped as a new `compass_headless` executable, with all UI code isolated in separate
libraries. Qt stays — `QObject`, signals/slots, `QString`, `QThread`, and (per decision below)
`QImage`/`QPainter` remain available in the core. Only **QtWidgets and QtCharts** are banished.

### Decisions taken

| Question | Decision |
|---|---|
| Deliverable | A `compass_headless` executable alongside `compass_client` |
| Qt level in the core | **QtCore + QtGui, no QtWidgets** — headless binary is a `QGuiApplication` with `QT_QPA_PLATFORM=offscreen` |
| Offline view rendering | Later phase; the seam is designed and cut in Phase 4 so no re-refactor is needed |
| Churn | Long-lived refactor branch (`headless-split`) |
| Execution order (2026-11-03) | After the dependency plan and the plugin plan, in the near future |

### Execution order and starting point

1. **Dependency plan** ([readme_dependencies.md](readme_dependencies.md)): now.
2. **Plugin plan** ([readme_plugins.md](readme_plugins.md)): next.
3. **This plan**: in the near future.

**Starting point.** The two earlier plans change the code this plan works on.

After the dependency plan:
- **Libraries:** the module libraries have acyclic dependencies. There are two new ones, `compass_util` at the bottom and `compass_result`. `compass_result` holds `TaskResult`, the report model and the report export: today's `compass_report` plus `src/task/result/`.
- **Widgets in core:** a few generic widgets from `src/gui` (`questiondialog`, validators, `waitspinner`, `rangeedit`, `popupmenu`, `dialogs`) have moved into core.
- **`COMPASS`** lives in `compass_core` as a thin access point. A composition root in `compass_app` constructs the managers in explicit, ordered startup steps.
- **`COMPASS::mainWindow()` and the `QWidget*` view facades** are replaced by narrow interfaces in core.

Files that belong to another library were moved to that library's directory (directory = library). Some paths in this document therefore changed: `async.{h,cpp}` is in `src/core/job/`, `geotiff*` in `src/core/projection/`, `property`/`propertylist` in `src/core/util/`, and the result and report files are in the `compass_result` directory.

After the plugin plan:
- **Shared libraries:** every module is its own shared library with its own export macro.
- **Plugins:** ProbIMM and the Geographic View are plugins with a core and a UI registration phase. The composition root creates `PluginManager` right after `COMPASS`.

Where this changes a phase below, the phase carries a short note.

### Direct answers to the open questions

**Do you need a Qt event loop?** Yes, and the headless binary must run one. Direct-connection
signals work without it, but the existing architecture depends on a live loop in three ways
that cannot be removed: `RTCommandRunner` marshals every command step onto the main thread with
`Qt::BlockingQueuedConnection` ([rtcommand_runner.cpp:107-295](src/core/rtcommand/rtcommand_runner.cpp#L107-L295));
`JobManager` and `RTCommandManager` are `QThread`s posting back to the main thread; and
`Utils::Async::pumpUntil` is `QCoreApplication::processEvents` in a loop. `QGuiApplication::exec()`
supplies all of this. Nothing about signals requires `QApplication`.

**Can you keep `QObject`?** Yes, everywhere. `QObject`, `QThread`, `QTimer`, `QString` and
signals/slots are QtCore. `QSignalSpy` (used by rtcommand wait conditions) is QtTest, also fine.

**Should UI go into a separate library?** Yes — but split by *CMake target assignment*, not by
moving files. See below.

### What the exploration found (the reasons this is cheaper than it looks)

1. **The module libraries already exist.** At the time of writing they were static archives with
   cyclic dependencies (the link line repeats archives). After the dependency and plugin plans they
   are shared libraries forming an acyclic graph
   (`compass_util → compass_config → compass_core → compass_gui →
   {compass_filter, compass_reconstruction} → compass_view → compass_result → compass_eval →
   compass_task → compass_app`). The work is splitting each node, not creating the graph.
2. **The two biggest scattered-usage categories already have single choke points.**
   `Utils::Async::waitDialogAsync*` ([async.h](src/core/job/async.h)) is already a free-function
   facade taking `std::function` — **no `QWidget` in its signature**. The `ulog*` macro family
   ([msghandler.h](src/core/util/msghandler.h)) already funnels every user-facing message through
   one static `MessageHandler` with a `LogStore` sink and an existing `qApp`-null guard. Both need
   a swappable backend, not 300 edits.
3. **Only 33 files need real `QMessageBox` migration.** Of the 83 `.cpp` files using `QMessageBox`,
   50 are widget/dialog files that move to the UI libs wholesale. The genuine list is small enough
   to enumerate (see Phase 2).
4. **`allowUserInteractions()` already exists** as the headless contract
   ([compass.h:164-167](src/core/compass.h#L164-L167)), set by an RAII guard around every runtime
   command ([rtcommand.cpp:350-370](src/core/rtcommand/rtcommand.cpp#L350-L370)). It is honoured in
   only ~1 place today. It becomes the *policy*; the UI provider becomes the *mechanism*.
5. **Report definitions are already pure data.** `ViewableDataConfig` is raw JSON with no Qt;
   `SectionContentViewable` is a `std::function<shared_ptr<json::object_t>()>`; the `ViewPointGen*`
   feature model ([viewpointgenerator.h](src/view/points/viewpointgenerator.h)) is declarative and
   QtWidgets-free. Only *rasterization* is widget-bound.
6. **A headless renderer precedent already exists**: `Grid2DLayerRenderer::render()`
   ([grid2dlayerrenderer.h](src/view/gridview/grid2dlayerrenderer.h)) is pure data → `QImage`,
   no widget.

### The three real blockers

Blockers 1 and 2 are resolved by the dependency plan (thin `COMPASS` in `compass_core`, composition
root, narrow interfaces for the former `mainWindow()` callers). Blocker 3 remains for this plan.

1. **`compass.cpp` lives in `compass_app`.** The `COMPASS` implementation
   ([src/app/compass.cpp](src/app/compass.cpp), 1348 lines) is in the same static archive as
   `MainWindow`, `Client` and `ToolBox`. Every other library resolves `COMPASS::` symbols from
   that archive at final link. Nothing can link COMPASS without linking MainWindow.
2. **`COMPASS::mainWindow()` constructs the `MainWindow`** ([compass.cpp:1082](src/app/compass.cpp#L1082))
   rather than receiving it. Mitigating factor: only **9 call sites** tree-wide.
3. **rtcommand object resolution is `QApplication`-only.**
   `qApp->topLevelWidgets()` / `qApp->activeModalWidget()`
   ([rtcommand_helpers.cpp:78-96](src/core/rtcommand/rtcommand_helpers.cpp#L78-L96)) do not exist on
   `QGuiApplication`, and most command *implementations* live in
   [mainwindow_commands*.cpp](src/app/client/mainwindow_commands.cpp) operating on `MainWindow*` —
   registered only when `MainWindow`'s constructor runs.

---

## Target architecture

**Split by CMake target assignment, not by moving files.** Each module's `CMakeLists.txt` gains a
second `target_sources()` block feeding a sibling `_ui` target. Files stay where they are. On a
long-lived branch across 375k lines this is the difference between a rebasable branch and an
unmergeable one. The naming convention already in the tree (`*_widget.cpp`, `*dialog.cpp`,
`*combobox.h`) makes the assignment nearly mechanical.

```
                       ── headless ──                    ── UI ──
compass_util            (exists, dependency plan)
compass_config          (no Qt)
compass_core            QtCore+QtGui            compass_core_ui       QtWidgets
compass_db              (split out of core)     compass_db_ui
compass_reconstruction  (already clean)              —
compass_filter                                  compass_filter_ui
compass_view            view logic + renderers  compass_view_ui       View/ViewWidget/charts
compass_result          document model, export  compass_result_ui     report widgets, view part of LatexVisitor
compass_eval                                    compass_eval_ui
compass_task                                    compass_task_ui
compass_render          (Phase 6)               compass_gui           shared widgets (exists)
compass_app_core        AppRunner, CLI          compass_app_ui        MainWindow, Client, ToolBox
────────────────────────────────────────────────────────────────────────────────────────────
compass_headless        QGuiApplication         compass_client        QApplication
plugins                 registerCore()          registerUI()          (plugin plan)
```

Every target is its own shared library with its own export macro (plugin plan). When a class
moves into a `_ui` target, it switches to that library's macro: mechanical, by target. Plugins link
the INTERFACE target `compass_plugin_sdk`, which then resolves to the core and `_ui` libraries
without touching plugin CMake files.

**The enforcement lever is the build system, not discipline.** Today
[CMakeLists.txt:150](CMakeLists.txt#L150) puts `${Qt5Widgets_INCLUDE_DIRS}` in a global
`include_directories()`, so `#include <QWidget>` compiles anywhere. Deleting that line and moving
to per-target `target_link_libraries(<t> Qt5::Widgets)` makes every accidental widget dependency a
**compile error**. Do this early — it is the single highest-leverage change in the plan, and it
turns the rest of the work into "fix what the compiler reports".

---

## The application layer: what goes where

At the time of writing, every startup step lives in `Client::run()`
([client.cpp:367-776](src/app/client/client.cpp#L367-L776)), a method on a `QApplication` subclass.
The dependency plan already moved manager construction into a composition root in `compass_app`
with explicit startup steps (create `COMPASS`, register the manager creators, config tree, manager
`init()`, `MainWindow`). The plugin plan creates `PluginManager` right after `COMPASS`. `AppRunner`
is extracted from that root: everything up to manager `init()`, plus the CLI/crash/TBB parts listed
below. The `MainWindow` step becomes the `onCompassCreated` hook. Both binaries load the plugins.
The headless binary calls only their `registerCore()`, so the plugin startup check must then cover
only the core registry entries.

The division splits that one function in two. `compass_app_core` gets an `AppRunner` that is
agnostic about which Qt application class is running; `compass_app_ui` gets the presentation steps.

| Step in `Client::run()` today | Goes to |
|---|---|
| TBB `global_control` / `task_scheduler_init`, `QThreadPool::setMaxThreadCount` | **core** |
| `AA_UseHighDpiPixmaps`, `QApplication::setStyle("Fusion")` | ui |
| Splash screen construction + the 50 ms event pump | ui |
| `RTCommandManager::open_port_ = true` *(must precede the COMPASS ctor)* | **core** |
| `compass_ = make_unique<COMPASS>(*config_manager_)` | **core** |
| `applyDarkMode()`, `setFont(QFontDatabase::systemFont(...))` | ui |
| `compass_->init()`, `projectionManager().test()` | **core** |
| `expertMode()`, `sensorStatusTimeHack()` | **core** |
| `mainWindow(); init(); show(); splash.finish()` | ui |
| `--list_contexts` / `--set_context` console handling | **core** |
| `main_window.disableConfigurationSaving()` for `--no_cfg_save` | **core**, via `COMPASS` |
| CLI → rtcommand translation (~130 options) | **core** |
| `rt_man.startCommandProcessing()`, `setAppState(AppState::Running)` | **core** |
| `exec()` | both — different application class |

Also moving to `compass_app_core`, out of [main.cpp](src/app/client/main.cpp): the terminate and
crash-signal handlers, the alternate signal stack, `initCrashLogPath()`, and the
`main_thread_stall` watchdog (its heartbeat `QTimer` needs only an event loop, so it is
`QGuiApplication`-safe). The `Client::notify()` breadcrumb override
([client.cpp:812-833](src/app/client/client.cpp#L812-L833)) becomes a mixin both application
classes use.

The two resulting entry points:

```
compass_client                        compass_headless
  Client : QApplication                 HeadlessApp : QGuiApplication
  installs WidgetUIProvider             installs HeadlessUIProvider   (logs, returns defaults)
  installs MainWindow as ViewHost       installs NullViewHost
  installs WidgetTargetResolver         installs RegistryTargetResolver
  registers LiveViewFigureRenderer      registers data-driven renderers (Phase 6)
        \                                      /
         `------ AppRunner (compass_app_core) -'
                 CLI parsing, crash handlers, TBB,
                 COMPASS lifecycle, CLI→rtcommand, shutdown
```

Both binaries take the **same CLI options** and open the **same rtcommand TCP port**, so the
existing Python test framework drives either one unchanged. That equivalence is what makes
Verification step 3 below a meaningful regression test rather than a smoke test.

### How the binaries select behaviour — no compile-time defines

**Rule: there is no `COMPASS_HEADLESS` macro, and no compute code ever asks whether it is headless.**
The distinction is made by *linkage* and *runtime injection* only.

This is not a style preference — a define genuinely cannot work here. The existing
`USE_EXPERIMENTAL_SOURCE` ([global.h.in:56](src/core/util/global.h.in#L56), 65 `#if` sites) is a
**global, configure-time** value: one setting per build tree, every translation unit sees the same
value, one set of archives. A headless/UI flag is different in kind, because it would have to vary
*per binary* while `compass_core` is shared by both. That leaves only two options, and both are bad:

- Compile every module twice (`compass_core` **and** `compass_core_headless`) — doubles build time
  and archive count, and doubles every ODR and link-order subtlety in a tree that already needs
  `compass_app compass_core` [repeated in the link line](src/app/client/CMakeLists.txt#L40-L50) to
  resolve static-archive ordering.
- Put the define on the executable target only — in which case it cannot affect library code at
  all, which is where all the branching would need to happen.

So the mechanism is three layers, none of them preprocessor:

**1. Compile-time: the linker enforces it.** After Phase 0, `compass_core` does not link
`Qt5::Widgets` and does not get its include path. `#include <QWidget>` in a core file is a *compile
error*, not a convention. There is nothing to remember and no way to cheat.

**2. Runtime: injection, the pattern already used throughout COMPASS.** Compute code calls
`compass.ui().error(...)` or `compass.ui().askYesNo(...)`. Which implementation answers is decided
by what `main()` installed. This is the same dependency-injection approach `CLAUDE.md` already
documents for `COMPASS` itself ("not singletons — created by `Client` in `main()` and passed by
reference").

**3. Defaults are headless.** `COMPASS`'s constructor installs the null providers itself. A
core-only link is fully functional with no configuration — the UI is the thing that has to opt in,
not the other way round. This matters: it means forgetting to wire something up degrades to
"logged, non-interactive" rather than to a null-pointer crash.

**In one sentence: the client's `main()` installs widget implementations into `COMPASS`; the
headless `main()` installs nothing and uses the defaults the `COMPASS` constructor already set.**
It is install vs. don't-install, not install-A vs. install-B. Headless is the un-configured state.

The interface and its two implementations:

```cpp
// ── src/core/ui/ui_provider.h ──────────────────────── compass_core (no widgets)
class UIProvider {
public:
    virtual ~UIProvider() = default;
    virtual bool canPrompt() const = 0;
    virtual void error   (const std::string& title, const std::string& text) = 0;
    virtual bool askYesNo(const std::string& title, const std::string& text, bool def) = 0;
    virtual boost::optional<std::string> chooseFile(const std::string& title,
                                                    const std::string& dir) = 0;
};

// ── src/core/ui/headless_ui_provider.h ─────────────── compass_core
class HeadlessUIProvider : public UIProvider {
    bool canPrompt() const override { return false; }
    void error(const std::string& t, const std::string& x) override { ulogerr("UI") << t << ": " << x; }
    bool askYesNo(const std::string& t, const std::string& x, bool def) override {
        ulogwrn("UI") << t << ": " << x << " - non-interactive, using default " << def;
        return def;
    }
    boost::optional<std::string> chooseFile(const std::string&, const std::string&) override {
        ulogerr("UI") << "file selection requires a user interface"; return {};
    }
};

// ── src/core/ui/widget_ui_provider.h ───────────────── compass_core_ui (widgets)
class WidgetUIProvider : public UIProvider {
public:
    explicit WidgetUIProvider(COMPASS& c) : compass_(c) {}
    bool canPrompt() const override { return compass_.allowUserInteractions(); }
    void error(const std::string& t, const std::string& x) override {
        if (!canPrompt()) { ulogerr("UI") << t << ": " << x; return; }
        QMessageBox::critical(QApplication::activeWindow(), t.c_str(), x.c_str());
    }
    bool askYesNo(const std::string& t, const std::string& x, bool def) override {
        if (!canPrompt()) return def;
        return QMessageBox::question(QApplication::activeWindow(), t.c_str(), x.c_str(),
                                     QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
    }
private:
    COMPASS& compass_;
};
```

`COMPASS`'s constructor — signature unchanged, defaults set here, no flag and no `bool headless_`:

```cpp
COMPASS::COMPASS(ConfigurationManager& config_manager)
:   config_manager_(config_manager)
,   ui_      (std::make_unique<HeadlessUIProvider>())
,   progress_(std::make_unique<LoggingProgressReporter>())
,   app_ui_  (nullptr)                                  // null object
{ ... }

void        COMPASS::setUI(std::unique_ptr<UIProvider> ui) { ui_ = std::move(ui); }
UIProvider& COMPASS::ui()                                  { return *ui_; }
```

**`src/app/headless/main.cpp` — complete:**

```cpp
#include "apprunner.h"
#include <QGuiApplication>

int main(int argc, char** argv)
{
    AppRunner runner(argc, argv);                    // CLI parse, crash handlers, TBB
    if (runner.quitRequested())
        return runner.exitCode();

    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM"))
        qputenv("QT_QPA_PLATFORM", "offscreen");

    QGuiApplication app(argc, argv);

    // nothing installed - COMPASS keeps the defaults from its constructor

    return runner.exec(app);
}
```

**`src/app/client/main.cpp` — complete:**

```cpp
#include "apprunner.h"
#include "widget_ui_provider.h"
#include "dialog_progress_reporter.h"
#include "widget_target_resolver.h"
#include "liveviewfigurerenderer.h"
#include "mainwindow.h"
#include <QApplication>

int main(int argc, char** argv)
{
    AppRunner runner(argc, argv);
    if (runner.quitRequested())
        return runner.exitCode();

    Client app(argc, argv);                          // QApplication + notify() breadcrumbs
    QApplication::setStyle(QStyleFactory::create("Fusion"));
    SplashScreen splash;
    splash.show();

    // runs after the COMPASS ctor, before startCommandProcessing()
    runner.onCompassCreated([&](COMPASS& compass)
    {
        compass.setUI      (std::make_unique<WidgetUIProvider>(compass));      // 1
        compass.setProgress(std::make_unique<DialogProgressReporter>());       // 2

        MainWindow& mw = MainWindow::create(compass);
        compass.setApplicationUI(&mw);                                         // 3
        compass.viewManager().init(mw.viewHost());                             // 4

        rtcommand::setTargetResolver(std::make_unique<WidgetTargetResolver>());
        registerWidgetCommands(compass);
        FigureRendererRegistry::instance()
            .add(std::make_unique<LiveViewFigureRenderer>(compass));           // 5

        mw.init();
        mw.show();
        splash.finish(&mw);
    });

    return runner.exec(app);
}
```

The `onCompassCreated` hook exists because ordering is load-bearing: `MainWindow` must be built
after `COMPASS` but before `startCommandProcessing()`. That ordering is currently implicit in the
middle of `Client::run()`; making it an explicit callback is what lets the two binaries share the
sequence.

**The complete difference between the two binaries:**

| | headless | client |
|---|---|---|
| Qt application object | `QGuiApplication` | `Client : QApplication` |
| `QT_QPA_PLATFORM` | forced to `offscreen` | untouched |
| `UIProvider` | ctor default `HeadlessUIProvider` | `setUI(WidgetUIProvider)` |
| `ProgressReporter` | ctor default `LoggingProgressReporter` | `setProgress(DialogProgressReporter)` |
| `ApplicationUI` | `nullptr` (null object) | `setApplicationUI(&mainWindow)` |
| `ViewHost` | never called → null host | `viewManager().init(mw.viewHost())` |
| rtcommand target resolver | ctor default registry resolver | `setTargetResolver(WidgetTargetResolver)` |
| UI-only rtcommands | not registered | `registerWidgetCommands()` |
| Figure renderers | data-driven (Phase 6) | `LiveViewFigureRenderer` |
| Splash / style / font | — | Fusion, splash, system font |

Everything else — `AppRunner`, `COMPASS`, and all nine compute libraries — is byte-identical
between the two binaries.

#### `UIProvider` never detects anything

`UIProvider` is abstract; it contains no "am I headless?" check because there is nothing to check.
`HeadlessUIProvider` *is* headless behaviour, `WidgetUIProvider` *is* GUI behaviour, and which
object exists was settled by whichever `main()` ran. The virtual call replaces the branch. Nothing
constructs it conditionally either — `COMPASS`'s constructor installs the headless default
unconditionally, and only `compass_client`'s `main()` overwrites it. There is no `if` anywhere in
the path.

There **is** a related question compute code legitimately asks, and it is worth keeping strictly
separate: *"can I prompt right now?"* That is not the same as *"am I the headless binary"*, because
**the GUI client is also non-interactive while it runs a batch.** `allowUserInteractions()` already
models exactly this, flipped false by an RAII guard around every runtime command
([rtcommand.cpp:356-370](src/core/rtcommand/rtcommand.cpp#L356-L370)). So there are two independent
axes:

| | GUI client, user driving | GUI client, running rtcommands / CLI batch | `compass_headless` |
|---|---|---|---|
| `ui()` implementation | Widget | Widget | Headless |
| `allowUserInteractions()` | true | **false** | false |
| `ui().canPrompt()` | true | **false** | false |

So `canPrompt()` derives from both — `HeadlessUIProvider::canPrompt()` returns false always;
`WidgetUIProvider::canPrompt()` returns `compass.allowUserInteractions()`. Compute code consults
`canPrompt()` and gets the right answer in all three columns.

**The consequence is the important part: headless is not a separate mode, it is the permanent case
of "cannot prompt".** Every non-interactive path the headless binary takes is already exercised by
the GUI client during batch and CI runs today. That is why there must be **no `isHeadless()`
predicate** — branching on binary identity would create code paths that only one binary ever runs,
which is precisely the class of bug that makes headless output diverge from client output. If a
branch is ever tempting, it is a `canPrompt()` branch.

**Trap to avoid: self-registration via file-static registrar objects.** It is tempting to have
`compass_core_ui` register itself via a file-static registrar object, so `main()` stays short. With
static libraries this silently does nothing — the linker only pulls in object files that resolve an
undefined symbol, so a translation unit nobody references is dropped along with its registrar. With
the shared libraries of the plugin plan it would work, but it runs at library load time in
unspecified order, before `COMPASS` exists. **Register explicitly from `main()`** (in the
composition root's startup steps). It is more verbose, but it is greppable, debuggable, and
correctly ordered.

**Where build-time conditionals legitimately remain:** whether an optional library *exists* at all —
today `EXPERIMENTAL_SRC` gating the geographic view, later the offscreen renderers. That is already
handled by CMake `if()` around `target_link_libraries` plus the corresponding registration call in
`main()`, and needs no macro in compute code. By the time this plan starts, ProbIMM and the
geographic view are plugins, and the plugin plan has reduced their `USE_EXPERIMENTAL_SOURCE` sites
(65 in total at the time of writing) to the composition root. The remaining
sites are out of scope for this effort — but note that several of them
(e.g. [reportexporter.cpp](src/task/result/report/reportexporter.cpp#L32) `#if`-ing on experimental
for DOCX export) are exactly the exporter/renderer-registry case, and would be better served by the
same pattern if they are ever revisited.

### What the headless binary deliberately cannot do

- Anything requiring a modal answer that has no sensible default. `HeadlessUIProvider` logs the
  question via `ulogerr` and returns the caller's default; commands that genuinely need a human
  fail with a clear error rather than hanging. This is the behaviour `allowUserInteractions()`
  already promises but does not yet deliver.
- Figure rendering, until Phase 6. Report *text* is complete from Phase 5; figures come with the
  renderers.
- UI-driving rtcommands (`ui_test::*`, window/tab manipulation). These stay registered only in
  `compass_app_ui`, so headless reports them as unknown commands rather than accepting and
  silently doing nothing.

### One bug to fix while splitting this out

`Client::run()` returns `bool`, and [main.cpp:529](src/app/client/main.cpp#L529) does
`if (!client.run())`. But the `--list_contexts` path
[returns `0`](src/app/client/client.cpp#L523) (i.e. `false` → treated as failure, process exits -1)
and the `--set_context` error path [returns `1`](src/app/client/client.cpp#L533) (i.e. `true` →
treated as success). Both are inverted — someone wrote them as `main()`-style exit codes. Fix when
these move into `AppRunner`, which should return a proper exit code rather than a `bool`.

---

## Phases

Each phase ends with `compass_client` building and its test suite green. That is the rebase anchor.

### Phase 0 — Build-system guardrail (small, do first)

- Delete `${Qt5Widgets_INCLUDE_DIRS}` and `${Qt5Charts_INCLUDE_DIRS}` from the global
  `include_directories()` in [CMakeLists.txt](CMakeLists.txt#L150).
- Add `Qt5::Widgets` explicitly to every target that currently needs it (initially: all of them).
- Land this alone. It changes no behaviour, but from here on removing `Qt5::Widgets` from one
  target's link line produces an exact, mechanical worklist for that module.
- Also refresh the stale `CLAUDE.md` build section (it documents a `compass` target that does not exist).

### Phase 1 — The UI abstraction layer

*Note: the dependency plan replaced `COMPASS::mainWindow()` with a narrow interface in core for its
call sites. `ApplicationUI` replaces that interface.*

New `src/core/ui/` (QtCore/QtGui only), holding pure-virtual interfaces plus null implementations:

- **`UIProvider`** — `error()`, `warning()`, `info()`, `askYesNo()`, `askText()`, `chooseFile()`.
  Headless impl logs via `ulog*` and returns the caller-supplied default. Widget impl (in
  `compass_core_ui`) is exactly today's `QMessageBox`/`QFileDialog` code.
- **`ProgressReporter`** — the backend for `Utils::Async::waitDialogAsync*`. Headless impl logs
  progress; widget impl is today's `QProgressDialog`. **`async.h`'s public signatures do not change** —
  only [async.cpp](src/core/job/async.cpp) is rewritten to delegate.
- **`ApplicationUI`** — the `mainWindow()` replacement, exposing only what the 9 call sites
  actually need (raise/activate, obtain a parent for a modal, set busy cursor). Null object in headless.
- **`ViewHost`** — what `ViewManager::init(QTabWidget*)` needs: create container, add/remove view tab,
  set current tab. Null object in headless.

Route `msghandler::MessageHandler` through `UIProvider` — this is the existing seam, it already has
the `qApp` guard and the `LogStore` sink.

`COMPASS` gains `UIProvider& ui()` plus a setter; the composition root installs the widget provider
on the client side; default is the headless provider. **No behaviour change in this phase** —
the widget provider does verbatim what the current code does.

### Phase 2 — Migrate compute-module call sites off QtWidgets

Mechanical, module by module, guided by the compiler once each module's `Qt5::Widgets` link is dropped.
*Note: since the dependency plan, the report files below belong to `compass_result` and live in its
directory instead of `src/task/result/report/`.*

The full `QMessageBox` worklist for files that stay headless (33):

```
src/app/compass.cpp                              src/task/import/asterix/asteriximporttask.cpp
src/core/util/msghandler.cpp                     src/task/import/gps/gpstrailimporttask.cpp
src/db/dbcontent/dbcontentmanager.cpp            src/task/import/json/jsonimporttask.cpp
src/db/interface/dbinterface.cpp                 src/task/import/view_points/viewpointsimporttask.cpp
src/eval/evaluationcalculator.cpp                src/task/reconstructor/reconstructortask.cpp
src/eval/evaluationmanager.cpp                   src/task/result/report/sectioncontenttable.cpp
src/eval/requirement/group.cpp                   src/task/taskmanager.cpp
src/eval/results/base/joined.cpp                 src/view/histogramview/histogramviewdatasource.cpp
src/eval/results/evaluationresultsgenerator.cpp  src/view/livecontroller.cpp
src/task/analyze/analyzedatasourcetask.cpp       src/view/points/report/viewpointsreportgenerator.cpp
src/task/assoc/createartasassociationstask.cpp   src/view/points/viewpointstablemodel.cpp
src/task/calc/radarplotpositioncalculatortask.cpp src/view/scatterplotview/scatterplotviewdatasource.cpp
src/app/client/client.cpp                        src/view/tableview/tableviewdatasource.cpp
src/app/watchdog/main.cpp                        src/view/viewmanager.cpp
+ 5 files in experimental_src (geographicviewdatasource, geometryitemprovider, test/*)
```

#### Custom dialogs owned by compute code

`UIProvider` handles generic interactions (message, yes/no, file choice). It does **not** cover
compute code that constructs a purpose-built dialog — `EvaluationDialog`
([evaluationmanager.cpp:213](src/eval/evaluationmanager.cpp#L213)), `ReportExportDialog`
([taskmanager.cpp:671-692](src/task/taskmanager.cpp#L671-L692)), `AsyncTaskDialog`, the ~10 context
dialogs in `src/core/context/`, `DBContentEditDialog`, `asterixjsonparserwidget`.

The wrong fix is adding a method to `UIProvider` per dialog — that grows a god-object interface
that every module must depend on. Instead **triage each into one of four categories.** After Phase 0
drops `Qt5::Widgets` from a module, each of these surfaces as a compile error naming the dialog
type, so the compiler produces the worklist.

**A — Parameter collection (the common case). Invert the direction.**
The dialog gathers settings that already have config-persisted defaults, then calls one function.
`ReportExportDialog` is exactly this: it collects base dir, report name, author and comment, then
calls `report_export_.exportReport(result, mode, fn, dir, section, show_ui)`
([reportexportdialog.cpp:344](src/task/result/report/reportexportdialog.cpp#L344)). The real work is
already a plain method; the dialog is a parameter form wrapped around it.

Split into a settings struct + an operation + a form:

```cpp
// compass_task  — plain data, Configurable-persisted
struct ReportExportSettings { std::string base_dir, report_name, author, comment; };

// compass_task  — the operation, no UI in the signature
ResultT<json> ReportExport::exportReport(TaskResult&, ReportExportMode,
                                         const ReportExportSettings&, ProgressReporter&);

// compass_task_ui — edits the struct, returns false if cancelled
bool editReportExportSettings(ReportExportSettings&, QWidget* parent);
```

Crucially, **the core never calls the form.** `main()`-installed UI code edits the settings *before*
invoking the operation. Compute code cannot "need a dialog" because it only ever receives a filled
struct. The rtcommand path already proves the parameters can come from elsewhere — and note
`no_interaction_mode` already exists on this dialog and auto-triggers via `showEvent`
([reportexportdialog.cpp:88](src/task/result/report/reportexportdialog.cpp#L88)), so the headless
behaviour is defined today; it just still needs a widget stack to reach it.

**B — Progress display.** `AsyncTaskDialog`, the progress half of `ReportExportDialog`, the
`QProgressDialog` inside `Utils::Async::waitDialogAsync`. Already covered by the `ProgressReporter`
interface from Phase 1. No new interfaces.

**C — A genuine mid-operation decision with no defensible default.** The real example is the
DB-context conflict dialog: a merge conflict the algorithm cannot resolve alone. This is where a
dedicated interface *is* justified — but model it as **data, not as a dialog**, and put it next to
the domain concept rather than on `UIProvider`:

```cpp
// src/core/context/ — a narrow role interface, not a UIProvider method
class ConflictResolver {
    virtual Resolution resolve(const ConflictRequest&) = 0;   // UseLocal | UseIncoming | Abort
};
```

Headless returns a configured policy (default `Abort`, overridable by a CLI option); the widget
implementation shows today's dialog. Several small role interfaces beat one wide `UIProvider`:
each is independently testable, and a module depends only on the ones it actually uses.

**D — The dialog *is* the feature.** `asterixjsonparserwidget`, `DBContentEditDialog`, the context
edit/create/rename dialogs. These edit configuration interactively and have no headless meaning.
They move to `_ui` wholesale and get **no** core interface at all. Headless consumes the resulting
`Configurable` JSON, which is already persisted — it never needs to author it. `readme_cli.md`
already states this limitation ("Configuration of the application still has to be performed using
the GUI"), so this is documenting existing reality, not a new restriction.

**Triage rule of thumb:** if the dialog's output is a value the operation could equally receive from
a config file or a CLI flag, it is **A**. If it only shows progress, it is **B**. If the operation
genuinely cannot continue without a human choice, it is **C**. If it exists to edit config, it is **D**.
Expect most to be A or D.

Also in this phase:

- **`QApplication::setOverrideCursor`/`restoreOverrideCursor`** → `ApplicationUI::busy(bool)` /
  an RAII `BusyScope`. Heaviest in [compass.cpp](src/app/compass.cpp) (the six DB-lifecycle methods)
  and `appMode()` at [compass.cpp:1233-1256](src/app/compass.cpp#L1233-L1256). **The `*Internal()`
  variants already return `Result`** — the GUI shell is a thin wrapper that simply moves out.
- **`QApplication::activeWindow()`** used as a modal parent → `ApplicationUI`.
- **`QApplication::processEvents`** → `QCoreApplication::processEvents` (already the case in most
  sites; `pumpUntil` is already clean).
- **`TaskManager::getMainWindow()`** ([taskmanager.cpp:550](src/task/taskmanager.cpp#L550)) currently
  scans `QApplication::topLevelWidgets()` and `dynamic_cast`s. Delete it; use `compass().ui()`.
- **`AsyncTask::runAsyncDialog(bool, QWidget*)`** ([asynctask.h](src/task/asynctask.h)) — drop the
  `QWidget*` parameter, route through `ProgressReporter`; move `AsyncTaskDialog` to `compass_task_ui`.
- **`Utils::Files::IconProvider`** ([files.cpp](src/core/util/files.cpp)) uses
  `QApplication::palette()/style()/allWidgets()`. Make icon theming a UI-side concern injected at
  startup, so the core keeps only paths.
- **`src/task/result/report/section.h` and `sectioncontenttable.h`** include `<QWidget>` because
  `SectionContent::addContentUI(QVBoxLayout*, bool)` is a virtual on the report *data model*.
  Remove that virtual; replace with a `ReportContentUIBuilder` visitor living in `compass_task_ui`.
  This is what lets the report model become headless.

### Phase 3 — Cut the `_ui` targets

*Notes from the earlier plans:*
- *Each `_ui` sibling is its own shared library with its own export macro (plugin plan). Classes
  moving into it switch macros, mechanically by target.*
- *The generic widgets the dependency plan moved into core move to `compass_core_ui`.*
- *`compass_result` gets a `compass_result_ui` sibling for the report widgets. The part of
  `LatexVisitor` that visits the host view types follows once `View` moves to `compass_view_ui`.*

Per module, add the sibling target and move `Qt5::Widgets` to it. Assignment is by filename
(`*widget*`, `*dialog*`, `*combobox*`, `*itemview*`, `*window*`) plus this hand-picked list of
non-obviously-named headers that pull QtWidgets:

```
src/core/source/{dbdatasourceselection,dstypeselection}combobox.h   src/core/test/ui_test_find.h
src/core/rtcommand/rtcommand_shell.h                                 src/eval/requirement/group.h
src/db/dbcontent/{stringrepresentation,dbcontent}combobox.h          src/eval/standard/evaluationstandardcombobox.h
src/db/dbcontent/variable/variabledatatypecombobox.h                 src/import/asterix/asterix*combobox.h
src/eval/requirement/base/comparisontypecombobox.h                   src/filter/filtercondition*combobox.h
src/task/asynctask.h    src/task/import/asterix/asteriximporttask.h  src/task/result/report/section.h
```

Notable sub-splits:

- **`src/core/context/`** — `DBContextManager`, serializer and diff stay; the ~10 dialogs
  (`db_context_{create,copy,delete,rename,merge,conflict,select,edit}_dialog`) and the two tool
  widgets go to `compass_core_ui`. Give this one its own `src/core/context/ui/` subdirectory —
  it is large and self-contained enough to justify the move.
- **`src/db`** — promote to its own `compass_db` target (today it is textually merged into
  `compass_core` via [src/core/CMakeLists.txt:21](src/core/CMakeLists.txt#L21)). With shared
  libraries this requires core ↔ db to be acyclic; the dependency plan's layering check can measure
  it. `DBContentManager`
  stays; `DBContentManagerWidget`, `DBContentEditDialog`, `TargetListWidget`, `dbcontentitem{view,model}`
  go to `compass_db_ui`.
- **`src/view`** — `compass_view` keeps `ViewManager` (core), `ViewableDataConfig`, `viewpoint.h`,
  the `ViewPointGen*` model, the presets *model*, and `Grid2DLayerRenderer`. `View`, `ViewWidget`,
  `ViewDataWidget`, `ViewContainer`, and all QtCharts code go to `compass_view_ui`.
  `ViewManager::init(QTabWidget*)` becomes `init(ViewHost&)`.
- **QtCharts is already fully confined to `src/view`** (15 files) — it needs no separate treatment.

Adjust `viewsBusy()` ([rtcommand_runner_stash.cpp:32-61](src/core/rtcommand/rtcommand_runner_stash.cpp#L32-L61))
to query `ViewHost` — headless reports "never busy" and relies on `DBContentManager::waitUntilEngineIdle()`
instead.

### Phase 4 — Design in the offscreen-render seam

Do this *now*, while touching the report code anyway, so Phase 6 needs no second refactor.

Today `TaskResult::renderFigure()` ([taskresult.cpp:848-910](src/task/result/taskresult.cpp#L848-L910))
pushes the figure's viewable into the **global** `ViewManager` (destroying whatever the user was
looking at), waits for a DB reload, then grabs every open non-table view with `renderData()`. Output
depends on the user's current tab layout.

Replace with a registry in `compass_view`. After the dependency plan, `TaskResult` lives in
`compass_result`, which sits above `compass_view`, so it can reach this registry directly:

```cpp
class FigureRenderer {
    virtual bool canRender(const nlohmann::json& viewable) const = 0;
    virtual std::vector<std::pair<QImage, std::string>> render(
        const nlohmann::json& viewable, const RenderSettings&) = 0;
};
```

`TaskResult::renderFigure()` delegates to `FigureRendererRegistry`. `compass_view_ui` registers a
`LiveViewFigureRenderer` reproducing today's behaviour byte-for-byte, so **the client is unchanged**.
Phase 6 adds data-driven renderers to the same registry. Also add an explicit output size to
`RenderSettings` — `ViewDataWidget::renderData()` currently takes its resolution from `this->size()`,
i.e. the on-screen geometry.

### Phase 5 — Move `COMPASS`, split the app layer, add the binary

- *Done by the dependency plan:* `COMPASS` lives in `compass_core` as a thin access point;
  `COMPASS::mainWindow()` and the three `QWidget*` view facades
  ([compass.h:130-137](src/core/compass.h#L130-L137)) are gone, and their 9 call sites
  (`asteriximporttask.cpp` ×2, `mainwindow_commands_import.cpp` ×2, `viewpointswidget.cpp`,
  `analyzedatasourcetask.cpp`, `client.cpp`, `test_lab_phil.cpp` ×2) use narrow interfaces, which
  Phase 1's `ApplicationUI` replaces. What remains here is removing the GUI shell from the
  `COMPASS` lifecycle methods (Phases 1–2).
- Split the composition root: everything up to manager `init()` goes to `compass_app_core`, the
  `MainWindow` step to `compass_app_ui`.
- New **`compass_app_core`** (`src/app/core/`), extracted from `Client`: boost::program_options
  parsing (currently inside the `QApplication` subclass constructor,
  [client.cpp:178-318](src/app/client/client.cpp#L178-L318)), crash/signal handlers and the stall
  watchdog from [main.cpp](src/app/client/main.cpp), TBB setup, `COMPASS` construction, the
  CLI→rtcommand translation, and shutdown. All Qt-app-class agnostic. The `Client::notify()`
  breadcrumb override becomes a shared mixin usable from both app classes.
- Move rtcommand registration out of `MainWindow`'s constructor into
  `registerCoreCommands(COMPASS&)` in `compass_app_core`; only genuinely UI-bound commands stay in
  `compass_app_ui`. Split `rtcommand_helpers.cpp` object resolution behind a `CommandTargetResolver` —
  headless resolves named `QObject` managers via a registry; the widget-tree walk stays in the UI lib.
- New **`src/app/headless/main.cpp`** → `compass_headless`: sets `QT_QPA_PLATFORM=offscreen` unless
  already set, constructs a `QGuiApplication`, runs `AppRunner`, `exec()`s. Same CLI options, same
  rtcommand TCP port. **Ship the `libqoffscreen.so` QPA plugin in the AppImage** — verify this
  explicitly, it is easy to miss and fails only at runtime.
- Convert `compass_handler` to `QGuiApplication` — it has only 3 `QApplication::` uses, all inside
  `if (!has_rt_cmd_port)` branches already skipped in automated runs.

### Phase 6 — Headless rendering (deferred, no re-refactor needed)

New `compass_render` library registering `FigureRenderer` implementations that consume
`ViewPointGen*` features directly:

- Histogram / scatter / grid — `QPainter` on `QImage`, reusing `Grid2DLayerRenderer` as the model.
  Note QtCharts cannot be reused here (`QChart` is a `QGraphicsWidget`, hence QtWidgets), so these
  are new renderers rather than moved code.
- Table figures — already excluded from figure rendering today.
- **Geographic view** is the hard one. It is a `QOpenGLWidget` with `GraphicsWindowQt` bound to the
  widget's window; `grabFramebuffer()` reads the widget's FBO. A real headless path needs an
  `osg::GraphicsContext` with pbuffer/FBO traits (or EGL). There is a shelved proof of concept at
  [experimental_src/unit_tests/test_offscreen_map.cpp](experimental_src/unit_tests/test_offscreen_map.cpp),
  currently commented out of [its CMakeLists](experimental_src/unit_tests/CMakeLists.txt#L9) — revive it
  as the starting point. Interim fallback: run `compass_client` under Xvfb, which already works
  (that is what CI does). By then the geographic view is a plugin (plugin plan), so its offscreen
  `FigureRenderer` is registered by the geo plugin in its core phase, not by `compass_render`.

### Phase 7 — Tests

- Split `compass_tests` (`QGuiApplication`, headless libs only) from `compass_tests_ui`
  (`QApplication` + Xvfb, keeps `src/gui/unit_tests`). Currently
  [test_main.cpp](src/unit_tests/test_main.cpp) constructs a `QApplication` and CI runs it under
  `xvfb-run` ([Jenkinsfile:128](Jenkinsfile#L128)).
- Add the first test that constructs a `COMPASS` instance without a window — **no test does this today**,
  which is why the coupling went unnoticed.
- The headless `compass_tests` keeps the direct registration calls for the plugin components
  (plugin plan) but calls only their core part.
- CI guard: assert no `compass_*` non-`_ui` target links `Qt5::Widgets`, e.g. via
  `readelf -d build/lib/libcompass_<module>.so` on every non-`_ui` library.

---

## Verification

1. **Per phase:** `cmake --build build`, then
   `xvfb-run -a ./build/bin/compass_tests`. `compass_client` must still start and open a DB.
2. **Link-level proof (the real test):** from Phase 3 onward,
   `ldd build/bin/compass_headless | grep -i Qt5Widgets` must return nothing, and
   `readelf -d build/lib/libcompass_core.so | grep -i Qt5Widgets` must return nothing (likewise for
   every non-`_ui` library).
3. **Behavioural equivalence:** pick a representative existing CLI batch invocation —
   `compass_client --open_db <f> --load_data --evaluate --export_report <dir> --quit` — and run the
   same options through `compass_headless`. Diff the resulting report directory. Report *text*
   should match exactly from Phase 5; *figures* only from Phase 6.
4. **RTCommand parity:** with `--open_rt_cmd_port`, drive both binaries from the existing Python
   test framework over `localhost:27960` and compare JSON responses. Commands that legitimately
   require a window should return a clear "not available headless" error, not hang.
5. **No-display smoke test:** run `compass_headless` with `DISPLAY` unset and no Xvfb. This is the
   acceptance criterion for the whole effort.

---

## Risks and effort

- **This is a multi-month effort**, not a sprint. ~375k lines; ~330 files across the compute modules
  touch QtWidgets. Phases 0-1 are days; Phase 2 is the bulk; Phase 6 (geographic view offscreen) is
  genuinely open-ended research.
- **Branch discipline is the main risk.** The `_ui` split is done by CMake target assignment
  precisely to avoid file moves, which are what make a long-lived branch unrebasable. Resist the
  urge to also tidy directory layout — the one exception is `src/core/context/ui/`.
- **`View` owns its data.** `BufferData data_` lives inside `ViewDataWidget`
  ([viewdatawidget.h:221](src/view/viewbase/viewdatawidget.h#L221)), a `QWidget`, and `View`'s
  constructor unconditionally `new QWidget()`s. This plan deliberately does **not** untangle that:
  views stay UI-only, and headless rendering goes through the already-pure-data
  `ViewableDataConfig`/`ViewPointGen*` path instead. Attempting a full `View`/`ViewDataModel`
  separation would roughly double the scope for little headless benefit.
- **`QT_QPA_PLATFORM=offscreen` is a tempting shortcut** — it would make today's widget code run
  without a display and deliver "headless" in an afternoon. It is worth doing as an interim
  capability, but it is not the separation asked for: the process still needs the QPA plugin, still
  builds the whole widget tree, and still cannot be linked as a compute library. Treat it as the
  Phase 6 fallback for the geographic view, not as the goal.
- **Stale documentation:** `CLAUDE.md` describes a `compass` library target and a `src/db/` layout
  that no longer match the tree. Update it in Phase 0 so the refactor does not inherit the confusion.
- **Effort after the other two plans:** the `COMPASS` move and the `mainWindow()` removal of Phase 5
  are done by the dependency plan. Everything else is unchanged.
- **Plugins.** Every `_ui` cut that moves a class a plugin derives from or connects to changes that
  class's export macro and the plugin's link set (through `compass_plugin_sdk`). Keep the plugin
  plan's export rule applied to the new `_ui` libraries.
