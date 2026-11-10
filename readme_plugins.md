# Splitting COMPASS into host + loadable plugins

*Design document / migration plan. Status: proposed, not started. First written 2026-08-28, revised 2026-10-23 against the then-current code of both repositories (line numbers refer to that state). Revised 2026-11-03: plugins are always present, so plugin presence is asserted once at startup instead of being handled at runtime. Revised again 2026-11-03: builds on the dependency plan, which runs first. Revised 2026-11-04 after the dependency plan was implemented: paths and line numbers refer to that state, starting point updated, findings of a symbol-level link check added (Workstream A, "Further host ↔ experimental edges").*

## Context

An external constraint requires that some COMPASS functionality be delivered as separately loadable plugins rather than as part of the monolithic client binary. The components in scope are **`ProbIMMReconstructor`** (with all of advanced reconstruction) and, later, the **Geographic View**.

The delivery model is unchanged: **all C++, all compiled from one tree at every release, host and fitting plugins always shipped together in the same package, from one hand.** Nothing is shipped or sold separately. There is no third-party plugin ecosystem, no ABI stability commitment, no version negotiation, and no API deprecation policy. That removes the hard 80% of what usually makes a plugin architecture expensive.

**All plugins are always present.** A missing or broken plugin is a packaging error, not a supported runtime state. The host checks once at startup that every expected plugin loaded and asserts otherwise. No other code checks for plugin presence, and there is no degraded mode.

### Execution order (decided 2026-11-03)

1. **Dependency plan** ([readme_dependencies.md](readme_dependencies.md)): now. It removes the cycles between the `src/` module libraries.
2. **This plan**: next.
3. **Compute core plan** ([readme_compute_core.md](readme_compute_core.md)): in the near future.

**Starting point.** After the dependency plan (implemented 2026-11-04):
- the `src/` module libraries are still static archives, but their dependencies are acyclic, in the includes and in the CMake link graph;
- the libraries' include paths only reach their own directory and the libraries below them, so an upward include does not compile (`scripts/check_layering.py`, run manually, checks this and the CMake files);
- `COMPASS` lives in `compass_core` as access point; the composition root `CompassRoot` in `compass_app` owns and constructs the managers in explicit startup steps;
- a symbol-level check (2026-11-04) confirmed that every host library's declared link dependencies cover all symbols it uses from other host libraries;
- the remaining cycles are those between the host and `experimental_src`: the host sites that name ProbIMM and `GeographicView` types, plus the edges listed under Workstream A, "Further host ↔ experimental edges".

Two further plans shape the design:

- **Native Windows build** ([readme_windows_port.md](readme_windows_port.md)). The plugin architecture must be identical on both platforms, which rules out the Linux-only `-rdynamic` shortcut (explained under [Target architecture](#target-architecture), "Why not the Linux-only shortcut").
- **Compute core / UI separation** follows this plan. This plan must not block it: the export, SDK and registration design below is chosen so that cutting modules into core and `_ui` targets later only touches CMake and the export annotations.

**Headline finding:** the library loading is the easy part. The real work is

1. the host sites that name plugin types concretely (~12 for ProbIMM, ~13 for the geo view). They must go through registries **before** the libraries can become shared, since each of them closes a cycle between a host library and an experimental one;
2. **one shared library per module**, which Windows plugin DLLs need. It is possible only because the dependency plan removed the cycles between the `src/` libraries;
3. a correct **export set**: every host class or data symbol a plugin derives from, connects to or reads (considerably more than "the base classes");
4. moving the three ProbIMM-specific tests out of `compass_test_lab`, which are a hidden host→ProbIMM edge.

## Phases

| Phase | Scope | Goal |
|---|---|---|
| **1: ProbIMM proof of concept** | Workstreams A–F | Registries for all host sites (ProbIMM and geo view), shared libraries, exports, loading, startup presence check, packaging and tests, proven on ProbIMM. The Geographic View stays a regular host library, registered statically. |
| **2: Geographic View** (follow-up) | Workstream G | Turn the geo view into a plugin on the infrastructure from phase 1. Started after phase 1 has been evaluated. |

**Why the geo view's host sites are in phase 1.** With shared libraries, `compass_geo_view` (which links `compass_view`, `compass_result` and `compass_task`) cannot be named by those libraries. Its host sites therefore have to go through registries in phase 1, even though the geo view only becomes a plugin in phase 2.

Phase 1 must not make phase 2 harder. The architecture, the export rule and the registry design below already account for the geo view, and phase 2 only adds a lifecycle registry and the module itself.

## Scoping decisions

- **ProbIMM plugin contains all of `compass_reconst_adv`** (`complex/`, `kalman/`, `jpda/`) **plus the three test lab tests that use its types** (Workstream D). This requires a Kalman-interface registry. Keeping `kalman/` in the host was rejected.
- **Plugin presence is asserted once at startup (decided 2026-11-03).** All plugins are always shipped, so a missing plugin aborts the client at startup with a clear message (Workstream E). There is no runtime presence query, no fallback and no UI gating for absent plugins.
- **Licence checks stay as they are.** Plugin presence no longer varies at runtime, so the licence is the only runtime axis, and the existing licence checks remain unchanged.
- **Plugins are CMake `MODULE` libraries**, loaded via `QLibrary`, with no import library on Windows. Nothing links against a plugin. The host only resolves the entry symbol `compass_plugin_create` at runtime, and everything else goes through the returned `Plugin` object and the registries it fills. CMake rejects `target_link_libraries()` on a `MODULE` target, which enforces that the host libraries, the client and the tests build without any plugin. The `.lib`/`.exp` that MSVC may still emit for the exported entry symbol goes unused and is not shipped.
- **Stay in the host:** `compass_osg_utils`, `compass_docx`, `compass_rtcmd_ext`, `compass_test_lab` (minus the three ProbIMM tests), the MLAT/ADS-B/SMR inspectors under `experimental_src/analysis/`, and in phase 1 also `compass_geo_view`. osgEarth stays a host dependency; host code uses it directly ([accuracy.cpp:24](src/util/accuracy.cpp#L24), [measurement.cpp:27](src/reconstruction/measurement.cpp#L27), [client.cpp:153](src/app/client/client.cpp#L153)).

## Target architecture

After phase 1:

```
libcompass_util.so … libcompass_app.so       one shared library per module,
libcompass_osg_utils, _docx, _rtcmd_ext,     incl. the host-side experimental libraries
_test_lab, _geo_view                         (compass_geo_view in phase 1 only)
      ▲              ▲
      │              │  (via INTERFACE target compass_plugin_sdk)
compass_client    compass_plugin_probimm.so / .dll   (incl. 3 test lab tests)
(main.cpp)
```

On Windows the host libraries are `compass_*.dll`. After phase 2, `compass_geo_view` leaves the host and becomes `compass_plugin_geoview.so / .dll` next to the ProbIMM plugin.

**Why one shared library per module.** The dependency plan removed the cycles between the `src/` libraries, and Workstream A removes those between host and experimental libraries. Each library can then be shared as it is:
- the linker enforces the layering (`--no-undefined`);
- a plugin links only the libraries it uses;
- the 65535-ordinal `.def` cap applies per library;
- the compute core plan's core/`_ui` split maps onto the same targets.

Merging everything into one library, the earlier design of this plan, was only needed while the cycles existed.

A plugin links the host libraries through `compass_plugin_sdk`, an INTERFACE target listing the libraries a plugin may link. ProbIMM needs `compass_core`, `compass_reconstruction`, `compass_task`, `compass_gui` and `compass_view` (plus osgEarth). The geo view additionally needs `compass_result` and `compass_osg_utils`.

**Why not the Linux-only shortcut.** On Linux, plugins would be possible without any shared host library:

- Keep linking all `compass_*` archives statically into `compass_client`, and link it with `-rdynamic`. That is `ENABLE_EXPORTS`, which [src/app/client/CMakeLists.txt:34](src/app/client/CMakeLists.txt#L34) already sets for crash backtraces. All of the executable's symbols then land in its dynamic symbol table.
- Build the plugin as a shared object that references host symbols (`ReconstructorBase`, `logger::debug_enabled`, Qt metaobjects, …) without linking anything that defines them. ELF allows undefined symbols in shared objects by default.
- When the plugin is loaded, the dynamic loader resolves these undefined symbols against the executable's exported symbols. Functions, vtables, static data and globals all resolve by name, with no annotations, and there is only one copy of every static, since the executable is the only definition.

Why this does not carry over to Windows:

- **No undefined symbols in a DLL.** A PE DLL must resolve every imported symbol at link time against an import library. Each import is bound to a fixed module name. There is no lookup by name across "whatever is already loaded", as with ELF.
- **Importing from the `.exe` binds the plugin to one executable.** An executable can export symbols and produce an import library, but a plugin linked against it imports from `compass_client.exe` by name. It can then only be loaded into that one process. `compass_tests` and a future headless binary (compute core plan) could not load it, and Windows does not support loading a second `.exe` as a dependency.
- **The export work remains anyway.** Even when importing from the executable, MSVC needs `dllexport` / `dllimport` for data symbols (B.5). The shortcut would save the shared libraries on Linux only, and Windows would need a different linking design later.

So both platforms use the same design: the host code lives in shared libraries, which the client, the tests and the plugins all link against.

**Forward compatibility with the core/UI split.** The compute core plan cuts each module into a core library and a `_ui` sibling. Three choices keep that cheap:

- **One export macro per library**, generated per target (B.5). On MSVC each DLL needs its own dllexport/dllimport macro. When the compute core plan moves a class into a `_ui` target, it switches to that library's macro: mechanical, by target.
- **Plugins link the INTERFACE target `compass_plugin_sdk`**, not the host libraries directly. Later it resolves to the core and `_ui` libraries without touching plugin CMake files.
- **Two-phase plugin registration** (see Extension points). A headless binary calls only the core phase.

## Extension points

`PluginManager` lives in `compass_core`. The composition root (`CompassRoot`) creates it right after `COMPASS`, i.e. **before** the config tree is instantiated, since [task_reconstructor.json](conf/default/task_reconstructor.json) names `ProbIMMReconstructor0` (and [views.json:16](conf/default/views.json#L16) names `GeographicView0`).

Plugin entry point, one exported C symbol per module:

```cpp
extern "C" compass::Plugin* compass_plugin_create();   // returns a static instance

class Plugin
{
public:
    virtual ~Plugin() = default;

    virtual std::string name() const = 0;
    virtual void registerCore(PluginRegistry& registry) = 0;   // no widgets
    virtual void registerUI(PluginUIRegistry& registry) = 0;   // widgets, views, menus
};
```

Each component provides its registration as two functions with exactly this split (Workstream A). As long as a component is a regular host library, the composition root calls them directly; once it is a plugin, its `Plugin` object calls them. `PluginRegistry` / `PluginUIRegistry` are thin aggregates handing out the registries, which live in the libraries of their consumers.

Registries:

| Registry | Reg. phase | Library | Replaces | Phase |
|---|---|---|---|---|
| `ReconstructorFactory` | core | `compass_task` | `new ProbIMMReconstructor` + `new ComplexAccuracyEstimator` at [reconstructortask.cpp:176-187](src/task/reconstructor/reconstructortask.cpp#L176) | 1 |
| `KalmanInterfaceFactory` | core | `compass_reconstruction` | experimental branches of `KalmanEstimator::createInterface()` at [kalman_estimator.cpp:158-191](src/reconstruction/kalman_estimator.cpp#L158) | 1 |
| `ReconstructorWidgetFactory` | UI | `compass_task` | concrete `probIMMReconstructor()->widget()` at [reconstructortaskdialog.cpp:115](src/task/reconstructor/reconstructortaskdialog.cpp#L115) | 1 |
| `ViewFactory` | UI | `compass_view` | `new GeographicView` at [viewcontainer.cpp:296-305](src/view/viewcontainer.cpp#L296), class list at [viewmanager.cpp:152](src/view/viewmanager.cpp#L152) | 1 |
| RT command registration | core | `compass_core` (`RTCommandRegistry`) | `init_geographic_view_commands()` at [viewmanager.cpp:106](src/view/viewmanager.cpp#L106) | 1 |
| `GeoImageSink` | UI | `compass_view` | `viewsOfType<GeographicView>()` in the grid view export | 1 |
| `TestLabRegistry` | UI | `compass_test_lab` | plugin-specific entries in the host test lab menu ([test_lab.cpp:67](experimental_src/test/test_lab.cpp#L67)) | 1 |
| `LifecycleHooks` | core | `compass_core` | `geo_view::shutdown()` at [compassroot.cpp:415](src/app/compassroot.cpp#L415) | 2 |

`Configurable::generateSubConfigurable()` is already a string-keyed factory, so the registries slot in without a new creation mechanism.

Widget creation for ProbIMM goes through the UI phase rather than a virtual on the reconstructor, so a future headless binary can load the ProbIMM plugin's core part without widgets. The plugin itself still links the UI libraries until it is split internally; that is a compute core plan concern.

### Startup presence check

`PluginManager` knows the set of **expected plugins** from the build (Workstream E) and loads exactly those. Every step of loading a plugin is checked with `traced_assert_msg`, and so is the registration result: each expected registry entry must exist after `registerCore()` / `registerUI()`. After this check, host code may rely on every registry entry being present. Registry lookups for expected entries assert instead of returning an "unavailable" result.

`traced_assert` does not depend on `NDEBUG` and is active in release builds, so the check also protects the AppImage.

---

## Phase 1: ProbIMM proof of concept

Order: A → B → C → D → E → F. A and B are functional no-ops: after them, ProbIMM and the geo view are still regular host libraries, registered statically by the composition root.

### A: Host sites → registries (~2.5 wk)

Removes the host ↔ experimental cycles. The experimental libraries stay regular libraries here. Each component gets a `registerCore(...)` / `registerUI(...)` pair, which the composition root calls in its startup sequence, after creating `COMPASS` and before instantiating the config tree.

**ProbIMM:**

- **Construction and access.** `ReconstructorTask` creates `ComplexAccuracyEstimator` and `ProbIMMReconstructor` through `ReconstructorFactory` (one call creating both). The concrete `probimm_reconstructor_` member and `probIMMReconstructor()` ([reconstructortask.cpp:392,407,1577](src/task/reconstructor/reconstructortask.cpp#L392)) become `ReconstructorBase*`.
- **Widget.** `ReconstructorBase` has no `widget()`; both derived classes declare a concrete-typed one ([simplereconstructor.h:63](src/task/reconstructor/simplereconstructor.h#L63), [probimmreconstructor.h:185](experimental_src/reconstruction/complex/probimmreconstructor.h#L185)). Hence `ReconstructorWidgetFactory` in the UI phase rather than a virtual.
- **Selection logic.** `checkReconstructor()` ([reconstructortask.cpp:197-204](src/task/reconstructor/reconstructortask.cpp#L197)) forces ProbIMM whenever the licence is enabled. This stays licence-only, as do the constructor fallback (`:98-104`), the setter assert (`:373-378`) and the selector entry ([reconstructortaskdialog.cpp:80](src/task/reconstructor/reconstructortaskdialog.cpp#L80)). They only stop naming the concrete type. These sites use `ProbImmReconstructorName` outside the `USE_EXPERIMENTAL_SOURCE` guard, while the header declares it inside one ([reconstructortask.h:139-143](src/task/reconstructor/reconstructortask.h#L139)); the name constant moves out of the guard.
- **Kalman interfaces.** `createInterface()` is the only production path that constructs experimental filters (`KalmanChain`, `KalmanOnlineTracker`, `ReconstructorBase`, `ReferenceCalculator` all go through `KalmanEstimator::init()`, and cloning is virtual). A `KalmanType → factory` registry closes it, and a lookup of an unregistered type is a programming error that asserts. In `referencecalculator.cpp`:
  - the compile-time `#if USE_EXPERIMENTAL_SOURCE == false` assert at [:1027](src/task/reconstructor/referencecalculator.cpp#L1027) becomes a registry lookup;
  - the hardcoded IMM→AM switch at [:1070](src/task/reconstructor/referencecalculator.cpp#L1070) means ProbIMM must register AM as well as IMM.

  The settings default `kalman_type_final = IMMKalman2D` ([referencecalculatorsettings.h:110](src/task/reconstructor/referencecalculatorsettings.h#L110)) stays.
- **Analysis widget: no change.** The `probimm_reconst_` bool in [reconstructortaskanalysiswidget.cpp](src/task/reconstructor/reconstructortaskanalysiswidget.cpp) is passed in by each reconstructor's own widget, so it is already plugin-neutral.

**Geographic View:**

- **View creation.** `new GeographicView` in `ViewContainer` and the class list in `ViewManager` go through `ViewFactory`. In `ViewContainer`, the "compiled w/o experimental source" branch stays a compile-time case.
- **RT commands.** `init_geographic_view_commands()` is called from the geo view's core registration instead of from `ViewManager`.
- **Static `GeographicView::instant_display_`** ([reportexporter.cpp:205-214](src/result/report/reportexporter.cpp#L205), [viewpointsreportgenerator.cpp:170,345](src/view/points/report/viewpointsreportgenerator.cpp#L170)) becomes a flag on the `View` base class.
- **Grid view export.** "Export to GeographicView" ([gridviewconfigwidget.cpp:233,536-663](src/view/gridview/gridviewconfigwidget.cpp#L536)) uses `viewsOfType<GeographicView>()` and the non-virtual `addInternalAnnotation()` at `:659`. It moves to an abstract `GeoImageSink` in `compass_view` with `addInternalAnnotation()`, implemented by the geo view. `viewsOfType<GeoImageSink>()` ([viewmanager.h:250](src/view/viewmanager.h#L250)) works unchanged.
- **LaTeX export.** The normal screenshot already goes through generic `View` functions (`showInTabWidget()`, `hasScreenshotContent()`, `prepareForRender()`, `renderData()`, [view.h:120-144](src/view/viewbase/view.h#L120)). Only the **overview screenshot** is geo-specific ([latexvisitor.cpp:350-367](src/view/points/report/latexvisitor.cpp#L350)): `zoomToDataSlot()`, `addDataMarker()`, `osgViewerWidget()->grabFramebuffer()`, `removeDataMarker()`. It becomes one `View` virtual returning the finished image, implemented by the geo view with exactly those four calls:

  ```cpp
  virtual QImage renderOverview() { return {}; }   // empty = view has no overview
  ```

  `visit(GeographicView*)` ([latexvisitor.h:60](src/view/points/report/latexvisitor.h#L60), [latexvisitor.cpp:292-371](src/view/points/report/latexvisitor.cpp#L292)) then folds into a generic visit of `View`. (`LatexVisitor` and the view points report generator are in `compass_view` since the dependency plan, the LaTeX document classes in `compass_core`.)
- **`ViewerWidget.{h,cpp}` and `GraphicsWindowQt.{h,cpp}`** move from `experimental_src/view/geographicview/` into `compass_osg_utils`. They are a generic Qt↔OSG windowing integration, used by the geo view, the host test lab ([test_lab_phil.cpp:4024](experimental_src/test/test_lab_phil.cpp#L4024), which references no `GeographicView` symbol, only this class) and [latexvisitor.cpp:44](src/view/points/report/latexvisitor.cpp#L44). Details:
  - `compass_osg_utils` stops being "no COMPASS deps": `ViewerWidget.cpp` uses `logger.h` and `Utils::String`, so it links `compass_util`. The include of `compass.h` appears unused; drop it.
  - Replace `OSGEARTHQT_EXPORT` (from the geo view's `Common.h`, which would expand to `dllimport` on MSVC) with `compass_osg_utils`' own export macro (B.5).
  - The module statics (`s_retired_contexts`, `HeartBeat`, `QtWindowingSystem`, `REGISTER_WINDOWINGSYSTEMINTERFACE`) move with them into the host and stay single-instance.
  - `compass_test_lab`'s link to `compass_geo_view` then goes.

**Both:**

- **Tests.** Tests that go through the registries (`test_kalman_imm`, `test_target_fixtures` via `KalmanEstimator::init(IMMKalman2D)`) need the registration. The test main calls the same registration functions as the root.
- **CMake.** Drop the include directories into `experimental_src` from host CMake files once no host file outside the root includes experimental headers:
  - [src/reconstruction/CMakeLists.txt:40-51](src/reconstruction/CMakeLists.txt#L40), including the link-only dependency on `compass_reconst_adv` that the dependency plan declared there for `kalman_estimator.cpp`;
  - the reconstruction and geo-view parts of the experimental block in [src/task/CMakeLists.txt:37-58](src/task/CMakeLists.txt#L37);
  - the geo-view include blocks in [src/view/CMakeLists.txt:38-53](src/view/CMakeLists.txt#L38), [src/result/CMakeLists.txt:29-46](src/result/CMakeLists.txt#L29) (the geo part only; `experimental_src/docx` stays), [src/app/CMakeLists.txt:46-55](src/app/CMakeLists.txt#L46), [src/app/client/CMakeLists.txt:52-60](src/app/client/CMakeLists.txt#L52);
  - the PUBLIC include directory `experimental_src/test` of `compass_core` ([src/core/CMakeLists.txt:37-48](src/core/CMakeLists.txt#L37)), see `test_target` below.

**Further host ↔ experimental edges** (found by a symbol-level check on 2026-11-04: for every library, the symbols it uses from other COMPASS libraries against its declared link dependencies). Besides the ProbIMM and geo-view sites above, which the check confirmed, there are:

- **`compass_task` ↔ `compass_test_lab`.** `ReferenceCalculator` has a `TestTarget` member ([referencecalculator.h:23,48](src/task/reconstructor/referencecalculator.h#L23)); `test_target.{h,cpp}` is compiled into `compass_test_lab`, which links `compass_task`. The header is only reachable because `compass_core` publishes `experimental_src/test` as include directory. Open decision: `test_target.{h,cpp}` (depends only on util, Eigen and json) becomes a small experimental library below `compass_task`, or moves into a host library (it is closed source today, so a move into `src/` publishes it).
- **`compass_app` ↔ `compass_rtcmd_ext`.** `extra_commands.cpp` calls `MainWindow::createInMemoryDB()` and `MainWindow::createDBFromMemory()` ([extra_commands.cpp:113-154](experimental_src/rtcommand/extra_commands.cpp#L113)), while `compass_app` calls `extra::init_extra_commands()`. Both `MainWindow` functions are `COMPASS::createInMemDBFile()` / `createNewDBFileFromMemory()` plus `updateBottomWidget()` and `updateMenus()` ([mainwindow.cpp:795-813](src/app/client/mainwindow.cpp#L795)). Open decision: the commands call `COMPASS` and `IMainWindow` (which has `updateMenus()`; `updateBottomWidget()` would be added), so `compass_rtcmd_ext` only needs `compass_core`.
- **Undeclared downward links** (no cycle, but must be declared for shared libraries, B.1): `compass_result` uses 24 symbols of `compass_docx` (DOCX export) without linking it; `compass_app` uses `compass_geo_view` (`geo_view::shutdown()`, `clearHiddenLayers()`), `compass_rtcmd_ext` and `compass_test_lab`, which only `compass_client` links ([src/app/CMakeLists.txt:46-55](src/app/CMakeLists.txt#L46) explains this with duplicate AUTOMOC symbols).
- **Side effect:** the `USE_EXPERIMENTAL_SOURCE` guards for ProbIMM and the geo view shrink to the root's registration calls. The build without `experimental_src` does not compile today; check whether it compiles again, but it is not a goal of this plan.

### B: Shared libraries + exports (~2 wk)

Everything here is testable on the current Linux build before any plugin exists.

1. **One shared library per module.** Every `compass_*` library in `src/` and the host-side libraries in [experimental_src/CMakeLists.txt](experimental_src/CMakeLists.txt) become `SHARED`, with `POSITION_INDEPENDENT_CODE ON` and `-Wl,--no-undefined` on Linux.
   - Each library links its real dependencies explicitly. Between the host libraries in `src/` this is already the case: the dependency plan made the link graph follow the layering, and the symbol check of 2026-11-04 found no missing host-to-host link. What remains are the undeclared links to experimental libraries listed in Workstream A (`compass_result` → `compass_docx`, `compass_app` → `compass_geo_view` / `compass_rtcmd_ext` / `compass_test_lab`).
   - Add the `compass_plugin_sdk` INTERFACE target.
2. **Client and tests.** `compass_client` is already only `main.cpp` + `cxa_throw_hook.cpp`. Link it against the host libraries; the repeated-archive link lines here and in [src/unit_tests/CMakeLists.txt](src/unit_tests/CMakeLists.txt) were already removed by the dependency plan. Check that crash backtraces still resolve names once `ENABLE_EXPORTS` is no longer what exposes the symbols (they now live in the libraries' dynamic symbol tables).
3. **Header-inline singletons.** Move `Logger::getInstance()` ([logger.h:189](src/util/logger.h#L189)) and `RTCommandRegistry::instance()` ([rtcommand_registry.h:45](src/core/rtcommand/rtcommand_registry.h#L45)) into `.cpp` files. Function-local statics in header-defined functions are merged across modules on Linux but **duplicated per DLL on Windows**. ProbIMM uses neither directly (the logging macros go through the out-of-line `logger::rootCategory()`), so this is cheap prevention; it becomes essential in phase 2, when the geo plugin registers RT commands. The real logging hazard is the data symbol in item 5.
4. **Performance check.** Shared libraries with `-fPIC` and default visibility can cost runtime in hot code (reconstruction, Kalman, evaluation). Build with `-fno-semantic-interposition` and compare a reconstruction run's wall time against the static build.
5. **Symbol export.** Generate one export macro per library via `generate_export_header` (`COMPASS_UTIL_API`, `COMPASS_CORE_API`, `COMPASS_VIEW_API`, …; empty on Linux) and enable `WINDOWS_EXPORT_ALL_SYMBOLS` on every host library.

   Auto-export covers **functions**. MSVC additionally requires `dllimport` on the consumer side for every **data symbol** a plugin references, otherwise it is an unresolved external at link time. Data symbols arise from:
   - **Deriving** from a host class: its `staticMetaObject` (moc superclass link, `qobject_cast`) and potentially its vtable.
   - **`connect()` with member function pointers** to a host signal: it references the sender's `staticMetaObject`. This applies to classes the plugin merely uses.
   - **`static const` members and globals** that are odr-used.

   The **rule**: annotate class-level, with the macro of the class's own library, every `Q_OBJECT` class and every class with static data members in the host, plus `logger::debug_enabled`. This is a mechanical, announced bulk edit, safer than chasing a hand-picked list, and it already covers phase 2. The verified minimum it must cover:

   | Kind | ProbIMM (phase 1) | Geo view (phase 2) |
   |---|---|---|
   | Bases | `ReconstructorBase`, `ReconstructorBaseSettings`, `ReconstructorAssociatorBase`, `AccuracyEstimatorBase`, `ReconstructorMainWidget`, `ReferenceCalculatorWidget`, `KalmanInterface`, `KalmanFilter`, `KalmanFilterLinear` | `View`, `ViewWidget`, `ViewDataWidget`, `TabStyleViewConfigWidget` (→ `ViewConfigWidget`, `ViewComponent`), `ViewLayerTreeModel` (→ `LayerTreeModel`), `LayerTreeItem`, `DBContentItemModel`, `DBContentItemProvider`, `DBContentLeafPayload`, `Configurable`, `ui_test::UITestable`, `rtcommand::RTCommand`, `GeoImageSink` |
   | Signals connected to | `context::DBContextManager` | `COMPASS`, `ViewManager`, `DBContentManager`, `context::DBContextManager`, `ViewAsyncProcessor`, `dbContent::LabelGenerator` |
   | Static data | `ReconstructorTarget::global_stats_` (written by the plugin), `TargetBase::emitter_specs_`, `KalmanEstimator::HighVar`, `DataSourceBase::*Key`, `targetReport::AccuracyTables`, `ViewPointKeys::VP_*` (in `compass_core` since the dependency plan) | `ViewPointKeys::VP_*`, `ViewPointGen*` field names, `DataSourceBase::*Key`, `DBContentItemProvider::DefaultGrouping`, `Configuration::ParameterSection`, `LabelGenerator` constants |
   | Global | `logger::debug_enabled` ([logger.h:47](src/util/logger.h#L47), read by every `logdbg`), `APP_FILENAME` ([global.cpp](src/util/global.cpp), defined in `compass_util` since the dependency plan) | same |

   C++17 `inline const` namespace variables (`dbcontent_vars::meta_var_*` in [idbvariableresolver.h:126](src/core/base/idbvariableresolver.h#L126)) get one copy per DLL on Windows. That is harmless as long as nothing compares their addresses; verify this.

   Measure the generated export count per library: `.def` files cap at 65535 ordinals per DLL. With one library per module this is unlikely to be hit; check the largest ones (`compass_core`, `compass_task`, `compass_eval`).

   Rejected alternatives: a **narrow C-style ABI** with host-side adapters (forwarding shims for every virtual, for ABI stability nobody needs) and **MinGW** (auto-imports data, but contradicts the MSVC/vcpkg decision of the port plan).

### C: Plugin infrastructure (~3 d)

- `src/core/plugin/plugin.h`: `Plugin` interface, `COMPASS_PLUGIN_API_VERSION` (a plain build-consistency check, no negotiation), entry-symbol declaration.
- `src/core/plugin/pluginregistry.{h,cpp}`: the `PluginRegistry` / `PluginUIRegistry` aggregates over the registries from Workstream A.
- `src/core/plugin/pluginmanager.{h,cpp}`: load the expected plugins, startup presence check (Workstream E), log a startup summary. Created by the composition root right after `COMPASS`.
- **Loading via `QLibrary`**, not hand-written `dlopen`/`LoadLibrary` wrappers. Qt is already a dependency, so this keeps the port plan's "no `dlopen`" property and adds no platform seam.
  - Use `ResolveAllSymbolsHint` (`RTLD_NOW`) so a missing symbol fails at startup.
  - Keep the default local symbol scope: plugins only need the host libraries, not each other.
  - **Never unload.** Qt metaobjects and osgEarth statics do not survive it.
- New directory `plugins/<name>/plugin.cpp` per plugin, holding only the `Plugin` subclass and the entry symbol. Sources stay where they are, and `experimental_src` remains its own repository.

### D: ProbIMM plugin (~0.5 wk)

`compass_reconst_adv` becomes the `MODULE` library `compass_plugin_probimm`, together with the three test lab tests that use its types. `experimental_src/util/` contributes only an include directory; `extra_resources.{h,cpp}` is compiled into `compass_core` ([src/core/CMakeLists.txt:37-48](src/core/CMakeLists.txt#L37)) for `License::verify()` and stays there.

- **Entry point.** `plugins/probimm/plugin.cpp`: a `Plugin` subclass whose `registerCore()` / `registerUI()` call the registration functions from Workstream A. The root's direct registration call for ProbIMM goes.
- **`friend class KalmanFilterIMM`** at [kalman_filter.h:181](src/reconstruction/kalman_filter.h#L181) survives the split (friendship is compile-time); the class-level export on `KalmanFilter` makes the befriended members reachable.
- **JPDA moves cleanly.** The JPDA references in the host ([reconstructortask.h:77,91](src/task/reconstructor/reconstructortask.h#L77), [reconstructortarget.h](src/task/reconstructor/reconstructortarget.h), [reconstructorbase.cpp:1811](src/task/reconstructor/reconstructorbase.cpp#L1811), [datasourceeditwidget.cpp:227](src/core/source/datasourceeditwidget.cpp#L227)) are debug flags, statistics counters and labels. The plugin fills the counters through `ReconstructorTarget::global_stats_`, which is why `ReconstructorTarget` is exported.
- **Test lab.** `compass_test_lab` stays in the host. Three of its 42 tests construct plugin types directly, and the plugin being loaded at runtime does not help there: code naming those types must be linked against them, and on Windows a host DLL must resolve every symbol at link time. These three move into the plugin:
  - Unscented Kalman Test (`unscentedKalmanTest`, [test_lab_phil.cpp:1042](experimental_src/test/test_lab_phil.cpp#L1042): `KalmanFilterUnscented`, `MerweScaledSigmaPoints`)
  - IMM Test (`immTest`, [:1965](experimental_src/test/test_lab_phil.cpp#L1965), runs `KalmanTest` from [kalman_test.cpp](experimental_src/test/kalman_test.cpp): AM/UM/ZM/IMM filters, `dynamic_cast<KalmanFilterIMM*>`)
  - JPDA Test (`jpdaTest`, [:2945](experimental_src/test/test_lab_phil.cpp#L2945): `JPDAFull`, `JPDASettings`, `JPDATentativeTarget`, `JPDAResult`)

  The plugin contributes them to the host's test lab menu through `TestLabRegistry`. `test_target.{h,cpp}` is plugin-free and stays in the host, where the plugin's tests can use it (where exactly is the open decision on `compass_task` ↔ `compass_test_lab` in Workstream A). The remaining Kalman/JPDA includes in `test_lab_phil.cpp` go, and so does `compass_test_lab`'s link to `compass_reconst_adv`.
- **Link dependencies.** The plugin calls `osgEarth::GeoMath` (jpda, associator, accuracy estimators) and uses `compass_view` classes (`Grid2DLayerRenderer`, `ColorMap`, `ScatterSeries`). On Windows the plugin DLL must link osgEarth itself; declare it.

### E: Startup presence check (~1 d)

All plugins are always present, so presence is checked exactly once, in `PluginManager` at startup, before the config tree is instantiated. Nothing else in the host checks for it.

- **Expected plugins come from the build.** CMake generates the list of plugin targets built in this configuration into a header (e.g. `COMPASS_EXPECTED_PLUGINS`), together with the registry entries each plugin must provide (ProbIMM: `ProbIMMReconstructor`, `ComplexAccuracyEstimator`, the IMM and AM Kalman types, the ProbIMM widget factory, the three test lab entries). `PluginManager` loads exactly these files by name and does not scan for other files.
- **Asserted steps**, each with `traced_assert_msg` and a message naming the plugin and the plugin directory. Before the assert, the details go to `logerr`, e.g. `QLibrary::errorString()`:
  1. the plugin file exists;
  2. `QLibrary::load()` succeeds;
  3. the entry symbol `compass_plugin_create` resolves;
  4. `COMPASS_PLUGIN_API_VERSION` and `COMPASS_BUILD_ID` match (Workstream F);
  5. `name()` matches the expected name;
  6. after `registerCore()` (and `registerUI()` in the UI phase), every expected registry entry exists.
- **Consequences for the rest of the host:**
  - no presence query, no `Availability` enum, no UI gating, no fallback paths;
  - `createSubConfigurables()` ([configurable.cpp:298](src/config/configurable.cpp#L298)) needs no "known class, provider unavailable" outcome: `ProbIMMReconstructor0` in `task_reconstructor.json` always finds its class;
  - config preservation for absent plugins is not needed (for the record: `sub_config_storage_` would keep such sub-configs anyway);
  - registry lookups of expected entries assert instead of returning null.
- **Licence checks unchanged.** `checkReconstructor()`, the selector entry, the MLAT/ADS-B/SMR inspectors (via `AnalyzeDataSourceTask::professionalLicenseEnabled()`, [analyzedatasourcetask.cpp:793](src/task/analyze/analyzedatasourcetask.cpp#L793)) and the DocX export ([taskresultswidget.cpp:122](src/task/taskresultswidget.cpp#L122)) keep checking the `ComponentProbIMMReconstructor` licence exactly as today.
- **A failed check aborts the client.** `compass_handler` reports the crash (message box or relayed exit code, [watchdog/main.cpp:74-94](src/app/watchdog/main.cpp#L74)) and does not restart the client, so there is no restart loop.
- **Startup summary** in the log: the loaded plugins, with name, file and build ID.
- **Builds without `experimental_src`.** The ProbIMM plugin is not built and the expected list is empty. After Workstream A, the only code referencing experimental components is the root's registration step, which is compiled only with `experimental_src`. This is not a runtime case.

### F: AppImage packaging + tests (~1 wk)

The AppImage is the only Linux deliverable, so it must contain the host libraries, every plugin and all of their dependencies, and must find the plugins at runtime on every supported distribution.

#### How the AppImage is built today

1. [docker/build_compass.sh](docker/build_compass.sh), inside the Debian 10 container: `cmake -DCMAKE_INSTALL_PREFIX=/usr`, build, `cmake --install`. This puts `compass_client` and `compass_handler` into `/usr/bin`, the (unused) static archives into `/usr/lib` and `conf`/`data` into `/usr/compass`.
2. [docker/deploy_compass.sh](docker/deploy_compass.sh):
   - empties `appimage/appdir/`;
   - copies the OSG plugins and `libosgEarth*` into `appdir/lib/`, and `data`, `conf` and `presets` into `appdir/compass/`;
   - runs `linuxdeploy` with `--executable` for both binaries plus the `qt` and `gtk` plugins.

   linuxdeploy copies the executables into `usr/bin/`, follows their `DT_NEEDED` entries **transitively** to bundle every shared library into `usr/lib/` (except its system excludelist), and sets `RUNPATH` so they find each other. The qt plugin adds Qt libraries and Qt plugins.
3. At runtime `AppRun` starts `compass_handler -f`, which starts `compass_client`. The client uses `$APPDIR` for the data paths, the OSG plugin path and `GDAL_DATA` ([client.cpp:933-955](src/app/client/client.cpp#L933)).

The CI then runs the Python integration tests against the finished AppImage ([docker/run_ci_tests.sh](docker/run_ci_tests.sh)).

#### What changes

1. **The host libraries are bundled automatically.**
   - Install them with `install(TARGETS ... DESTINATION lib)`, replacing the static `COMPASS_INSTALL_LIBS` list in [CMakeLists.txt:269-279](CMakeLists.txt#L269).
   - `compass_client` has them as `DT_NEEDED`, so linuxdeploy copies them into `usr/lib/` and also bundles their dependencies (osgEarth, jASTERIX, DuckDB, log4cpp, …), which are now dependencies of the libraries rather than of the executable.
   - `compass_handler` does not link COMPASS libraries and is unaffected.
2. **The plugin is invisible to linuxdeploy.** No file has it as `DT_NEEDED`, so it would silently be missing from the AppImage. Two explicit steps in `deploy_compass.sh`:
   - install the plugin to `/usr/lib/compass_plugins/` and copy that directory to `appdir/usr/lib/compass_plugins/`;
   - add `--deploy-deps-only=<appdir>/usr/lib/compass_plugins` to the linuxdeploy call. This bundles every dependency of the plugins without moving the plugin files themselves.

   Today ProbIMM needs nothing the host doesn't already bring (osgEarth, Boost, Qt; Eigen is header-only). The step keeps the AppImage correct as soon as a plugin gains its own dependency. `--library=` is not used because it would put the plugin into `usr/lib/` next to all other libraries.
3. **Plugin `RUNPATH`.**
   - Set `INSTALL_RPATH "$ORIGIN/.."` on the plugin target. It resolves to `usr/lib/` in the AppImage and to `/usr/lib/` in the container install, so the plugin finds the host libraries and the bundled dependencies in both.
   - Verify with `readelf -d` that linuxdeploy does not rewrite it.
4. **Finding the plugins at runtime, without `$APPDIR`.** `PluginManager` derives the directory from the executable: `QCoreApplication::applicationDirPath() + "/../lib/compass_plugins"`. The same rule works everywhere:

   | Environment | Executable | Plugin directory |
   |---|---|---|
   | AppImage | `$APPDIR/usr/bin/compass_client` | `$APPDIR/usr/lib/compass_plugins` |
   | container install | `/usr/bin/compass_client` | `/usr/lib/compass_plugins` |
   | dev build | `build/bin/compass_client` | `build/lib/compass_plugins` (set the plugin's `LIBRARY_OUTPUT_DIRECTORY` accordingly) |

   An environment variable `COMPASS_PLUGIN_PATH` overrides the directory. It is meant for development and tests, e.g. pointing it at an empty directory to check that the startup check fires on the finished AppImage.
5. **Host and plugins must come from the same build.**
   - Generate a `COMPASS_BUILD_ID` at configure time (git hashes of `compass` and `experimental_src` plus a configure timestamp) and compile it into both host and plugins.
   - The startup check (Workstream E) asserts on a plugin with a different ID, with the message "plugin from a different build".
   - In the AppImage this cannot happen, since `deploy_compass.sh` empties the appdir. In dev build directories, stale plugin files are a real risk, and the check turns them from mysterious crashes into a clear message.
6. **Qt modules.** linuxdeploy-plugin-qt decides which Qt libraries and Qt plugins to bundle by scanning the deployed ELF files. Verify that it scans `usr/lib/compass_plugins/` too. If it doesn't, the rule is that a plugin may only use Qt modules the host links, which is the case for ProbIMM.
7. **ASan builds** need no extra handling. The plugin is built with the same flags, and `--deploy-deps-only` bundles `libasan` for it as for the client.

#### Checking the AppImage

- **New `docker/check_appimage.sh`, run at the end of [make_appimage.sh](docker/make_appimage.sh).** It extracts the AppImage and fails the build if any of these is wrong:
  - every host library `usr/lib/libcompass_*.so` and every expected `usr/lib/compass_plugins/compass_plugin_*.so` exist;
  - the plugin `RUNPATH` is `$ORIGIN/..`;
  - `ldd` on `compass_client`, every host library and every plugin, with only the appdir's library directories on the path, reports no `not found`;
  - no `libcompass_*.a` archives were bundled.
- **CI integration tests on the AppImage.**
  - A running client means the startup check passed, so no plugin query command is needed.
  - A ProbIMM reconstruction on a test dataset is compared against a stored reference result.
  - A negative run starts the same AppImage with `COMPASS_PLUGIN_PATH` set to an empty directory and asserts a non-zero exit code with the plugin assert message in the log.
- **Manual check on clean systems.** Start the Debian 10–built AppImage on the supported distributions (Debian 10+, Ubuntu 18.04+, Linux Mint 18.3+) and check the plugin summary in the log.

#### Windows

Windows packaging is Phase 7 of the port plan:
- the host `compass_*.dll` files and third-party DLLs flat next to `compass_client.exe`, plugins in `plugins\`;
- there is no `$ORIGIN`. A plugin's dependencies resolve through the standard DLL search order, which includes the executable's directory. Verify this with a plugin loaded from `plugins\`;
- the runtime rule in item 4 becomes `applicationDirPath() + "/plugins"` on Windows.

#### Unit tests

Build the plugin's sources once as an `OBJECT` library consumed by both the `MODULE` target and `compass_tests`, which replaces `compass_tests`' link to `compass_reconst_adv`. Tests that construct plugin types directly (`test_jpda`, `test_accuracy_*`, `test_adsb_quality`, `test_dubious_references`) link normally. Tests that go **through the registry** keep the direct registration call in the test main from Workstream A; no module is loaded. `compass_tests` does not run the startup presence check.

### Phase 1 verification

1. **Workstreams A and B are a functional no-op on Linux.**
   - `cmake --build build && ./build/bin/compass_tests` passes unchanged;
   - `docker/run_make_appimage.sh` produces a working AppImage with the host libraries bundled, and the CI integration tests pass on it;
   - crash backtraces still resolve names;
   - reconstruction wall time is within noise of the static build;
   - the geo view still renders, grid export lands in it, and LaTeX/PDF export contains its screenshots including the overview.
2. **Single instances across modules.** Log the address of `Logger`, `RTCommandRegistry` and `logger::debug_enabled` from host and plugin; they must match. Toggling the debug log level must affect plugin `logdbg` output.
3. **Functional parity.** A ProbIMM reconstruction against a reference recording produces **identical output to the pre-split build** (same target count, same reference trajectories). This catches a broken Kalman registry. The three plugin tests appear in the test lab menu and run.
4. **Startup check.** Each failure aborts at startup with a message naming the plugin and the cause, before any config is loaded and without touching `~/.compass/<version>/`:
   - the plugin file removed from the install;
   - a plugin from a different build (stale file in a dev build dir);
   - a plugin that skips one of its expected registrations (temporary test hack).
5. **AppImage.** `check_appimage.sh` passes, the CI integration tests (ProbIMM reference result, negative run with an empty plugin dir) pass on the AppImage, and the AppImage starts with the plugin loaded on clean Debian 10, Ubuntu 18.04 and Linux Mint installations.
6. **Windows milestones** (when the port gets there): plugin DLL links against the host libraries' import libraries → loads at startup → `compass_tests` passes → a ProbIMM run matches the Linux result.

**Evaluation point.** After phase 1, review effort, export-set friction and runtime cost before starting phase 2.

---

## Phase 2: Geographic View (follow-up)

### G: Geographic View plugin (~1 wk)

`compass_geo_view` leaves the host and becomes the `MODULE` library `compass_plugin_geoview`. Its host sites already go through registries (Workstream A), so this phase only turns the library into a plugin. Line numbers below are from 2026-11-04 and must be re-checked when phase 2 starts.

- **`LifecycleHooks`** (core registry). Replaces the root's direct `geo_view::shutdown()` call ([compassroot.cpp:415](src/app/compassroot.cpp#L415)). [geo_view_api.h](experimental_src/view/geo_view_api.h) is already the right shape. `geo_view::clearHiddenLayers()` at [mainwindow.cpp:1226](src/app/client/mainwindow.cpp#L1226) is now a no-op and can simply go.
- **Entry point.** `plugins/geoview/plugin.cpp`: a `Plugin` subclass wrapping the geo view's registration functions. The root's direct registration call for the geo view goes.
- **Startup check.** Add the geo plugin and its expected registry entries (`GeographicView` view class, its RT commands, its lifecycle hooks, its `GeoImageSink` implementation) to the expected list. No runtime presence checks: add-view menu, `GeographicView0` in `views.json`, grid view export and LaTeX overview rely on the plugin being there.
- **CMake.** `compass_geo_view` becomes a `MODULE` and links the host through `compass_plugin_sdk`.
- **Tests.** The geo unit tests (overlay, labels, raster, rec num index, added 2026-10) construct geo types directly and link the plugin's `OBJECT` library like the ProbIMM tests.
- **AppImage.** No new mechanism: the plugin lands in `usr/lib/compass_plugins/` and `--deploy-deps-only` picks up its dependencies. Add it to the expected files in `check_appimage.sh`. The OSG plugins, `libosgEarth*` and `GDAL_DATA` handling stay host-side, since the host uses osgEarth itself.

**Verification.** Map renders, overlays and labels draw, grid export lands in the geographic view, LaTeX/PDF export contains the geographic screenshots including the overview, `set_map` registered by the plugin is reachable over `localhost:27960`. The startup check fires for a missing geo plugin as in phase 1.

---

## Relationship to the other plans

**Dependency plan.** Implemented 2026-11-04: the `src/` libraries are acyclic in includes and links, and their include paths enforce the layering. This plan relies on that for Workstream B and on its composition root (`CompassRoot`) for registering and loading components before the config tree exists.

**Windows port.** Phase 1 of this plan is platform-neutral and is done and verified on Linux first.

| Port phase | Absorbs |
|---|---|
| Phase 1 (CMake modernisation) | Workstream B: shared libraries, export macros, dropping `ENABLE_EXPORTS` |
| Phase 3 (platform abstraction) | plugin search path resolution next to the `appDir()` rework (loading itself is `QLibrary`, no new seam) |
| Phase 7 (packaging) | the `plugins/` directory in both the AppImage appdir and the Windows ZIP |

Doing the plugin split Linux-only first and retrofitting Windows later would mean discarding the linking design. Planning for both now costs roughly 1–1.5 weeks extra, about half of which Phase 1 of the port does anyway.

**Compute core.** Follows this plan in the near future. This plan prepares it through the per-library export macros, the SDK interface target and two-phase registration.

## Risks

- **Export set completeness on Windows.** The class-level rule in B.5 covers what is known. Stragglers surface as unresolved externals at the first Windows plugin link: shallow and mechanical, the same character as the MSVC error tail the port plan already expects.
- **Hidden link-level dependencies.** Largely retired: the dependency plan removed the static-archive link repetitions (which uncovered `APP_FILENAME` and the Kalman interfaces), and the symbol check of 2026-11-04 found no missing link between host libraries. Remaining stragglers surface in B.1 as `--no-undefined` link errors.
- **The proof of concept validates Linux only** until the Windows port reaches its plugin milestones. The Windows-specific parts (data-symbol imports, per-DLL statics, DLL search order) are designed for but not exercised by phase 1 on Linux.
- **Inline namespace variables** duplicated per DLL on Windows (B.5) are harmless unless something compares addresses. Check before the first Windows run.
- **Three repositories move in lockstep:** `compass`, `experimental_src` and `jASTERIX`, as the port plan already notes.
- **No degraded mode.** A missing or broken plugin makes the client unusable instead of losing one feature. This is intended, since plugins always ship, and `check_appimage.sh` plus the CI runs catch packaging errors before release.

## Effort

For one senior developer who knows this codebase, working with Claude Code.

| Workstream | Realistic | Pessimistic |
|---|---|---|
| A: host sites → registries (ProbIMM + geo view) | 2.5 wk | 3.5 wk |
| B: shared libraries + exports | 2 wk | 3 wk |
| C: plugin infrastructure | 3 d | 1 wk |
| D: ProbIMM plugin + test lab entries | 0.5 wk | 1 wk |
| E: startup presence check | 1 d | 2 d |
| F: AppImage packaging + tests | 1 wk | 1.5 wk |
| **Phase 1 total** | **~7 wk** | **~10.5 wk** |
| G: Geographic View (phase 2) | 1 wk | 1.5 wk |

Workstream B carries the full export rule, so phase 2 needs no new annotation pass, only fixes for stragglers. Almost all of it is well-specified refactoring against a compiler that reports mistakes immediately.

**Milestone to track:** the AppImage running on Linux with ProbIMM loaded as a plugin, `compass_tests` and the CI integration tests green, around week 6–7.
