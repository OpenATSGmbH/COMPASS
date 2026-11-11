# Splitting COMPASS into host + loadable plugins

*Design document / migration plan. Status: proposed, not started. First written 2026-08-28, revised 2026-10-23 against the then-current code of both repositories (line numbers refer to that state). Revised 2026-11-03: plugins are always present, so plugin presence is asserted once at startup instead of being handled at runtime. Revised again 2026-11-03: builds on the dependency plan, which runs first. Revised 2026-11-04 after the dependency plan was implemented: paths and line numbers refer to that state, starting point updated, findings of a symbol-level link check added (Workstream A, "Further host ↔ experimental edges"). Revised 2026-11-09 after workstreams J and K of the dependency plan (`COMPASS` in `src/app`, runtime commands registered with their managers, app-level experimental libraries): starting point, registration timing, location of the plugin infrastructure and the remaining host ↔ experimental cycles updated, line numbers refer to that state. Revised 2026-11-10: sub-configurable factories (static, generic `childCreated()` hook) and the `ReconstructorTask` remodel added to Workstream A; the Kalman interface factory replaced by a `ReconstructorBase` virtual; plugins as `Plugin` objects with init steps, the list built in `main` (loaded by expected name from C on), no registry aggregates. Revised 2026-11-11: Workstream A complete; Workstream B done on Linux (shared libraries, MinGW instead of MSVC for Windows, so no export annotations; jASTERIX and NemaTode shared).*

## Context

An external constraint requires that some COMPASS functionality be delivered as separately loadable plugins rather than as part of the monolithic client binary. The components in scope are **`ProbIMMReconstructor`** (with all of advanced reconstruction) and, later, the **Geographic View**.

The delivery model is unchanged: **all C++, all compiled from one tree at every release, host and fitting plugins always shipped together in the same package, from one hand.** Nothing is shipped or sold separately. There is no third-party plugin ecosystem, no ABI stability commitment, no version negotiation, and no API deprecation policy. That removes the hard 80% of what usually makes a plugin architecture expensive.

**All plugins are always present.** A missing or broken plugin is a packaging error, not a supported runtime state. The host checks once at startup that every expected plugin loaded and asserts otherwise. No other code checks for plugin presence, and there is no degraded mode.

### Execution order (decided 2026-11-03)

1. **Dependency plan** ([readme_dependencies.md](readme_dependencies.md)): now. It removes the cycles between the `src/` module libraries.
2. **This plan**: next.
3. **Compute core plan** ([readme_compute_core.md](readme_compute_core.md)): in the near future.

**Starting point.** After the dependency plan (workstreams A–I implemented 2026-11-04/05, J, K.1 and K.3 2026-11-06):
- the `src/` module libraries are still static archives, but their dependencies are acyclic, in the includes and in the CMake link graph;
- the libraries' include paths only reach their own directory and the libraries below them, so an upward include does not compile (`scripts/check_layering.py`, run manually, checks this and the CMake files);
- `COMPASS` lives in `compass_app` (`src/app/compass.h`) as application facade and is used only at app level; code below gets its managers injected. The composition root `CompassRoot` in `compass_app` creates the managers through the config tree (`COMPASS::createManagers()`), wires them (`setManagers(...)`, checked via `IManager::checkWired()`) and runs the startup and shutdown steps;
- runtime commands get their managers in the constructor and are registered by `init_*_commands(registry, managers...)` of their module (K.3). They can therefore only be registered once the managers exist;
- the app-level parts of `experimental_src` (`compass_rtcmd_ext`, `compass_test_lab`) link `compass_app`, since they use `COMPASS` or `CompassRoot`;
- a symbol-level check (2026-11-04) confirmed that every host library's declared link dependencies cover all symbols it uses from other host libraries;
- the remaining cycles are those between the host and `experimental_src`: the host sites that name ProbIMM and `GeographicView` types, plus the edges listed under Workstream A, "Further host ↔ experimental edges". Static archives tolerate them; shared libraries do not.

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

- **ProbIMM plugin contains all of `compass_reconst_adv`** (`complex/`, `kalman/`, `jpda/`) **plus the three test lab tests that use its types** (Workstream D). This requires that the host stops constructing experimental Kalman interfaces (solved by a `ReconstructorBase` virtual, Workstream A). Keeping `kalman/` in the host was rejected.
- **Plugin presence is asserted once at startup (decided 2026-11-03).** All plugins are always shipped, so a missing plugin aborts the client at startup with a clear message (Workstream E). There is no runtime presence query, no fallback and no UI gating for absent plugins.
- **Licence checks stay as they are.** Plugin presence no longer varies at runtime, so the licence is the only runtime axis, and the existing licence checks remain unchanged.
- **Plugins are CMake `MODULE` libraries**, loaded via `QLibrary`, with no import library on Windows. Nothing links against a plugin. The host only resolves the entry symbol `compass_plugin_create` at runtime, and everything else goes through the returned `Plugin` object and the registries it fills. CMake rejects `target_link_libraries()` on a `MODULE` target, which enforces that the host libraries, the client and the tests build without any plugin. The `.lib`/`.exp` that MSVC may still emit for the exported entry symbol goes unused and is not shipped.
- **Stay in the host:** `compass_osg_utils`, `compass_docx`, `compass_rtcmd_ext`, `compass_test_lab` (minus the three ProbIMM tests), the MLAT/ADS-B/SMR inspectors under `experimental_src/analysis/`, and in phase 1 also `compass_geo_view`. osgEarth stays a host dependency; host code uses it directly ([accuracy.cpp:24](src/util/accuracy.cpp#L24), [measurement.cpp:27](src/reconstruction/measurement.cpp#L27), [client.cpp:67-69](src/app/client/client.cpp#L67)).

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

A plugin links the host libraries through `compass_plugin_sdk`, an INTERFACE target listing the libraries a plugin may link. ProbIMM needs `compass_core`, `compass_reconstruction`, `compass_task`, `compass_gui` and `compass_view` (plus osgEarth). The geo view additionally needs `compass_result` and `compass_osg_utils`. Both link `compass_app` for the `Plugin` interface, whose implementation in the plugin's directory `experimental_src/plugins/<name>/` is the only plugin code that uses it.

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
- **Phased plugin registration** (see Extension points). A headless binary calls only the core and command steps.

## Extension points

**Plugin list built in `main` (decided 2026-11-10).** Each plugin is a `Plugin` object with init steps (see below). `compass_client`'s `main.cpp` builds the list of plugin objects and hands it via `Client` to `CompassRoot`, which keeps it; at fixed points of the startup the host calls each step for every plugin in the list. Where the list comes from changes, the call points do not:
- **Phase 1 until Workstream C:** the plugins are statically linked; `main.cpp` (and the test main) create the plugin objects directly, under `USE_EXPERIMENTAL_SOURCE`.
- **From Workstream C on:** `PluginManager` (`src/app/plugin/`, called from `main`) loads the expected plugins by name (no folder scan, there are no drop-in plugins), runs the startup check and returns the list.

The core step runs **before** the config tree and with it the managers are instantiated (`COMPASS::createManagers()`), since [task_reconstructor.json](conf/default/task_reconstructor.json) names `ProbIMMReconstructor0` (and [views.json:16](conf/default/views.json#L16) names `GeographicView0`).

**Location (decided 2026-11-10).** The earlier design put the plugin infrastructure into `compass_core`. That conflicts with the layering: the init steps hand out `CompassRoot` and with it the upper managers (e.g. `ViewManager` for the command registration). The infrastructure therefore lives at the top, in `compass_app`, next to `CompassRoot`, and plugins link `compass_app` through `compass_plugin_sdk` (`compass_app` itself never names a plugin, only `main.cpp` does, as with `AppHooks`). For the compute core plan this means a headless binary uses the same app-level plugin loading without the UI step.

**Registration timing (decided 2026-11-10).** Since K.3, runtime commands take their managers in the constructor, so they cannot be registered before the managers exist, while the factories must be registered before. Registration therefore has three steps instead of two:

| Step | When | Registers |
|---|---|---|
| `initCore()` | in `CompassRoot`, after creating `COMPASS`, before `createManagers()` | factory entries (reconstructor); the factories are static, so no parameters |
| `initCommands(RTCommandRegistry&, CompassRoot&)` | in `CompassRoot`, after `startManagers()` | runtime commands, with the managers they need (K.3 pattern, like `extra::init_extra_commands(registry, root)`) |
| `initUI(CompassRoot&)` | when the UI is set up, before the `MainWindow` constructor calls `ViewManager::init()` ([mainwindow.cpp:209](src/app/client/mainwindow.cpp#L209)), which creates the view containers and with them `GeographicView0` | views (view factory entries), widgets, test lab entries |

The factory registries must exist before the managers. Decided 2026-11-10: they are owned by `CompassRoot` and injected into their consuming managers (constructor or `setManagers(...)`, like the other dependencies), not singletons; the dependency plan only keeps `Logger` and `RTCommandRegistry` as singletons. **Exception (decided 2026-11-10):** factories that create sub-configurables (reconstructors, views) are static members of their parent configurable, see "Sub-configurable factories" below.

### Sub-configurable factories (decided 2026-11-10)

Reconstructors and views are sub-configurables: their parent creates them by class name from the config tree (`Configurable::generateSubConfigurable()`, an if/else chain per parent). A generic factory in `compass_config` lets a parent create them by class name without naming the concrete types, so plugins can add types:

- **`SubConfigurableFactory<Base, Parent>`** (`src/config/subconfigurablefactory.h`): maps class names to creators `std::unique_ptr<Base>(nlohmann::json&, Parent&)`.
  - `add(class_name, creator)` for any constructor, `add<T>(class_name)` as shorthand for the common constructor `T(nlohmann::json&, Parent*)`. Class names are unique (asserted). An optional display name per class serves menus (e.g. the "Add view" menu).
  - `canCreate()`, `create()`, `classNames()`.
- **Static members via macro.** `DECLARE_SUBCONFIGURABLE_FACTORY(Name, Base, Parent, Defaults)` in the public part of the parent's class declares a typedef `NameType` and a static accessor `static NameType& Name();`. The parent's .cpp defines the accessor with a function-local static that is initialized with the built-in types:

  ```cpp
  // viewcontainer.h
  DECLARE_SUBCONFIGURABLE_FACTORY(viewFactory, View, ViewContainer, None)

  // viewcontainer.cpp
  ViewContainer::viewFactoryType& ViewContainer::viewFactory()
  {
      static viewFactoryType factory = { { "TableView", &viewFactoryType::createDefault<TableView>, "Table View" },
                                         ... };
      return factory;
  }

  // geo view registration
  ViewContainer::viewFactory().add<GeographicView>("GeographicView");
  ```

  A static accessor instead of a static data member: functions are exported automatically on Windows (no B.5 annotation needed for this), and the factory is initialized on first use (no static initialization order issue). Plugins add their types at runtime in their registration step, never from static initializers. Registrations persist for the whole process, also across test cases: a test registers a plugin type once.
- **Optional, several per parent, attached automatically.** Besides the accessor, the macro declares a member `SubConfigurableFactoryAttacher Name_attacher_ { this, Name() }` that attaches the factory to its instance (`Configurable::addSubConfigurableFactory()`, protected, the attacher is a friend). Members are initialized after the `Configurable` base and before the constructor body, so the factory is attached before `createSubConfigurables()`; derived parents inherit it. No attach statement in the parent's constructor, and no global collection of factories. A configurable without factories behaves as before. (Decided 2026-11-10; rejected: an explicit `addSubConfigurableFactory()` call per factory in the constructor, and a global list of all factories filtered by parent type.)
- **Dispatch via a wrapper.** The non-virtual `Configurable::createSubConfigurable(child_json)` asks the attached factories (a class name known to several asserts) and otherwise calls the virtual `generateSubConfigurable()`, so all 39 overrides and the base's "not implemented" throw stay unchanged. `createSubConfigurables()`, `generateSubConfigurableFromConfig()`, `generateSubConfigurableFromJSON()` and the reconfigure path ([configuration.cpp:1083](src/config/configuration.cpp#L1083)) call the wrapper; direct callers of `generateSubConfigurable()` in parents with factories switch to it (`ViewContainer::addView()`), the others can stay.
- **Hook.** `virtual void childCreated(Configurable* child)` in `Configurable`, called for every factory-created child. The derived parent **takes ownership** there (stores it in its own `std::unique_ptr` container, connects signals, inits it). The base default asserts, so a parent with a factory but no override fails loudly instead of leaking.
- **Ownership stays in the derived parent**, as today. Deleting a child is erasing it from the parent's container; `~Configurable` of the child already removes it from the parent's children and its sub-config entry (as for views today, [viewcontainer.cpp:212-230](src/view/viewcontainer.cpp#L212)). No `Configurable::deleteChild()`.
- **Default instances (decided 2026-11-10).** The required 4th macro parameter `Defaults` (`SubConfigurableDefaults::None` / `OnePerClass`, no default value) is passed to `addSubConfigurableFactory()` by the attacher. `createSubConfigurables()` runs a private, non-virtual `createDefaultSubConfigurables()` after the creation from the config and before the virtual `checkSubConfigurables()` (all overrides unchanged): for `OnePerClass` factories it creates `<class_name>0` for every class without a child of that class (any instance name), including types added at runtime by plugins. Views use `None` (no view per type in each container), reconstructors `OnePerClass` (removes `ReconstructorTask::checkSubConfigurables()` with its `USE_EXPERIMENTAL_SOURCE` block).

Rejected alternatives: root-owned injected factories for these (3a; static members preferred, the factory belongs to its parent class), a typed hook per factory (`reconstructorCreated()`; one generic hook suffices), a mixin base class per factory type (extra multiple inheritance on `QObject` classes), ownership in the base `Configurable` (typed access would need a second list, and deletion a base API).

Plugin interface (`src/app/plugin/plugin.h`, Workstream A) and, from Workstream C on, one exported C symbol per module:

```cpp
extern "C" Plugin* compass_plugin_create();   // returns a static instance (Workstream C)

class Plugin
{
public:
    virtual ~Plugin() = default;

    virtual std::string name() const = 0;

    virtual void initCore() {}                                                             // factory entries, no widgets
    virtual void initCommands(rtcommand::RTCommandRegistry& registry, CompassRoot& root) {} // runtime commands
    virtual void initUI(CompassRoot& root) {}                                              // widgets, views, menus
    virtual void shutdown() {}                                                             // before the managers go (phase 2)
};
```

The plugin's components provide their registration as functions with exactly this split, taking only what they need (e.g. `init_geographic_view_commands(registry, view_manager)`), so they stay below `compass_app`; the `Plugin` object calls them and is the only part that sees `CompassRoot`. No registry aggregates (`PluginRegistry` / `PluginUIRegistry` of the earlier design): the sub-configurable factories are static, the remaining registries are reached through `CompassRoot`.

Registries:

| Registry | Reg. phase | Library | Replaces | Phase |
|---|---|---|---|---|
| `ReconstructorTask::reconstructorFactory()` (static, sub-configurable factory) | core | `compass_task` | `new ProbIMMReconstructor` + `new ComplexAccuracyEstimator` at [reconstructortask.cpp:176-188](src/task/reconstructor/reconstructortask.cpp#L176) | 1 |
| `ViewContainer::viewFactory()` (static, sub-configurable factory) | UI | `compass_view` | built-in views and `new GeographicView` at [viewcontainer.cpp:296-305](src/view/viewcontainer.cpp#L296), class list at [viewmanager.cpp:243](src/view/viewmanager.cpp#L243) | 1 |
| RT command registration | commands | `compass_core` (`RTCommandRegistry`) | `init_geographic_view_commands(registry, view_manager)` in the `ViewManager` constructor ([viewmanager.cpp:113](src/view/viewmanager.cpp#L113)) | 1 |
| `GeoImageSink` (interface, done 2026-11-10) | – | `compass_view` | `viewsOfType<GeographicView>()` in the grid view export | 1 |
| `TestLabRegistry` | UI | `compass_test_lab` | plugin-specific entries in the host test lab menu (`TestLabPhil().addTestsToMenu()` at [test_lab.cpp:97](experimental_src/test/test_lab.cpp#L97)) | 1 |
| `Plugin::shutdown()` (formerly `LifecycleHooks`) | shutdown | `compass_app` | `geo_view::shutdown()` in `CompassRoot::shutdown()` (done 2026-11-10 via `GeoViewPlugin`) | 1 |

The sub-configurable factories build on `Configurable::generateSubConfigurable()`, which is already a string-keyed factory, see "Sub-configurable factories" above.

Widget creation for reconstructors is a virtual on the reconstructor (`ReconstructorBase::createWidget()`, Workstream A). The plugin links the UI libraries until it is split internally into core and UI parts; that is a compute core plan concern.

### Startup presence check

`PluginManager` knows the set of **expected plugins** from the build (Workstream E) and loads exactly those. Every step of loading a plugin is checked with `traced_assert_msg`, and so is the registration result: each expected registry entry must exist after `initCore()` / `initCommands()` / `initUI()`. After this check, host code may rely on every registry entry being present. Registry lookups for expected entries assert instead of returning an "unavailable" result.

`traced_assert` does not depend on `NDEBUG` and is active in release builds, so the check also protects the AppImage.

---

## Phase 1: ProbIMM proof of concept

Order: A → B → C → D → E → F. A and B are functional no-ops: after them, ProbIMM and the geo view are still regular host libraries, registered statically by the composition root.

### A: Host sites → registries (~3 wk)

Removes the host ↔ experimental cycles. The experimental libraries stay regular libraries here. Each component gets its registration functions (core, commands, UI; see Extension points, "Registration timing"), called by its `Plugin` object's init steps.

**Plugin location (decided 2026-11-10).** Every plugin is one directory `experimental_src/plugins/<name>/` holding all of its sources (in subdirectories as needed), which is the plugin's library (directory = library; the `MODULE` from D/G on). Plugins are always closed source, so they live in the `experimental_src` repository, never in the COMPASS repository. The sources move there at the start of the plugin's part of Workstream A, so paths and includes change only once:
- ProbIMM: `experimental_src/reconstruction/` (`complex/`, `kalman/`, `jpda/`, readmes) → `experimental_src/plugins/probimm/`, **done 2026-11-10** (plain move, CMake paths in `experimental_src/CMakeLists.txt`, `src/reconstruction/CMakeLists.txt`, `src/task/CMakeLists.txt` and the doc links updated). The library keeps its name `compass_reconst_adv` until D makes it the `MODULE` `compass_plugin_probimm`. `experimental_src/util/` (`extra_resources`, compiled into `compass_core`) is not part of the plugin and stays.
- Geo view (decided 2026-11-10, after briefly keeping it a regular library): a plugin like ProbIMM, i.e. a static `GeoViewPlugin` object in phase 1, a loadable module only in the follow-up G. `experimental_src/view/` (facade `geo_view_api.{h,cpp}` + `geographicview/`) → `experimental_src/plugins/geoview/`, **done 2026-11-10** (plain move; paths in `experimental_src/CMakeLists.txt`, the host include blocks of `src/view`, `src/result`, `src/task` and the doc links updated). The library keeps its name `compass_geo_view` until G.

**Plugin interface and list** (before ProbIMM's registration): `src/app/plugin/plugin.h` with the `Plugin` interface (moved here from C); `Client` and `CompassRoot` take the plugin list from `main.cpp` (`CompassRoot::plugins()`); `CompassRoot` calls `initCore()` before `createManagers()` and `initCommands()` after `startManagers()`, `MainWindow` calls `initUI()` before `ViewManager::init()`. The plugin objects are statically linked and created in `main.cpp` under `USE_EXPERIMENTAL_SOURCE`; the test main creates the same objects and calls `initCore()`. **Done 2026-11-10:** [plugin.h](src/app/plugin/plugin.h) (`Plugin`, `Plugins` = non-owning `std::vector<Plugin*>`, the plugin objects are static instances); the list travels in `AppHooks::plugins` (already passed `main` → `Client` → `CompassRoot`, `root.appHooks().plugins`); `CompassRoot` calls `initCore()` before `createManagers()`, `initCommands()` after `startManagers()` and `shutdown()` in its shutdown sequence (before `geo_view::shutdown()`), `MainWindow` calls `initUI()` before `ViewManager::init()`.

**Sub-configurable factories** (first, see Extension points, "Sub-configurable factories"): `SubConfigurableFactory`, `DECLARE_SUBCONFIGURABLE_FACTORY` with the attacher member, `Configurable::addSubConfigurableFactory()`, the wrapper `createSubConfigurable()` and the `childCreated()` hook, with unit tests in `src/config/unit_tests/`. **Done 2026-11-10** ([subconfigurablefactory.h](src/config/subconfigurablefactory.h), [test_subconfigurable_factory.cpp](src/config/unit_tests/test_subconfigurable_factory.cpp)).

**ProbIMM:**

- **Remodel of `ReconstructorTask` (decided 2026-11-10, part of A).** The reconstructors are held generically in `std::vector<std::unique_ptr<ReconstructorBase>> reconstructors_`, replacing the members `simple_reconstructor_` and `probimm_reconstructor_`.
  - `DECLARE_SUBCONFIGURABLE_FACTORY(reconstructorFactory, ReconstructorBase, ReconstructorTask, OnePerClass)`. `SimpleReconstructor` (with `SimpleAccuracyEstimator`) is a built-in entry; ProbIMM adds `ProbIMMReconstructor` (with `ComplexAccuracyEstimator`, one creator creating both) in `ProbIMMPlugin::initCore()`. `ProbIMMPlugin` (`experimental_src/plugins/probimm/probimmplugin.{h,cpp}` in `compass_reconst_adv`, which then links `compass_app`; the entry symbol follows in D) is created in `main.cpp`.
  - `generateSubConfigurable()` ([reconstructortask.cpp:161-192](src/task/reconstructor/reconstructortask.cpp#L161)) is replaced by the factory dispatch; `childCreated()` stores the reconstructor and connects `configChanged`.
  - `checkSubConfigurables()` ([:1570-1584](src/task/reconstructor/reconstructortask.cpp#L1570)) is removed, the factory's `OnePerClass` creates the default instances.
  - `simpleReconstructor()` / `probIMMReconstructor()` (6 uses) become a lookup by class name returning `ReconstructorBase*`; `currentReconstructor()` keeps its interface.
  - **Done 2026-11-10**, incl. the ProbIMM registration: [probimmplugin.cpp](experimental_src/plugins/probimm/probimmplugin.cpp) adds the entry in `initCore()` (`compass_reconst_adv` links `compass_app`), created in `main.cpp` and the test main, checked by `test_probimm_plugin.cpp`; `reconstructorFactory()` only holds `SimpleReconstructor`. The reconstructor names (`current_reconstructor_str_`, persisted) are the factory display names (`ScoringUMReconstructorName`, `ProbImmReconstructorName`, the latter now outside the `USE_EXPERIMENTAL_SOURCE` guard), so the constructor fallback, the setter assert and `currentReconstructor()` go through `reconstructorClassName()` without naming types. New public `reconstructor(class_name)` and `reconstructorClassName(name)`; the typed `simpleReconstructor()` / `probIMMReconstructor()` are removed with the widget change (the dialog was their only user).
- **Widget: a reconstructor virtual, no factory (decided 2026-11-10).** `ReconstructorBase` had no `widget()`; both derived classes declare a concrete-typed one ([simplereconstructor.h:63](src/task/reconstructor/simplereconstructor.h#L63), [probimmreconstructor.h:185](experimental_src/plugins/probimm/complex/probimmreconstructor.h#L185)). `ReconstructorBase` gets `virtual QWidget* createWidget() = 0` (ownership by caller), implemented by both via their `widget()`. **Done 2026-11-10:** [reconstructortaskdialog.cpp](src/task/reconstructor/reconstructortaskdialog.cpp) offers ProbIMM if licensed and known to the reconstructor factory (no `#if`), and adds one `createWidget()` per combo box entry in the same order (index = stack index); no ProbIMM includes in the task and the dialog any more, and `compass_task`'s include paths into `experimental_src/plugins/probimm/` are gone.

  Rejected alternative: a `ReconstructorWidgetFactory` filled in the UI step, meant to let a headless binary load ProbIMM's core without widgets. The widget code is part of the reconstructor classes and the plugin library anyway, so a factory would not separate it; that split is compute core plan work.
- **Selection logic.** `checkReconstructor()` ([reconstructortask.cpp:199-207](src/task/reconstructor/reconstructortask.cpp#L199)) forces ProbIMM whenever the licence is enabled. This stays licence-only, as do the constructor fallback (`:99-105`), the setter assert (`:375-380`) and the selector entry ([reconstructortaskdialog.cpp:79](src/task/reconstructor/reconstructortaskdialog.cpp#L79)). They only stop naming the concrete type. These sites use `ProbImmReconstructorName` outside the `USE_EXPERIMENTAL_SOURCE` guard, while the header declares it inside one ([reconstructortask.h:139-143](src/task/reconstructor/reconstructortask.h#L139)); the name constant moves out of the guard.
- **Kalman interfaces: a reconstructor virtual, no factory (decided 2026-11-10).** `KalmanEstimator::createInterface()` ([kalman_estimator.cpp:158-192](src/reconstruction/kalman_estimator.cpp#L158)) is the only production path that constructs experimental filters (`KalmanChain`, `KalmanChainPredictors`, `KalmanOnlineTracker` all go through `KalmanEstimator::init()`). All production calls of `init(ktype)` sit in `ReconstructorBase` ([reconstructorbase.cpp:2327,2356,2373,2394](src/task/reconstructor/reconstructorbase.cpp#L2327)) or in `ReferenceCalculator` ([referencecalculator.cpp:1030,1072](src/task/reconstructor/referencecalculator.cpp#L1030)), which holds `reconstructor_`. Every `init()` already has an overload taking a finished interface (chain and chain predictors clone it).
  - `ReconstructorBase` gets `virtual std::unique_ptr<KalmanInterface> createKalmanInterface(kalman::KalmanType ktype, const KalmanEstimator::Settings& settings) const`. The base implementation forwards to `KalmanEstimator::createInterface()`, which is reduced to the built-in `UMKalman2D` and throws for other types (its `#if` branches go).
  - `ProbIMMReconstructor` overrides it for AM, IMM and UMFull (incl. AM for the hardcoded IMM→AM resample switch at [referencecalculator.cpp:1069](src/task/reconstructor/referencecalculator.cpp#L1069)) and falls back to the base otherwise. The plugin needs no Kalman registration.
  - The call sites use the interface overloads: `init(createKalmanInterface(ktype, settings))`, in `ReferenceCalculator` via `reconstructor_`. The compile-time `#if USE_EXPERIMENTAL_SOURCE == false` assert at [referencecalculator.cpp:1026](src/task/reconstructor/referencecalculator.cpp#L1026) goes, the base's throw covers it.
  - The `init(ktype)` overloads stay for the built-in UM (host unit tests `test_kalman_estimator`, `test_kalman_chain`, and `test_jpda`, `test_target_fixtures`). Experimental code using IMM outside a reconstructor (`test_kalman_imm.cpp:98`, `kalman_test.cpp:350`) constructs the experimental interface directly and uses the interface overload.
  - No behaviour change: `SimpleReconstructor` uses UM for association and final ([simplereconstructor.cpp:63-66](src/task/reconstructor/simplereconstructor.cpp#L63), its widget only offers "Uniform Motion"); only `ProbIMMReconstructor` uses IMM ([task_reconstructor.json](conf/default/task_reconstructor.json): `ref_rec_type_final` 2 vs. 0). The settings default `kalman_type_final = IMMKalman2D` ([referencecalculatorsettings.h:110](src/task/reconstructor/referencecalculatorsettings.h#L110)) stays.

  **Done 2026-11-10:** virtual in [reconstructorbase.h](src/task/reconstructor/reconstructorbase.h), override in [probimmreconstructor.cpp](experimental_src/plugins/probimm/complex/probimmreconstructor.cpp); the 4 call sites in `ReconstructorBase` and the 2 in `ReferenceCalculator` use the interface overloads (with the settings of the estimator they init); `createInterface()` only builds `UMKalman2D` and throws "not built in" otherwise. Plugin-side code creates all types through one function, `reconstruction::createProbIMMKalmanInterface(ktype, settings)` ([kalman_interfaces_probimm.cpp](experimental_src/plugins/probimm/kalman/kalman_interfaces_probimm.cpp): AM, IMM, UM full, built-in types via `createInterface()`): the `ProbIMMReconstructor` override, `test_kalman_imm.cpp` and `kalman_test.cpp`. `compass_reconstruction` lost its include path into `experimental_src` and its link-only dependency on `compass_reconst_adv` (the host ↔ experimental cycle of `kalman_estimator.cpp`).

  Rejected alternative: a `KalmanInterfaceFactory` registry (root-owned and injected or static). Injection would have to pass it down the whole `init(ktype)` chain, and the override keeps the experimental filters with the reconstructor that uses them.
- **Analysis widget: no change.** The `probimm_reconst_` bool in [reconstructortaskanalysiswidget.cpp](src/task/reconstructor/reconstructortaskanalysiswidget.cpp) is passed in by each reconstructor's own widget, so it is already plugin-neutral.

**Geographic View:**

- **`GeoViewPlugin`** ([geoviewplugin.cpp](experimental_src/plugins/geoview/geoviewplugin.cpp), in `compass_geo_view`, which now links `compass_app`; created in `main.cpp`): `initUI()` → `geo_view::initViews()`, `initCommands()` → `geo_view::initCommands()`, `shutdown()` → `geo_view::shutdown()` (the facade functions in `geo_view_api.h`). `compass_app` names no geo view code any more: its include path into the facade, the `#if` calls in `CompassRoot` (shutdown) and `MainWindow` and the no-op `geo_view::clearHiddenLayers()` call are gone. **Done 2026-11-10**, with the next two items; `test_geoview_plugin.cpp` checks the view factory entries.
- **View creation.** `ViewContainer` gets `DECLARE_SUBCONFIGURABLE_FACTORY(viewFactory, View, ViewContainer, None)` with the four built-in views as entries (with their display names); the geo view adds `GeographicView` via `GeoViewPlugin::initUI()`. **Done 2026-11-10.** The if/else chain in `ViewContainer::generateSubConfigurable()` ([viewcontainer.cpp:264-305](src/view/viewcontainer.cpp#L264)) is replaced by the dispatch, and `childCreated()` does what each branch does today (`views_.emplace_back()`, `init()`, `addView()`). The class list in `ViewManager` (`viewClassList()`, `newViewName()`) comes from `classNames()` plus the display names (`view_class_list_` removed); `ViewContainer::addView(class_name)` calls `createSubConfigurable()`. In `ViewContainer`, the "compiled w/o experimental source" branch stays a compile-time case (its remaining `generateSubConfigurable()` override, only reached for classes unknown to the factory).
- **RT commands.** `init_geographic_view_commands(registry, view_manager)` is called from `GeoViewPlugin::initCommands()` (after `startManagers()`) instead of from the `ViewManager` constructor. Since K.3 the commands already get the `ViewManager` in their constructor, so only the call site moves. **Done 2026-11-10.**
- **Static `GeographicView::instant_display_`** ([reportexporter.cpp:211-220](src/result/report/reportexporter.cpp#L211), [viewpointsreportgenerator.cpp:171,346](src/view/points/report/viewpointsreportgenerator.cpp#L171)) becomes a flag on the `View` base class. **Done 2026-11-10:** static `View::instantDisplay()` / `setInstantDisplay()`, defined in `view.cpp` (not inline, so other libraries do not reference the static data member, relevant for Windows); the report exporters set it without `#if`, the geo view's data widget reads it.
- **Grid view export.** "Export to GeographicView" ([gridviewconfigwidget.cpp:233,537-663](src/view/gridview/gridviewconfigwidget.cpp#L537)) uses `viewsOfType<GeographicView>()` and the non-virtual `addInternalAnnotation()` at `:659`. It moves to an abstract `GeoImageSink` in `compass_view` with `addInternalAnnotation()`, implemented by the geo view. `viewsOfType<GeoImageSink>()` ([viewmanager.h:288](src/view/viewmanager.h#L288)) works unchanged. **Done 2026-11-10:** [geoimagesink.h](src/view/viewbase/geoimagesink.h), `GeographicView : public View, public GeoImageSink`; the dialog gets the view names via `dynamic_cast<const View*>`; the export menu entry is no longer behind `#if`, it is only added if a `GeoImageSink` view exists when the menu is created (decided 2026-11-10, accepting that a geo view opened later does not add it).
- **LaTeX export: the view exports itself (decided 2026-11-10).** `LatexVisitor::visit(GeographicView*)` was geo-specific (wait for the map, screenshot, overview with zoom/data marker/framebuffer grab). The geo view decides its export itself:
  - `View` gets `virtual void visitLatex(LatexVisitor& v) {}` (default: nothing), `LatexVisitor` a `visit(View*)` that calls it; `GeographicView::accept()` calls `v.visit(static_cast<View*>(this))`.
  - `GeographicView::visitLatex()` holds the former visit code unchanged, using the visitor's accessors `screenshotFolder()`, `currentSection()`, `imagePrefix()`, `addOverviewScreenshot()`, `waitOnMapLoading()`.
  - The other views keep their own visits. `LatexVisitor` names no geo view code any more.
  - **Done 2026-11-10.**

  Rejected alternatives: a generic `visit(View*)` doing the screenshot via the `View` interface plus a `View::renderOverview()` virtual (it does not fit the other views exactly: they check `hasVisibleContent()` and partly grab the chart via `renderPixmap()`), and moving the code into `accept()` directly. (`LatexVisitor` and the view points report generator are in `compass_view` since the dependency plan, the LaTeX document classes in `compass_core`.)
- **`ViewerWidget.{h,cpp}` and `GraphicsWindowQt.{h,cpp}` stay in the geo view plugin (decided 2026-11-10).** The earlier plan moved them into `compass_osg_utils`. Since the LaTeX change no host (`src/`) code uses them (only two commented-out lines in the grid and scatter plot data widgets); the remaining users outside the geo view are the test lab's `osgEarth1MPointsTest` ([test_lab_phil.cpp:4016](experimental_src/test/test_lab_phil.cpp#L4016)) and the disabled unit test `test_offscreen_map.cpp`. The test lab link `compass_test_lab` → `compass_geo_view` is downward and fine for B; it only has to go in G (see there).

**Workstream A is complete (2026-11-10).**

**Both:**

- **Tests.** Tests that go through the registries need the registration; the test main creates the same plugin objects as `main.cpp` and calls their `initCore()`. The Kalman tests need none (see "Kalman interfaces" above).
- **CMake.** Drop the include directories into `experimental_src` from host CMake files once no host file outside the root includes experimental headers:
  - ~~`src/reconstruction/CMakeLists.txt`, including the link-only dependency on `compass_reconst_adv` that the dependency plan declared there for `kalman_estimator.cpp`~~: done 2026-11-10 with the Kalman interfaces;
  - ~~the reconstruction and geo-view parts of the experimental block in `src/task/CMakeLists.txt`~~: done 2026-11-10 (`docx`, `libs/osg`, `test` stay);
  - ~~the geo-view include blocks in `src/view/CMakeLists.txt`, `src/result/CMakeLists.txt` (the geo part only; `experimental_src/docx` and `libs/osg` stay) and `src/app/CMakeLists.txt`~~: done 2026-11-10; after it only `main.cpp` includes a geo view header (`geoviewplugin.h`). `src/app/client/CMakeLists.txt` keeps linking the experimental libraries into `compass_client`, which creates the plugin objects;
  - ~~the PUBLIC include directory `experimental_src/test` of `compass_core`~~: done 2026-11-10 (`test_target` moved to `src/reconstruction/`, test lab hooked in via `AppHooks`, see below).

**Further host ↔ experimental edges** (found by a symbol-level check on 2026-11-04: for every library, the symbols it uses from other COMPASS libraries against its declared link dependencies; updated 2026-11-09 after workstreams J and K). Besides the ProbIMM and geo-view sites above, which the check confirmed, there were three cycles. They are declared or tolerated in the static build and must be removed before B, since shared libraries cannot have cycles:

- **`compass_task` ↔ `compass_test_lab`: resolved 2026-11-10.** `ReferenceCalculator` has a `TestTarget` member ([referencecalculator.h:23,48](src/task/reconstructor/referencecalculator.h#L23)), which was compiled into `compass_test_lab`. Decided 2026-11-10: `test_target.{h,cpp}` (depends only on util, Eigen, json and Boost) moved unchanged into `src/reconstruction/` (`compass_reconstruction`, now GPL source), below all its users (`ReferenceCalculator`, `kalman_test` and `test_lab_phil.cpp`, later the ProbIMM plugin's tests). The CMake edge `compass_task` → `compass_test_lab` is gone.
- **`compass_app` ↔ `compass_rtcmd_ext` and `compass_app` ↔ `compass_test_lab`: resolved 2026-11-10** via `AppHooks` ([src/app/apphooks.h](src/app/apphooks.h)): `compass_client`'s `main.cpp` fills `register_commands` (`extra::init_extra_commands`) and `add_debug_menu_entries` (`TestLabCollection::appendTestLabs`) and passes them via `Client` to `CompassRoot` (`appHooks()`). `MainWindow` calls them at the former call sites (commands still only outside the AppImage). `compass_app` no longer names either library, its include path into `experimental_src/rtcommand` and `compass_core`'s PUBLIC include path into `experimental_src/test` are gone (with the unused `test_plot.h` include in `kalman_filter_imm.cpp`). The ProbIMM plugin's test lab entries (Workstream D) can use the test lab's hook. Background of both cycles:
- *`compass_app` ↔ `compass_rtcmd_ext` (before 2026-11-10).* Since J and K.3 the extra commands take `CompassRoot&` and call `MainWindow::createInMemoryDB()` / `createDBFromMemory()` via `dynamic_cast<MainWindow*>(rtcommand::mainWindow())` ([extra_commands.cpp:134-136,180-182](experimental_src/rtcommand/extra_commands.cpp#L134)), so `compass_rtcmd_ext` links `compass_app`. In the other direction the `MainWindow` constructor calls `extra::init_extra_commands(cmd_registry, root_)` ([mainwindow.cpp:136-139](src/app/client/mainwindow.cpp#L136)). The earlier option "the commands only need `compass_core`" no longer exists, since `COMPASS` is in `compass_app`. The direction app → `compass_rtcmd_ext` has to go.
- *`compass_app` ↔ `compass_test_lab` (before 2026-11-10).* `compass_test_lab` links `compass_app` (`TestLab::compass()`), and the `MainWindow` debug menu calls `TestLabCollection().appendTestLabs(debug_menu, compass_, taskManager, viewManager)` ([mainwindow.cpp:1661](src/app/client/mainwindow.cpp#L1661)). The direction app → `compass_test_lab` has to go.

  Rejected alternative: move the extra commands and the test lab into `compass_client`. That contradicts "directory = library", since their sources live in `experimental_src/`.
- **Undeclared downward links** (no cycle, but must be declared for shared libraries, B.1): `compass_result` uses 24 symbols of `compass_docx` (DOCX export) without linking it; `compass_app` uses `compass_geo_view` (`geo_view::shutdown()`, `clearHiddenLayers()`) with only an include path, while only `compass_client` links it ([src/app/CMakeLists.txt:49-58](src/app/CMakeLists.txt#L49) explains this with duplicate AUTOMOC symbols).
- **Side effect:** the `USE_EXPERIMENTAL_SOURCE` guards for ProbIMM and the geo view shrink to the creation of the plugin objects in `main.cpp` and the test main. The build without `experimental_src` does not compile today; check whether it compiles again, but it is not a goal of this plan.

### B: Shared libraries + exports (~2 wk)

Everything here is testable on the current Linux build before any plugin exists.

**Status (2026-11-11):** done and verified on Linux. Open: the AppImage check (needs the `compass/build_deb10` image rebuilt and `run_make_appimage.sh deb10 --clean`, since `build_deb10` caches the old `libjasterix.a`) and the Windows check (see below).

**Decisions (2026-11-11):**
- **Explicit `SHARED`** in every library's `add_library()`, no static fallback (a `MODULE` plugin on static archives would get duplicate statics from D on). `-Wl,--no-undefined` for all shared libraries in the top-level CMakeLists.
- **Windows toolchain is MinGW** (upstream cross build in Docker, `docker/win_build.sh`, since 2026-11-03), not MSVC as assumed below. MinGW exports all symbols of a DLL and auto-imports data symbols, so **B.5 adds no export annotations**; what remains of it is the export count check per DLL. A later MSVC build would need the class-level annotations described in B.5.
- **Third-party static archives in shared libraries need `-fPIC`.** jASTERIX (used by `compass_core` and `compass_task`) and NemaTode (used by `compass_task`) were non-PIC static archives. Both become **shared libraries**, so there is exactly one copy: jASTERIX with `-DBUILD_SHARED_LIBS=ON` (no source change; `compass_core` links it `PUBLIC`, the separate links in `compass_task`, client and tests go), NemaTode shared by default in the hpuhr/NemaTode fork (hard-coded `STATIC` replaced by `option(BUILD_SHARED_LIBS ... ON)`). Changed: `docker/build_jasterix.sh`, `docker/win_build.sh`, the NemaTode builds in `Dockerfile_deb10`, `Dockerfile_alma8` and `Dockerfile_win_deb12`.
- **Windows deploy:** `win_deploy.sh` copies the `compass_*.dll` files (debug information split into `<name>.dll.debug` as for `compass_client.exe`) and `libjasterix.dll`; `crunch_win_deploy.sh` copies all `*.debug` files.
- **`compass_plugin_sdk`** is defined in [src/app/CMakeLists.txt](src/app/CMakeLists.txt) (core, reconstruction, gui, view, result, task, app); `compass_reconst_adv` and `compass_geo_view` already link it instead of the individual host libraries.
- `compass_app` is installed with the other libraries (it was missing from `COMPASS_INSTALL_LIBS`).
- **B.4 measured (2026-11-11):** ProbIMM reconstruction of `at_20230422_05h` (1643 targets in both), static build 4:25, shared build 4:24, i.e. no measurable cost; `-fno-semantic-interposition` is not needed.
- **Verified on Linux (2026-11-11):** build with `--no-undefined`, all unit tests, layering check; `Logger::getInstance()`, `RTCommandRegistry::instance()` and `logger::debug_enabled` are defined once (in `compass_util` / `compass_core`), and no data symbol is defined in more than one binary. Crash backtraces resolve names in the libraries (`--segfault`: `libcompass_app.so(Client::Client(...)+0x2d76)`); `ENABLE_EXPORTS` on `compass_client` stays for the executable's own frames (`main`, signal handler).
- **Open:** the Windows cross build (DLL links, export counts per DLL, `win_deploy.sh`, `inline` namespace variables from B.5) is not verified yet: the `compass/build_win_deb12` image is not on the development machine; it is checked later on a machine with the image.

1. *Done 2026-11-11 (see Decisions; the link `compass_app` → `compass_geo_view` no longer exists after A).* **One shared library per module.** Every `compass_*` library in `src/` and the host-side libraries in [experimental_src/CMakeLists.txt](experimental_src/CMakeLists.txt) become `SHARED`, with `POSITION_INDEPENDENT_CODE ON` and `-Wl,--no-undefined` on Linux.
   - Each library links its real dependencies explicitly. Between the host libraries in `src/` this is already the case: the dependency plan made the link graph follow the layering, and the symbol check of 2026-11-04 found no missing host-to-host link. What remains are the undeclared links to experimental libraries listed in Workstream A (`compass_result` → `compass_docx`, `compass_app` → `compass_geo_view`); the app ↔ `compass_rtcmd_ext` / `compass_test_lab` cycles are gone after A.
   - Add the `compass_plugin_sdk` INTERFACE target.
2. *Done 2026-11-11, backtraces verified (see Decisions).* **Client and tests.** `compass_client` is already only `main.cpp` + `cxa_throw_hook.cpp`. Link it against the host libraries; the repeated-archive link lines here and in [src/unit_tests/CMakeLists.txt](src/unit_tests/CMakeLists.txt) were already removed by the dependency plan. Check that crash backtraces still resolve names once `ENABLE_EXPORTS` is no longer what exposes the symbols (they now live in the libraries' dynamic symbol tables).
3. *Done 2026-11-11 (`logger.cpp`, new `rtcommand_registry.cpp`).* **Header-inline singletons.** Move `Logger::getInstance()` ([logger.h:189](src/util/logger.h#L189)) and `RTCommandRegistry::instance()` ([rtcommand_registry.h:51](src/core/rtcommand/rtcommand_registry.h#L51)) into `.cpp` files. Function-local statics in header-defined functions are merged across modules on Linux but **duplicated per DLL on Windows**. ProbIMM uses neither directly (the logging macros go through the out-of-line `logger::rootCategory()`), so this is cheap prevention; it becomes essential in phase 2, when the geo plugin registers RT commands. The real logging hazard is the data symbol in item 5.
4. *Done 2026-11-11: no measurable cost, `-fno-semantic-interposition` not needed (see Decisions).* **Performance check.** Shared libraries with `-fPIC` and default visibility can cost runtime in hot code (reconstruction, Kalman, evaluation). Build with `-fno-semantic-interposition` and compare a reconstruction run's wall time against the static build.
5. *Not applicable with MinGW (see Decisions): no export macros, no annotations; the text below is kept for a later MSVC build. Still open for Windows: the export counts (Linux estimate: `compass_core` exports 10,289 symbols, the most of all libraries, far below 65535) and the `inline` namespace variables (last paragraph).* **Symbol export.** Generate one export macro per library via `generate_export_header` (`COMPASS_UTIL_API`, `COMPASS_CORE_API`, `COMPASS_VIEW_API`, …; empty on Linux) and enable `WINDOWS_EXPORT_ALL_SYMBOLS` on every host library.

   Auto-export covers **functions**. MSVC additionally requires `dllimport` on the consumer side for every **data symbol** a plugin references, otherwise it is an unresolved external at link time. Data symbols arise from:
   - **Deriving** from a host class: its `staticMetaObject` (moc superclass link, `qobject_cast`) and potentially its vtable.
   - **`connect()` with member function pointers** to a host signal: it references the sender's `staticMetaObject`. This applies to classes the plugin merely uses.
   - **`static const` members and globals** that are odr-used.

   The **rule**: annotate class-level, with the macro of the class's own library, every `Q_OBJECT` class and every class with static data members in the host, plus `logger::debug_enabled`. This is a mechanical, announced bulk edit, safer than chasing a hand-picked list, and it already covers phase 2. The verified minimum it must cover:

   | Kind | ProbIMM (phase 1) | Geo view (phase 2) |
   |---|---|---|
   | Bases | `ReconstructorBase`, `ReconstructorBaseSettings`, `ReconstructorAssociatorBase`, `AccuracyEstimatorBase`, `ReconstructorMainWidget`, `ReferenceCalculatorWidget`, `KalmanInterface`, `KalmanFilter`, `KalmanFilterLinear` | `View`, `ViewWidget`, `ViewDataWidget`, `TabStyleViewConfigWidget` (→ `ViewConfigWidget`, `ViewComponent`), `ViewLayerTreeModel` (→ `LayerTreeModel`), `LayerTreeItem`, `DBContentItemModel`, `DBContentItemProvider`, `DBContentLeafPayload`, `Configurable`, `ui_test::UITestable`, `rtcommand::RTCommand`, `GeoImageSink` |
   | Signals connected to | `context::DBContextManager` | `AppState`, `ViewManager`, `DBContentManager`, `context::DBContextManager`, `ViewAsyncProcessor`, `dbContent::LabelGenerator` |
   | Static data | `ReconstructorTarget::global_stats_` (written by the plugin), `TargetBase::emitter_specs_`, `KalmanEstimator::HighVar`, `DataSourceBase::*Key`, `targetReport::AccuracyTables`, `ViewPointKeys::VP_*` (in `compass_core` since the dependency plan) | `ViewPointKeys::VP_*`, `ViewPointGen*` field names, `DataSourceBase::*Key`, `DBContentItemProvider::DefaultGrouping`, `Configuration::ParameterSection`, `LabelGenerator` constants |
   | Global | `logger::debug_enabled` ([logger.h:47](src/util/logger.h#L47), read by every `logdbg`), `APP_FILENAME` ([global.cpp](src/util/global.cpp), defined in `compass_util` since the dependency plan) | same |

   C++17 `inline const` namespace variables (`dbcontent_vars::meta_var_*` in [idbvariableresolver.h:126](src/core/base/idbvariableresolver.h#L126)) get one copy per DLL on Windows. That is harmless as long as nothing compares their addresses; verify this.

   Measure the generated export count per library: `.def` files cap at 65535 ordinals per DLL. With one library per module this is unlikely to be hit; check the largest ones (`compass_core`, `compass_task`, `compass_eval`).

   Rejected alternatives: a **narrow C-style ABI** with host-side adapters (forwarding shims for every virtual, for ABI stability nobody needs) and **MinGW** (auto-imports data, but contradicts the MSVC/vcpkg decision of the port plan).

### C: Plugin infrastructure (~3 d)

In `compass_app` (see Extension points, "Location"):
- `src/app/plugin/plugin.h` (interface from Workstream A): add `COMPASS_PLUGIN_API_VERSION` (a plain build-consistency check, no negotiation) and the entry-symbol declaration.
- `src/app/plugin/pluginmanager.{h,cpp}`: load the expected plugins by name, startup presence check (Workstream E), log a startup summary, return the plugin list. Called from `main.cpp` instead of the static creation of the plugin objects; the call points of the init steps stay as in Workstream A. Check where in `main` the logger is available for the error messages.
- **Loading via `QLibrary`**, not hand-written `dlopen`/`LoadLibrary` wrappers. Qt is already a dependency, so this keeps the port plan's "no `dlopen`" property and adds no platform seam.
  - Use `ResolveAllSymbolsHint` (`RTLD_NOW`) so a missing symbol fails at startup.
  - Keep the default local symbol scope: plugins only need the host libraries, not each other.
  - **Never unload.** Qt metaobjects and osgEarth statics do not survive it.
- Each plugin is its directory `experimental_src/plugins/<name>/` with all of its sources (see Workstream A, "Plugin location"); its `Plugin` subclass gets the entry symbol there. `experimental_src` remains its own repository.

### D: ProbIMM plugin (~0.5 wk)

`compass_reconst_adv` becomes the `MODULE` library `compass_plugin_probimm`, together with the three test lab tests that use its types. `experimental_src/util/` contributes only an include directory; `extra_resources.{h,cpp}` is compiled into `compass_core` ([src/core/CMakeLists.txt:38-50](src/core/CMakeLists.txt#L38)) for `License::verify()` and stays there.

- **Entry point.** `ProbIMMPlugin` from Workstream A (already in `experimental_src/plugins/probimm/`) gets the entry symbol (ProbIMM registers no runtime commands today, so `initCommands()` stays empty). `main.cpp`'s static creation of `ProbIMMPlugin` goes.
- **`friend class KalmanFilterIMM`** at [kalman_filter.h:181](src/reconstruction/kalman_filter.h#L181) survives the split (friendship is compile-time); the class-level export on `KalmanFilter` makes the befriended members reachable.
- **JPDA moves cleanly.** The JPDA references in the host ([reconstructortask.h:77,91](src/task/reconstructor/reconstructortask.h#L77), [reconstructortarget.h](src/task/reconstructor/reconstructortarget.h), [reconstructorbase.cpp:1811](src/task/reconstructor/reconstructorbase.cpp#L1811), [datasourceeditwidget.cpp:227](src/core/source/datasourceeditwidget.cpp#L227)) are debug flags, statistics counters and labels. The plugin fills the counters through `ReconstructorTarget::global_stats_`, which is why `ReconstructorTarget` is exported.
- **Test lab.** `compass_test_lab` stays in the host. Three of its 42 tests construct plugin types directly, and the plugin being loaded at runtime does not help there: code naming those types must be linked against them, and on Windows a host DLL must resolve every symbol at link time. These three move into the plugin:
  - Unscented Kalman Test (`unscentedKalmanTest`, [test_lab_phil.cpp:1043](experimental_src/test/test_lab_phil.cpp#L1043): `KalmanFilterUnscented`, `MerweScaledSigmaPoints`)
  - IMM Test (`immTest`, [:1966](experimental_src/test/test_lab_phil.cpp#L1966), runs `KalmanTest` from [kalman_test.cpp](experimental_src/test/kalman_test.cpp): AM/UM/ZM/IMM filters, `dynamic_cast<KalmanFilterIMM*>`)
  - JPDA Test (`jpdaTest`, [:2946](experimental_src/test/test_lab_phil.cpp#L2946): `JPDAFull`, `JPDASettings`, `JPDATentativeTarget`, `JPDAResult`)

  They move into their own source files in the plugin directory (e.g. `experimental_src/plugins/probimm/test/`, decided 2026-11-10: tests move into their plugin, but in their own source files, not merged into existing plugin sources), together with `kalman_test.{h,cpp}`. The plugin contributes them to the host's test lab menu through `TestLabRegistry` (possibly the same hook as the app ↔ `compass_test_lab` decision in Workstream A). `test_target.{h,cpp}` is plugin-free and in the host (`compass_reconstruction` since 2026-11-10), where the plugin's tests can use it. The remaining Kalman/JPDA includes in `test_lab_phil.cpp` go, and so does `compass_test_lab`'s link to `compass_reconst_adv`.
- **Link dependencies.** The plugin calls `osgEarth::GeoMath` (jpda, associator, accuracy estimators) and uses `compass_view` classes (`Grid2DLayerRenderer`, `ColorMap`, `ScatterSeries`). On Windows the plugin DLL must link osgEarth itself; declare it.

### E: Startup presence check (~1 d)

All plugins are always present, so presence is checked exactly once, in `PluginManager` at startup, before the config tree is instantiated. Nothing else in the host checks for it.

- **Expected plugins come from the build.** CMake generates the list of plugin targets built in this configuration into a header (e.g. `COMPASS_EXPECTED_PLUGINS`), together with the registry entries each plugin must provide (ProbIMM: `ProbIMMReconstructor` in the reconstructor factory (its creator also creates `ComplexAccuracyEstimator`; the Kalman types and the widget come with the reconstructor's overrides, no entry), the three test lab entries). `PluginManager` loads exactly these files by name and does not scan for other files.
- **Asserted steps**, each with `traced_assert_msg` and a message naming the plugin and the plugin directory. Before the assert, the details go to `logerr`, e.g. `QLibrary::errorString()`:
  1. the plugin file exists;
  2. `QLibrary::load()` succeeds;
  3. the entry symbol `compass_plugin_create` resolves;
  4. `COMPASS_PLUGIN_API_VERSION` and `COMPASS_BUILD_ID` match (Workstream F);
  5. `name()` matches the expected name;
  6. after each init step (`initCore()`, `initCommands()`, `initUI()`), every expected entry of that step exists.
- **Consequences for the rest of the host:**
  - no presence query, no `Availability` enum, no UI gating, no fallback paths;
  - `createSubConfigurables()` ([configurable.cpp:298](src/config/configurable.cpp#L298)) needs no "known class, provider unavailable" outcome: `ProbIMMReconstructor0` in `task_reconstructor.json` always finds its class;
  - config preservation for absent plugins is not needed (for the record: `sub_config_storage_` would keep such sub-configs anyway);
  - registry lookups of expected entries assert instead of returning null.
- **Licence checks unchanged.** `checkReconstructor()`, the selector entry, the MLAT/ADS-B/SMR inspectors (via `AnalyzeDataSourceTask::professionalLicenseEnabled()`, [analyzedatasourcetask.cpp:793](src/task/analyze/analyzedatasourcetask.cpp#L793)) and the DocX export ([taskresultswidget.cpp:125](src/task/taskresultswidget.cpp#L125)) keep checking the `ComponentProbIMMReconstructor` licence exactly as today.
- **A failed check aborts the client.** `compass_handler` reports the crash (message box or relayed exit code, [watchdog/main.cpp:74-94](src/app/watchdog/main.cpp#L74)) and does not restart the client, so there is no restart loop.
- **Startup summary** in the log: the loaded plugins, with name, file and build ID.
- **Builds without `experimental_src`.** The ProbIMM plugin is not built and the expected list is empty. After Workstream A, the only code referencing experimental components is the creation of the plugin objects in `main.cpp` (until C) and the test main, compiled only with `experimental_src`. This is not a runtime case.

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

Build the plugin's sources once as an `OBJECT` library consumed by both the `MODULE` target and `compass_tests`, which replaces `compass_tests`' link to `compass_reconst_adv`. Tests that construct plugin types directly (`test_jpda`, `test_accuracy_*`, `test_adsb_quality`, `test_dubious_references`) link normally. Tests that go **through the registry** keep creating the plugin object and calling `initCore()` in the test main from Workstream A; no module is loaded. `compass_tests` does not run the startup presence check.

### Phase 1 verification

1. **Workstreams A and B are a functional no-op on Linux.**
   - `cmake --build build && ./build/bin/compass_tests` passes unchanged;
   - `docker/run_make_appimage.sh` produces a working AppImage with the host libraries bundled, and the CI integration tests pass on it;
   - crash backtraces still resolve names;
   - reconstruction wall time is within noise of the static build;
   - the geo view still renders, grid export lands in it, and LaTeX/PDF export contains its screenshots including the overview.
2. **Single instances across modules.** Log the address of `Logger`, `RTCommandRegistry` and `logger::debug_enabled` from host and plugin; they must match. Toggling the debug log level must affect plugin `logdbg` output.
3. **Functional parity.** A ProbIMM reconstruction against a reference recording produces **identical output to the pre-split build** (same target count, same reference trajectories). This catches a broken `createKalmanInterface()` override. The three plugin tests appear in the test lab menu and run.
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

Optional follow-up. Since 2026-11-10 the geo view is already a plugin object (`GeoViewPlugin`, statically linked) in `experimental_src/plugins/geoview/`; here `compass_geo_view` leaves the host and becomes the `MODULE` library `compass_plugin_geoview`. Its host sites already go through registries (Workstream A), so this phase only turns the library into a plugin. Line numbers below are from 2026-11-09 and must be re-checked when phase 2 starts.

- ~~**`Plugin::shutdown()`** replaces the root's direct `geo_view::shutdown()` call; `geo_view::clearHiddenLayers()` goes~~: done in Workstream A (2026-11-10, `GeoViewPlugin::shutdown()`).
- **Entry point.** `GeoViewPlugin` (Workstream A, already in `experimental_src/plugins/geoview/`) gets the entry symbol. `main.cpp`'s static creation of it goes.
- **Startup check.** Add the geo plugin and its expected registry entries (`GeographicView` view class, its RT commands, its lifecycle hooks, its `GeoImageSink` implementation) to the expected list. No runtime presence checks: add-view menu, `GeographicView0` in `views.json`, grid view export and LaTeX overview rely on the plugin being there.
- **CMake.** `compass_geo_view` becomes a `MODULE` and links the host through `compass_plugin_sdk`.
- **Tests.** The geo unit tests (overlay, labels, raster, rec num index, added 2026-10) construct geo types directly and link the plugin's `OBJECT` library like the ProbIMM tests.
- **Test lab.** `osgEarth1MPointsTest` ([test_lab_phil.cpp:4016](experimental_src/test/test_lab_phil.cpp#L4016), the only test lab user of `ViewerWidget`) moves into its own source file in the plugin directory (e.g. `experimental_src/plugins/geoview/test/`), contributed to the test lab menu like the ProbIMM tests (D); `compass_test_lab`'s link to `compass_geo_view` goes. `ViewerWidget` / `GraphicsWindowQt` stay in the plugin. `test_offscreen_map.cpp` (disabled) goes with the plugin if it is re-enabled.
- **AppImage.** No new mechanism: the plugin lands in `usr/lib/compass_plugins/` and `--deploy-deps-only` picks up its dependencies. Add it to the expected files in `check_appimage.sh`. The OSG plugins, `libosgEarth*` and `GDAL_DATA` handling stay host-side, since the host uses osgEarth itself.

**Verification.** Map renders, overlays and labels draw, grid export lands in the geographic view, LaTeX/PDF export contains the geographic screenshots including the overview, `set_map` registered by the plugin is reachable over `localhost:27960`. The startup check fires for a missing geo plugin as in phase 1.

---

## Relationship to the other plans

**Dependency plan.** Implemented 2026-11-04 to 2026-11-06 (K.2 still planned): the `src/` libraries are acyclic in includes and links, and their include paths enforce the layering. This plan relies on that for Workstream B, on its composition root (`CompassRoot`) for registering and loading components before the config tree exists, and on the K.3 command registration for the command step.

**Windows port.** Phase 1 of this plan is platform-neutral and is done and verified on Linux first.

| Port phase | Absorbs |
|---|---|
| Phase 1 (CMake modernisation) | Workstream B: shared libraries, export macros, dropping `ENABLE_EXPORTS` |
| Phase 3 (platform abstraction) | plugin search path resolution next to the `appDir()` rework (loading itself is `QLibrary`, no new seam) |
| Phase 7 (packaging) | the `plugins/` directory in both the AppImage appdir and the Windows ZIP |

Doing the plugin split Linux-only first and retrofitting Windows later would mean discarding the linking design. Planning for both now costs roughly 1–1.5 weeks extra, about half of which Phase 1 of the port does anyway.

**Compute core.** Follows this plan in the near future. This plan prepares it through the per-library export macros, the SDK interface target and the phased registration.

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
| A: host sites → registries (ProbIMM + geo view), incl. sub-configurable factories and `ReconstructorTask` remodel | 3 wk | 4 wk |
| B: shared libraries + exports | 2 wk | 3 wk |
| C: plugin infrastructure | 3 d | 1 wk |
| D: ProbIMM plugin + test lab entries | 0.5 wk | 1 wk |
| E: startup presence check | 1 d | 2 d |
| F: AppImage packaging + tests | 1 wk | 1.5 wk |
| **Phase 1 total** | **~7.5 wk** | **~11 wk** |
| G: Geographic View (phase 2) | 1 wk | 1.5 wk |

Workstream B carries the full export rule, so phase 2 needs no new annotation pass, only fixes for stragglers. Almost all of it is well-specified refactoring against a compiler that reports mistakes immediately.

**Milestone to track:** the AppImage running on Linux with ProbIMM loaded as a plugin, `compass_tests` and the CI integration tests green, around week 7.
