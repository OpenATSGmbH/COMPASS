# DBContent Loading

Describes how surveillance data is read from the database into the in-memory `Buffer`s that
views and tasks operate on, both offline (one-shot loads) and live (a rolling in-memory
feed). The design separates three responsibilities:

- **`DBContentDataEngine`** — the single door to the DB connection for DBContent data: it
  runs every load, insert and delete (one `DBContentReadDBJob` per `DBContent`, serialised
  behind the single DB connection in the JobManager). It owns the running load and the
  single-operation **safety net**.
- **`DBContentManager`** — the *static model* (definitions, schema/variables, metadata,
  targets, data-source model, min/max). It owns **no dataset** and does **no** distribution.
  Its `load()` is a two-line pass-through; it forwards insert/delete to the engine.
- **`ViewManager`** — owns the *displayed* dataset (`current_source_`), the distribution
  dispatch, and the load/live lifecycle bookends. It delegates the two lifecycles to two
  collaborators it owns: **`LoadController`** (offline load UX) and **`LiveController`**
  (the live session).

## The load object

A load is a **`LoadOperation`** ([loadoperation.h](loadoperation.h)) — a `DBContentDataSet`
subclass that carries a `LoadSpec` (the immutable `LoadRequest`), a state machine
(`Created/Running/Done/Cancelled/Failed`), `cancel()`, an event-pumping `wait()`, and the
loaded buffers + a lazily-built index/accessor. It emits its own `startedSignal` /
`finishedSignal` and, being a data set, one `dataChangedSignal(names, reset, last)`.

`LoadRequest` ([loadrequest.h](loadrequest.h)) is the spec used to build one:

| Field | Default | Meaning |
|---|---|---|
| `dbcontents_` | `{}` | `{"*"}` = all eligible (`loadable() && loadingWanted()`); explicit names = exactly those (gated by `loadable()`); empty = no-op load |
| `apply_datasrc_filters_` | `true` | Apply DBContext ds/line filters to WHERE |
| `apply_view_filters_` | `true` | Apply FilterManager conditions to WHERE (and let filters add required vars to the read set) |
| `custom_filter_clause_` | unset | Per-content `std::function<std::string(name)>` AND-ed after the filter clauses; built via the clause toolkit (see *WHERE composition*) |
| `read_set_` | unset | Per-content `std::function<VariableSet(name)>`; defaults to `mgr.getReadSet(name)` |
| `show_status_` | `true` | The issuer's progress UX applies (view loads only; see *LoadController*). Batch loads set `false` |
| `cancellable_` | `true` | The progress dialog gets a Cancel button (→ `quitLoading()`) |
| `measure_db_performance_` | `false` | Enables `DBInterface` perf metrics for this load |

Factories: `LoadRequest::standard()` (fan-out, all filters on), `LoadRequest::withFilter(clause)`
(fan-out + uniform raw-string clause — an escape hatch), `LoadRequest::forContent(name, rs, clause="")`
(single content, no filters). A default `LoadRequest{}` loads nothing — the `{"*"}` sentinel must
be explicit, so accidental empty-set bugs are benign. `LoadRequest::perContentClause(...)` (a
precomputed `map`, or a `(contents, generator)` pair that renders one up front) builds a
`custom_filter_clause_` function without the caller hand-rolling the per-content lookup.

## Who issues loads

- **View loads** — `ViewManager::reload()` builds a `LoadRequest::standard()`, wraps it in a
  `LoadOperation`, makes it `current_source_`, and runs it through `DBContentManager::load(op, blocking)`.
- **Batch consumers** (Evaluation, Reconstructor, ARTAS, RadarPlot, RT `get_data`,
  AnalysisDataset) — build their own `LoadOperation` and call **`dataEngine().load(op)`
  directly**, then read `op->buffers()`. They never touch `current_source_`, so a batch load
  raises **no** view/UI chrome and can't clobber the displayed dataset.

```cpp
void DBContentManager::load(std::shared_ptr<LoadOperation> op, bool blocking = false);
// pure wrapper: engine.load(op); if (blocking) op->wait();
```

The **single-operation safety net** lives in `DBContentDataEngine::load()`: if a load is still
running it waits (pumping events, `logwrn`) rather than asserting or quitting — a consumer may
be awaiting the result. Because it's in the engine, *every* path (view + batch) gets it.

## Load flow (offline)

1. `reload()` — `captureSelection()` (for carry-over), `clearDataInViews()`, build the op,
   `setCurrentSource(op)`, `dbContentManager().load(op, blocking)`.
   `setCurrentSource` subscribes ViewManager to the op's `dataChangedSignal`
   (`sourceDataChangedSlot`) and — for a `LoadOperation` — to its `startedSignal`/`finishedSignal`
   (`loadingStartedSlot`/`loadingDoneSlot`).
2. `DBContentDataEngine::load(op)` — safety-net wait, `setState(Running)` (→ `startedSignal`),
   `resolveTargetSet`, then per content: resolve read set (`spec.read_set_` or
   `mgr.getReadSet`), `composeWhereClause` (ds/line filter + FilterManager conditions +
   `custom_filter_clause_`), augment with the core meta-vars (`rec_num`/`ds_id`/`line_id`/`timestamp`),
   create a `DBContentReadDBJob`, submit via `jobManager()`.
3. **Per arrival** (`readJobDoneSlot`, GUI thread) — `transformVariables` (DB→var names) +
   add `selected_`, `op->setBuffer(name)`, `op->emitChanged({name}, reset=false, last=false)`.
   Empty contents don't emit.
4. **Finalize** (`finish()`) — `op->emitChanged({}, reset=false, last=true)`, then
   `setState(Done|Cancelled)` (→ `finishedSignal`), perf-metrics stop.
5. `sourceDataChangedSlot` distributes each event (see *Distribution*); `loadingDoneSlot`
   runs the completion (view finalize + bookend). `loadingDone(DBContent&)` and the old
   `DBContent::loadInternal`/`readJobDoneSlot` path are dead (the engine owns its own read path).

`load(op, /*blocking=*/true)` calls `op->wait()` (event-pumping) — used by viewpoint apply and
the live resume reload.

## WHERE composition (filter-clause library)

`DBContentDataEngine::composeWhereClause(name, spec, read_set)` assembles each content's WHERE
from three `FilterClause` parts, `combineAnd`-ed:

1. **datasource/line** — from `DBContextManager::loadingSelection(name)`, the single source for
   both the offline SQL here and the live in-memory `filterDataSources`. An empty selection
   emits the `1=0` no-row sentinel.
2. **view filters** — `FilterManager::viewClause(name)` when `apply_view_filters_` (each active
   filter's `getClause`, `combineAnd`-ed), replacing the old side-effecting `getSQLCondition`.
3. **custom clause** — the issuer's `custom_filter_clause_(name)`.

A `FilterClause` is `{ std::string sql; VariableSet required_vars; }`; the engine unions
`required_vars` into the read set, so a column that is filtered-on but not otherwise read gets
added automatically.

Clauses are built with the shared toolkit in [filterclause.h](../../filter/filterclause.h):
- `DBFilterCondition::sqlFor(resolver, content, variable, var_dbcontent, op, value…)` renders one
  leaf condition — any operator via the `filter_op::` tokens (the single home for `=`/`>=`/`IN`/
  `BETWEEN`/`NULL`/… strings) — and **self-guards** via the static `variableResolvable(...)`,
  returning an empty clause when the variable is absent for that content (so callers need no
  per-content `metaCanGetVariable` guard).
- per-filter `sqlFor` statics (`TimestampFilter::sqlFor`, `UTNFilter::sqlFor`, Mode3A/ACAD/…)
  render the specialised conditions; `combineAnd`/`combineOr` join them (unioning required vars).

Batch consumers build their `custom_filter_clause_` this way instead of hand-concatenating SQL
or mutating the global FilterManager: eval (ROI bbox + UTN set + timestamp bounds — the hijack
is gone, the user's filters are untouched), reconstructor (per-slice half-open timestamp + sector
bbox), ARTAS (CAT062 ds/line `IN`), live prime (`timestamp >=`). `LoadRequest::perContentClause`
wraps a per-content generator into the clause function.

## Bookends (ViewManager)

`loadingStartedSignal()` / `loadingDoneSignal()` live on **ViewManager**, not the manager —
the manager's loads may be issuer-private batch loads that must not drive view/UI chrome.
They are emitted from `loadingStartedSlot` / `loadingDoneSlot`, which fire off the operation's
own `startedSignal`/`finishedSignal` (wired in `setCurrentSource`). Consumers connect to
ViewManager: MainWindow chrome, `TargetListWidget` focus-restore, and the RT wait
`compass.viewmanager.loadingDoneSignal()`. The live session synthesises one `started` on
entry and one `done` on exit (see *Live mode*), so it reads as a single
`started → tick × N → done` cycle.

## LoadController — the offline load UX

`ViewManager` owns a **`LoadController`** ([loadcontroller.h](../../view/loadcontroller.h))
that holds the modal `QProgressDialog`, the wait cursor, and the two-phase progress (load
0..50% / view 50..100%). ViewManager drives it at its dispatch points:

- `begin(op)` — from `loadingStartedSlot` (i.e. off the op's `startedSignal`, **after** the
  engine's safety-net wait). Idempotent: it `end()`s any prior cycle first, so a
  superseded/overlapping load can't leak a dialog or unbalance the cursor. Sized via
  `resolveTargetSet(op.spec())`. Skips the dialog when `spec.show_status_` is false or there's
  nothing to load (wait cursor only).
- **Load-phase progress** is driven **directly off the op's `dataChangedSignal`**
  (`opDataChangedSlot`, connected in `begin`/disconnected in `end`) — *not* off ViewManager's
  re-entrancy-deferrable `sourceDataChangedSlot`. A deferred arrival re-running the advance
  used to yank the bar back to 50%.
- `beginViewPhase(n)` / `advanceViewPhase()` — in `loadingDoneSlot`'s per-view loop.
- `end()` — closes the dialog + restores the cursor. Called from `loadingDoneSlot`
  **outside** the `processing_data_` guard, so a deferred `sourceDataChangedSlot` (e.g. the
  geo view's redraw) drains while the dialog is still up — matching the pre-refactor
  `finishLoading` ordering (no late geo update after "done").

## Distribution

The displayed dataset is the **current source** — a `DBContentDataSet` (`LoadOperation`
offline, `LiveDataFeed` live) owned by `ViewManager` as `current_source_`. It owns the buffer
map plus a lazily-built `DBContentDataIndex` (`dbc_id → ds_id → line_id → buffer indices`) and
accessor, and emits **one** change signal:

| Signal | Payload | Meaning |
|---|---|---|
| `dataChangedSignal(names, reset, last)` | `vector<string>`, `bool`, `bool` | Buffers/indices for `names` changed. `reset` → drop state for contents NOT in `names` (full replacement). `last` → final event of a batch; listeners run finalize work. The index is rebuilt/consistent *before* the signal fires. |

**`ViewManager` is the sole subscriber** (`sourceDataChangedSlot`). Per registered view it
drives, in one guarded turn, the single callback `View::updateFromSource(source, names, reset,
last)` (the old dual `loadedData`/`updateData` path is gone — step 8). The base
`ViewDataWidget::updateFromSource` mirrors `data_ = source.buffers()` (so the existing
`viewData()`/redraw/selection machinery is undisturbed), feeds a base-owned
`DBContentItemProvider` if the view has one, and calls the view's `updateFromSource_impl`:
- **Standard views** (table/scatter/grid) read `source.buffers()`; histogram (via
  `VariableViewDataWidget`) does a full refresh on `last`. None own a provider.
- **Geographic View** owns a `GeometryItemProvider`, fed here: `setSource(&source)` +
  `applyChange(names, reset, last)` → `resetData()` (if `reset`) → `rebuildContent(id)` per name
  → `contentRebuilt()` (if `last`), reading `source.index()`/`source.buffers()`. So OSG never
  paints the empty intermediate state between wipe and rebuild. `GeographicView::updateFromSource`
  overrides to skip the **whole** update (provider + finalize) on live overload.

Also in `sourceDataChangedSlot`: selection carry-over (`applyCarriedSelection`), data-source
loaded counts (`dbContextManager().setLoadedCounts`), and — on `last` — `dataDistributedSignal`
(the data-sources status widget refreshes off it).

### Re-entrancy guard + threshold event-pumping

`loadingStartedSlot` / `sourceDataChangedSlot` / `loadingDoneSlot` share a single
`processing_data_` flag (`QScopedValueRollback` for the per-view loop). If any is invoked while
the flag is set, it re-posts itself via `Qt::QueuedConnection` and returns — preserving the
`started → changed* → done` ordering under event-loop pumping.

`loadingDoneSlot` is the one place a load can take long enough for the window manager to flag
the app unresponsive. After each `view->loadingDone()` it calls `processEvents(ExcludeUserInput
| ExcludeSocketNotifiers)` — but only **after the loop has run longer than `pump_threshold_ms`
(3 s)**. Pumping unconditionally lets queued RT commands (posted from the asio runner thread,
which bypass `QSocketNotifier`) interleave with view dispatch and break UI-test signal
injection; the threshold keeps short loads (typical UI tests) below the pump line entirely.
The `LoadController` progress helpers use `dialog_->repaint()` (a synchronous widget paint),
**not** `processEvents()`, for the same reason. Treat unconditional `processEvents()` inside the
load lifecycle as suspect; prefer `repaint()` on the specific widget.

## Live mode (`LiveController`)

`ViewManager` owns a **`LiveController`** ([livecontroller.h](../../view/livecontroller.h))
that holds the `LiveDataFeed`, subscribes to the engine's `insertedDataSignal`, and runs the
per-tick orchestration. The feed and both controllers are **private** — the ASTERIX watchdog
fires `ViewManager::forceLiveUpdate()` and the latency façade reads `ViewManager::hasMaxLatency()/maxLatency()`.

**Key fact:** pause does **not** stop ingestion. The ASTERIX decode/insert pipeline keeps
running and the DB keeps accumulating; the display freezes only because the engine emits
`insertedDataSignal` **only in `LiveRunning`** (in `LivePaused` it inserts to the DB and
clears `insert_data_` without announcing). So the feed freezes while the DB grows.

Transitions (`ViewManager::appModeSwitchSlot`, driven by `COMPASS::appModeSwitchSignal`;
`compass.cpp::appMode()` no longer special-cases pause/resume — it just switches the importer
and emits):

| Transition | Action |
|---|---|
| **Fresh entry** (Offline→Live) | `current_source_ = feed` (empty → **blank**), one `started` bookend, `refreshDisplay()` (no-op) |
| **Pause** (Live→Paused) | nothing — the feed stays the source, frozen; ingestion continues into the DB |
| **Resume** (Paused→Live) | `reloadWindow()` (catch up), `current_source_ = feed`, `refreshDisplay()` |
| **Exit** (Live→Offline) | `current_source_ = null`, `clearFeed()`, one `done` bookend |

**Resume catch-up** — `LiveController::reloadWindow()` harvest-loads `timestamp >= now -
maxLiveDataAgeCache` from the DB (a private blocking `LoadOperation`, never the display source,
so no bookend/dialog) and `seedFrom`-**replaces** the frozen feed with the current window
(picking up everything that accumulated while paused). It sets a `reloading_` flag around the
blocking load so a pump-fired tick can't issue an overlapping `deleteOlderThan` or lose a
concurrent insert to the `seedFrom` (inserts still stage and merge on the next tick).
`refreshDisplay()` then runs `processTick()` (feed `cutCachedData` + distribute) **without** a
DB delete — the every-tick DB bound resumes on the next real tick, so nothing overlaps here.

**The per-tick** (`processLiveModeSlot`, from `insertedDataSlot` + the watchdog): guard on
`LiveRunning` (and not `reloading_`), `deleteDBContentData(now - maxLiveDataAgeDb)` (the DB
bound — runs every tick, unchanged), then `processTick()`. `LiveDataFeed::processTick()` merges
staged inserts, `cutCachedData` (trims the display to `maxLiveDataAgeCache`), filters, updates
latency, and emits **one** atomic `dataChangedSignal(all, reset=true, last=true)` →
`sourceDataChangedSlot`. The single queued event means the geo provider wipes + rebuilds +
finalizes in one event-loop turn (no empty intermediate OSG paint). The **display window** is
bound by the feed's `cutCachedData`, independent of the DB delete.

### Layer churn in the per-tick rebuild

The feed's per-tick wipe + rebuild is atomic, but `GeometryItemProvider::reset_impl()` still
destroys every `GeometryItemGroupLayer` and rebuilds a new one — each label is torn down and
re-registered per tick (visible label flicker). The deeper fix is to keep existing
`(dbc, ds, line)` layers alive across ticks and refresh buffer/index references in place,
destroying only vanished keys.

## Migrating away from per-content loads

`DBContent::loadInternal` is dead (the engine owns the read path). Tasks that previously called
`DBContent::load(...)` directly build a `LoadOperation` and call `dataEngine().load(op)`:

- **Time-sliced (Reconstructor).** One op per slice with `dbcontents_` explicit, `apply_*_filters_=false`,
  `read_set_`/`custom_filter_clause_` lambdas, `show_status_=false`; iterate by connecting the
  op's `finishedSignal` and issuing the next slice.
- **Single content (RT `get_data`).** `LoadRequest::forContent(name, rs, clause)`, `show_status_=false`;
  `op->wait()` synchronously, with an `isLoading()` guard (RT bypasses UI modality).
- **Restricted fan-out (ARTAS, RadarPlot).** Explicit `dbcontents_`, `apply_*_filters_=false`,
  `read_set_` lambda, optional `custom_filter_clause_`.
