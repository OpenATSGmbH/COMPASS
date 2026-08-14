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
  It has no `load()` at all — issuers go to the engine directly; it forwards insert/delete.
- **`ViewManager`** — owns the *displayed* dataset (`current_source_`), the distribution
  dispatch, and the load/live lifecycle bookends. It delegates the two lifecycles to two
  collaborators it owns: **`LoadController`** (offline load UX) and **`LiveController`**
  (the live session).

## The load object

A load is a **`LoadOperation`** ([loadoperation.h](loadoperation.h)) — a `DBContentDataSet`
subclass that carries a `LoadSpec` (an alias of `LoadRequest`), a state machine
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
| `datasrc_selection_` | unset | Per-op ds/line selection (`ds_id → lines`); replaces the DBContext selection for this load only, so a batch consumer never mutates the user's |
| `measure_db_performance_` | `false` | Enables `DBInterface` perf metrics for this load |

Factories: `LoadRequest::standard()` (fan-out, all filters on), `LoadRequest::withFilter(clause)`
(fan-out + uniform raw-string clause — an escape hatch), `LoadRequest::forContent(name, rs, clause="")`
(single content, no filters). A default `LoadRequest{}` loads nothing — the `{"*"}` sentinel must
be explicit, so accidental empty-set bugs are benign. `LoadRequest::perContentClause(...)` (a
precomputed `map`, or a `(contents, generator)` pair that renders one up front) builds a
`custom_filter_clause_` function without the caller hand-rolling the per-content lookup.

## Who issues loads

- **View loads** — `ViewManager::reload()` builds a `LoadRequest::standard()` and hands it to
  `issueLoad()`, which wraps it in a `LoadOperation`, makes it `current_source_` and loads it.
- **Batch consumers** (Evaluation, Reconstructor, ARTAS, RadarPlot, RT `get_data`,
  AnalysisDataset) — build their own `LoadOperation` and call `dataEngine().load(op)`, then read
  `op->buffers()`. They never touch `current_source_`, so a batch load raises **no** view/UI
  chrome and can't clobber the displayed dataset.

There is **no manager `load` facade**: the engine is the public door for loads, since issuers
need the operation object anyway and `isLoading()`/`cancelLoad()`/`resolveTargetSet()` sit on the
same engine. Blocking issuers call `op->wait()`. (The manager keeps facades for insert/delete.)

The **single-operation safety net** lives in `DBContentDataEngine::load()`: if a load is still
running it waits (pumping events, `logwrn`) rather than asserting or quitting — a consumer may
be awaiting the result. Because it's in the engine, *every* path (view + batch) gets it.

## Concurrency & ordering rules

Only one DB operation runs at a time and the main thread pumps events while waiting, so ordering
is enforced by convention rather than by a queue. The rules the code follows:

1. **Only the engine waits for DB work.** Consumers pick one of: *refuse* (RT `get_data` and
   `delete_data` report busy), *skip* (the live tick drops its DB bound when a delete is in
   flight), *defer* (ASTERIX import postpones an insert while a load runs), or *ask the engine
   to wait* (`waitUntilEngineIdle()` — before closing the DB, applying a view point, or
   rendering a report figure). `waitUntilEngineIdle()` also waits out pending asynchronous view
   processing (`ViewManager::hasPendingViewProcessing()` - the geo view's geometry builds), as
   do `issueLoad()` before a new load and `COMPASS::shutdown()`. Don't spin on `isLoading()`
   privately. The exception is a consumer waiting on **its own** pipeline, of which engine
   state is one term - the reconstructor's drain loop - which still belongs to that consumer.
   Where a wait guards an operation rather than a call site, put it **on the operation**:
   `setViewableDataConfig()` waits, so all four view-point entry points are covered, not just
   the one whose widget remembered.
2. **Pumping excludes user input** unless a modal is already up. The engine's `load()` wait and
   `LoadOperation::wait()` pump everything because their callers are modal (progress dialog, or
   `LiveController::reloadWindow`'s message box); everything else uses
   `ExcludeUserInputEvents`.
3. **Transitions are posted, never nested.** Nothing may start a load from inside the
   `appModeSwitchSignal` chain — the paused display is loaded from a queued
   `ViewManager::loadPausedDisplay()`, so the switch reaches every receiver and unwinds first.
   Same reason `setCurrentSource(feedPtr())` precedes `resumeSession()`: the outgoing operation
   must be disconnected before the catch-up load pumps.
4. **View dispatch is guarded, progress UI never pumps** — see *Re-entrancy guard + threshold
   event-pumping* below.

## Load flow (offline)

1. `reload()` — `captureSelection()` (for carry-over), `clearDataInViews()`, build the op,
   `setCurrentSource(op)`, `dataEngine().load(op)` (all via the shared `issueLoad()`).
   `setCurrentSource` subscribes ViewManager to the op's `dataChangedSignal`
   (`sourceDataChangedSlot`) and — for a `LoadOperation` — to its `startedSignal`/`finishedSignal`
   (`loadingStartedSlot`/`loadingDoneSlot`).
2. `DBContentDataEngine::load(op)` — safety-net wait, `setState(Running)` (→ `startedSignal`),
   `resolveTargetSet`, then per content: resolve read set (`spec.read_set_` or
   `mgr.getReadSet`), `composeWhereClause` (ds/line filter + FilterManager conditions +
   `custom_filter_clause_`), augment with the core meta-vars (`rec_num`/`ds_id`/`line_id`/`timestamp`),
   create a `DBContentReadDBJob`, submit via `jobManager()`.
3. **Per arrival** (GUI thread) — `DBContent::readJobDoneSlot` massages the buffer
   (`transformVariables` DB→var names, add `selected_`) and announces `readDoneSignal(name, buffer)`;
   the engine's `contentReadDoneSlot` does `op->setBuffer(name)` +
   `op->emitChanged({name}, reset=false, last=false)`. Empty contents don't emit.
4. **Finalize** (`finish()`) — `op->emitChanged({}, reset=false, last=true)`, then
   `setState(Done|Cancelled|Failed)` (→ `finishedSignal`), perf-metrics stop. On `Failed` the
   partial buffers are dropped and `result()` carries the error; on `Cancelled` they are kept.
5. `sourceDataChangedSlot` distributes each event (see *Distribution*); `loadingDoneSlot`
   runs the completion (view finalize + bookend).

**The read job stays inside `DBContent`** (`loadInternal` / `read_job_` / `readJobDoneSlot`),
symmetric with its update/delete jobs; the engine *orchestrates* — target set, read set, WHERE,
then collects each announced buffer. Blocking issuers call `op->wait()` (event-pumping) — the
live window load and the batch consumers do.

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
`compass.viewmanager.loadingDoneSignal()`.

**Treat them as edge notifications, not a matched pair.** A live session brackets itself
(`beginLiveSession()` on entry, `endLiveSession()` on exit), but a **pause raises an ordinary
offline pair inside those brackets**, because the paused display is a real `LoadOperation` whose
`started`/`finished` are wired by `setCurrentSource`. A session with one pause therefore emits:

```
started (entry) → started (paused load) → done (paused load) → [resume: nothing] → done (exit)
```

Both current consumers are edge-based, so this is harmless: `MainWindow` only stamps a time on
`started` and clears `loading_` / re-enables the Load button on `done` — which is what you want
while paused — and the RT view-point wait can't fire while live is running. A consumer that
counted pairs or treated `done` as "the session ended" would be wrong.

Each bookend also drives the **views** (`view->loadingStarted()` / `loadingDone()`), so the same
nesting applies there. One consequence: exiting live **from paused** runs every view's
`loadingDone_impl()` a second time — once for the paused load, once for `endLiveSession()` — over
unchanged data. Redundant work, not a wrong state.

**The done edge is deferred while a view still processes asynchronously.** Every view except the
tiny ones now does its heavy per-load work on a worker thread (offline and paused only - live
stays synchronous, see [readme_blocked_events.md](../../../readme_blocked_events.md)):
geographic geometry builds, table row indexes, histogram scan and generator, scatter/grid stash.
`loadingDoneSlot` still runs its per-view loop at DB finish, but the tail (`finishLoadingDone`:
dialog close + `loadingDoneSignal`) waits until no view reports `hasPendingProcessing()` anymore
(`View::processingFinishedSignal` -> `viewProcessingFinishedSlot`). The RT wait and the UI tests
therefore see done only after everything is committed, and the modal dialog keeps the user out of
a half-updated display - its view-phase progress advances per **finished** view, not per
dispatched one. A new load drops a still-deferred done of the superseded load (`done_pending_`
cleared in `loadingStartedSlot`).

The shared machinery is `ViewAsyncProcessor` plus the deferral in `ViewDataWidget`
([viewasyncprocessor.h](../../view/viewbase/viewasyncprocessor.h)): a view launches
`work` (worker thread, must touch no Qt object) and `commit` (main thread), and the base class
handles the deferral, the invalidation on a new load and the completion signal. Two hazards it
encodes, both hit in practice: results of a superseded generation are discarded rather than
committed, and `QProgressDialog::setValue` pumps the event loop, so a commit can re-enter the
load lifecycle - see the guards in `LoadController::advanceViewPhase`.

Commits are **drained one per zero-timer firing**, not run directly from the queued worker
completion. Several views' completions arrive as posted events, and Qt dispatches a
posted-event batch back to back - seconds of apply work without one native event in between,
which is exactly the unanswered-window-manager-ping "not responding" case. One commit per
timer firing lets the native loop breathe between commits (pings, repaints). Deliberately NOT
a `processEvents()` inside the commit: that dispatches other posted events too, and a
dispatched waiter (a runtime command gating on quiet views, see below) then waits inside the
pump for a `finishedSignal` that can only be emitted once the pump returns - a deadlock, found
the hard way. This is the same lesson as the threshold pumping note further down: treat
`processEvents()` inside the view lifecycle as suspect.

**Destruction contract:** the base destructor joins outstanding workers, but only after the
derived widget is already destroyed - a work function capturing `this` would read a half-dead
object. Any widget whose work functions capture `this` (the stash views, the histogram) calls
`shutdownAsyncProcessor()` (invalidate + join; queued commits are discarded) in its OWN
destructor. Snapshot-based workers (table) and the geo provider (owns its processor) do not
need it.

## LoadController — the offline load UX

`ViewManager` owns a **`LoadController`** ([loadcontroller.h](../../view/loadcontroller.h))
that holds the modal `QProgressDialog`, the wait cursor, and the two-phase progress (load
0..50% / view 50..100%). ViewManager drives it at its dispatch points:

- `begin(op)` — from `loadingStartedSlot` (i.e. off the op's `startedSignal`, **after** the
  engine's safety-net wait). Idempotent: it `end()`s any prior cycle first, so a
  superseded/overlapping load can't leak a dialog or unbalance the cursor. Sized via
  `resolveTargetSet(op.spec())`. Skips the dialog when there is nothing to load (wait cursor only).
- **Load-phase progress** is driven **directly off the op's `dataChangedSignal`**
  (`opDataChangedSlot`, connected in `begin`/disconnected in `end`) — *not* off ViewManager's
  re-entrancy-deferrable `sourceDataChangedSlot`. A deferred arrival re-running the advance
  used to yank the bar back to 50%.
- `beginViewPhase(n)` / `advanceViewPhase()` — in `loadingDoneSlot`'s per-view loop.
- `end(drain = true)` — closes the dialog + restores the cursor; `drain` pumps first (only the
  normal completion path does). The controller also **ends itself** off the op's `finishedSignal`
  when the op is no longer `currentSource()` — an abandoned load (a resume swapping the feed in
  over a paused load) is disconnected from ViewManager and would otherwise strand the dialog.
  Called from `loadingDoneSlot`
  **outside** the `processing_data_` guard, so a deferred `sourceDataChangedSlot` (e.g. the
  geo view's redraw) drains while the dialog is still up — matching the pre-refactor
  `finishLoading` ordering (no late geo update after "done").

Two dialog details, both flicker/black-frame related:

- **Deferred show (700 ms).** The load dialog no longer shows immediately: `begin()` sets
  `setMinimumDuration(700)` and arms it with `setValue(0)`, so Qt's own force-show timer
  displays it only once 700 ms elapsed - a load finishing faster never flashes a dialog.
  This works because the load dialog is value driven; the indeterminate busy dialog (below)
  never arms that timer and needs an explicit deferred `show()` instead.
- **Synchronous paint on show.** With the deferred shows, the first paint event can starve
  behind long queued work and the window maps black for a second. A `LoadController`
  event filter catches `QEvent::Show` on both dialogs and calls `repaint()` - the
  single-widget synchronous paint, deliberately NOT a `processEvents()` (see the threshold
  pumping section on why pumping inside the load lifecycle is suspect).

## Interactive redraws - the same contract outside a load

An interactive **recompute redraw** (Color Mode, style or filter change, view preset apply)
used to run its compute phase inline on the main thread - the "application is not responding"
case for large datasets, long after loads had gone asynchronous. Offline it now runs on the
same worker/commit machinery as the load path; live stays synchronous everywhere, as before.

Per view, `redrawData(recompute = true)` routes as follows (`redrawData(false)` display-only
refreshes stay synchronous where they were):

- **Geographic View**: `GeometryItemProvider::redrawGeometry(RecomputeDisplay)` goes through
  `applyChange(loaded contents, reset = false, last = true)` - the same staged asynchronous
  rebuild as a data arrival. Layers are built detached on workers and swapped in on commit; a
  later rebuild's attach replaces an earlier one's layers. The in-place per-layer recompute
  survives only for live and the no-data case.
- **Table View**: `launchAsyncPrepare()` (layer scan + row index + sort on a worker,
  `commitLoadedData` applies) - shared verbatim with `loadingDone_impl`.
- **Histogram / Scatterplot / Grid**: `VariableViewDataWidget::redrawData_impl` fires the same
  `postLoadTrigger()` the load-done path uses (generator / stash fill on a worker, display
  commit on the main thread). Annotation display and live decline and fall through to the
  synchronous path.

Three pieces make this safe against interaction, mirroring what the load dialog provides
during loads:

1. **`viewRefreshed` waits for the commit.** `ViewWidget::redrawDone()` holds the signal back
   while the data widget reports `hasPendingProcessing()`, released by
   `dataWidgetProcessingFinished()` - the exact mirror of the `loadingDone()` deferral. To
   every consumer (runtime commands, UI tests) `viewRefreshed` means "the view shows the
   result", launch or not. A recompute requested while work is still outstanding is
   **coalesced** (`pending_recompute_redraw_`): the running work's inputs are already stale,
   so one recompute runs after the commit instead of stacking jobs; a new load drops the
   pending request.
2. **Runtime commands gate on quiet views.** Both main-thread command execution entries
   (`RTCommandRunnerStash::executeCommand[Async]`) pump via `Utils::Async::pumpUntil` until
   `ViewManager::hasPendingViewProcessing()` is false. Without this, a `uiset` mutating view
   state can land while a worker still reads it - a torn read that aborted in practice
   (grid preset apply during a running stash worker). Same pattern as `issueLoad`'s wait.
3. **A modal busy dialog covers user interaction.** `LoadController::beginViewProcessing()` /
   `endViewProcessing()`: view processing starting OUTSIDE a load cycle (reported through
   `ViewAsyncProcessor::startedSignal` -> `View::processingStartedSignal` ->
   `ViewManager::viewProcessingStartedSlot`) shows an application-modal, cancel-less
   "Updating views..." dialog; the last commit closes it. Modality is the point - the user
   cannot mutate view config or close a view while workers read state. The dialog is shown by
   an explicit deferred `show()` (700 ms single-shot): an indeterminate `QProgressDialog`
   never arms its `minimumDuration` timer (only `setValue` does), so auto-show would never
   display it, and showing immediately would flicker on every small selection update. A load
   cycle's own dialog supersedes it (`begin` ends any interactive busy state first).

Two data-shape optimizations complete the picture; without them a Color Mode change stayed a
multi-second stall even with all compute on workers:

- **Layer aggregation caches** (histogram + table): `rebuildLayerTree()` used to rerun
  `view_layer_scan::aggregateLayers` - a full row scan, seconds at millions of rows -
  although the aggregation depends only on the loaded buffers, not on colors. Both views
  now keep the aggregation (`layer_agg_cache_`, filled by the async commit or the sync
  scan, invalidated on data change/clear), so a color mode change restyles the panel from
  the cached aggregates in milliseconds.
- **The geo compile stage** (see *Distribution*) covers the GPU side of the same scenario.

**Measuring stalls:** the client has a hidden `--stall_watchdog` option - any main-thread
stretch above 3 s dumps its own live backtrace to stderr (`=== MAIN THREAD STALL ===` in the
captured test logs). Every improvement above was located and verified with it; see
readme_testing.md for usage. The Color Mode cycling test (`ui_geographicview_style_modes` on
the skeyes dataset) went from 204 s with 22 stall dumps to 21 s with zero.

## Distribution

The displayed dataset is the **current source** — a `DBContentDataSet` (`LoadOperation`
offline **and while paused**, the `LiveController`-owned `LiveDataFeed` while live runs) held by
`ViewManager` as `current_source_`. It owns the buffer
map plus a lazily-built `DBContentDataIndex` (`dbc_id → ds_id → line_id → buffer indices`) and
accessor, and emits **one** change signal:

| Signal | Payload | Meaning |
|---|---|---|
| `dataChangedSignal(names, reset, last)` | `vector<string>`, `bool`, `bool` | Buffers/indices for `names` changed. `reset` → drop state for contents NOT in `names` (full replacement). `last` → final event of a batch; listeners run finalize work. The index is rebuilt **lazily** on first `index()`/`accessor()` access, so a slot sees it coherent with the buffers as of that access. |

**`ViewManager` is the sole subscriber** (`sourceDataChangedSlot`). Per registered view it
drives, in one guarded turn, the single callback `View::updateFromSource(source, names, reset,
last)` (the old dual `loadedData`/`updateData` path is gone — step 8). The base
`ViewDataWidget::updateFromSource` mirrors `data_ = source.buffers()` (so the existing
`viewData()`/redraw/selection machinery is undisturbed), feeds the borrowed
`DBContentItemProvider` if the view has one, and calls the view's `updateFromSource_impl`:
- **Standard views** (table/scatter/grid) read `source.buffers()`; histogram (via
  `VariableViewDataWidget`) does a full refresh on `last`. None own a provider.
- **Geographic View** owns a `GeometryItemProvider`, fed here: `setSource(&source)` +
  `applyChange(names, reset, last)` → `resetData()` (if `reset`) → `rebuildContent(id)` per name
  → `contentRebuilt()` (if `last`), reading `source.index()`/`source.buffers()`. So OSG never
  paints the empty intermediate state between wipe and rebuild. `GeographicView::updateFromSource`
  overrides to skip the **whole** update (provider + finalize) on live overload. Offline (and
  paused) the layer geometry itself is built asynchronously: `rebuildContent_impl` stages the
  layers, one worker per arrival builds them (its layers in parallel), and they attach in a queued
  completion turn - the live tick stays fully synchronous (see the deferred done edge under
  *Bookends*).

  The attach itself has a **GPU compile stage** (`attachArrival`): with the viewer's
  incremental compile operation available (the pager's ICO), the built layers attach
  MASKED and their GL objects are uploaded in ~50 ms per-frame slices - a single-frame
  upload of a large arrival blocks the main thread inside the GPU driver for seconds
  (measured as fence waits in `drmSyncobjTimelineWait`). The completion callback fires
  inside `frame()` (single threaded viewer) and hops out via a queued call
  (`arrivalCompileDone`): unmask, swap into `group_layers_` - the OLD layers stayed
  visible during the upload, so the switch is seamless. Because rendering is on demand
  and the compile only progresses inside frames, a **frame pump** in the `ViewerWidget`
  re-requests a frame every ~30 ms while compiles are outstanding
  (`setFramePumpActive`). `hasPendingBuilds()` includes this stage, so the done edge,
  `viewRefreshed`, the busy dialog and the command gate all wait for the RESIDENT
  result; `cancelPendingBuilds` bumps a compile generation, and superseded completions
  only clean up.

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

The session runs an explicit state machine — **`Stopped` / `Running` / `Paused`** — driven by
`ViewManager::appModeSwitchSlot` through `startSession()` / `pauseSession()` / `resumeSession()` /
`stopSession()`. Ticks run only while `Running`; `Paused` (including the catch-up load inside
`resumeSession`) and `Stopped` suppress them.

**Key fact:** pause does **not** stop ingestion. The ASTERIX decode/insert pipeline keeps running
and the DB keeps accumulating. The engine announces *every* insert (it is app-mode-free); the
live/offline gate sits in `LiveController::insertedDataSlot`, which drops the announcement unless
the app mode is `LiveRunning`. So the feed goes stale while the DB grows — and the **display**
leaves the feed entirely for its own offline load (see the table).

Transitions (`ViewManager::appModeSwitchSlot`, driven by `COMPASS::appModeSwitchSignal`;
`compass.cpp::appMode()` no longer special-cases pause/resume — it just switches the importer
and emits):

| Transition | Action |
|---|---|
| **Fresh entry** (Offline→Live) | `current_source_ = feed`, `startSession()` → `reloadWindow()` primes the feed with the recent window, `refreshDisplay()` distributes it, one `started` bookend |
| **Pause** (Live→Paused) | `pauseSession()` (ticks off, DB keeps ingesting) + a **queued** `ViewManager::loadPausedDisplay()` that swaps the display to a plain offline `LoadOperation` — so paused behaves like offline: load button, filters and the time window all work, and `reload()` is allowed |
| **Resume** (Paused→Live) | `current_source_ = feed` **first** (cancelling the paused op), then `resumeSession()` → `reloadWindow()` catch-up, then `refreshDisplay()` |
| **Exit** (Live→Offline) | `current_source_ = null`, `stopSession()` → `clearFeed()`, one `done` bookend |

Entry and resume are the **same fetch**: `reloadWindow()` runs while the state is still
`Stopped`/`Paused`, so a pump-fired tick cannot overlap it.

**The window load** — `LiveController::reloadWindow(title)` harvest-loads `timestamp >= now -
maxLiveDataAgeCache` from the DB (a private blocking `LoadOperation`, never the display source, so
no bookend/dialog — it raises its own modal) and `seedFrom`-**replaces** the feed with the current
window. Overlap is prevented by the state machine rather than a flag: the load runs while the
state is not yet `Running`, so `processLiveModeSlot`'s `if (!running()) return;` suppresses any
pump-fired tick (inserts still stage and merge on the first tick after). `refreshDisplay()` then
runs `processTick()` (feed `cutCachedData` + distribute) **without** a DB delete — the every-tick
bound resumes on the next real tick.

**The per-tick** (`processLiveModeSlot`, from `insertedDataSlot` + the watchdog): `if (!running())
return;`, then the DB bound `deleteDBContentData(now - maxLiveDataAgeDb)` — **skipped with a
`logwrn` when a delete is still in flight** (`hasActiveDeleteJob()`; the display cut is
independent, so the next tick re-bounds) — then `processTick()`. `LiveDataFeed::processTick()` merges
staged inserts, `cutCachedData` (trims the display to `maxLiveDataAgeCache`), filters, updates
latency, and emits **one** atomic `dataChangedSignal(all, reset=true, last=true)` →
`sourceDataChangedSlot`. The single queued event means the geo provider wipes + rebuilds +
finalizes in one event-loop turn (no empty intermediate OSG paint). The **display window** is
bound by the feed's `cutCachedData`, independent of the DB delete.

### Layer keep-alive in the per-tick rebuild

The feed's per-tick wipe + rebuild is atomic, and the geo provider handles it as a
**mark-and-sweep rebind** instead of destroying and recreating every layer
(`live_inplace_dispatch_`, set for a live-running dispatch): `reset_impl` stashes the
existing `(dbc, ds, line)` layers (mark), `rebuildContent_impl` hands a stashed layer of
the same key the new item group via `GeometryItemGroupLayer::rebuildFromGroup` - an
in-place data rebuild (point data, item ranges, asset geometry arrays) that keeps the
scene-graph node, the labels and the render state alive - and the finalize destroys only
the layers whose key vanished (sweep). Kept TR labels are re-anchored to the new point
data by record number, vanished ones dropped; no per-tick label teardown (the old label
flicker) and no per-tick label backup/restore JSON round trip. The layer holds its item
group as `shared_ptr`, so the data a stashed layer references survives the provider
reset until rebind or sweep. All other resets (offline load, paused display load,
grouping change) stay on the full wipe + backup/restore path.

**Label texts are part of the per-tick snapshot.** The feed mutates the shared
buffers in place (merge + timestamp sort + front trim), so between a `processTick()`
and its queued rebuild turn every stored point/buffer index is stale - a label text
regeneration in that window would read other targets' rows. Kept labels therefore
regenerate their texts only in the anchored per-tick path (the re-anchor in
`updateItemLabels`/`updateTRLabels`, fresh indexes, same turn as the rebuild); all
other triggers (LOD change in the overlay pass, `updateLabelContents` from a layer
style update) go through `OverlayItem::requestContentUpdate()`, which in live-running
mode only marks the content dirty for the next anchored update
(`GeometryItemGroupPointLabel::deferContentRegen`). Offline regenerates immediately,
as before - there the buffer never changes under the layer.

## Migrating away from per-content loads

`DBContent::load(...)` is gone as a public entry point — `loadInternal`/`read_job_` remain, but
only the engine drives them. Tasks build a `LoadOperation` and call `dataEngine().load(op)`:

- **Time-sliced (Reconstructor).** One op per slice with `dbcontents_` explicit, `apply_*_filters_=false`,
  `read_set_`/`custom_filter_clause_` lambdas; iterate by connecting the op's `finishedSignal` and
  issuing the next slice.
- **Single content (RT `get_data`).** `LoadRequest::forContent(name, rs, clause)`; `op->wait()`
  synchronously, with an `isLoading()` guard (RT bypasses UI modality).
- **Restricted fan-out (ARTAS, RadarPlot).** Explicit `dbcontents_`, `apply_*_filters_=false`,
  `read_set_` lambda, optional `custom_filter_clause_`.
- **Own data sources (Evaluation, AnalysisDataset).** `datasrc_selection_` instead of mutating the
  global DBContext selection.

None of them check the op's terminal state yet: a `Failed` load looks like an empty one and a
`Cancelled` one hands back partial buffers reporting success — see the `@TODO`s at the harvest sites.
