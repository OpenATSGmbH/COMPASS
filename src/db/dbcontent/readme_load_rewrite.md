# Rework: LoadOperation-based data loading

## Context

Today all surveillance-data loading flows through `DBContentManager`, which owns the one in-memory dataset (`data_`, keyed by DBContent name) and drives **two parallel distribution paths**:
- **Path A** — lifecycle signals (`loadingStarted`/`loadedData`/`loadingDone`) → `ViewManager` → per-view `loadedData(buffers, reset)`. The standard views (table, histogram, scatter, grid) consume the raw buffer map.
- **Path B** — `DBContentDataStore` (a `dbc_id → ds_id → line_id → row-index` index + accessor over the same buffers) emits `dataChangedSignal(dbc_ids, reset, last)` → `DBContentItemProvider` subclasses. Its **only** consumer is `GeometryItemProvider` (the Geographic View).

Problems:
- **Single shared buffer slot.** Every `load()` does `clearData()` then full-replace. Batch consumers (Evaluation, Reconstructor, ARTAS, RadarPlot, RT `get_data`) can't coexist with a view load — they call `enableDataDistribution(false)`/`disableDataDistribution(true)`, borrow `data_`, then `clearData()`. Fragile, and couples unrelated consumers through one slot.
- **No first-class load object.** `LoadRequest` ([loadrequest.h](src/db/dbcontent/loadrequest.h)) is a stateless value; runtime state (flags, progress dialog, current request, selection carry-over, buffers) is scattered across `DBContentManager`. No cancellable/inspectable handle, no `Failed` state.
- **Data movement is fused into `DBContentManager`,** which also owns DBContent definitions, schema/variables, metadata, targets/associations, load, insert and delete. No separable data-I/O engine.
- **Two divergent distribution paths** for what is one dataset — the index is wired as a second global path just to serve one view.
- **Filters are ad-hoc string concatenation.** `getSQLCondition()` threads a shared `bool& first` and *mutates the read set* as a side effect ([filtermanager.cpp:220](src/filter/filtermanager.cpp#L220)); datasource/line constraints are built separately in `composeWhereClause`; and the same clauses are re-implemented by eval (global-FilterManager hijack) and reconstructor/compass/ARTAS (hand-written SQL).

**Goal:** make a **`LoadOperation`** the unit of loading (constraints in, buffers out). Move **all** DBContent DB I/O — read, insert, delete — into a **`DBContentDataEngine`** serialized on the single DB connection. `DBContentManager` shrinks to the *static model* (definitions/schema/metadata/targets/data-sources) plus thin facades; it owns no dataset and does no data movement or distribution. **Unify the two paths** onto a single indexed source with per-view item providers, owned by `ViewManager`. Live mode is a **`LiveDataFeed`** (a `LoadOperation` sibling) owned by `ViewManager`, doing in-memory display work only. Filter SQL comes from one shared clause library.

## Decisions locked in

1. **Single-operation assumption — no queueing.** Exactly one `LoadOperation` runs at a time, and only one is issued at a time because the **UI blocks during a load**. No queue/coalescing/preemption. A wait-for-current safety net (not an assert) stays at the top of `load()`.
2. **Manager = static model; engine = all DB I/O.** `DBContentDataEngine` owns `load(op)`, `insert(buffers)`, and `deleteOlderThan(ts)`, all serialized on the single DB connection. `DBContentManager` keeps definitions/schema/metadata/targets/data-sources and thin facades; it owns no dataset, moves no data, and distributes nothing (`loadedDataSignal`/`data_`/`enableDataDistribution`/`distribute_data_` removed).
3. **Big-bang buffer-ownership removal.** `DBContentManager::data_`/`loadedData()` go; consumers read buffers from a data set.
4. **Displayer owns the source.** `ViewManager` owns a single `current_source_` (`shared_ptr<DBContentDataSet>`) — a `LoadOperation` offline, a `LiveDataFeed` live. The engine only *borrows* a running operation to fill it and *announces* inserts; COMPASS/manager own no dataset.
5. **Unified distribution.** One shared indexed source (buffers **+** index **+** accessor) with **one** change signal, delivered to every view as `updateFromSource(source, names, reset, last)` — the incremental `(names, reset, last)` contract every view consumes. Views read the source directly (buffers/index/accessor) in their `updateFromSource_impl()`; **only the Geographic View sets a `DBContentItemProvider`** (for its grouping/layer machinery), the others need none. ViewManager is the **sole dispatcher**. Path A/Path B merge. Migration is incremental (the source exposes both `.buffers()` and `.index()`; the old raw-buffer `loadedData`/`updateData` path stays until each view's `updateFromSource_impl` is filled in).
6. **Index lives on the data set.** `DBContentDataSet` owns/lazily-builds its index + accessor. `DBContentDataStore` is absorbed into it.
7. **Live is a sibling, not a subclass of a load.** `DBContentDataSet` is the common base; `LoadOperation` (one-shot) and `LiveDataFeed` (continuous, in-memory display only) derive from it.
8. **Shared, parameterized filter-clause library** decoupled from stored filter state.

## Class shape

```
DBContentDataSet          base — owns buffer map + lazily-built index + accessor; ONE dataChangedSignal
  ├─ LoadOperation        one-shot query; the only type load() accepts        (issuer-owned)
  └─ LiveDataFeed         continuous, in-memory display accumulation           (ViewManager-owned)

DBContentDataEngine       ALL DBContent DB I/O on the one connection: load(op) / insert / deleteOlderThan
DBContentItemProvider     optional; turns the shared index into view-specific items per item-mode
  └─ GeometryItemProvider   (Geographic View only; other views consume the source directly)
```

Keep `DBContentDataSet` at global scope (avoids `dbContent::DBContentDataSet` stutter). The absorbed index type is `DBContentDataIndex` (renamed from `DBContentDataStore`), now a member of `DBContentDataSet` rather than a standalone distribution node. `ViewManager` owns the current source (`shared_ptr<DBContentDataSet> current_source_`), pointing at whichever of the two subtypes is live.

## Architecture

### 1. `DBContentDataSet` (base) — the shared source

Owns the buffer map **and** its derived index + accessor, and emits the single change signal. "The current data set" is thus a complete, self-describing source any provider can attach to.

```cpp
class DBContentDataSet : public QObject
{
    Q_OBJECT
public:
    using BufferMap = std::map<std::string, std::shared_ptr<Buffer>>;
    const BufferMap& buffers() const;
    const DBContentDataIndex& index() const;                 // lazily built/cached
    const dbContent::DBContentAccessor& accessor() const;    // lazily built/cached
    bool empty() const;
signals:
    // names = contents whose buffers changed; reset = drop all prior state;
    // last  = final event of a logical batch → listeners run finalize work.
    void dataChangedSignal(const std::vector<std::string>& names, bool reset, bool last);
protected:
    BufferMap buffers_;
    void setBuffer(const std::string& name, std::shared_ptr<Buffer> b);   // invalidates index
    void emitChanged(const std::vector<std::string>& names, bool reset, bool last);
};
```

- The `(names, reset, last)` contract is the one `DBContentDataStore::dataChangedSignal` already uses ([dbcontentdatastore.h:67](src/db/dbcontent/dbcontentdatastore.h#L67)) — proven, and it already carries the atomicity semantics (a single queued event so OSG never paints the empty intermediate state). The index is rebuilt/consistent **before** the signal fires, so consumers pulling `index()` in the slot see a coherent view.
- **Still thin re: query concepts.** No spec/state/cancel/wait on the base — those are `LoadOperation`'s. Owning an index/accessor is not a query concept; it's a projection of data the base already owns. That discipline is what keeps the base from collapsing into "live is-a load".

### 2. `LoadOperation` (one-shot)

```cpp
class LoadOperation : public DBContentDataSet
{
public:
    explicit LoadOperation(LoadSpec spec);
    const LoadSpec& spec() const;
    State state() const;              // Created/Running/Done/Cancelled/Failed
    void cancel();
    void wait();                      // event-pumping wait
signals:
    void startedSignal();
    void finishedSignal();            // Done | Cancelled | Failed
private:
    LoadSpec spec_;
    State    state_;
    friend class DBContentDataEngine;  // engine fills buffers_ and drives state
};
```

- **`LoadSpec` (immutable once issued)** supersedes `LoadRequest`: target dbcontents, captured per-content constraint (a WHERE **string** snapshot during migration; a `FilterClause` once the clause library lands), the **read-set spec** (see below), and the existing flags (`show_status_`, `cancellable_`, `measure_db_performance_`, `custom_filter_clause_`). Keep the `standard()/withFilter()/forContent()` factories. Resolve `custom_filter_clause_` to a string at issue time to preserve snapshot immutability.
- **`Failed`** is a new terminal state; `cancel()` uses the existing cooperative `setObsolete()`, partial buffers discarded with the op.
- **Simplification:** `reset` is unneeded for the offline path — each load is a *new, empty* data set, so switching to it inherently replaces everything. `reset=true` stays meaningful only for live (per-tick window trim).

**Read set — a first-class `LoadSpec` field.** Which columns to read is part of the spec, so any caller (including a fully custom load) passes exactly what it needs:

- `LoadSpec::read_set` — an optional per-content provider `std::function<dbContent::VariableSet(const std::string& dbcontent_name)>`. A custom load sets it directly; a convenience factory wraps an explicit `std::map<std::string, dbContent::VariableSet>` for callers that already know their targets (as `forContent` does today via a captured `rs`).
- If **unset**, the engine falls back to the **issuer's default** — for view loads, `ViewManager`'s union of registered view read sets, plus the optional `utn` (today part of `addStandardVariables`). This replaces the manager's `getReadSet` reaching into `viewManager()`/`evaluationManager()`. The default-only extras (`utn`, and any view-specific vars) do **not** reach a custom load that supplies its own `read_set` — matching today's Reconstructor, which overrides `read_set_` and does not inherit them.
- The engine always **augments** the resolved per-content set before executing, so no caller can omit an essential: (1) the **core meta-vars `rec_num`/`ds_id`/`line_id`/`timestamp`** — always. Note this **promotes `timestamp` into the core**: today `loadInternal` force-adds only the first three, yet every load orders by `timestamp` (`use_order_=true`) and nearly every consumer needs it, so a custom `read_set` that forgets it currently yields a buffer ordered-by-but-missing timestamp. Core-augmenting it closes that footgun. (2) the `FilterClause.required_vars` from the clause library, when view filters apply (a filter may constrain a column not otherwise read).
- **Single source for the standard variables.** The "standard variables" are defined in **exactly one place** — a canonical `DBContentManager::standardVariables(name)` (the static model owns the schema). Today the list is duplicated across three call sites with hardcoded var lists: `addStandardVariables` ([dbcontentmanager.cpp:2208](src/db/dbcontent/dbcontentmanager.cpp#L2208)), the force-add in `loadInternal` ([dbcontent.cpp:288](src/db/dbcontent/dbcontent.cpp#L288)), and inside `addInsertedDataToChache` ([dbcontentmanager.cpp:1552](src/db/dbcontent/dbcontentmanager.cpp#L1552)). All collapse onto the one function. The **always-added core** (`rec_num`/`ds_id`/`line_id`/`timestamp`) and the optional `utn` are expressed as subsets of that single definition, so membership is decided once and the engine, the ViewManager read-set default, and the feed's prune stay in lockstep by construction.
- **Effective read set** per content, computed at execution = `read_set(name) (or issuer default) ∪ core meta-vars (rec_num/ds_id/line_id/timestamp) ∪ filter-required`. Self-contained and snapshot-able, while callers stay terse. `utn` remains genuinely optional (default path / explicit request only).

### 3. `DBContentDataEngine` (extracted from `DBContentManager`)

The single door to the DB connection for DBContent data — all three verbs serialized behind the one connection (as the read/insert/delete jobs already are in `JobManager`):

- **`load(op)`** — stateless per load (all state lives in the operation): `resolveTargetSet`, WHERE composition / `FilterClause` rendering, per-content fan-out (`DBContent::loadInternal`, read-job creation), buffer accumulation from `readJobDoneSlot` → `op.setBuffer(name)` → per-content `dataChangedSignal({name}, false, false)`, completion → `dataChangedSignal({}, false, true)` + `finishedSignal()`, progress accounting, cancel propagation, the wait/pump helper.
- **`insert(buffers)`** — the ASTERIX/import insert path (`DBContentInsertDBJob`); on completion emits an **`insertedDataSignal(buffers)`** announcement (the `LiveDataFeed` consumes it — the engine does not know about the feed).
- **`deleteOlderThan(ts)`** (and other content deletes) — the DB write that bounds the live cache (today's `deleteDBContentData`), now DB-side only.

`DBContentManager` keeps thin facades (`load`, `insertData`) that delegate to the engine so existing callers (e.g. ASTERIX import → `insertData`) barely change.

### 4. `DBContentManager` slims to the static model

Retains: DBContent definitions, schema/variables, metadata, targets/associations, data-source model, min/max timestamps and counts. Owns **no** dataset and does **no** data movement — it delegates to the engine and holds `current_operation_` only *while a load runs* (a borrowed pointer, so read-job callbacks route to the right op), releasing it on finish.

- `void load(std::shared_ptr<LoadOperation> op)` → facade over `engine.load(op)`. `isLoading()` = an operation is running.
- `void insertData(...)` → facade over `engine.insert(...)`; the live old-row deletion is `engine.deleteOlderThan(...)`.
- **Wait-for-current safety net, not an assert.** Retain the preamble at [dbcontentmanager.cpp:609](src/db/dbcontent/dbcontentmanager.cpp#L609) but simplified: drop `quitLoading()` (quitting would silently discard a load another component awaits), keep the `processEvents()` spin until the running op finishes, add a `logwrn` when it must wait. Graceful degradation beats aborting a long acquisition.
- **Remove:** `data_`, `loadedData()`, `addLoadedData`'s manager sink, `enableDataDistribution`/`distribute_data_`, `loadedDataSignal`, and the manager-owned `DBContentDataStore`. Keep thin lifecycle notifications (`operationStarted(op)`/`operationFinished(op)`) for incidental observers (MainWindow load chrome [mainwindow.cpp:290](src/app/client/mainwindow.cpp#L290), `TargetListWidget`, `DataSourcesStatusWidget`).
- **Read-set composition untangled:** the read set is a `LoadSpec` field (§2 *Read set*) resolved+augmented by the engine, replacing `getReadSet`'s reach into `viewManager()`/`evaluationManager()` and the in-place read-set mutation during WHERE composition.
- **Progress dialog stays manager-owned** (must also serve non-view `show_status_` loads, e.g. the app-mode-switch load [compass.cpp:1259](src/app/compass.cpp#L1259)). Driven by the operation's DB-load progress; the view phase (50–100%) is driven by ViewManager via the existing `beginViewProgressPhase`/`advanceViewProgress`.
- **`loadBlocking` → `LoadOperation::wait()`** — must pump events (GUI-thread callers run while DB jobs execute on the worker), preserving the readme's re-entrancy guards (3 s pump threshold, `repaint()`-not-`processEvents()`).

### 5. `LiveDataFeed` (live mode) — ViewManager-owned, in-memory only

```cpp
class LiveDataFeed : public DBContentDataSet
{
public:
    void addInserted(BufferMap inserted);   // from the engine's insertedDataSignal
    void processTick();                      // merge + in-mem trim + filter → one atomic emit
    boost::posix_time::time_duration latency() const;
};
```

Absorbs only the **in-memory** half of `processLiveModeSlot` ([dbcontentmanager.cpp:1335](src/db/dbcontent/dbcontentmanager.cpp#L1335)): `addInsertedDataToChache` ([dbcontentmanager.cpp:1530](src/db/dbcontent/dbcontentmanager.cpp#L1530)), `cutCachedData`, `filterDataSources`, `filterBuffers`, latency. The **DB write** half (`deleteDBContentData`/insert) lives in the engine (§3) — a view-side object must never issue DB writes. Per tick, emits **one** `dataChangedSignal(all, reset=true, last=true)`. Being a `DBContentDataSet`, it is a **fully-indexed source** exactly like a `LoadOperation` — providers attach identically.

**Ownership & feed:** owned by `ViewManager` as its `current_source_` during a live session. It self-feeds by connecting to the engine's `insertedDataSignal(buffers)` — the manager/engine never push into it. `deleteOlderThan` runs independently on the engine to bound the live DB.

**Read set for live.** The feed runs no query, so it has **no `LoadSpec`** — but it **does need a read set**. `addInsertedDataToChache` uses one as a **pruning target**: the ASTERIX decoder emits a *wide* buffer, and the feed trims each insert down to the columns the views need, then `transformVariables` (DB→var names) and adds the `selected_` flag (the same massaging the load path does in `readJobDoneSlot`, except SQL reads arrive already-narrow while decoded inserts are wide and must be trimmed). ViewManager, owning the feed, supplies a **read-set provider** — the *same* live view read set it uses to build the priming `LoadOperation`, so there is one live read-set source. It is evaluated per tick (reflecting currently-open views) and, in live, includes the CAT063 sensor-status vars (`con`/`sensor_sac`/`sensor_sic`) today added inline in `addInsertedDataToChache`.

Two consequences: live filtering is **in-memory** (`filterBuffers`), so the filter's variables must survive the prune — the read-set provider must include filter-required vars just as the offline augmentation does; and a view opened mid-live needing a new variable cannot retroactively populate already-pruned rows (new inserts carry it, old cached rows don't) — an existing live limitation, not introduced here.

**Live decomposes into what the code already does:** on live entry a **real one-shot `LoadOperation`** primes the cache with `timestamp >= now - max_age` — literally [compass.cpp:1282](src/app/compass.cpp#L1282) today; its buffers seed the feed; from there the feed accumulates per tick. No never-ending operation.

**The offline↔live switch is a local pointer swap in `ViewManager::appModeSwitchSlot`:**
- **Entry:** issue the priming `LoadOperation` (don't distribute it, just harvest `op.buffers()`) → create a `LiveDataFeed`, seed it → `disconnect` old source, set `current_source_ = feed`, `connect` `feed->dataChangedSignal` (to distribute) and `feed`←`engine.insertedDataSignal` (to feed) → `loadingStateChanged(true)` → one distribution of the seeded state.
- **Exit:** `disconnect`/drop the feed → `loadingStateChanged(false)` → clear views or issue a normal offline `LoadOperation`.

Because both subtypes are `DBContentDataSet`, ViewManager subscribes to `current_source_->dataChangedSignal` in exactly one place and never branches on the kind.

*Alternative considered and rejected:* deriving `ContinuousLoadOperation` from `LoadOperation`. It buys one buffer-ownership rule and free live bookends, but asserts "live is-a load operation" (no issuer, a `wait()` that never returns, buffers mutating after completion, a meaningless spec, and `load()` would type-accept a feed). The sibling design keeps the benefits structurally; ViewManager re-synthesizes the bookends (§6).

### 6. Unified distribution — one source, per-view updateFromSource, ViewManager dispatches

Path A and Path B merge into one flow:

```
current source (DBContentDataSet: LoadOperation or LiveDataFeed)
   └── dataChangedSignal(names, reset, last)
        └── ViewManager  (SOLE subscriber; re-entrancy guard + 3 s event-pumping)
             └── for each View: view->dataChanged(source, names, reset, last)
                  └── View::updateFromSource_impl(source, names, reset, last)
                        (reads source.buffers()/index()/accessor() incrementally;
                         the geo view additionally feeds its DBContentItemProvider)
```

**Per-view intake — provider optional.** Every view receives `updateFromSource(source, names, reset, last)` and does its incremental intake in `updateFromSource_impl()`, reading the source's buffers/index/accessor and using `names` to rebuild only what changed. Most views need **no** provider — they update themselves directly. **Only the Geographic View sets a `DBContentItemProvider`** (`GeometryItemProvider`, for its grouping/layer machinery); the base owns it and feeds it before calling the view's `_impl`. Since the source exposes both buffers and index, views migrate **incrementally** — an unported view keeps consuming `source.buffers()` through the raw-buffer `loadedData`/`updateData` path until its `updateFromSource_impl` is filled in.

**Unified `View` API** replaces the four entry points (`loadingStarted`/`loadedData`/`loadingDone` + the provider slot) with **two**:
- `View::loadingStateChanged(bool loading)` — chrome, driven from the operation's `started`/`finished` (and the live bookends).
- `View::dataChanged(const DBContentDataSet& source, names, reset, last)` — data; the view drives its provider (or pulls buffers) from `source`.

**ViewManager responsibilities:**

| # | Responsibility | Detail |
|---|---|---|
| 1 | **Own the current source** | `std::shared_ptr<DBContentDataSet> current_source_` — the `LoadOperation` when offline, the `LiveDataFeed` (which ViewManager also creates/owns) in live. Single owner of whatever is displayed. |
| 2 | **Build & issue view loads** | Compose `LoadSpec` (clause library + view read sets); create the `LoadOperation`; `dbContentManager().load(op)`. |
| 3 | **Sole subscriber + dispatcher** | Subscribe to `current_source_->dataChangedSignal`; drive each view's `dataChanged` in a controlled loop. Retains the `processing_data_` guard (`QScopedValueRollback`) and 3 s `pump_threshold_ms` logic ([viewmanager.cpp:994](src/view/viewmanager.cpp#L994)). |
| 4 | **`appModeSwitchSlot`: swap the source** | Offline↔live pointer swap per §5 — priming load, create/seed/connect the feed, disconnect/drop it on exit. Hands the feed its **read-set provider** (the live view read set, same one used for the priming load and offline loads). |
| 5 | **Synthesize live bookends** | One `loadingStateChanged(true)` on live entry, `(false)` on exit — moved off the manager's `appModeSwitchSlot` to its right owner. |
| 6 | **Selection carry-over** | Seed each new operation from the prior one's selection (replaces the manager's `tmp_selected_rec_nums_`). |
| 7 | **View-phase progress** | Drive 50–100% via the manager's existing progress API. |
| 8 | **Reload-state bookkeeping** | Unchanged (`reload_needed_`, `notifyReloadStateChanged`, per-view `reloadNeeded`). |

Explicitly **not** ViewManager's job: executing loads (engine), live accumulation (feed), the index itself (on the data set), DBContent schema/metadata (manager). `disable_data_distribution_` is retired — batch operations are simply sources ViewManager never points at.

### 6a. Provider ↔ view-framework integration (the shape of step 3c)

Today Path B is a *side channel*: `GeometryItemProvider` binds to the manager-owned `DBContentDataStore` in its ctor and self-subscribes to `data_store_->dataChangedSignal` — the view framework knows nothing about it, and `GeographicViewDataWidget`'s framework `_impl` methods do no geometry work (a comment at [geographicviewdatawidget.cpp:625](experimental_src/view/geographicview/geographicviewdatawidget.cpp#L625) notes "geometry is built/refreshed via Path B"). Step 3c dissolves the side channel by making the provider a **first-class, base-owned component of `ViewDataWidget`**, driven by the framework from `current_source_` — the provider never self-subscribes.

**Provider reshape (`DBContentItemProvider` + `GeometryItemProvider`).**
- Ctor takes `DBContentManager&` (for `dbContentWithId`/`dbContentId`/`targetModel`, and to derive `context_manager_` via `compass().dbContextManager()`) instead of `DBContentDataStore&`. The `auto_update` signal wiring is removed.
- Data comes from a non-owning current source: `void setSource(const DBContentDataSet* source)` — nullable, re-pointed by the framework, cleared to `nullptr` when the source is swapped away. Rebuild/reset read `source_->index()` (id-keyed `indices()` / `buffer(dbc_id)` / `targetReportAccessor(dbc_id)` — **full parity** with the old store, confirmed) and `source_->buffers()` (the by-value buffer loops in `updateInternal` and the geo selection/label-gating paths don't care about map keying).
- The old id-keyed private slot `dataChanged(dbc_ids, reset, last)` becomes a plain method `applyChange(const std::vector<std::string>& names, bool reset, bool last)` that maps names→ids via `dbcont_man_.dbContentId(name)` and runs the existing body verbatim (`reset→resetData`; per-id `rebuildContent`; `last→contentRebuilt`; always `dataChanged_impl`). Every other method and `_impl` hook (`reset`, `update`, `redrawGeometry`, `reset_impl`, `rebuildContent_impl`, `contentRebuilt_impl`, `dataChanged_impl`, …) is **unchanged** — the id-keyed index parity is what lets the internals stay put.

**Base seam (`ViewDataWidget`) — base owns the provider.** `std::unique_ptr<DBContentItemProvider> item_provider_` + `setItemProvider(std::unique_ptr<…>)` / `hasItemProvider()`. New framework input entry: `updateFromSource(const DBContentDataSet& source, names, reset, last)` → `item_provider_->setSource(&source); item_provider_->applyChange(names, reset, last)`. `clearData()` → `item_provider_->reset()` + `setSource(nullptr)`. The geo-specific **output** signals (`requestFrameUpdate→drawSlot`, `requestCountUpdate→updateCountSlot`, `requestSelectionUpdate→requestSelectionUpdateSlot`) are wired in the **derived** widget *before* it hands the instance to the base via `setItemProvider(std::move(...))`.

**ViewManager drives it (sole subscriber, one seam).** `sourceDataChangedSlot(names, reset, last)` already holds `current_source_` and the `processing_data_` / queued-deferral guard ([viewmanager.cpp:946](src/view/viewmanager.cpp#L946)). Extend it to also call, per view, `view->dataWidget()->updateFromSource(*current_source_, names, reset, last)`. **Single unified finalize (decided):** honor the `last=true` event here — including the empty-names synthetic finalize, dropping the current `names.empty()` early-return for that case — so the provider's `contentRebuilt` fires from the source event, live and offline alike; `loadingDoneSlot` no longer finalizes the provider (it keeps driving the view's chrome/redraw). **Ordering requirement:** the source's `last=true` dispatch must reach the provider *before* the view's completion redraw, preserving today's "no empty intermediate OSG paint" (the guard already serializes the whole wipe→rebuild→finalize into one event-loop turn).

**Live.** `current_source_ = feed`; the feed's per-tick `emitChanged(all, reset=true, last=true)` flows through the *same* `sourceDataChangedSlot` → one atomic `applyChange` (wipe+rebuild+finalize) in a single guarded turn. The provider work in `liveReload_impl`/`clearData_impl` collapses into this one path.

**Then retire Path B.** Delete `DBContentManager::data_store_`, its `update`/`reset` calls ([dbcontentmanager.cpp:613,835,915,922](src/db/dbcontent/dbcontentmanager.cpp)), the `dataStore()` accessors, the `DBContentDataStore` class, and the dead `current_request_` member. `DBContentDataIndex` (already built on every `DBContentDataSet`) becomes the one index — no duplicate.

**Sub-stages (build + runtime-verify between 3 and 4):**
1. ✅ Reshape `DBContentItemProvider`/`GeometryItemProvider` to `DBContentManager&` + `setSource`/`applyChange` (compiles alongside the old store path; nothing rewired yet).
2. ✅ Add the base-owned provider + `updateFromSource` seam to `ViewDataWidget`; extend `ViewManager::sourceDataChangedSlot` to drive providers and honor the unified `last=true` finalize.
3. ✅ `GeographicViewDataWidget` constructs `GeometryItemProvider(dbMan, node, view)`, wires its output signals, `setItemProvider(...)`; drop the `data_store_` ctor arg + self-subscription. *(runtime-verified: geo view renders identically offline + live.)*
4. ✅ Deleted `data_store_`, `dataStore()`, `DBContentDataStore`, `current_request_`; the base provider + `GeometryItemProvider` now read only from the current source (`curBuffers()`/`curIndices()` facade). `readme_loading.md` updated.

This also sets up **step 8** for free: since every view already receives `updateFromSource(source, names, reset, last)`, porting a view to incremental intake is just "fill in its `updateFromSource_impl()` to read the source and drop its raw-buffer `updateData` path" — no per-view provider needed (only the geo view has one).

### 7. Batch consumers hold their own operation

`EvaluationManager` ([evaluationmanager.cpp:579](src/eval/evaluationmanager.cpp#L579)), `ReconstructorTask` (per-slice), `CreateARTASAssociationsTask`, `RadarPlotPositionCalculatorTask`, RT `get_data` ([dbcontent_commands.cpp](src/db/dbcontent/dbcontent_commands.cpp)), `AnalysisDataset`: each constructs a `LoadOperation`, issues `load(op)`, waits on `finishedSignal`/`wait()`, and reads `op->buffers()` (and, for free, `op->index()`/`op->accessor()`). Retires the enable/disable-distribution dance and the borrow-`data_`-then-`clearData()` pattern. All issue loads strictly sequentially (the Reconstructor slice loop starts slice N+1 from slice N's `finishedSignal`).

**RT `get_data` needs an extra guard:** RT commands bypass UI modality (they post `QMetaCallEvent`s directly from the asio runner thread and can be dispatched during the event-pumping a long load performs), so a mid-load `load()` would spin the wait preamble nested inside the outer load's pump. If `isLoading()`, reject or defer instead.

### 8. Filter clause library (`src/filter/`)

Removes **duplicated / state-coupled SQL generation**: eval hijacks the global FilterManager (`configureLoadFilters` → `disableAllFilters()` + `loadViewPointConditions()`, [evaluationmanager.cpp:628](src/eval/evaluationmanager.cpp#L628), clobbering the user's filters); reconstructor ([reconstructortask.cpp:633](src/task/reconstructor/reconstructortask.cpp#L633)), compass ([compass.cpp:1278](src/app/compass.cpp#L1278)) and ARTAS ([createartasassociationstask.cpp:270](src/task/assoc/createartasassociationstask.cpp#L270)) hand-concatenate SQL the Timestamp/Position filters already emit.

Fix = parameterized, state-decoupled builders exchanging `struct FilterClause { std::string sql; dbContent::VariableSet required_vars; }`, in **three layers** (only the top is polymorphic — `sqlFor` signatures are heterogeneous):

- **Leaf primitive (real dedup point):** refactor `DBFilterCondition::getConditionString` ([dbfiltercondition.cpp:99](src/filter/dbfiltercondition.cpp#L99)) — already encoding column resolution, operator/`BETWEEN`/`IS NULL`/`ABS`, read-set augmentation — into `makeClause(var/column, op, value(s), include_null)` (+ range / `IN`).
- **`sqlFor(...)` — per-filter, explicit params, non-polymorphic:** instance methods building from the primitive **without stored config** — `TimestampFilter::sqlFor(begin,end,dbcontent)`, `PositionFilter::sqlFor(bbox,dbcontent)`, `UTNFilter::sqlFor(utns,dbcontent)`. Called by name (never through a base pointer, so heterogeneous signatures are fine).
- **`getClause(dbcontent) -> FilterClause` — the one uniform virtual, for the view path:** reads stored config and delegates to `sqlFor(...)`; replaces `getConditionString` (drops `bool& first`) and the `getSQLCondition` read-set side effect (`required_vars` becomes explicit output). Generic condition-list filters walk their conditions through the primitive, as today.

Plus a **datasource/line builder** (from `DBContextManager::ds_loading_wanted_`/`line_loading_wanted_`, preserving the `"1=0"` empty-set sentinel) and a trivial **combiner** (AND/OR + union of variable sets). Full predicate AST deferred until a filter-expression UI / serialized predicates are wanted. **Caveat:** clauses with no filter (ARTAS CAT062, ad-hoc WHERE) keep the leaf primitive or a `custom_filter_clause_`.

## Migration order

Buffer-ownership and the filter library are independent; do ownership first with a WHERE-**string** snapshot, swap in the library after. Steps landed somewhat out of order (4 and 5 before 3 finished) — status markers below reflect actual progress, not the numbering. **Legend:** ✅ done · ⏳ in progress · 🔲 not started.

1. ✅ **DONE.** Add `DBContentDataSet` (buffers + absorbed `DBContentDataIndex` + accessor + one signal) and `LoadOperation`. Extract `DBContentDataEngine` and move `load` into it (manager keeps a facade); `load(op)` fills `op` buffers. Capture the WHERE as today's `getSQLCondition`/`composeWhereClause` string. **Duplicate** `insert`/`deleteOlderThan` into the engine as working copies now — do **not** cut them from `DBContentManager` yet. The manager's existing `insertData`/`deleteDBContentData`/`processLiveModeSlot` path keeps driving live in production untouched; the engine simply gains the capability early so later steps can switch onto it. (Temporary duplication is deliberate — it de-risks the insert/live cutover.)
2. ✅ **DONE.** Apply the single-operation assumption: simplify the `load()` preamble to wait-only (drop `quitLoading()`, keep the spin, add `logwrn`); add the `isLoading()` guard to RT `get_data`.
3. ✅ **DONE.** **Unify distribution:** ViewManager is the sole subscriber to `current_source_->dataChangedSignal` and drives the views; `GeometryItemProvider` consumes `source.index()`/`source.buffers()`; `loadedDataSignal`/`distribute_data_`/`disable_data_distribution_` and the manager-owned store removed. Other views keep pulling `source.buffers()` via the raw-buffer path (their `updateFromSource_impl` incremental intake lands in step 8; only the geo view gets a provider). Selection carry-over + read-set composition moved to owners.
   - ✅ **3a** — dead distribution toggles (`distribute_data_`/`disable_data_distribution_`, `enable/disableDataDistribution`) removed.
   - ✅ **3b** — ViewManager owns `current_source_`, issues offline loads (`reload()`) + the live prime (an interim `primeLiveCache()`, later replaced by `reloadWindow`/`refreshDisplay` in 9d/9e), subscribes to `current_source_->dataChangedSignal` via `sourceDataChangedSlot`, drives views; `appModeSwitchSlot` swaps the source; `reload()` live-guard added. *(runtime-verified incl. live entry/pause/resume/stop + geo-view time-filter re-enable fix.)*
   - ✅ **3c** — the provider↔view-framework integration in §6a: `DBContentItemProvider`/`GeometryItemProvider` are source-fed (`setSource`/`applyChange`, `curBuffers()`/`curIndices()` facade) + base-owned by `ViewDataWidget` and framework-driven; `data_store_`/`DBContentDataStore`/`dataStore()`/`current_request_` deleted. *(runtime-verified offline + live.)*
4. ✅ **DONE.** Cut each batch consumer (Eval, Reconstructor, ARTAS, RadarPlot, RT `get_data`, AnalysisDataset) to its own `LoadOperation`; delete the enable/disable-distribution calls. *(runtime-verify still pending.)*
5. ✅ **DONE.** Insert path runs on the engine's `insert`/`deleteOlderThan`; the dead manager insert copy + `addInsertedDataToChache`/`cutCachedData`/`filterDataSources` removed; the engine announces fresh buffers via a new `insertedDataSignal(buffers)` the feed consumes. *(**Superseded by step 9 for live ownership:** the `LiveDataFeed` moved from ViewManager to the private `LiveController`; `primeLiveCache`/`processLiveTick`/`viewManager().liveFeed()` and the manager's thin `processLiveModeSlot` trigger no longer exist — see 9a/9d/9e for the final live design. This entry records the interim state.)*
6. ✅ **DONE.** Deleted `DBContentManager::data_` + `tmp_selected_rec_nums_`. `data()`/`loadedData()` delegate to `ViewManager::currentSource()->buffers()` (per-arrival side-effects — counts/status-chrome — now run in `ViewManager::sourceDataChangedSlot`; step 9b removed the manager's `addLoadedData`/`loadOpDataChangedSlot`). **Selection carry-over moved to ViewManager**: `captureSelection()` before the source swap in `reload()`, `applyCarriedSelection()` at the top of `sourceDataChangedSlot` (before views are driven, preserving the "geo sees selection at build" guarantee), `storeSelectedRecNums`/`clearSelectedRecNums` there too; `DBContentManager::storeSelectedRecNums` is a thin façade (avoids a filter→view dependency); the engine no longer restores. *(runtime-verify selection-across-reload + live + status widget pending.)*
7. 🔲 **TODO.** Land the filter-clause library; cut eval (drop the hijack) and reconstructor/compass/ARTAS (drop hand-rolled SQL) onto it. Unit-test SQL + required-vars parity before switching each site.
8. ✅ **DONE. Retired `ViewDataWidget::updateData()`.** `updateFromSource()` is the single data-delivery callback: the base mirrors `data_ = source.buffers()` (so `viewData()`/redraw/selection are undisturbed), feeds the item provider, and calls `updateFromSource_impl`. `ViewManager::sourceDataChangedSlot` drops the Path-A `loadedData` call; `View::loadedData`, `ViewDataWidget::updateData()`, and the pure-virtual `updateData_impl()` (+ all overrides) are removed. **Per view:** non-geo (table/scatter/grid = no-op; histogram via `VariableViewDataWidget` does `if (last) updateDataEvent(reset)` — a **full refresh on the load finalize** instead of per-arrival). Geo keeps its per-arrival min/max-time + draw, now gated on non-empty `viewData()` (reset handled first, so it fires even on an emptied live tick). Geo live-overload skipping moved to a `GeographicView::updateFromSource` override (`View::updateFromSource` made virtual) that returns early on overload, so the **whole** update (provider + finalize) is skipped. *(builds green + tests pass; runtime re-test of the views pending.)* Original framing: It is now an alternate of `updateFromSource()` — both deliver the same data. Today the non-geo views (table/histogram/scatter/grid) live entirely on `updateData()` → `data_` → `updateData_impl()`, while their `updateFromSource_impl()` are empty stubs; the geo view lives on `updateFromSource()` (its provider, base-fed from the source). Consolidate onto the **single** callback: move each non-geo view's `updateData_impl` body into its `updateFromSource_impl` (reading buffers from `source.buffers()` instead of the pushed `data_`), drop the Path-A `loadedData`→`updateData` call in `sourceDataChangedSlot` (and `View::loadedData`), and remove `updateData()`/`updateData_impl()`. `data_` can stay as a base-managed cache populated from `source.buffers()` inside `updateFromSource()`, so each view's existing `viewData()`/redraw/selection machinery is undisturbed — only the delivery entry point changes, and each view keeps its current **full-refresh** behavior. Two things are explicitly **out of scope** (available, not required): using `(names, reset, last)` for **incremental** partial rebuilds, and adding a per-view **`DBContentItemProvider`** — only the Geographic View has one, others may gain one eventually but there are no immediate plans.
9. **Ownership cleanup: `DBContentManager` → static model + *pure* load mechanism; `ViewManager` → thin owner + two controllers.** The manager ends up owning no dataset, no distribution, no load UX, no live orchestration; its `load(op, blocking)` is a two-line wrapper and the concurrency safety net lives in the engine. ViewManager owns `current_source_` + the distribution dispatch + the bookend signals, and delegates the load/live *lifecycles* to two collaborators. Sub-steps:
   - ✅ **9a** — **live driven by ViewManager.** `processLiveModeSlot` (LiveRunning guard + DB-bound `deleteOlderThan` + in-memory tick) moved onto ViewManager, fired by the engine's `insertedDataSignal` (via `insertedDataSlot`) and the ASTERIX watchdog (re-pointed to `ViewManager::processLiveModeSlot`); the manager's `processLiveModeSlot` is deleted; the engine only *announces* inserts.
   - ✅ **9b** — **load bookends → ViewManager + manager `load()` made pure.** `loadingStarted/DoneSignal` now live on ViewManager, driven off the operation's own `started/finishedSignal` (wired in `setCurrentSource`); MainWindow / TargetListWidget / the RT wait (`compass.viewmanager.loadingDoneSignal()`) re-pointed; the manager's versions removed. The single-op **safety net moved into `DBContentDataEngine::load()`** (covers batch consumers too); `DBContentManager::load(op, blocking)` is a pure wrapper (`loadBlocking` deleted). Removed from the manager: `clearData()` (callers → `viewManager().clearDataInViews()`; the delete path clears in `deleteData` before delegating), `addLoadedData`/`loadedData()`, `finishLoading`/`loadOpFinishedSlot`/`loadOpDataChangedSlot`, `loadingDone(DBContent&)`, and the whole progress dialog + wait cursor.
   - ✅ **9c** — **`LoadController`** (`src/view/loadcontroller.{h,cpp}`, ViewManager-owned): the view-load UX — modal progress dialog, wait cursor, two-phase progress (load 0..50 / view 50..100) — extracted out of ViewManager. ViewManager drives it at its dispatch points (`begin` on op-start, `beginViewPhase`/`advanceViewPhase` in the view loop, `end` on finish). `begin()` is **idempotent** and fired off the op's `startedSignal` (i.e. *after* the engine's single-op wait), so a superseded/overlapping load can no longer leak a dialog or unbalance the cursor. **Load-phase progress is driven directly off the op's `dataChangedSignal`** (an un-guarded `opDataChangedSlot`, connected in `begin`/disconnected in `end`), not off ViewManager's re-entrancy-deferrable `sourceDataChangedSlot` — a deferred arrival re-running `contentArrived` used to yank the bar back to 50. Also: `loadingDoneSlot` closes the dialog **outside** the `processing_data_` guard, so a deferred `sourceDataChangedSlot` (e.g. the geo view's) drains while the dialog is still up, matching the pre-refactor `finishLoading` ordering. *(runtime-verified offline: dialog/cursor/progress/cancel/selection + geo timing.)*
   - ✅ **9d** — **`LiveController`** (`src/view/livecontroller.{h,cpp}`, ViewManager-owned): owns the `LiveDataFeed` (constructed eagerly, cleared-not-recreated on stop), the engine's `insertedDataSignal` subscription, the read-set provider, the per-tick orchestration (`processLiveModeSlot` = DB bound + `processTick`), and the latency. The feed + both controllers are **private** (no outside accessor); the ASTERIX watchdog fires `ViewManager::forceLiveUpdate()` and the latency façade reads `ViewManager::hasMaxLatency()/maxLatency()`. **`compass.cpp::appMode()` collapsed** — the three-branch pause/resume/else block became one `importer.appModeSwitchSlot` + `emit appModeSwitchSignal`. **Live entry semantics (corrected from the original plan — pause keeps ingesting into the DB while the display freezes, since the engine emits `insertedDataSignal` only in `LiveRunning`):** fresh entry (Offline→Live) starts **blank**; **resume (Paused→Live) = `reloadWindow()`** — a harvest-only blocking load of `timestamp >= now - maxLiveDataAgeCache` that `seedFrom`-**replaces** the frozen feed (catching up the pause-accumulated data), then `refreshDisplay()` (feed `cutCachedData` + distribute, **no** DB delete on entry); exit clears. `reloadWindow` sets a `reloading_` flag around its event-pumping load so a pump-fired tick can't issue an overlapping `deleteOlderThan` (the crash) or lose a concurrent insert to the `seedFrom`; the every-tick DB delete on normal ticks is **kept unchanged** (not throttled). The primed-seed/`primeLiveCache`/`seedIfEmpty` machinery is deleted. *(runtime-verified live: fresh entry/pause/resume incl. long-pause window trim/stop + watchdog + latency; resume-overlap crash fixed.)*
   - ✅ **9e** — **`LiveController` explicit state machine + distinct live bookends.** `LiveController` now runs an explicit `enum class State { Stopped, Running, Paused }` driven by ViewManager through four transitions — `startSession()` (fresh entry → Running), `pauseSession()` (→ Paused), `resumeSession()` (`reloadWindow()` while still Paused → Running), `stopSession()` (→ Stopped + `clearFeed()`). The ad-hoc `reloading_` flag is **folded into the state**: `reloadWindow()` runs while `state_` stays Paused, so `processLiveModeSlot`'s single guard `if (!running()) return;` suppresses a pump-fired tick during the resume load (replacing the old `appMode()!=LiveRunning || reloading_` check — the controller no longer reaches into `compass_.appMode()`). `feed()`/`clearFeed()`/`reloadWindow()` are now **private** (only `feedPtr()`/`refreshDisplay()`/latency/`processLiveModeSlot` stay public). ViewManager gains **`beginLiveSession()`/`endLiveSession()`** — lean live bookends (view chrome + external `loadingStarted/DoneSignal`, none of the offline-op dialog/progress-phase/view-point/pump/selection-carry work) — so `loadingStartedSlot`/`loadingDoneSlot` stop being borrowed on live entry/exit and are now **purely offline-op-driven** (via the `LoadOperation` `started/finishedSignal` connections in `setCurrentSource`). `appModeSwitchSlot` restructured accordingly (explicit `LivePaused` branch → `pauseSession`). *(builds green; live re-test pending.)*
   - ✅ **9f** — **post-review robustness fixes (L1/L3).** **L1:** `DBContentDataEngine::deleteOlderThan` no longer `traced_assert(!delete_job_)` — it returns early with a `logwrn` when a bound-delete is still draining. Live runs this every tick and `delete_job_` clears only on the queued `deleteJobDoneSlot`, so rapid ticks could assert-crash; the DB bound is independent of the display cut (`cutCachedData`), so skipping one bound is harmless. **L3:** the engine is now **app-mode-free** — `finishInsertingSlot` always emits `insertedDataSignal` (the `appMode()` branch + `appmode.h` include removed); the live/offline gate moved to the consumer (`LiveController::insertedDataSlot`, gated on `appMode()==LiveRunning` to preserve the exact prior staging behavior through the resume pump). *(builds green; live re-test pending.)*

## How to do the rewrite

Rules that govern the migration steps above — not optional:

- **Nothing is left out.** Every behavior that exists today must exist at the end of the rewrite. No "wire up the main path now, handle the rest later", no stubbed slot, no `// TODO` standing in for a dropped feature, no quietly narrowed scope. Before removing an old code path, confirm its behavior is reproduced on the new one. In particular, carry over the easily-forgotten details: selection carry-over across loads, viewpoint-apply loads (`doViewPointAfterLoad`), the per-content progress ticks and the view-phase progress, `measure_db_performance_`, min/max timestamp tracking, live latency (`max_latency_`), the CAT063 sensor-status vars, the `utn`/assoc property add and `sortByProperty(timestamp)`, `filterDataSources`, `cutCachedData`, reload-needed state, the RT command surface, and the responsiveness guards (3 s pump threshold, `repaint()`-not-`processEvents()`, re-entrancy guard). Use the current behavior — and [readme_loading.md](src/db/dbcontent/readme_loading.md) — as the checklist; the doc is updated only *after* parity is confirmed.
- **Staged ≠ left out.** The migration is staged, but **every step lands complete and behavior-preserving** — the app is fully functional after each step, with no feature disabled awaiting a later one. Removals (e.g. deleting `data_`, `processLiveModeSlot`, the duplicate insert copy, and step 8's retirement of `ViewDataWidget::updateData()` in favour of `updateFromSource()`) happen only once their replacement is proven — never leaving a gap in between. Optional additive work (incremental `names`-based rebuilds, a future per-view `DBContentItemProvider`) lands *on top of* the preserved behaviour and is not on the critical path.
- **Comments stay compact.** Match the surrounding code's comment density. Header (`.h`) comments are terse — a one-line intent per class/method, no verbose block prose; the design narrative lives in this plan and `readme_loading.md`, not in headers. Prefer a short comment only where the *why* is non-obvious; delete commented-out dead code rather than carrying it forward.
- **Function-definition banner convention.** In `.cpp` files, precede each function definition with the repo's `/**\n */` banner (empty by default, matching e.g. `dbcontentdatastore.cpp`); add a one-line note inside it only where the behavior is non-obvious.

## Migration status / gap checklist

Living tracker of what the new classes reproduce vs. what `DBContentManager` still owns. `DBContentDataEngine` is **constructed and owned by the manager**, and the **load, insert, and delete paths all run through it**. **`ViewManager` owns `current_source_`**, issues offline loads + the live prime, and is the sole subscriber/dispatcher of `current_source_->dataChangedSignal` (Path A). The `LiveDataFeed` drives the live tick. Path B is retired (step 3c) and the **`data_` buffer mirror is deleted** (step 6) — `data()`/`loadedData()` delegate to `ViewManager::currentSource()`, and selection carry-over lives in ViewManager. The manager now owns no dataset; step 8 (retiring `ViewDataWidget::updateData()` in favour of the single `updateFromSource()` callback) is **done** — the remaining structural work is the filter-clause library (step 7). *(Legend: `[x]` done · `[~]` partial/transitional · `[ ]` not started.)*

**Done — implemented in the new classes**
- [x] Buffer-map ownership + lazily-built index/accessor + one `dataChangedSignal` (`DBContentDataSet` / `DBContentDataIndex`, absorbing `DBContentDataStore`'s index logic).
- [x] One-shot load state machine (`Created/Running/Done/Cancelled/Failed`), cooperative `cancel()`, event-pumping `wait()`, single-shot `started`/`finished` signals (`LoadOperation`).
- [x] Offline read orchestration (`DBContentDataEngine::load`): target resolve + WHERE (reused from manager), read-set core via the single `addStandardVariables(...,add_utn=false)`, read-job fan-out, `verify → transformVariables → selected_`, accumulate into the op, terminal `Done`/`Cancelled`.
- [x] Live in-memory tick (`LiveDataFeed`): prune-to-read-set / transform / `selected_` / merge / sort, `cutCachedData`, `filterDataSources`, `filterBuffers`, latency.
- [x] Single standard-variables source (`DBContentManager::addStandardVariables` + `add_utn_if_available`); `timestamp` promoted into the always-added core in the engine.
- [x] Engine constructed/owned by the manager; **delete path migrated** — `deleteData(json)` + `deleteOlderThan` + `finishDeleting`/`deleteJobDoneSlot` + `dataDeletedSignal` live in the engine; `DBContentManager::deleteData`/`deleteDBContentData` delegate and forward `dataDeletedSignal`.
- [x] **Offline load path (final, step 9b/9c)** — `DBContentManager::load(op, blocking)` is a pure two-line wrapper over `engine.load(op)`; the single-op **safety net lives in the engine**. `ViewManager` owns the offline cycle: it builds the `LoadOperation` (`reload()`), makes it `current_source_`, and drives the bookends (`loadingStarted/DoneSignal`) off the op's own `started/finishedSignal`. The `addLoadedData`/`finishLoading`/`data_`/`loadedData` bridge is **gone**. Load UX (dialog/cursor/two-phase progress) is the ViewManager-owned `LoadController`. *(runtime-verified offline.)*
- [x] **Live path (final, step 9a/9d/9e)** — the `LiveDataFeed` is owned by the **ViewManager-owned `LiveController`** (private; explicit `Stopped/Running/Paused` state machine). The engine announces inserts via `insertedDataSignal`; `LiveController::insertedDataSlot` (gated on `appMode()==LiveRunning`) stages + ticks. The manager's `processLiveModeSlot`, the `data_` mirror, and the `data_store_`/`loadedDataSignal` distribution are all **gone**; distribution rides the feed's own `dataChangedSignal` → `ViewManager::sourceDataChangedSlot`. *(runtime-verified live: fresh/pause/resume/stop.)*
- [x] **DB-persisted metadata consolidated into the engine (all DB I/O in one place)** — the write-path allocation counters (`max_rec_num_wo_dbcontid_`, `max_reftraj_track_num_`, as `std::optional`; `loadMax…`/`clearMaxNumbers` on open/close) and the dataset-extent metadata (`timestamp_min_/max_` + lat/long min/max; `loadMinMaxInfo`/`updateInsertMinMax`/`clearMinMaxInfo`) now live in `DBContentDataEngine`. The manager keeps thin façade accessors (`maxRecordNumberWODBContentID()`, `minMaxTimestamp()`, …) so all external readers (dbinterface rec-num allocation, gpstrail, eval, filters, reconstructor, Geographic View, status widgets) are untouched. `updateInsertMinMax` is now engine-internal (no more insert→manager callback); it announces via `emit dbcont_man_.dbContentStatusChanged()`. Live latency also delegated: manager `hasMaxLatency()`/`maxLatency()` forward to `ViewManager` → `LiveController` (the `max_latency_` mirror is gone). *(needs runtime test: offline load, insert/import, live)*

- [x] **Batch consumers cut to their own `LoadOperation` (step 4)** — all six (`AnalysisDataset`, `RadarPlotPositionCalculatorTask`, `CreateARTASAssociationsTask`, `EvaluationManager`, `ReconstructorTask`, RT `get_data`) now build a `LoadOperation`, call `dataEngine().load(op)` directly, and read `op->buffers()` — no more borrow-`data_` / `enable/disableDataDistribution` / `clearData()` dance, so a batch run no longer clobbers the user's view dataset. Read sets migrate verbatim (each already passed its bespoke set via `LoadRequest::read_set_`/`forContent`); eval's implicit `getReadSet` + `needs_additional_variables_` hook is preserved (the flag still brackets the engine `load()`, which calls `getReadSet` synchronously). Blocking consumers use `op->wait()`; async ones (RadarPlot/ARTAS/reconstructor) connect `op->finishedSignal`. RT `get_data` is now a synchronous `run_impl` (`op->wait()`, no signal wait condition) with an `isLoading()` guard (RT bypasses UI modality). `load_op_` cleanup verified on all exit paths (local op or member nulled after harvest; reconstructor cancel path nulls explicitly). *(needs runtime test: eval, reconstructor slices, ARTAS, radar-calc, analyze-datasource, RT get_data)*

**Outstanding — not yet reproduced/wired**

*A. Load side-effects — moved to their owners (ViewManager / LoadController / engine)*
- [x] Selection carry-over → **ViewManager** (`captureSelection`/`applyCarriedSelection`/`restoreSelectionInto`/`storeSelectedRecNums`/`clearSelectedRecNums` + `carried_selection_`); manager keeps only a thin `storeSelectedRecNums` façade; engine no longer restores.
- [x] Loaded counts → **ViewManager** (`sourceDataChangedSlot` calls `dbContextManager().setLoadedCounts(current_source_->loadedCounts())` per arrival, from the source index).
- [x] `doViewPointAfterLoad()` + wait cursor + progress dialog → **ViewManager `loadingDoneSlot` / `LoadController`**; `measure_db_performance_` → **engine** (`load()` starts / `finish()` stops the metrics).
- [x] Real `Failed` terminal state on DB read error (mechanism only; consumers don't react yet). `Job` now records a caught exception (`hasError()`/`errorMessage()` — previously swallowed); `DBContent::readJobDoneSlot` emits `readFailedSignal(name, error)` instead of a silent empty result; the engine's `contentReadFailedSlot` fails the whole load atomically (obsoletes the remaining pending reads, discards partial buffers), and `finish()` sets `Failed` (precedence over a concurrent `Cancelled`). `LoadOperation` carries a **`Result`** (`result()` — `Result::failed(msg)` on Failed, `succeeded()` otherwise), set before `setState` so a `finishedSignal` consumer sees it. The same `finishedSignal` fires on Failed, so waiters (`wait()`) wake identically. **Deferred:** consumer reactions (ViewManager error dialog, batch-consumer `state()==Failed` guards) — to be added later.

*B. Signals / distribution — mostly migrated in 3b*
- [x] **Path A → ViewManager.** ViewManager is the sole subscriber to `current_source_->dataChangedSignal` (`sourceDataChangedSlot`, with the `processing_data_` guard + queued deferral) and drives each view's `loadedData`. The old `loadedDataSignal` is **gone**; the data-sources status widget now refreshes off ViewManager's `dataDistributedSignal` (emitted on the offline finalize + each live tick).
- [x] `loadingStartedSignal`/`loadingDoneSignal` now **owned by ViewManager** — offline driven off the op's `started/finishedSignal` (via `setCurrentSource`), live driven by `beginLiveSession()`/`endLiveSession()`. The manager no longer emits them; MainWindow/TargetListWidget/RT waits connect to ViewManager.
- [x] **Path B → ViewManager-driven provider** (§6a). The `GeometryItemProvider` is base-owned by `GeographicViewDataWidget` and framework-driven: `ViewManager::sourceDataChangedSlot` calls `updateFromSource(*current_source_, names, reset, last)` → `setSource`/`applyChange`. `DBContentDataStore`/`data_store_`/`dataStore()`/`current_request_` deleted; the geo provider reads only from the current source.

*C. Insert path — DONE (sub-step B): insert moved into the engine*
- [x] `insertData`/`finishInserting` moved into `DBContentDataEngine` (`prepareInsert`, `updateDataSourcesBeforeInsert`, ds/counts signals, `DBContentInsertDBJob`, `finalizeInsert`, `insertDoneSignal`). Manager `insertData`/`insertInProgress` delegate; manager forwards `insertDoneSignal`. On live the engine feeds the feed + triggers the tick; `processLiveModeSlot` dropped `addInserted`. Dead `addInsertedDataToChache`/`cutCachedData`/`filterDataSources` + `insert_data_`/`insert_in_progress_`/`insert_job_` removed from the manager.
- [x] Insert min/max ts/pos calc — now fully engine-owned (`DBContentDataEngine::updateInsertMinMax()`, engine-internal, no manager callback; storage + load-on-open + persist all in the engine).

*D. Delete path — DONE (migrated into the engine; manager delegates + forwards)*
- [x] `DBContentDeleteDBJob` + `finishDeleting` + `dataDeletedSignal` into the engine.
- [x] `deleteData(json)` (arbitrary delete-by-criteria).

*E. Old paths still authoritative (reconcile/remove)*
- [x] `data_` deleted; `data()`/`loadedData()` delegate to `ViewManager::currentSource()->buffers()` (buffers live in the current source, not mirrored on the manager).
- [x] `DBContentDataStore` (old Path B) **deleted** in step 3c; `DBContentDataIndex` on the `DBContentDataSet` is now the one index feeding the geo provider.
- [x] **Read job kept inside `DBContent`** (design correction — symmetric with `update_job_`/`delete_job_`). `DBContent` owns `read_job_` via `loadInternal(read_set, where)` + `readJobDoneSlot` (buffer massaging: `transformVariables` + `selected_`) + `readJobObsoleteSlot` + `quitLoading`, and announces the finished buffer via `readDoneSignal(name, buffer)`. The **engine orchestrates**: `DBContentDataEngine::load` resolves the target set, composes the core-augmented read set (`addStandardVariables`, timestamp-in-core preserved) + WHERE, calls `obj.loadInternal(...)`, and collects each buffer into the `LoadOperation` via `contentReadDoneSlot` (tracks `pending_contents_`, drives op state; cancel → per-content `quitLoading`). Only the genuinely-dead `status()`/`loadedCount()` were removed (they queried the view layer for content status).
- [x] `DBContentManager::load(op, blocking)` is a pure wrapper over the engine; the safety net is engine-side; `quitLoading()` removed (cancels go via `dataEngine().cancelLoad()`).
- [x] `clearData()` removed from the manager — callers use `viewManager().clearDataInViews()`; a new op is a fresh empty set, so there is nothing to reconcile.
- [x] `DBContentManager::data()` **removed** — the displayed-buffer accessor moved to its owner as `ViewManager::currentBuffers()` (current source's buffers, or empty). All callers (labelgenerator, datasourcesstatuswidget, geo osglayermodel, ViewManager's own view-point code, test lab) repointed; the manager's db→view reach is gone.

*F. Live orchestration — owned by the ViewManager-side `LiveController`*
- [x] `LiveDataFeed` **owned by `LiveController`** (constructed eagerly, cleared-not-recreated on stop; the feed + controller are private). Read-set provider set there; fed by the engine's `insertedDataSignal` → `LiveController::insertedDataSlot` (gated on `appMode()==LiveRunning`) → `feed.addInserted`.
- [x] The manager's `processLiveModeSlot` is **deleted**. The tick is `LiveController::processLiveModeSlot` (DB bound via `deleteDBContentData`→engine, then `LiveDataFeed::processTick` = merge/trim/filter/latency); distribution rides the feed's own `dataChangedSignal` → `ViewManager::sourceDataChangedSlot`.
- [x] `ViewManager::appModeSwitchSlot` drives the live session via the `LiveController` state machine (`startSession`/`pauseSession`/`resumeSession`/`stopSession`) + swaps `current_source_` (feed ↔ null); `compass.cpp::appMode()` collapsed to one `importer.appModeSwitchSlot` + `emit appModeSwitchSignal`.
- [~] Live watchdog (`ASTERIXImportTask::checkDataReceivedSlot`) fires `ViewManager::forceLiveUpdate()` → `LiveController::processLiveModeSlot`; confirm the no-new-packets force-tick still fires at runtime.
- [x] `maxLatency()`/`hasMaxLatency()` are manager façades over `viewManager().hasMaxLatency()/maxLatency()` → `LiveController`; the geo-view overload check reads them unchanged.

*Intentionally staying in `DBContentManager` (not gaps — the static model):* DBContent definitions, meta-variables, targets/associations, data-source model, dialogs/widgets. (The min/max extent metadata + write-path counters moved to the engine as DB I/O, with façade accessors kept on the manager.)

## Blind spots explicitly handled

- **Read-set / SQL coupling:** the read set is a `LoadSpec` field (§2); clause builders yield required vars as first-class output, unioned with the resolved read set + core meta-vars by the engine before execution. **`timestamp` is promoted into the always-added core** (the load always orders by it), so no custom read set can omit it; `utn` stays optional.
- **Single standard-variables source:** one canonical `standardVariables(name)` replaces the three hardcoded copies (`addStandardVariables`, `loadInternal`, `addInsertedDataToChache`), so load / insert / feed / priming all agree by construction (§2).
- **Live read set is a pruning target, not a query** — the feed trims wide decoded inserts to the live view read set (a provider from ViewManager, same as the priming load's), then transforms + adds `selected_`; in-memory live filters need their vars to survive the prune (§5).
- **Datasource/line filter is not a FilterManager filter** — pulled into the same clause library or captured constraints are incomplete.
- **Filter-SQL duplication / global-state hijack** — removed via the shared builders (§8).
- **Single-operation assumption** — UI blocking + wait-plus-warning safety net (not assert); RT `get_data` gets an explicit `isLoading()` guard since it bypasses UI modality.
- **Path unification vs pacing** — ViewManager stays the sole dispatcher so the re-entrancy guard + 3 s pumping are preserved; providers are driven, not self-subscribed.
- **Index consistency/atomicity** — the data set rebuilds its index before emitting, so a queued slot sees buffers+index coherent (preserves today's no-empty-intermediate-state guarantee).
- **`Failed` state**, **operation/job lifetime** (manager holds `current_operation_` shared_ptr until jobs finish; buffers dropped if cancelled; single-op keeps `DBContent::read_job_`'s single slot valid).
- **Live DB-write vs display split** — engine owns `insert`/`deleteOlderThan` (DB writes); `LiveDataFeed` (view-side) is memory-only. A view-side object never issues DB writes.
- **Live watchdog** — the ASTERIX 1 s `checkDataReceivedSlot` force-tick ([asteriximporttask.cpp:1556](src/task/import/asterix/asteriximporttask.cpp#L1556)) now nudges the feed's `processTick` (via a signal) instead of `processLiveModeSlot`; confirm it still fires when no new packets arrive.
- **Live bookends** re-synthesized by ViewManager.
- **Incidental manager-signal consumers** move to `operationStarted/Finished`; verify each needs only lifecycle, not payload.
- **`wait()` pumps events** (guards preserved); **progress dialog** stays manager-owned.
- **Base-class discipline** — `DBContentDataSet` owns data+index only; leaking query concepts rebuilds the rejected design.

## Critical files

- **New:** `src/db/dbcontent/dbcontentdataset.{h,cpp}`, `src/db/dbcontent/loadoperation.{h,cpp}` (+ `LoadSpec`, superseding `LoadRequest`), `src/db/dbcontent/dbcontentdataengine.{h,cpp}` (load + insert + deleteOlderThan), `src/db/dbcontent/livedatafeed.{h,cpp}`, `src/filter/filterclause.{h,cpp}`. Per-view provider subclasses added incrementally under each view dir.
- **Heavily changed:** [dbcontentmanager.cpp/.h](src/db/dbcontent/dbcontentmanager.cpp) (shrink to static model + facades; remove `data_`/distribution/store; delegate load/insert/delete to engine), [viewmanager.cpp/.h](src/view/viewmanager.cpp) (sole dispatcher, unified `dataChanged`, own current-source, own+switch `LiveDataFeed` in `appModeSwitchSlot`), [view.cpp/.h](src/view/viewbase/view.cpp) (unified API + own provider), [dbcontentdatastore.cpp/.h](src/db/dbcontent/dbcontentdatastore.cpp) → `DBContentDataIndex` absorbed into the data set, [dbcontentitemprovider.cpp/.h](src/db/dbcontent/dbcontentitemprovider.cpp) (source from a data set; per-view), [geometryitemprovider.*](experimental_src/view/geographicview/geometry/geometryitemprovider.cpp), [asteriximporttask.cpp](src/task/import/asterix/asteriximporttask.cpp) (insert path now via the engine facade; live tick reaches the feed via `insertedDataSignal`), [filtermanager.cpp/.h](src/filter/filtermanager.cpp) + `DBFilter` subclasses, [dbfilter.cpp](src/filter/dbfilter.cpp), [dbfiltercondition.cpp](src/filter/dbfiltercondition.cpp), [compass.cpp](src/app/compass.cpp).
- **Consumer cutovers:** [evaluationmanager.cpp](src/eval/evaluationmanager.cpp), [reconstructortask.cpp](src/task/reconstructor/reconstructortask.cpp), [createartasassociationstask.cpp](src/task/assoc/createartasassociationstask.cpp), [radarplotpositioncalculatortask.cpp](src/task/), [dbcontent_commands.cpp](src/db/dbcontent/dbcontent_commands.cpp), [analysisdataset.cpp](src/task/analyze/analysisdataset.cpp), [mainwindow.cpp](src/app/client/mainwindow.cpp).
- Update [readme_loading.md](src/db/dbcontent/readme_loading.md).

## Verification

- **Unit tests** (`./build/bin/compass_tests`): clause-library parity (`FilterClause` SQL + required-vars vs. old output across Mode3A/Timestamp/UTN/Position/datasource-line/empty-set `1=0`); `LoadOperation` state transitions + single-use; spec/clause snapshot immutability; `DBContentDataSet` change-signal semantics (`reset`/`last`) and index/accessor coherence in the slot. Register in the module `unit_tests/CMakeLists.txt`.
- **RT/UI integration tests as a regression gate:** re-run the popup-menu / signal-injection UI tests after the ViewManager cutover — distribution now fires off the unified `dataChanged`, and the event-pumping that protects RT-command ordering must still hold.
- **End-to-end** (`/run` on a sample recording):
  - Offline: Load with filters + partial datasource/line selection; views + Geographic View render identically to pre-refactor; change a filter, Reload → new WHERE applied.
  - Cancel a large load mid-flight → `Cancelled`, clean views, subsequent load works.
  - Confirm the UI blocks during a load, RT `get_data` mid-load is rejected/deferred, and the "waited for running load" warning stays absent in normal operation.
  - Evaluation while a view dataset is loaded → eval neither clobbers the view dataset nor disturbs the user's filters.
  - Live: ASTERIX import → priming load + per-tick feed, window trim, no geo-view label-flicker regression, one `started → done` bookend per session.
  - Reconstructor time-sliced run + RT `get_data` → each reads its own operation's buffers without corrupting the view dataset.
