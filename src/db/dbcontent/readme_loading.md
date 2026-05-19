# DBContent Loading

Describes how data is read from the database into the in-memory `Buffer`s that views and tasks operate on. Every load runs through one entry point on `DBContentManager` and produces one `DBContentReadDBJob` per `DBContent` (DB jobs serialise behind the single DB connection in the JobManager).

## Single entry point

```cpp
DBContentManager::load(const LoadRequest& req);
DBContentManager::loadBlocking(const LoadRequest& req, unsigned sleep_ms = 1);
```

`LoadRequest` ([loadrequest.h](loadrequest.h)) is a simple struct:

| Field | Default | Meaning |
|---|---|---|
| `dbcontents_` | `{}` | `{"*"}` = all eligible (`loadable() && loadingWanted()`); explicit names = exactly those (still gated by `loadable()`); empty = no-op load |
| `apply_datasrc_filters_` | `true` | Apply DBContext ds/line filters to WHERE |
| `apply_view_filters_` | `true` | Apply FilterManager conditions to WHERE (and let filters add required vars to read set) |
| `custom_filter_clause_` | unset | Per-content `std::function<std::string(name)>` AND-ed after the filter clauses |
| `read_set_` | unset | Per-content `std::function<VariableSet(name)>`; defaults to `mgr.getReadSet(name)` |
| `show_status_` | `true` | Manager owns a `QProgressDialog` (one tick per content). Set `false` if the caller owns its own progress UI |
| `cancellable_` | `true` | Dialog Cancel button calls `quitLoading()` |
| `measure_db_performance_` | `false` | Enables `DBInterface` perf metrics for this load |

Three factories cover the common shapes:

```cpp
LoadRequest::standard();                                      // fan-out, all filters on
LoadRequest::withFilter(std::string clause);                  // fan-out + uniform clause
LoadRequest::forContent(std::string name, VariableSet rs,
                        std::string clause = "");              // single content, no filters
```

Default-constructed `LoadRequest{}` loads nothing - the `{"*"}` sentinel must be explicit. This makes accidental empty-set bugs benign.

## Workflow

**1. Start.** `load(req)` saves the request, sets the wait cursor, and if a previous load is still running iterates `quitLoading()` on each content and spins `processEvents()` until `load_in_progress_` clears. Then `saveSelectedRecNums()`, `clearData()`, `load_in_progress_ = true`. If `req.show_status_` and the resolved target set is non-empty, a `QProgressDialog` is created with `setMinimumDuration(500)` (so fast loads don't flicker); the Cancel button is wired to `quitLoading()` when `req.cancellable_`.

**2. Resolve target set.** `resolveTargetSet(req)` returns the names to load: if `req.dbcontents_ == {"*"}`, all `loadable() && loadingWanted()` contents; otherwise the explicit set intersected with `loadable()`.

**3. Per-content fan-out.** For each target name the manager:
- gets the read set (`req.read_set_(name)` or `getReadSet(name)`);
- composes WHERE in `composeWhereClause(name, req, read_set)`: DBContext ds/line filter (if `apply_datasrc_filters_`), then `FilterManager::getSQLCondition(name, read_set)` (which may add filter-required vars to `read_set`), then `req.custom_filter_clause_(name)` - AND-ed in that order;
- calls `DBContent::loadInternal(read_set, where)`.

`loadInternal` is private and only callable by the manager (`friend class DBContentManager;`). It appends `rec_num`, `ds_id`, `line_id` to the read set, creates the `DBContentReadDBJob`, connects its `doneSignal` and `obsoleteSignal` to `DBContent` slots with `Qt::QueuedConnection`, and submits via `jobManager().addDBJob(...)`.

After the loop the manager emits **`loadingStartedSignal()`**. If no jobs were created, `finishLoading()` is invoked immediately.

**4. Job execution (`DBContentReadDBJob::run_impl`, DB worker thread)** - `prepareRead` → loop `readDataChunk` accumulating into `cached_buffer_` via `seizeBuffer` → on `last_buffer` break out → `finalizeReadStatement` → `done_ = true`. If canceled mid-loop, `cached_buffer_` is nulled. JobManager then fires the base-class `doneSignal()` on the GUI thread.

**5. `DBContent::readJobDoneSlot`** (GUI thread). Calls `read_job_->takeBuffer()` to retrieve the accumulated buffer; if non-empty, verifies the read-list properties, runs `buffer_utils::transformVariables` (renames DB columns to variable names), adds the `selected_` bool property, and hands the buffer to `DBContentManager::addLoadedData({{name, buffer}})`. Then `read_job_ = nullptr; dbcont_manager_.loadingDone(*this)`.

`readJobObsoleteSlot` is declared but never reached today (JobManager only emits `doneSignal`).

**6. `DBContentManager::addLoadedData`** seizes the buffer into `data_[name]`, updates `inserted/loaded` counts, restores selection, calls `data_store_->update(changed_dbc_contents)` (incremental, per-content), emits **`loadedDataSignal(data, false)`**, and advances the progress dialog by one tick. Two observers fan out from this point - see *Two parallel observers* below.

**7. `DBContentManager::loadingDone(object)`** polls every DBContent; if any `isLoading()`, returns. Otherwise calls `finishLoading()`.

**8. `finishLoading`** closes/clears the progress dialog, resets counters, calls `doViewPointAfterLoad`, logs perf metrics, emits **`loadingDoneSignal()`**, restores cursor, sets `loading_done_ = true`, clears `current_request_`.

## Observable signals

| Signal | Emitter | Meaning |
|---|---|---|
| `loadingStartedSignal()` | `DBContentManager` | Load has begun. |
| `loadedDataSignal(data, reset)` | `DBContentManager` | One DBContent's buffer has arrived (once per content; the buffer is delivered in full on `doneSignal`, no streaming). |
| `loadingDoneSignal()` | `DBContentManager` | All contents finished (or were aborted). |

Polling helpers: `DBContentManager::loadInProgress()`, `DBContent::status()`, `DBContent::isLoading()`.

`loadBlocking` is a convenience spin-wait wrapper around `load(req)` for callers that genuinely need to block (app-mode switch, EvaluationManager, ViewManager viewpoint apply). Treat it as a future-removal target - async + signal-driven continuation is preferred.

## Two parallel observers

After step 6 there are **two** distribution paths that fan out from the same data:

**Path A - manager-signal path → `ViewManager` → `View`s.** `ViewManager` is connected to `loadingStartedSignal` / `loadedDataSignal` / `loadingDoneSignal` and dispatches `View::loadingStarted()` / `View::loadedData(data, reset)` / `View::loadingDone()` to every registered view (see *View distribution* below).

**Path B - data-store path → `DBContentItemProvider` subclasses.** `DBContentManager` owns a `DBContentDataStore` ([dbcontentdatastore.h](dbcontentdatastore.h)) holding the buffer map plus a precomputed `dbc_id → ds_id → line_id → buffer indices` index. It exposes three signals:

| Signal | Emitted by | Meaning |
|---|---|---|
| `dataResetSignal()` | `DBContentDataStore::reset()` | Store cleared. |
| `dataChangedSignal(dbc_id)` | `DBContentDataStore::update(name, buf, true)` | One content's buffer + index rebuilt. |
| `dataRefreshedSignal()` | `DBContentDataStore::update()` *and* (queued) on `DBContentManager::loadingDoneSignal` | Refresh complete. |

Subclasses of `DBContentItemProvider` (e.g. `GeometryItemProvider` for the Geographic View) connect to all three with `Qt::QueuedConnection` when constructed with `auto_update=true`. `dataChanged(dbc_id)` walks the data-store indices and (re)builds per-(dbc, ds, line) `ItemGroup`s; `dataRefreshed()` finalises them. From there, downstream signals (e.g. `layersResetSignal` / `layersChangedSignal` in the Geographic View) drive the visual rebuild - independent of `View::loadedData()`.

In a normal load both paths fire concurrently:
- step 6 calls `data_store_->update(changed_dbc_contents)` → Path B, then emits `loadedDataSignal` → Path A;
- step 8 emits `loadingDoneSignal` → Path A, and the data store's queued connection re-emits it as `dataRefreshedSignal` → Path B.

In the live-mode update only Path B fires - see next section.

## Live-mode update (`processLiveModeSlot`)

Driven once per second from `ASTERIXImportTask::checkDataReceivedSlot` via the import → manager chain. It is **not** a load - no DB job runs and no `LoadRequest` is built. The freshly inserted buffers (`insert_data_`) are merged into `data_` and redistributed in place.

Sequence ([dbcontentmanager.cpp:1288](dbcontentmanager.cpp#L1288)):

1. Compute per-DBContent min timestamp for the latency log.
2. `deleteDBContentData(old_time)` - drop rows older than the live cache window.
3. `addInsertedDataToChache()` - move freshly inserted buffers into `data_`.
4. `cutCachedData()`, `filterDataSources()`, optional `filterManager().filterBuffers(data_)`.
5. **Distribute** - `data_store_->reset()`; if `data_.size()` then `data_store_->update()`, else `viewManager().clearDataInViews()`.
6. `updateNumLoadedCounts()`; update `max_latency_`.

Step 5 fires **only** Path B: `data_store_->update()` triggers `dataChangedSignal(dbc_id)` per content and then `dataRefreshedSignal()` - providers rebuild item groups and views consuming the data store (currently the Geographic View via `GeometryItemProvider`) refresh.

`loadingStartedSignal` / `loadedDataSignal` / `loadingDoneSignal` are **not** emitted (the old `emit loadedDataSignal(data_, true)` is intentionally retired, see commit `ad038381`). Consequence: in live mode `ViewManager::loaded*Slot` / `loading*Slot` never run, and *anything* a view does in `loadingStarted()` / `loadedData(data, reset)` / `loadingDone()` is bypassed.

### What the live path does not do today

For the Geographic View specifically, `GeographicView::loadedData(...)` ([geographicview.cpp:376](../../../experimental_src/view/geographicview/geographicview.cpp#L376)) is the home of:
- overload detection - `num_packets_in_processing` and `maxLatency()` thresholds, sets/clears `overload_detected_` and the overlay text;
- `View::loadedData` → `GeographicViewDataWidget::updateData_impl` - `TimeFilterWidget::updateMinMaxTime`, `timestamp_drawn_*` recompute, first-load `zoomToLoadedData()`, `updateInfoText`, `updateStatusMessage`, `drawSlot()`;
- `label_generator_.updateAvailableLabelLines()` after each tick.

None of this runs in live mode now. The geometry tree IS rebuilt (Path B does that), but the surrounding view chrome - time-window scrubber, overload overlay, info/status panels, per-tick draw kick - does not refresh on the 1 Hz tick. New layers can also sit unrendered until something else fires `drawSlot`.

### Other rough edges in the live path

- **Double reset.** Step 5 calls `data_store_->reset()` unconditionally; if data exists, `data_store_->update()` then calls `reset()` again as its first step. `dataResetSignal` is emitted twice and `DBContentItemProvider::reset` (with subclass `reset_impl()`, e.g. clearing `group_layers_`) runs twice per tick.
- **No `dataRefreshedSignal` from `loadingDoneSignal`** - that connection in `DBContentDataStore` only fires for `load()`-driven refreshes. Live mode relies on the explicit emit at the end of `update()`.
- **Stale code in `GeographicViewDataWidget::updateData_impl`** (lines 581-604) - commented-out `osg_layer_model_->processBuffers(...)` references `OSGLayerModel`, which is no longer the live geometry path; the active type is `GeographicViewLayerModel` driven by `GeometryItemProvider`. Dead block, safe to delete.

## Progress granularity

Coarse only: one tick per DBContent, advanced from `addLoadedData`. There is no per-row / per-chunk progress signal. Two paths to fine-grained feedback if needed later:
- emit per-chunk from the job (instead of accumulating into `cached_buffer_` and delivering once on done) and tally rows in `addLoadedData`;
- precompute expected totals via `SELECT COUNT(*)` with the same WHERE before submitting and report arrival % against that.

## View distribution (`ViewManager`)

`ViewManager` is the canonical consumer of the three lifecycle signals; it fans them out to every registered `View`:

| Slot | Per-view call | Notes |
|---|---|---|
| `loadingStartedSlot` | `view->loadingStarted()` | Resets `reload_needed_`. Skipped when `disable_data_distribution_` is set (used during processing-only loads). |
| `loadedDataSlot(data, reset)` | `view->loadedData(data, reset)` | Fires once per DBContent (one buffer arrival per signal). |
| `loadingDoneSlot` | `view->loadingDone()` | Heavy work - view rebuilds, scene-graph re-zooms, table-model index rebuilds. |

### Re-entrancy guard

All three slots share a single `processing_data_` flag, scoped via `QScopedValueRollback` for the duration of the per-view loop. If any of them is invoked while `processing_data_` is set, it re-posts itself via `Qt::QueuedConnection` and returns immediately. This preserves the contracted ordering `started → loaded* → done` even when the slots are interrupted by event-loop pumping (see below).

### Threshold-based event pumping in `loadingDoneSlot`

`loadingDoneSlot` is the single place where a load can take long enough on the GUI thread for the window manager to flag the application as unresponsive (`_NET_WM_PING` timeout on X11 / similar on Wayland). For wide loads with several heavy views the per-view loop can run for tens of seconds - the original symptom was a Linux "Application is not responding - Wait / Force Quit" dialog mid-load.

To keep the GUI responsive, the loop calls

```cpp
QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents
                              | QEventLoop::ExcludeSocketNotifiers);
```

after each `view->loadingDone()` - but only **after the loop has been running longer than `pump_threshold_ms` (3 s)**. The threshold matters: pumping unconditionally lets queued events (notably RT commands posted from the asio-driven `RTCommandManager` runner thread) interleave with view dispatch. UI integration tests waiting on a single view's `viewRefreshed` signal would then issue their next command while later views in the loop are still being processed, breaking signal-injection assumptions (e.g. popup-menu tests). The threshold keeps short loads (typical UI tests, ~1–2 s total) below the pump line entirely, so the loop runs uninterrupted; long loads still get periodic pumping once they pass 3 s.

Excluded event categories (`UserInput`, `SocketNotifiers`) further narrow what can fire during a pump:
- no menu/button slot from a real user click
- no Qt `QSocketNotifier` (RT commands from the boost::asio session bypass this - they post `QMetaCallEvent`s directly, hence the additional threshold and re-entrancy guard above)

What still fires during a pump: paint events, timer events, WM ping replies, internal queued slots. The re-entrancy guard ensures that if a queued `loadedDataSlot` / `loadingDoneSlot` happens to be among them, it defers cleanly instead of running mid-iteration.

`loadingStartedSlot` and `loadedDataSlot` do **not** pump. The chunk-by-chunk delivery of `loadedDataSignal` from the worker thread already yields the event loop between contents; pumping inside their per-view loops adds no responsiveness gain and used to allow `loadedDataSlot` to be re-posted past a queued `loadingDoneSlot`, breaking the ordering contract.

### Progress dialog updates use `repaint()`, not `processEvents()`

`DBContentManager::beginViewProgressPhase` and `advanceViewProgress` are called from inside the `loadingDoneSlot` loop after each view, to advance the `QProgressDialog` value. The natural choice would be `QCoreApplication::processEvents()` after `setValue(...)` so the dialog repaints - but that would re-introduce the same problem the threshold above avoids: an unconditional pump on every progress tick dispatches all queued events, including RT commands waiting on the main-thread queue. A regression of the popup-injection failure was traced to exactly this path.

The helpers therefore call `progress_dialog_->repaint()` instead - a synchronous paint of the dialog widget, with no event-queue dispatch. The dialog updates visually without pumping. The single coarse-grained pump in `loadingDoneSlot` (gated by the 3 s threshold) remains the only place where queued events get a chance to run during the loop.

Treat unconditional `processEvents()` calls inside the load lifecycle as suspect; prefer `repaint()` on the specific widget that needs to update.

## Migrating away from per-content loads

`DBContent::loadInternal` is private. Tasks that previously called `DBContent::load(read_set, ...)` directly (Reconstructor, RadarPlot, ARTAS, RT `get_data`) now build a `LoadRequest` and call the manager. Patterns:

- **Time-sliced loads (Reconstructor).** Build one `LoadRequest` per slice with `dbcontents_` populated explicitly (the slice's targets), `apply_*_filters_=false`, `read_set_` and `custom_filter_clause_` lambdas for per-content variation, `show_status_=false`, `cancellable_=false`. The slice-to-slice iteration is driven by listening to `loadingDoneSignal` and re-issuing `load(req)` for the next slice.
- **Single content (RT `get_data`).** `LoadRequest::forContent(name, rs, clause)` plus `show_status_=false`, `cancellable_=false`.
- **Restricted fan-out (ARTAS, RadarPlot).** Set `dbcontents_` to the explicit target names, `apply_*_filters_=false`, `read_set_` lambda, optionally `custom_filter_clause_` lambda for per-content WHERE.
