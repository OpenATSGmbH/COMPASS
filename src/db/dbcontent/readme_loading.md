# DBContent Loading

Describes how data is read from the database into the in-memory `Buffer`s that views and tasks operate on. A load always ends up constructing one `DBContentReadDBJob` per `DBContent` and submitting it to the `JobManager`'s DB worker (DB jobs are serialised behind one DB connection).

There is exactly **one low-level path**: `DBContent::loadFiltered` in [dbcontent.cpp:434](dbcontent.cpp#L434) — new `shared_ptr<DBContentReadDBJob>` → connect 3 signals → `compass_.jobManager().addDBJob(read_job_)`.

## Entry points

- **`DBContentManager::load(custom_filter_clause, measure_db_performance, custom_read_set)`** — [dbcontentmanager.cpp:392](dbcontentmanager.cpp#L392). Fan-out: iterates every DBContent where `loadable() && DBContextManager::loadingWanted()`, builds a `VariableSet` via `getReadSet(name)`, then calls `DBContent::load(...)` on each. This is the standard load. Callers:
  - [../../view/viewmanager.cpp:380](../../view/viewmanager.cpp#L380), [../../view/viewbase/view.cpp:613](../../view/viewbase/view.cpp#L613), [../../view/viewbase/viewwidget.cpp:396](../../view/viewbase/viewwidget.cpp#L396) — view reload.
  - [../../app/client/mainwindow.cpp:1362](../../app/client/mainwindow.cpp#L1362) — "Reload" menu action.
  - [../../app/compass.cpp:1224](../../app/compass.cpp#L1224), [../../app/compass.cpp:1253](../../app/compass.cpp#L1253) — CLI batch mode (followed by a `loadInProgress()` spin).
- **`DBContentManager::loadBlocking(...)`** — [dbcontentmanager.cpp:475](dbcontentmanager.cpp#L475). Thin wrapper: calls `load(...)` then spins `QCoreApplication::processEvents()` until `loading_done_`.
- **`DBContent::load(read_set, use_datasrc_filters, use_filters, custom_filter_clause)`** — [dbcontent.cpp:277](dbcontent.cpp#L277). Single-content load. Builds the `WHERE` clause from DBContext ds/line filters + FilterManager conditions + custom clause, then calls `loadFiltered`. Direct caller outside the manager: [dbcontent_commands.cpp:111-114](dbcontent_commands.cpp#L111-L114) — the `get_data` RT command.
- **`DBContent::loadFiltered(read_set, custom_filter_clause)`** — [dbcontent.cpp:414](dbcontent.cpp#L414). Lowest-level; always appends `rec_num`, `ds_id`, `line_id` to the read set, then creates the job.

No other code paths construct a `DBContentReadDBJob`.

## Workflow

**1. Start (`DBContentManager::load`).** `loading_done_ = false`; wait-cursor set. If a load is already running, iterates `quitLoading()` on each DBContent (sets `read_job_->setObsolete()`), spins events until `load_in_progress_` clears. Saves selected rec_nums, `clearData()` (wipes `data_`, resets `DBContentDataStore`, clears views), sets `load_in_progress_ = true`.

**2. Fan-out.** For every eligible DBContent it calls `DBContent::load(...)`. Emits **`loadingStartedSignal()`**. If no job got created, immediately calls `finishLoading()`.

**3. `DBContent::loadFiltered`.** Asserts `!read_job_`, creates the `DBContentReadDBJob`, connects the three job signals to `DBContent` slots **with `Qt::QueuedConnection`** (important — the job runs on a worker thread, slots run on the GUI thread), submits via `jobManager().addDBJob(...)`.

**4. Job execution (`DBContentReadDBJob::run_impl`, DB worker thread)** — [../job/dbcontentreaddbjob.cpp:50](../job/dbcontentreaddbjob.cpp#L50):
- `started_ = true`; fast-exit if already obsolete.
- `db_interface_.prepareRead(...)` → loops `readDataChunk(dbcontent_)` pulling chunks, accumulates them into `cached_buffer_` via `seizeBuffer`.
- When `last_buffer` is true (or obsolete), emits **`intermediateSignal(cached_buffer_)` once** — current code emits only on last chunk even though the loop collects many (see [../job/dbcontentreaddbjob.cpp:102](../job/dbcontentreaddbjob.cpp#L102) `if (last_buffer)`).
- `finalizeReadStatement()`; `done_ = true`. JobManager then emits the base-class `doneSignal` (or `obsoleteSignal`).

**5. Back on the GUI thread, `DBContent` slots:**
- `readJobIntermediateSlot(buffer)` — verifies properties, renames DB columns → variable names, adds the `selected_` bool property, calls `DBContentManager::addLoadedData({{name_, buffer}})`.
- `readJobDoneSlot()` / `readJobObsoleteSlot()` — `read_job_ = nullptr`; call `DBContentManager::loadingDone(*this)`.

Ordering caveat: `intermediateSignal` and `doneSignal` are both queued; `addLoadedData` also calls `loadingDone` as a belt-and-braces when `!isLoading()` ([dbcontent.cpp:770](dbcontent.cpp#L770)).

**6. `DBContentManager::addLoadedData`** — [dbcontentmanager.cpp:491](dbcontentmanager.cpp#L491). Seizes the buffer into `data_[name]`, updates `inserted/loaded` counts, restores selection, refreshes `DBContentDataStore`, emits **`loadedDataSignal(data_, false)`** — views and tasks pick up buffers here.

**7. `DBContentManager::loadingDone(object)`.** Polls every DBContent; if any `isLoading()`, bails. Otherwise `finishLoading()`.

**8. `DBContentManager::finishLoading`** — [dbcontentmanager.cpp:712](dbcontentmanager.cpp#L712). `load_in_progress_ = false`, `doViewPointAfterLoad`, logs perf, emits **`loadingDoneSignal()`**, restores cursor, `loading_done_ = true`.

## Observable signals

| Signal | Emitter | Meaning |
|---|---|---|
| `loadingStartedSignal()` | `DBContentManager` | Load has begun. |
| `loadedDataSignal(data, reset)` | `DBContentManager` | One DBContent's buffer has arrived (roughly once per content, because `intermediateSignal` fires only on last chunk). |
| `loadingDoneSignal()` | `DBContentManager` | All contents finished (or were aborted). |

Polling helpers:
- `DBContentManager::loadInProgress()` (bool).
- `DBContent::status()` — returns `"Idle" / "Queued" / "Started" / "Loading"` ([dbcontent.cpp:231](dbcontent.cpp#L231)).
- `DBContent::isLoading()` — `read_job_ != nullptr`.

## Progress granularity

There is **no per-row progress today**. The job increments `row_count_` internally ([../job/dbcontentreaddbjob.cpp:88](../job/dbcontentreaddbjob.cpp#L88)) but only emits the buffer once on the last chunk; no intermediate progress is surfaced.

Two realistic levels:
- **Coarse (no changes to the job):** N contents = N steps. Listen to `loadedDataSignal`, count which contents' buffers have arrived (or query `loaded_counts_`), compare to the set of contents with `loadable() && loadingWanted()`. Precedents: `DataSourcesStatusWidget` at [../../core/source/datasourcesstatuswidget.cpp:220](../../core/source/datasourcesstatuswidget.cpp#L220); `MainWindow` at [../../app/client/mainwindow.cpp:287](../../app/client/mainwindow.cpp#L287).
- **Fine (row-level %):** precompute expected totals per content (a `SELECT COUNT(*)` with the same filter clause before submitting, or reuse the inserted/loaded counts on `DBContextManager` as an upper bound), and either (a) emit buffer chunks from the job per chunk rather than only on last, or (b) tally `buffer->size()` inside `addLoadedData` for an after-the-fact arrival %. (b) is the minimally invasive option; (a) gives smoother feedback on very large contents.
