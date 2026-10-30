# Layer tree rework

State handling of the layer tree (checked state, expansion, selection, item config).
Status of the completed work, the step plan for the open work, future improvements (weak spots),
architecture notes and decisions.

The work came in two rounds: steps 1 - 8 (and R1 - R3) built a remembered state per key (session
map) around trees that were destroyed and rebuilt; the update rework (U0 - U6, R4, R5) then made the
producers update the tree in place and removed the session map again. The sections "Done" and "Step
plan" describe the first round as it was built, the concepts below the current state. The section
"Target report context menu in live mode" (M1 - M4) enables that menu in live running.

## Concepts (as implemented)

- **Update in place** - producers configure the tree once and update it via
  `LayerTreeItem::update()` / `LayerTreeModel::updateSubtree()` / `updateAll()`: dynamic items
  answer `childKeys()` and create / refresh their children (`createChild()`, `updateChild()`), the
  base keeps the matching children, moves, inserts and removes with scoped notifications (U1).
  Kept rows keep everything (checked state on the item, expansion, selection, scroll in the view),
  removed rows are forgotten.
- **Item keys** - `updateKey()` matches the children on updates (default `stateKeyId()`, else the
  name; e.g. FFT runtime id, annotation `#n` keys); `stateKey()` (not virtual) keys the config: the
  parent's key plus the item's escaped segment, `stateKeyId()` if set (default `persistenceId()`),
  else the name (`#n` for repeated sibling names). Duplicate keys are logged.
- **Item config** - `LayerTreeState` (owned by the model, `LayerTreeModel::state()`) holds the bound
  config root (`bindConfig()`, flat `{ key: {...} }`, geographic view: `layer_config`) and the
  effective-hidden ids. `config()` / `config() const` per item; `persistsSubtreeToConfig()` persists
  the checked state of a whole subtree, written on every change, applied on attach and on
  `configChanged()` (reconfigure, preset). `rename()` moves the configs of the subtree. Hooks
  `onAttached()` / `onConfigChanged()`.
- **Effective-hidden ids** - `persistenceId()` of the items hidden themselves or via an ancestor,
  read by the views' data filters (worker threads via a main-thread snapshot), kept until database
  close.
- **Stale mode** - a reload keeps the Data rows: payloads detached, rows greyed with a wait icon
  (optional), partial updates skipped, the final update rebinds (U2).
- **Panel view state** - new rows get the default expansion policy; a policy change moves the rows
  following the old default; a destroyed panel leaves its view state on the items
  (`LayerTreeItem::ViewHandover`) for the next panel on the same model (U5).
- **Geographic drawables** - the OSG content lives in `Drawable*Collection`s owned by
  `GeographicViewDrawables`, the tree items are thin views by id (U4).

## Done

First round (session map era). Items marked (U5) were removed again by the update rework.

- [x] Key, session map (U5), panel snapshot/restore (U5), default expansion policy
- [x] Store only deviations from the defaults (U5)
- [x] Persist flags (`PersistHidden`, `PersistExpansion`, `PersistSelection`) (U5)
- [x] Pruning of missing items (`pruneItemStates()` / `doPruneItem()`) (U5)
- [x] Distributed item config in the base
- [x] Geographic view: OSG context items moved to the base mechanisms, legacy `layer_config`
      sections removed (one-time loss of the old visibility and ring/label settings)
- [x] Step 5: color mode expansion of the Data subtree in all views as the panel's default policy
- [x] R1: `LayerTreeState` extracted from `LayerTreeModel`
- [x] R2: color mode wiring out of `ViewConfigWidget`, color mode names central in `ColorProvider`
- [x] R3: color mode to depth mapping moved to `ViewLayerPanelWidget::dataExpansionDepth()`
- [x] Step 6: unpaired `layoutChanged` replaced by a viewport repaint
- [x] Step 7: scoped notifications instead of model resets in the geographic view
- [x] Step 4: subtree-scoped pruning, the same in all views (U5)
- [x] Step 2: stable, escaped keys (`stateKeyId()`), data source groups by id (geographic view)
- [x] Step 3: renamed items keep their remembered states (`rename()`, sector layer rename signal;
      since U5 only the configs move, the sector layers keep their session id)
- [x] Step 1: context menu actions safe across rebuilds (`connectItemAction()`, context objects)

Second round: the update rework U0 - U6 with R4 / R5, see "Update rework" below.

Tests: `src/view/viewbase/unit_tests/test_layerpanelviewstate.cpp`, `test_layertreemodelstate.cpp`,
`test_layertreeitemconfig.cpp`, `test_layertreestate.cpp`, `test_layertreeupdate.cpp`,
`experimental_src/unit_tests/test_drawabletreeitems.cpp`, FFT runtime id in
`src/core/context/unit_tests/test_fft_matching.cpp`.

### Problems fixed

- Expansion was reset to the default expansion on every model reset (setMap, context changes,
  measurement add/delete, annotation changes): `modelChangedSignal` re-applied it via
  `LayerPanelWidget::autoExpand()` and `GeographicViewConfigWidget::expandLayersSlot()`, also
  re-expanding items the user had collapsed.
- DBContent children came back collapsed after every DBContent rebuild (every live tick).
- Selection and current item were lost on every rebuild and reset.
- Annotation subtrees were force-expanded completely on every refresh (`ViewLayerPanelWidget`).
- Checked state was handled by three separate mechanisms:
  - DBContent: `stored_unchecked_ids_` (persistence ids, unchecks only, applied in
    `refreshSubtree()` only, not on resets)
  - OSG context items: per-item `json&` / `bool&` references into `layer_config` (`show_layer`,
    `show_radars`, `show_fft`, `data_sources`, `show_layers`, `show_sectors`), dangling once the
    parameter is replaced (reconfigure, preset)
  - all other items: no memory
- DBContent leaves read their `persistenceId()` from the payload, which the provider may destroy
  before the leaf is removed.
- New leaves below an unchecked ancestor got no `onEffectiveHiddenChanged()` on insert (worked in
  the geographic view only via a side effect of `syncDBContentHiddenFromLayers()`).

### Base (`src/view/viewbase/layerpanel`)

As built in the first round; the session map parts (item states, `defaultHidden()`, persist flags,
pruning, panel snapshot / restore) are gone since U5, see the U sections.

- `LayerTreeItem`:
  - `stateKey()` (see Concepts), `defaultHidden()` captured at first attach (`setModel()`)
  - `setHidden()` / `hideSubtree()` / `showSubtree()` store the checked state (map + config)
  - `appendChild()` reports newly attached subtrees to the model (`subtreeAttached()`)
  - item config: `config()` (lazy entry, asserts attach), `config() const` (read-only, empty object
    if missing or not attached), `persistsSubtreeToConfig()` (override point),
    `persistHiddenStateToConfig()` (derived via the parent chain)
  - hooks: `onAttached()` (after attach, no structural changes), `onConfigChanged()`
- `LayerTreeModel`:
  - `ItemState` map, `itemState()` / `updateItemState()` (entries erased when empty),
    `setPendingCurrent()`, `clearPendingSelection()`
  - `storeHiddenState()`: deviation from `defaultHidden()` only, gated by `PersistHidden`
  - `subtreeAttached()`: restore (map, else config, else default), effective-hidden cascade below
    unchecked ancestors, duplicate key check, `onAttached()`; hidden signal deferred and emitted once
    at the end of `addRootItem()` / `refreshSubtree()` / `endResetModel()`
  - `setPersistFlags()` / `persists()`, disabled components are dropped from the map
  - `pruneItemStates()` + virtual `doPruneItem()`
  - `bindConfig()` (applies the config to existing items), `configRoot()`, scratch config while
    unbound, `configChanged()` (config wins over the map there, calls `onConfigChanged()`)
  - `clearStoredHiddenState()` (database close) clears the map and the effective-hidden ids
    (since R1: `state().clear()`)
  - `stored_unchecked_ids_` removed, `captureHiddenState()` keeps the effective-hidden ids only
  - new signal `viewStateHandedOverSignal()`
- `LayerPanelWidget`:
  - snapshot of expansion/selection on `rowsAboutToBeRemoved` / `modelAboutToBeReset` (connected
    before `setModel()`, so it runs before the selection model prunes), deviation-only expansion
  - restore on `rowsInserted` / `modelReset`, restored pending selection is consumed, a new user
    selection discards it
  - `setDefaultExpansion()` (policy for items without remembered expansion)
  - handover to a successor panel: snapshot in the destructor (external model only, guarded against
    a destroyed model) + `viewStateHandedOverSignal()`, restore at the end of `init()`
  - `enableAutoExpand()` / `autoExpand()` removed
- `ViewLayerPanelWidget`: annotation subtree auto-expansion replaced by a default policy
- `DBContentLeafItem`: persistence id cached at construction

### Geographic view (`experimental_src/view/geographicview`)

As built in the first round; since U4 the radar, FFT, RU, sector, map, measurement and annotation
items are thin views onto drawable collections (settings and root menus unchanged), see U4.

- `GeographicViewLayerModel`: binds `layer_config` at the start of the constructor (before the OSG
  items attach); pruning of the DBContent rebuild see step 4
- `GeographicViewConfigWidget`: default policy (up to `MaxExpansionDepth`, annotations fully),
  `expandLayersSlot()` removed
- `GeographicView`: `removeLegacyLayerConfig()` drops the old `radars` / `ffts` / `remote_units` /
  `sectors` sections (constructor + `onConfigurationChanged_impl()`, old presets still carry them),
  `configChanged()` on reconfigure
- `OSGContextRootItem`: `persistsSubtreeToConfig()` => the whole context subtree persists its
  checked state (incl. the context root itself, new)
- `OSGMapRootTreeItem`: fixed key `"map"` (its name follows the earth file)
- Radars / FFTs roots: settings (`show_range_rings` / `show_range_ring`, `show_labels`) via
  `config()`, const getters, settings pushed in `onAttached()` / `onConfigChanged()` / after context
  changes, unchecked by default via the constructor
- RU root: `show_symbols` / `show_labels` via `config()`, same pattern
- RU data source items keyed by data source id, RU items by unit index (constructor takes the index)
- Radar, FFT, RU, sector layer and sector items: `bool& show_` / `json& config_` / `layer_config_`,
  `show_layer` defaults, `showLayer()` and the constructor `setHidden(!show_)` removed

### Behavior notes (current)

- Persisted items write their current checked state to the config on every change (not only
  deviations).
- Kept items keep their checked state over states they bring along on an update (e.g. the `hidden`
  flag of a recurring viewpoint annotation); new items start with it.
- Rows that vanish and come back start with their defaults (config for persisted items), also after
  incomplete removals (F7).
- `#n` suffixes depend on sibling order; renames of kept items go through `rename()` (configs move),
  name changes of name-keyed items that are removed and recreated lose their configs.
- Not yet verified in the running application (F2).

## Step plan

First round, kept as the record of what was built and why; the mechanisms described here (session
map, pruning, capture / restore) were replaced by the update rework where marked in "Done".

Each step: build, full unit test suite, check in the application (geographic view in live mode,
reload, database close/open, application restart, preset apply where relevant).

### Step 1 - Context menu use-after-free on reload (done)

Done:

- `LayerTreeModel::findItem(key)`: item by state key, descends only into key prefixes
- `LayerTreeItem::connectItemAction(action, func)`: captures the model (`QPointer`) and the key,
  resolves the item when triggered => its successor after a rebuild (same key), nothing after a
  removal or with a destroyed model; for items that are no QObject
- `DBContentGroupItem::buildContextMenu()` (all views): Show / Hide All Children via
  `connectItemAction()`
- `GeometryItemProvider::addNodeActions()`: Show / Hide via `connectItemAction()` on the clicked
  item or the one resolved from the filter (target report menu)
- `OSGAnnotationsTreeItemInternal::buildContextMenu()` (zoom, export, opacity) and
  `OSGAnnotationsRootTreeItemViewPoint` (show / hide): `this` as context object
- internal annotations, item busy when destroyed: `renameItem()` guards itself with a `QPointer`
  across its modal dialog (own event loop); "Remove" (`deleteItemSignal`) reaches
  `removeAnnotation()` queued and guarded, so the item is not destroyed inside its own signal
  emission (as the measurements already did)
- already safe: the panel's "All" section (panel as context), the provider's item / target report
  actions (resolved by id / reference), all other OSG item menus (`this` as context), annotation
  leaves (the radio activation rebuilds the Data subtree, not the annotation subtree)
- open menus are not closed on a rebuild: entries of vanished items stay visible, triggering them
  does nothing
- tests: `findItem()` incl. a name prefix sibling, action after rebuild / removal / model destroyed

Original plan: menus run a nested event loop (`QMenu::exec()`), a rebuild during it (live tick, load completion,
runtime command) destroys the items the actions captured.

- `GeometryItemProvider::addNodeActions()`: Show/Hide capture the raw item (also when resolved from
  the filter for the target report menu) => capture the filter, resolve via `findDBContentItem()`
  when triggered.
- `DBContentGroupItem::buildContextMenu()`: lambdas capture `this` (no QObject, no context) =>
  capture the model and the state key, resolve when triggered (new model lookup by key).
- `OSGAnnotationsTreeItemInternal::buildContextMenu()`: context-less lambdas capturing `this` =>
  pass `this` as context object.

### Step 2 - Stable keys (done)

Done:

- `LayerTreeItem::stateKey()` no longer virtual, always `parent_key + "/" + segment`; items override
  `virtual stateKeyId()` (plain id among the siblings, default `persistenceId()`, empty => the name
  with `#n`) => no override can drop the parent prefix
- segments escaped (`\` `/` `#` => `\\` `\/` `\#`): names and ids may contain `/` (e.g. the `SAC/SIC`
  name of unknown data sources) and `#`; keeps the prefix match of the scoped pruning sound and
  separates a literal `x#1` from a repeated `x`
- `stateKeyId()` overrides: map root `"map"`, RU data sources (ds id), RUs (unit index) (keys
  unchanged), radars (ds id), sectors (sector id), DBContent data source groups (`ds:<id>` if known)
- DBContent groups: `stateKeyId()` empty => keyed by name (the key path covers the parents, the
  old `group:<path>` id stays as `persistenceId()` for the effective-hidden ids)
- `DBContentRootItem::LeafEntry::ds_id` (optional), `DBContentGroupItem::dsID()`;
  `findOrCreateGroup()` matches by `ds_id` if given, else by name
- geographic view passes the `ds_id` => same-named data sources get separate groups; also fixes
  `findDBContentItem()` / the group context menus, which took the ids from the first leaf of a
  merged group
- scope decision: tree keys only. Histogram, table, grid, scatter pass no `ds_id`: their layer ids
  (`<ds_type>:<ds_name>:L<n>:<dbcontent>`, parsed again for colors and filtering) already merge
  same-named data sources in the data
- keys changed once (remembered states lost once): radars, sectors, DBContent groups and leaves,
  names with `\` `/` `#`; persisted (config) entries affected: radars, sectors, context items with
  such characters
- tests: escaping (`/` in names and ids, literal `#`, scoped prune with a `/` name), data source
  groups by id / by name

Original plan:

- DBContent groups by `ds_id` instead of the data source name: fixes the merge of different data
  sources with the same name (`findOrCreateGroup()` matches by name) and makes their keys stable.
  Needs the ids in `DBContentRootItem::LeafEntry` and on the group items; affects all views.
- Radars by `ds_id` (radar names can repeat), sectors by sector id.
- Changes these keys once => remembered states of those items are lost once.

### Step 3 - Renaming (done)

Done:

- `LayerTreeState::renameItemStates(old_key, new_key)`: moves the session map and config entries of
  the key and of the keys below it (`old_key + "/"`), entries already present under the new keys are
  replaced
- `LayerTreeItem::rename(name)`: renames an attached item, moves its subtree's states, notifies the
  change; id-keyed items keep their key (nothing moves); `setName()` stays for detached items and
  items whose key does not follow the name (map root)
- internal annotations: `renameAnnotation()` uses `rename()`
- sector layers (renamed in `DBContextManager::renameSectorLayer()`, also from the context edit
  dialog): new `DBContextManager::sectorLayerRenamedSignal(old, new)` right before the
  `sectorsChangedSignal`; `OSGSectorRootTreeItem::sectorLayerRenamedSlot()` renames the live layer
  item first => states follow (incl. its sectors), the rebuild's view snapshot already uses the new
  key
- sectors and radars are id-keyed (step 2) => their renames need nothing
- limits: renames change the `#n` suffixes of same-named siblings only for name-keyed items with
  repeated names (internal annotations and sector layers have unique names)
- tests: `LayerTreeState` move incl. a name prefix sibling ("A/ab") and the config, item rename
  (subtree states move, restored after a rebuild with the new name, id-keyed item unchanged)

Original plan: renaming an item changes its key and orphans its map and config entries (e.g. sector layers,
which have no id; sectors are keyed by id since step 2).

- `LayerTreeModel::renameItemStates(old_key, new_key)`: moves the map and config entries of the
  item and its subtree (keys with the old key as path prefix).
- Called by the rename code of the affected items before the tree is rebuilt.
- Note: since step 4 all keys carry their parent's key as prefix => id-based keys move along too.

### Step 4 - Subtree-scoped pruning (done)

Done:

- `LayerTreeItem::stateKey()`: `parent_key + "/" + persistenceId()` for id-based items
- `LayerTreeState::prune(func, key_prefix)`, `LayerTreeModel::pruneItemStates(scope)` (nullptr =>
  whole tree)
- pruning in the base, no calls in derived items:
  - `LayerTreeItem::replaceChildren(build, complete = true)` prunes below the item after a complete
    rebuild (radars, FFTs, remote units, map layers, viewpoint annotations of all views, remove
    all of measurements / internal annotations); sectors pass `complete = sectorsLoaded()`
  - `removeChildNotified()` / `LayerTreeModel::removeChildItem()` drop the removed subtree's
    states (after the removal, the panel snapshots the removed rows); a re-added item starts with
    its defaults
  - `LayerTreeModel::refreshSubtree(parent, build, complete)`: `complete` without default, every
    caller decides; `replaceChildren()` forwards to it
- Data subtree, independent of reload vs. live tick:
  - `ViewLayerTreeModel::refreshDataSubtree(build)` used by all views (incl. geographic rebuild
    and clear), prunes only while the data is complete (`setDataComplete()`)
  - `ViewDataWidget` sets it: `updateFromSource(last)` => complete = `last` (set before the provider
    and the derived class rebuild), `clearData()` => incomplete (loading start, live session start)
  - every load operation emits the final update (done, cancelled, failed:
    `DBContentDataEngine`), every live tick emits a single final one (`LiveDataFeed`)
  - reload: clear => empty rebuild (kept) => final update => rebuild from the data (pruned);
    live tick: final update => rebuild (pruned)
  - views' layer model members / `ViewDataWidget::layerTreeModel()` typed `ViewLayerTreeModel*`
  - the scatter view's annotation display (`updateFromAnnotations()`, series from an annotation,
    not the loaded data) rebuilds without pruning (`VariableView::showsAnnotation()`) => the states
    of the data items survive the annotation display
  - known edges: a cancelled load (truncated data) and a failed load (buffers discarded) count as
    complete => states of the missing items are dropped
- tests: id keys below the parent, scoped prune incl. the scope's own entry and a name prefix
  sibling ("Data" vs "Database"), whole tree prune, `LayerTreeState` prefix range, removed items
  drop their subtree states, `replaceChildren()` complete / incomplete, `refreshDataSubtree()`
  with incomplete / complete data
- not checked in the application yet

Today the prune after a DBContent rebuild checks the whole tree, so a changed OSG item that is
missing at that moment (e.g. sectors not loaded yet) loses its session state.

Concept: every rebuild source prunes only its own subtree once its rebuild is complete.

Checked, feasible with hierarchical keys instead of recorded scopes:

- `stateKey()`: `persistenceId()` replaces only the item's own path segment
  (`parent_key + "/" + id`) => every key starts with its parent's key, any item can be a scope.
  Only DBContent items use `persistenceId()` (session map only, never in the config) => no loss of
  persisted states; the effective-hidden ids keep using the bare `persistenceId()`.
- `pruneItemStates(const LayerTreeItem* scope)`: entries in the key range `scope_key + "/"`
  (ordered map => `lower_bound`), checked against the items present below the scope.
  `pruneItemStates()` (whole tree) stays.
- No `isPruneScope()`, no scope stored per entry, no change to the writers.
- Pruning cannot run on every rebuild unconditionally: a reload first empties the Data subtree
  (`clearData()`, `clearDBContentSubtree()`) and rebuilds it later. Holding pruning between
  `loadingStarted()` and `loadingDone()` does not work either: a live session is one such bracket
  (`beginLiveSession()` / `endLiveSession()`). Hence the data-complete flag from the source's final
  update (see above).
- Same behavior in all views (before, the other views never pruned => entries of vanished items
  lived until database close).

The verify points:

- entries written before the scope root exists: impossible, keys derive from the attached parents
  and only attached items write entries
- items moving between scopes: the key changes => the state is lost (as for every path key), the
  old entry is pruned with its old scope
- entries of the scope root itself: outside the `scope_key + "/"` range => pruned by the parent
  scope
- `/` in names broke the prefix match (e.g. a top-level item "Data/x"): segments escaped since
  step 2

### Step 5 - Color mode expansion in all views (done)

Done (current state after R2 / R3): the Data subtree of every view (incl. geographic) is expanded
by default to the depth of the global color mode.

- `ViewLayerPanelWidget::setDataExpansion(depth)`: default expansion of the Data subtree (root +
  depth levels); a changed depth resets the remembered Data expansion to the new default
  (`LayerTreeModel::clearItemStates(subtree, flags)`), an unchanged depth keeps it
- `ViewLayerPanelWidget::followColorMode(COMPASS&)`: sets the depth from the color mode and follows
  `colorModeChangedSignal`; mapping in the static `dataExpansionDepth()` (DSType 0, DataSource 1,
  DataSourceLine 2, DBContent 3 levels below the root)
- `setDefaultExpansion()` (virtual, override in `ViewLayerPanelWidget`): items outside the Data
  subtree, default: annotations fully; `LayerPanelWidget::expansion_policy_` is the effective
  combined policy
- `LayerPanelWidget::setDefaultExpansion()` snapshots the current view state first, so deviations
  from the previous default survive a policy change; `captureViewStateAll()` /
  `restoreViewStateAll()` protected
- config widgets (histogram, scatter, grid, table, geographic) call `followColorMode()`; all but
  the geographic view show a `ColorModeLabel`; color mode names central in
  `context::ColorProvider::modeName()` / `modes()`
- geographic view: items outside Data keep `MaxExpansionDepth` / annotations fully (before, Data
  used the fixed `MaxExpansionDepth` too)
- removed: `applyDefaultExpansionForColorMode()`, the `layerTreeRebuiltSignal` expansion wiring
  and the color mode slots / labels of the config widgets

Original problem: histogram, scatter, grid and table call `applyDefaultExpansionForColorMode()` on
`layerTreeRebuiltSignal` and on color mode changes, after the panel's restore, and it only expands:

- collapsed items within the color mode depth are expanded again after every rebuild
- the panel's default policy only covers annotations there => color mode expansions are recorded
  as deviations, deeper rows stay expanded after switching to a shallower color mode

Fix:

- Color mode depth as the panel's default policy (depth relative to the DBContent root, mapping
  moved to a static helper on `DBContentRootItem`), combined with the annotations policy of
  `ViewLayerPanelWidget` (e.g. `ViewLayerPanelWidget::setDBContentExpansionDepth(int)`).
- On a color mode change: clear the expansion entries of the DBContent items, re-apply the policy.
- Remove `applyDefaultExpansionForColorMode()` and the `applyDefaultExpansionSlot()` wiring.

### Architecture refactoring (before steps 6 - 8)

#### R1 - Extract LayerTreeState from LayerTreeModel (done)

Done as planned: `layertreestate.h/.cpp`, model access via `state()`; the model keeps
`pruneItemStates()` / `doPruneItem()`, `clearItemStates(subtree)`, `bindConfig()` /
`configChanged()`, storing and restoring the checked state. Callers: `storedHiddenIds()` =>
`state().hiddenIds()`, `clearStoredHiddenState()` => `state().clear()`. Tests:
`test_layertreestate.cpp` (no Qt model).

`LayerTreeModel` holds the Qt model plus all state handling: session map, persist flags, pruning,
config binding / `configChanged()`, effective-hidden ids for data filtering, and the unused
viewpoint helpers `persistedHiddenIds()` / `applyPersistedHiddenIds()`.

- New class `LayerTreeState` (owned by the model): item state map, persist flags, pruning, config
  root, effective-hidden ids. The model delegates and keeps the Qt model duties plus the
  attach/restore orchestration.
- State logic testable without a Qt model.
- Remove the unused `persistedHiddenIds()` / `applyPersistedHiddenIds()` (model and panel).

#### R2 - Color mode wiring out of ViewConfigWidget (done)

Done as planned: `ViewLayerPanelWidget::followColorMode()`, `ColorModeLabel`
(`src/view/viewbase/colormodelabel.h/.cpp`), `ViewConfigWidget` back to its previous state.
Additionally the color mode names are central: `context::ColorProvider::modeName()` and
`modes()` (UI order), used by `ColorModeLabel` and the color mode combo of
`DataSourcesToolWidget`.

`ViewConfigWidget` (base of every view's config widget) now depends on the layer panel and the
COMPASS color mode via `initLayerPanel()` / `createColorModeLabel()`.

- `ViewLayerPanelWidget::followColorMode(COMPASS& compass)`: sets the Data expansion from the color
  mode and follows `colorModeChangedSignal` (dependency injection as elsewhere).
- Small `ColorModeLabel` widget instead of `createColorModeLabel()`.
- `ViewConfigWidget` back to generic, the config widgets create the label and call
  `followColorMode()`.

#### R3 - Color mode to depth mapping (done)

Done: `DBContentRootItem::expansionDepthForColorMode()` replaced by the public static
`ViewLayerPanelWidget::dataExpansionDepth(context::ColorProvider::Mode)` (switch over the enum,
same mapping), `dbcontentlayer.h/.cpp` free of color mode knowledge.

`DBContentRootItem::expansionDepthForColorMode()` puts COMPASS color mode knowledge into a tree
item.

- Move next to the color mode definition, or at least into `ViewLayerPanelWidget` (its only user).

### Step 6 - Unpaired layoutChanged (done)

Done: the model no longer emits `layoutChanged` on `hiddenChangedSignal`; `LayerPanelWidget`
repaints its viewport instead (the delegate paints the checkboxes from `hidden()`, no data role
depends on it). Nothing else relied on the `layoutChanged`. Test: no `layoutChanged` on hidden
changes (`test_layertreemodelstate.cpp`).

`LayerTreeModel` emits `layoutChanged` on every `hiddenChangedSignal` without
`layoutAboutToBeChanged`: Qt contract violation, full relayout per checkbox click.

- Remove the connection, let the panel repaint (`viewport()->update()`) on `hiddenChangedSignal`.
- Check for code relying on the `layoutChanged`.

### Step 7 - Scoped notifications in the OSG items (done)

Done:

- `LayerTreeModel::appendChildItem()` / `removeChildItem()` / `itemChanged()` (`addRootItem()` uses
  `appendChildItem()`), `LayerTreeItem::appendChildNotified()` / `removeChildNotified()` /
  `replaceChildren()` / `notifyChanged()` (plain operations while detached)
- radars, FFTs, remote units, sectors, viewpoint annotations, map layers: create functions return
  detached items, rebuilds via `replaceChildren()`
- measurements and internal annotations: add / remove / remove all / rename scoped; the internal
  annotation add was not notified at all before (Qt contract violation)
- `GeographicViewLayerModel::setMap()` and `OSGAnnotationsRootTreeItem::updateTree()` without reset;
  the sector layers built on the first map notify themselves (`OSGSectorRootTreeItem::rebuildSectorLayers()`)
- no model resets left in the geographic view (`modelChangedSignal` has no listener anymore)
- tests: notified operations without reset, restore on a notified append, detached fallback

Full model resets for local changes: measurement add/delete, annotation remove/rename, radar/FFT/
RU/sector rebuilds, viewpoint annotation update (on every load).

- Single inserts/removes via `beginInsertRows` / `beginRemoveRows`, renames via `dataChanged`,
  subtree rebuilds via `refreshSubtree()`.

### Step 8 - In-place DBContent reconciliation (accepted for now)

Not implemented for now: the scoped `refreshSubtree()` rebuild plus the state restore is accepted as is.

The DBContent subtree is destroyed and rebuilt on every live tick (and every style refresh).

- Match existing groups/leaves by key, swap the payload pointers, insert/remove only changed rows.
- Style refresh: update colors in place instead of rebuilding.
- Release old payloads only after the swap (the provider currently destroys them before the tree
  is rebuilt).
- See F1: the main architectural improvement left.
- Superseded by the update rework (U2), see below.

## Update rework (U0 - U6)

Goal: update the tree in place instead of destroying and rebuilding it (F1). Keep the item design,
move the OSG content of the geographic items into drawable classes (U4), and

- configure the tree once in the view (as today: root items added at model creation)
- update it via `updateAll()` / `updateSubtree()`
- updates keep existing children, drop vanished ones, add new ones
- the update mechanism lives in the base classes

Status: [x] U0  [x] U1  [x] U2  [x] U3  [x] U4  [x] U5  [x] U6 (verification in the application
open, see F2)

### Research findings

- `refreshSubtree()` removes all old children before `build()` runs; `appendChildItem()` only
  appends; `removeChildItem()` always drops states (ignores `complete`).
- Group rows aggregate count / custom columns live in `data()` (`collectChildValues()`), leaves
  cache their color at construction (`DBContentLeafItem` ctor) and read count / custom columns live
  through their payload; new payloads start visible, a visibility push only happens on restore.
- Panel: capture on `rowsAboutToBeRemoved`, restore on `rowsInserted`; moves are handled by
  QTreeView natively (persistent indexes).
- Pre-existing hazards:
  - scatter: the async recompute's worker lambda calls `clearIntermediateRedrawData()` =>
    `resetStashDependentData()` => `resetSeries()` => `rebuildLayerTree()` => Qt model mutated on a
    worker thread; with the data-complete flag set it also prunes all Data states
  - radars / FFTs roots only follow `activeContextChangedSignal`, not `dataSourcesChangedSignal` /
    `fftsChangedSignal` => their `const context::DataSource&` / `const context::FFT&` dangle after
    deletes / imports (FFTs live in a `std::vector`)
  - radar, FFT, RU data source and RU item destructors never remove their group from the parent
    group => every rebuild leaks empty OSG groups
- Domain containers: data sources `std::map<unsigned int, DataSource>` (look up by id), FFTs
  `std::vector<FFT>` (no id, names not reliably unique), sectors `vector<shared_ptr<Sector>>`
  (edited in place), `rebuildSectorLayers()` recreates the `SectorLayer` objects, a database open
  replaces the whole context.
- Data subtree producers: histogram (load commit, color mode, live tick), table (also on every
  recompute redraw incl. hidden toggles), grid (stash commit, old payloads released after the
  rebuild via `retired_payloads_`), scatter (payloads point into `scatter_series_`, "Selected" leaf
  directly under the root), geographic (provider recreates the payloads per finalize; rebuild on
  every tick, redraw and style refresh, `syncDBContentHiddenFromLayers()` after).

### Design (base)

`LayerTreeItem` hooks for dynamic items (static items keep the defaults):

- `childKeys()`: desired child keys in display order, `std::nullopt` => static children (not
  reconciled)
- `createChild(key)`: creates the child for a key not present yet
- `updateChild(child)`: refreshes a kept child from the domain, true if its display changed
- `onSubtreeUpdated()`: after the item's subtree was reconciled (e.g. group color recompute)
- `update(complete = true)`: `model()->updateSubtree(this, complete)` if attached
- `updateKey()`: child matching key, default `stateKeyId()` if set, else the name; separate from
  the state key, so a session-only id can match items while the persisted config keeps its key
  (FFTs, U4). Keys of dynamic children must be unique (duplicates logged and skipped).

`LayerTreeModel`:

- `insertChildItem(parent, row, item)` (generalizes `appendChildItem()`, restore via
  `subtreeAttached()`), `moveChildItem(parent, from, to)` (`beginMoveRows`),
  `removeChildItem(parent, row, drop_states = true)`
- `updateSubtree(item, complete = true)` / `updateAll(complete = true)`: per dynamic parent, walk
  the desired keys by position: keep (+ `updateChild()`), move a later match up, or `createChild()`
  + insert; remove the rest (contiguous ranges); recurse into kept and new children, static items
  are passed through; then `onSubtreeUpdated()` bottom-up; one `dataChanged` per parent over its
  kept rows (all columns => aggregates repaint); then `checkUniqueKeys()`,
  `flushHiddenNotification()`, `captureHiddenState()` once
- `complete = false` => removals keep their states, `complete = true` => removals drop them (until
  U5; afterwards removed items always take their state with them)
- `refreshSubtree()` / `replaceChildren()` stay until all producers are converted
- `ViewLayerTreeModel::updateDataSubtree()` replaces `refreshDataSubtree()` (passes
  `dataComplete()`)

Tests: new `src/view/viewbase/unit_tests/test_layertreeupdate.cpp`: keep / drop / add / move
without remove + insert of kept rows, recursion through static items, complete vs. incomplete
removal, restore of re-added items, `dataChanged` for kept rows and ancestors, duplicate keys, a
QTreeView keeps the expansion of kept rows.

### U0 - Pre-existing hazards (done)

Done:

- scatter:
  - `resetSeries()` no longer rebuilds the tree (it ran on the recompute worker); the tree is
    rebuilt on the main thread only: commit, `updateFromAnnotations()` (also on its failure paths),
    `resetVariableDisplay()` (part of `clearData()` => empty tree as before; since U2 the base
    empties on clear), `attachLayerPanel()`
  - `ScatterLeafPayload` is a snapshot (name, count, null count, color, visibility), no pointer into
    `scatter_series_`; `applyLayerVisibility()` copies the flags to the series at the end of
    `rebuildLayerTree()` and in `updateChartSlot()`
  - the check: `updateChartSlot()` (hidden toggle) did draw from `scatter_series_` while the worker
    refilled it (race), and the old payloads wrote into series the worker had cleared => it now
    returns while async work is pending, the commit applies the flags and draws
  - side effect: a recompute no longer empties the tree first (no prune of all Data states with
    complete data); a failed recompute (worker throws) leaves the last result in the tree and the
    chart; a superseded one is always followed by `clearData()` (only `shutdownAsyncProcessor()`
    invalidates, from `clearData()` and the destructor), which empties the tree
- radars / FFTs roots also rebuild on `dataSourcesChangedSignal` / `fftsChangedSignal`
- new `DBContextManager::deleteAllDataSources()` (saves, emits `dataSourcesChangedSignal`), used by
  the context edit dialog's "Delete All" and the `delete_all_data_sources` runtime command, which
  cleared the container without a signal (dangling radar / RU references); the runtime command now
  also saves the context and fails while an opened database contains imported data
  (`dbOpened() && hasInsertedData()`, as the context switch command), description updated
- radar, FFT, RU data source and RU destructors remove their group from the parent group (interim)
- grid (added after U2, same pattern as scatter):
  - payload lifetime: `processStash()` (may run on the worker) swapped `payloads_` and parked the
    old ones, still referenced by the tree, in `retired_payloads_`; a failed recompute skipped the
    commit, the next `processStash()` overwrote `retired_payloads_` => leaves pointing at freed
    payloads until that commit. Now `processStash()` only fills `pending_payloads_`, the main thread
    commit makes them current after rebinding the tree (old ones released afterwards);
    `retired_payloads_` removed, a failed / discarded run leaves the tree untouched
  - `layersChangedSlot()` (checkbox toggle) ran `buildGridFromStash()` on the main thread while the
    worker filled the stash and the grid => returns while async work is pending
  - the worker read `layer_model_->state().hiddenIds()` => new main thread hook
    `VariableViewStashDataWidget::prepareStashProcessing()` (before the async launch and before the
    synchronous processing), the grid snapshots the hidden ids there (`hidden_series_`); the commit
    compares the current hidden ids with the snapshot (toggled during the recompute, restored on
    new leaves) and rebuilds the grid if they differ
- not checked in the application yet

Original plan:

- scatter: `resetSeries()` must not touch the tree (runs on the worker); the tree is only changed
  on the main thread, in `commitStashDisplayData()` / `updateFromAnnotations()`
- scatter payloads decoupled from the series: `ScatterLeafPayload` becomes a snapshot built at the
  commit (name, count, null count, color, own visibility flag) instead of a `DataSeries*` view; the
  commit applies the snapshots' flags to the new series before drawing, `updateChartSlot()` does the
  same before a redraw => the worker may clear / refill `scatter_series_` freely, the tree never
  reads it; no extra memory; during a recompute the tree shows the last finished result
- check: does `updateChartSlot()` (hidden toggle) draw while the async recompute runs? If so, defer
  the redraw to the commit
- radars / FFTs roots: connect `dataSourcesChangedSignal` / `fftsChangedSignal`
- radar, FFT, RU data source and RU destructors remove their group from the parent group (interim,
  the classes are replaced in U4)

### U1 - Base mechanism (done)

Done (the design above, details as implemented):

- `LayerTreeItem`:
  - hooks `childKeys()`, `createChild(key)`, `updateChild(child)`, `onSubtreeUpdated()`,
    `updateKey()`; `update(complete = true)` => `LayerTreeModel::updateSubtree()` if attached, plain
    operations if detached (same algorithm)
  - per dynamic item: first the children whose key vanished are removed (contiguous ranges, from
    the back), then the keys are walked by position: keep (+ `updateChild()`), move a later match
    up, or create; consecutive new children are inserted with one notification
  - a new child is filled by its own `update()` while still detached => it arrives with its whole
    subtree in one insert (one restore, as `refreshSubtree()` before); only kept children are
    recursed into afterwards
  - kept rows get one `dataChanged` per parent if a kept child reported a change or a subtree below
    changed (aggregates); `onSubtreeUpdated()` bottom-up on every visited item
  - duplicate keys: later ones skipped (logged), duplicate existing children (added by other means)
    removed (logged); `createChild()` returning nullptr is logged and skipped
  - plain `insertChild(row)` (`appendChild()` uses it), `moveChild(from, to)`, `removeChildrenAt()`
- `LayerTreeModel`:
  - `insertChildItem()` / `insertChildItems()` (`appendChildItem()` uses them), `moveChildItem()`
    (`beginMoveRows`), `removeChildItems(first, last, drop_states)`, `removeChildItem(row,
    drop_states = true)`, `childItemsChanged(parent)`
  - `updateSubtree(item, complete)` / `updateAll(complete)`: the hidden signal and the
    effective-hidden ids follow once at the end (deferred while updating); `dataChanged` for the
    item and its ancestors if anything changed; key uniqueness checked per insert batch
- tests (`test_layertreeupdate.cpp`): keep / drop / add without touching kept rows, move instead of
  remove + insert, new child with its subtree in one insert, static items passed through, complete
  vs. incomplete removal (state of a returning child), `dataChanged` for a changed leaf and its
  ancestors / for a structural change below / none without changes, duplicate keys, bottom-up
  `onSubtreeUpdated()`, detached update, a panel keeps the expansion of kept and moved rows
- no producer uses it yet (U2 - U4)

### U2 - Data subtree (done)

Done:

- `DBContentRootItem::updateFrom(entries, top_leaves, complete)` (replaces `buildChildrenFrom()`):
  - the entries are referenced only during the call (update keys per entry and level computed
    once), outside it the Data subtree is static => `updateAll()` from elsewhere never touches it
  - root and groups answer `childKeys()` from the entries matching their key path (DSType =>
    `ds:<id>` or data source name => line => leaf persistence id, else the dbcontent name), the
    root lists the top leaves first (`TopLeafEntry`, the scatter "Selected" leaf)
  - `createChild()`: group or leaf; `updateChild()`: groups follow name changes (e.g. a renamed
    data source), leaves `setPayload()` (persistence id asserted equal, color re-read, visibility
    pushed); root `onSubtreeUpdated()` => `recomputeColorsRecursive()`
  - groups carry the root (`data_root_`) and their key path, so detached new groups can fill
    themselves before their insert
- `DBContentLeafItem`: `setPayload()` (nullptr detaches), empty count and custom columns without a
  payload
- `ViewLayerTreeModel` (replaces `refreshDataSubtree()`):
  - `updateDataSubtree(entries, top_leaves = {}, keep_states = false)`: vanished items drop their
    states with complete data; while reloading, updates from incomplete data are skipped (the stale
    rows stay until the final data); `keep_states` for displays not from the loaded data (scatter
    annotation series): ends a reload, keeps the states
  - `beginDataReload()`: leaves detached, Data subtree stale (`DBContentRootItem::setStale()`), no-op
    while reloading or empty; `clearDataSubtree()`: removes the rows (states kept), ends a reload;
    `dataReloading()`, `hasStaleItems()`, the subtree is repainted on every stale change
- views: histogram, table, grid, scatter, geographic set the entries and call `updateDataSubtree()`,
  old payloads released afterwards (grid: pending payloads, see U0); table / grid guards and
  `pushLayerStateToModel()` kept;
  geographic: `clearDBContentSubtree()` and the provider's `layersResetSignal` removed (no other
  listener), the payload's `setVisible()` only acts on changes (pushed on every rebind)
- reload phase (deviation from the plan, see below):
  - `ViewManager::issueLoad()` marks the reload in all views (`View::beginDataReload()` =>
    `ViewDataWidget::beginDataReload()`) before its `clearDataInViews()`
  - `ViewDataWidget::setItemProvider()` connects the provider's `dataAboutToBeResetSignal` (live
    ticks in place, map switch, grouping change) to `beginDataReload()` (own connection handle, the
    derived widget's connections stay)
  - `ViewDataWidget::clearData()`: outside a reload the data rows are removed (states kept) before
    the payloads are released; within a reload they stay stale
  - safety net: still reloading at `dataLoaded` (the view updated nothing) => rows removed, logged
  - why not from `clearData()` as planned: views are also cleared without a reload following (data
    delete, empty live feed, views not following live data, reconstructor, empty UTN selection,
    database close) => stale rows would stay forever; the reload start is known at the load issue
- stale display (optional): `LayerTreeItem::isStale()`, `LayerTreeModel::hasStaleItems()` /
  `staleChangedSignal()`; `LayerTreeItemDelegate` flags `StaleGreyed` (greyed text) and
  `StaleSpinner` (rotating open arc in the count column, which is empty while stale), default both;
  `LayerPanelWidget::setStaleIndication(flags)`; a 100 ms timer runs only while the model has stale
  items and the icon is enabled, it repaints the viewport (= visible rows only) and skips while the
  panel is hidden; stale rows have a "Data is being reloaded" tooltip
- scatter: the empty rebuild in `resetVariableDisplay()` (U0) removed, the base empties on clear
- tests (`test_layertreeupdate.cpp`): rebind of kept leaves (color, visibility of a hidden leaf),
  new leaf, top leaves first, reload (detached rows stay stale, empty count, idempotent start,
  partial data skipped, final data rebinds and pushes a toggle made while stale, vanished leaf
  removed), `keep_states` / clear end the reload and keep the states, no reload of an empty
  subtree, panel wait icon timer (on while stale, off when disabled or ended); ported:
  data-complete state drop, data source groups by id
- known edges: the `ViewDataWidget` flows (`issueLoad()` / clear / provider reset / safety net) have
  no unit test (needs a `ViewDataWidget` test double, see F3); a provider reset outside a load that
  never finalizes would keep the rows stale; lookups are linear per update (entries x groups),
  fine for hundreds of layers, index if needed
- not checked in the application yet

Original plan:

- `DBContentRootItem::setEntries()`; root and groups answer `childKeys()` from the entries filtered
  by their path (ds type => ds id or name => line => dbcontent); the root also lists extra top
  leaves (scatter "Selected")
- `createChild()`: group or leaf; `updateChild()` on a leaf: `setPayload()` (new) swaps the
  payload, keeps the cached persistence id (asserted equal), re-reads the color, pushes the
  visibility (`setVisible(!effectiveHidden())`); root `onSubtreeUpdated()` runs
  `recomputeColorsRecursive()`
- views: set the entries, `updateDataSubtree()`, release old payloads only afterwards (histogram,
  table: keep `payloads_` until after the update; grid: already via `retired_payloads_`; scatter:
  snapshots, U0; geographic: the provider reset detaches first, see stale mode); the table / grid
  `in_layer_recompute_` guards and the table's `pushLayerStateToModel()` stay
- order: geographic (live ticks), histogram, table, grid, scatter
- reload phase, stale mode:
  - `ViewLayerTreeModel::beginDataReload()`: every leaf drops its payload (`setPayload(nullptr)`),
    the Data subtree is marked stale (greyed rows, empty count, "loading" tooltip), one
    `dataChanged` over the subtree; idempotent. Separate from the data-complete flag (which only
    decides whether removals drop states): a live tick detaches and rebinds within one turn while
    its data is complete.
  - called only from base classes, right before payloads are released:
    `ViewDataWidget::clearData()` (all views, before `clearData_impl()` and the provider reset:
    loading start, live session start, `ViewManager::clearDataInViews()`), and the connection of
    `ViewDataWidget` to `DBContentItemProvider::dataAboutToBeResetSignal` of its item provider
    (every provider reset: map switch, grouping change, live ticks)
  - rows stay => scroll, focus, expansion and selection stay natively; checkbox toggles while stale
    are stored on the items and pushed on rebind; context menu data actions find no layers and do
    nothing
  - the final `updateDataSubtree(complete = true)` rebinds kept leaves, adds new ones, removes
    vanished ones and clears the stale mark; database close still clears the tree completely
  - `clearDBContentSubtree()` on `layersResetSignal` is dropped (the reset detaches instead)

### R4 - Shared layer panel code of the views (before U3, done)

Done (as planned below, details as implemented):

- `ViewLayerSnapshot` (`src/view/viewbase/viewlayersnapshot.h`) replaces the four payload classes
  (headers removed); custom column 0 = null count ("# NULL" of grid / scatter), `setTopLeaf()` for
  the scatter "Selected" series
- `view_layer_scan`: `LayerId`, `LayerAgg : LayerId`, `parseLayerId()`, `layerIdOrUnknown()` (the
  "<unknown>" fallback of grid / scatter), `lineIndexFrom()`; scatter's color resolving uses
  `parseLayerId()` too
- `ViewDataWidget`: `attachLayerPanel(model)` (own connection handles, a second attach replaces the
  first), `layerTreeModel()` default = the attached model (geographic keeps its override),
  `layerPanelModel()`, `hasLayerPanel()`, `updateLayerTree()`, `refreshLayerTree()`,
  `layerSnapshots()`, hooks `rebuildLayerTree()` (default: refresh), `layerVisibilityChanged()`,
  `colorModeChanged()` (default: rebuild), `layerPanelAttached()`; aggregation cache
  `layerAggregates()` / `setLayerAggregates()` / `invalidateLayerAggregates()`,
  `snapshotsFromAggregates()`
- `VariableViewDataWidget`: annotation subtree sync (constructor connection + `layerPanelAttached()`)
- views: histogram / table build snapshots from the aggregates, grid builds pending snapshots in
  `processStash()` (U0), scatter from its series; grid's `rebuildLayerTree()` is the base default
  now; `in_layer_recompute_` guards, `layersChangedSlot()`, the config widgets' hidden signal
  connections and `db_content_root_` members, `layerTreeRebuiltSignal` removed
- found on the way (same race as grid, U0): the histogram's generator update (worker) read
  `state().hiddenIds()` => snapshot `hidden_layer_ids_` taken in `postLoadTrigger()` (main thread,
  right before both the asynchronous and the synchronous generator update)
- behavior: hidden states restored by the tree update itself no longer trigger the view's
  visibility reaction (histogram redraw, scatter chart update) - a redraw follows those updates
  anyway (load done, commit); scatter builds its snapshots also without a panel
- tests: layer id splitting (well-formed, malformed, line index) in `test_layertreeupdate.cpp`; the
  view hooks have no unit test (need view test doubles)
- not checked in the application yet

Plan:

Histogram, table, grid and scatter repeat their layer panel code; since U0 / U2 their payloads are
all plain snapshots. Into the bases (the table view derives from `ViewDataWidget` directly, so the
generic part goes there, the annotation sync into `VariableViewDataWidget`):

- `ViewLayerSnapshot` (viewbase, one class instead of `HistogramLeafPayload`, `TableLeafPayload`,
  `GridLeafPayload`, `ScatterLeafPayload`): layer key, id parts, count, null count, color,
  visibility, top leaf flag
- `view_layer_scan::LayerId` + `parseLayerId()` (the hand-written key splits of grid and scatter),
  `LayerAgg` derives from `LayerId`
- `ViewDataWidget`:
  - `attachLayerPanel(ViewLayerTreeModel*)` (members `db_content_root_` / `layer_model_` /
    `layerTreeModel()` overrides of the views removed; the geographic view keeps its own model)
  - owns the snapshots: `updateLayerTree(snapshots, keep_states)` (entries, `updateDataSubtree()`,
    old snapshots released afterwards), `refreshLayerTree()`, `layerSnapshots()`
  - hooks `rebuildLayerTree()`, `layerVisibilityChanged()` (not for hidden signals of its own tree
    update => replaces the `in_layer_recompute_` guards of table and grid), `colorModeChanged()`
    (default: rebuild), `layerPanelAttached()`; the hidden signal connections of the grid and
    scatter config widgets move here
  - layer aggregation cache of histogram and table (`layerAggregates()`,
    `setLayerAggregates()`, `invalidateLayerAggregates()`) and `snapshotsFromAggregates()` (the
    identical color resolving loops)
- `VariableViewDataWidget`: annotation subtree sync (`annotationsChangedSignal` + on attach),
  members `annotations_root_` removed from the views
- `layerTreeRebuiltSignal` removed (no listener since R2)
- not included: the panel creation in the config widgets (small, optional)

### R5 - Further shared view code (before U3, done)

Done (histogram, table, grid, scatter; the geographic view keeps its own variants):

- `ViewDataWidget::invertSelectionSlot()` / `clearSelectionSlot()` (virtual public slots, the
  identical copies of histogram / grid / scatter removed; the geographic view overrides them via
  its geometry item provider); emitted via `View::emitSelectionChange()` (= the same signal)
- `ViewDataWidget::releaseWidgetLater(widget, layout)`: the deferred chart release (deleteLater,
  the QtCharts crash fix) of histogram / scatter `releaseChartView()` and grid `resetGridChart()`
- `ViewDataWidget::snapshotHiddenLayerIds()` / `hiddenLayerIds()` / `hiddenLayerIdsChanged()`:
  the main thread snapshot for worker processing (grid: `prepareStashProcessing()` and its commit,
  histogram: `postLoadTrigger()`, call sites unchanged)
- `VariableViewDataWidget::reportAnnotationError(msg)`: histogram, scatter and grid; the grid
  asserted on `hasViewPoint()`, now skips the report without a view point (behavior change)
- `ViewConfigWidget::createLayerPanel(columns, show_annotations)` (panel, attach, color mode
  expansion; placement and the color mode label stay in the views) and
  `ViewLayerPanelWidget::nullCountColumn()` (grid / scatter, header unified to "#NULL" - before
  "# Null" in the grid, "# NULL" in the scatter view), the duplicated `sumULongLong()` removed
- not included: zoom capture / restore (chart type specific), the asynchronous recompute launches
  (histogram two-stage vs. the stash base), scatter-only axis handling

### U3 - Annotations (other views, done)

Done:

- `AnnotationsRootItem::updateFrom(groups, current_group, current_annotation, view)` (was
  `update()`, the only caller is `VariableViewDataWidget::updateAnnotationsTree()`): builds a layout
  of the groups and annotations, then updates in place
  - update keys: groups `group:<name>`, annotations `anno:<title>` with `#n` for repeated titles
    below the same parent (the state keys stay name based); annotations of an unnamed group
    directly below the root
  - `AnnotationGroupItem` (key + root, like the data groups) answers its leaves from the root's
    layout; new leaves start with the radio state of the view's current annotation, kept leaves
    get their indices (`setIndices()`) and the radio state (`setActive()`, silent: no activation,
    no hidden signal, repainted via `dataChanged`)
- new `LayerTreeItem::remembersHiddenState()` (default true): annotation leaves return false =>
  the model never stores or restores their checked state (it is the view's current annotation).
  Before, the session map could bring back a stale radio state for a recurring annotation.
- the subtree follows `annotationsChangedSignal` (view point scan, clear); activating an annotation
  does not emit it, the leaves switch the radio state themselves
- tests (`test_layertreeupdate.cpp`): in place update (kept group and leaves, indices and radio state
  follow, new / vanished items), repeated titles kept apart, radio state not remembered
- not checked in the application yet

Original plan:

- `AnnotationsRootItem`: keys = group / annotation names, `updateChild()` applies the radio state
  (active annotation) to kept leaves. The geographic annotations move to U4.

### U4 - Drawables decoupled from the geographic tree items (done)

Done:

- `experimental_src/view/geographicview/drawable/composite/` (namespace `geographicView`):
  - `DrawableComposite`: group below a parent group, starts hidden, `setVisible()` via node mask,
    `setMapNode()` => `mapNodeChanged()`, detaches its group in the destructor
  - `DrawableCollection` (QObject): tree-facing interface by id (`ids(parent_id)`, `name()`,
    `itemData()`, `icon()`, `tooltip()`, `stateKeyId()`, `canHide()`, `initiallyHidden()`,
    `setVisible()`, `setRootVisible()`, `buildContextMenu()`, `complete()`, `showHideAllMenu()`),
    `changedSignal` / `requestFrameSignal`, a root group below the map node; `OrderedIdMap<T>`
    reconciles entries by id (keep + update, create, remove)
  - `DrawableStation` (marker, label, rings from a copied `StationInfo`) with `DrawableRadar` /
    `DrawableFFT` / `DrawableRemoteUnit` (style + `infoFrom()` only), `DrawableStationCollection`
    with the radar / FFT / remote unit collections (label, ring, symbol settings)
  - `DrawableSector` (copied `SectorInfo`, rebuilt only on a change, `setEditing()`) +
    `DrawableSectorCollection` (layer and sector menus)
  - `DrawableMapCollection` (osgEarth layers by name, no composites, map node via `setMap()`)
  - `DrawableMeasurement` + `DrawableMeasurementCollection`
  - `DrawableAnnotation` (feature drawables, render bin counter, detached drawables follow the
    visibility) + `DrawableAnnotationCollection` with `DrawableViewPointAnnotationCollection` and
    `DrawableInternalAnnotationCollection` (grid menus, unique names)
- `GeographicViewDrawables` (owned by the data widget, created before and destroyed after the
  layer model): owns all collections, `setMap()` / `resetMap()`, the collections' frame requests go
  queued to `drawSlot()`
- tree side (`drawabletreeitem.h`): `DrawableTreeItem` (id only, pushes its effective visibility on
  attach, hide and every update, follows name changes via `rename()`, Show All / Hide All if the
  collection asks for it) and `DrawableCollectionRootItem` (fills itself detached in the
  constructor, `update(complete())` on `changedSignal`, Show All / Hide All slots). The root classes
  keep their names (`OSGRadarsRootTreeItem`, ...) with their config settings and root menus;
  `OSGContextRootItem` and `OSGAnnotationsRootTreeItem` are static groupings
- ids (update keys) and state keys:

| Entries | Id | State key |
|---|---|---|
| radars | ds id | ds id |
| FFTs | `FFT::runtimeId()` | name (a rename moves the states) |
| remote unit data sources / units | ds id / `<ds id>/<index>` | ds id / index |
| sector layers / sectors | session id `L<n>` (follows `sectorLayerRenamedSignal`) / `S<sector id>` | name / sector id |
| map layers | layer name | layer name |
| measurements | counter | counter |
| view point annotations | name path (`\x1f` separated, `#n` for repeated names), legend entries by index | name |
| internal annotations | session id `A<n>` | name (renamable) |

- `context::FFT::runtimeId()`: random boost uuid created in `FFT()`, copied with the value, not
  serialized, not compared; `DBContextManager::fftByRuntimeId()` (not an `fft()` overload, both take
  a string)
- removed: `OSGRadarTreeItem`, `OSGFFTTreeItem`, `OSGRUDSTreeItem`, `OSGRUTreeItem`,
  `OSGSectorLayerTreeItem`, `OSGSectorTreeItem`, `OSGMapTreeItem`, `OSGMeasurementTreeItem`,
  `OSGAnnotationsTreeItem` (+ internal / view point), `OSGAnnotationColorLegendTreeItem`, the
  `index_` maps, `GeographicViewLayerModel::setMap()` / `resetMap()`,
  `OSGContextRootItem::setMapNode()`; non-tree users (sector edit tool, measure tool, annotation
  commands, grid legends, view point bounds) talk to the collections
- no producer uses `replaceChildren()` / `refreshSubtree()` any more (U6)
- deviations from the plan: radar, FFT and remote unit share one entity (`DrawableStation`); two
  annotation collections instead of one; the map collection holds no composites
- behavior changes:
  - rows survive domain changes (data source / FFT / sector edits, context switch with the same
    ids, map switch, view point change): expansion, selection and scroll stay
  - a renamed FFT keeps its row and states (before: a new row with default states)
  - a map switch keeps the states of recurring layer names and pushes them to the new layers; the
    map root visibility is reapplied to a new map node
  - a sector is only redrawn if its values changed; sectors with < 3 points are skipped (assert
    before)
  - the grid legends of the screenshot come from the collections (visible = pushed effective
    visibility), not from a tree walk
- tests: `experimental_src/unit_tests/test_drawabletreeitems.cpp` against a fake collection (nested
  ids => children, in place update with kept items, visibility pushes, state key / hideability /
  menu by id, Show All / Hide All, `OrderedIdMap`); `test_fft_matching.cpp`: FFT runtime id
- open: the collections' OSG code has no unit tests (needs a map node); nothing checked in the
  application yet

Original plan:

All geographic roots follow the pattern of the Data subtree (the provider owns the content, the
tree item only forwards): the OSG content moves out of the tree items into drawable classes, the
tree items become thin. Built and tested one collection at a time (radars, FFTs, remote units,
sectors, map, measurements, annotations), delivered as one stage.

| Role | Name |
|---|---|
| primitives (unchanged) | `Drawable`, `DrawableMarker`, `DrawableText`, ... in `drawable/` |
| composite base | `DrawableComposite` in `drawable/composite/` |
| one entity | `DrawableRadar`, `DrawableFFT`, `DrawableRemoteUnit`, `DrawableSector`, `DrawableMeasurement`, `DrawableAnnotation` |
| reconciling collection per type | `DrawableRadarCollection`, `DrawableFFTCollection`, `DrawableRemoteUnitCollection`, `DrawableSectorCollection`, `DrawableMeasurementCollection`, `DrawableAnnotationCollection`, `DrawableMapCollection` |
| owner of all collections | `GeographicViewDrawables` (owned by `GeographicViewDataWidget`, next to the geometry provider) |

- `DrawableComposite` (not derived from the primitive `Drawable`): owns a group node and its child
  drawables, `hide()`, `setMapNode()`, `update()` from its domain object, detaches its group in the
  destructor. Entities reuse the drawing code of the current items (radar / FFT marker, rings,
  label, `updatePositions()`; RU marker / label; sector polygon `rebuild()` / `updateGeometry()`
  incl. `editing_`; measurement lines / text; annotation drawables via
  `DrawableFactory::fromJSON()`, render bins, legend).
- Collections: listen to their domain signals (context, measure tool, annotation sources),
  reconcile their composites by id (create / update / remove), emit a changed signal. Tree-facing
  interface (common base, testable with a fake): `ids(parent_id)` in display order (nested for
  sector layers / sectors, RU data sources / RUs, annotations / child annotations), `name(id)`,
  `icon(id)`, `tooltip(id)`, `setVisible(id, bool)`, settings (labels, rings, symbols, opacity).
- Ids:
  - radars, RU data sources: ds id; RUs: index within the data source
  - FFTs: runtime id, `context::FFT` gets a randomized boost uuid created in `FFT()`, not
    serialized, copied with the value (vector reallocation keeps it), `runtimeId()`,
    `DBContextManager::fft(runtime_id)` lookup
  - sector layers: name (rename via `sectorLayerRenamedSignal`); sectors: sector id
  - measurements: the existing counter
  - internal annotations: session id from a counter (replaces the renamable name as identity)
  - viewpoint annotations: name; on a viewpoint change the collection rebuilds the drawables of
    recurring annotations from the new JSON (the item is kept)
  - map layers: layer name; `DrawableMapCollection` wraps the current osgEarth map (layers by name,
    visibility, opacity, map node mask) and rebinds on a map switch
- collections look domain objects up by id (no `const&` members); `setMapNode()` stays a pure map
  node switch, refreshes go through the collection's update
- tree side: generic thin items `DrawableTreeItem` + `DrawableCollectionRootItem`: key + collection
  reference; `itemData()` / `icon()` / `tooltip()` from the collection;
  `onEffectiveHiddenChanged()` => `collection.setVisible(key, !effectiveHidden())`; roots answer
  `childKeys()` from `ids()`, `createChild()` creates the thin item, `updateChild()` notifies name /
  icon changes; the root's `update()` runs on the collection's changed signal. FFT items:
  `updateKey()` = runtime id, `stateKey()` stays name-based (the context subtree is persisted, a
  session id would orphan saved settings)
- the tree stays the source of truth for the checked state and the config; settings stay in the
  roots' `config()` and are pushed to the collection; new composites start hidden until the tree
  pushes their visibility
- menus call the collection by id (zoom, export, opacity, rename, delete, copy text, sector edit /
  color / delete, map file); the sector edit tool and other non-tree users (measure tool
  `addMeasurement()`, runtime commands adding annotations) talk to the collections, not to the tree
- sectors pass `complete = sectorsLoaded()`; removed: the `OSG*TreeItem` content classes and the
  `index_` maps of the roots
- tests: the thin items and roots against a fake collection (no OSG): ids => children, nested ids,
  visibility forwarding, changed signal => update, menu actions by id

### U5 - State in the items, session map removed (done)

Done:

- `LayerTreeState` keeps only the config root (`renameItemConfigs()`, was `renameItemStates()`) and
  the effective-hidden ids (`clear()` on database close drops those); removed: `ItemState`, the
  persist flags, pending selection, pruning
- `LayerTreeModel`: no storing on hidden changes, no pruning (`pruneItemStates()`,
  `clearItemStates()`, `doPruneItem()`), `subtreeAttached()` applies only the configured hidden
  state (persisted subtrees), else the constructed state stays; removals forget their subtree
- `LayerTreeItem`: removed `defaultHidden()`, `remembersHiddenState()` (U3, the annotation leaf
  override too); new `ViewHandover` (expanded, selected, current) for the panel handover
- the `complete` / `drop_states` / `keep_states` plumbing is gone (user decision: incomplete
  removals forget too, the small memory is noted as F7): `update()`, `updateSubtree()`,
  `updateAll()`, `removeChildItem(s)()`, `DBContentRootItem::updateFrom()`,
  `DrawableCollection::complete()`; `updateDataSubtree()` / `ViewDataWidget::updateLayerTree()`
  take `content_final` instead of `keep_states` (still ends a reload, e.g. scatter annotation
  display)
- pulled forward from U6: `refreshSubtree()`, `replaceChildren()`, `moveChildrenOut()` and
  `subtreeRefreshedSignal` removed (no producer left, they took `complete`)
- `LayerPanelWidget`:
  - new rows (insert, reset) get the default expansion policy, kept rows keep their expansion and
    selection natively, nothing is captured on removal
  - handover: the destructor writes `ViewHandover` onto all items, the next panel applies it in
    `init()` (or on `viewStateHandedOverSignal` if it was created first, then clears it at once);
    rows carrying a handover keep their expansion when a derived panel sets its policy in its
    constructor; cleared on the next event loop turn
  - `setDefaultExpansion()`: rows whose expansion equals the old default follow the new one, user
    deviations stay (`defaultExpansionChanged(old_default)`, compared live instead of via the map)
- `ViewLayerPanelWidget::setDataExpansion()`: the Data rows get the new default
  (`applyDefaultExpansion()`), as before (their remembered expansion was cleared)
- behavior changes: the checked state, expansion and selection of rows that vanish and come back
  are forgotten in all cases, also for incomplete removals: `clearDataInViews()` without a reload
  (reconstruction, live start / stop, viewpoint commands), scatter annotation display round trips,
  sectors before they are loaded (covered by the config there); persisted items get their checked
  state from the config as before
- tests: `test_layertreestate.cpp` (config rename, hidden ids), `test_layertreemodelstate.cpp`
  (forgotten on removal / reset, keys, rename moves configs, menu actions, effective-hidden ids),
  `test_layerpanelviewstate.cpp` (default for new rows, kept selection, policy change keeps user
  deviations, handover in both orders and against a derived panel's policy, cleared after
  construction), `test_layertreeupdate.cpp` / `test_layertreeitemconfig.cpp` adapted
- not checked in the application yet

Original plan:

- kept rows keep expansion, selection and current row natively in the QTreeView => no continuous
  sync needed
- panel handover (config widget rebuilt => new QTreeView on the same model): the old panel writes
  expanded / selected / current flags onto the items in its destructor, the new panel applies them
  in `init()` and clears them
- new rows: checked state from the config (persisted items), else the constructed default;
  expansion from the default policy => "vanished => forgotten" by construction
- removed: `LayerTreeState` item states, the panel capture / restore on row removal, pruning, the
  session part of `renameItemStates()`; kept: config persistence and its renames, effective-hidden
  ids
- incomplete removals (cancelled / failed load) then forget their states (F3 edge, unchanged)

### U6 - Cleanup and documentation (done)

Done:

- code comments referring to the session map, remembered states and rebuilds updated
  (`dbcontentlayer.h`, `layertreemodel.h/.cpp`, `viewdatawidget.cpp`, `gridviewdatawidget.cpp`,
  `geographicviewlayermodel.cpp`, `drawableannotationcollection.cpp`)
- readme: concepts rewritten for the current state, the first-round sections ("Done", "Base",
  "Geographic view", "Step plan") marked as history with the parts removed by U5, behavior notes
  current, F1 done, F2 checklist for the current behavior

Original plan:

- ~~remove `replaceChildren()` / `refreshSubtree()` where no producer uses them anymore~~ (done in
  U5); the config and the effective-hidden ids remain; a database open clears the tree (all objects
  new)
- readme: done notes per stage, update F1 (destroy and rebuild dropped), F2 checklist, the "Done"
  overview at the top and the step sections describing the session map

### T1 - Testable geographic collections (after U6, done)

- `geographicView::DrawableDomain` (`drawable/composite/drawabledomain.h`): what the collections
  read from the application - active `DBContext`, sector layers and their loaded flag, sector
  height settings, view point data and error reporting, accepted annotation feature types, the five
  context change signals; `view()` for the context menus (nullptr => no menus)
- `GeographicViewDrawableDomain`: the implementation over `GeographicView` and `DBContextManager`
  (forwards the manager's signals), owned by `GeographicViewDrawables`, destroyed last
- radar / FFT / remote unit / sector / annotation collections and `DrawableSector` take the domain
  instead of `DBContextManager&` / `GeographicView&`; `DrawableCollection::rootNode()` (read-only)
- without a map node `DrawableAnnotation::build()` skips the features (was an assert, the view
  always has one), the internal annotations' `add()` assert on the map node is gone
- tests (`experimental_src/unit_tests/test_drawablecollections.cpp`, fake domain, no map node):
  radars by ds id (kept composite and visibility on a change, no context), FFTs by runtime id
  (rename, reallocation), remote units nested (names, tooltips, state keys, kept groups), sector
  layers by session id (not loaded, rename keeps the id, vanished => new id), view point
  annotations (repeated names, nested reverse order, unaccepted skipped, error reported, hidden
  flag, recurring entry kept), internal annotations (unique names, rename, remove)
- still untested: the drawing with a map node (positions, sector geometry, annotation features,
  legends), the map collection, the menus, `GeographicViewLayerModel`

### T2 - Drawing tests with an in-memory map (after T1, done)

- spike result: an osgEarth 3.8 `Map` with a profile (no files, no network, optionally
  `DebugImageLayer`s) and its `MapNode` open headless in the test binary, no GL needed
- bug found and fixed: `DrawableMeasurement` created its `DrawableLineStringInterpolated` without a
  map node, which dereferences it in its constructor => every measurement add would have crashed
  (U4 regression, the old item passed the current map node). Now the line is created per map node in
  `mapNodeChanged()` (also fixes the stale SRS of the line after a map switch, pre-existing)
- tests (`experimental_src/unit_tests/test_drawablemap.cpp`, fakes shared via `drawabletestfakes.h`):
  sector polygon on geodetic (lon / lat) and projected maps (map coordinates), no redraw without a
  change, edit tool mask, 3D walls with heights, degenerate sectors skipped; station composites
  across map switches, root group re-parented; measurement drawing, column texts, clipboard text,
  map switch and add before a map node; view point and internal annotations with text features
  (drawables rebuilt in the kept group); map collection (layer ids, title, initial visibility,
  layer and root visibility, rebinding on a map switch, root visibility reapplied)
- geo image annotations (in-memory image via `ViewPointGenFeatureGeoImage`): legend entries (names,
  icons, not hideable, only for a single geo image), the image's map layer follows the pushed
  visibility (not below the group), `visibleGeoImages()` for the screenshot legends, the opacity
  menu, the map layer removed with the annotation and not piling up on view point updates
- still untested: drawn positions of markers / labels / rings, the menus besides opacity,
  `GeographicViewLayerModel` (decided: application check only)

Verification per stage: `cmake --build build`, `QT_QPA_PLATFORM=offscreen ./build/bin/compass_tests`,
plus in the application: geographic live mode (rows, expansion, scroll and selection stay across
ticks, no full row removal per tick), reload (rows stay greyed, no crash on scroll / hover / context
menu), style / color mode change, data source / FFT / sector add, edit, delete, map switch,
viewpoint change, each other view with a reload and a hidden toggle, no Qt model access from worker
threads (scatter).

## Target report context menu in live mode (M1 - M4, done, application check open)

The context menu of a single target report in the geographic view (right click on a drawn point,
`ClickHandler::handleReleaseRMB()`) is switched off while live running ("Action not available in
Live Mode: Running"). The gate dates from menus that held raw layer / tree item pointers. Goal: drop
the gate and make the menu itself robust against the data vanishing while it is open, as done for
the layer tree menus (step 1, `connectItemAction()`).

Requirements (user):

- the opened menu is robust (no crash and no wrong target whatever happens while it is open)
- an action does nothing if its data is already gone when it is triggered
- the parent layers go into a submenu "Parents", on the same level as the "Target Report" submenu

### Current state (research)

- the menu is built by `GeometryItemProvider::addTargetReportActions()`. The same function also
  builds the menu of a label overlay (`GeometryItemGroupPointLabelWidget::buildContextMenu()` ->
  `GeometryItemGroupLayer::addLabelActions()`), and that menu has **no** live gate => the actions
  already run in live mode today, only the pick menu is blocked
- target report actions capture a `TargetReportRef` (layer key + record number) and resolve it on
  trigger (`resolveTargetReport()`); a missing report shows a tooltip "Target report not available
  (any more), please retry" (`notifyTargetReportUnavailable()`)
- the parent scope submenus (`addNodeActions()` with a `LayerFilter`) capture the filter; Show / Hide
  go through `connectItemAction()` (model + key, resolved on trigger)
- the grouping item submenu (e.g. "UTN 123", `addItemActions()`) captures the item id json only,
  not the grouping it belongs to
- live tick: reset -> rebuild -> finalize runs in one event loop turn (`reset_impl()`, stale layers
  drained in the same turn); asynchronous builds only offline / paused => a triggered action never
  sees a half-built state, only "before" or "after" a tick
- live data window: old records leave the buffers => `resolve(rec_num)` (buffer row needed) fails
  for them
- the pick runs inside the frame on the currently drawn layers; the left click label toggle uses
  the same pick in live mode already

Main risk: the record leaves the data while the menu is open (live data window, reload). Covered
by the lookup on trigger:

- the record is looked up by its number, not by a point or buffer index
  (`GeometryItemGroupLayer::findPoint()` via the record number index), and for buffer reads also its
  buffer row is checked (`resolve()` -> `hasValidBufferRow()`: the row must still exist and carry the
  same record number, the buffer may run ahead of the point data in live mode)
- record numbers are never reused within a database session: assigned at insert from a running
  counter plus the dbcontent id (`DBInterface::initDBContentBuffer()`,
  `maxRecordNumberWODBContentID()`), reset only when the data engine is cleared (database close)
  => a vanished record can not resolve to a newer record that took its number
- a live tick changes buffers and layers within one event loop turn => a trigger sees the record
  either still present or gone, never a half update

What can realistically change while the menu is open (it grabs the input, the user can not reach
the rest of the UI):

- data: live ticks, an arriving asynchronous build, a finishing load => **safeguarded**, by the
  lookup on trigger (above)
- not realistic, no safeguard: the provider / view being destroyed (the provider lives as long as
  the data widget, created once in its constructor), a grouping change (config widget) => same
  grouping + same item id = same item (user), the "Edit Content" dbcontent vanishing (dbcontents are
  static)
- tree items: Show / Hide already resolve by key (`connectItemAction()`)

Weak spots found and decisions:

1. missing data showed tooltips ("Target report not available (any more), please retry", "Action
   not available in Live Mode: Running") => decided: tooltips removed, a warning is logged instead,
   also for item actions
2. "Select Data" (mode Update) clears the current selection before it looks for the data
   (`resetSelectionVector()` first) => with the data gone it still clears the selection (decided:
   ok as is, unchanged)
3. selection in live running: the select tool, "Invert Selection" and "Show Only Selected" are
   disabled in live running (`geographicviewwidget.cpp`), the menu selection actions (here and in
   the layer tree node menus) are not (decided: accepted for now, actions stay enabled)

### M1 - Menu lifetime (done, no change)

- the menu stays unparented and released with `deleteLater()` (see the note above
  `handleReleaseRMB()`: no deletion of an open menu by a parent)
- the menu is a snapshot of the build time: captions and enabled states are not refreshed while it
  is open (it shows what was clicked); whether an action applies is decided on trigger (M2)
- no context objects for the action connections (owner destruction while open not realistic)

### M2 - Actions resolve on trigger, warning if gone (done)

- target report actions (Toggle Label, Show / Clear ARTAS usage, Show / Clear MLAT RUs) keep their
  methods, which resolve the `TargetReportRef`; a missing report => `logwrn`, nothing else
  (`notifyTargetReportUnavailable()` and its tooltip removed, also for the left click label toggle)
- item actions (zoom, labels, selection, associations, ground lines, speed vectors, ellipses, MLAT
  RUs): `iterateLayers(item_id)` logs a warning for an item without locations, the actions then
  find nothing to act on
- parent scope actions: no layer matches the filter => warning (local `connectFilterAction` in
  `addNodeActions()`), the action then finds nothing to act on; Show / Hide: `connectItemAction()`
  logs a warning for a missing tree item (all views' layer tree menus)
- Clear ARTAS usage / Clear MLAT RUs only need the layer (cleared even if the record is gone), a
  missing layer => warning; "Reset Position" (label overlay menu) on a vanished label => warning
  (`OverlayWidget::resetManualLabelPosition()`)
- no data involved: "Copy '...'" (text taken at the build), "Edit Content" (dbcontents are static)
- exception: "Select Data" (Update) with the data gone still clears the selection, as today (weak
  spot 2, ok as is)
- a report already gone when the menu is built (label menu of a record that just left) keeps the
  disabled "Target report not available" entry

### M3 - Menu structure (done)

```
Toggle Label
Edit Content                   (labels of dbcontents with target reports)
Reset Position                 (label overlay menu only)
---
Target Report         >        Show / Clear ARTAS Target Report Usage, Show / Clear MLAT RUs
UTN 123               >        grouping item actions (unchanged)
Parents               >
    DBContent CAT062  >        addNodeActions() per scope, order unchanged
    Line L1           >
    DataSource ARTAS  >
    DSType Tracker    >
```

- applies to both menus (pick and label overlay, shared `addTargetReportActions()`)
- decided: the grouping item submenu stays at the top level, between "Target Report" and
  "Parents" (it is the target, not a layer parent)

### M4 - Enable in live mode (done)

- app mode gate and its tooltip removed in `ClickHandler::handleReleaseRMB()` => the menu shows in
  live running as offline
- selection actions stay enabled while live running (weak spot 3); for consistency the toolbar's
  select tool, "Toggle Show Only Selected" and "Invert Selection" are enabled in live running too
  (their live disable callbacks removed in `geographicviewwidget.cpp`)
- extra geometry actions (ARTAS usage, ground lines, ground speed vectors, position accuracy
  ellipses, MLAT RUs; show and clear) are disabled while live running, decided when the menu is
  built: in the target report actions (the "Target Report" submenu stays, its actions greyed), the
  item actions and the layer actions (`addTargetReportActions()`, `addItemActions()`,
  `addNodeActions()`, so also the layer tree and item list menus)

### Tests and verification

- the provider needs `DBContentManager` and `GeographicView` => no unit tests for the menu building
- build and unit tests pass (no new tests)
- open, in the application (F2 / F8): live running, open the pick menu and the label menu and wait
  for ticks, then trigger each action on a report that is still there and on one that left the data
  window (warning in the log, no tooltip, no action); menu structure in both menus; offline behavior
  unchanged

## Future improvements

Weak spots of the current state, roughly by importance.

### F1 - Drop destroy and rebuild (architecture, done)

Was: the producers destroyed and rebuilt their subtrees, the state lived outside the items and was
carried over by key (snapshot before the rows vanished, restore on insert): two sources of truth,
correctness depending on capture timing, identity via string keys, per-row view state (scroll,
hover, focus) lost, the whole Data subtree rebuilt per live tick.

Done by the update rework (U0 - U6): all producers update in place (Data subtree with payload
rebind and stale reloads, annotations of the other views, all geographic roots via the drawable
collections), the session map is gone (U5). What the key map used to cover besides that:

- reload => stale rows (U2)
- panel handover => item flags (U5)
- data context switch => kept rows where the ids recur (U4)
- persisted config => unchanged
- incomplete removals => forgotten for now (F7)

Left open: scroll position and focus across live ticks are expected to stay (rows are kept) but are
not verified (F2).

### F2 - Verification in the application

Nothing is verified in the running application yet (automation: F8). The thin geographic tree items have unit tests
against a fake collection, the collections against a fake domain (T1), the drawing and the map
collection with an in-memory map (T2); the menus and `GeographicViewLayerModel` have none.
Checklist:

- geographic live mode: rows, expansion, selection and scroll position stay across ticks, no full
  row removal per tick
- reload (all views): Data rows stay greyed with the wait icon, a toggle while stale is applied on
  rebind, no crash on scroll / hover / context menus while stale; stale indication switchable
- geographic node actions while stale: the stale leaves have no payload => `findNodeItemGroup()`
  finds no item group, the Data node menus show no Show / Hide / Select entries and the map-click
  menus find no tree node until the reload ends. Expected, not decided; keeping them would need the
  ids on stale leaves. The filter / level mapping (`targetLevel()`, `itemGroupMatches()`) has no
  unit test (decided: checked in the application only)
- database close / open, application restart: persisted (Context) states come back, Data states
  start fresh
- preset apply, color mode change (Data expansion follows the mode, other rows keep theirs)
- config widget rebuild: expansion, selection and current row handed over
- data source / FFT / remote unit add, edit, delete, FFT rename: other rows keep their state, a
  renamed FFT keeps its row and state
- sector add / rename / edit / color / delete, sector layer rename (row kept), sector edit tool
  hides the line
- map switch (recurring layer names keep their state), map / layer opacity
- measurement add / copy / delete; internal annotation add / rename / export / zoom / opacity /
  remove; viewpoint change with recurring and new annotations, show / hide by name, grid legends in
  screenshots
- scatter annotation display round trip (Data states forgotten, F7), each of the other four views
  with a reload and a hidden toggle
- hidden data sources after a reconstruction or live start / stop: shown again (F7, expected)

### F3 - Data-complete flag

- since U5 the flag only decides when a reload ends (partial updates are skipped while stale), the
  states of vanished items are forgotten either way
- cancelled and failed loads count as complete (the engine sends the final update in every case,
  the operation state is set only after it) => the reload ends with the truncated data, the items
  not loaded are removed. Fix: the final update tells whether it is truncated, `ViewDataWidget` sets
  the flag only for a normal end.
- the flag handling in `ViewDataWidget` (`updateFromSource()` / `clearData()`) has no unit test
  (needs a `ViewDataWidget` test double, check the cost).
- assumes no Data rebuild from incomplete data after the final update (holds today, not enforced).

### F4 - Config entries are never cleaned up

Persisted item configs of deleted items (sectors, radars of removed data sources) stay in
`layer_config` and travel into presets. Pre-existing (before the rework every radar / layer /
sector got an entry at construction, keyed by name, never erased); now only changed items get
entries. Pruning on absence is wrong for configs (radars and sectors vanish on database / context
switches without being deleted). Options:

- delete-driven: `LayerTreeState::removeItemStates(key)` (map + config, subtree), called on real
  deletions - `removeChildNotified()` plus owner signals (sector deleted, all sectors deleted, data
  source deleted) handled by the roots before their rebuild
- optional: persist only deviations from the default in the config (fewer entries, untouched items
  follow changed defaults)

### F5 - Key clashes and reuse

- sector ids are probably unique per data context only => sectors of different contexts share a
  config key (pre-existing with names). Check the id assignment, add the context to the key path if
  needed.
- sector id reuse after a delete would let a new sector inherit the persisted config (see F4)
- same-named data source groups (geographic view) share their `persistenceId()` (`group:<path>`)
  => their effective-hidden ids overwrite each other. Latent, no consumer reads group ids.
- duplicate keys are only logged

### F6 - Developer and user facing

- many concepts for developers (keys and `stateKey()` vs. `updateKey()`, default policies, the
  handover, `rename()`), documented here only
- user visible: checked states of data items survive a reload (stale rows) but not a clear without
  reload or a restart, context items survive both; map layers keep their states across a map switch
  if the names recur. Document in the user manual if needed.

### F7 - Memory for incomplete removals

Rows that vanish while their domain object still exists come back with their default checked state
since U5 (user decision: forget for now). Affected: the Data rows after `clearDataInViews()`
without a reload (before a reconstruction, live start / stop, viewpoint commands, DBContentManager)
and the scatter annotation display round trip; the Context subtree is covered by the config.

Possible improvement: the model keeps only the checked state of rows removed by an incomplete update
(key -> hidden), restores it when a row with that key attaches, drops it on the next complete
update of that subtree and on database close. No recording per toggle, no panel capture; expansion
and selection of returning rows still follow the default. Needs the `complete` flag back on the
update path (removed in U5).

### F8 - Automated application checks via the runtime command interface

Run the F2 checklist automatically with the Python test framework (`experimental_src/py`, see
`readme_test_writing.md`) instead of by hand. Already available: `uiset` / `uiget` / `uiget_json` /
`uiinject` / `uifiledialog`, `layer_tree` item paths, item menus via `contextmenu(A|B)` and
`uiset popup`, the domain commands (load data, database, sector / FFT / data source imports,
`set_view_point`, `set_map`, live network import with pause / resume / stop, `client_info`), UDP
replay for live mode, test data in `~/test_data`, offline maps (`minimal.earth`,
`minimal_2d.earth`).

Missing:

- tree state: `uiget_json layer_tree --what entries` only returns name, column 1 and children.
  Make `LayerPanelWidget` a `ui_test::UITestable`: per row checked (own / effective), expanded,
  stale, selected / current, the scroll value, the wait icon timer, model insert / remove counters
  since a reset point (rows kept vs. recreated); `uiSet()` toggling a row's checkbox by path (a
  click on the row is consumed by the delegate's visibility toggle). About half a day.
- `resolveTreeItem()` expands the parents of an item path => the new getter must not use it, the
  expansion checks must account for it (or a non-expanding / restoring variant)
- visual checks (greyed rows, wait icon, drawn sectors / annotations): a screenshot of a named
  widget to a file (check the report export's screenshot path first), images readable by Claude
- environment: the display `:0` (windows appear on the desktop) with a copied config directory
  (`~/.compass/<version>` must not be touched) and a free port (27960 may be in use, `--fresh`
  would close a running instance); cleaner: Xvfb + Mesa (needs installing)
- not covered either way: subjective effects (flicker, perceived scroll jumps in live mode)

Plan: the `UITestable` getter / setter first, then test scripts in the framework's style for the
deterministic F2 points (reload stale and rebound, rows kept across live ticks, checked states
across a database reopen, panel handover, sector / FFT edits keeping other rows, F7 after a
reconstruction), kept as permanent integration tests.

## Architecture notes

- **One panel per model** - the panel handover goes through flags on the items; two tree views on
  one model would overwrite each other's handover (expansion and selection themselves live in each
  tree view since U5).
- **Default expansion policy signature** - the policy gets the panel depth, but
  `ViewLayerPanelWidget` computes the depth relative to the Data root itself and ignores the
  parameter there. Passing only the item (policies compute depth where needed) would be cleaner.
- **Config keys follow the tree layout** - path based keys orphan persisted settings when the tree
  is restructured (e.g. radars moved out of Context). Id based keys (step 2) reduce, but do not
  remove this.
- **Checked state on attach** - the config (persisted items), else the constructed default (since
  U5, before: session map first). Kept items hold their own; `configChanged()` (reconfigure, preset)
  applies the config to all items. Covered by `test_layertreeitemconfig.cpp`.

## Decisions

- Presets keep carrying `layer_config` (item configs incl. persisted checked states); presets can
  be loaded without the layer config if not wanted.
- Only checked states persisted to the config survive a database close/open; expansion and
  selection are cleared then.
- Legacy `layer_config` sections are dropped (one-time loss accepted).
- Step 8 deferred: the DBContent subtree keeps being rebuilt per live tick / style refresh
  (superseded by U2, updated in place).
- Viewpoint annotations on a viewpoint change: vanished annotations lose their state, recurring ones
  (same key) keep it, also over the `hidden` flag of the new JSON (step 4 via pruning; since U4 the
  rows are kept).
- Pruning works the same in all views (step 4; removed in U5).
- Update rework (U0 - U6): update in place instead of destroy and rebuild, the update mechanism in
  the base; reload phase in stale mode (rows kept, payloads detached), `beginDataReload()` called
  from base classes only; scatter payloads decoupled from the series (snapshots, no double data);
  state moves into the items (U5); geographic OSG content decoupled into `Drawable*` classes incl.
  measurements, annotations and the map (U4); FFTs identified by a session-only boost uuid, not
  stored in the context JSON.
- U5: incomplete removals forget their states too (no small memory for now, noted as F7).
