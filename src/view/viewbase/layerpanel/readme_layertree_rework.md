# Layer tree rework

State handling of the layer tree (checked state, expansion, selection, item config).
Status of the completed work, the step plan for the open work, future improvements (weak spots),
architecture notes and decisions.

## Concepts (as implemented)

- **Item key** - `LayerTreeItem::stateKey()` (not virtual): the parent's key plus the item's escaped
  segment, which is `stateKeyId()` if set (default `persistenceId()`), else the name (`#n` for
  repeated sibling names) => every key starts with its parent's key. `stateKeyId()` overrides: map
  root, RU data sources, RUs, radars, sectors, DBContent data source groups (see step 2). DBContent
  leaves cache their payload's id. Duplicate sibling keys are logged.
- **Remembered state** - `LayerTreeState` (owned by the model, `LayerTreeModel::state()`): session
  map, persist flags, item config root, effective-hidden ids. Knows nothing about the tree, the
  model applies it (attach, config changes, pruning).
- **Session map** - `LayerTreeState::ItemState` per key: `hidden`, `expanded`, `selected`, `current`.
  Only deviations from the defaults (`defaultHidden()` = state at first attach, the panel's default
  expansion policy). Survives rebuilds, live ticks, resets and config widget rebuilds. Cleared on
  database close. Components switchable via `PersistHidden` / `PersistExpansion` / `PersistSelection`.
- **Restore** - checked state on attach (`subtreeAttached()`): map, else config (persisted items),
  else default; effective-hidden cascade below unchecked ancestors; one deferred hidden signal per
  structural operation. Expansion and selection by the panel (snapshot before rows vanish, restore
  on insert/reset, handover to a successor panel).
- **Pruning** - `pruneItemStates(scope)` drops the entries below a subtree that `doPruneItem()`
  rejects (default: item not in the subtree). Runs on complete rebuilds (`replaceChildren()`,
  `refreshDataSubtree()` with complete data) and on removals (`removeChildNotified()`), see step 4.
- **Item config** - `bindConfig()` binds a flat `{ key: {...} }` object (geographic view:
  `layer_config`), `config()` / `config() const` per item. `persistsSubtreeToConfig()` persists the
  checked state of a whole subtree (`persistHiddenStateToConfig()`), written on every change.
  Hooks `onAttached()` / `onConfigChanged()`, `configChanged()` after a reconfigure / preset.

## Done

- [x] Key, session map, panel snapshot/restore, default expansion policy
- [x] Store only deviations from the defaults
- [x] Persist flags (`PersistHidden`, `PersistExpansion`, `PersistSelection`)
- [x] Pruning of missing items (`pruneItemStates()` / `doPruneItem()`)
- [x] Distributed item config in the base
- [x] Geographic view: OSG context items moved to the base mechanisms, legacy `layer_config`
      sections removed (one-time loss of the old visibility and ring/label settings)
- [x] Step 5: color mode expansion of the Data subtree in all views as the panel's default policy
- [x] R1: `LayerTreeState` extracted from `LayerTreeModel`
- [x] R2: color mode wiring out of `ViewConfigWidget`, color mode names central in `ColorProvider`
- [x] R3: color mode to depth mapping moved to `ViewLayerPanelWidget::dataExpansionDepth()`
- [x] Step 6: unpaired `layoutChanged` replaced by a viewport repaint
- [x] Step 7: scoped notifications instead of model resets in the geographic view
- [x] Step 4: subtree-scoped pruning, the same in all views
- [x] Step 2: stable, escaped keys (`stateKeyId()`), data source groups by id (geographic view)
- [x] Step 3: renamed items keep their remembered states (`rename()`, sector layer rename signal)
- [x] Step 1: context menu actions safe across rebuilds (`connectItemAction()`, context objects)

Tests: `src/view/viewbase/unit_tests/test_layerpanelviewstate.cpp`, `test_layertreemodelstate.cpp`,
`test_layertreeitemconfig.cpp`.

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

### Behavior notes

- Persisted items write their current checked state to the config on every change (not only
  deviations); only the session map is deviation-only.
- The session map wins over states an item brings along when it recurs (e.g. the `hidden` flag of a
  viewpoint annotation, see decisions).
- `#n` suffixes depend on sibling order; renames go through `rename()` / the sector layer rename
  signal (step 3), other name changes of name-keyed items lose their states.
- Pruning is subtree-scoped and runs on complete rebuilds only (step 4).
- Not yet verified in the running application.

## Step plan

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

## Future improvements

Weak spots of the current state, roughly by importance.

### F1 - Drop destroy and rebuild (architecture)

The producers destroy and rebuild their subtrees (geometry provider payloads, view aggregates,
reload emptying the tree), so the state lives outside the items and is carried over by key:
snapshot before the rows vanish, restore on insert. Works, but:

- two sources of truth between rebuilds (live view vs. last snapshot); correctness depends on
  capture timing (connect before `setModel()`, `rowsAboutToBeRemoved`, rename before the rebuild)
- identity via string keys (names, paths, ids, escaping, `#n`) - producers changing their naming
  silently break the restore
- per-row view state is not restored: scroll position (to check, may jump per live tick), hover,
  keyboard focus, tooltips; open context menus show entries of vanished items (safe since step 1,
  but they do nothing)
- the whole Data subtree is rebuilt and captured / restored per live tick

Dropping destroy and rebuild in favor of in-place reconciliation (step 8: keep the items, swap
payloads, insert / remove only changed rows) would improve the architecture: the tree view keeps
expansion, selection, scroll, focus and hover natively, context menus keep valid items, the cost
per tick drops, and the key map shrinks to the gaps it alone can cover (reload, panel handover,
data context switch, persisted config). Needs the payload lifetime decoupled from the provider's
rebuild (old payloads released only after the swap).

### F2 - Verification in the application

Nothing is verified in the running application yet; the geographic items have no unit tests.
Checklist: geographic live mode (collapsed / unchecked items across ticks), reload, database
close / open, application restart, preset apply, color mode change, sector rename / delete / edit,
measurement and annotation add / rename / delete, scatter annotation display, each of the other
four views with a reload, scroll position across live ticks.

### F3 - Data-complete flag

- cancelled and failed loads count as complete (the engine sends the final update in every case,
  the operation state is set only after it) => states of the items not loaded are dropped. Fix: the
  final update tells whether it is truncated, `ViewDataWidget` sets the flag only for a normal end.
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
- sector id reuse after a delete would let a new sector inherit remembered states within the
  session (pruning on complete rebuilds covers the session map, not the config, see F4)
- same-named data source groups (geographic view) share their `persistenceId()` (`group:<path>`)
  => their effective-hidden ids overwrite each other. Latent, no consumer reads group ids.
- duplicate keys are only logged

### F6 - Developer and user facing

- many concepts for developers (keys and `stateKeyId()`, deviations, default policies, complete vs.
  incomplete rebuilds, `rename()`), documented here only
- user visible: checked states of data items survive a reload but not a restart, context items
  survive both; map layer states are forgotten on a map switch. Document in the user manual if
  needed.

## Architecture notes

- **One panel per model** - expansion and selection are view state but live in the model's item
  state map (enables the handover between panels). Two tree views on one model would share them.
- **Default expansion policy signature** - the policy gets the panel depth, but
  `ViewLayerPanelWidget` computes the depth relative to the Data root itself and ignores the
  parameter there. Passing only the item (policies compute depth where needed) would be cleaner.
- **Config keys follow the tree layout** - path based keys orphan persisted settings when the tree
  is restructured (e.g. radars moved out of Context). Id based keys (step 2) reduce, but do not
  remove this.
- **Two sources for the checked state** - precedence on attach: session map, then config
  (persisted items), then the constructed default. On `configChanged()` (reconfigure, preset) the
  config wins over the map. Deliberate, covered by `test_layertreeitemconfig.cpp`.

## Decisions

- Presets keep carrying `layer_config` (item configs incl. persisted checked states); presets can
  be loaded without the layer config if not wanted.
- Only checked states persisted to the config survive a database close/open; expansion and
  selection are cleared then.
- Legacy `layer_config` sections are dropped (one-time loss accepted).
- Step 8 deferred: the DBContent subtree keeps being rebuilt per live tick / style refresh.
- Viewpoint annotations are pruned on a viewpoint change (step 4): vanished annotations lose their
  state, recurring ones (same key) keep it, also over the `hidden` flag of the new JSON.
- Pruning works the same in all views (step 4).
