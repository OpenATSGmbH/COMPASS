/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#include "catch.hpp"
#include "layertreeitem.h"
#include "layertreemodel.h"
#include "viewlayertreemodel.h"
#include "dbcontentlayer.h"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QAction>
#include <QSignalSpy>

namespace
{
    /**
     * Remembers the last effective hidden state pushed to it.
     */
    class TrackingItem : public LayerTreeItem
    {
    public:
        using LayerTreeItem::LayerTreeItem;

        void onEffectiveHiddenChanged() override { pushed_hidden = effectiveHidden(); }

        std::optional<bool> pushed_hidden;
    };

    /**
     * Item spec for a rebuild: name + constructed (default) hidden state.
     */
    struct Spec
    {
        std::string name;
        bool        hidden = false;
    };

    const std::vector<Spec> DefaultSpecs = { { "d1" }, { "d2", true } };

    /**
     * Fresh detached items for the given specs.
     */
    std::vector<std::unique_ptr<LayerTreeItem>> buildItems(const std::vector<Spec>& specs)
    {
        std::vector<std::unique_ptr<LayerTreeItem>> items;
        items.reserve(specs.size());

        for (const auto& spec : specs)
        {
            auto item = std::make_unique<TrackingItem>(spec.name);
            if (spec.hidden)
                item->setHidden(true);

            items.push_back(std::move(item));
        }

        return items;
    }

    /**
     * Top-level item "Data" whose children are rebuilt per load.
     */
    struct Fixture
    {
        Fixture()
        {
            data = model.addRootItem(std::make_unique<LayerTreeItem>("Data"));
            rebuild(DefaultSpecs);
        }

        void rebuild(const std::vector<Spec>& specs)
        {
            model.refreshSubtree(data, [ & ] () { return buildItems(specs); }, false);
        }

        void resetRebuild(const std::vector<Spec>& specs)
        {
            model.beginResetModel();

            data->clearChildren();
            for (auto& item : buildItems(specs))
                data->appendChild(std::move(item));

            model.endResetModel();
        }

        TrackingItem* item(int row) const { return static_cast<TrackingItem*>(data->child(row)); }

        LayerTreeModel model;
        LayerTreeItem* data = nullptr;
    };
}

TEST_CASE("LayerTreeModel restores the checked state of recurring items", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->setHidden(true);
    f.item(1)->setHidden(false); // hidden by default

    SECTION("after a subtree rebuild")
    {
        f.rebuild(DefaultSpecs);

        REQUIRE(f.item(0)->hidden());
        REQUIRE(!f.item(1)->hidden());
    }

    SECTION("after a model reset")
    {
        f.resetRebuild(DefaultSpecs);

        REQUIRE(f.item(0)->hidden());
        REQUIRE(!f.item(1)->hidden());
    }

    SECTION("after the item vanished for a while")
    {
        f.rebuild({ { "d2", true } });
        f.rebuild(DefaultSpecs);

        REQUIRE(f.item(0)->hidden());
    }

    SECTION("with a single hidden signal per rebuild")
    {
        QSignalSpy spy(&f.model, &LayerTreeModel::hiddenChangedSignal);

        f.rebuild(DefaultSpecs);

        REQUIRE(spy.count() == 1);
    }
}

TEST_CASE("LayerTreeModel stores only checked states deviating from the default", "[view][layerpanel]")
{
    Fixture f;

    REQUIRE(!f.item(0)->defaultHidden());
    REQUIRE(f.item(1)->defaultHidden());

    f.item(0)->setHidden(true);
    REQUIRE(f.model.state().itemState("Data/d1"));

    f.item(0)->setHidden(false);
    REQUIRE(!f.model.state().itemState("Data/d1"));

    f.data->showSubtree(); // bulk action: d1 stays default, d2 deviates

    REQUIRE(!f.model.state().itemState("Data/d1"));
    REQUIRE(f.model.state().itemState("Data/d2"));
    REQUIRE(!*f.model.state().itemState("Data/d2")->hidden);
}

TEST_CASE("LayerTreeModel does not remember checked states without PersistHidden", "[view][layerpanel]")
{
    Fixture f;

    REQUIRE(f.model.state().persistFlags() == LayerTreeState::PersistAll);

    f.item(0)->setHidden(true);

    f.model.state().setPersistFlags(LayerTreeState::PersistAll & ~LayerTreeState::PersistHidden);

    REQUIRE(!f.model.state().itemState("Data/d1")); // dropped on disable

    f.item(1)->setHidden(false);
    REQUIRE(!f.model.state().itemState("Data/d2"));

    f.rebuild(DefaultSpecs);

    REQUIRE(!f.item(0)->hidden());
    REQUIRE(f.item(1)->hidden());
}

TEST_CASE("LayerTreeModel prunes the states of missing items", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->setHidden(true);
    f.item(1)->setHidden(false);

    f.rebuild({ { "d2", true } }); // d1 missing
    f.model.pruneItemStates();

    REQUIRE(!f.model.state().itemState("Data/d1"));
    REQUIRE(f.model.state().itemState("Data/d2"));

    f.rebuild(DefaultSpecs);

    REQUIRE(!f.item(0)->hidden()); // back with its default
    REQUIRE(!f.item(1)->hidden()); // kept
}

namespace
{
    /**
     * Model which never prunes.
     */
    class KeepingModel : public LayerTreeModel
    {
    protected:
        bool doPruneItem(const std::string&, const LayerTreeState::ItemState&, const std::set<std::string>&) const override
        {
            return false;
        }
    };
}

TEST_CASE("LayerTreeModel prunes by the doPruneItem() criterion", "[view][layerpanel]")
{
    KeepingModel model;

    auto* data = model.addRootItem(std::make_unique<LayerTreeItem>("Data"));
    model.refreshSubtree(data, [] () { return buildItems(DefaultSpecs); }, false);

    data->child(0)->setHidden(true);

    model.refreshSubtree(data, [] () { return buildItems({}); }, false);
    model.pruneItemStates();

    REQUIRE(model.state().itemState("Data/d1"));
}

namespace
{
    /**
     * Item keyed by an id instead of its name.
     */
    class IdItem : public LayerTreeItem
    {
    public:
        IdItem(const std::string& name, const std::string& id) : LayerTreeItem(name), id_(id) {}

        std::string persistenceId() const override { return id_; }

    private:
        std::string id_;
    };
}

TEST_CASE("LayerTreeItem keys ids below the parent's key", "[view][layerpanel]")
{
    Fixture f;

    auto* item = f.data->appendChildNotified(std::make_unique<IdItem>("name", "id:1"));

    REQUIRE(item->stateKey() == "Data/id:1");
}

namespace
{
    /**
     * Item identified among its siblings by an id instead of its name.
     */
    class KeyIdItem : public LayerTreeItem
    {
    public:
        KeyIdItem(const std::string& name, const std::string& id) : LayerTreeItem(name), id_(id) {}

        std::string stateKeyId() const override { return id_; }

    private:
        std::string id_;
    };

    /**
     * Minimal DBContent leaf payload.
     */
    class TestPayload : public DBContentLeafPayload
    {
    public:
        explicit TestPayload(const std::string& id) : id_(id) {}

        std::string name() const override { return id_; }
        unsigned int count() const override { return 1; }
        QColor color() const override { return QColor(); }
        void setVisible(bool) override {}
        std::string persistenceId() const override { return id_; }

    private:
        std::string id_;
    };
}

TEST_CASE("LayerTreeItem escapes the key separators in names and ids", "[view][layerpanel]")
{
    LayerTreeModel model;

    auto* slash = model.addRootItem(std::make_unique<LayerTreeItem>("a/b"));
    auto* a     = model.addRootItem(std::make_unique<LayerTreeItem>("a"));
    auto* b     = a->appendChildNotified(std::make_unique<LayerTreeItem>("b"));

    REQUIRE(slash->stateKey() == "a\\/b");
    REQUIRE(b->stateKey() == "a/b");

    SECTION("a literal '#' does not collide with a repetition suffix")
    {
        auto* hash = a->appendChildNotified(std::make_unique<LayerTreeItem>("x#1"));
        a->appendChildNotified(std::make_unique<LayerTreeItem>("x"));
        auto* x2 = a->appendChildNotified(std::make_unique<LayerTreeItem>("x"));

        REQUIRE(hash->stateKey() == "a/x\\#1");
        REQUIRE(x2->stateKey() == "a/x#1");
    }

    SECTION("ids are escaped and get the parent's key as prefix")
    {
        auto* id_item = a->appendChildNotified(std::make_unique<KeyIdItem>("name", "12/34"));

        REQUIRE(id_item->stateKey() == "a/12\\/34");
    }

    SECTION("a '/' in a name does not put the item into another subtree")
    {
        slash->setHidden(true);
        b->setHidden(true);

        model.refreshSubtree(a, [] () { return std::vector<std::unique_ptr<LayerTreeItem>>(); }, true);

        REQUIRE(!model.state().itemState("a/b"));
        REQUIRE(model.state().itemState("a\\/b"));
    }
}

TEST_CASE("LayerTreeItem keeps the remembered states of a renamed item", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->appendChildNotified(std::make_unique<LayerTreeItem>("c"));
    f.item(0)->setHidden(true);
    f.item(0)->child(0)->setHidden(true);

    int changes = 0;
    QObject::connect(&f.model, &QAbstractItemModel::dataChanged, [ & ] { ++changes; });

    f.item(0)->rename("renamed");

    REQUIRE(f.item(0)->name() == "renamed");
    REQUIRE(changes == 1);
    REQUIRE(!f.model.state().itemState("Data/d1"));
    REQUIRE(f.model.state().itemState("Data/renamed"));
    REQUIRE(f.model.state().itemState("Data/renamed/c"));

    SECTION("a rebuild with the new name restores them")
    {
        f.rebuild({ { "renamed" }, { "d2", true } });

        REQUIRE(f.item(0)->hidden());
    }

    SECTION("id-keyed items keep their key")
    {
        auto* id_item = f.data->appendChildNotified(std::make_unique<KeyIdItem>("name", "id"));
        id_item->setHidden(true);

        id_item->rename("other");

        REQUIRE(f.model.state().itemState("Data/id"));
    }
}

TEST_CASE("LayerTreeModel finds items by state key", "[view][layerpanel]")
{
    Fixture f;

    auto* c = f.item(0)->appendChildNotified(std::make_unique<LayerTreeItem>("c"));
    f.model.addRootItem(std::make_unique<LayerTreeItem>("Database")); // "Data" as name prefix

    REQUIRE(f.model.findItem("Data") == f.data);
    REQUIRE(f.model.findItem("Data/d1/c") == c);
    REQUIRE(f.model.findItem("Data/d3") == nullptr);
    REQUIRE(f.model.findItem("") == nullptr);
}

TEST_CASE("LayerTreeItem resolves menu actions by key when triggered", "[view][layerpanel]")
{
    auto model = std::make_unique<LayerTreeModel>();
    auto* data = model->addRootItem(std::make_unique<LayerTreeItem>("Data"));
    model->refreshSubtree(data, [] () { return buildItems(DefaultSpecs); }, false);

    std::vector<std::string> triggered;

    QAction action(nullptr);
    data->child(0)->connectItemAction(&action, [ & ] (LayerTreeItem& item) { triggered.push_back(item.stateKey()); });

    SECTION("a rebuilt item => its successor")
    {
        model->refreshSubtree(data, [] () { return buildItems(DefaultSpecs); }, false);
        action.trigger();

        REQUIRE(triggered == std::vector<std::string>{ "Data/d1" });
    }

    SECTION("a removed item => nothing")
    {
        data->removeChildNotified(0);
        action.trigger();

        REQUIRE(triggered.empty());
    }

    SECTION("a destroyed model => nothing")
    {
        model.reset();
        action.trigger();

        REQUIRE(triggered.empty());
    }
}

TEST_CASE("DBContentRootItem groups data sources by id if given", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);

    auto* root = model.dbContentRootItem();

    TestPayload p1("p1"), p2("p2");

    auto rebuild = [ & ] (const std::vector<DBContentRootItem::LeafEntry>& entries)
    {
        model.refreshDataSubtree([ & ] () { return root->buildChildrenFrom(entries); });
    };

    SECTION("same-named data sources with ids stay apart")
    {
        rebuild({ { "Radar", "DS", "L1", "CAT048", &p1, 1u },
                  { "Radar", "DS", "L1", "CAT048", &p2, 2u } });

        REQUIRE(root->childCount() == 1);
        REQUIRE(root->child(0)->childCount() == 2);
        REQUIRE(root->child(0)->child(0)->stateKey() == "Data/Radar/ds:1");
        REQUIRE(root->child(0)->child(1)->stateKey() == "Data/Radar/ds:2");
    }

    SECTION("data sources without ids are grouped by name")
    {
        rebuild({ { "Radar", "DS", "L1", "CAT048", &p1 },
                  { "Radar", "DS", "L2", "CAT048", &p2 } });

        REQUIRE(root->child(0)->childCount() == 1);
        REQUIRE(root->child(0)->child(0)->stateKey() == "Data/Radar/DS");
    }
}

TEST_CASE("LayerTreeModel prunes only the states below the given scope", "[view][layerpanel]")
{
    Fixture f;

    auto* other    = f.model.addRootItem(std::make_unique<LayerTreeItem>("Other"));
    auto* database = f.model.addRootItem(std::make_unique<LayerTreeItem>("Database")); // "Data" as name prefix
    other->appendChildNotified(std::make_unique<LayerTreeItem>("o1"));
    database->appendChildNotified(std::make_unique<LayerTreeItem>("b1"));

    f.data->setHidden(true); // the scope's own entry
    f.item(0)->setHidden(true);
    other->child(0)->setHidden(true);
    database->child(0)->setHidden(true);

    // rebuilds without pruning => d1, o1, b1 missing
    auto none = [] () { return std::vector<std::unique_ptr<LayerTreeItem>>(); };

    f.rebuild({ { "d2", true } });
    f.model.refreshSubtree(other, none, false);
    f.model.refreshSubtree(database, none, false);

    f.model.pruneItemStates(f.data);

    REQUIRE(!f.model.state().itemState("Data/d1"));
    REQUIRE(f.model.state().itemState("Data"));
    REQUIRE(f.model.state().itemState("Other/o1"));
    REQUIRE(f.model.state().itemState("Database/b1"));

    SECTION("pruned by its own scope")
    {
        f.model.pruneItemStates(other);

        REQUIRE(!f.model.state().itemState("Other/o1"));
        REQUIRE(f.model.state().itemState("Database/b1"));
    }

    SECTION("pruned by the whole tree")
    {
        f.model.pruneItemStates();

        REQUIRE(!f.model.state().itemState("Other/o1"));
        REQUIRE(!f.model.state().itemState("Database/b1"));
        REQUIRE(f.model.state().itemState("Data"));
    }
}

TEST_CASE("ViewLayerTreeModel prunes data states only while the data is complete", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);

    auto rebuild = [ & ] (const std::vector<Spec>& specs)
    {
        model.refreshDataSubtree([ & ] () { return buildItems(specs); });
    };

    model.setDataComplete(true);
    rebuild(DefaultSpecs);

    const std::string key = model.dbContentRootItem()->child(0)->stateKey();
    model.dbContentRootItem()->child(0)->setHidden(true);

    // reload: cleared => emptied, then rebuilt from the final data
    model.setDataComplete(false);
    rebuild({});

    REQUIRE(model.state().itemState(key));

    model.setDataComplete(true);
    rebuild(DefaultSpecs);

    REQUIRE(model.dbContentRootItem()->child(0)->hidden());

    // live tick / reload without the item
    rebuild({ { "d2", true } });

    REQUIRE(!model.state().itemState(key));
}

TEST_CASE("LayerTreeModel signals hidden changes without a layout change", "[view][layerpanel]")
{
    Fixture f;

    QSignalSpy hidden_spy(&f.model, &LayerTreeModel::hiddenChangedSignal);

    int layout_changes = 0;
    QObject::connect(&f.model, &QAbstractItemModel::layoutChanged, [ & ] { ++layout_changes; });

    f.item(0)->setHidden(true);
    f.data->showSubtree();
    f.rebuild(DefaultSpecs); // restores d1 unchecked

    REQUIRE(hidden_spy.count() == 3);
    REQUIRE(layout_changes == 0);
}

TEST_CASE("LayerTreeItem notifies structural changes of attached items without a reset", "[view][layerpanel]")
{
    Fixture f;

    int resets = 0, inserts = 0, removes = 0, changes = 0;
    QObject::connect(&f.model, &QAbstractItemModel::modelReset,   [ & ] { ++resets; });
    QObject::connect(&f.model, &QAbstractItemModel::rowsInserted, [ & ] { ++inserts; });
    QObject::connect(&f.model, &QAbstractItemModel::rowsRemoved,  [ & ] { ++removes; });
    QObject::connect(&f.model, &QAbstractItemModel::dataChanged,  [ & ] { ++changes; });

    f.data->appendChildNotified(std::make_unique<LayerTreeItem>("d3"));
    REQUIRE(f.data->childCount() == 3);
    REQUIRE(inserts == 1);

    f.data->removeChildNotified(2);
    REQUIRE(f.data->childCount() == 2);
    REQUIRE(removes == 1);

    f.item(0)->setName("renamed");
    f.item(0)->notifyChanged();
    REQUIRE(changes == 1);

    f.data->replaceChildren([] () { return buildItems(DefaultSpecs); });
    REQUIRE(f.data->childCount() == 2);
    REQUIRE(removes == 2);
    REQUIRE(inserts == 2);

    REQUIRE(resets == 0);
}

TEST_CASE("LayerTreeItem drops the states of removed items", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->appendChildNotified(std::make_unique<LayerTreeItem>("c"));
    f.item(0)->setHidden(true);
    f.item(0)->child(0)->setHidden(true);
    f.item(1)->setHidden(false);

    f.data->removeChildNotified(0);

    REQUIRE(!f.model.state().itemState("Data/d1"));
    REQUIRE(!f.model.state().itemState("Data/d1/c"));
    REQUIRE(f.model.state().itemState("Data/d2"));

    f.data->appendChildNotified(std::make_unique<TrackingItem>("d1"));

    REQUIRE(f.data->child(1)->name() == "d1");
    REQUIRE(!f.data->child(1)->hidden()); // back with its default
}

TEST_CASE("LayerTreeItem prunes on complete child replacements only", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->setHidden(true);

    SECTION("complete")
    {
        f.data->replaceChildren([] () { return buildItems({ { "d2", true } }); });

        REQUIRE(!f.model.state().itemState("Data/d1"));
    }

    SECTION("incomplete")
    {
        f.data->replaceChildren([] () { return buildItems({ { "d2", true } }); }, false);

        REQUIRE(f.model.state().itemState("Data/d1"));
    }
}

TEST_CASE("LayerTreeItem changes detached items without notification", "[view][layerpanel]")
{
    LayerTreeItem detached("Detached");

    detached.appendChildNotified(std::make_unique<LayerTreeItem>("a"));
    detached.replaceChildren([] () { return buildItems({ { "b" }, { "c" } }); });
    detached.removeChildNotified(0);
    detached.notifyChanged();

    REQUIRE(detached.childCount() == 1);
    REQUIRE(detached.child(0)->name() == "c");
}

TEST_CASE("LayerTreeModel keeps the default of new items", "[view][layerpanel]")
{
    Fixture f;

    f.rebuild({ { "d1" }, { "d2", true }, { "d3" }, { "d4", true } });

    REQUIRE(!f.item(2)->hidden());
    REQUIRE(f.item(3)->hidden());
}

TEST_CASE("LayerTreeModel pushes the effective state to items under an unchecked ancestor", "[view][layerpanel]")
{
    Fixture f;

    f.data->setHidden(true);

    f.rebuild(DefaultSpecs);

    REQUIRE(f.item(0)->pushed_hidden.has_value());
    REQUIRE(*f.item(0)->pushed_hidden);
}

TEST_CASE("LayerTreeModel forgets the item states on database close", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->setHidden(true);

    f.model.state().clear();

    REQUIRE(!f.model.state().itemState("Data/d1"));

    f.rebuild(DefaultSpecs);

    REQUIRE(!f.item(0)->hidden());
}
