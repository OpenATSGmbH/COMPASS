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

#include "json.hpp"

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
     * Removes all children of an attached parent and appends fresh ones.
     */
    void rebuildChildren(LayerTreeModel& model, LayerTreeItem* parent, const std::vector<Spec>& specs)
    {
        if (parent->childCount() > 0)
            model.removeChildItems(parent, 0, parent->childCount() - 1);

        for (auto& item : buildItems(specs))
            model.appendChildItem(parent, std::move(item));
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
            rebuildChildren(model, data, specs);
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

TEST_CASE("LayerTreeModel forgets the checked state of removed items", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->setHidden(true);
    f.item(1)->setHidden(false); // hidden by default

    SECTION("after a removal and a new insert")
    {
        f.rebuild(DefaultSpecs);

        REQUIRE(!f.item(0)->hidden());
        REQUIRE(f.item(1)->hidden());
    }

    SECTION("after a model reset")
    {
        f.resetRebuild(DefaultSpecs);

        REQUIRE(!f.item(0)->hidden());
        REQUIRE(f.item(1)->hidden());
    }
}

TEST_CASE("LayerTreeItem keys ids below the parent's key", "[view][layerpanel]")
{
    Fixture f;

    auto* item = f.data->appendChildNotified(std::make_unique<IdItem>("name", "id:1"));

    REQUIRE(item->stateKey() == "Data/id:1");
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
        REQUIRE(model.findItem("a/b") == b);
        REQUIRE(model.findItem("a\\/b") == slash);
    }
}

TEST_CASE("LayerTreeItem moves the item configs of a renamed item", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->appendChildNotified(std::make_unique<LayerTreeItem>("c"));
    f.item(0)->config()[ "x" ] = 1;
    f.item(0)->child(0)->config()[ "y" ] = 2;
    f.item(0)->setHidden(true);

    int changes = 0;
    QObject::connect(&f.model, &QAbstractItemModel::dataChanged, [ & ] { ++changes; });

    f.item(0)->rename("renamed");

    const auto& root = f.model.state().configRoot();

    REQUIRE(f.item(0)->name() == "renamed");
    REQUIRE(f.item(0)->hidden()); // the item keeps its state
    REQUIRE(changes == 1);
    REQUIRE(!root.contains("Data/d1"));
    REQUIRE(root.contains("Data/renamed"));
    REQUIRE(root.contains("Data/renamed/c"));

    SECTION("id-keyed items keep their key")
    {
        auto* id_item = f.data->appendChildNotified(std::make_unique<KeyIdItem>("name", "id"));
        id_item->config()[ "x" ] = 1;

        id_item->rename("other");

        REQUIRE(root.contains("Data/id"));
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
    rebuildChildren(*model, data, DefaultSpecs);

    std::vector<std::string> triggered;

    QAction action(nullptr);
    data->child(0)->connectItemAction(&action, [ & ] (LayerTreeItem& item) { triggered.push_back(item.stateKey()); });

    SECTION("a replaced item => its successor")
    {
        rebuildChildren(*model, data, DefaultSpecs);
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
        model.updateDataSubtree(entries);
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

TEST_CASE("ViewLayerTreeModel keeps the checked state of kept data rows only", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);

    TestPayload p1("p1"), p2("p2");

    const std::vector<DBContentRootItem::LeafEntry> both  = { { "Radar", "DS", "L1", "CAT048", &p1 },
                                                              { "Radar", "DS", "L1", "CAT021", &p2 } };
    const std::vector<DBContentRootItem::LeafEntry> only2 = { { "Radar", "DS", "L1", "CAT021", &p2 } };

    auto firstLeaf = [ & ] () { return model.dbContentRootItem()->child(0)->child(0)->child(0)->child(0); };

    model.setDataComplete(true);
    model.updateDataSubtree(both);

    firstLeaf()->setHidden(true);

    // kept
    model.updateDataSubtree(both);
    REQUIRE(firstLeaf()->hidden());

    // vanished, also from incomplete data => forgotten
    model.setDataComplete(false);
    model.updateDataSubtree(only2);
    REQUIRE(firstLeaf()->name() == "CAT021");

    model.updateDataSubtree(both);
    REQUIRE(firstLeaf()->name() == "CAT048");
    REQUIRE(!firstLeaf()->hidden());
}

TEST_CASE("LayerTreeModel signals hidden changes without a layout change", "[view][layerpanel]")
{
    Fixture f;

    QSignalSpy hidden_spy(&f.model, &LayerTreeModel::hiddenChangedSignal);

    int layout_changes = 0;
    QObject::connect(&f.model, &QAbstractItemModel::layoutChanged, [ & ] { ++layout_changes; });

    f.item(0)->setHidden(true);
    f.data->showSubtree();
    f.rebuild(DefaultSpecs); // new items with their defaults => no hidden change

    REQUIRE(hidden_spy.count() == 2);
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

    REQUIRE(resets == 0);
}

TEST_CASE("LayerTreeItem forgets removed items", "[view][layerpanel]")
{
    Fixture f;

    f.item(0)->appendChildNotified(std::make_unique<LayerTreeItem>("c"));
    f.item(0)->setHidden(true);
    f.item(0)->child(0)->setHidden(true);

    f.data->removeChildNotified(0);
    f.data->appendChildNotified(std::make_unique<TrackingItem>("d1"));

    REQUIRE(f.data->child(1)->name() == "d1");
    REQUIRE(!f.data->child(1)->hidden()); // back with its default
}

TEST_CASE("LayerTreeItem changes detached items without notification", "[view][layerpanel]")
{
    LayerTreeItem detached("Detached");

    detached.appendChildNotified(std::make_unique<LayerTreeItem>("a"));
    detached.appendChildNotified(std::make_unique<LayerTreeItem>("b"));
    detached.removeChildNotified(0);
    detached.notifyChanged();

    REQUIRE(detached.childCount() == 1);
    REQUIRE(detached.child(0)->name() == "b");
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

TEST_CASE("LayerTreeModel keeps the effective-hidden ids current", "[view][layerpanel]")
{
    Fixture f;

    auto* item = f.data->appendChildNotified(std::make_unique<IdItem>("name", "id:1"));
    REQUIRE(f.model.state().hiddenIds().empty());

    item->setHidden(true);
    REQUIRE(f.model.state().hiddenIds() == std::set<std::string>{ "id:1" });

    f.data->setHidden(false);
    item->setHidden(false);
    f.data->setHidden(true); // via the ancestor
    REQUIRE(f.model.state().hiddenIds() == std::set<std::string>{ "id:1" });

    // ids without an item stay until the database close
    f.data->removeChildNotified(f.data->indexOf(item));
    REQUIRE(f.model.state().hiddenIds() == std::set<std::string>{ "id:1" });

    f.model.state().clear();
    REQUIRE(f.model.state().hiddenIds().empty());
}
