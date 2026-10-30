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
#include "layerpanelwidget.h"
#include "layertreeitem.h"
#include "layertreeitemdelegate.h"
#include "layertreemodel.h"
#include "viewlayertreemodel.h"
#include "viewlayerscan.h"
#include "dbcontentlayer.h"
#include "annotationsrootitem.h"
#include "annotationleafitem.h"

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <QSignalSpy>
#include <QTreeView>

namespace
{
    /**
     * Domain the test items are updated from.
     */
    struct Domain
    {
        std::map<std::string, std::vector<std::string>> children;        // dynamic item path -> child keys
        std::set<std::string>                            changed;         // paths whose display changes on update
        std::vector<std::string>                         subtree_updated; // paths in onSubtreeUpdated() order
        int                                              created = 0;     // created children
    };

    /**
     * Item with dynamic children if the domain lists its path, else static.
     */
    class DomainItem : public LayerTreeItem
    {
    public:
        DomainItem(const std::string& name, Domain& domain, const std::string& path)
        :   LayerTreeItem(name)
        ,   domain_(domain)
        ,   path_  (path)
        {
        }

        std::optional<std::vector<std::string>> childKeys() const override
        {
            auto it = domain_.children.find(path_);
            if (it == domain_.children.end())
                return std::nullopt;

            return it->second;
        }

        std::unique_ptr<LayerTreeItem> createChild(const std::string& key) override
        {
            ++domain_.created;
            return std::make_unique<DomainItem>(key, domain_, path_ + "/" + key);
        }

        bool updateChild(LayerTreeItem& child) override
        {
            return domain_.changed.count(static_cast<DomainItem&>(child).path_) > 0;
        }

        void onSubtreeUpdated() override { domain_.subtree_updated.push_back(path_); }

    private:
        Domain&     domain_;
        std::string path_;
    };

    /**
     * Model with a top-level item "Data" updated from the domain.
     */
    struct Fixture
    {
        Fixture()
        {
            data = model.addRootItem(std::make_unique<DomainItem>("Data", domain, "Data"));
        }

        Domain         domain;
        LayerTreeModel model;
        LayerTreeItem* data = nullptr;
    };

    /**
     */
    std::vector<std::string> childNames(const LayerTreeItem* item)
    {
        std::vector<std::string> names;
        for (int i = 0; i < item->childCount(); ++i)
            names.push_back(item->child(i)->name());

        return names;
    }

    /**
     */
    QModelIndex indexOf(const LayerTreeModel& model, const LayerTreeItem* item)
    {
        const LayerTreeItem* parent     = item->parentItem();
        const QModelIndex    parent_idx = parent == model.invisibleRootItem() ? QModelIndex() : indexOf(model, parent);

        return model.index(item->row(), 0, parent_idx);
    }

    /**
     * True if one of the recorded dataChanged ranges covers the index.
     */
    bool covers(const QSignalSpy& data_changed, const QModelIndex& index)
    {
        for (const auto& args : data_changed)
        {
            const QModelIndex top_left     = args.at(0).value<QModelIndex>();
            const QModelIndex bottom_right = args.at(1).value<QModelIndex>();

            if (top_left.parent() == index.parent() &&
                top_left.row() <= index.row() &&
                bottom_right.row() >= index.row())
                return true;
        }

        return false;
    }

    /**
     */
    size_t position(const std::vector<std::string>& values, const std::string& value)
    {
        for (size_t i = 0; i < values.size(); ++i)
            if (values[ i ] == value)
                return i;

        return values.size();
    }

    /**
     * Leaf payload remembering the visibility pushed to it.
     */
    class TrackingPayload : public DBContentLeafPayload
    {
    public:
        TrackingPayload(const std::string& id, const QColor& color = QColor())
        :   id_   (id)
        ,   color_(color)
        {
        }

        std::string name() const override { return id_; }
        unsigned int count() const override { return 1; }
        QColor color() const override { return color_; }
        void setVisible(bool v) override { visible = v; }
        std::string persistenceId() const override { return id_; }

        std::optional<bool> visible;

    private:
        std::string id_;
        QColor      color_;
    };

    typedef std::vector<DBContentRootItem::LeafEntry> Entries;

    /**
     * One data source with the given leaves (payload ids), all on line L1.
     */
    Entries entriesOf(const std::vector<TrackingPayload*>& payloads)
    {
        Entries entries;
        for (auto* payload : payloads)
            entries.push_back({ "Radar", "DS", "L1", payload->persistenceId(), payload, 1u });

        return entries;
    }

    /**
     */
    LayerTreeItem* lineOf(ViewLayerTreeModel& model)
    {
        return model.dbContentRootItem()->child(0)->child(0)->child(0);
    }

    /**
     */
    DBContentLeafItem* leafOf(ViewLayerTreeModel& model, int row)
    {
        return static_cast<DBContentLeafItem*>(lineOf(model)->child(row));
    }

    typedef std::vector<VariableView::AnnotationGroup> AnnotationGroups;

    /**
     * Group with annotations of the given titles.
     */
    VariableView::AnnotationGroup annotationGroup(const std::string& name, const std::vector<std::string>& titles)
    {
        VariableView::AnnotationGroup group;
        group.name = name;

        for (const auto& title : titles)
            group.annotations.push_back({ PlotMetadata(title, "", "", ""), {} });

        return group;
    }

    /**
     */
    AnnotationLeafItem* annotationLeaf(LayerTreeItem* parent, int row)
    {
        return dynamic_cast<AnnotationLeafItem*>(parent->child(row));
    }
}

TEST_CASE("LayerTreeModel update creates, keeps and drops children in place", "[view][layerpanel]")
{
    Fixture f;

    QSignalSpy inserted(&f.model, &QAbstractItemModel::rowsInserted);
    QSignalSpy removed (&f.model, &QAbstractItemModel::rowsRemoved);
    QSignalSpy moved   (&f.model, &QAbstractItemModel::rowsMoved);

    f.domain.children[ "Data" ] = { "a", "b", "c" };
    f.data->update();

    REQUIRE(childNames(f.data) == std::vector<std::string>{ "a", "b", "c" });
    REQUIRE(f.domain.created == 3);

    // new children inserted together
    REQUIRE(inserted.count() == 1);
    REQUIRE(inserted.at(0).at(1).toInt() == 0);
    REQUIRE(inserted.at(0).at(2).toInt() == 2);

    LayerTreeItem* b = f.data->child(1);
    LayerTreeItem* c = f.data->child(2);

    inserted.clear();

    f.domain.children[ "Data" ] = { "b", "c", "d" };
    f.data->update();

    REQUIRE(childNames(f.data) == std::vector<std::string>{ "b", "c", "d" });
    REQUIRE(f.data->child(0) == b);
    REQUIRE(f.data->child(1) == c);
    REQUIRE(f.domain.created == 4);

    // only the vanished row removed, only the new one inserted, kept rows untouched
    REQUIRE(removed.count() == 1);
    REQUIRE(removed.at(0).at(1).toInt() == 0);
    REQUIRE(removed.at(0).at(2).toInt() == 0);

    REQUIRE(inserted.count() == 1);
    REQUIRE(inserted.at(0).at(1).toInt() == 2);
    REQUIRE(inserted.at(0).at(2).toInt() == 2);

    REQUIRE(moved.count() == 0);
}

TEST_CASE("LayerTreeModel update moves reordered children", "[view][layerpanel]")
{
    Fixture f;

    f.domain.children[ "Data" ] = { "a", "b", "c" };
    f.data->update();

    LayerTreeItem* a = f.data->child(0);
    LayerTreeItem* c = f.data->child(2);

    QSignalSpy inserted(&f.model, &QAbstractItemModel::rowsInserted);
    QSignalSpy removed (&f.model, &QAbstractItemModel::rowsRemoved);
    QSignalSpy moved   (&f.model, &QAbstractItemModel::rowsMoved);

    f.domain.children[ "Data" ] = { "c", "a", "b" };
    f.data->update();

    REQUIRE(childNames(f.data) == std::vector<std::string>{ "c", "a", "b" });
    REQUIRE(f.data->child(0) == c);
    REQUIRE(f.data->child(1) == a);

    REQUIRE(moved.count() == 1);
    REQUIRE(inserted.count() == 0);
    REQUIRE(removed.count() == 0);
    REQUIRE(f.domain.created == 3);
}

TEST_CASE("LayerTreeModel update inserts new children with their subtree", "[view][layerpanel]")
{
    Fixture f;

    QSignalSpy inserted(&f.model, &QAbstractItemModel::rowsInserted);

    f.domain.children[ "Data" ]   = { "g" };
    f.domain.children[ "Data/g" ] = { "x", "y" };
    f.data->update();

    // one insert below Data, the new group arrives filled
    REQUIRE(inserted.count() == 1);
    REQUIRE(childNames(f.data->child(0)) == std::vector<std::string>{ "x", "y" });
    REQUIRE(f.data->child(0)->child(0)->model() == &f.model);
}

TEST_CASE("LayerTreeModel update passes through static items", "[view][layerpanel]")
{
    Fixture f;

    LayerTreeItem* top = f.model.addRootItem(std::make_unique<LayerTreeItem>("Static"));
    LayerTreeItem* dyn = top->appendChildNotified(std::make_unique<DomainItem>("Dyn", f.domain, "Dyn"));

    f.domain.children[ "Dyn" ] = { "a" };
    f.model.updateAll();

    REQUIRE(top->childCount() == 1);
    REQUIRE(childNames(dyn) == std::vector<std::string>{ "a" });

    // "Data" is not listed => static, untouched
    REQUIRE(f.data->childCount() == 0);
}

TEST_CASE("LayerTreeModel update keeps the state of kept children, forgets removed ones", "[view][layerpanel]")
{
    Fixture f;

    f.domain.children[ "Data" ] = { "a", "b" };
    f.data->update();
    f.data->child(0)->setHidden(true);
    f.data->child(1)->setHidden(true);

    f.domain.children[ "Data" ] = { "a" };
    f.data->update();

    f.domain.children[ "Data" ] = { "a", "b" };
    f.data->update();

    REQUIRE(f.data->child(0)->hidden());         // kept
    REQUIRE(f.data->child(1)->name() == "b");
    REQUIRE(!f.data->child(1)->hidden());        // returned with its default
}

TEST_CASE("LayerTreeModel update notifies changed kept rows and their ancestors", "[view][layerpanel]")
{
    Fixture f;

    f.domain.children[ "Data" ]   = { "g" };
    f.domain.children[ "Data/g" ] = { "x" };
    f.data->update();

    LayerTreeItem* g = f.data->child(0);
    LayerTreeItem* x = g->child(0);

    QSignalSpy data_changed(&f.model, &QAbstractItemModel::dataChanged);

    SECTION("nothing changed => no notification")
    {
        f.data->update();

        REQUIRE(data_changed.count() == 0);
    }

    SECTION("a changed leaf => the leaf and its ancestors (aggregates)")
    {
        f.domain.changed = { "Data/g/x" };
        f.data->update();

        REQUIRE(covers(data_changed, indexOf(f.model, x)));
        REQUIRE(covers(data_changed, indexOf(f.model, g)));
        REQUIRE(covers(data_changed, indexOf(f.model, f.data)));
    }

    SECTION("a structural change below => the ancestors")
    {
        f.domain.children[ "Data/g" ] = { "x", "y" };
        f.data->update();

        REQUIRE(covers(data_changed, indexOf(f.model, g)));
        REQUIRE(covers(data_changed, indexOf(f.model, f.data)));
    }
}

TEST_CASE("LayerTreeModel update skips duplicate keys", "[view][layerpanel]")
{
    Fixture f;

    f.domain.children[ "Data" ] = { "a", "a", "b" };
    f.data->update();

    REQUIRE(childNames(f.data) == std::vector<std::string>{ "a", "b" });
}

TEST_CASE("LayerTreeModel update calls onSubtreeUpdated bottom-up", "[view][layerpanel]")
{
    Fixture f;

    f.domain.children[ "Data" ]   = { "g" };
    f.domain.children[ "Data/g" ] = { "x" };

    auto checkOrder = [ & ] ()
    {
        const auto& order = f.domain.subtree_updated;

        REQUIRE(position(order, "Data/g/x") < position(order, "Data/g"));
        REQUIRE(position(order, "Data/g") < position(order, "Data"));
        REQUIRE(position(order, "Data") < order.size());
    };

    // new items
    f.data->update();
    checkOrder();

    // kept items
    f.domain.subtree_updated.clear();
    f.data->update();
    checkOrder();
}

TEST_CASE("LayerTreeItem update fills a detached item", "[view][layerpanel]")
{
    Domain     domain;
    DomainItem item("Data", domain, "Data");

    domain.children[ "Data" ] = { "a", "b" };
    item.update();

    REQUIRE(childNames(&item) == std::vector<std::string>{ "a", "b" });

    LayerTreeItem* b = item.child(1);

    domain.children[ "Data" ] = { "b" };
    item.update();

    REQUIRE(childNames(&item) == std::vector<std::string>{ "b" });
    REQUIRE(item.child(0) == b);
}

TEST_CASE("LayerTreeModel update keeps the expansion of kept rows in a panel", "[view][layerpanel]")
{
    Fixture f;

    f.domain.children[ "Data" ]   = { "g", "h" };
    f.domain.children[ "Data/g" ] = { "x" };
    f.domain.children[ "Data/h" ] = { "y" };
    f.data->update();

    LayerPanelWidget panel(nullptr, nullptr, &f.model, true);
    QTreeView*       view = panel.treeView();

    LayerTreeItem* g = f.data->child(0);
    LayerTreeItem* h = f.data->child(1);

    view->expand(indexOf(f.model, f.data));
    view->expand(indexOf(f.model, g));
    view->collapse(indexOf(f.model, h));

    // move, insert and a changed subtree
    f.domain.children[ "Data" ]   = { "h", "g", "k" };
    f.domain.children[ "Data/g" ] = { "x", "z" };
    f.data->update();

    REQUIRE(f.data->child(0) == h);
    REQUIRE(f.data->child(1) == g);

    REQUIRE(view->isExpanded(indexOf(f.model, f.data)));
    REQUIRE(view->isExpanded(indexOf(f.model, g)));
    REQUIRE(!view->isExpanded(indexOf(f.model, h)));
}

TEST_CASE("ViewLayerTreeModel updates the data subtree in place", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);
    model.setDataComplete(true);

    TrackingPayload a("a"), b("b");
    model.updateDataSubtree(entriesOf({ &a, &b }));

    DBContentLeafItem* leaf_a = leafOf(model, 0);
    DBContentLeafItem* leaf_b = leafOf(model, 1);

    leaf_b->setHidden(true);

    // next tick: new payloads for the same layers
    TrackingPayload a2("a", Qt::red), b2("b"), c2("c");
    model.updateDataSubtree(entriesOf({ &a2, &b2, &c2 }));

    // kept leaves rebound, color re-read, visibility pushed
    REQUIRE(leafOf(model, 0) == leaf_a);
    REQUIRE(leafOf(model, 1) == leaf_b);
    REQUIRE(leaf_a->payload() == &a2);
    REQUIRE(leaf_a->color() == QColor(Qt::red));
    REQUIRE(a2.visible == std::optional<bool>(true));
    REQUIRE(b2.visible == std::optional<bool>(false));

    // new leaf
    REQUIRE(lineOf(model)->childCount() == 3);
    REQUIRE(leafOf(model, 2)->payload() == &c2);
}

TEST_CASE("ViewLayerTreeModel puts top leaves first", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);

    TrackingPayload a("a"), selected("Selected");
    model.updateDataSubtree(entriesOf({ &a }), { { "Selected", &selected } });

    auto* root = model.dbContentRootItem();

    REQUIRE(root->childCount() == 2);
    REQUIRE(root->child(0)->name() == "Selected");
    REQUIRE(static_cast<DBContentLeafItem*>(root->child(0))->payload() == &selected);
    REQUIRE(root->child(1)->name() == "Radar");
}

TEST_CASE("ViewLayerTreeModel keeps the data rows stale during a reload", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);
    model.setDataComplete(true);

    TrackingPayload a("a"), b("b");
    model.updateDataSubtree(entriesOf({ &a, &b }));

    DBContentLeafItem* leaf_a = leafOf(model, 0);

    QSignalSpy stale_changed(&model, &LayerTreeModel::staleChangedSignal);

    model.beginDataReload();
    model.setDataComplete(false);

    // rows stay, detached, shown stale with an empty count
    REQUIRE(model.hasStaleItems());
    REQUIRE(stale_changed.count() == 1);
    REQUIRE(leafOf(model, 0) == leaf_a);
    REQUIRE(!leaf_a->payload());
    REQUIRE(leaf_a->isStale());
    REQUIRE(lineOf(model)->isStale());
    REQUIRE(!leaf_a->itemData(1).isValid());

    // a second start changes nothing
    model.beginDataReload();
    REQUIRE(stale_changed.count() == 1);

    // a toggle while detached is pushed on rebind
    leaf_a->setHidden(true);

    TrackingPayload a2("a"), c2("c");

    SECTION("partial data is skipped, the final data ends the reload")
    {
        model.updateDataSubtree(entriesOf({ &c2 }));

        REQUIRE(lineOf(model)->childCount() == 2);
        REQUIRE(model.hasStaleItems());

        model.setDataComplete(true);
        model.updateDataSubtree(entriesOf({ &a2, &c2 }));

        REQUIRE(!model.hasStaleItems());
        REQUIRE(stale_changed.count() == 2);
        REQUIRE(leafOf(model, 0) == leaf_a);
        REQUIRE(leaf_a->payload() == &a2);
        REQUIRE(a2.visible == std::optional<bool>(false));
        REQUIRE(!leaf_a->isStale());

        // b vanished
        REQUIRE(lineOf(model)->childCount() == 2);
        REQUIRE(leafOf(model, 1)->name() == "c");
    }

    SECTION("a display not from the loaded data ends the reload")
    {
        model.updateDataSubtree(entriesOf({ &c2 }), {}, true);

        REQUIRE(!model.hasStaleItems());
        REQUIRE(lineOf(model)->childCount() == 1);
        REQUIRE(leafOf(model, 0)->name() == "c");
    }

    SECTION("a clear ends the reload")
    {
        model.clearDataSubtree();

        REQUIRE(!model.hasStaleItems());
        REQUIRE(model.dbContentRootItem()->childCount() == 0);
    }
}

TEST_CASE("ViewLayerTreeModel does not start a reload of an empty data subtree", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);

    model.beginDataReload();

    REQUIRE(!model.hasStaleItems());
    REQUIRE(!model.dataReloading());
}

TEST_CASE("AnnotationsRootItem updates its subtree in place", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, true);
    auto* root = model.annotationsRootItem();

    root->updateFrom({ annotationGroup("G1", { "A", "B" }), annotationGroup("", { "C" }) }, 0, 1, nullptr);

    // named group, then the unnamed group's annotation directly below the root
    REQUIRE(childNames(root) == std::vector<std::string>{ "G1", "C" });

    LayerTreeItem*      g1 = root->child(0);
    AnnotationLeafItem* a  = annotationLeaf(g1, 0);
    AnnotationLeafItem* b  = annotationLeaf(g1, 1);

    REQUIRE(a->hidden());
    REQUIRE(!b->hidden());

    // a new annotation in front, another one current
    root->updateFrom({ annotationGroup("G1", { "D", "A", "B" }), annotationGroup("", { "C" }) }, 0, 1, nullptr);

    REQUIRE(root->child(0) == g1);
    REQUIRE(childNames(g1) == std::vector<std::string>{ "D", "A", "B" });
    REQUIRE(annotationLeaf(g1, 1) == a);
    REQUIRE(annotationLeaf(g1, 2) == b);

    // indices and radio state follow the view
    REQUIRE(a->annotationIdx() == 1);
    REQUIRE(!a->hidden());
    REQUIRE(b->hidden());
    REQUIRE(annotationLeaf(g1, 0)->hidden());

    // vanished group
    root->updateFrom({ annotationGroup("", { "C" }) }, 0, 0, nullptr);

    REQUIRE(childNames(root) == std::vector<std::string>{ "C" });
    REQUIRE(!annotationLeaf(root, 0)->hidden());
}

TEST_CASE("AnnotationsRootItem keeps repeated titles apart", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, true);
    auto* root = model.annotationsRootItem();

    root->updateFrom({ annotationGroup("G", { "A", "A" }) }, 0, 1, nullptr);

    LayerTreeItem* g = root->child(0);

    REQUIRE(g->childCount() == 2);

    AnnotationLeafItem* first  = annotationLeaf(g, 0);
    AnnotationLeafItem* second = annotationLeaf(g, 1);

    REQUIRE(first->updateKey() != second->updateKey());
    REQUIRE(!second->hidden());

    root->updateFrom({ annotationGroup("G", { "A", "A" }) }, 0, 0, nullptr);

    REQUIRE(annotationLeaf(g, 0) == first);
    REQUIRE(annotationLeaf(g, 1) == second);
    REQUIRE(!first->hidden());
    REQUIRE(second->hidden());
}

TEST_CASE("AnnotationsRootItem takes the radio state from the view only", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, true);
    auto* root = model.annotationsRootItem();

    root->updateFrom({ annotationGroup("", { "A", "B" }) }, 0, 0, nullptr);

    // kept leaves change their radio state
    root->updateFrom({ annotationGroup("", { "A", "B" }) }, 0, 1, nullptr);

    REQUIRE(annotationLeaf(root, 0)->hidden());
    REQUIRE(!annotationLeaf(root, 1)->hidden());

    // B vanishes and returns while A is current => B comes back unchecked
    root->updateFrom({ annotationGroup("", { "A" }) }, 0, 0, nullptr);
    root->updateFrom({ annotationGroup("", { "A", "B" }) }, 0, 0, nullptr);

    REQUIRE(!annotationLeaf(root, 0)->hidden());
    REQUIRE(annotationLeaf(root, 1)->hidden());
}

TEST_CASE("view_layer_scan splits layer ids", "[view][layerpanel]")
{
    SECTION("well-formed id")
    {
        auto id = view_layer_scan::parseLayerId("Radar:DS 1:L2:CAT048");

        REQUIRE(id);
        REQUIRE(id->ds_type == "Radar");
        REQUIRE(id->ds_name == "DS 1");
        REQUIRE(id->line == "L2");
        REQUIRE(id->line_index == 1);
        REQUIRE(id->dbcontent == "CAT048");
    }

    SECTION("malformed id")
    {
        REQUIRE(!view_layer_scan::parseLayerId("Selected"));
        REQUIRE(!view_layer_scan::parseLayerId("Radar:DS:x:L1:CAT048"));

        auto id = view_layer_scan::layerIdOrUnknown("Selected");

        REQUIRE(id.ds_type == "<unknown>");
        REQUIRE(id.ds_name == "Selected");
        REQUIRE(id.line == "L1");
        REQUIRE(id.dbcontent == "<unknown>");
    }

    SECTION("line index")
    {
        REQUIRE(view_layer_scan::lineIndexFrom("L1") == 0);
        REQUIRE(view_layer_scan::lineIndexFrom("L4") == 3);
        REQUIRE(view_layer_scan::lineIndexFrom("x") == 0);
        REQUIRE(view_layer_scan::lineIndexFrom("L0") == 0);
    }
}

TEST_CASE("LayerPanelWidget animates the wait icon only while items are stale", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, false);
    model.setDataComplete(true);

    TrackingPayload a("a");
    model.updateDataSubtree(entriesOf({ &a }));

    LayerPanelWidget panel(nullptr, nullptr, &model, true);

    REQUIRE(!panel.isSpinnerActive());

    model.beginDataReload();
    REQUIRE(panel.isSpinnerActive());

    // the indication is optional
    panel.setStaleIndication(LayerTreeItemDelegate::StaleGreyed);
    REQUIRE(!panel.isSpinnerActive());

    panel.setStaleIndication(LayerTreeItemDelegate::StaleGreyed | LayerTreeItemDelegate::StaleSpinner);
    REQUIRE(panel.isSpinnerActive());

    TrackingPayload a2("a");
    model.updateDataSubtree(entriesOf({ &a2 }));

    REQUIRE(!panel.isSpinnerActive());
}
