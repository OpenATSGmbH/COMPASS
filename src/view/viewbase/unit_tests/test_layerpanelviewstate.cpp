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
#include "layertreemodel.h"
#include "viewlayerpanelwidget.h"
#include "viewlayertreemodel.h"
#include "dbcontentlayer.h"
#include "annotationsrootitem.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <QCoreApplication>
#include <QItemSelectionModel>
#include <QTreeView>

namespace
{
    typedef std::vector<std::pair<std::string, std::vector<std::string>>> Layout; // group name -> leaf names

    const Layout DefaultLayout = { { "A", { "a1", "a2" } },
                                   { "B", { "b1" } },
                                   { "B", { "b2" } } }; // repeated group name

    /**
     * Fresh detached group for the given layout entry.
     */
    std::unique_ptr<LayerTreeItem> buildGroup(const std::string& name, const std::vector<std::string>& leaves)
    {
        auto group = std::make_unique<LayerTreeItem>(name);
        for (const auto& leaf : leaves)
            group->appendChild(std::make_unique<LayerTreeItem>(leaf));

        return group;
    }

    /**
     * Model with a single top-level item "Top" holding the given layout, viewed by a panel.
     */
    struct Fixture
    {
        Fixture()
        {
            top = model.addRootItem(std::make_unique<LayerTreeItem>("Top"));
            rebuild(DefaultLayout);

            panel.reset(new LayerPanelWidget(nullptr, nullptr, &model, true));
        }

        /// removes all groups, inserts fresh ones
        void rebuild(const Layout& layout)
        {
            if (top->childCount() > 0)
                model.removeChildItems(top, 0, top->childCount() - 1);

            for (const auto& [ name, leaves ] : layout)
                model.appendChildItem(top, buildGroup(name, leaves));
        }

        void resetRebuild(const Layout& layout)
        {
            model.beginResetModel();

            top->clearChildren();
            for (const auto& [ name, leaves ] : layout)
                top->appendChild(buildGroup(name, leaves));

            model.endResetModel();
        }

        QModelIndex index(const std::vector<int>& rows) const
        {
            QModelIndex idx;
            for (int row : rows)
                idx = model.index(row, 0, idx);

            return idx;
        }

        QTreeView*           view() const      { return panel->treeView(); }
        QItemSelectionModel* selection() const { return panel->treeView()->selectionModel(); }

        LayerTreeModel                    model;
        LayerTreeItem*                    top = nullptr;
        std::unique_ptr<LayerPanelWidget> panel;
    };

    const std::vector<int> TopRows = { 0 };
    const std::vector<int> ARows   = { 0, 0 };
    const std::vector<int> A2Rows  = { 0, 0, 1 };
    const std::vector<int> B1Rows  = { 0, 1 };
    const std::vector<int> B2Rows  = { 0, 2 };
    const std::vector<int> CRows   = { 0, 3 };
}

TEST_CASE("LayerPanelWidget gives replaced rows the default expansion", "[view][layerpanel]")
{
    Fixture f;

    f.view()->expand(f.index(TopRows));
    f.view()->expand(f.index(ARows));

    SECTION("after a removal and a new insert")
    {
        f.rebuild(DefaultLayout);

        REQUIRE(f.view()->isExpanded(f.index(TopRows))); // kept
        REQUIRE(!f.view()->isExpanded(f.index(ARows)));
    }

    SECTION("after a model reset")
    {
        f.resetRebuild(DefaultLayout);

        REQUIRE(!f.view()->isExpanded(f.index(TopRows)));
        REQUIRE(!f.view()->isExpanded(f.index(ARows)));
    }
}

TEST_CASE("LayerPanelWidget applies the default expansion to new rows only", "[view][layerpanel]")
{
    Fixture f;

    f.panel->setDefaultExpansion([] (const LayerTreeItem&, int) { return true; });

    REQUIRE(f.view()->isExpanded(f.index(ARows)));
    REQUIRE(f.view()->isExpanded(f.index(B1Rows)));

    f.view()->collapse(f.index(ARows));

    f.model.appendChildItem(f.top, buildGroup("C", { "c1" }));

    REQUIRE(!f.view()->isExpanded(f.index(ARows))); // kept, collapsed by the user
    REQUIRE(f.view()->isExpanded(f.index(B1Rows)));
    REQUIRE(f.view()->isExpanded(f.index(CRows)));  // new => default
}

TEST_CASE("LayerPanelWidget keeps the selection of kept rows", "[view][layerpanel]")
{
    Fixture f;

    f.selection()->setCurrentIndex(f.index(A2Rows), QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

    f.model.appendChildItem(f.top, buildGroup("C", { "c1" }));

    REQUIRE(f.selection()->isSelected(f.index(A2Rows)));
    REQUIRE(f.view()->currentIndex() == f.index(A2Rows));

    f.rebuild(DefaultLayout);

    REQUIRE(!f.selection()->isSelected(f.index(A2Rows))); // replaced => forgotten
}

TEST_CASE("LayerPanelWidget keeps user expansion when the default policy changes", "[view][layerpanel]")
{
    Fixture f;

    f.panel->setDefaultExpansion([] (const LayerTreeItem&, int depth) { return depth == 0; });

    REQUIRE(f.view()->isExpanded(f.index(TopRows)));

    f.view()->expand(f.index(ARows)); // against the default

    f.panel->setDefaultExpansion([] (const LayerTreeItem&, int depth) { return depth == 1; });

    REQUIRE(!f.view()->isExpanded(f.index(TopRows))); // followed the default
    REQUIRE(f.view()->isExpanded(f.index(ARows)));    // user deviation
    REQUIRE(f.view()->isExpanded(f.index(B1Rows)));   // followed the default
}

namespace
{
    /**
     * View panel with a DBContent subtree T1 > S1 > L1 (DSType > data source > line).
     */
    struct DBContentFixture
    {
        DBContentFixture()
        {
            rebuild();
        }

        /// replaces the subtree below the DBContent root
        void rebuild()
        {
            auto* model = panel.model();
            auto* root  = model->dbContentRootItem();

            if (root->childCount() > 0)
                model->removeChildItems(root, 0, root->childCount() - 1);

            auto line = std::make_unique<LayerTreeItem>("L1");
            auto ds   = std::make_unique<LayerTreeItem>("S1");
            auto type = std::make_unique<LayerTreeItem>("T1");

            ds->appendChild(std::move(line));
            type->appendChild(std::move(ds));

            model->appendChildItem(root, std::move(type));
        }

        bool expanded(const std::vector<int>& rows) const
        {
            QModelIndex idx;
            for (int row : rows)
                idx = panel.model()->index(row, 0, idx);

            return panel.treeView()->isExpanded(idx);
        }

        void setExpanded(const std::vector<int>& rows, bool value)
        {
            QModelIndex idx;
            for (int row : rows)
                idx = panel.model()->index(row, 0, idx);

            panel.treeView()->setExpanded(idx, value);
        }

        ViewLayerPanelWidget panel { {}, false };
    };

    const std::vector<int> RootRows = { 0 };
    const std::vector<int> T1Rows   = { 0, 0 };
    const std::vector<int> S1Rows   = { 0, 0, 0 };
}

TEST_CASE("ViewLayerPanelWidget expands the DBContent subtree to the color mode depth", "[view][layerpanel]")
{
    DBContentFixture f;

    f.panel.setDataExpansion(ViewLayerPanelWidget::dataExpansionDepth(context::ColorProvider::Mode::DataSource)); // => 1

    REQUIRE(f.expanded(RootRows));
    REQUIRE(f.expanded(T1Rows));
    REQUIRE(!f.expanded(S1Rows));

    SECTION("replaced rows get the default")
    {
        f.setExpanded(T1Rows, false);

        f.rebuild();

        REQUIRE(f.expanded(T1Rows));
    }

    SECTION("user changes")
    {
        f.setExpanded(T1Rows, false);
        f.setExpanded(S1Rows, true);

        SECTION("the same depth again keeps them")
        {
            f.panel.setDataExpansion(1);

            REQUIRE(!f.expanded(T1Rows));
            REQUIRE(f.expanded(S1Rows));
        }

        SECTION("a new depth resets them to its default")
        {
            f.panel.setDataExpansion(0);

            REQUIRE(f.expanded(RootRows));
            REQUIRE(!f.expanded(T1Rows));
            REQUIRE(!f.expanded(S1Rows));

            f.panel.setDataExpansion(2);

            REQUIRE(f.expanded(T1Rows));
            REQUIRE(f.expanded(S1Rows));
        }
    }
}

TEST_CASE("ViewLayerPanelWidget applies the default expansion outside the Data subtree only", "[view][layerpanel]")
{
    DBContentFixture f;

    auto other = std::make_unique<LayerTreeItem>("Other");
    other->appendChild(std::make_unique<LayerTreeItem>("o1"));
    f.panel.model()->addRootItem(std::move(other));

    f.panel.setDataExpansion(1);
    f.panel.setDefaultExpansion([] (const LayerTreeItem&, int) { return true; });

    REQUIRE(f.expanded({ 1 }));     // Other
    REQUIRE(f.expanded(T1Rows));    // Data: color mode depth
    REQUIRE(!f.expanded(S1Rows));
}

TEST_CASE("LayerPanelWidget hands its view state over to the next panel", "[view][layerpanel]")
{
    Fixture f;

    f.view()->expand(f.index(TopRows));
    f.view()->expand(f.index(B1Rows));
    f.selection()->setCurrentIndex(f.index(A2Rows), QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

    auto checkTakenOver = [ & ] ()
    {
        REQUIRE(f.view()->isExpanded(f.index(TopRows)));
        REQUIRE(f.view()->isExpanded(f.index(B1Rows)));
        REQUIRE(!f.view()->isExpanded(f.index(ARows)));
        REQUIRE(f.selection()->isSelected(f.index(A2Rows)));
        REQUIRE(f.view()->currentIndex() == f.index(A2Rows));
    };

    SECTION("created after the predecessor is gone")
    {
        f.panel.reset();
        f.panel.reset(new LayerPanelWidget(nullptr, nullptr, &f.model, true));

        checkTakenOver();
    }

    SECTION("created before the predecessor is gone (e.g. config widget rebuild)")
    {
        f.panel.reset(new LayerPanelWidget(nullptr, nullptr, &f.model, true));

        checkTakenOver();
        REQUIRE(!f.top->viewHandover()); // taken over => cleared
    }

    SECTION("the handover is cleared after the construction")
    {
        f.panel.reset();
        f.panel.reset(new LayerPanelWidget(nullptr, nullptr, &f.model, true));

        REQUIRE(f.top->viewHandover());

        QCoreApplication::sendPostedEvents();
        QCoreApplication::processEvents();

        REQUIRE(!f.top->viewHandover());
    }
}

TEST_CASE("ViewLayerPanelWidget keeps the handed over expansion against its policy", "[view][layerpanel]")
{
    ViewLayerTreeModel model({}, true);

    // the annotations root expands by default
    model.appendChildItem(model.annotationsRootItem(), std::make_unique<LayerTreeItem>("a1"));

    const QModelIndex anno_index = model.index(model.annotationsRootItem()->row(), 0);

    auto panel = std::make_unique<ViewLayerPanelWidget>(std::vector<LayerColumnSpec>(), true, nullptr, nullptr, &model);
    REQUIRE(panel->treeView()->isExpanded(anno_index));

    panel->treeView()->collapse(anno_index);

    panel.reset();
    panel = std::make_unique<ViewLayerPanelWidget>(std::vector<LayerColumnSpec>(), true, nullptr, nullptr, &model);

    REQUIRE(!panel->treeView()->isExpanded(anno_index));
}
