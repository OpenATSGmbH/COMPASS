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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <QItemSelectionModel>
#include <QTreeView>

namespace
{
    typedef std::vector<std::pair<std::string, std::vector<std::string>>> Layout; // group name -> leaf names

    const Layout DefaultLayout = { { "A", { "a1", "a2" } },
                                   { "B", { "b1" } },
                                   { "B", { "b2" } } }; // repeated group name

    /**
     * Fresh detached groups for the given layout.
     */
    std::vector<std::unique_ptr<LayerTreeItem>> buildChildren(const Layout& layout)
    {
        std::vector<std::unique_ptr<LayerTreeItem>> children;
        children.reserve(layout.size());

        for (const auto& [ name, leaves ] : layout)
        {
            auto group = std::make_unique<LayerTreeItem>(name);
            for (const auto& leaf : leaves)
                group->appendChild(std::make_unique<LayerTreeItem>(leaf));

            children.push_back(std::move(group));
        }

        return children;
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

        void rebuild(const Layout& layout)
        {
            model.refreshSubtree(top, [ & ] () { return buildChildren(layout); }, false);
        }

        void resetRebuild(const Layout& layout)
        {
            model.beginResetModel();

            top->clearChildren();
            for (auto& child : buildChildren(layout))
                top->appendChild(std::move(child));

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

TEST_CASE("LayerPanelWidget restores expansion after a subtree rebuild", "[view][layerpanel]")
{
    Fixture f;

    f.view()->expand(f.index(TopRows));
    f.view()->expand(f.index(ARows));

    f.rebuild(DefaultLayout);

    REQUIRE(f.view()->isExpanded(f.index(TopRows)));
    REQUIRE(f.view()->isExpanded(f.index(ARows)));
    REQUIRE(!f.view()->isExpanded(f.index(B1Rows)));
    REQUIRE(!f.view()->isExpanded(f.index(B2Rows)));
}

TEST_CASE("LayerPanelWidget restores expansion after a model reset", "[view][layerpanel]")
{
    Fixture f;

    f.view()->expand(f.index(TopRows));
    f.view()->expand(f.index(B2Rows)); // second of the repeated "B" groups

    f.resetRebuild(DefaultLayout);

    REQUIRE(f.view()->isExpanded(f.index(TopRows)));
    REQUIRE(f.view()->isExpanded(f.index(B2Rows)));
    REQUIRE(!f.view()->isExpanded(f.index(B1Rows)));
    REQUIRE(!f.view()->isExpanded(f.index(ARows)));
}

TEST_CASE("LayerPanelWidget applies the default expansion to unknown items only", "[view][layerpanel]")
{
    Fixture f;

    f.panel->setDefaultExpansion([] (const LayerTreeItem&, int) { return true; });

    REQUIRE(f.view()->isExpanded(f.index(ARows)));
    REQUIRE(f.view()->isExpanded(f.index(B1Rows)));

    f.view()->collapse(f.index(ARows));

    Layout layout = DefaultLayout;
    layout.push_back({ "C", { "c1" } });

    f.rebuild(layout);

    REQUIRE(!f.view()->isExpanded(f.index(ARows))); // collapsed by the user
    REQUIRE(f.view()->isExpanded(f.index(B1Rows)));
    REQUIRE(f.view()->isExpanded(f.index(CRows)));  // new => default
}

TEST_CASE("LayerPanelWidget records only expansion deviating from the default", "[view][layerpanel]")
{
    Fixture f;

    f.panel->setDefaultExpansion([] (const LayerTreeItem&, int depth) { return depth == 0; });

    f.view()->collapse(f.index(TopRows));
    f.view()->expand(f.index(ARows));

    f.resetRebuild(DefaultLayout);

    REQUIRE(f.model.state().itemState("Top"));
    REQUIRE(!*f.model.state().itemState("Top")->expanded);  // collapsed against the default
    REQUIRE(f.model.state().itemState("Top/A"));
    REQUIRE(*f.model.state().itemState("Top/A")->expanded); // expanded against the default
    REQUIRE(!f.model.state().itemState("Top/B"));           // default
    REQUIRE(!f.model.state().itemState("Top/A/a1"));        // default

    REQUIRE(!f.view()->isExpanded(f.index(TopRows)));
    REQUIRE(f.view()->isExpanded(f.index(ARows)));
}

TEST_CASE("LayerPanelWidget restores selection and current item", "[view][layerpanel]")
{
    Fixture f;

    f.selection()->setCurrentIndex(f.index(A2Rows), QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

    SECTION("after a subtree rebuild")
    {
        f.rebuild(DefaultLayout);

        REQUIRE(f.selection()->isSelected(f.index(A2Rows)));
        REQUIRE(f.view()->currentIndex() == f.index(A2Rows));
    }

    SECTION("after a model reset")
    {
        f.resetRebuild(DefaultLayout);

        REQUIRE(f.selection()->isSelected(f.index(A2Rows)));
        REQUIRE(f.view()->currentIndex() == f.index(A2Rows));
    }

    SECTION("unless the user selected something else meanwhile")
    {
        f.rebuild({});

        f.selection()->setCurrentIndex(f.index(TopRows), QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

        f.rebuild(DefaultLayout);

        REQUIRE(!f.selection()->isSelected(f.index(A2Rows)));
        REQUIRE(f.selection()->isSelected(f.index(TopRows)));
        REQUIRE(f.view()->currentIndex() == f.index(TopRows));
    }

    SECTION("unless the database was closed meanwhile")
    {
        f.view()->expand(f.index(TopRows));
        f.view()->expand(f.index(ARows));

        f.rebuild({});
        f.model.state().clear();
        f.rebuild(DefaultLayout);

        REQUIRE(!f.selection()->isSelected(f.index(A2Rows)));
        REQUIRE(!f.view()->isExpanded(f.index(ARows)));
    }
}

TEST_CASE("LayerPanelWidget remembers only the enabled view state components", "[view][layerpanel]")
{
    Fixture f;

    f.view()->expand(f.index(TopRows));
    f.view()->expand(f.index(ARows));
    f.selection()->setCurrentIndex(f.index(A2Rows), QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);

    SECTION("without PersistExpansion")
    {
        f.model.state().setPersistFlags(LayerTreeState::PersistAll & ~LayerTreeState::PersistExpansion);

        f.rebuild(DefaultLayout);

        REQUIRE(!f.view()->isExpanded(f.index(ARows)));
        REQUIRE(f.selection()->isSelected(f.index(A2Rows)));
    }

    SECTION("without PersistSelection")
    {
        f.model.state().setPersistFlags(LayerTreeState::PersistAll & ~LayerTreeState::PersistSelection);

        f.rebuild(DefaultLayout);

        REQUIRE(f.view()->isExpanded(f.index(ARows)));
        REQUIRE(!f.selection()->isSelected(f.index(A2Rows)));
        REQUIRE(f.view()->currentIndex() != f.index(A2Rows));
    }
}

TEST_CASE("LayerPanelWidget keeps user expansion when the default policy changes", "[view][layerpanel]")
{
    Fixture f;

    f.view()->expand(f.index(TopRows));
    f.view()->expand(f.index(ARows));

    f.panel->setDefaultExpansion([] (const LayerTreeItem&, int) { return false; });

    REQUIRE(f.view()->isExpanded(f.index(TopRows)));
    REQUIRE(f.view()->isExpanded(f.index(ARows)));
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

        void rebuild()
        {
            auto* root = panel.model()->dbContentRootItem();

            panel.model()->refreshSubtree(root, [] ()
            {
                auto line = std::make_unique<LayerTreeItem>("L1");
                auto ds   = std::make_unique<LayerTreeItem>("S1");
                auto type = std::make_unique<LayerTreeItem>("T1");

                ds->appendChild(std::move(line));
                type->appendChild(std::move(ds));

                std::vector<std::unique_ptr<LayerTreeItem>> children;
                children.push_back(std::move(type));
                return children;
            }, false);
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

    SECTION("user changes survive rebuilds")
    {
        f.setExpanded(T1Rows, false);
        f.setExpanded(S1Rows, true);

        f.rebuild();

        REQUIRE(!f.expanded(T1Rows));
        REQUIRE(f.expanded(S1Rows));

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

    f.panel.reset(new LayerPanelWidget(nullptr, nullptr, &f.model, true)); // e.g. config widget rebuild

    REQUIRE(f.view()->isExpanded(f.index(TopRows)));
    REQUIRE(f.view()->isExpanded(f.index(B1Rows)));
    REQUIRE(!f.view()->isExpanded(f.index(ARows)));
}
