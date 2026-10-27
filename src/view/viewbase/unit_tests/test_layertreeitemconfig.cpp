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

#include "json.hpp"

#include <memory>
#include <string>

namespace
{
    /**
     * Subtree root persisting the hidden states of its subtree to the config.
     */
    class PersistedRoot : public LayerTreeItem
    {
    public:
        using LayerTreeItem::LayerTreeItem;

        bool persistsSubtreeToConfig() const override { return true; }
    };

    /**
     * Counts the hook calls.
     */
    class HookItem : public LayerTreeItem
    {
    public:
        using LayerTreeItem::LayerTreeItem;

        void onAttached() override { ++attached; }
        void onConfigChanged() override { ++config_changed; }

        int attached       = 0;
        int config_changed = 0;
    };

    /**
     * Fresh detached "Ctx" subtree with r1 and r2 (hidden by default).
     */
    std::unique_ptr<LayerTreeItem> makeContext()
    {
        auto ctx = std::make_unique<PersistedRoot>("Ctx");
        ctx->appendChild(std::make_unique<HookItem>("r1"));

        auto r2 = std::make_unique<HookItem>("r2");
        r2->setHidden(true);
        ctx->appendChild(std::move(r2));

        return ctx;
    }

    /**
     * Recreates the context items within a model reset (as the OSG roots do).
     */
    void resetContext(LayerTreeModel& model, LayerTreeItem* ctx)
    {
        model.beginResetModel();

        ctx->clearChildren();

        auto fresh = makeContext();
        for (auto& child : fresh->moveChildrenOut())
            ctx->appendChild(std::move(child));

        model.endResetModel();
    }

    HookItem* child(LayerTreeItem* parent, int row) { return static_cast<HookItem*>(parent->child(row)); }
}

TEST_CASE("LayerTreeItem inherits config persistence from the subtree root", "[view][layerpanel]")
{
    LayerTreeModel model;

    auto* ctx  = model.addRootItem(makeContext());
    auto* data = model.addRootItem(std::make_unique<LayerTreeItem>("Data"));

    REQUIRE(ctx->persistHiddenStateToConfig());
    REQUIRE(ctx->child(0)->persistHiddenStateToConfig());
    REQUIRE(!data->persistHiddenStateToConfig());
}

TEST_CASE("LayerTreeItem writes persisted hidden states to the config", "[view][layerpanel]")
{
    nlohmann::json config = nlohmann::json::object();

    LayerTreeModel model;
    model.bindConfig(&config);

    auto* ctx  = model.addRootItem(makeContext());
    auto* data = model.addRootItem(std::make_unique<LayerTreeItem>("Data"));

    ctx->child(0)->setHidden(true);
    data->setHidden(true);

    REQUIRE(config.contains("Ctx/r1"));
    REQUIRE(config.at("Ctx/r1").at("hidden") == true);
    REQUIRE(!config.contains("Data"));
}

TEST_CASE("LayerTreeItem reads the persisted hidden state on attach", "[view][layerpanel]")
{
    nlohmann::json config = { { "Ctx/r1", { { "hidden", true } } } };

    LayerTreeModel model;
    model.bindConfig(&config);

    auto* ctx = model.addRootItem(makeContext());

    REQUIRE(ctx->child(0)->hidden());
    REQUIRE(ctx->child(1)->hidden()); // default
    REQUIRE(!model.state().itemState("Ctx/r1")); // config value, not a session state

    SECTION("also after a database close")
    {
        ctx->child(1)->setHidden(false);

        model.state().clear();
        resetContext(model, ctx);

        REQUIRE(ctx->child(0)->hidden());
        REQUIRE(!ctx->child(1)->hidden());
    }

    SECTION("the session map wins over the config")
    {
        ctx->child(1)->setHidden(false);              // deviation => session state
        config[ "Ctx/r2" ][ "hidden" ] = true;        // stale config

        resetContext(model, ctx);

        REQUIRE(!ctx->child(1)->hidden());
    }
}

TEST_CASE("LayerTreeModel applies a replaced config to the items", "[view][layerpanel]")
{
    nlohmann::json config = nlohmann::json::object();

    LayerTreeModel model;
    model.bindConfig(&config);

    auto* ctx = model.addRootItem(makeContext());

    ctx->child(1)->setHidden(false);

    config = { { "Ctx/r2", { { "hidden", true } } } }; // e.g. preset applied
    model.configChanged();

    REQUIRE(ctx->child(1)->hidden()); // config wins here
    REQUIRE(child(ctx, 0)->config_changed == 1);
    REQUIRE(child(ctx, 1)->config_changed == 1);
}

TEST_CASE("LayerTreeItem is notified once on attach", "[view][layerpanel]")
{
    LayerTreeModel model;

    auto* ctx = model.addRootItem(makeContext());

    REQUIRE(child(ctx, 0)->attached == 1);
    REQUIRE(child(ctx, 1)->attached == 1);
}

TEST_CASE("LayerTreeItem creates config entries on write access only", "[view][layerpanel]")
{
    nlohmann::json config = nlohmann::json::object();

    LayerTreeModel model;
    model.bindConfig(&config);

    auto* ctx = model.addRootItem(makeContext());

    const LayerTreeItem& r1 = *ctx->child(0);
    REQUIRE(r1.config().empty());
    REQUIRE(!config.contains("Ctx/r1"));

    ctx->child(0)->config()[ "show_labels" ] = false;
    REQUIRE(config.at("Ctx/r1").at("show_labels") == false);

    SECTION("unbound models use a scratch config")
    {
        LayerTreeModel unbound;
        auto* item = unbound.addRootItem(std::make_unique<LayerTreeItem>("Item"));

        item->config()[ "x" ] = 1;
        REQUIRE(item->config().at("x") == 1);
    }
}
