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
#include "viewrenderwidget.h"
#include "viewtoolswitcher.h"
#include "viewtoolwidget.h"

#include <QAction>
#include <QImage>
#include <QPixmap>

#include <vector>

namespace
{
    /**
     * ViewRenderWidget is abstract, this is the smallest concrete one.
     */
    class TestRenderWidget : public ViewRenderWidget
    {
    public:
        void redraw() override {}
        void clear() override {}
        bool render(QImage&, const QSize&, double) override { return false; }
    };

    QIcon someIcon()
    {
        QPixmap pixmap(16, 16);
        pixmap.fill(Qt::red);

        return QIcon(pixmap);
    }

    ViewTool makeTool(int id, const QString& name, ViewTool::Kind kind, std::vector<bool>& calls)
    {
        ViewTool tool;
        tool.id       = id;
        tool.name     = name;
        tool.icon     = someIcon();
        tool.kind     = kind;
        tool.callback = [ &calls ] (bool on) { calls.push_back(on); };

        return tool;
    }

    QAction* toolbarAction(const ViewToolWidget& toolbar, const QString& text)
    {
        for (auto action : toolbar.actions())
            if (action->text() == text)
                return action;

        return nullptr;
    }

    enum ToolId
    {
        Navigate = 0,
        Select,
        Zoom,
        Home,
        Panel
    };
}

TEST_CASE("a trigger tool runs its callback once per click", "[view][viewtool]")
{
    TestRenderWidget render_widget;
    ViewToolWidget   toolbar(&render_widget.toolSwitcher());

    std::vector<bool> calls;
    render_widget.registerTool(makeTool(Home, "Zoom to Home", ViewTool::Kind::Trigger, calls));

    auto action = toolbarAction(toolbar, "Zoom to Home");
    REQUIRE(action);
    CHECK(!action->isCheckable());

    CHECK(calls.empty());

    action->trigger();
    CHECK(calls == std::vector<bool>{ true });

    action->trigger();
    CHECK(calls == std::vector<bool>{ true, true });
}

TEST_CASE("a trigger tool shows its key in the toolbar", "[view][viewtool]")
{
    TestRenderWidget render_widget;
    ViewToolWidget   toolbar(&render_widget.toolSwitcher());

    std::vector<bool> calls;
    auto tool = makeTool(Home, "Zoom to Home", ViewTool::Kind::Trigger, calls);
    tool.key = Qt::Key_Space;

    render_widget.registerTool(tool);

    auto action = toolbarAction(toolbar, "Zoom to Home [Space]");
    REQUIRE(action);
    CHECK(action->shortcut() == QKeySequence(Qt::Key_Space));
}

TEST_CASE("a toggle tool reports on and off", "[view][viewtool]")
{
    TestRenderWidget render_widget;
    ViewToolWidget   toolbar(&render_widget.toolSwitcher());

    std::vector<bool> calls;
    render_widget.registerTool(makeTool(Panel, "Toggle Panel", ViewTool::Kind::Toggle, calls));

    auto action = toolbarAction(toolbar, "Toggle Panel");
    REQUIRE(action);
    CHECK(action->isCheckable());
    CHECK(!action->isChecked());

    action->trigger();
    CHECK(action->isChecked());
    CHECK(calls == std::vector<bool>{ true });

    action->trigger();
    CHECK(!action->isChecked());
    CHECK(calls == std::vector<bool>{ true, false });
}

TEST_CASE("a toggle tool does not change the mode tool", "[view][viewtool]")
{
    TestRenderWidget render_widget;
    ViewToolWidget   toolbar(&render_widget.toolSwitcher());

    std::vector<bool> navigate_calls, panel_calls;
    render_widget.registerTool(makeTool(Navigate, "Navigate", ViewTool::Kind::Mode, navigate_calls));
    render_widget.registerTool(makeTool(Panel, "Toggle Panel", ViewTool::Kind::Toggle, panel_calls));

    toolbarAction(toolbar, "Toggle Panel")->trigger();

    CHECK(render_widget.toolSwitcher().currentTool() == Navigate);
    CHECK(navigate_calls == std::vector<bool>{ true });
}

TEST_CASE("the first mode tool is the default tool and is active", "[view][viewtool]")
{
    TestRenderWidget render_widget;

    std::vector<bool> navigate_calls, select_calls;
    render_widget.registerTool(makeTool(Navigate, "Navigate", ViewTool::Kind::Mode, navigate_calls));
    render_widget.registerTool(makeTool(Select, "Select", ViewTool::Kind::Mode, select_calls));

    CHECK(render_widget.toolSwitcher().currentTool() == Navigate);
    CHECK(navigate_calls == std::vector<bool>{ true });
    CHECK(select_calls.empty());
}

TEST_CASE("mode tools are exclusive and report on and off", "[view][viewtool]")
{
    TestRenderWidget render_widget;
    ViewToolWidget   toolbar(&render_widget.toolSwitcher());

    std::vector<bool> navigate_calls, select_calls, zoom_calls;
    render_widget.registerTool(makeTool(Navigate, "Navigate", ViewTool::Kind::Mode, navigate_calls));
    render_widget.registerTool(makeTool(Select, "Select", ViewTool::Kind::Mode, select_calls));
    render_widget.registerTool(makeTool(Zoom, "Zoom", ViewTool::Kind::Mode, zoom_calls));

    auto select_action = toolbarAction(toolbar, "Select");
    auto zoom_action   = toolbarAction(toolbar, "Zoom");
    REQUIRE(select_action);
    REQUIRE(zoom_action);

    CHECK(select_action->isCheckable());
    CHECK(!select_action->isChecked());
    CHECK(!zoom_action->isChecked());

    SECTION("a click activates the tool and leaves the default tool")
    {
        select_action->trigger();

        CHECK(render_widget.toolSwitcher().currentTool() == Select);
        CHECK(select_action->isChecked());
        CHECK(!zoom_action->isChecked());
        CHECK(navigate_calls == std::vector<bool>{ true, false });
        CHECK(select_calls == std::vector<bool>{ true });
        CHECK(zoom_calls.empty());
    }

    SECTION("another mode tool replaces the active one")
    {
        select_action->trigger();
        zoom_action->trigger();

        CHECK(render_widget.toolSwitcher().currentTool() == Zoom);
        CHECK(!select_action->isChecked());
        CHECK(zoom_action->isChecked());
        CHECK(select_calls == std::vector<bool>{ true, false });
        CHECK(zoom_calls == std::vector<bool>{ true });
    }

    SECTION("a second click on the active tool gives the default tool")
    {
        select_action->trigger();
        select_action->trigger();

        CHECK(render_widget.toolSwitcher().currentTool() == Navigate);
        CHECK(!select_action->isChecked());
        CHECK(select_calls == std::vector<bool>{ true, false });
        CHECK(navigate_calls == std::vector<bool>{ true, false, true });
    }

    SECTION("the end of a tool gives the default tool")
    {
        zoom_action->trigger();
        render_widget.toolSwitcher().endCurrentTool();

        CHECK(render_widget.toolSwitcher().currentTool() == Navigate);
        CHECK(!zoom_action->isChecked());
        CHECK(zoom_calls == std::vector<bool>{ true, false });
        CHECK(navigate_calls == std::vector<bool>{ true, false, true });
    }
}

TEST_CASE("the update callback sets the enabled state", "[view][viewtool]")
{
    TestRenderWidget render_widget;
    ViewToolWidget   toolbar(&render_widget.toolSwitcher());

    bool enabled = false;

    std::vector<bool> calls;

    for (auto kind : { ViewTool::Kind::Trigger, ViewTool::Kind::Toggle, ViewTool::Kind::Mode })
    {
        auto tool = makeTool((int)kind, QString("Tool %1").arg((int)kind), kind, calls);
        tool.update = [ &enabled ] (QAction* action) { action->setEnabled(enabled); };

        render_widget.registerTool(tool);
    }

    toolbar.updateItems();

    for (auto kind : { ViewTool::Kind::Trigger, ViewTool::Kind::Toggle, ViewTool::Kind::Mode })
    {
        auto action = toolbarAction(toolbar, QString("Tool %1").arg((int)kind));
        REQUIRE(action);
        CHECK(!action->isEnabled());
    }

    enabled = true;
    toolbar.updateItems();

    for (auto kind : { ViewTool::Kind::Trigger, ViewTool::Kind::Toggle, ViewTool::Kind::Mode })
        CHECK(toolbarAction(toolbar, QString("Tool %1").arg((int)kind))->isEnabled());
}

TEST_CASE("the toolbar shows the tools with an icon in the order of registration", "[view][viewtool]")
{
    TestRenderWidget render_widget;

    std::vector<bool> calls;

    //registered before the toolbar exists, as a render widget does it
    auto navigate = makeTool(Navigate, "Navigate", ViewTool::Kind::Mode, calls);
    navigate.icon = QIcon();

    render_widget.registerTool(navigate);
    render_widget.registerTool(makeTool(Select, "Select", ViewTool::Kind::Mode, calls));

    ViewToolWidget toolbar(&render_widget.toolSwitcher());

    render_widget.registerTool(makeTool(Home, "Zoom to Home", ViewTool::Kind::Trigger, calls));

    QStringList names;
    for (auto action : toolbar.actions())
        names << action->text();

    CHECK(names == QStringList{ "Select", "Zoom to Home" });
}

TEST_CASE("a tool id can be registered only once", "[view][viewtool]")
{
    TestRenderWidget render_widget;

    std::vector<bool> calls;
    render_widget.registerTool(makeTool(Home, "Zoom to Home", ViewTool::Kind::Trigger, calls));

    CHECK_THROWS(render_widget.registerTool(makeTool(Home, "Other", ViewTool::Kind::Toggle, calls)));
}
