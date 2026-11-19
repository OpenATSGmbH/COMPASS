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
#include "toolbox.h"
#include "toolboxwidget.h"
#include "viewdeck.h"

#include <QAbstractButton>
#include <QCoreApplication>
#include <QHBoxLayout>
#include <QTest>

namespace
{

/**
 * The content of the main window: Flight Deck with two tools, View Deck with two views.
 */
struct MainContentFixture
{
    MainContentFixture()
    {
        tool_box  = new ToolBox;
        view_deck = new ViewDeck;

        tool_a = new QWidget;
        tool_b = new QWidget;

        tool_box->addTool(new WrappedToolBoxWidget(tool_a, "Tool A", "Info A", { "Tool", "A" }, QIcon(), false, 30));
        tool_box->addTool(new WrappedToolBoxWidget(tool_b, "Tool B", "Info B", { "Tool B" }, QIcon(), false, 30));

        tool_box->setViewDeck(view_deck);

        // as the main window: both decks get the common button size
        const QSize deck_button_size = tool_box->buttonSize().expandedTo(view_deck->buttonSize());

        tool_box->setButtonSize(deck_button_size);
        view_deck->setButtonSize(deck_button_size);

        tool_box->selectTool("Tool A");

        auto layout = new QHBoxLayout(&window);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
        layout->addWidget(tool_box);
        layout->addWidget(view_deck);

        table = new QWidget;
        histo = new QWidget;

        view_deck->addView(table, "TableView0", "Table View 0", "Table 0", QIcon(), "");
        view_deck->addView(histo, "HistogramView0", "Histogram View 0", "Histo 0", QIcon(), "");

        window.resize(1600, 900);
        window.show();
        QCoreApplication::processEvents();
    }

    QAbstractButton* toolButton(const QString& label) const
    {
        for (auto b : tool_box->findChildren<QAbstractButton*>())
            if (b->text() == label)
                return b;

        return nullptr;
    }

    QAbstractButton* viewButton(const QString& name) const
    {
        return view_deck->findChild<QAbstractButton*>(name);
    }

    void press(QAbstractButton* button) const
    {
        REQUIRE(button);
        QTest::mouseClick(button, Qt::LeftButton);
        QCoreApplication::processEvents();
    }

    bool toolShown(QWidget* tool) const { return tool->isVisible(); }
    bool viewShown(QWidget* view) const { return view->isVisible(); }

    bool flightDeckOpen() const { return window.findChild<QWidget*>("toolbox_panel")->isVisible(); }
    bool viewDeckOpen() const { return window.findChild<QWidget*>("toolbox_views")->isVisible(); }

    QWidget   window;
    ToolBox*  tool_box  = nullptr;
    ViewDeck* view_deck = nullptr;
    QWidget*  tool_a    = nullptr;
    QWidget*  tool_b    = nullptr;
    QWidget*  table     = nullptr;
    QWidget*  histo     = nullptr;
};

}

TEST_CASE("Decks: both decks are open at the start", "[toolbox_decks]")
{
    MainContentFixture f;

    REQUIRE(f.toolShown(f.tool_a));
    REQUIRE(f.viewShown(f.table));
    REQUIRE(f.toolButton("Tool\nA")->isChecked());
    REQUIRE(f.viewButton("view_deck_tableview0")->isChecked());

    SECTION("the View Deck icon bar is at the right window edge, the Flight Deck buttons at the left edge")
    {
        REQUIRE(f.view_deck->geometry().right() == f.window.rect().right());
        REQUIRE(f.toolButton("Tool\nA")->mapTo(&f.window, QPoint(0, 0)).x() == 0);
    }
    SECTION("both decks have the same button size")
    {
        REQUIRE(f.toolButton("Tool\nA")->size() == f.viewButton("view_deck_tableview0")->size());
    }
    SECTION("a view added later keeps the button size of both decks")
    {
        const QSize size = f.toolButton("Tool\nA")->size();

        f.view_deck->addView(new QWidget, "ScatterPlotView12", "Scatterplot View 12", "Scatter 12", QIcon(), "");
        QCoreApplication::processEvents();

        REQUIRE(f.viewButton("view_deck_scatterplotview12")->size() == size);
        REQUIRE(f.toolButton("Tool\nA")->size() == size);
    }
}

TEST_CASE("Decks: a click on the selected view collapses the View Deck", "[toolbox_decks]")
{
    MainContentFixture f;

    const int panel_width_before = f.window.findChild<QWidget*>("toolbox_panel")->width();

    f.press(f.viewButton("view_deck_tableview0"));

    REQUIRE(!f.viewDeckOpen());
    REQUIRE(!f.viewShown(f.table));
    REQUIRE(f.toolShown(f.tool_a));
    REQUIRE(!f.viewButton("view_deck_tableview0")->isChecked());

    SECTION("the icon bar stays, the Flight Deck takes the width of the view area")
    {
        REQUIRE(f.view_deck->isVisible());
        REQUIRE(f.view_deck->geometry().right() == f.window.rect().right());

        const QWidget* panel = f.window.findChild<QWidget*>("toolbox_panel");

        REQUIRE(panel->width() > panel_width_before);
        REQUIRE(panel->mapTo(&f.window, QPoint(panel->width(), 0)).x() == f.view_deck->geometry().left());
    }
    SECTION("a click on a view opens it again with that view")
    {
        f.press(f.viewButton("view_deck_histogramview0"));

        REQUIRE(f.viewShown(f.histo));
        REQUIRE(f.toolShown(f.tool_a));
        REQUIRE(f.viewButton("view_deck_histogramview0")->isChecked());
        REQUIRE(f.window.findChild<QWidget*>("toolbox_panel")->width() == panel_width_before);
    }
    SECTION("another tool keeps the View Deck collapsed")
    {
        f.press(f.toolButton("Tool B"));

        REQUIRE(f.toolShown(f.tool_b));
        REQUIRE(!f.viewDeckOpen());
    }
    SECTION("showing a view from the application opens it again")
    {
        f.view_deck->showView(f.histo);

        REQUIRE(f.viewShown(f.histo));
        REQUIRE(f.toolShown(f.tool_a));
    }
}

TEST_CASE("Decks: a click on the selected tool collapses the Flight Deck", "[toolbox_decks]")
{
    MainContentFixture f;

    f.press(f.toolButton("Tool\nA"));

    REQUIRE(!f.flightDeckOpen());
    REQUIRE(f.viewShown(f.table));
    REQUIRE(!f.toolButton("Tool\nA")->isChecked());
    REQUIRE(f.viewButton("view_deck_tableview0")->isChecked());
}

TEST_CASE("Decks: the decks are never collapsed at the same time", "[toolbox_decks]")
{
    MainContentFixture f;

    SECTION("collapsing the View Deck opens the collapsed Flight Deck with its last tool")
    {
        f.press(f.toolButton("Tool B"));
        f.press(f.toolButton("Tool B"));

        REQUIRE(!f.flightDeckOpen());

        f.press(f.viewButton("view_deck_tableview0"));

        REQUIRE(!f.viewDeckOpen());
        REQUIRE(f.toolShown(f.tool_b));
        REQUIRE(f.toolButton("Tool B")->isChecked());
        REQUIRE(!f.toolButton("Tool\nA")->isChecked());
    }
    SECTION("collapsing the Flight Deck opens the collapsed View Deck with its last view")
    {
        f.press(f.viewButton("view_deck_histogramview0"));
        f.press(f.viewButton("view_deck_histogramview0"));

        REQUIRE(!f.viewDeckOpen());

        f.press(f.toolButton("Tool\nA"));

        REQUIRE(!f.flightDeckOpen());
        REQUIRE(f.viewShown(f.histo));
        REQUIRE(f.viewButton("view_deck_histogramview0")->isChecked());
    }
    SECTION("any sequence of clicks leaves one deck open")
    {
        const std::vector<QAbstractButton*> buttons = { f.toolButton("Tool\nA"),
                                                        f.toolButton("Tool B"),
                                                        f.viewButton("view_deck_tableview0"),
                                                        f.viewButton("view_deck_histogramview0") };

        for (int sequence = 0; sequence < 4 * 4 * 4 * 4; ++sequence)
        {
            for (int step = 0, code = sequence; step < 4; ++step, code /= 4)
            {
                f.press(buttons.at(code % 4));

                REQUIRE((f.flightDeckOpen() || f.viewDeckOpen()));
                REQUIRE(f.flightDeckOpen() == (f.toolShown(f.tool_a) || f.toolShown(f.tool_b)));
                REQUIRE(f.viewDeckOpen() == (f.viewShown(f.table) || f.viewShown(f.histo)));
            }
        }
    }
}

TEST_CASE("Decks: selecting a tool by name keeps it open", "[toolbox_decks]")
{
    MainContentFixture f;

    REQUIRE(f.tool_box->uiSet("Tool A"));
    REQUIRE(f.toolShown(f.tool_a));

    REQUIRE(f.tool_box->uiSet("Tool B"));
    REQUIRE(f.toolShown(f.tool_b));
    REQUIRE(f.toolButton("Tool B")->isChecked());
    REQUIRE(!f.toolButton("Tool\nA")->isChecked());

    REQUIRE(!f.tool_box->uiSet("No Such Tool"));
}
