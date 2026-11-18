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
#include "viewdeck.h"
#include "deckbutton.h"

#include <QAbstractButton>
#include <QCoreApplication>
#include <QHBoxLayout>
#include <QSignalSpy>
#include <QTest>

namespace
{

/**
 * A window with a View Deck and three views, laid out like a separate view window.
 */
struct DeckFixture
{
    DeckFixture()
    {
        deck = new ViewDeck;

        auto layout = new QHBoxLayout(&window);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
        layout->addWidget(deck->viewArea(), 1);
        layout->addWidget(deck);

        table = new QWidget;
        histo = new QWidget;
        geo   = new QWidget;

        deck->addView(table, "TableView0", "Table View 0", "Table 0", QIcon(), "");
        deck->addView(histo, "HistogramView0", "Histogram View 0", "Histo 0", QIcon(), "");
        deck->addView(geo, "GeographicView1", "Geographic View 1", "Geo 1", QIcon(), "");
    }

    QAbstractButton* button(const QString& name) const
    {
        return window.findChild<QAbstractButton*>(name);
    }

    void press(const QString& name) const
    {
        auto b = button(name);
        REQUIRE(b);
        QTest::mouseClick(b, Qt::LeftButton);
    }

    QWidget   window;
    ViewDeck* deck  = nullptr;
    QWidget*  table = nullptr;
    QWidget*  histo = nullptr;
    QWidget*  geo   = nullptr;
};

}

TEST_CASE("View Deck: each view has a button with the short name, the view name is the tooltip", "[view_deck]")
{
    DeckFixture f;

    auto table_button = f.button("view_deck_tableview0");
    auto geo_button   = f.button("view_deck_geographicview1");

    REQUIRE(table_button);
    REQUIRE(geo_button);
    REQUIRE(table_button->text() == "Table 0");
    REQUIRE(table_button->toolTip() == "Table View 0");
    REQUIRE(geo_button->text() == "Geo 1");

    REQUIRE(f.button("view_deck_tableview0_close"));
    REQUIRE(f.button("add_view"));
    REQUIRE(!f.button("add_view")->toolTip().isEmpty());
    REQUIRE(!f.button("view_deck_tableview0_close")->toolTip().isEmpty());
}

TEST_CASE("View Deck: the buttons keep the creation order of the views", "[view_deck]")
{
    DeckFixture f;

    f.window.resize(800, 600);
    f.window.show();
    QCoreApplication::processEvents();

    REQUIRE(f.button("view_deck_tableview0")->y() < f.button("view_deck_histogramview0")->y());
    REQUIRE(f.button("view_deck_histogramview0")->y() < f.button("view_deck_geographicview1")->y());

    SECTION("the add button is at the end of the icon bar")
    {
        const QWidget* last = f.button("view_deck_geographicview1");
        const QWidget* add  = f.button("add_view");

        REQUIRE(last->mapTo(f.deck, QPoint(0, 0)).y() < add->mapTo(f.deck, QPoint(0, 0)).y());
    }
    SECTION("the icon bar is at the right edge of the window")
    {
        REQUIRE(f.deck->geometry().right() == f.window.rect().right());
        REQUIRE(f.deck->viewArea()->geometry().right() < f.deck->geometry().left());
    }
}

TEST_CASE("View Deck: a press on a view button shows that view", "[view_deck]")
{
    DeckFixture f;

    SECTION("the first view is shown at the start")
    {
        REQUIRE(f.deck->currentView() == f.table);
        REQUIRE(f.button("view_deck_tableview0")->isChecked());
        REQUIRE(!f.button("view_deck_histogramview0")->isChecked());
    }
    SECTION("a press selects the view")
    {
        f.press("view_deck_histogramview0");

        REQUIRE(f.deck->currentView() == f.histo);
        REQUIRE(f.button("view_deck_histogramview0")->isChecked());
        REQUIRE(!f.button("view_deck_tableview0")->isChecked());
        REQUIRE(!f.histo->isHidden());
        REQUIRE(f.table->isHidden());
    }
}

TEST_CASE("View Deck: a press on the selected view asks the host to collapse the deck", "[view_deck]")
{
    DeckFixture f;

    QSignalSpy collapse_spy(f.deck, &ViewDeck::collapseRequestedSignal);
    QSignalSpy open_spy(f.deck, &ViewDeck::openRequestedSignal);

    SECTION("another view does not ask for it")
    {
        f.press("view_deck_histogramview0");

        REQUIRE(collapse_spy.count() == 0);
        REQUIRE(open_spy.count() == 0);
    }
    SECTION("without a host for the request the view stays shown (separate view window)")
    {
        f.press("view_deck_tableview0");

        REQUIRE(collapse_spy.count() == 1);
        REQUIRE(!f.deck->collapsed());
        REQUIRE(f.deck->currentView() == f.table);
        REQUIRE(f.button("view_deck_tableview0")->isChecked());
    }
    SECTION("a collapsed deck shows no selected view and asks to open on a press")
    {
        f.deck->setCollapsedSlot(true);

        REQUIRE(!f.button("view_deck_tableview0")->isChecked());

        f.press("view_deck_tableview0");

        REQUIRE(collapse_spy.count() == 0);
        REQUIRE(open_spy.count() == 1);

        f.press("view_deck_histogramview0");

        REQUIRE(open_spy.count() == 2);
        REQUIRE(f.deck->currentView() == f.histo);

        f.deck->setCollapsedSlot(false);

        REQUIRE(f.button("view_deck_histogramview0")->isChecked());
    }
}

TEST_CASE("View Deck: showView opens a collapsed deck", "[view_deck]")
{
    DeckFixture f;

    QSignalSpy open_spy(f.deck, &ViewDeck::openRequestedSignal);

    f.deck->showView(f.geo);

    REQUIRE(f.deck->currentView() == f.geo);
    REQUIRE(open_spy.count() == 0);

    f.deck->setCollapsedSlot(true);
    f.deck->showView(f.table);

    REQUIRE(f.deck->currentView() == f.table);
    REQUIRE(open_spy.count() == 1);
}

TEST_CASE("View Deck: the close icon asks to close its view", "[view_deck]")
{
    DeckFixture f;

    QSignalSpy close_spy(f.deck, &ViewDeck::closeViewRequestedSignal);

    f.press("view_deck_histogramview0_close");

    REQUIRE(close_spy.count() == 1);
    REQUIRE(close_spy.at(0).at(0).value<QWidget*>() == f.histo);

    // the close icon is not a press on the view button
    REQUIRE(f.deck->currentView() == f.table);
}

TEST_CASE("View Deck: a deleted view leaves the deck", "[view_deck]")
{
    DeckFixture f;

    SECTION("a view that is not shown")
    {
        delete f.histo;
        QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);

        REQUIRE(!f.button("view_deck_histogramview0"));
        REQUIRE(f.deck->currentView() == f.table);
        REQUIRE(f.button("view_deck_tableview0")->isChecked());
    }
    SECTION("the shown view, the next view is shown")
    {
        delete f.table;
        QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);

        REQUIRE(!f.button("view_deck_tableview0"));
        REQUIRE(f.deck->currentView() == f.histo);
        REQUIRE(f.button("view_deck_histogramview0")->isChecked());
    }
    SECTION("the last view, the view before is shown")
    {
        f.deck->showView(f.geo);

        delete f.geo;
        QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);

        REQUIRE(f.deck->currentView() == f.histo);
        REQUIRE(f.button("view_deck_histogramview0")->isChecked());
    }
    SECTION("all views")
    {
        delete f.table;
        delete f.histo;
        delete f.geo;
        QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);

        REQUIRE(f.deck->currentView() == nullptr);
        REQUIRE(f.button("add_view"));
    }
    SECTION("a view closed from its own close icon")
    {
        QObject::connect(f.deck, &ViewDeck::closeViewRequestedSignal, [] (QWidget* w) { delete w; });

        f.press("view_deck_tableview0_close");
        QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);

        REQUIRE(!f.button("view_deck_tableview0"));
        REQUIRE(f.deck->currentView() == f.histo);
    }
}

TEST_CASE("View Deck: a disabled view does not stay shown", "[view_deck]")
{
    DeckFixture f;

    f.deck->setViewEnabled(f.table, false);

    REQUIRE(!f.button("view_deck_tableview0")->isEnabled());
    REQUIRE(f.deck->currentView() == f.histo);

    SECTION("the last enabled view stays shown")
    {
        f.deck->setViewEnabled(f.geo, false);
        f.deck->setViewEnabled(f.histo, false);

        REQUIRE(f.deck->currentView() == f.histo);
    }
    SECTION("an enabled view can be selected again")
    {
        f.deck->setViewEnabled(f.table, true);
        f.press("view_deck_tableview0");

        REQUIRE(f.deck->currentView() == f.table);
    }
}

TEST_CASE("View Deck: only the shown view counts for the minimum width", "[view_deck]")
{
    DeckFixture f;

    // the minimum width of a view comes from its content
    auto setContentWidth = [] (QWidget* view, int width)
    {
        auto content = new QWidget;
        content->setMinimumWidth(width);

        auto layout = new QHBoxLayout(view);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(content);
    };

    setContentWidth(f.table, 900);
    setContentWidth(f.histo, 100);

    f.deck->showView(f.table);
    REQUIRE(f.deck->viewArea()->minimumSizeHint().width() >= 900);

    f.deck->showView(f.histo);
    REQUIRE(f.deck->viewArea()->minimumSizeHint().width() < 900);
}

TEST_CASE("View Deck: views are selected by name for UI tests", "[view_deck]")
{
    DeckFixture f;

    REQUIRE(f.deck->uiSet("Geographic View 1"));
    REQUIRE(f.deck->currentView() == f.geo);

    // selecting the shown view again keeps it shown
    QSignalSpy collapse_spy(f.deck, &ViewDeck::collapseRequestedSignal);
    REQUIRE(f.deck->uiSet("Geographic View 1"));
    REQUIRE(collapse_spy.count() == 0);

    REQUIRE(!f.deck->uiSet("No Such View"));
    REQUIRE(f.deck->currentView() == f.geo);
}

TEST_CASE("View Deck: add and close can be disabled", "[view_deck]")
{
    DeckFixture f;

    f.deck->setAddRemoveEnabled(false);

    REQUIRE(!f.button("add_view")->isEnabled());
    REQUIRE(!f.button("view_deck_tableview0_close")->isEnabled());

    QSignalSpy add_spy(f.deck, &ViewDeck::addViewRequestedSignal);

    f.deck->setAddRemoveEnabled(true);
    f.press("add_view");

    REQUIRE(add_spy.count() == 1);
}

TEST_CASE("Deck buttons: one row labels get the height of two row labels", "[view_deck]")
{
    const DeckButton button;

    const QSize flight_deck = DeckButton::sizeForLabels({ { "Data", "Sources" }, { "Log" } }, button.font());
    const QSize view_deck   = DeckButton::sizeForLabels({ { "Table 0" }, { "Geo 1" } }, button.font());

    REQUIRE(view_deck.height() == flight_deck.height());
    REQUIRE(view_deck.width() >= DeckButton::IconSize);

    SECTION("the buttons of a deck have the common size")
    {
        DeckFixture f;

        REQUIRE(f.button("view_deck_tableview0")->size() == f.button("view_deck_geographicview1")->size());
        REQUIRE(f.button("view_deck_tableview0")->height() == flight_deck.height());
    }
}
