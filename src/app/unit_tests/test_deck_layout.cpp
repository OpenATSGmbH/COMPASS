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
#include "decklayout.h"

using namespace deck;

TEST_CASE("Deck layout: a click on the selected view collapses the View Deck", "[deck_layout]")
{
    const Layout layout = collapsed(Layout::Both, Deck::View);

    REQUIRE(isOpen(layout, Deck::Flight));
    REQUIRE(!isOpen(layout, Deck::View));
}

TEST_CASE("Deck layout: a click on the selected tool collapses the Flight Deck", "[deck_layout]")
{
    const Layout layout = collapsed(Layout::Both, Deck::Flight);

    REQUIRE(!isOpen(layout, Deck::Flight));
    REQUIRE(isOpen(layout, Deck::View));
}

TEST_CASE("Deck layout: the decks are never collapsed at the same time", "[deck_layout]")
{
    SECTION("collapsing the View Deck opens a collapsed Flight Deck")
    {
        const Layout layout = collapsed(Layout::ViewDeckOnly, Deck::View);

        REQUIRE(isOpen(layout, Deck::Flight));
        REQUIRE(!isOpen(layout, Deck::View));
    }
    SECTION("collapsing the Flight Deck opens a collapsed View Deck")
    {
        const Layout layout = collapsed(Layout::FlightDeckOnly, Deck::Flight);

        REQUIRE(!isOpen(layout, Deck::Flight));
        REQUIRE(isOpen(layout, Deck::View));
    }
    SECTION("no sequence of clicks collapses both decks")
    {
        for (Layout start : { Layout::Both, Layout::FlightDeckOnly, Layout::ViewDeckOnly })
        {
            for (Deck first : { Deck::Flight, Deck::View })
            {
                for (Deck second : { Deck::Flight, Deck::View })
                {
                    const Layout layout = collapsed(collapsed(start, first), second);

                    REQUIRE((isOpen(layout, Deck::Flight) || isOpen(layout, Deck::View)));
                    REQUIRE(!isOpen(layout, second));
                }
            }
        }
    }
}

TEST_CASE("Deck layout: opening a deck keeps the other deck", "[deck_layout]")
{
    SECTION("a click on a view opens the collapsed View Deck next to the Flight Deck")
    {
        const Layout layout = opened(Layout::FlightDeckOnly, Deck::View);

        REQUIRE(isOpen(layout, Deck::Flight));
        REQUIRE(isOpen(layout, Deck::View));
    }
    SECTION("a click on a tool opens the collapsed Flight Deck next to the View Deck")
    {
        const Layout layout = opened(Layout::ViewDeckOnly, Deck::Flight);

        REQUIRE(isOpen(layout, Deck::Flight));
        REQUIRE(isOpen(layout, Deck::View));
    }
    SECTION("a collapsed deck stays collapsed when the open deck is selected again")
    {
        REQUIRE(!isOpen(opened(Layout::FlightDeckOnly, Deck::Flight), Deck::View));
        REQUIRE(!isOpen(opened(Layout::ViewDeckOnly, Deck::View), Deck::Flight));
    }
    SECTION("an open deck stays open")
    {
        for (Deck deck : { Deck::Flight, Deck::View })
        {
            const Layout layout = opened(Layout::Both, deck);

            REQUIRE(isOpen(layout, Deck::Flight));
            REQUIRE(isOpen(layout, Deck::View));
        }
    }
}
