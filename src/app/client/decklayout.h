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

#pragma once

/**
 * Which of the two decks of the main window are open. A collapsed deck shows only its icon bar.
 * The state with both decks collapsed does not exist.
 */
namespace deck
{

enum class Deck
{
    Flight,
    View
};

enum class Layout
{
    Both,
    FlightDeckOnly,
    ViewDeckOnly
};

constexpr Deck other(Deck deck)
{
    return deck == Deck::Flight ? Deck::View : Deck::Flight;
}

constexpr Layout only(Deck deck)
{
    return deck == Deck::Flight ? Layout::FlightDeckOnly : Layout::ViewDeckOnly;
}

constexpr bool isOpen(Layout layout, Deck deck)
{
    return layout == Layout::Both || layout == only(deck);
}

/// opens the given deck, the other deck keeps its state
constexpr Layout opened(Layout layout, Deck deck)
{
    return isOpen(layout, other(deck)) ? Layout::Both : only(deck);
}

/// collapses the given deck, the other deck opens if it was collapsed
constexpr Layout collapsed(Layout, Deck deck)
{
    return only(other(deck));
}

}
