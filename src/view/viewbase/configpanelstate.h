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

#include <string>

/**
 * Show state of the config panel of a view. The panel collapses and expands by the view width, unless the
 * user collapsed it.
 */
namespace config_panel
{

enum class State
{
    Expanded,
    AutoCollapsed,
    UserCollapsed
};

enum class UserAction
{
    None,
    Open,
    Close
};

/// data width next to the panel below which the panel collapses by itself
constexpr int ComfortableDataWidth = 600;
/// the panel expands only above the collapse width plus this, so it does not open and close while the width moves
constexpr int Hysteresis = 50;

/// view widths
struct Thresholds
{
    int collapse_below = 0;
    int expand_above   = 0;
};

inline Thresholds thresholdsFor(int panel_min_width)
{
    Thresholds t;
    t.collapse_below = panel_min_width + ComfortableDataWidth;
    t.expand_above   = t.collapse_below + Hysteresis;

    return t;
}

/// The panel collapses only when the view gets narrow (previous_width at or above the collapse width), so a panel
/// the user opened in a narrow view stays open.
inline State nextState(State state, int previous_width, int width, const Thresholds& t, UserAction action)
{
    if (action == UserAction::Open)
        return State::Expanded;
    if (action == UserAction::Close)
        return State::UserCollapsed;

    switch (state)
    {
        case State::Expanded:
            return previous_width >= t.collapse_below && width < t.collapse_below ? State::AutoCollapsed
                                                                                  : State::Expanded;
        case State::AutoCollapsed:
            return width > t.expand_above ? State::Expanded : State::AutoCollapsed;
        case State::UserCollapsed:
            return State::UserCollapsed;
    }

    return state;
}

inline bool isShown(State state)
{
    return state == State::Expanded;
}

inline std::string toString(State state)
{
    switch (state)
    {
        case State::Expanded:      return "Expanded";
        case State::AutoCollapsed: return "AutoCollapsed";
        case State::UserCollapsed: return "UserCollapsed";
    }

    return "";
}

}
