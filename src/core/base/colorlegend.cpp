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

#include "colorlegend.h"
#include "json.hpp"

/**
 */
bool ColorLegend::empty() const
{
    return entries_.empty();
}

/**
 */
void ColorLegend::clear()
{
    entries_.clear();
}

/**
 */
void ColorLegend::addEntry(const QColor& color, const std::string& descr)
{
    entries_.emplace_back(color, descr);
}

/**
 */
nlohmann::json ColorLegend::toJSON() const
{
    nlohmann::json j;

    j[ "entries" ] = nlohmann::json::array();

    auto& entries_json = j[ "entries" ];

    for (const auto& e : entries_)
    {
        nlohmann::json ej;

        ej[ "color"       ] = e.first.name().toStdString();
        ej[ "description" ] = e.second;

        entries_json.push_back(ej);
    }

    return j;
}

/**
 */
bool ColorLegend::fromJSON(const nlohmann::json& j)
{
    clear();

    if (!j.is_object())
        return false;

    if (!j.contains("entries"))
        return false;

    const nlohmann::json& entries_json = j[ "entries" ];
    if (!entries_json.is_array())
        return false;

    for (const auto& ej : entries_json)
    {
        if (!ej.contains("color") ||
            !ej.contains("description"))
            return false;

        std::string col_str = ej[ "color" ];
        std::string descr   = ej[ "description" ];

        entries_.emplace_back(QColor(QString::fromStdString(col_str)), descr);
    }

    return true;
}
