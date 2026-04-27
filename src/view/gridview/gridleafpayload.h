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

#include "dbcontentlayer.h"

#include <string>

/**
 * DBContentLeafPayload adapter for a GridView series.
 *
 * Grid view aggregates rows into bins, so there is no live per-series data
 * object to toggle — the payload only carries counts for the panel. The data
 * widget reads the panel's hidden set at recompute time to decide which stash
 * groups contribute to the grid.
 *
 * No color: the grid view does not color its layers, so color() returns an
 * invalid QColor and the panel renders with the icon column suppressed.
 */
class GridLeafPayload : public DBContentLeafPayload
{
public:
    GridLeafPayload(const std::string& full_key,
                    const std::string& display_name,
                    unsigned int count,
                    unsigned int null_count)
        : full_key_(full_key)
        , name_(display_name)
        , count_(count)
        , null_count_(null_count)
    {}

    std::string name() const override { return name_; }
    unsigned int count() const override { return count_; }
    unsigned int nullCount() const override { return null_count_; }

    QColor color() const override { return QColor(); }

    void setVisible(bool) override {}

    std::string persistenceId() const override { return full_key_; }

    QVariant customColumn(int view_col_idx) const override
    {
        if (view_col_idx == 0)
            return null_count_ ? QVariant((qulonglong)null_count_) : QVariant();
        return {};
    }

private:
    std::string  full_key_;
    std::string  name_;
    unsigned int count_     {0};
    unsigned int null_count_{0};
};
