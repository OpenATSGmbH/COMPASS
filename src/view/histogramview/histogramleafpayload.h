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

#include <QColor>

#include <string>

/**
 * DBContentLeafPayload adapter for Histogram View.
 *
 * At this stage the histogram chart still groups bars per DBContent; the layer
 * panel exposes the finer-grained layers that *would* feed the histogram if
 * per-layer grouping were enabled. A payload holds just the layer identity,
 * its row count, and the color resolved for the current color mode. Visibility
 * flips are stored locally but not yet acted on by the chart.
 *
 * Persisted id uses the same key convention as scatter/table
 * ("<ds_type>:<ds_name>:L<n>:<dbcontent>") so hidden state will round-trip
 * across reloads and view-point restores once per-layer filtering is wired up.
 */
class HistogramLeafPayload : public DBContentLeafPayload
{
public:
    HistogramLeafPayload(const std::string& full_key,
                         unsigned int count,
                         const QColor& color)
        : full_key_(full_key)
        , count_(count)
        , color_(color)
    {}

    std::string name() const override { return full_key_; }
    unsigned int count() const override { return count_; }
    QColor color() const override { return color_; }

    void setVisible(bool v) override { visible_ = v; }

    std::string persistenceId() const override { return full_key_; }

    bool visible() const { return visible_; }

private:
    std::string  full_key_;
    unsigned int count_ {0};
    QColor       color_;
    bool         visible_ {true};
};
