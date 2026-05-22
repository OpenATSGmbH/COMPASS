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
#include "scatterseries.h"

#include <string>

/**
 * DBContentLeafPayload adapter over a ScatterSeriesCollection::DataSeries*.
 *
 * Owned by ScatterPlotViewDataWidget; its lifetime is tied to the current
 * scatter_series_ rebuild cycle. Forwards visibility toggles back to the
 * underlying DataSeries so the chart picks them up on next redraw.
 *
 * persistenceId() returns the full series key ("ds_type:ds_name:L<n>:
 * dbcontent") so viewpoint round-trip survives reloads and color-mode changes.
 */
class ScatterLeafPayload : public DBContentLeafPayload
{
public:
    ScatterLeafPayload(const std::string& full_key,
                       ScatterSeriesCollection::DataSeries* series)
        : full_key_(full_key), series_(series)
    {}

    std::string name() const override { return series_ ? series_->name : std::string{}; }

    unsigned int count() const override
    {
        return series_ ? (unsigned int)series_->scatter_series.points.size() : 0u;
    }

    unsigned int nullCount() const override
    {
        return series_ ? (unsigned int)series_->null_count : 0u;
    }

    QColor color() const override { return series_ ? series_->color : QColor(); }

    void setVisible(bool v) override
    {
        if (series_) series_->visible = v;
    }

    std::string persistenceId() const override { return full_key_; }

    QVariant customColumn(int view_col_idx) const override
    {
        // col 0 (of custom cols) = "# NULL"
        if (view_col_idx == 0)
        {
            const unsigned int n = nullCount();
            return n ? QVariant((qulonglong)n) : QVariant();
        }
        return {};
    }

private:
    std::string                          full_key_;
    ScatterSeriesCollection::DataSeries* series_{nullptr};
};
