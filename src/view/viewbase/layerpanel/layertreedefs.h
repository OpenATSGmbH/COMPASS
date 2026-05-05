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

#include <QHeaderView>
#include <QString>
#include <QVariant>

#include <functional>
#include <vector>

/**
 * Spec for a custom column. Custom columns start at index 2 (Name=0, Count=1).
 *
 * group_aggregator: optional — when a group row has no own value for this
 * column (itemData returns invalid), the model collects valid values from the
 * full descendant subtree and passes them to this callback. Without an
 * aggregator the group row stays empty.
 */
struct LayerColumnSpec
{
    QString                 header;
    int                     default_width = 80;
    QHeaderView::ResizeMode resize_mode   = QHeaderView::Interactive;
    Qt::Alignment           alignment     = Qt::AlignLeft | Qt::AlignVCenter;
    std::function<QVariant(const std::vector<QVariant>&)> group_aggregator;
};
