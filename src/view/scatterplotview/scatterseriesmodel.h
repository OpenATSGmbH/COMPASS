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

#include "scatterseries.h"
#include "scatterseriestreeitem.h"


#include <set>

#include <QAbstractItemModel>
#include <QColor>
#include <QModelIndex>
#include <QVariant>

class COMPASS;

class ScatterSeriesModel : public QAbstractItemModel
{
    Q_OBJECT

signals:
    void visibilityChangedSignal();
    void colorChangedSignal(const std::string& series_name, const QColor& color);

public:
    ScatterSeriesModel();
    virtual ~ScatterSeriesModel();

    QVariant data(const QModelIndex& index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column,
                      const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;

    /// builds the 4-level tree: DSType -> DS -> Line -> DBContent. Group-level
    /// icon colors are resolved via `compass` (context palettes + DataSource
    /// baseColor / lineColor). Leaf colors come from `DataSeries::color`.
    void updateFrom(ScatterSeriesCollection& collection, COMPASS& compass);

    enum DataRole
    {
        IconRole = Qt::UserRole + 100
    };

    void deselectAll();

    std::set<std::string> hiddenSeriesNames() const;
    void applyHiddenSeriesNames(const std::set<std::string>& names);

private:
    std::unique_ptr<ScatterSeriesTreeItem> root_item_;
};

