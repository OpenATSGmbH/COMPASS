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

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>

#include <memory>

class EvaluationManager;


namespace EvaluationResultsReport
{
    using namespace std;

    class RootItem;

    class TreeModel : public QAbstractItemModel
    {
    public:
        TreeModel(EvaluationManager& eval_man);

        QVariant data(const QModelIndex& index, int role) const override;
        Qt::ItemFlags flags(const QModelIndex& index) const override;
        QVariant headerData(int section, Qt::Orientation orientation,
                            int role = Qt::DisplayRole) const override;
        QModelIndex index(int row, int column,
                          const QModelIndex& parent = QModelIndex()) const override;
        QModelIndex parent(const QModelIndex& index) const override;
        int rowCount(const QModelIndex& parent = QModelIndex()) const override;
        int columnCount(const QModelIndex& parent = QModelIndex()) const override;

        QModelIndex findItem (const string& id); // "Report:Results:Overview"

        void beginReset();
        void clear();
        void endReset();

        shared_ptr<RootItem> rootItem() const;

    protected:
        EvaluationManager& eval_man_;

        shared_ptr<RootItem> root_item_;
    };

}
