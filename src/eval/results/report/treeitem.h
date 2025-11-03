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

#include <QVariant>

#include <memory>

namespace EvaluationResultsReport
{
    using namespace std;

    class TreeItem
    {
    public:
        TreeItem(const string& name, TreeItem* parent_item);

        virtual TreeItem *child(int row) = 0;
        virtual int childCount() const = 0;
        virtual int columnCount() const = 0;
        virtual QVariant data(int column) const = 0;
        virtual int row() const = 0;
        TreeItem* parentItem();

        string name() const;
        string id() const; // (parent_id):name

    protected:
        string name_;
        string id_;

        TreeItem* parent_item_ {nullptr};
    };
}
