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

class EvaluationStandardTreeItem
{
public:
    EvaluationStandardTreeItem(EvaluationStandardTreeItem* parent_item = nullptr);

    //void appendChild(EvaluationStandardTreeItem *child);

    virtual EvaluationStandardTreeItem *child(int row) = 0;
    virtual int childCount() const = 0;
    virtual int columnCount() const = 0;
    virtual QVariant data(int column) const = 0;
    virtual int row() const = 0;
    virtual bool checkable() const { return false; }
    virtual bool used() const { return true; }
    virtual void use(bool ok) {}
    EvaluationStandardTreeItem* parentItem();

protected:
    EvaluationStandardTreeItem* parent_item_ {nullptr};
};
