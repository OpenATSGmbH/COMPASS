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

#include "report/reportitem.h"

#include <QVariant>

#include <memory>

namespace ResultReport
{

/**
 * ReportItem which can be visualized as part of an item model (Report, Section, etc.)
 */
class TreeItem : public ReportItem
{
public:
    TreeItem(const std::string& name,
             TreeItem* parent_item);
    TreeItem(TreeItem* parent_item);
    virtual ~TreeItem();

    virtual TreeItem *child(int row) = 0;
    virtual int childCount() const = 0;
    virtual int columnCount() const = 0;
    virtual QVariant data(int column) const = 0;
    virtual int row() const = 0;

    virtual TreeItem* parentItem() override;
    virtual const TreeItem* parentItem() const override;
};

}
