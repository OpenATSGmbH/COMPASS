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

#include "report/treeitem.h"

#include "logger.h"

namespace ResultReport
{

/**
 */
TreeItem::TreeItem(const std::string& name, 
                   TreeItem* parent_item)
:   ReportItem(name, parent_item)
{
}

/**
 */
TreeItem::TreeItem(TreeItem* parent_item)
:   ReportItem(parent_item)
{
}

/**
 */
TreeItem::~TreeItem() = default;

/**
 */
TreeItem* TreeItem::parentItem()
{
    return dynamic_cast<TreeItem*>(parent_item_);
}

/**
 */
const TreeItem* TreeItem::parentItem() const
{
    return dynamic_cast<const TreeItem*>(parent_item_);
}

}
