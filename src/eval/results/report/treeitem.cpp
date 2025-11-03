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

#include "eval/results/report/treeitem.h"
#include "logger.h"

namespace EvaluationResultsReport
{
    TreeItem::TreeItem(const string& name, TreeItem* parent_item)
        : name_(name), parent_item_(parent_item)
    {
        if (parent_item_)
            id_ = parent_item_->id()+":"+name_;
        else
            id_ = name_;

        logdbg << "id '" << id_ << "'";
    }

    TreeItem* TreeItem::parentItem()
    {
        return parent_item_;
    }

    string TreeItem::name() const
    {
        return name_;
    }
    
    string TreeItem::id() const
    {
        return id_;
    }
}

