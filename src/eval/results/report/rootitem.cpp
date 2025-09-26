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

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/section_id.h"

#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;

namespace EvaluationResultsReport
{

    RootItem::RootItem(EvaluationManager& eval_man)
        : TreeItem("Report", nullptr), eval_man_(eval_man)
    {
        root_section_ = make_shared<Section>("Results", "", this, eval_man_);
    }

    RootItem::~RootItem()
    {
        loginf << "end";
    }

    TreeItem* RootItem::child(int row)
    {
        if (row < 0 || row > 0)
            return nullptr;

        return root_section_.get();
    }

    int RootItem::childCount() const
    {
        return 1;
    }

    int RootItem::columnCount() const
    {
        return 1;
    }

    QVariant RootItem::data(int column) const
    {
        traced_assert(column == 0);

        return name_.c_str();
    }

    int RootItem::row() const
    {
        return 0;
    }

    std::shared_ptr<Section> RootItem::rootSection()
    {
        return root_section_;
    }

    Section& RootItem::getSection (const std::string& id)
    {
        logdbg << "id '" << id << "'";

        traced_assert(id.size());

        std::vector<std::string> parts = SectionID::subSections(id);
        traced_assert(parts.size());

        Section* tmp;

        for (unsigned int cnt=0; cnt < parts.size(); ++cnt)
        {
            std::string& heading = parts.at(cnt);

            if (cnt == 0) // first
            {
                if (!root_section_->hasSubSection(heading))
                    root_section_->addSubSection(heading);

                tmp = &root_section_->getSubSection(heading);
            }
            else // previous section
            {
                traced_assert(tmp);

                if (!tmp->hasSubSection(heading))
                    tmp->addSubSection(heading);

                tmp = &tmp->getSubSection(heading);
            }
        }

        traced_assert(tmp);
        return *tmp;
    }
}
