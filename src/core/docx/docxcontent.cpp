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

#include "docxcontent.h"
#include "docxsection.h"
#include "docxtable.h"
#include "docximage.h"

#include <sstream>

DocxContent::DocxContent()
{
}

std::string DocxContent::toXml()
{
    std::stringstream ss;

    for (auto& cont_it : sub_content_)
        ss << cont_it->toXml();

    return ss.str();
}

DocxSection* DocxContent::findSubSection(const std::string& heading)
{
    for (auto& cont_it : sub_content_)
    {
        auto tmp = dynamic_cast<DocxSection*>(cont_it.get());
        if (tmp && tmp->heading() == heading)
            return tmp;
    }
    return nullptr;
}

DocxTable* DocxContent::findTable(const std::string& name)
{
    for (auto& cont_it : sub_content_)
    {
        auto tmp = dynamic_cast<DocxTable*>(cont_it.get());
        if (tmp && tmp->name() == name)
            return tmp;
    }
    return nullptr;
}

DocxImage* DocxContent::findImage(const std::string& filename)
{
    for (auto& cont_it : sub_content_)
    {
        auto tmp = dynamic_cast<DocxImage*>(cont_it.get());
        if (tmp && tmp->filename() == filename)
            return tmp;
    }
    return nullptr;
}
