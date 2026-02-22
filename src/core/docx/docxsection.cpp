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

#include "docxsection.h"
#include "docxtable.h"
#include "docximage.h"
#include "docxtext.h"

#include <sstream>
#include <cassert>

int DocxSection::next_bookmark_id_ = 0;

DocxSection::DocxSection(int level, const std::string& heading)
:   level_      (level)
,   heading_    (heading)
,   bookmark_id_(next_bookmark_id_++)
{
}

int DocxSection::level() const { return level_; }
std::string DocxSection::heading() const { return heading_; }
int DocxSection::bookmarkId() const { return bookmark_id_; }
std::string DocxSection::bookmarkName() const { return bookmark_name_; }
void DocxSection::setBookmarkName(const std::string& name) { bookmark_name_ = name; }

bool DocxSection::hasSubSection(const std::string& heading)
{
    return findSubSection(heading) != nullptr;
}

DocxSection& DocxSection::getSubSection(const std::string& heading)
{
    auto* s = findSubSection(heading);
    assert(s);
    return *s;
}

void DocxSection::addSubSection(const std::string& heading)
{
    assert(!hasSubSection(heading));
    sub_content_.push_back(std::make_unique<DocxSection>(level_ + 1, heading));
}

void DocxSection::addText(const std::string& text)
{
    sub_content_.push_back(std::make_unique<DocxText>(text));
}

bool DocxSection::hasTable(const std::string& name)
{
    return findTable(name) != nullptr;
}

DocxTable& DocxSection::getTable(const std::string& name)
{
    auto* t = findTable(name);
    assert(t);
    return *t;
}

void DocxSection::addTable(const std::string& name, unsigned int num_columns,
                           std::vector<std::string> headings)
{
    assert(!hasTable(name));
    sub_content_.push_back(std::make_unique<DocxTable>(name, num_columns, std::move(headings)));
}

bool DocxSection::hasImage(const std::string& filename)
{
    return findImage(filename) != nullptr;
}

DocxImage& DocxSection::getImage(const std::string& filename)
{
    auto* img = findImage(filename);
    assert(img);
    return *img;
}

void DocxSection::addImage(const std::string& filename, const std::string& caption,
                           const std::string& rel_id, int width_emu, int height_emu)
{
    assert(!hasImage(filename));
    sub_content_.push_back(std::make_unique<DocxImage>(filename, caption, rel_id, width_emu, height_emu));
}

std::string DocxSection::toXml()
{
    std::stringstream ss;

    // heading paragraph with style
    std::string escaped;
    for (char c : heading_)
    {
        switch (c)
        {
            case '&':  escaped += "&amp;";  break;
            case '<':  escaped += "&lt;";   break;
            case '>':  escaped += "&gt;";   break;
            case '"':  escaped += "&quot;"; break;
            default:   escaped += c;        break;
        }
    }

    std::string style = "Heading" + std::to_string(std::min(level_, 6));

    ss << "<w:p>"
       << "<w:pPr><w:pStyle w:val=\"" << style << "\"/></w:pPr>";

    if (!bookmark_name_.empty())
    {
        ss << "<w:bookmarkStart w:id=\"" << bookmark_id_ << "\" w:name=\"" << bookmark_name_ << "\"/>"
           << "<w:bookmarkEnd w:id=\"" << bookmark_id_ << "\"/>";
    }

    ss << "<w:r>"
       << "<w:t xml:space=\"preserve\">" << escaped << "</w:t>"
       << "</w:r>"
       << "</w:p>\n";

    // render children (text, tables, images, subsections)
    ss << DocxContent::toXml();

    return ss.str();
}
