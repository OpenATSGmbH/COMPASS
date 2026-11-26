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

#include "latexsection.h"
#include "latextable.h"
#include "lateximage.h"
#include "latextext.h"

#include "traced_assert.h"
#include <sstream>

using namespace std;

LatexSection::LatexSection(LatexSectionLevel level, const std::string& heading)
    : level_(level), heading_(heading)
{
}

LatexSectionLevel LatexSection::level() const
{
    return level_;
}

std::string LatexSection::heading() const
{
    return heading_;
}

bool LatexSection::hasSubSection (const std::string& heading)
{
    return findSubSection(heading) != nullptr;
}

LatexSection& LatexSection::getSubSection (const std::string& heading)
{
    LatexSection* tmp = findSubSection (heading);
    traced_assert(tmp);
    return *tmp;
}

void LatexSection::addSubSection (const std::string& heading)
{
    traced_assert(!hasSubSection(heading));
    traced_assert(level_ <= LatexSectionLevel::PARAGRAPH);
    unsigned int next_level = static_cast<unsigned int>(level_) + 1;
    sub_content_.push_back(unique_ptr<LatexSection>(
                           new LatexSection(static_cast<LatexSectionLevel>(next_level), heading)));
    traced_assert(hasSubSection(heading));
}

void LatexSection::addText (const std::string& latex_str)
{
    sub_content_.push_back(unique_ptr<LatexText>(new LatexText(latex_str)));
}

bool LatexSection::hasTable (const std::string& name)
{
    return findTable(name) != nullptr;
}

LatexTable& LatexSection::getTable (const std::string& name)
{
    LatexTable* tmp = findTable (name);
    traced_assert(tmp);
    return *tmp;
}

void LatexSection::addTable (const std::string& name, unsigned int num_columns,
                             std::vector<std::string> headings, std::string heading_alignment,
                             bool convert_to_latex)
{
    traced_assert(!hasTable(name));
    sub_content_.push_back(unique_ptr<LatexTable>(
                           new LatexTable(name, num_columns, headings, heading_alignment, convert_to_latex)));
    traced_assert(hasTable(name));
}

bool LatexSection::hasImage (const std::string& filename)
{
    return findImage(filename) != nullptr;
}

LatexImage& LatexSection::getImage (const std::string& filename)
{
    LatexImage* tmp = findImage (filename);
    traced_assert(tmp);
    return *tmp;
}

void LatexSection::addImage (const std::string& filename, const std::string& caption)
{
    traced_assert(!hasImage(filename));
    sub_content_.push_back(unique_ptr<LatexImage>(
                           new LatexImage(filename, caption)));
    traced_assert(hasImage(filename));
}

std::string LatexSection::toString()
{
    stringstream ss;

    if (level_ == LatexSectionLevel::CHAPTER)
        ss << R"(\chapter{)" << heading_ << "}";
    else if (level_ == LatexSectionLevel::SECTION)
        ss << R"(\section{)" << heading_ << "}";
    else if (level_ == LatexSectionLevel::SUBSECTION)
        ss << R"(\subsection{)" << heading_ << "}";
    else if (level_ == LatexSectionLevel::SUBSUBSECTION)
        ss << R"(\subsubsection{)" << heading_ << "}";
    else if (level_ == LatexSectionLevel::PARAGRAPH)
        ss << R"(\paragraph{)" << heading_ << "}";
    else if (level_ == LatexSectionLevel::SUBPARAGRAPH)
        ss << R"(\subparagraph{)" << heading_ << "}";
    else
        throw std::runtime_error ("LatexSection: toString: unkown section level");

    if (label_.size())
        ss << "\n" << R"(\label{)" << label_ << "}";

    if (level_ == LatexSectionLevel::PARAGRAPH || level_ == LatexSectionLevel::SUBPARAGRAPH)
        ss << R"(\ \\)"; // for correct placement after

    ss << "\n";

    ss << LatexContent::toString();

    return ss.str();
}

std::string LatexSection::label() const
{
    return label_;
}

void LatexSection::label(const std::string& label)
{
    label_ = label;
}
