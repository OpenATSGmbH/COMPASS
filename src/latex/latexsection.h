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

#include "latexcontent.h"

enum class LatexSectionLevel
{
    CHAPTER=0,
    SECTION,
    SUBSECTION,
    SUBSUBSECTION,
    PARAGRAPH,
    SUBPARAGRAPH,
};

class LatexTable;
class LatexImage;

class LatexSection : public LatexContent
{
public:
    LatexSection(LatexSectionLevel level, const std::string& heading);

    LatexSectionLevel level() const;

    std::string heading() const;

    bool hasSubSection (const std::string& heading);
    LatexSection& getSubSection (const std::string& heading);
    void addSubSection (const std::string& heading);

    void addText (const std::string& latex_str);

    bool hasTable (const std::string& name);
    LatexTable& getTable (const std::string& name);
    void addTable (const std::string& name, unsigned int num_columns,
                   std::vector<std::string> headings, std::string heading_alignment="",
                   bool convert_to_latex=true);

    bool hasImage (const std::string& filename);
    LatexImage& getImage (const std::string& filename);
    void addImage (const std::string& filename, const std::string& caption);

    virtual std::string toString() override;

    std::string label() const;
    void label(const std::string& label);

protected:
    LatexSectionLevel level_;
    std::string heading_;

    std::string label_;
};
