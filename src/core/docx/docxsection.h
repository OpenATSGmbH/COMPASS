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

#include "docxcontent.h"

#include <vector>
#include <string>

class DocxTable;
class DocxImage;

class DocxSection : public DocxContent
{
public:
    // level 1 = Heading1, 2 = Heading2, etc.
    DocxSection(int level, const std::string& heading);

    int level() const;
    std::string heading() const;

    bool hasSubSection(const std::string& heading);
    DocxSection& getSubSection(const std::string& heading);
    void addSubSection(const std::string& heading);

    void addText(const std::string& text);

    bool hasTable(const std::string& name);
    DocxTable& getTable(const std::string& name);
    void addTable(const std::string& name, unsigned int num_columns,
                  std::vector<std::string> headings);

    bool hasImage(const std::string& filename);
    DocxImage& getImage(const std::string& filename);
    void addImage(const std::string& filename, const std::string& caption,
                  const std::string& rel_id, int width_emu, int height_emu);

    std::string toXml() override;

protected:
    int level_;
    std::string heading_;
};
