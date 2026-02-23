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

#include <string>
#include <memory>
#include <vector>

class DocxSection;
class DocxTable;
class DocxImage;

class DocxContent
{
public:
    DocxContent();
    virtual ~DocxContent() = default;

    virtual std::string toXml();

protected:
    std::vector<std::unique_ptr<DocxContent>> sub_content_;

    DocxSection* findSubSection(const std::string& heading);
    DocxTable* findTable(const std::string& name);
    DocxImage* findImage(const std::string& filename);
};
