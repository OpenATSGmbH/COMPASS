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

class LatexSection;
class LatexTable;
class LatexImage;

class LatexContent
{
public:
    LatexContent();

    virtual std::string toString();

protected:
    //std::vector<std::string> content_; // main content as latex strings

    std::vector<std::unique_ptr<LatexContent>> sub_content_;

    LatexSection* findSubSection (const std::string& heading); // nullptr if not found
    LatexTable* findTable (const std::string& name); // nullptr if not found
    LatexImage* findImage (const std::string& filename); // nullptr if not found
};
