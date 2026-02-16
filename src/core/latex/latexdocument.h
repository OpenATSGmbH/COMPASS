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

class LatexSection;

class LatexDocument : public LatexContent
{
public:
    LatexDocument(const std::string& path, const std::string& filename); // path has to end with /

    void write();

    std::string title() const;
    void title(const std::string& title);

    std::string author() const;
    void author(const std::string& author);

    std::string abstract() const;
    void abstract(const std::string& abstract);

    virtual std::string toString() override;

    LatexSection& getSection (const std::string& id); // bla:bla2

    bool hasSubSection (const std::string& heading);
    LatexSection& getSubSection (const std::string& heading);
    void addSubSection (const std::string& heading);

    std::string path() const;
    std::string filename() const;

protected:
    std::string path_;
    std::string filename_;

    std::string title_;
    std::string author_;
    std::string abstract_;

    std::string footer_left_;
    std::string footer_right_;
};
