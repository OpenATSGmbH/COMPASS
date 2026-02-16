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

#include <vector>
#include <string>

class LatexTable : public LatexContent
{
public:
    LatexTable(const std::string& name, unsigned int num_columns,
               std::vector<std::string> headings, std::string heading_alignment="", bool convert_to_latex=true);

    void addRow (std::vector<std::string> row); // not latefied yet

    virtual std::string toString() override;

    std::string name() const;

    void setWideTable(bool wide_table);
    void setMaxRowCount(int max_row_count);

protected:
    std::string name_;
    unsigned int num_columns_;
    std::vector<std::string> headings_;
    std::string heading_alignment_;
    bool convert_to_latex_ {true};
    bool wide_table_ {false};
    int num_max_rows_ = -1;

    std::vector<std::vector<std::string>> rows_;

    std::string getLine (const std::vector<std::string>& row, bool bold=false);
    std::string getPointsLine ();


};
