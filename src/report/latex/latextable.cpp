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

#include "latextable.h"
#include "stringconv.h"
#include "logger.h"

#include "traced_assert.h"
#include <sstream>

using namespace std;
using namespace Utils;

LatexTable::LatexTable(const std::string& name, 
                       unsigned int num_columns,
                       std::vector<std::string> headings, 
                       std::string heading_alignment,
                       bool convert_to_latex)
    : name_(name), num_columns_(num_columns), headings_(headings), heading_alignment_(heading_alignment),
      convert_to_latex_(convert_to_latex)
{
    traced_assert(headings_.size() == num_columns_);

    if (!heading_alignment_.size())
    {
        stringstream ss;

        ss << "|";
        for (unsigned int cnt=0; cnt < num_columns_; ++cnt)
            ss << " X |";

        heading_alignment_ = ss.str();
    }

    loginf << "name " << name << " num cols " << num_columns_
           << " heading alignment '" << heading_alignment_ << "'";

    traced_assert(String::split(heading_alignment_, '|').size() == num_columns_+1);
}

void LatexTable::addRow (std::vector<std::string> row)
{
    traced_assert(row.size() == num_columns_);
    rows_.push_back(row);
}

std::string LatexTable::toString()
{
    //assert (!content_.size());

    stringstream ss;

    if (wide_table_)
    {
        //ss << R"(\newgeometry{margin=1cm})" << "\n";

        //landscape2 is our own macro for landscape pages, which generates horizontal footers also on landscape pages
        ss << R"(\begin{landscape2})" << "\n";

        //note: this makes the footer disappear on the first landscape page, so its commented out for now
        //ss << R"(\thispagestyle{empty})" << "\n";
    }

    //note: textwidth is now true for both portrait and landscape
    //if (wide_table_)
        ss << R"(\begin{tabularx}{\textwidth}[H]{)" << heading_alignment_ << " }\n";
    //else
    //    ss << R"(\begin{tabularx}{\linewidth}[H]{)" << heading_alignment_ << " }\n";

    ss << R"(\hline)" << "\n";
    ss << getLine(headings_, true);

    unsigned int row_cnt=0;
    bool max_rows_reached = false;
    for (auto& row : rows_)
    {
        if (num_max_rows_ >= 0 && row_cnt > (unsigned int)num_max_rows_)
        {
            max_rows_reached = true;
            break;
        }

        ss << getLine(row);

        ++row_cnt;
    }

    if (max_rows_reached)
        ss << getPointsLine();

    ss << R"(\end{tabularx})" << "\n";

    ss << R"(\ \\)" << "\n";

    if (wide_table_)
    {
        ss << R"(\end{landscape2})" << "\n";
        //ss << R"(\restoregeometry)" << "\n";
    }

    return ss.str();
}

std::string LatexTable::name() const
{
    return name_;
}

void LatexTable::setWideTable(bool wide_table)
{
    wide_table_ = wide_table;
}

void LatexTable::setMaxRowCount(int max_row_count)
{
    num_max_rows_ = max_row_count;
}

std::string LatexTable::getLine (const std::vector<std::string>& row, bool bold)
{
    stringstream ss;

    for (unsigned int cnt=0; cnt < row.size(); ++cnt)
    {
        if (cnt != 0)
            ss << " & ";

        if (bold)
            ss << R"(\textbf{)";

        if (convert_to_latex_)
            ss << String::latexString(row.at(cnt));
        else
            ss << row.at(cnt);

        if (bold)
            ss << "}";
    }

    ss << R"( \\ \hline)" << "\n";

    return ss.str();
}

std::string LatexTable::getPointsLine ()
{
    stringstream ss;

    for (unsigned int cnt=0; cnt < num_columns_; ++cnt)
    {
        if (cnt != 0)
            ss << " & ";

        ss << "...";
    }

    ss << R"( \\ \hline)" << "\n";

    return ss.str();
}
