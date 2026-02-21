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

#include "docxtable.h"

#include <sstream>
#include <iomanip>
#include <cstdio>

// CellStyle bit flags from reportdefs.h — duplicated here to avoid dependency
// from core/docx on task/result/report
namespace
{
    const unsigned int StyleTextBold       = 1 << 5;
    const unsigned int StyleTextItalic     = 1 << 6;
    const unsigned int StyleTextStrikeOut  = 1 << 7;

    const unsigned int StyleTextColorRed    = 1 << 10;
    const unsigned int StyleTextColorOrange = 1 << 11;
    const unsigned int StyleTextColorGreen  = 1 << 12;
    const unsigned int StyleTextColorGray   = 1 << 13;

    const unsigned int StyleBGColorRed     = 1 << 20;
    const unsigned int StyleBGColorOrange  = 1 << 21;
    const unsigned int StyleBGColorGreen   = 1 << 22;
    const unsigned int StyleBGColorGray    = 1 << 23;
    const unsigned int StyleBGColorYellow  = 1 << 24;

    const unsigned int StyleInactive       = 1 << 2;
}

DocxTable::DocxTable(const std::string& name, unsigned int num_columns,
                     std::vector<std::string> headings)
:   name_       (name)
,   num_columns_(num_columns)
,   headings_   (std::move(headings))
{
}

std::string DocxTable::name() const { return name_; }

void DocxTable::setWideTable(bool wide_table) { wide_table_ = wide_table; }
void DocxTable::setMaxRowCount(int max_row_count) { num_max_rows_ = max_row_count; }

void DocxTable::addRow(std::vector<std::string> row, std::vector<unsigned int> cell_styles)
{
    rows_.push_back(std::move(row));
    row_styles_.push_back(std::move(cell_styles));
}

std::string DocxTable::xmlEscape(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s)
    {
        switch (c)
        {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default:   out += c;        break;
        }
    }
    return out;
}

std::string DocxTable::colorToHex(int r, int g, int b)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%02X%02X%02X", r, g, b);
    return std::string(buf);
}

std::string DocxTable::cellShadingXml(unsigned int style)
{
    if (style & StyleInactive)
        return R"(<w:shd w:val="clear" w:color="auto" w:fill="C0C0C0"/>)"; // lightGray

    if (style & StyleBGColorRed)
        return R"(<w:shd w:val="clear" w:color="auto" w:fill=")" + colorToHex(240,128,128) + R"("/>)";
    if (style & StyleBGColorOrange)
        return R"(<w:shd w:val="clear" w:color="auto" w:fill=")" + colorToHex(255,165,0) + R"("/>)";
    if (style & StyleBGColorGreen)
        return R"(<w:shd w:val="clear" w:color="auto" w:fill=")" + colorToHex(144,238,144) + R"("/>)";
    if (style & StyleBGColorGray)
        return R"(<w:shd w:val="clear" w:color="auto" w:fill="C0C0C0"/>)";
    if (style & StyleBGColorYellow)
        return R"(<w:shd w:val="clear" w:color="auto" w:fill=")" + colorToHex(255,255,153) + R"("/>)";

    return "";
}

std::string DocxTable::cellRunXml(const std::string& text, unsigned int style)
{
    std::stringstream ss;
    ss << "<w:r>";

    // run properties
    bool has_props = (style & (StyleTextBold | StyleTextItalic | StyleTextStrikeOut |
                               StyleTextColorRed | StyleTextColorOrange |
                               StyleTextColorGreen | StyleTextColorGray |
                               StyleInactive)) != 0;
    if (has_props)
    {
        ss << "<w:rPr>";

        if (style & StyleTextBold)
            ss << "<w:b/>";
        if (style & StyleTextItalic)
            ss << "<w:i/>";
        if (style & StyleTextStrikeOut)
            ss << "<w:strike/>";

        // text color
        if (style & StyleInactive)
            ss << R"(<w:color w:val="808080"/>)"; // darkGray
        else if (style & StyleTextColorRed)
            ss << R"(<w:color w:val=")" << colorToHex(220,20,60) << R"("/>)";
        else if (style & StyleTextColorOrange)
            ss << R"(<w:color w:val=")" << colorToHex(255,140,0) << R"("/>)";
        else if (style & StyleTextColorGreen)
            ss << R"(<w:color w:val=")" << colorToHex(0,128,0) << R"("/>)";
        else if (style & StyleTextColorGray)
            ss << R"(<w:color w:val="808080"/>)";

        ss << "</w:rPr>";
    }

    ss << R"(<w:t xml:space="preserve">)" << xmlEscape(text) << "</w:t></w:r>";
    return ss.str();
}

std::string DocxTable::toXml()
{
    std::stringstream ss;

    // section break for landscape if wide table
    if (wide_table_)
    {
        ss << "<w:p><w:pPr><w:sectPr>"
           << R"(<w:pgSz w:w="16838" w:h="11906" w:orient="landscape"/>)"
           << R"(<w:pgMar w:top="1134" w:right="1134" w:bottom="1134" w:left="1134" w:header="709" w:footer="709" w:gutter="0"/>)"
           << "</w:sectPr></w:pPr></w:p>\n";
    }

    // table start
    ss << "<w:tbl>\n";

    // table properties: full width, borders
    ss << "<w:tblPr>"
       << R"(<w:tblW w:w="5000" w:type="pct"/>)"  // 100% width
       << "<w:tblBorders>"
       << R"(<w:top w:val="single" w:sz="4" w:space="0" w:color="000000"/>)"
       << R"(<w:left w:val="single" w:sz="4" w:space="0" w:color="000000"/>)"
       << R"(<w:bottom w:val="single" w:sz="4" w:space="0" w:color="000000"/>)"
       << R"(<w:right w:val="single" w:sz="4" w:space="0" w:color="000000"/>)"
       << R"(<w:insideH w:val="single" w:sz="4" w:space="0" w:color="000000"/>)"
       << R"(<w:insideV w:val="single" w:sz="4" w:space="0" w:color="000000"/>)"
       << "</w:tblBorders>"
       << R"(<w:tblLook w:val="04A0" w:firstRow="1" w:lastRow="0" w:firstColumn="0" w:lastColumn="0" w:noHBand="0" w:noVBand="1"/>)"
       << "</w:tblPr>\n";

    // grid columns (equal width)
    ss << "<w:tblGrid>";
    for (unsigned int i = 0; i < num_columns_; ++i)
        ss << "<w:gridCol/>";
    ss << "</w:tblGrid>\n";

    // header row (bold, gray background)
    ss << "<w:tr>";
    for (unsigned int col = 0; col < num_columns_; ++col)
    {
        ss << "<w:tc>"
           << "<w:tcPr>"
           << R"(<w:shd w:val="clear" w:color="auto" w:fill="D9D9D9"/>)"
           << "</w:tcPr>"
           << "<w:p>"
           << "<w:r><w:rPr><w:b/></w:rPr>"
           << R"(<w:t xml:space="preserve">)" << xmlEscape(col < headings_.size() ? headings_[col] : "") << "</w:t>"
           << "</w:r>"
           << "</w:p>"
           << "</w:tc>";
    }
    ss << "</w:tr>\n";

    // data rows
    unsigned int max_rows = (num_max_rows_ > 0) ? (unsigned int)num_max_rows_ : (unsigned int)rows_.size();
    unsigned int num_rows_to_write = std::min((unsigned int)rows_.size(), max_rows);

    for (unsigned int row = 0; row < num_rows_to_write; ++row)
    {
        const auto& row_data = rows_[row];
        const auto& styles = row_styles_[row];

        ss << "<w:tr>";
        for (unsigned int col = 0; col < num_columns_; ++col)
        {
            unsigned int style = (col < styles.size()) ? styles[col] : 0;

            ss << "<w:tc>";

            // cell properties (shading)
            auto shading = cellShadingXml(style);
            if (!shading.empty())
                ss << "<w:tcPr>" << shading << "</w:tcPr>";

            // cell content
            ss << "<w:p>";
            if (col < row_data.size())
                ss << cellRunXml(row_data[col], style);
            ss << "</w:p>";

            ss << "</w:tc>";
        }
        ss << "</w:tr>\n";
    }

    // truncation indicator
    if (rows_.size() > num_rows_to_write)
    {
        ss << "<w:tr>";
        for (unsigned int col = 0; col < num_columns_; ++col)
        {
            ss << "<w:tc><w:p><w:r>"
               << R"(<w:t xml:space="preserve">...</w:t>)"
               << "</w:r></w:p></w:tc>";
        }
        ss << "</w:tr>\n";
    }

    ss << "</w:tbl>\n";

    // return to portrait after wide table
    if (wide_table_)
    {
        ss << "<w:p><w:pPr><w:sectPr>"
           << R"(<w:pgSz w:w="11906" w:h="16838"/>)"
           << R"(<w:pgMar w:top="1134" w:right="1134" w:bottom="1134" w:left="1134" w:header="709" w:footer="709" w:gutter="0"/>)"
           << "</w:sectPr></w:pPr></w:p>\n";
    }

    return ss.str();
}
