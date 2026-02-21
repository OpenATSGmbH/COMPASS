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

#include "docxtext.h"

#include <sstream>

DocxText::DocxText(const std::string& text)
:   text_(text)
{
}

std::string DocxText::toXml()
{
    std::stringstream ss;

    // escape XML special characters
    std::string escaped;
    escaped.reserve(text_.size());
    for (char c : text_)
    {
        switch (c)
        {
            case '&':  escaped += "&amp;";  break;
            case '<':  escaped += "&lt;";   break;
            case '>':  escaped += "&gt;";   break;
            case '"':  escaped += "&quot;";  break;
            case '\'': escaped += "&apos;"; break;
            default:   escaped += c;        break;
        }
    }

    ss << R"(<w:p><w:r><w:t xml:space="preserve">)" << escaped << "</w:t></w:r></w:p>\n";

    return ss.str();
}
