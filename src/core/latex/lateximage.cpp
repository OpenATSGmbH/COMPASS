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

#include "lateximage.h"

#include "traced_assert.h"
#include <sstream>

using namespace std;

LatexImage::LatexImage(const std::string& filename, const std::string& caption)
    : filename_(filename), caption_(caption)
{
}

std::string LatexImage::toString()
{
    //assert (!content_.size());

    stringstream ss;

    ss << R"(\begin{figure}[H])" << "\n";
    //    \hspace*{-1cm}
    ss << R"(\includegraphics[width=18cm]{)" << filename_ << "}\n";
    ss << R"(\caption{)" << caption_ << "}\n";
    ss << R"(\end{figure})" << "\n";

    return ss.str();
}

std::string LatexImage::filename() const
{
    return filename_;
}
