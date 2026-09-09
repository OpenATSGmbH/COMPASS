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

#include "latexrun.h"

namespace latex
{

/**
 */
std::string pdfLatexCommand(const std::string& path, const std::string& filename)
{
    return "cd \"" + path + "\" && pdflatex --interaction=nonstopmode \"" + filename
            + "\" | awk 'BEGIN{IGNORECASE = 1}/warning|!/,/^$/;'";
}

/**
 */
bool hasFatalError(const std::string& pdflatex_output)
{
    return pdflatex_output.find("! LaTeX Error")    != std::string::npos
        || pdflatex_output.find("! Emergency stop") != std::string::npos
        || pdflatex_output.find("Fatal error")      != std::string::npos;
}

/**
 */
bool needsRerun(const std::string& pdflatex_output)
{
    return pdflatex_output.find("Rerun to get outlines right")         != std::string::npos
        || pdflatex_output.find("Rerun to get cross-references right") != std::string::npos;
}

}
