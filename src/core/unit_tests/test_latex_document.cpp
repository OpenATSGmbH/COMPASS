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

#include "catch.hpp"
#include "latexdocument.h"

TEST_CASE("LatexDocument escapes title, author and licensee", "[latex]")
{
    LatexDocument doc("unused_path", "report.tex", "Licensed to A & B");
    doc.title("Radar En-Route & TMA Feichtberg Mode-S L1 Evaluation Report");
    doc.author("C_D");

    const std::string tex = doc.toString();

    REQUIRE(tex.find(R"(\title{Radar En-Route \& TMA Feichtberg Mode-S L1 Evaluation Report})") != std::string::npos);
    REQUIRE(tex.find(R"(\textbf{Radar En-Route \& TMA Feichtberg Mode-S L1 Evaluation Report})") != std::string::npos);
    REQUIRE(tex.find(R"(\author{C\_D})") != std::string::npos);
    REQUIRE(tex.find(R"(\fancyfoot[L]{Licensed to A \& B})") != std::string::npos);
}
