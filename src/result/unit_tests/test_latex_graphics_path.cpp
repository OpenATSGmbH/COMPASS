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

// LatexDocument::graphicsPath: paths of images in a LaTeX document are relative to the
// document folder (pdflatex runs there) and use '/'. A backslash starts a LaTeX command,
// so a Windows path like icons\done.png would include icons.png.

#include "catch.hpp"
#include "latexdocument.h"

TEST_CASE("LaTeX graphics path relative to the document folder", "[latex]")
{
    SECTION("file inside the document folder")
    {
        REQUIRE(LatexDocument::graphicsPath("/data/report", "/data/report/screenshots/Sector Overview.png")
                == "screenshots/Sector Overview.png");
    }

    SECTION("document folder with trailing separator")
    {
        REQUIRE(LatexDocument::graphicsPath("/data/report/", "/data/report/screenshots/a.jpg")
                == "screenshots/a.jpg");
    }

    SECTION("file outside the document folder stays absolute")
    {
        REQUIRE(LatexDocument::graphicsPath("/data/report", "/data/other/a.png") == "/data/other/a.png");
    }

    SECTION("relative path stays relative")
    {
        REQUIRE(LatexDocument::graphicsPath("/data/report", "icons/done.png") == "icons/done.png");
    }
}

#ifdef _WIN32
TEST_CASE("LaTeX graphics path on Windows uses forward slashes", "[latex]")
{
    SECTION("mixed separators inside the document folder")
    {
        REQUIRE(LatexDocument::graphicsPath("C:\\data\\report",
                                            "C:\\data\\report/screenshots\\Sectors/a.png")
                == "screenshots/Sectors/a.png");
    }

    SECTION("document folder with trailing backslash")
    {
        REQUIRE(LatexDocument::graphicsPath("C:\\data\\report\\", "C:\\data\\report\\icons\\done.png")
                == "icons/done.png");
    }

    SECTION("relative path with backslash")
    {
        REQUIRE(LatexDocument::graphicsPath("C:\\data\\report", "icons\\done.png") == "icons/done.png");
    }

    SECTION("file on another drive stays absolute")
    {
        REQUIRE(LatexDocument::graphicsPath("C:\\data\\report", "D:\\images\\a.png") == "D:/images/a.png");
    }
}
#endif
