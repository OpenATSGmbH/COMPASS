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

// latex::runPDFLatex: pdflatex started without a shell (also on Windows), output filter and
// error classification. Skipped where pdflatex is not installed.

#include "catch.hpp"
#include "latexrun.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QTemporaryDir>

#include <string>

namespace
{
// writes the document into the folder, returns its file name
std::string writeDocument(const QTemporaryDir& dir, const std::string& name, const std::string& body)
{
    QFile file(dir.filePath(QString::fromStdString(name)));
    REQUIRE(file.open(QIODevice::WriteOnly));
    file.write(body.c_str());
    file.close();

    return name;
}
}

TEST_CASE("latex::runPDFLatex runs pdflatex and classifies the output", "[report][latex]")
{
    if (QStandardPaths::findExecutable("pdflatex").isEmpty())
    {
        WARN("pdflatex not installed, test skipped");
        return;
    }

    QTemporaryDir dir;
    REQUIRE(dir.isValid());
    const std::string path = dir.path().toStdString();

    SECTION("valid document: PDF written, no fatal error")
    {
        const std::string fn = writeDocument(dir, "valid.tex",
            "\\documentclass{article}\n\\begin{document}\nCOMPASS test\n\\end{document}\n");

        const std::string out = latex::runPDFLatex(path, fn);

        CHECK_FALSE(latex::hasFatalError(out));
        CHECK(QFileInfo::exists(dir.filePath("valid.pdf")));
    }

    SECTION("missing package: fatal error in the filtered output")
    {
        const std::string fn = writeDocument(dir, "missing_package.tex",
            "\\documentclass{article}\n\\usepackage{compassnonexistingpackage}\n"
            "\\begin{document}\nx\n\\end{document}\n");

        const std::string out = latex::runPDFLatex(path, fn);

        CHECK(latex::hasFatalError(out));
        CHECK(out.find("compassnonexistingpackage") != std::string::npos);
        CHECK_FALSE(QFileInfo::exists(dir.filePath("missing_package.pdf")));
    }
}
