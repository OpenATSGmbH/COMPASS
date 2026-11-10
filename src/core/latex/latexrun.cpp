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
#include "logger.h"

#include <QProcess>
#include <QString>
#include <QStringList>

namespace latex
{

/**
 * Started directly, without a shell (no cd/awk on Windows). The output filter keeps the
 * blocks from a line with "warning" (any case) or "!" up to the next empty line, as the
 * former awk filter /warning|!/,/^$/ did.
 */
std::string runPDFLatex(const std::string& path, const std::string& filename)
{
    QProcess process;
    process.setWorkingDirectory(QString::fromStdString(path));
    process.setProcessChannelMode(QProcess::MergedChannels);
    process.start("pdflatex", { "--interaction=nonstopmode", QString::fromStdString(filename) });

    if (!process.waitForStarted())
    {
        // "Fatal error": hasFatalError() fails the export
        const std::string error = "Fatal error: could not start pdflatex: "
                                  + process.errorString().toStdString();
        logerr << error;
        return error + "\n";
    }

    process.waitForFinished(-1);

    const QStringList lines = QString::fromLocal8Bit(process.readAll()).split('\n');

    std::string filtered;
    bool in_block = false;

    for (QString line : lines)
    {
        if (line.endsWith('\r'))
            line.chop(1);

        if (!in_block && (line.contains("warning", Qt::CaseInsensitive) || line.contains('!')))
            in_block = true;

        if (in_block)
        {
            filtered += line.toStdString() + "\n";

            if (line.isEmpty())
                in_block = false;
        }
    }

    return filtered;
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
