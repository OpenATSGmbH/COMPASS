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

#include "latexdocument.h"
#include "logger.h"
#include "files.h"
#include "stringconv.h"
#include "latexsection.h"
#include "compass.h"
#include "licensemanager.h"
#include "reportdefs.h"

#include <QDateTime>

#include <memory>

#include <fstream>
#include <sstream>

using namespace std;
using namespace Utils;

LatexDocument::LatexDocument(const std::string& path, const std::string& filename)
    : path_(path), filename_(filename)
{
    loginf << "path '" << path_ << "' filename '" << filename_ << "'";

    footer_left_  = COMPASS::instance().versionString(false, true);
    footer_right_ = COMPASS::instance().licenseeString(true);
}

void LatexDocument::write()
{
    loginf << "path '" << path_ << "' filename '" << filename_ << "'";

    bool ret = Files::createMissingDirectories(path_);

    if (!ret)
        throw runtime_error("LatexDocument: write: unable to create directories for '"+path_+"'");

    std::string path = path_ + "/" + filename_;

    loginf << "writing file to '" << path << "'";

    ofstream file (path);
    if (!file.is_open())
        throw runtime_error("LatexDocument: write: could not write tex file to '" + path + "'");

    file << toString();
    if (!file)
        throw runtime_error("LatexDocument: write: could not write tex file to '" + path + "'");

    file.close();

    loginf << "done";
}

std::string LatexDocument::toString()
{
    stringstream ss;

    auto color_defs = ResultReport::Colors::latexCustomColorDefines();

    ss << R"(\documentclass[twoside,a4paper]{report}
          \usepackage{geometry}
          \geometry{legalpaper, margin=1.5cm}

          \usepackage{fancyhdr}

          \usepackage{graphicx}
          \usepackage{float}
          \usepackage[export]{adjustbox}

          \usepackage{makeidx}
          \usepackage{ifthen}
          \usepackage{listings}
          \usepackage{hyperref}
          \usepackage{tabularx}
          \usepackage{ltablex}
          \usepackage{pdflscape}

          \usepackage{xcolor}
          \usepackage[table]{xcolor}
          \definecolor{lbcolor}{rgb}{0.9,0.9,0.9}
          \definecolor{darkgreen}{rgb}{0.0, 0.5, 0.13})";

    ss << "\n";

    //define additional report colors
    for (const auto& cd : color_defs)
        ss << "\t  " << cd << "\n";

    ss << R"(
          \usepackage{silence}

          \pagestyle{fancy}       % Set the page style to fancy
          \fancyhf{}              % Clear default header and footer

          % Define header and footer content
          \fancyhead[L]{}
          \fancyhead[C]{}
          \fancyhead[R]{}
          \fancyfoot[L]{)" << footer_left_ << R"(}
          \fancyfoot[C]{\thepage}
          \fancyfoot[R]{)" << footer_right_ << R"(}

          % No header line
          \renewcommand{\headrulewidth}{0pt}

          % Macro for landscape pages
          % Emulates a landscape page by swapping page width and heigth
          % This makes landscape pages work with fancyhdr
          \newcommand{\newvar}[2]{
            \newlength#1
            \setlength#1{#2}
          }

          \newvar{\initialPaperWidth}{\paperwidth}
          \newvar{\initialPaperHeight}{\paperheight}
          \newvar{\initialTextWidth}{\textwidth}
          \newvar{\initialTextHeight}{\textheight}

          \makeatletter

          \newenvironment{landscape2}{%%%
            \clearpage
            \paperwidth=\initialPaperHeight
            \paperheight=\initialPaperWidth
            \pdfpagewidth=\initialPaperHeight
            \pdfpageheight=\initialPaperWidth
            \textwidth=\initialTextHeight
            \textheight=\initialTextWidth
            \fancyheadoffset{0pt}%%%
            \fancyfootoffset{0pt}%%%
            \fancyhfoffset{0pt}%%%
            \pagestyle{fancy}%%%
            \vsize=\textheight
            \@colroom=\vsize
            \@colht\vsize
            \hsize=\textwidth
            \clearpage
          }{%%%
            \clearpage
            \textwidth=\initialTextWidth
            \textheight=\initialTextHeight
            \global\pdfpagewidth=\initialPaperWidth
            \global\pdfpageheight=\initialPaperHeight
            \global\@colht=\textheight
            \global\vsize=\textheight
            \global\@colroom=\textheight
            \pagestyle{fancy}%%%
            \clearpage
          }

          \makeatother

          \WarningFilter{latex}{Hyper reference}
          \WarningFilter{latex}{There were undefined references}

          \lstset{
            basicstyle=\ttfamily,
            columns=fullflexible,
            %frame=single,
            breaklines=true,
            backgroundcolor=\color{lbcolor},
            %postbreak=\mbox{\textcolor{red}{$\hookrightarrow$}\space},
            postbreak=\mbox{$\hookrightarrow$\space},
          }

          \def\bs{\char'134 } % backslash in \tt font.
          \newcommand{\ie}{i.\,e.,}
          \newcommand{\eg}{e.\,g..}
          \DeclareRobustCommand\cs[1]{\texttt{\char`\\#1}})" << "\n";

    if (title_.size())
        ss << R"(\title{)" << title_ << "}"  << "\n";

    if (author_.size())
        ss << R"(\author{)" << author_ << "}"  << "\n";

    ss << R"(\date{)" << QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss").toStdString() << "}\n";

    ss << R"(\emergencystretch1em  %

          \makeindex

          \setcounter{tocdepth}{4}

          \begin{document})" << "\n";

    if (title_.size())
        ss << R"(\maketitle)" << "\n";

    if (abstract_.size())
        ss << R"(\begin{abstract})" << "\n" << abstract_ << "\n" << R"(\end{abstract}
                                                            \ \\)"  << "\n";

    ss << R"(\newpage

          \tableofcontents

          \newpage)" << "\n";

    ss << LatexContent::toString() << "\n";

    ss << R"(\printindex

    \end{document})" << "\n";

    return ss.str();
}

LatexSection& LatexDocument::getSection (const std::string& id)
{
    traced_assert(id.size());
    std::vector<std::string> parts = String::split(id, ':');
    traced_assert(parts.size());

    loginf << "id '"+id+"' parts " << parts.size();

    //std::string& top = parts.at(0);

    LatexSection* tmp = nullptr;

    for (unsigned int cnt=0; cnt < parts.size(); ++cnt)
    {
        std::string& heading = parts.at(cnt);

        if (cnt == 0) // first
        {
            if (!hasSubSection(heading))
                addSubSection(heading);

            tmp = &getSubSection(heading);
        }
        else // previous section
        {
            traced_assert(tmp);

            if (!tmp->hasSubSection(heading))
                tmp->addSubSection(heading);

            tmp = &tmp->getSubSection(heading);
        }
    }

    traced_assert(tmp);
    return *tmp;
}

bool LatexDocument::hasSubSection (const std::string& heading)
{
    return findSubSection(heading) != nullptr;
}

LatexSection& LatexDocument::getSubSection (const std::string& heading)
{
    LatexSection* tmp = findSubSection (heading);
    traced_assert(tmp);
    return *tmp;
}

void LatexDocument::addSubSection (const std::string& heading)
{
    traced_assert(!hasSubSection(heading));
    sub_content_.push_back(unique_ptr<LatexSection>(new LatexSection(LatexSectionLevel::SECTION, heading)));
    traced_assert(hasSubSection(heading));
}

std::string LatexDocument::path() const
{
    return path_;
}

std::string LatexDocument::filename() const
{
    return filename_;
}

std::string LatexDocument::title() const
{
    return title_;
}

void LatexDocument::title(const std::string& title)
{
    title_ = title;
}

std::string LatexDocument::author() const
{
    return author_;
}

void LatexDocument::author(const std::string& author)
{
    author_ = author;
}

std::string LatexDocument::abstract() const
{
    return abstract_;
}

void LatexDocument::abstract(const std::string& abstract)
{
    abstract_ = abstract;
}
