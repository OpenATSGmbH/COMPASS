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

#pragma once

#include <string>

/**
 * Shared pdflatex invocation and result classification. Used by both report
 * generators, the task result reports and the view points report, so that they
 * agree on what counts as a failed PDF run.
 */
namespace latex
{
    /**
     * Command running pdflatex on the given document, with its output filtered
     * down to the warning and error blocks.
     */
    std::string pdfLatexCommand(const std::string& path, const std::string& filename);

    /**
     * Whether the pdflatex output reports a genuine LaTeX error, meaning no PDF
     * was produced. pdflatex emits plenty of benign warnings (Underfull and
     * Overfull hbox, package "First Aid" notes, rerun requests) that the output
     * filter also captures; those must not abort a successfully generated PDF.
     */
    bool hasFatalError(const std::string& pdflatex_output);

    /**
     * Whether pdflatex asks for another run to resolve outlines or
     * cross-references.
     */
    bool needsRerun(const std::string& pdflatex_output);
}
