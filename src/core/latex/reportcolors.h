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
#include <vector>

#include <QColor>

namespace ResultReport
{
    /**
     * Members are defined inline to avoid static initialization order issues
     * across translation units (e.g. sectioncontenttable.cpp referencing these).
     */
    class Colors
    {
    public:
        static inline const QColor TextRed    = QColor(220,20,60);
        static inline const QColor TextOrange = QColor(255,140,0);
        static inline const QColor TextGreen  = QColor(0,128,0);
        static inline const QColor TextGray   = Qt::darkGray;

        static inline const QColor BGRed      = QColor(240,128,128);
        static inline const QColor BGOrange   = QColor(255,165,0);
        static inline const QColor BGGreen    = QColor(144,238,144);
        static inline const QColor BGGray     = Qt::lightGray;
        static inline const QColor BGYellow   = QColor(255,255,153);

        static inline const std::string TextLatexRed    = "celldarkred";
        static inline const std::string TextLatexOrange = "celldarkorange";
        static inline const std::string TextLatexGreen  = "celldarkgreen";
        static inline const std::string TextLatexGray   = "celldarkgray";

        static inline const std::string BGLatexRed      = "celllightred";
        static inline const std::string BGLatexOrange   = "celllightorange";
        static inline const std::string BGLatexGreen    = "celllightgreen";
        static inline const std::string BGLatexGray     = "celllightgray";
        static inline const std::string BGLatexYellow   = "celllightyellow";

        static std::vector<std::string> latexCustomColorDefines();
    };
}
