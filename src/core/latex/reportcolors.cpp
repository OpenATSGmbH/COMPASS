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

#include "reportcolors.h"

namespace ResultReport
{

/**
 */
std::vector<std::string> Colors::latexCustomColorDefines()
{
    std::vector<std::string> defs;

    auto addDefine = [ & ] (const std::string& name, const QColor& color)
    {
        std::string d = "\\definecolor{" + name + "}{rgb}{" +
                        QString::number(color.redF()  , 'f', 2).toStdString() + "," +
                        QString::number(color.greenF(), 'f', 2).toStdString() + "," +
                        QString::number(color.blueF() , 'f', 2).toStdString() + "}";
        defs.push_back(d);
    };

    addDefine(TextLatexRed   , TextRed   );
    addDefine(TextLatexOrange, TextOrange);
    addDefine(TextLatexGreen , TextGreen );
    addDefine(TextLatexGray  , TextGray  );

    addDefine(BGLatexRed     , BGRed     );
    addDefine(BGLatexOrange  , BGOrange  );
    addDefine(BGLatexGreen   , BGGreen   );
    addDefine(BGLatexGray    , BGGray    );
    addDefine(BGLatexYellow  , BGYellow  );

    return defs;
}

}
