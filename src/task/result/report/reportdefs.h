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

#include <limits>
#include <string>
#include <vector>

#include <QColor>

namespace ResultReport
{
    enum CellStyle
    {
        CellStyleCheckable         = 1 << 0,  // cell obtains an icon coded into the cells bool value
        CellStyleIcon              = 1 << 1,  // cell is checkable, its check state coded into the cells string value
        CellStyleInactive          = 1 << 2,  // cell is inactive

        CellStyleTextBold          = 1 << 5,  // cell obtains bold text
        CellStyleTextItalic        = 1 << 6,  // cell obtains italic text
        CellStyleTextStrikeOut     = 1 << 7,  // cell text is strike-out

        CellStyleTextColorRed      = 1 << 10,
        CellStyleTextColorOrange   = 1 << 11,
        CellStyleTextColorGreen    = 1 << 12,
        CellStyleTextColorGray     = 1 << 13,

        CellStyleBGColorRed        = 1 << 20,
        CellStyleBGColorOrange     = 1 << 21,
        CellStyleBGColorGreen      = 1 << 22,
        CellStyleBGColorGray       = 1 << 23,
        CellStyleBGColorYellow     = 1 << 24
    };

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

    enum class ReportExportMode
    {
        JSONFile = 0,
        JSONBlob,
        Latex,
        LatexPDF,
        DOCX,
        CSV //@TODO: to be fully implemented
    };

    extern ReportExportMode reportExportModeFromString(const std::string& str);
    extern std::string reportExportMode2String(ReportExportMode mode);
    extern std::string reportExportMode2Extension(ReportExportMode mode);
    //extern std::string reportExportMode2Folder(ReportExportMode mode);

    enum class ResourceDir
    {
        Root = 0,
        Screenshots,
        Tables,
        Icons
    };

    struct ReportExportSettings
    {
        //general settings
        std::string      author;
        std::string      abstract;

        bool             open_created_file = true;

        //latex settings
        int              latex_table_max_rows      = 500;
        int              latex_table_max_col_width = 24;
        int              latex_table_min_cols_wide = 9;
        unsigned int     latex_pdf_max_reruns      = 3;

        //json settings
        int              json_table_max_rows_inline = 0;
        int              json_table_max_cols_inline = 0;
    };

    enum class SectionContentType
    {
        Figure = 0,
        Table,
        Text
    };

    struct TableColumnGroup
    {
        std::string      name;
        std::vector<int> columns;
        bool             enabled_on_init = true;
        bool             enabled         = true;
    };

    struct TableHeader
    {
        std::vector<std::string>      headings;
        std::vector<TableColumnGroup> column_groups;
    };
}
