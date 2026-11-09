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

namespace ResultReport
{

/**
 * Functionality for creating unique result section ids of various types.
 */
class SectionID 
{
public:
    //tools
    static std::vector<std::string> subSections(const std::string& section_id);
    static std::string sectionID(const std::vector<std::string>& sub_sections);
    static std::string sectionID(const std::string& section0, const std::string& section1);
    static std::string sectionID2Path(const std::string& section_id);
    static std::string prependReportResults(const std::string& section_id);
    static std::string sectionIDWithoutResults(const std::string& section_id);

    static std::string reportResultID();
    static std::string reportResultOverviewID();

    static const std::string Sep;
    static const std::string SectionReport;
    static const std::string SectionResults;
    static const std::string SectionOverview;
};

}
