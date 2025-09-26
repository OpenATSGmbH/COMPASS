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

#include "task/result/report/sectionid.h"

#include "stringconv.h"
#include "traced_assert.h"

#include <QString>
#include <boost/algorithm/string/replace.hpp>

namespace ResultReport
{

const std::string SectionID::Sep             = ":";
const std::string SectionID::SectionReport   = "Report";
const std::string SectionID::SectionResults  = "Results";
const std::string SectionID::SectionOverview = "Overview";

/**
*/
std::vector<std::string> SectionID::subSections(const std::string& section_id) 
{
    return Utils::String::split(section_id, Sep[ 0 ]);
}

/**
*/
std::string SectionID::sectionID(const std::vector<std::string>& sub_sections)
{
    return Utils::String::compress(sub_sections, Sep[ 0 ]);
}

/**
*/
std::string SectionID::sectionID(const std::string& section0, const std::string& section1)
{
    traced_assert(!section0.empty() && !section1.empty());

    return (section0 + Sep + section1);
}

/**
*/
std::string SectionID::sectionID2Path(const std::string& section_id)
{
    std::string p = section_id;

    boost::replace_all(p, ":", "/");
    boost::replace_all(p, " ", "_");

    return (p + "/");
}

/**
*/
std::string SectionID::prependReportResults(const std::string& section_id)
{
    const std::string prefix = SectionID::reportResultID() + Sep;

    //already prepended?
    if (QString::fromStdString(section_id).startsWith(QString::fromStdString(prefix)))
        return section_id;

    return prefix + section_id;
}

/**
*/
std::string SectionID::sectionIDWithoutResults(const std::string& section_id)
{
    //root?
    if (section_id == SectionResults)
        return "";

    const std::string ResultsHeader = SectionResults + Sep;

    //valid?
    traced_assert(section_id.rfind(ResultsHeader, 0) == 0);

    //chop result header
    std::string ret = section_id;
    ret.erase(0,ResultsHeader.size());

    return ret;
}

/**
*/
std::string SectionID::reportResultID()
{
    return SectionReport + Sep + SectionResults;
}

/**
*/
std::string SectionID::reportResultOverviewID()
{
    return SectionID::reportResultID() + Sep + SectionOverview;
}

}
