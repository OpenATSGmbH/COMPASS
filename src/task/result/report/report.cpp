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

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectionid.h"
#include "taskmanager.h"
#include "taskresult.h"
#include "taskresultswidget.h"

#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;

namespace ResultReport
{

const std::string Report::FieldRootSection = "root_section";

/**
 */
Report::Report(TaskResult* result)
:   TreeItem(SectionID::SectionReport, nullptr)
,   result_ (result)
{
    traced_assert(result_);

    root_section_ = std::make_shared<Section>(SectionID::SectionResults, "", this, this);
}

/**
 */
Report::~Report()
{
    logdbg;
}

/**
 */
const TaskManager& Report::taskManager() const
{
    return result_->taskManager();
}

/**
 */
TaskManager& Report::taskManager()
{
    return result_->taskManager();
}

/**
 */
void Report::clear()
{
    root_section_ = std::make_shared<Section>(SectionID::SectionResults, "", this, this);
}

/**
 */
void Report::updateContents()
{
    root_section_->updateContents(true);
}

/**
 */
TreeItem* Report::child(int row)
{
    if (row < 0 || row > 0)
        return nullptr;

    return root_section_.get();
}

/**
 */
int Report::childCount() const
{
    return 1;
}

/**
 */
int Report::columnCount() const
{
    return 1;
}

/**
 */
QVariant Report::data(int column) const
{
    traced_assert(column == 0);

    return name().c_str();
}

/**
 */
int Report::row() const
{
    return 0;
}

/**
 */
std::shared_ptr<Section> Report::rootSection()
{
    return root_section_;
}

/**
 */
std::vector<std::shared_ptr<Section>> Report::reportSections() const
{
    std::vector<std::shared_ptr<Section>> sections = { root_section_ };
    
    auto subs = root_section_->subSections(true);
    sections.insert(sections.end(), subs.begin(), subs.end());

    return sections;
}

/**
 */
std::vector<std::string> Report::getReportSectionIDs(ReportExportMode* export_mode) const
{
    auto sections = reportSections();

    std::vector<std::string> ids;

    for (const auto& s : sections)
        if (!export_mode || s->exportEnabled(*export_mode))
            ids.push_back(s->id());

    return ids;
}

/**
 */
std::vector<std::shared_ptr<SectionContent>> Report::reportContents(bool with_extra_content) const
{
    return root_section_->recursiveContent(with_extra_content);
}

/**
 */
bool Report::hasSection(const std::string& id) const
{
    if (id.empty())
        return false;

    loginf << "Checking for section '" << id << "'";

    std::string id_in = root_section_->relativeID(id);

    loginf << "Checking for relative section '" << id_in << "'";

    std::vector<std::string> parts = SectionID::subSections(id_in);
    if (parts.empty())
        return false;

    Section* tmp = nullptr;

    for (unsigned int cnt=0; cnt < parts.size(); ++cnt)
    {
        std::string& heading = parts.at(cnt);

        if (cnt == 0) // first
        {
            if (!root_section_->hasSubSection(heading))
                return false;

            tmp = &root_section_->getSubSection(heading);
        }
        else // previous section
        {
            traced_assert(tmp);

            if (!tmp->hasSubSection(heading))
                return false;

            tmp = &tmp->getSubSection(heading);
        }
    }

    return tmp != nullptr;
}

/**
 */
Section& Report::getSection (const std::string& id)
{
    logdbg << "id '" << id << "'";

    traced_assert(id.size());

    std::string id_in = root_section_->relativeID(id);

    std::vector<std::string> parts = SectionID::subSections(id_in);
    traced_assert(parts.size());

    Section* tmp = nullptr;

    for (unsigned int cnt=0; cnt < parts.size(); ++cnt)
    {
        std::string& heading = parts.at(cnt);

        if (cnt == 0) // first
        {
            if (!root_section_->hasSubSection(heading))
                root_section_->addSubSection(heading);

            tmp = &root_section_->getSubSection(heading);
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

const Section& Report::getSection (const std::string& id) const
{
    logdbg << "id '" << id << "'";

    traced_assert(id.size());

    std::string id_in = root_section_->relativeID(id);

    std::vector<std::string> parts = SectionID::subSections(id_in);
    traced_assert(parts.size());

    Section* tmp = nullptr;

    for (unsigned int cnt=0; cnt < parts.size(); ++cnt)
    {
        std::string& heading = parts.at(cnt);

        if (cnt == 0) // first
        {
            if (!root_section_->hasSubSection(heading))
                root_section_->addSubSection(heading);

            tmp = &root_section_->getSubSection(heading);
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

/**
 */
void Report::toJSON_impl(nlohmann::json& j) const
{
    j[ FieldRootSection ] = root_section_->toJSON();
}

/**
 */
bool Report::fromJSON_impl(const nlohmann::json& j)
{
    if (!j.is_object() ||
        !j.contains(FieldRootSection))
    {
        logerr << "report does not contain needed fields: isobject = " << j.is_object();
        return false;
    }

    if (!root_section_->fromJSON(j[ FieldRootSection ]))
    {
        logerr << "could not read root section";
        return false;
    }

    return true;
}

/**
 */
Result Report::toJSONDocument_impl(nlohmann::json& j,
                                   const std::string* resource_dir,
                                   ReportExportMode export_style) const
{
    //nothing to do yet

    return Result::succeeded();
}

/**
 */
void Report::setCurrentViewable(const nlohmann::json::object_t& data,
                                bool load_blocking)
{
    result_->taskManager().setViewableDataConfig(data, load_blocking);
}

/**
 */
void Report::unsetCurrentViewable()
{
    result_->taskManager().unsetViewableDataConfig();
}

/**
 */
void Report::setCurrentSection(const std::string& section_name, bool show_figure)
{
    std::string full_id = SectionID::prependReportResults(section_name);

    result_->taskManager().widget()->selectID(full_id, show_figure);
}

/**
 */
std::shared_ptr<ResultReport::SectionContent> Report::loadContent(ResultReport::Section* section, 
                                                                  unsigned int content_id,
                                                                  bool show_dialog) const
{
    return result_->taskManager().loadContent(section, content_id, show_dialog);
}

}
