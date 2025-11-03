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

#include "task/result/report/sectioncontent.h"

#include <vector>
#include <string>

class TaskManager;

namespace ResultReport
{

class Section;

/**
 */
class SectionContentText : public SectionContent
{
public:
    SectionContentText(unsigned int id, 
                       const std::string& name, 
                       Section* parent_section);
    SectionContentText(Section* parent_section);

    void addText (const std::string& text);

    virtual std::string resourceExtension() const override;

    virtual void addContentUI(QVBoxLayout* layout,
                              bool force_ui_reset) override;

    const std::vector<std::string>& texts() const;

    static const std::string FieldTexts;

protected:
    void clearContent_impl() override final;

    void toJSON_impl(nlohmann::json& j) const override final;
    bool fromJSON_impl(const nlohmann::json& j) override final;
    Result toJSONDocument_impl(nlohmann::json& j,
                               const std::string* resource_dir,
                               ReportExportMode export_style) const override final;

    std::vector<std::string> texts_;
};

}
