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

#include "task/result/report/sectioncontenttext.h"
#include "task/result/report/reportexporter.h"
#include "task/result/taskresult.h"

#include "taskmanager.h"
#include "logger.h"

#include <QLabel>
#include <QVBoxLayout>

#include "traced_assert.h"

namespace ResultReport
{

const std::string SectionContentText::FieldTexts = "texts";

/**
 */
SectionContentText::SectionContentText(unsigned int id,
                                       const std::string& name, 
                                       Section* parent_section)
:   SectionContent(ContentType::Text, id, name, parent_section)
{
}

/**
 */
SectionContentText::SectionContentText(Section* parent_section)
:   SectionContent(ContentType::Text, parent_section)
{
}

/**
 */
void SectionContentText::addText(const std::string& text)
{
    texts_.push_back(text);
}

/**
 */
std::string SectionContentText::resourceExtension() const
{
    return ReportExporter::ExportTextFormat;
}

/**
 */
void SectionContentText::addContentUI(QVBoxLayout* layout, 
                                      bool force_ui_reset)
{
    traced_assert(layout);

    if (isLocked())
    {
        layout->addWidget(lockStatePlaceholderWidget());
    }
    else
    {
        for (auto& text : texts_)
        {
            QLabel* label = new QLabel((text+"\n\n").c_str());
            label->setWordWrap(true);

            layout->addWidget(label);
        }
    }
}

/**
 */
const std::vector<std::string>& SectionContentText::texts() const
{
    return texts_;
}

/**
 */
void SectionContentText::clearContent_impl()
{
    texts_.clear();
}

/**
 */
void SectionContentText::toJSON_impl(nlohmann::json& j) const
{
    //call base
    SectionContent::toJSON_impl(j);

    j[ FieldTexts ] = texts_;
}

/**
 */
bool SectionContentText::fromJSON_impl(const nlohmann::json& j)
{
    //call base
    if (!SectionContent::fromJSON_impl(j))
        return false;
    
    if (!j.is_object() ||
        !j.contains(FieldTexts))
    {
        logerr << "section content text does not obtain needed fields";
        return false;
    }

    texts_ = j[ FieldTexts ].get<std::vector<std::string>>();

    return true;
}

/**
 */
Result SectionContentText::toJSONDocument_impl(nlohmann::json& j, 
                                               const std::string* resource_dir,
                                               ReportExportMode export_style) const
{
    //call base
    auto r = SectionContent::toJSONDocument_impl(j, resource_dir, export_style);
    if (!r.ok())
        return r;

    j[ FieldTexts ] = texts_;

    return Result::succeeded();
}

}
