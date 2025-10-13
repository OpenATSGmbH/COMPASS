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

#include "task/result/report/sectioncontent.h"
#include "task/result/report/section.h"
#include "task/result/report/sectionid.h"
#include "task/result/report/report.h"
#include "task/result/report/reportexporter.h"

#include "taskresult.h"

#include "files.h"

#include "traced_assert.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QFrame>

namespace ResultReport
{

const std::string  SectionContent::DBTableName         = "report_viewables";
const Property     SectionContent::DBColumnContentID   = Property("content_id"  , PropertyDataType::UINT);
const Property     SectionContent::DBColumnResultID    = Property("result_id"   , PropertyDataType::UINT);
const Property     SectionContent::DBColumnType        = Property("type"        , PropertyDataType::INT );
const Property     SectionContent::DBColumnJSONContent = Property("json_content", PropertyDataType::JSON);
const PropertyList SectionContent::DBPropertyList      = PropertyList({ SectionContent::DBColumnContentID,
                                                                        SectionContent::DBColumnResultID,
                                                                        SectionContent::DBColumnType,
                                                                        SectionContent::DBColumnJSONContent });
const std::string SectionContent::FieldContentType   = "content_type";
const std::string SectionContent::FieldContentID     = "content_id";
const std::string SectionContent::FieldOnDemand      = "on_demand";
const std::string SectionContent::FieldLockStateSafe = "lock_state_safe";

/**
 */
SectionContent::SectionContent(ContentType type,
                               unsigned int id,
                               const std::string& name, 
                               Section* parent_section)
:   ReportItem     (name, parent_section)
,   content_type_  (type                )
,   content_id_    (id                  )
{
    traced_assert(parent_section);

    report_ = parent_section->report();
    traced_assert(report_);
}

/**
 */
SectionContent::SectionContent(ContentType type,
                               Section* parent_section)
:   ReportItem     (parent_section)
,   content_type_  (type          )
{
    traced_assert(parent_section);

    report_ = parent_section->report();
    traced_assert(report_);
}

/**
 */
Section* SectionContent::parentSection()
{
    return dynamic_cast<Section*>(parent_item_);
}

/**
 */
const Section* SectionContent::parentSection() const
{
    return dynamic_cast<const Section*>(parent_item_);
}

/**
 */
std::string SectionContent::contentPath() const
{
    return ResultReport::SectionID::sectionID2Path(parentSection()->compoundResultsHeading());
}

/**
 * Filename of the content in the resource filesystem.
 */
std::string SectionContent::resourceFilename(const std::string& postfix) const
{
    auto pf = (postfix.empty() ? "" : "_") + postfix;
    auto fn = name() + pf + resourceExtension();

    return fn;
}

/**
 * Relative directory of the content in the resource filesystem.
 */
std::string SectionContent::resourceRelDirectory(ResourceDir rdir) const
{
    return Utils::Files::join(ReportExporter::resourceSubDir(rdir), contentPath());
}

/**
 * Relative path of the content in the resource filesystem.
 */
std::string SectionContent::resourceLink(ResourceDir rdir, const std::string& postfix) const
{
    return resourceRelDirectory(rdir) + resourceFilename(postfix);
}

/**
 */
ResultT<SectionContent::ResourceLink> SectionContent::prepareResource(const std::string& resource_dir,
                                                                      ResourceDir rdir,
                                                                      const std::string& prefix) const
{
    //store data in temp dir
    auto link = resourceLink(rdir, prefix);
    auto path = Utils::Files::join(resource_dir, link);
    auto dir  = Utils::Files::getDirectoryFromPath(path);

    if (!Utils::Files::createMissingDirectories(dir))
        return ResultT<ResourceLink>::failed("Could not create resource subdirectory for content '" + name() + "'");

    ResourceLink rl;
    rl.link = link;
    rl.path = path;

    return ResultT<ResourceLink>::succeeded(rl);
}

/**
 */
std::string SectionContent::contentTypeAsString(ContentType type)
{
    switch(type)
    {
        case ContentType::Figure:
            return "figure";
        case ContentType::Table:
            return "table";
        case ContentType::Text:
            return "text";
        default:
            return "";
    }
    return "";
}

/**
 */
boost::optional<SectionContent::ContentType> SectionContent::contentTypeFromString(const std::string& type_str)
{
    if (type_str == "figure")
        return ContentType::Figure;
    else if (type_str == "table")
        return ContentType::Table;
    else if (type_str == "text")
        return ContentType::Text;

    return boost::optional<SectionContent::ContentType>();
}

/**
 */
SectionContent::ContentType SectionContent::contentType() const
{
    return content_type_;
}

/**
 */
std::string SectionContent::contentTypeAsString() const
{
    return SectionContent::contentTypeAsString(content_type_);
}

/**
 */
unsigned int SectionContent::contentID() const
{
    return content_id_;
}

/**
 */
TaskResult* SectionContent::taskResult()
{
    return &parentSection()->report()->result();
}

/**
 */
const TaskResult* SectionContent::taskResult() const
{
    return &parentSection()->report()->result();
}

/**
 */
void SectionContent::setLockStateSafe()
{
    lock_state_safe_ = true;
}

/**
 */
bool SectionContent::isLockStateSafe() const
{
    return lock_state_safe_;
}

/**
 */
bool SectionContent::isLocked() const
{
    return isOnDemand() && 
           !isLockStateSafe() &&
           taskResult()->isLocked();
}

/**
 */
void SectionContent::setOnDemand()
{
    on_demand_ = true;
}

/**
 */
bool SectionContent::isOnDemand() const
{
    return on_demand_;
}

/**
 */
bool SectionContent::isLoading() const
{
    return loading_;
}

/**
 */
bool SectionContent::isComplete() const
{
    return complete_;
}

/**
 */
bool SectionContent::loadOnDemandIfNeeded()
{
    if (!isOnDemand() || isComplete())
        return true;

    return loadOnDemand();
}

/**
 */
bool SectionContent::loadOnDemand()
{
    loading_ = true;

    //if the task result is locked we should never give an opportunity to load on demand content
    traced_assert(!isLocked());

    loginf << "loading on-demand data for content '" << name() << "' of type '" << contentTypeAsString() << "'";

    traced_assert(isOnDemand());
    traced_assert(!isComplete());
    traced_assert(report_);

    bool ok = report_->result().loadOnDemandContent(this);

    complete_ = ok;
    loading_  = false;

    if (!ok)
    {
        logerr << "could not load on-demand data for content '" << name() << "' of type '" << contentTypeAsString() << "'";
        return false;
    }

    return true;
}

/**
 */
bool SectionContent::forceReload()
{
    if (!isOnDemand())
        return true;

    clearContent();
    
    return loadOnDemand();
}

/**
 */
void SectionContent::clearOnDemandContent()
{
    //reset already loaded on-demand content
    if (isOnDemand() && isComplete())
        clearContent();
}

/**
 */
void SectionContent::clearContent()
{
    clearContent_impl();

    complete_ = false;
}

/**
 */
QWidget* SectionContent::lockStatePlaceholderWidget() const
{
    QFrame*      w      = new QFrame;
    QVBoxLayout* layout = new QVBoxLayout;

    w->setLayout(layout);
    w->setFrameStyle(QFrame::Box | QFrame::Plain);
    w->setLineWidth(1);

    auto txt = QString::fromStdString(contentTypeAsString(contentType()));
    txt[ 0 ] = txt[ 0 ].toUpper();

    QLabel* label = new QLabel(txt + " is locked. Refresh to show on-demand content.");
    layout->addWidget(label);

    return w;
}

/**
 */
void SectionContent::toJSON_impl(nlohmann::json& j) const
{
    j[ FieldContentType   ] = contentTypeAsString(content_type_);
    j[ FieldContentID     ] = content_id_;
    j[ FieldOnDemand      ] = on_demand_;
    j[ FieldLockStateSafe ] = lock_state_safe_;
}

/**
 */
bool SectionContent::fromJSON_impl(const nlohmann::json& j)
{
    if (!j.is_object()                ||
        !j.contains(FieldContentType) ||
        !j.contains(FieldContentID)   ||
        !j.contains(FieldOnDemand))
    {
        logerr << "section content does not obtain needed fields";
        return false;
    }

    if (j.contains(FieldLockStateSafe))
        lock_state_safe_ = j[ FieldLockStateSafe ];

    std::string t_str = j[ FieldContentType ];
    auto t = contentTypeFromString(t_str);
    if (!t.has_value())
    {
        logerr << "could not deduce section content type";
        return false;
    }

    content_type_  = t.value();
    content_id_    = j[ FieldContentID ];
    on_demand_     = j[ FieldOnDemand  ];

    return true;
}

/**
 */
Result SectionContent::toJSONDocument_impl(nlohmann::json& j,
                                           const std::string* resource_dir,
                                           ReportExportMode export_style) const
{
    j[ FieldContentType ] = contentTypeAsString(content_type_);

    return Result::succeeded();
}

}
