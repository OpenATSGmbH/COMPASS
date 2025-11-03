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

#include "task/result/report/reportitem.h"
#include "task/result/report/reportdefs.h"

#include "logger.h"
#include "json_fwd.hpp"

#include <string>

#include "property.h"
#include "propertylist.h"

#include <boost/optional.hpp>

class QVBoxLayout;
class QWidget;

class TaskResult;

namespace ResultReport
{

class Section;
class Report;

/**
 */
struct SectionContentViewable
{
    typedef std::function<std::shared_ptr<nlohmann::json::object_t>()> ViewableFunc;

    SectionContentViewable() {}
    SectionContentViewable(const ViewableFunc& func) 
    :   viewable_func(func) {}
    SectionContentViewable(const nlohmann::json::object_t& content)
    {
        setCallback(content);
    }

    bool hasCallback() const { return viewable_func ? true : false; }

    SectionContentViewable& setCallback(const ViewableFunc& func)
    {
        viewable_func = func;
        return *this;
    }

    SectionContentViewable& setCallback(const nlohmann::json::object_t& content)
    {
        std::shared_ptr<nlohmann::json::object_t> c(new nlohmann::json::object_t);
        *c = content;
        viewable_func = [ = ] () { return c; };

        return *this;
    }

    SectionContentViewable& setCaption(const std::string& c)
    {
        caption = c;
        return *this;
    }

    SectionContentViewable& setRenderDelayMS(int delay)
    {
        render_delay_msec = delay;
        return *this;
    }

    SectionContentViewable& setOnDemand()
    {
        on_demand = true;
        return *this;
    }

    ViewableFunc viewable_func;
    std::string  caption;
    int          render_delay_msec = 0;
    bool         on_demand         = false;
};

/**
 */
class SectionContent : public ReportItem
{
public:
    typedef SectionContentType ContentType;

    SectionContent(ContentType type,
                   unsigned int id,
                   const std::string& name, 
                   Section* parent_section);
    SectionContent(ContentType type,
                   Section* parent_section);

    ContentType contentType() const;
    std::string contentTypeAsString() const;
    unsigned int contentID() const;

    Section* parentSection();
    const Section* parentSection() const;

    std::string contentPath() const;

    std::string resourceRelDirectory(ResourceDir rdir) const;
    std::string resourceFilename(const std::string& postfix = "") const;
    std::string resourceLink(ResourceDir rdir, const std::string& postfix = "") const;

    virtual std::string resourceExtension() const = 0;

    void setLockStateSafe();
    bool isLockStateSafe() const;
    bool isLocked() const;

    void setOnDemand();
    bool isOnDemand() const;
    bool isLoading() const;
    bool isComplete() const;
    bool loadOnDemandIfNeeded();
    bool forceReload();

    void clearOnDemandContent();
    
    virtual void addContentUI(QVBoxLayout* layout, 
                              bool force_ui_reset) = 0;

    static std::string contentTypeAsString(ContentType type);
    static boost::optional<ContentType> contentTypeFromString(const std::string& type_str);

    static const std::string  DBTableName;
    static const Property     DBColumnContentID;
    static const Property     DBColumnResultID;
    static const Property     DBColumnType;
    static const Property     DBColumnJSONContent;
    static const PropertyList DBPropertyList;

    static const std::string FieldContentType;
    static const std::string FieldContentID;
    static const std::string FieldOnDemand;
    static const std::string FieldLockStateSafe;

protected:
    struct ResourceLink
    {
        std::string link;
        std::string path;
    };

    ResultT<ResourceLink> prepareResource(const std::string& resource_dir,
                                          ResourceDir rdir,
                                          const std::string& prefix = "") const;
    virtual void clearContent_impl() = 0;

    virtual void toJSON_impl(nlohmann::json& j) const override;
    virtual bool fromJSON_impl(const nlohmann::json& j) override;
    Result toJSONDocument_impl(nlohmann::json& j,
                               const std::string* resource_dir,
                               ReportExportMode export_style) const override;

    virtual bool loadOnDemand();

    TaskResult* taskResult();
    const TaskResult* taskResult() const;

    QWidget* lockStatePlaceholderWidget() const;

    ContentType  content_type_;
    unsigned int content_id_     = 0;
    Report*      report_         = nullptr;

private:
    void clearContent();

    bool on_demand_       = false;
    bool loading_         = false;
    bool complete_        = false;
    bool lock_state_safe_ = false;
};

}
