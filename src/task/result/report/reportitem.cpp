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

#include "task/result/report/reportitem.h"
#include "task/result/report/sectionid.h"

#include "logger.h"

namespace ResultReport
{

const std::string ReportItem::FieldName        = "name";
const std::string ReportItem::FieldID          = "id";
const std::string ReportItem::FieldProperties  = "properties";
const std::string ReportItem::FieldExportFlags = "export_flags";

/**
*/
ReportItem::ReportItem(const std::string& name, 
                       ReportItem* parent_item)
:   parent_item_(parent_item)
,   name_       (name       )
{
    export_flags_.set();

    updateID();
}

/**
*/
ReportItem::ReportItem(ReportItem* parent_item)
:   parent_item_(parent_item)
{
    export_flags_.set();
}

/**
*/
ReportItem::~ReportItem() = default;

/**
 */
ReportItem* ReportItem::parentItem()
{
    return parent_item_;
}

/**
 */
const ReportItem* ReportItem::parentItem() const
{
    return parent_item_;
}

/**
 */
const std::string& ReportItem::name() const
{
    return name_;
}

/**
*/
const std::string& ReportItem::id() const
{
    return id_;
}

/**
 */
void ReportItem::updateID()
{
    if (parent_item_)
        id_ = parent_item_->id() + SectionID::Sep + name_;
    else
        id_ = name_;
}

/**
*/
bool ReportItem::exportEnabled(ReportExportMode mode) const
{
    return export_flags_[ (size_t)mode ];
}

/**
*/
void ReportItem::enableExport(ReportExportMode mode, bool ok)
{
    export_flags_.set((size_t)mode, ok);
}

/**
 */
void ReportItem::enableExports(bool ok)
{
    if (ok)
        export_flags_.set();
    else
        export_flags_.reset();
}

/**
 */
void ReportItem::setJSONProperty(const std::string& name, const nlohmann::json& value)
{
    properties_[ name ] = value;
}

/**
 */
bool ReportItem::hasJSONProperty(const std::string& name) const
{
    return properties_.contains(name);
}

/**
 */
nlohmann::json ReportItem::jsonProperty(const std::string& name) const
{
    if (!hasJSONProperty(name))
        return nlohmann::json();

    return properties_.at(name);
}

/**
*/
nlohmann::json ReportItem::toJSON() const
{
    nlohmann::json j;

    j[ FieldName        ] = name_;
    j[ FieldProperties  ] = properties_;
    j[ FieldExportFlags ] = export_flags_.to_ulong();

    toJSON_impl(j);

    return j;
}

/**
*/
bool ReportItem::fromJSON(const nlohmann::json& j)
{
    if (!j.is_object()               ||
        !j.contains(FieldName)       ||
        !j.contains(FieldProperties) ||
        !j.contains(FieldExportFlags))
    {
        logerr << "item does not obtain needed fields";
        return false;
    }

    try
    {
        name_       = j[ FieldName       ];
        properties_ = j[ FieldProperties ];

        unsigned long export_flags = j[ FieldExportFlags ];
        export_flags_ = ExportFlags(export_flags);

        updateID();

        if (!fromJSON_impl(j))
            return false;
    }
    catch(const std::exception& ex)
    {
        logerr << ex.what();
        return false;
    }
    catch(...)
    {
        logerr << "unknown JSON error";
        return false;
    }

    return true;
}

/**
*/
ResultT<nlohmann::json> ReportItem::toJSONDocument(const std::string* resource_dir,
                                                   ReportExportMode export_style) const
{
    nlohmann::json j;

    auto addBaseItems = [ & ] (nlohmann::json& j_item)
    {
        j_item[ FieldName ] = name_;

        // id is only added in sections
    };

    auto res = toJSONDocument_impl(j, resource_dir, export_style);
    if (!res.ok())
        return res;

    if (j.is_array())
    {
        //some report items generate multiple objects in an array (e.g. figures),
        //so add base items to each generated item
        for (auto& j_item : j)
            addBaseItems(j_item);
    }
    else
    {
        //default case: add base items to single object
        addBaseItems(j);
    }

    return ResultT<nlohmann::json>::succeeded(j);
}

}
