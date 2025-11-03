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

#include "reportdefs.h"
#include "result.h"

#include <bitset>

#include <json.hpp>

namespace ResultReport
{

/**
 * Base class for all items in a report (Report, Section, SectionContent, etc.)
 * Basic functionality such as a name, a unique id, json properties, flags for item export, json serialization, ...
 */
class ReportItem
{
public:
    typedef std::bitset<32> ExportFlags;

    ReportItem(const std::string& name, 
               ReportItem* parent_item);
    ReportItem(ReportItem* parent_item);
    virtual ~ReportItem();

    virtual ReportItem* parentItem();
    virtual const ReportItem* parentItem() const;

    virtual nlohmann::json jsonConfig() const { return nlohmann::json(); }
    virtual bool configure(const nlohmann::json& j) { return false; }

    const std::string& name() const;
    const std::string& id() const;

    bool exportEnabled(ReportExportMode mode) const;
    void enableExport(ReportExportMode mode, bool ok);
    void enableExports(bool ok);

    void setJSONProperty(const std::string& name, const nlohmann::json& value);
    bool hasJSONProperty(const std::string& name) const;
    nlohmann::json jsonProperty(const std::string& name) const;

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    ResultT<nlohmann::json> toJSONDocument(const std::string* resource_dir = nullptr,
                                           ReportExportMode export_style = ReportExportMode::JSONFile) const;

    static const std::string FieldName;
    static const std::string FieldID;
    static const std::string FieldProperties;
    static const std::string FieldExportFlags;

protected:
    virtual void toJSON_impl(nlohmann::json& j) const = 0;
    virtual bool fromJSON_impl(const nlohmann::json& j) = 0;

    virtual Result toJSONDocument_impl(nlohmann::json& j,
                                       const std::string* temp_dir,
                                       ReportExportMode export_style) const = 0;

    ReportItem* parent_item_ = nullptr;

private:
    void updateID();

    std::string    name_;         // item name
    std::string    id_;           // item id (= full hierarchy link, e.g. Report:Results:Targets)
    nlohmann::json properties_;   // attachable custom json properties
    ExportFlags    export_flags_; // export flags, sepcifying if the item is exported for a certain export type
};

}
