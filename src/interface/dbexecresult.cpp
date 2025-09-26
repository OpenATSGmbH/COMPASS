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

#include "dbexecresult.h"

#include "buffer.h"
#include "logger.h"

/**
 */
std::shared_ptr<Buffer> DBExecResult::toBuffer(const std::string& dbcontent_name,
                                               const boost::optional<size_t>& offset,
                                               const boost::optional<size_t>& max_entries)
{
    auto property_list = propertyList();
    
    if (!property_list.has_value())
        logerr << "property list could not be retrieved from result";
    traced_assert(property_list.has_value());

    return toBuffer(property_list.value(), dbcontent_name, offset, max_entries);
}

/**
 */
std::shared_ptr<Buffer> DBExecResult::toBuffer(const PropertyList& properties,
                                               const std::string& dbcontent_name,
                                               const boost::optional<size_t>& offset,
                                               const boost::optional<size_t>& max_entries)
{
    if (properties.size() < 1)
        return std::shared_ptr<Buffer>();

    //create buffer
    std::shared_ptr<Buffer> buffer(new Buffer(properties, dbcontent_name));

    if (!toBuffer(*buffer, offset, max_entries))
        return std::shared_ptr<Buffer>();

    return buffer;
}
