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

#include "propertylist.h"
#include "result.h"

#include <string>
#include <memory>

#include <boost/optional.hpp>

class Buffer;

/**
 */
class DBExecResult
{
public:
    DBExecResult() = default;
    virtual ~DBExecResult() = default;

    virtual bool hasError() const = 0;
    virtual std::string errorString() const = 0;
    virtual boost::optional<PropertyList> propertyList() const { return boost::optional<PropertyList>(); }
    virtual boost::optional<size_t> numColumns() const { return boost::optional<size_t>(); }
    virtual boost::optional<size_t> numRows() const { return boost::optional<size_t>(); }

    virtual bool toBuffer(Buffer& buffer,
                          const boost::optional<size_t>& offset = boost::optional<size_t>(),
                          const boost::optional<size_t>& max_entries = boost::optional<size_t>()) = 0;

    virtual std::shared_ptr<Buffer> toBuffer(const std::string& dbcontent_name = "",
                                             const boost::optional<size_t>& offset = boost::optional<size_t>(),
                                             const boost::optional<size_t>& max_entries = boost::optional<size_t>());
    virtual std::shared_ptr<Buffer> toBuffer(const PropertyList& properties,
                                             const std::string& dbcontent_name = "",
                                             const boost::optional<size_t>& offset = boost::optional<size_t>(),
                                             const boost::optional<size_t>& max_entries = boost::optional<size_t>());

    virtual ResultT<bool> readNextChunk(Buffer& buffer,
                                        size_t max_entries) = 0;
};
