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

#include "filterclause.h"

#include <map>
#include <memory>
#include <string>

class Buffer;

/**
 * The filter functions that code below the filter layer uses, via COMPASS::filterProvider().
 * Implemented by FilterManager.
 */
class IFilterProvider
{
public:
    virtual ~IFilterProvider() = default;

    virtual bool useFilters() const = 0;
    virtual FilterClause viewClause(const std::string& dbcontent_name) = 0;
    virtual void filterBuffers(std::map<std::string, std::shared_ptr<Buffer>>& data) = 0;
};
