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

#include "bufferaccessor.h"
#include "dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"

namespace dbContent 
{

/**
*/
BufferAccessor::BufferAccessor(const std::string& dbcontent_name, 
                               const std::shared_ptr<Buffer>& buffer,
                               const DBContentManager& dbcont_man)
:   lookup_(new DBContentVariableLookup(dbcontent_name, buffer))
{
    updateLookup(dbcont_man);
}

/**
*/
BufferAccessor::BufferAccessor(const std::shared_ptr<DBContentVariableLookup>& lookup)
:   lookup_(lookup)
{
    traced_assert(lookup_);
}

/**
*/
BufferAccessor::~BufferAccessor() = default;

/**
*/
void BufferAccessor::updateLookup(const DBContentManager& dbcontent_manager)
{
    lookup_->update(dbcontent_manager);
}

unsigned int BufferAccessor::size() const
{
    traced_assert(lookup_);
    return lookup_->size();
}


} // namespace dbContent
