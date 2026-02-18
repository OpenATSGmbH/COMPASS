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

#include "dbcontentvariablelookup.h"
#include "dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"

namespace dbContent 
{

/**
*/
DBContentVariableLookup::DBContentVariableLookup(const std::string& dbcontent_name,
                                                 const std::shared_ptr<Buffer>& buffer)
:   dbcontent_name_(dbcontent_name)
,   buffer_        (buffer        )
{
    traced_assert(buffer_);
    traced_assert(!dbcontent_name_.empty());
}

/**
*/
DBContentVariableLookup::~DBContentVariableLookup() = default;

/**
*/
void DBContentVariableLookup::update(const DBContentManager& dbcontent_manager)
{
    meta_var_lookup_.clear();

    //collect metavars existing for buffer's dbcontent
    for (const auto& meta_var_it : dbcontent_manager.metaVariables())
    {
        if (meta_var_it.second->existsIn(dbcontent_name_) && // // meta exists in buf dbcont name
            buffer_->hasAnyPropertyNamed(meta_var_it.second->getNameFor(dbcontent_name_)))
        {
            // meta var -> var exists in buffer
            meta_var_lookup_[meta_var_it.first] = meta_var_it.second->getNameFor(dbcontent_name_);
        }
    }
}

/**
*/
void DBContentVariableLookup::print() const
{
    std::cout << "dbcontent = " << dbcontent_name_ << std::endl;

    for (const auto& elem : meta_var_lookup_)
        std::cout << "    " << elem.first << " => " << elem.second << std::endl;
}

unsigned int DBContentVariableLookup::size() const
{
    if (buffer_)
        return buffer_->size();
    else
        return 0;
}

} // namespace dbContent
