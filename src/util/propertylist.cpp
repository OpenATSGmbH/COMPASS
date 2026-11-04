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

#include "propertylist.h"
#include "logger.h"
#include "traced_assert.h"

PropertyList::PropertyList()
{
    logdbg;
}

PropertyList::PropertyList(const std::vector<Property>& properties)
:   properties_(properties)
{
    logdbg;
}

PropertyList::~PropertyList()
{
    logdbg;
    clear();
    logdbg << "end";
}

PropertyList::PropertyList(const PropertyList& org)
{
    properties_ = org.properties_;
    // loginf << "properties " << properties_.size();
}


void PropertyList::operator=(const PropertyList& org)
{
    properties_ = org.properties_;
}

void PropertyList::addPropertyList(const PropertyList& org)
{
    properties_.insert(properties_.end(), org.properties_.begin(), org.properties_.end());
}

void PropertyList::addProperty(std::string id, PropertyDataType type)
{
    logdbg;
    logdbg << "id '" << id << "' type " << Property::asString(type);
    traced_assert(!id.empty());

    if (hasProperty(id))
    {
        logwrn << "property " << id << " already added";
        return;
    }

    properties_.push_back(Property(id, type));
    logdbg << "end";
}

void PropertyList::addProperty(Property& property)
{
    logdbg;

    if (hasProperty(property.name()))
    {
        logwrn << "property " << property.name() << " already added";
        return;
    }

    properties_.push_back(property);
    logdbg << "end";
}

void PropertyList::addProperty(const Property& property)
{
    logdbg;

    if (hasProperty(property.name()))
    {
        logwrn << "property " << property.name() << " already added";
        return;
    }

    properties_.push_back(property);
    logdbg << "end";
}

const Property& PropertyList::at(unsigned int index) const
{
    traced_assert(index < properties_.size());
    return properties_.at(index);
}

void PropertyList::removeProperty(const std::string& id)
{
    logdbg;
    traced_assert(hasProperty(id));

    std::vector<Property>::iterator it;

    for (it = properties_.begin(); it != properties_.end(); it++)
    {
        if (it->name().compare(id) == 0)
        {
            properties_.erase(it);
            logdbg << "end";
            return;
        }
    }
    logerr << "property " << id << " could not be removed";
    traced_assert(false);
}

const Property& PropertyList::get(const std::string& id) const
{
    logdbg;
    traced_assert(hasProperty(id));

    std::vector<Property>::const_iterator it;

    for (it = properties_.begin(); it != properties_.end(); it++)
    {
        if (it->name().compare(id) == 0)
        {
            return *it;
        }
    }
    logerr << "property " << id << " not found";
    throw std::runtime_error("PropertyList: get: property " + id + " not found");
}

unsigned int PropertyList::getPropertyIndex(const std::string& id) const
{
    logdbg;
    if (!hasProperty(id))
        throw std::runtime_error("PropertyList: getPropertyIndex: property " + id +
                                 " does not exists");

    unsigned int cnt = 0;
    for (auto& it : properties_)
    {
        if (it.name() == id)
        {
            return cnt;
        }
        cnt++;
    }
    throw std::runtime_error("PropertyList: getPropertyIndex: property " + id + " not found");
}

bool PropertyList::hasProperty(const std::string& id) const
{
    logdbg;

    for (auto& it : properties_)
    {
        if (it.name() == id)
            return true;
    }

    return false;
}

bool PropertyList::hasProperty(const Property& prop) const
{
    logdbg;

    for (auto& it : properties_)
    {
        if (it.name() == prop.name() && it.dataType() == prop.dataType())
            return true;
    }

    return false;
}

void PropertyList::clear()
{
    logdbg;
    properties_.clear();
    logdbg << "end";
}

void PropertyList::print () const
{
    for (auto& it : properties_)
    {
        loginf << "Property id '" << it.name() << "' type " << it.dataTypeString();
    }
}
