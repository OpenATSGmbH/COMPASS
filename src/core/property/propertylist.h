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

#include <vector>

#include "property.h"

/**
 * @brief List of Property instances
 *
 * Uses copy constructor for assignment.
 *
 */
// TODO maybe rework to map?
class PropertyList
{
  protected:
    /// Container with all properties
    std::vector<Property> properties_;

  public:
    /// @brief Constructor
    PropertyList();
    /// @brief Constructor
    PropertyList(const std::vector<Property>& properties);
    /// @brief Desctructor. Calls clear.
    virtual ~PropertyList();

    /// @brief Copy constructor
    PropertyList(const PropertyList& org);

    /// @brief Copy operator
    void operator=(const PropertyList& org);

    void addPropertyList(const PropertyList& org);

    /// @brief Adds a property
    void addProperty(std::string id, PropertyDataType type);

    /// @brief Adds a property
    void addProperty(Property& property);

    void addProperty(const Property& property);

    /// @brief Return container with all properties
    const std::vector<Property>& properties() const { return properties_; }

    const Property& at(unsigned int index) const;
    /**
     * @brief Removes a property
     *
     * \exception std::runtime_error if identifier not found
     */
    void removeProperty(const std::string& id);

    /**
     * @brief Returns a property by id
     *
     * \exception std::runtime_error if identifier not found
     */
    const Property& get(const std::string& id) const;

    /**
     * @brief Returns index of a property
     *
     * \exception std::runtime_error if identifier not found
     */
    unsigned int getPropertyIndex(const std::string& id) const;

    /// @brief Returns flag indicating if property is in list
    bool hasProperty(const std::string& id) const;

    bool hasProperty(const Property& prop) const;

    /// @brief Returns flag indicating if property with given indexis in list
    bool hasProperty(unsigned int index) const { return index < properties_.size(); }
    /// @brief Removes all properties
    void clear();

    void print () const;

    /// @brief Return number of properties in list
    unsigned int size() const { return properties_.size(); }
};
