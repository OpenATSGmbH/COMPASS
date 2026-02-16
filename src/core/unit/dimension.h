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

#include "configurable.h"

class Unit;

class Dimension : public Configurable
{
  public:
    Dimension(const std::string& class_id, const std::string& instance_id, Configurable* parent);
    /// @brief Destructor
    virtual ~Dimension();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    void addUnit(const std::string& name, double factor, const std::string& definition);
    /// @brief Returns factor from one unit to another
    bool hasUnit(const std::string& unit) const;
    double getFactor(const std::string& unit_source, const std::string& unit_destination) const;

    const std::map<std::string, Unit*>& units() const { return units_; }

  protected:
    std::map<std::string, Unit*> units_;

    virtual void checkSubConfigurables() {}
};
