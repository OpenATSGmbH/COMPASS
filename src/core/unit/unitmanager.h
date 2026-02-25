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
#include "singleton.h"

class COMPASS;
class Dimension;

/**
 * @brief Holds and manages all units
 *
 * All units have to call registerUnit.
 */
class UnitManager : public Configurable, public Singleton
{
  public:
    /// @brief Constructor backed by a json reference (for testing / standalone use)
    UnitManager(nlohmann::json& config, COMPASS* parent);
    /// @brief Destructor
    virtual ~UnitManager();

    bool hasDimension(const std::string& name) { return dimensions_.count(name) > 0; }

    /// @brief Returns unit with a given name
    const Dimension& dimension(const std::string& name)
    {
        traced_assert(hasDimension(name));
        return *dimensions_.at(name);
    }
    /// @brief Return container with all units
    const std::map<std::string, Dimension*>& dimensions() { return dimensions_; }

    void generateSubConfigurable(nlohmann::json& child_json) override;

  protected:
    /// Container with all units (unit name (length, time) -> unit)
    std::map<std::string, Dimension*> dimensions_;

    void checkSubConfigurables() override;

    /// @brief Constructor
    UnitManager();

  public:
    static UnitManager& instance()
    {
        static UnitManager instance;
        return instance;
    }
};
