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

#include <cmath>
#include <map>
#include <string>

#include "configurable.h"
#include "dimension.h"
#include "logger.h"

/**
 * @brief Definition of a base unit
 *
 * Automatically registers to UnitManager, has a name (length, time), allows registering sub units
 * with scaling factors.
 */
class Unit : public Configurable
{
  public:
    /// @brief Constructor with a name
    Unit(const std::string& class_id, const std::string& instance_id, Dimension& parent)
        : Configurable(class_id, instance_id, &parent)
    {
        registerParameter("definition", &definition_, std::string());
        registerParameter("factor", &factor_, 1.0);

        logdbg << "dimension " << parent.instanceId() << " unit " << instance_id
               << " factor " << factor_;

        traced_assert(factor_ != 0);
        traced_assert(!std::isinf(factor_));
    }
    /// @brief Destructor
    virtual ~Unit() {}

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id)
    {
        traced_assert(false);
    }

    double factor() const { return factor_; }

  private:
    /// Comment definition
    std::string definition_;
    /// Scaling factor
    double factor_;

  protected:
    virtual void checkSubConfigurables() {}
};
