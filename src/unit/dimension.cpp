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

#include "dimension.h"
#include "unit.h"

#include <cmath>


Dimension::Dimension(const std::string& class_id, const std::string& instance_id,
                     Configurable* parent)
    : Configurable(class_id, instance_id, parent)
{
    createSubConfigurables();
}

Dimension::~Dimension()
{
    for (auto it : units_)
        delete it.second;
    units_.clear();
}

void Dimension::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    if (class_id == "Unit")
    {
        Unit* unit = new Unit(class_id, instance_id, *this);
        traced_assert(units_.find(unit->instanceId()) == units_.end());
        units_.insert(std::pair<std::string, Unit*>(unit->instanceId(), unit));
    }
    else
        throw std::runtime_error("UnitManager: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

void Dimension::addUnit(const std::string& name, double factor, const std::string& definition)
{
    auto config = Configuration::create("Unit", name);
    config->addParameter<double>("factor", factor);
    config->addParameter<std::string>("definition", definition);

    generateSubConfigurableFromConfig(std::move(config));
}

bool Dimension::hasUnit(const std::string& unit) const { return units_.find(unit) != units_.end(); }

double Dimension::getFactor(const std::string& unit_source,
                            const std::string& unit_destination) const
{
    logdbg << "unit src '" << unit_source << "' dst '" << unit_destination
           << "'";

    traced_assert(units_.find(unit_source) != units_.end());
    traced_assert(units_.find(unit_destination) != units_.end());
    double factor = 1.0;

    factor /= units_.at(unit_source)->factor();
    factor *= units_.at(unit_destination)->factor();

    logdbg << "src factor " << units_.at(unit_source)->factor()
           << " dest factor " << units_.at(unit_destination)->factor() << " result " << factor;

    traced_assert(factor != 0);
    traced_assert(!std::isinf(factor));

    return factor;
}
