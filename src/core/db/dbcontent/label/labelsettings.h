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

#include "labeldirection.h"

#include <map>

namespace dbContent
{

/**
 * Label settings needed by label display items (implemented by the LabelGenerator).
 * - decouples label items from the generator, which allows to test them with fake settings
 */
class LabelSettings
{
public:
    virtual ~LabelSettings() = default;

    static constexpr float MaxTransparency = 0.67f; // max transparency of the label background selectable in the ui

    virtual unsigned int currentLOD() const = 0;                     // current label level of detail (1-3)
    virtual float labelOpacity() const = 0;                          // 0 = transparent, 1 = opaque
    virtual LabelDirection labelDirection(unsigned int ds_id) = 0;   // label direction of the data source
    virtual const std::map<unsigned long, unsigned int>& viewPointLabels() const = 0; // view point defined labels: rec_num -> lod
};

}
