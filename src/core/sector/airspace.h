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

#include <memory>
#include <string>
#include <vector>

class QColor;

class SectorLayer;

namespace dbContent {
class TargetPosition;
}

/**
*/
class AirSpace
{
public:
    enum class AboveCheckResult
    {
        OutOfAirspace,
        Above,
        Below
    };

    AirSpace();
    virtual ~AirSpace();

    void clear();
    bool readJSON(const std::string& fn, unsigned int base_id = 0);

    const std::vector<std::shared_ptr<SectorLayer>>& layers() const;

    static AboveCheckResult isAbove(const SectorLayer* layer,
                                    const dbContent::TargetPosition& pos,
                                    bool has_ground_bit, 
                                    bool ground_bit_set);

    std::shared_ptr<SectorLayer> lowerHeightFilterLayer() const;

    static const std::string LowerHeightLayerName;
    static const QColor      DefaultSectorColor;

private:
    std::vector<std::shared_ptr<SectorLayer>> layers_;
};
