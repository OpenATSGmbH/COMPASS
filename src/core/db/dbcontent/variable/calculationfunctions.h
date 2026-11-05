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

class Buffer;
class DBContentManager;

namespace dbContent
{

/**
 * The hardcoded calculations behind the "function" operator of CalculationPlan, see
 * readme_calc_operators.md. A function gets the whole buffer and the application, it may read
 * the variable model and the managers, and writes its output columns itself.
 */
namespace calculation_functions
{

/// "radar_projection": "Latitude*" and "Longitude*" of CAT001 and CAT048 from range, azimuth and
/// the sensor position, rows with a position stay, see ProjectionManager::doRadarPlotPositionCalculations
void radarProjection(DBContentManager& dbcont_man, const std::string& dbcontent_name, const std::shared_ptr<Buffer>& buffer);

/// "adsb_sgv_speed_track": "Ground Speed*" and "Track Angle*" of CAT021, filled from the decoded
/// I021/160 values first, then from the SGV fields of the REF for surface targets
void adsbSGVSpeedTrack(DBContentManager& dbcont_man, const std::string& dbcontent_name, const std::shared_ptr<Buffer>& buffer);

}

}
