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

#include <string>
#include <utility>
#include <vector>

namespace sector_utils
{

struct ImportedSector
{
    std::string name;
    std::vector<std::pair<double, double>> points; // lat, lon
};

/// Parse a GDAL-supported vector file and return polygon data.
/// Each linear ring (exterior + interior) becomes a separate ImportedSector.
/// Sector names are derived from the GDAL layer name + a running index.
/// Returns empty vector on failure.
std::vector<ImportedSector> parseGDALFile(const std::string& filepath);

} // namespace sector_utils
