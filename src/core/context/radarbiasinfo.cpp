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

#include "radarbiasinfo.h"

RadarBiasInfo RadarBiasInfo::fromMap(const std::map<std::string, double>& bias)
{
    RadarBiasInfo info;

    if (bias.count("azimuth_bias"))
    {
        info.azimuth_bias_deg_ = bias.at("azimuth_bias");
        info.azimuth_bias_valid_ = true;
    }

    if (bias.count("range_bias"))
    {
        info.range_bias_m_ = bias.at("range_bias");
        info.range_bias_valid_ = true;
    }

    if (bias.count("range_gain"))
        info.range_gain_ = bias.at("range_gain");

    if (bias.count("azimuth_bias_stddev")
        && bias.count("range_bias_stddev")
        && bias.count("range_gain_stddev"))
    {
        info.azimuth_bias_stddev_deg_ = bias.at("azimuth_bias_stddev");
        info.range_bias_stddev_m_ = bias.at("range_bias_stddev");
        info.range_gain_stddev_ = bias.at("range_gain_stddev");

        info.has_bias_stddevs_ =
            info.azimuth_bias_stddev_deg_ > 0
            && info.range_bias_stddev_m_ > 0
            && info.range_gain_stddev_ > 0;
    }

    return info;
}
