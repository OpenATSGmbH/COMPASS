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

#include "fft.h"

#include "traced_assert.h"

#include <json.hpp>

using namespace std;
using namespace nlohmann;

namespace context
{

static const string latitude_key = "latitude";
static const string longitude_key = "longitude";
static const string altitude_key = "altitude";

FFT::FFT() = default;

bool FFT::hasPosition() const
{
    return info_.contains(latitude_key)
        && info_.contains(longitude_key);
}

double FFT::latitude() const
{
    return info_.at(latitude_key);
}

void FFT::latitude(double value)
{
    info_[latitude_key] = value;
}

double FFT::longitude() const
{
    return info_.at(longitude_key);
}

void FFT::longitude(double value)
{
    info_[longitude_key] = value;
}

bool FFT::hasAltitude() const
{
    return info_.contains(altitude_key);
}

double FFT::altitude() const
{
    return info_.at(altitude_key);
}

void FFT::altitude(double value)
{
    info_[altitude_key] = value;
}

json FFT::toJSON() const
{
    json j;

    j["name"] = name_;

    if (!info_.is_null() && !info_.empty())
        j["info"] = info_;

    return j;
}

FFT FFT::fromJSON(const json& j)
{
    FFT fft;

    traced_assert(j.contains("name"));
    fft.name_ = j.at("name");

    // keep the default empty object if the stored info is absent or null
    if (j.contains("info") && j.at("info").is_object())
        fft.info_ = j.at("info");

    return fft;
}

bool FFT::operator==(const FFT& other) const
{
    return name_ == other.name_
        && info_ == other.info_;
}

} // namespace context
