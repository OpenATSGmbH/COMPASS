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

#include <cmath>

using namespace std;
using namespace nlohmann;

namespace context
{

static const string latitude_key = "latitude";
static const string longitude_key = "longitude";
static const string altitude_key = "altitude";
static const string mode_s_address_key = "mode_s_address";
static const string mode_3a_code_key = "mode_3a_code";
static const string mode_c_code_key = "mode_c_code";

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

bool FFT::hasModeSAddress() const
{
    return info_.contains(mode_s_address_key);
}

unsigned int FFT::modeSAddress() const
{
    return info_.at(mode_s_address_key);
}

void FFT::modeSAddress(unsigned int value)
{
    info_[mode_s_address_key] = value;
}

bool FFT::hasMode3ACode() const
{
    return info_.contains(mode_3a_code_key);
}

unsigned int FFT::mode3ACode() const
{
    return info_.at(mode_3a_code_key);
}

void FFT::mode3ACode(unsigned int value)
{
    info_[mode_3a_code_key] = value;
}

bool FFT::hasModeCCode() const
{
    return info_.contains(mode_c_code_key);
}

float FFT::modeCCode() const
{
    return info_.at(mode_c_code_key);
}

void FFT::modeCCode(float value)
{
    info_[mode_c_code_key] = value;
}

bool FFT::hasSecondaryIdentification() const
{
    return hasModeSAddress() || hasMode3ACode();
}

bool FFT::isAlwaysFFTCode(boost::optional<unsigned int> mode_a_code)
{
    return mode_a_code && *mode_a_code == always_fft_mode_3a_code_;
}

pair<bool, float> FFT::matches(double latitude_deg, double longitude_deg,
                               boost::optional<unsigned int> mode_s_address,
                               bool ignore_mode_s,
                               boost::optional<unsigned int> mode_a_code,
                               boost::optional<float> mode_c_code) const
{
    bool secondary_matched = false;

    // mode S address check
    if (!ignore_mode_s && hasModeSAddress() && mode_s_address)
    {
        if (modeSAddress() != *mode_s_address)
            return {false, 0};

        secondary_matched = true;
    }

    // mode 3/A code check
    if (hasMode3ACode() && mode_a_code)
    {
        if (mode3ACode() != *mode_a_code)
            return {false, 0};

        secondary_matched = true;
    }

    // mode C code check, confirmation only
    if (hasModeCCode() && mode_c_code)
    {
        if (modeCCode() != *mode_c_code)
            return {false, 0};
    }

    // without a secondary identification the target report can be from any
    // aircraft overflying the FFT position
    if (!secondary_matched)
        return {false, 0};

    // position check
    if (hasPosition())
    {
        // approximate distance check using GeographicLib or simple haversine
        double dlat = latitude_deg - latitude();
        double dlon = longitude_deg - longitude();

        // rough distance in meters (1 degree ~ 111km lat, cos(lat)*111km lon)
        double cos_lat = cos(latitude_deg * M_PI / 180.0);
        double dist_m = sqrt(dlat * dlat + dlon * dlon * cos_lat * cos_lat) * 111000.0;

        if (dist_m > max_plot_distance_m_)
            return {false, 0};
    }

    float alt = 0;
    if (hasAltitude())
        alt = static_cast<float>(altitude());

    return {true, alt};
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
