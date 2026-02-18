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

#include "fft/fftbase.h"
#include "logger.h"
#include "number.h"
#include "traced_assert.h"

using namespace Utils;
using namespace std;
using namespace nlohmann;

const string latitude_key = "latitude";
const string longitude_key = "longitude";
const string altitude_key = "altitude";
const string mode_s_address_key = "mode_s_address";
const string mode_3a_code_key = "mode_3a_code";
const string mode_c_code_key = "mode_c_code";

FFTBase::FFTBase()
{
    info_ = json::object_t();
}

std::string FFTBase::name() const
{
    return name_;
}

void FFTBase::name(const std::string &name)
{
    name_ = name;
}

nlohmann::json& FFTBase::info()
{
    return info_;
}

void FFTBase::info(const nlohmann::json& info)
{
    traced_assert(info.is_object());
    info_ = info;
}

std::string FFTBase::infoStr()
{
    return info_.dump();
}

bool FFTBase::hasModeSAddress()
{
    return info_.contains(mode_s_address_key);
}
unsigned int FFTBase::modeSAddress()
{
    traced_assert(hasModeSAddress());
    return info_.at(mode_s_address_key);
}
void FFTBase::modeSAddress(unsigned int value)
{
    info_[mode_s_address_key] = value;
}

void FFTBase::clearModeSAddress()
{
    if (info_.contains(mode_s_address_key))
        info_.erase(mode_s_address_key);
}

bool FFTBase::hasMode3ACode()
{
    return info_.contains(mode_3a_code_key);
}
unsigned int FFTBase::mode3ACode()
{
    traced_assert(hasMode3ACode());
    return info_.at(mode_3a_code_key);
}

void FFTBase::mode3ACode(unsigned int value)
{
    info_[mode_3a_code_key] = value;
}

void FFTBase::clearMode3ACode()
{
    if (info_.contains(mode_3a_code_key))
        info_.erase(mode_3a_code_key);
}

bool FFTBase::hasModeCCode()
{
    return info_.contains(mode_c_code_key);
}
float FFTBase::modeCCode()
{
    traced_assert(hasModeCCode());
    return info_.at(mode_c_code_key);
}

void FFTBase::modeCCode(float value)
{
    info_[mode_c_code_key] = value;
}

void FFTBase::clearModeCCode()
{
    if (info_.contains(mode_c_code_key))
        info_.erase(mode_c_code_key);
}

bool FFTBase::hasPosition() const
{
    return info_.contains(latitude_key)
            && info_.contains(longitude_key);
}

void FFTBase::latitude (double value)
{
    info_[latitude_key] = value;
}
double FFTBase::latitude () const
{
    if (!info_.contains(latitude_key))
        return 0.0;
    else
        return info_.at(latitude_key);
}

void FFTBase::longitude (double value)
{
    info_[longitude_key] = value;
}
double FFTBase::longitude () const
{
    if (!info_.contains(longitude_key))
        return 0.0;
    else
        return info_.at(longitude_key);
}

bool FFTBase::hasAltitude() const
{
    return info_.contains(altitude_key);
}

void FFTBase::altitude (double value)
{
    info_[altitude_key] = value;
}

double FFTBase::altitude () const
{
    if (!info_.contains(altitude_key))
        return 0.0;
    else
        return info_.at(altitude_key);
}

void FFTBase::setFromJSON (const nlohmann::json& j)
{
    traced_assert(j.contains("name"));
    name_ = j.at("name");

    traced_assert(j.contains("info"));
    traced_assert(j.at("info").is_object());
    info_ = j.at("info");
}

json FFTBase::getAsJSON() const
{
    json j;

    j["name"] = name_;

    j["info"] = info_;

    return j;
}





