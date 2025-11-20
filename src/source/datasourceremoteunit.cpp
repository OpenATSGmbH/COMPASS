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

#include "datasourceremoteunit.h"
#include "traced_assert.h"

const std::string DataSourceRemoteUnit::KeyIndex     = "index";
const std::string DataSourceRemoteUnit::KeyName      = "name";
const std::string DataSourceRemoteUnit::KeyComment   = "comment";
const std::string DataSourceRemoteUnit::KeyLatitude  = "latitude";
const std::string DataSourceRemoteUnit::KeyLongitude = "longitude";
const std::string DataSourceRemoteUnit::KeyAltitude  = "altitude";

/**
 */
DataSourceRemoteUnit::DataSourceRemoteUnit(nlohmann::json& config)
:  config_(config)
{
}

/**
 */
DataSourceRemoteUnit::~DataSourceRemoteUnit() = default;

/**
 */
void DataSourceRemoteUnit::configure(const RemoteUnitDefinition& def)
{
    //once set an index shall not be changed
    traced_assert(!config_.contains(KeyIndex) || index() == def.index);

    index(def.index);
    name(def.name);
    comment(def.comment);
    latitude(def.latitude);
    longitude(def.longitude);
    altitude(def.altitude);
}

/**
 */
RemoteUnitDefinition DataSourceRemoteUnit::toDefinition() const
{
    RemoteUnitDefinition def;
    def.index     = index();
    def.name      = name();
    def.comment   = comment();
    def.latitude  = latitude();
    def.longitude = longitude();
    def.altitude  = altitude();

    return def;
}

/**
 */
int DataSourceRemoteUnit::index() const
{
    traced_assert(config_.contains(KeyIndex));
    traced_assert(config_.at(KeyIndex).is_number_integer());
    return config_.at(KeyIndex).get<int>();
}

/**
 */
void DataSourceRemoteUnit::index(int idx)
{
    config_[KeyIndex] = idx;
}

/**
 */
std::string DataSourceRemoteUnit::name() const
{
    traced_assert(config_.contains(KeyName));
    traced_assert(config_.at(KeyName).is_string());
    return config_.at(KeyName).get<std::string>();
}

/**
 */
void DataSourceRemoteUnit::name(const std::string& name)
{
    config_[KeyName] = name;
}

/**
 */
std::string DataSourceRemoteUnit::comment() const
{
    traced_assert(config_.contains(KeyComment));
    traced_assert(config_.at(KeyComment).is_string());
    return config_.at(KeyComment).get<std::string>();
}

/**
 */
void DataSourceRemoteUnit::comment(const std::string& comment)
{
    config_[KeyComment] = comment;
}

/**
 */
double DataSourceRemoteUnit::latitude() const
{
    traced_assert(config_.contains(KeyLatitude));
    traced_assert(config_.at(KeyLatitude).is_number_float());
    return config_.at(KeyLatitude).get<double>();
}

/**
 */
void DataSourceRemoteUnit::latitude(double lat)
{
    config_[KeyLatitude] = lat;
}

/**
 */
double DataSourceRemoteUnit::longitude() const
{
    traced_assert(config_.contains(KeyLongitude));
    traced_assert(config_.at(KeyLongitude).is_number_float());
    return config_.at(KeyLongitude).get<double>();
}

/**
 */
void DataSourceRemoteUnit::longitude(double lon)
{
    config_[KeyLongitude] = lon;
}

/**
 */
double DataSourceRemoteUnit::altitude() const
{
    traced_assert(config_.contains(KeyAltitude));
    traced_assert(config_.at(KeyAltitude).is_number_float());
    return config_.at(KeyAltitude).get<double>();
}

/**
 */
void DataSourceRemoteUnit::altitude(double alt)
{
    config_[KeyAltitude] = alt;
}

/**
 */
std::string DataSourceRemoteUnit::asString() const
{
    return config_.dump(4);
}
