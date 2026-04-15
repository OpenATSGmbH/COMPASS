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

#include "data_source.h"

#include "number.h"
#include "traced_assert.h"

#include <json.hpp>

using namespace std;
using namespace nlohmann;

namespace context
{

DataSource::DataSource() = default;

unsigned int DataSource::id() const
{
    return Utils::Number::dsIdFrom(sac_, sic_);
}

void DataSource::shortName(const std::string& short_name)
{
    has_short_name_ = true;
    short_name_ = short_name;
}

void DataSource::removeShortName()
{
    has_short_name_ = false;
    short_name_.clear();
}

// ============================================================
// Convenience accessors (mirrors DataSourceBase API)
// ============================================================

static const string position_key = "position";
static const string latitude_key = "latitude";
static const string longitude_key = "longitude";
static const string altitude_key = "altitude";

bool DataSource::hasPosition() const
{
    return info_.contains(position_key)
        && info_.at(position_key).contains(latitude_key)
        && info_.at(position_key).contains(longitude_key);
}

double DataSource::latitude() const { return info_.at(position_key).at(latitude_key); }
void DataSource::latitude(double value) { info_[position_key][latitude_key] = value; }

double DataSource::longitude() const { return info_.at(position_key).at(longitude_key); }
void DataSource::longitude(double value) { info_[position_key][longitude_key] = value; }

double DataSource::altitude() const { return info_.at(position_key).at(altitude_key); }
void DataSource::altitude(double value) { info_[position_key][altitude_key] = value; }

bool DataSource::groundOnly() const
{
    return info_.contains("ground_only") && info_.at("ground_only").get<bool>();
}

bool DataSource::hasRadarRanges() const
{
    return info_.contains("radar_range");
}

map<string, double> DataSource::radarRanges() const
{
    if (!hasRadarRanges()) return {};
    return info_.at("radar_range").get<map<string, double>>();
}

bool DataSource::hasNetworkLines() const
{
    return info_.contains("network_lines");
}

bool DataSource::hasRemoteUnits() const
{
    return info_.contains("remote_units");
}

bool DataSource::hasRemoteUnit(int index) const
{
    if (!hasRemoteUnits()) return false;
    string key = to_string(index);
    return info_.at("remote_units").contains(key);
}

string DataSource::remoteUnitName(int index) const
{
    string key = to_string(index);
    const auto& ru = info_.at("remote_units").at(key);
    if (ru.contains("name"))
        return ru.at("name").get<string>();
    return key;
}

bool DataSource::remoteUnitIsValid(int index) const
{
    if (!hasRemoteUnit(index))
        return false;

    const auto& ru = info_.at("remote_units").at(to_string(index));

    return ru.contains("latitude")  && ru.at("latitude").is_number()
        && ru.contains("longitude") && ru.at("longitude").is_number()
        && ru.contains("altitude")  && ru.at("altitude").is_number();
}

double DataSource::remoteUnitLatitude(int index) const
{
    return info_.at("remote_units").at(to_string(index)).at("latitude").get<double>();
}

double DataSource::remoteUnitLongitude(int index) const
{
    return info_.at("remote_units").at(to_string(index)).at("longitude").get<double>();
}

double DataSource::remoteUnitAltitude(int index) const
{
    return info_.at("remote_units").at(to_string(index)).at("altitude").get<double>();
}

bool DataSource::isCalculatedReferenceSource() const
{
    return info_.contains("calculated_reftraj") && info_.at("calculated_reftraj").get<bool>();
}

void DataSource::setCalculatedReferenceSource()
{
    info_["calculated_reftraj"] = true;
}

// ============================================================
// Serialization
// ============================================================

json DataSource::toJSON() const
{
    json j;

    j["ds_type"] = ds_type_;
    j["sac"] = sac_;
    j["sic"] = sic_;
    j["name"] = name_;

    if (has_short_name_)
        j["short_name"] = short_name_;

    if (!info_.is_null())
        j["info"] = info_;

    return j;
}

DataSource DataSource::fromJSON(const json& j)
{
    DataSource ds;

    traced_assert(j.contains("ds_type"));
    ds.ds_type_ = j.at("ds_type");

    traced_assert(j.contains("sac"));
    ds.sac_ = j.at("sac");

    traced_assert(j.contains("sic"));
    ds.sic_ = j.at("sic");

    traced_assert(j.contains("name"));
    ds.name_ = j.at("name");

    if (j.contains("short_name"))
    {
        ds.has_short_name_ = true;
        ds.short_name_ = j.at("short_name");
    }

    if (j.contains("info"))
        ds.info_ = j.at("info");

    return ds;
}

bool DataSource::operator==(const DataSource& other) const
{
    return ds_type_ == other.ds_type_
        && sac_ == other.sac_
        && sic_ == other.sic_
        && name_ == other.name_
        && has_short_name_ == other.has_short_name_
        && short_name_ == other.short_name_
        && info_ == other.info_;
}

// ============================================================
// Additional convenience methods for edit widget compatibility
// ============================================================

void DataSource::groundOnly(bool value)
{
    info_["ground_only"] = value;
}

bool DataSource::hasUpdateInterval() const
{
    return info_.contains("update_interval") && info_.at("update_interval") != 0;
}

float DataSource::updateInterval() const
{
    traced_assert(hasUpdateInterval());
    return info_.at("update_interval");
}

void DataSource::updateInterval(float value)
{
    info_["update_interval"] = value;
}

void DataSource::removeUpdateInterval()
{
    if (info_.contains("update_interval"))
        info_.erase("update_interval");
}

int DataSource::detectionTypeInt() const
{
    if (info_.contains("detection_type") && info_["detection_type"].is_string())
    {
        auto s = info_["detection_type"].get<string>();
        if (s == "PrimaryOnly") return 1;
        if (s == "ModeAC") return 2;
        if (s == "ModeACCombined") return 3;
        if (s == "ModeS") return 4;
        if (s == "ModeSCombined") return 5;
    }
    return 0; // Undefined
}

void DataSource::detectionTypeInt(int value)
{
    static const char* names[] = {"Undefined", "PrimaryOnly", "ModeAC", "ModeACCombined", "ModeS", "ModeSCombined"};
    if (value >= 0 && value <= 5)
        info_["detection_type"] = names[value];

    if (value != 1 && groundOnly())
        groundOnly(false);
}

bool DataSource::ignoreRadarAzmRange() const
{
    if (!info_.contains("ignore_radar_azm_range"))
        return false;
    return info_.at("ignore_radar_azm_range");
}

void DataSource::ignoreRadarAzmRange(bool value)
{
    info_["ignore_radar_azm_range"] = value;
}

double DataSource::probabilityOfDetection() const
{
    return info_.at("pd");
}

void DataSource::probabilityOfDetection(double value)
{
    info_["pd"] = value;
}

double DataSource::clutterRate() const
{
    return info_.at("clutter_rate");
}

void DataSource::clutterRate(double value)
{
    info_["clutter_rate"] = value;
}

void DataSource::radarRange(const string& key, double value)
{
    info_["radar_range"][key] = value;
}

void DataSource::removeRadarRange(const string& key)
{
    if (info_.contains("radar_range") && info_.at("radar_range").contains(key))
        info_.at("radar_range").erase(key);
}

void DataSource::addRadarRangesIfMissing()
{
    if (!hasRadarRanges())
        info_["radar_range"] = json::object();
}

bool DataSource::hasRadarAccuracies() const
{
    return info_.contains("radar_accuracy");
}

map<string, double> DataSource::radarAccuracies() const
{
    if (!hasRadarAccuracies()) return {};
    return info_.at("radar_accuracy").get<map<string, double>>();
}

void DataSource::radarAccuracy(const string& key, double value)
{
    info_["radar_accuracy"][key] = value;
}

void DataSource::addRadarAccuraciesIfMissing()
{
    if (!hasRadarAccuracies())
        info_["radar_accuracy"] = json::object();
}

void DataSource::addNetworkLinesIfMissing()
{
    if (!hasNetworkLines())
        info_["network_lines"] = json::object();
}

void DataSource::addRemoteUnitsIfMissing()
{
    if (!hasRemoteUnits())
        info_["remote_units"] = json::object();
}

void DataSource::removeRemoteUnits()
{
    if (info_.contains("remote_units"))
        info_.erase("remote_units");
}

void DataSource::removeRemoteUnit(int index)
{
    if (!info_.contains("remote_units") || !info_["remote_units"].is_object())
        return;

    string key = to_string(index);
    info_["remote_units"].erase(key);
}

// ============================================================
// Static DS type utilities
// ============================================================

static const vector<string> ds_type_strings = {"Radar", "MLAT", "ADSB", "Tracker", "RefTraj", "Other"};

const vector<string>& DataSource::dsTypeStrings()
{
    return ds_type_strings;
}

string DataSource::dsTypeToString(int type_enum)
{
    static const char* names[] = {"ADSB", "MLAT", "Radar", "Tracker", "RefTraj", "Other"};
    if (type_enum >= 0 && type_enum <= 5)
        return names[type_enum];
    return "Other";
}

int DataSource::dsTypeFromString(const string& type_str)
{
    if (type_str == "ADSB")    return 0;
    if (type_str == "MLAT")    return 1;
    if (type_str == "Radar")   return 2;
    if (type_str == "Tracker") return 3;
    if (type_str == "RefTraj") return 4;
    if (type_str == "Other")   return 5;
    return 5; // Other
}

} // namespace context
