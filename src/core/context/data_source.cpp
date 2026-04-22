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

#include "color_provider.h"
#include "number.h"
#include "traced_assert.h"

#include <json.hpp>

#include <cmath>

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
// Colors
// ============================================================

void DataSource::baseColor(const QColor& color)
{
    base_color_ = color;

    if (!color.isValid())
    {
        for (auto& lc : line_colors_)
            lc = QColor();
        return;
    }

    auto shades = ColorProvider::autoLineColors(color);
    for (size_t i = 0; i < line_colors_.size() && i < shades.size(); ++i)
        line_colors_[i] = shades[i];
}

QColor DataSource::lineColor(unsigned int line_id) const
{
    traced_assert(line_id < line_colors_.size());
    return line_colors_[line_id];
}

void DataSource::setLineColor(unsigned int line_id, const QColor& color)
{
    traced_assert(line_id < line_colors_.size());
    line_colors_[line_id] = color;
}

void DataSource::resetLineColor(unsigned int line_id)
{
    traced_assert(line_id < line_colors_.size());

    if (!base_color_.isValid())
    {
        line_colors_[line_id] = QColor();
        return;
    }

    auto shades = ColorProvider::autoLineColors(base_color_);
    line_colors_[line_id] = shades[line_id];
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

    if (base_color_.isValid())
        j["base_color"] = base_color_.name().toStdString();

    bool any_line_valid = false;
    for (const auto& lc : line_colors_)
        if (lc.isValid()) { any_line_valid = true; break; }

    if (any_line_valid)
    {
        json arr = json::array();
        for (const auto& lc : line_colors_)
            arr.push_back(lc.isValid() ? lc.name().toStdString() : string{});
        j["line_colors"] = arr;
    }

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

    if (j.contains("base_color"))
    {
        string hex = j.at("base_color").get<string>();
        if (!hex.empty())
            ds.base_color_ = QColor(QString::fromStdString(hex));
    }

    if (j.contains("line_colors") && j.at("line_colors").is_array())
    {
        const auto& arr = j.at("line_colors");
        for (size_t i = 0; i < ds.line_colors_.size() && i < arr.size(); ++i)
        {
            string hex = arr.at(i).is_string() ? arr.at(i).get<string>() : string{};
            ds.line_colors_[i] = hex.empty() ? QColor() : QColor(QString::fromStdString(hex));
        }
    }

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
        && info_ == other.info_
        && base_color_ == other.base_color_
        && line_colors_ == other.line_colors_;
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

bool DataSource::hasRadarBias() const
{
    return info_.contains("radar_bias");
}

map<string, double> DataSource::radarBias() const
{
    if (!hasRadarBias()) return {};
    return info_.at("radar_bias").get<map<string, double>>();
}

void DataSource::radarBias(const string& key, double value)
{
    info_["radar_bias"][key] = value;
}

void DataSource::addRadarBiasIfMissing()
{
    if (!hasRadarBias())
        info_["radar_bias"] = json::object();
}

RadarBiasInfo DataSource::radarBiasInfo() const
{
    if (!hasRadarBias())
        return {};

    return RadarBiasInfo::fromMap(radarBias());
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
// Radar accuracy
// ============================================================

RadarPositionAccuracy DataSource::radarAccuracy(double range_m,
                                                double bearing_rad,
                                                DetectionType type,
                                                const RadarAccuracyDefaults& defaults) const
{
    // per-channel values: use configured if available, otherwise defaults
    // SMR (ground-only PSR) uses separate defaults
    bool is_ground = groundOnly();
    double psr_azm = is_ground ? defaults.primary_azimuth_stddev_ground : defaults.primary_azimuth_stddev;
    double psr_rng = is_ground ? defaults.primary_range_stddev_ground   : defaults.primary_range_stddev;
    double ssr_azm = defaults.secondary_azimuth_stddev;
    double ssr_rng = defaults.secondary_range_stddev;
    double ms_azm  = defaults.mode_s_azimuth_stddev;
    double ms_rng  = defaults.mode_s_range_stddev;

    if (hasRadarAccuracies())
    {
        auto acc = radarAccuracies();

        if (acc.count("primary_azimuth_stddev"))   psr_azm = acc.at("primary_azimuth_stddev");
        if (acc.count("primary_range_stddev"))      psr_rng = acc.at("primary_range_stddev");
        if (acc.count("secondary_azimuth_stddev"))  ssr_azm = acc.at("secondary_azimuth_stddev");
        if (acc.count("secondary_range_stddev"))    ssr_rng = acc.at("secondary_range_stddev");
        if (acc.count("mode_s_azimuth_stddev"))     ms_azm  = acc.at("mode_s_azimuth_stddev");
        if (acc.count("mode_s_range_stddev"))        ms_rng  = acc.at("mode_s_range_stddev");
    }

    // select channel based on detection type (min for combined)
    double azm_stddev_deg, rng_stddev_m;

    switch (type)
    {
        case DetectionType::PrimaryOnly:
            azm_stddev_deg = psr_azm;
            rng_stddev_m   = psr_rng;
            break;
        case DetectionType::ModeAC:
            azm_stddev_deg = ssr_azm;
            rng_stddev_m   = ssr_rng;
            break;
        case DetectionType::ModeACCombined:
            azm_stddev_deg = min(psr_azm, ssr_azm);
            rng_stddev_m   = min(psr_rng, ssr_rng);
            break;
        case DetectionType::ModeS:
            azm_stddev_deg = ms_azm;
            rng_stddev_m   = ms_rng;
            break;
        case DetectionType::ModeSCombined:
            azm_stddev_deg = min(psr_azm, ms_azm);
            rng_stddev_m   = min(psr_rng, ms_rng);
            break;
        case DetectionType::Undefined:
        default:
            azm_stddev_deg = psr_azm;
            rng_stddev_m   = psr_rng;
            break;
    }

    // convert azimuth stddev from degrees to meters at this range
    double azm_stddev_m = azm_stddev_deg * 2.0 * M_PI * range_m / 360.0;

    // polar covariance -> Cartesian via rotation
    double rng_var = rng_stddev_m * rng_stddev_m;
    double azm_var = azm_stddev_m * azm_stddev_m;

    double sin_b = sin(bearing_rad);
    double cos_b = cos(bearing_rad);

    // rotation matrix A = [sin(b) cos(b); cos(b) -sin(b)]
    // C_cart = A * diag(rng_var, azm_var) * A^T
    double xx = rng_var * sin_b * sin_b + azm_var * cos_b * cos_b;
    double yy = rng_var * cos_b * cos_b + azm_var * sin_b * sin_b;
    double xy = (rng_var - azm_var) * sin_b * cos_b;

    return {sqrt(xx), sqrt(yy), xy};
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
