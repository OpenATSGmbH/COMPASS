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

#include "radar_accuracy_defs.h"

#include <json.hpp>

#include <map>
#include <string>

namespace context
{

/**
 * Unified data source (sensor) definition for a DBContext.
 * Merges the old ConfigurationDataSource / DBDataSource split into a single class.
 * Runtime-only state (counts, loading_wanted) is NOT part of this class.
 */
class DataSource
{
public:
    DataSource();

    std::string dsType() const { return ds_type_; }
    void dsType(const std::string& ds_type) { ds_type_ = ds_type; }

    unsigned int sac() const { return sac_; }
    void sac(unsigned int sac) { sac_ = sac; }

    unsigned int sic() const { return sic_; }
    void sic(unsigned int sic) { sic_ = sic; }

    unsigned int id() const; // computed from sac/sic

    std::string name() const { return name_; }
    void name(const std::string& name) { name_ = name; }

    bool hasShortName() const { return has_short_name_; }
    std::string shortName() const { return short_name_; }
    void shortName(const std::string& short_name);
    void removeShortName();

    nlohmann::json& info() { return info_; }
    const nlohmann::json& info() const { return info_; }
    void info(const nlohmann::json& info) { info_ = info; }

    // convenience accessors for common info fields (mirrors DataSourceBase API)
    bool hasPosition() const;
    double latitude() const;
    void latitude(double value);
    double longitude() const;
    void longitude(double value);
    double altitude() const;
    void altitude(double value);

    bool groundOnly() const;
    void groundOnly(bool value);

    bool hasUpdateInterval() const;
    float updateInterval() const;
    void updateInterval(float value);
    void removeUpdateInterval();

    int detectionTypeInt() const;
    void detectionTypeInt(int value);

    bool ignoreRadarAzmRange() const;
    void ignoreRadarAzmRange(bool value);

    double probabilityOfDetection() const;
    void probabilityOfDetection(double value);

    double clutterRate() const;
    void clutterRate(double value);

    bool hasRadarRanges() const;
    std::map<std::string, double> radarRanges() const;
    void radarRange(const std::string& key, double value);
    void removeRadarRange(const std::string& key);
    void addRadarRangesIfMissing();

    bool hasRadarAccuracies() const;
    std::map<std::string, double> radarAccuracies() const;
    void radarAccuracy(const std::string& key, double value);
    void addRadarAccuraciesIfMissing();

    bool hasRadarBias() const;
    std::map<std::string, double> radarBias() const;
    void radarBias(const std::string& key, double value);
    void addRadarBiasIfMissing();

    RadarPositionAccuracy radarAccuracy(double range_m,
                                        double bearing_rad,
                                        DetectionType type,
                                        const RadarAccuracyDefaults& defaults) const;

    bool hasNetworkLines() const;
    void addNetworkLinesIfMissing();

    bool hasRemoteUnits() const;
    bool hasRemoteUnit(int index) const;
    std::string remoteUnitName(int index) const;
    bool remoteUnitIsValid(int index) const;
    double remoteUnitLatitude(int index) const;
    double remoteUnitLongitude(int index) const;
    double remoteUnitAltitude(int index) const;
    void addRemoteUnitsIfMissing();
    void removeRemoteUnits();
    void removeRemoteUnit(int index);

    bool isCalculatedReferenceSource() const;
    void setCalculatedReferenceSource();

    nlohmann::json toJSON() const;
    static DataSource fromJSON(const nlohmann::json& j);

    // static DS type utilities (migrated from DataSourceManager)
    static const std::vector<std::string>& dsTypeStrings();
    static std::string dsTypeToString(int type_enum);
    static int dsTypeFromString(const std::string& type_str);

    bool operator==(const DataSource& other) const;
    bool operator!=(const DataSource& other) const { return !(*this == other); }

private:
    std::string ds_type_;
    unsigned int sac_{0};
    unsigned int sic_{0};
    std::string name_;
    bool has_short_name_{false};
    std::string short_name_;
    nlohmann::json info_;
};

} // namespace context
