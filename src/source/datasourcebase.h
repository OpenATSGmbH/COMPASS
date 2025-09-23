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

#include <json.hpp>
#include "datasourcelineinfo.h"

#include <string>

namespace dbContent
{

enum class DataSourceType
{
    ADSB,
    MLAT,
    Radar,
    Tracker,
    RefTraj,
    Other
};

class DataSourceBase
{
public:
    enum class DetectionType
    {
        Undefined,
        PrimaryOnly,
        ModeAC,
        ModeACCombined,
        ModeS,
        ModeSCombined
    };

    static const std::string DetectionKey;
    static const std::string GroundOnlyKey;

    static const std::string PSRIRMinKey;
    static const std::string PSRIRMaxKey;
    static const std::string SSRIRMinKey;
    static const std::string SSRIRMaxKey;
    static const std::string ModeSIRMinKey;
    static const std::string ModeSIRMaxKey;

    static const std::string PSRAzmSDKey;
    static const std::string PSRRngSDKey;
    static const std::string SSRAzmSDKey;
    static const std::string SSRRngSDKey;
    static const std::string ModeSAzmSDKey;
    static const std::string ModeSRngSDKey;

    DataSourceBase();

    std::string dsType() const;
    void dsType(const std::string& ds_type);

    unsigned int sac() const;
    void sac(unsigned int sac);

    unsigned int sic() const;
    void sic(unsigned int sic);

    virtual unsigned int id() const; // from sac/sic

    std::string name() const;
    void name(const std::string &name);

    bool hasShortName() const;
    void removeShortName();
    void shortName(const std::string& short_name);
    const std::string& shortName() const;

    void info(const std::string& info);
    nlohmann::json& info(); // for direct use, var->value
    std::string infoStr();

    DetectionType detectionType() const;
    void detectionType(DetectionType type);

    bool groundOnly() const;
    void groundOnly(bool value);

    bool hasUpdateInterval() const;
    void removeUpdateInterval();
    void updateInterval (float value);
    float updateInterval () const;

    bool hasPosition() const;

    void latitude (double value);
    double latitude () const;

    void longitude (double value);
    double longitude () const;

    void altitude (double value);
    double altitude () const;

    bool hasRadarRanges() const;
    void addRadarRanges();
    std::map<std::string, double> radarRanges() const;
    void radarRange (const std::string& key, const double range);
    void removeRadarRange(const std::string& key);

    bool hasRadarAccuracies() const;
    void addRadarAccuracies();
    std::map<std::string, double> radarAccuracies() const;
    void radarAccuracy (const std::string& key, const double value);

    bool hasNetworkLines() const;
    void addNetworkLines();
    std::map<std::string, std::shared_ptr<DataSourceLineInfo>> networkLines() const;
    bool hasNetworkLine (const std::string& key) const;
    void createNetworkLine (const std::string& key);
    std::shared_ptr<DataSourceLineInfo> networkLine (const std::string& key); // creates if not exists

    void setFromJSONDeprecated (const nlohmann::json& j);
    void setFromJSON (const nlohmann::json& j);

    virtual nlohmann::json getAsJSON() const;

    bool isCalculatedReferenceSource();
    void setCalculatedReferenceSource();

    static std::string detectionTypeToString(DetectionType type);
    static DetectionType detectionTypeFromString(const std::string& str);

protected:
    std::string ds_type_;

    unsigned int sac_{0};
    unsigned int sic_{0};

    std::string name_;

    bool has_short_name_{false};
    std::string short_name_;

    nlohmann::json info_;

    std::map<std::string, std::shared_ptr<DataSourceLineInfo>> line_info_;

    void parseNetworkLineInfo();
};

}
