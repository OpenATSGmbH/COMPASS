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

#include "json.hpp"

struct RemoteUnitDefinition
{
    int         index = -1;
    std::string name;
    std::string comment;
    double      latitude;
    double      longitude;
    double      altitude;
};

/**
 */
class DataSourceRemoteUnit
{
public:
    DataSourceRemoteUnit(nlohmann::json& config);
    ~DataSourceRemoteUnit();

    void configure(const RemoteUnitDefinition& def);

    int index() const;
    void index(int idx);

    std::string name() const;
    void name(const std::string& name);

    std::string comment() const;
    void comment(const std::string& comment);

    double latitude() const;
    void latitude(double lat);

    double longitude() const;
    void longitude(double lon);

    double altitude() const;
    void altitude(double alt);

    std::string asString() const;

    static const std::string KeyIndex;
    static const std::string KeyName;
    static const std::string KeyComment;
    static const std::string KeyLatitude;
    static const std::string KeyLongitude;
    static const std::string KeyAltitude;

private:
    nlohmann::json& config_;
};
