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

#include <string>

class FFTBase
{
public:
    FFTBase();

    std::string name() const;
    void name(const std::string &name);

    nlohmann::json& info();
    void info(const nlohmann::json& info);
    std::string infoStr();

    bool hasModeSAddress();
    unsigned int modeSAddress();
    void modeSAddress(unsigned int value);
    void clearModeSAddress();

    bool hasMode3ACode();
    unsigned int mode3ACode();
    void mode3ACode(unsigned int value);
    void clearMode3ACode();

    bool hasModeCCode();
    float modeCCode(); // ft
    void modeCCode(float value);
    void clearModeCCode();

    bool hasPosition() const;

    void latitude (double value);
    double latitude () const;

    void longitude (double value);
    double longitude () const;

    bool hasAltitude() const;
    void altitude (double value);
    double altitude () const; // ft

    void setFromJSON (const nlohmann::json& j);

    virtual nlohmann::json getAsJSON() const;

protected:
    std::string name_;

    nlohmann::json info_;

};



