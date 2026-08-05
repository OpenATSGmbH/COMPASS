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

namespace context
{

/**
 * Unified FFT (Fixed Frequency Transmitter) definition for a DBContext.
 * Merges the old ConfigurationFFT / DBFFT split into a single class.
 */
class FFT
{
public:
    FFT();

    std::string name() const { return name_; }
    void name(const std::string& name) { name_ = name; }

    nlohmann::json& info() { return info_; }
    const nlohmann::json& info() const { return info_; }
    void info(const nlohmann::json& info) { info_ = info; }

    bool hasPosition() const;
    double latitude() const;
    void latitude(double value);
    double longitude() const;
    void longitude(double value);

    bool hasAltitude() const;
    double altitude() const;
    void altitude(double value);

    nlohmann::json toJSON() const;
    static FFT fromJSON(const nlohmann::json& j);

    bool operator==(const FFT& other) const;
    bool operator!=(const FFT& other) const { return !(*this == other); }

private:
    std::string name_;
    // always an object - erase() throws on a null json, and a newly created
    // FFT is edited before any info is set
    nlohmann::json info_ = nlohmann::json::object();
};

} // namespace context
