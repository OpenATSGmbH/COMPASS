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

#include <boost/optional.hpp>

#include <string>
#include <utility>

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

    // --- secondary identification ---
    // The Mode S address and the Mode 3/A code identify an FFT in the data.
    // The Mode C code only confirms a match, it never establishes one.

    bool hasModeSAddress() const;
    unsigned int modeSAddress() const;
    void modeSAddress(unsigned int value);

    bool hasMode3ACode() const;
    unsigned int mode3ACode() const;
    void mode3ACode(unsigned int value);

    bool hasModeCCode() const;
    float modeCCode() const;
    void modeCCode(float value);

    /// True if at least one secondary identification value is set. An FFT
    /// without any such value can not be identified in the data.
    bool hasSecondaryIdentification() const;

    /**
     * Tests one target report against this FFT. Returns the match result and
     * the FFT altitude to be used for the Radar slant range correction.
     *
     * A match needs at least one secondary identification value that exists on
     * both sides and is equal. The position alone is never sufficient, since
     * aircraft can overfly the FFT position. Any value that exists on both
     * sides and differs vetoes the match.
     *
     * ignore_mode_s skips the Mode S address check for data that carries no
     * aircraft address, e.g. CAT001.
     */
    std::pair<bool, float> matches(double latitude_deg, double longitude_deg,
                                   boost::optional<unsigned int> mode_s_address,
                                   bool ignore_mode_s,
                                   boost::optional<unsigned int> mode_a_code,
                                   boost::optional<float> mode_c_code) const;

    /// The Mode 3/A code 7777 is reserved for SSR monitoring. A target report
    /// carrying it is always from an FFT, without any further check.
    static bool isAlwaysFFTCode(boost::optional<unsigned int> mode_a_code);

    /// Maximum distance between a target report position and the FFT position
    /// for a position match, in meters.
    static constexpr double max_plot_distance_m_ = 5000.0;

    /// Mode 3/A code 7777 octal, stored as the 12 bit value.
    static constexpr unsigned int always_fft_mode_3a_code_ = 4095;

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
