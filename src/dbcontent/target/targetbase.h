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

#include "traced_assert.h"
#include "json.hpp"

#include "logger.h"

#include <boost/optional.hpp>

#include <string>


class TargetBase
{

public:

    enum class Category // slightly related to ADS-B ECAT
    {
        Unknown = 0,

        // Light general aviation aircraft (e.g., Cessna 152, Piper Cub)
        LightAircraft = 1,

        // Small high-performance piston or turboprop aircraft (e.g., Beechcraft Bonanza, Piper Malibu)
        SmallAircraft = 2,

        // Regional jets and smaller airliners (e.g., Embraer E175, Bombardier CRJ700)
        MediumAircraft = 3,

        // Large wake-generating aircraft (e.g., Airbus A310, Boeing 757)
        HighVortexLargeAircraft = 4,

        // Large commercial airliners (e.g., Boeing 777, Airbus A380)
        HeavyAircraft = 5,

        // High-speed fighter or interceptor jets (e.g., F-16, Eurofighter Typhoon)
        HighSpeedManoeuvrable = 6,

        // Helicopters and VTOL aircraft (e.g., Bell 412, Eurocopter EC135)
        Rotocraft = 10,

        // Gliders, UAVs, ultralights, lighter-than-air, etc. (e.g., DJI Matrice 300, ASH 31 Mi, Paramotor)
        OtherAirborne = 11,

        // other aircraft, not from ECAT but from mode C
        AnyAircraft = 12,

        // Ground vehicles
        Vehicle = 20,
        Obstacle = 22, // obstruction or obstacles
        FFT = 25
    };

    TargetBase();
    virtual ~TargetBase() {}

    virtual void targetCategory(Category ecat)=0;
    virtual Category targetCategory() const=0;
    std::string emitterCategoryStr() const;

    // Convert ADS-B ECAT (optional) to EmitterCategory enum
    static Category fromECAT(boost::optional<unsigned int> ecat) {
        if (!ecat) return Category::Unknown;

        switch (*ecat) {
        case 1: return Category::LightAircraft;
        case 2: return Category::SmallAircraft;
        case 3: return Category::MediumAircraft;
        case 4: return Category::HighVortexLargeAircraft;
        case 5: return Category::HeavyAircraft;
        case 6: return Category::HighSpeedManoeuvrable;
        case 10: return Category::Rotocraft;
        case 11: case 12: case 13: case 14: case 15: case 16:
            return Category::OtherAirborne;
        case 20: case 21:
            return Category::Vehicle;
        case 22: case 23: case 24:
            return Category::Obstacle;
        case 25:
            return Category::FFT; // not original in CAT021, extended using FFTManager
        default: return Category::Unknown;
        }
    }

    // String representation of EmitterCategory
    static std::string toString(Category ecat) {
        switch (ecat) {
        case Category::LightAircraft: return "LightAircraft";
        case Category::SmallAircraft: return "SmallAircraft";
        case Category::MediumAircraft: return "MediumAircraft";
        case Category::HighVortexLargeAircraft: return "HighVortexLargeAircraft";
        case Category::HeavyAircraft: return "HeavyAircraft";
        case Category::HighSpeedManoeuvrable: return "HighSpeedManoeuvrable";
        case Category::Rotocraft: return "Rotocraft";
        case Category::OtherAirborne: return "OtherAirborne";
        case Category::AnyAircraft: return "AnyAircraft";
        case Category::Vehicle: return "Vehicle";
        case Category::Obstacle: return "Obstacle";
        case Category::FFT: return "FFT";
        default: return "Unknown";
        }
    }

    static Category fromString(const std::string& name) {
        static const std::unordered_map<std::string, Category> str_to_category = {
            { "LightAircraft", Category::LightAircraft },
            { "SmallAircraft", Category::SmallAircraft },
            { "MediumAircraft", Category::MediumAircraft },
            { "HighVortexLargeAircraft", Category::HighVortexLargeAircraft },
            { "HeavyAircraft", Category::HeavyAircraft },
            { "HighSpeedManoeuvrable", Category::HighSpeedManoeuvrable },
            { "Rotocraft", Category::Rotocraft },
            { "OtherAirborne", Category::OtherAirborne },
            { "AnyAircraft", Category::AnyAircraft },
            { "Vehicle", Category::Vehicle },
            { "Obstacle", Category::Obstacle },
            { "FFT", Category::FFT },
            { "Unknown", Category::Unknown }
        };

        auto it = str_to_category.find(name);
        if (it != str_to_category.end()) {
            return it->second;
        }
        return Category::Unknown;
    }

    static bool checkEmitterSpecs(Category ecat)
    {
        if (!emitter_specs_.count(toString(ecat)))
            logerr << "TargetBase:: checkEmitterSpecs: unknown ecat " << (unsigned int) ecat;
        traced_assert(emitter_specs_.count(toString(ecat)));
        return true;
    }

    // Accessor helpers
    static double getAvgSize(Category ecat) {

        //loginf << (unsigned int) ecat << " str " << toString(ecat);
        traced_assert(checkEmitterSpecs(ecat));
        traced_assert(emitter_specs_.at(toString(ecat)).count("avg_size_m"));
        return emitter_specs_.at(toString(ecat)).at("avg_size_m");
    }

    static double getMaxSpeedKnots(Category ecat) {
        traced_assert(checkEmitterSpecs(ecat));
        traced_assert(emitter_specs_.at(toString(ecat)).count("max_speed_knots"));
        return emitter_specs_.at(toString(ecat)).at("max_speed_knots");
    }

    static double getMaxAccel(Category ecat) {
        traced_assert(checkEmitterSpecs(ecat));
        traced_assert(emitter_specs_.at(toString(ecat)).count("max_accel_mps2"));
        return emitter_specs_.at(toString(ecat)).at("max_accel_mps2");
    }

    static bool isGroundOnly(Category ecat) {
        traced_assert(checkEmitterSpecs(ecat));
        traced_assert(emitter_specs_.at(toString(ecat)).count("ground_only"));
        return emitter_specs_.at(toString(ecat)).at("ground_only");
    }

    static double processNoiseFactorGround(Category ecat) {
        traced_assert(checkEmitterSpecs(ecat));
        traced_assert(emitter_specs_.at(toString(ecat)).count("process_noise_factor_ground"));
        return emitter_specs_.at(toString(ecat)).at("process_noise_factor_ground");
    }

    static double processNoiseFactorAir(Category ecat) {
        traced_assert(checkEmitterSpecs(ecat));
        traced_assert(emitter_specs_.at(toString(ecat)).count("process_noise_factor_air"));
        return emitter_specs_.at(toString(ecat)).at("process_noise_factor_air");
    }

    static const nlohmann::json emitter_specs_;
};
