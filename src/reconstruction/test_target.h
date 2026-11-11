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

#include <vector>
#include <string>

#include "json_fwd.hpp"

#include <boost/optional.hpp>

/**
*/
class TestTarget
{
public:
    struct Measurement
    {
        double x;
        double y;
        double vx;
        double vy;
        double ax;
        double ay;
        double t;

        double dirx; 
        double diry;

        boost::optional<double> x_stddev;
        boost::optional<double> y_stddev;
        boost::optional<double> xy_cov;
        boost::optional<double> vx_stddev;
        boost::optional<double> vy_stddev;
        boost::optional<double> ax_stddev;
        boost::optional<double> ay_stddev;
    };

    struct Settings
    {
        double dt    = 1.0;
        double noise = 0.0; 
    };

    TestTarget();
    virtual ~TestTarget();

    bool exportJSON(const std::string& fn) const;
    bool exportJSON(nlohmann::json& j) const;
    bool importJSON(const std::string& fn);

    void clear();
    void addMeasurement(const Measurement& mm);

    void reset(double x = 0.0, double y = 0.0, double dirx = 1.0, double diry = 0.0);

    void moveConstant(unsigned int steps);
    void moveAccelerated(double accel, unsigned int steps);
    void turnConstant(double dtheta_deg, unsigned int steps);
    void turnAccelerated(double dtheta_deg, double accel, unsigned int steps);

    const std::vector<Measurement>& measurements() const { return measurements_; }

    Settings& settings() { return settings_; }

private:
    void addMeasurement();

    double x_;
    double y_;
    double dirx_;
    double diry_;
    double v_;
    double a_;
    double t_;

    std::vector<Measurement> measurements_;

    Settings settings_;
};
