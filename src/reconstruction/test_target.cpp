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

#include "test_target.h"
#include "files.h"
#include "traced_assert.h"

#include "logger.h"
#include "json.hpp"

#include <cmath>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Dense>

/**
*/
TestTarget::TestTarget()
{
    reset();
}

/**
*/
TestTarget::~TestTarget() = default;

/**
*/
void TestTarget::addMeasurement()
{
    Measurement mm;
    mm.x    = x_;
    mm.y    = y_;
    mm.vx   = dirx_ * v_;
    mm.vy   = diry_ * v_;
    mm.ax   = dirx_ * a_;
    mm.ay   = diry_ * a_;
    mm.t    = t_;
    mm.dirx = dirx_;
    mm.diry = diry_;

    if (settings_.noise > 0.0)
    {
        auto N = Eigen::VectorXd::Random(6);

        const double noise2 = settings_.noise * settings_.noise;
        const double noise3 = settings_.noise * noise2;

        mm.x  += N[ 0 ] * settings_.noise;
        mm.y  += N[ 1 ] * settings_.noise;
        mm.vx += N[ 2 ] * noise2;
        mm.vy += N[ 3 ] * noise2;
        mm.ax += N[ 4 ] * noise3;
        mm.ay += N[ 5 ] * noise3;
    }

    measurements_.push_back(mm);
}

/**
*/
void TestTarget::reset(double x, double y, double dirx, double diry)
{
    measurements_.clear();

    x_    = x;
    y_    = y;
    dirx_ = dirx;
    diry_ = diry;
    v_    = 0.0;
    a_    = 0.0;
    t_    = 0.0;

    const double norm = std::sqrt(dirx * dirx + diry * diry);
    traced_assert(norm > 1e-09);

    dirx_ /= norm;
    diry_ /= norm;

    addMeasurement();
}

/**
*/
void TestTarget::clear()
{
    reset();
    measurements_.clear();
}

/**
*/
void TestTarget::addMeasurement(const Measurement& mm)
{
    measurements_.push_back(mm);
}

/**
*/
void TestTarget::moveConstant(unsigned int steps)
{
    a_ = 0;

    for (unsigned int i = 0; i < steps; ++i)
    {
        t_ += settings_.dt;

        x_ += settings_.dt * dirx_ * v_;
        y_ += settings_.dt * diry_ * v_;

        addMeasurement();
    }
}

/**
*/
void TestTarget::moveAccelerated(double accel, unsigned int steps)
{
    a_ = accel;

    const double dt2 = settings_.dt * settings_.dt;

    for (unsigned int i = 0; i < steps; ++i)
    {
        t_ += settings_.dt;

        x_ += settings_.dt * dirx_ * v_ + 0.5 * dt2 * dirx_ * a_;
        y_ += settings_.dt * diry_ * v_ + 0.5 * dt2 * diry_ * a_;
        v_ += settings_.dt * a_;

        addMeasurement();
    }
}

/**
*/
void TestTarget::turnConstant(double dtheta_deg, unsigned int steps)
{
    a_ = 0;

    double theta  = std::atan2(diry_, dirx_);
    double dtheta = dtheta_deg * M_PI / 180.0;

    for (unsigned int i = 0; i < steps; ++i)
    {
        t_ += settings_.dt;

        theta += dtheta * settings_.dt;

        dirx_ = std::cos(theta);
        diry_ = std::sin(theta);

        x_ += settings_.dt * dirx_ * v_;
        y_ += settings_.dt * diry_ * v_;

        addMeasurement();
    }
}

/**
*/
void TestTarget::turnAccelerated(double dtheta_deg, double accel, unsigned int steps)
{
    a_ = accel;

    double theta  = std::atan2(diry_, dirx_);
    double dtheta = dtheta_deg * M_PI / 180.0;

    const double dt2 = settings_.dt * settings_.dt;

    for (unsigned int i = 0; i < steps; ++i)
    {
        t_ += settings_.dt;

        theta += dtheta * settings_.dt;

        dirx_ = std::cos(theta);
        diry_ = std::sin(theta);

        x_ += settings_.dt * dirx_ * v_ + 0.5 * dt2 * dirx_ * a_;
        y_ += settings_.dt * diry_ * v_ + 0.5 * dt2 * diry_ * a_;
        v_ += settings_.dt * a_;

        addMeasurement();
    }
}

/**
*/
bool TestTarget::exportJSON(nlohmann::json& j) const
{
    j = nlohmann::json::array();

    for (const auto& mm : measurements_)
    {
        nlohmann::json jmm;
        jmm[ "x"    ] = mm.x;
        jmm[ "y"    ] = mm.y;
        jmm[ "vx"   ] = mm.vx;
        jmm[ "vy"   ] = mm.vy;
        jmm[ "ax"   ] = mm.ax;
        jmm[ "ay"   ] = mm.ay;
        jmm[ "t"    ] = mm.t;
        jmm[ "dirx" ] = mm.dirx;
        jmm[ "diry" ] = mm.diry;

        if (mm.x_stddev.has_value() ) jmm[ "x_stddev"  ] = mm.x_stddev.value();
        if (mm.y_stddev.has_value() ) jmm[ "y_stddev"  ] = mm.y_stddev.value();
        if (mm.xy_cov.has_value()   ) jmm[ "xy_cov"    ] = mm.xy_cov.value();
        if (mm.vx_stddev.has_value()) jmm[ "vx_stddev" ] = mm.vx_stddev.value();
        if (mm.vy_stddev.has_value()) jmm[ "vy_stddev" ] = mm.vy_stddev.value();
        if (mm.ax_stddev.has_value()) jmm[ "ax_stddev" ] = mm.ax_stddev.value();
        if (mm.ay_stddev.has_value()) jmm[ "ay_stddev" ] = mm.ay_stddev.value();

        j.push_back(jmm);
    }

    return true;
}

/**
*/
bool TestTarget::exportJSON(const std::string& fn) const
{
    nlohmann::json j;
    if (!exportJSON(j))
        return false;

    std::string data = j.dump(4);

    std::ofstream os(fn, std::ios::out);
    if (!os.is_open())
        return false;

    os << data;
    if (!os)
        return false;

    os.close();

    return true;
}

/**
*/
bool TestTarget::importJSON(const std::string& fn)
{
    reset();

    if (!Utils::Files::fileExists(fn))
    {
        logerr << "file does not exist";
        return false;
    }

    std::ifstream is(fn, std::ios::in);
    if (!is.is_open())
    {
        logerr << "bad stream";
        return false;
    }

    nlohmann::json j;
    is >> j;

    if (!j.is_array())
    {
        logerr << "invalid json root";
        return false;
    }

    if (j.size() == 0)
    {
        logerr << "no measurements";
        return false;
    }

    measurements_.clear();

    for (const auto& mm_j : j)
    {
        Measurement mm;
        mm.x    = mm_j[ "x"     ];
        mm.y    = mm_j[ "y"     ];
        mm.vx   = mm_j[ "vx"    ];
        mm.vy   = mm_j[ "vy"    ];
        mm.ax   = mm_j[ "ax"    ];
        mm.ay   = mm_j[ "ay"    ];
        mm.t    = mm_j[ "t"     ];
        mm.dirx = mm_j[ "dirx"  ];
        mm.diry = mm_j[ "diry"  ];

        if (mm_j.contains("x_stddev" ) && !mm_j[ "x_stddev"  ].is_null()) mm.x_stddev  = mm_j[ "x_stddev"  ];
        if (mm_j.contains("y_stddev" ) && !mm_j[ "y_stddev"  ].is_null()) mm.y_stddev  = mm_j[ "y_stddev"  ];
        if (mm_j.contains("xy_cov"   ) && !mm_j[ "xy_cov"    ].is_null()) mm.xy_cov    = mm_j[ "xy_cov"    ];
        if (mm_j.contains("vx_stddev") && !mm_j[ "vx_stddev" ].is_null()) mm.vx_stddev = mm_j[ "vx_stddev" ];
        if (mm_j.contains("vy_stddev") && !mm_j[ "vy_stddev" ].is_null()) mm.vy_stddev = mm_j[ "vy_stddev" ];
        if (mm_j.contains("ax_stddev") && !mm_j[ "ax_stddev" ].is_null()) mm.ax_stddev = mm_j[ "ax_stddev" ];
        if (mm_j.contains("ay_stddev") && !mm_j[ "ay_stddev" ].is_null()) mm.ay_stddev = mm_j[ "ay_stddev" ];

        measurements_.push_back(mm);
    }

    const auto& mm_last = measurements_.back();

    double v = std::sqrt(mm_last.vx * mm_last.vx + mm_last.vy * mm_last.vy);
    double a = std::sqrt(mm_last.ax * mm_last.ax + mm_last.ay * mm_last.ay);

    x_    = mm_last.x;
    y_    = mm_last.y;
    dirx_ = mm_last.dirx;
    diry_ = mm_last.diry;
    v_    = v;
    a_    = a;
    t_    = mm_last.t;
    
    return true;
}
