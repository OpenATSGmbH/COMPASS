
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

#include "valueaccumulator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "traced_assert.h"

/**
*/
double ValueAccumulator::min() const
{
    return value_min_;
}

/**
*/
double ValueAccumulator::max() const
{
    return value_max_;
}

/**
*/
double ValueAccumulator::mean() const
{
    return value_avg_;
}

/**
*/
double ValueAccumulator::rms() const
{
    return value_rms_;
}

/**
*/
double ValueAccumulator::var() const
{
    return value_var_;
}

/**
*/
double ValueAccumulator::stddev() const
{
    return value_stddev_;
}

/**
*/
size_t ValueAccumulator::numValues() const
{
    return value_cnt_;
}

/**
*/
void ValueAccumulator::reset()
{
    value_sum_     = 0;
    value_sum_sqr_ = 0;

    value_min_     = 0;
    value_max_     = 0;
    value_avg_     = 0;
    value_rms_     = 0;
    value_var_     = 0;
    value_stddev_  = 0;
    value_cnt_     = 0;
}

/**
*/
void ValueAccumulator::accumulate(const std::vector<double>& values, bool update_stats)
{
    if (!values.empty())
    {
        auto vmin = *std::min_element(values.begin(), values.end());
        auto vmax = *std::max_element(values.begin(), values.end());
        auto sum  = std::accumulate(values.begin(), values.end(), 0.0);

        if (value_cnt_ == 0 || vmin < value_min_) value_min_ = vmin;
        if (value_cnt_ == 0 || vmax > value_max_) value_max_ = vmax;

        value_sum_ += sum;

        for (const auto& v : values)
            value_sum_sqr_ += std::pow(v, 2);

        value_cnt_ += values.size();
    }

    if (update_stats)
        updateStats();
}

/**
*/
bool ValueAccumulator::updateStats()
{
    if (value_cnt_ == 0)
        return false;

    value_avg_    = value_sum_     / (double)value_cnt_;
    value_var_    = value_sum_sqr_ / (double)value_cnt_;
    value_rms_    = value_var_;
    value_var_   -= value_avg_ * value_avg_;
    value_rms_    = std::sqrt(value_rms_);
    value_stddev_ = std::sqrt(value_var_);

    return true;
}

/**
*/
void ValueAccumulator::join(const ValueAccumulator& other, bool update_stats)
{
    value_sum_     += other.value_sum_;
    value_sum_sqr_ += other.value_sum_sqr_;
    value_cnt_     += other.value_cnt_;

    if (other.value_cnt_ > 0 && (value_cnt_ == 0 || other.value_min_ < value_min_)) value_min_ = other.value_min_;
    if (other.value_cnt_ > 0 && (value_cnt_ == 0 || other.value_max_ > value_max_)) value_max_ = other.value_max_;

    if (update_stats)
        updateStats();
}
