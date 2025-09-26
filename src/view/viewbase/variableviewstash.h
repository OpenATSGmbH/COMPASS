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

#include <vector>
#include <map>
#include <cmath>

#include <boost/optional.hpp>

/**
*/
template <typename T>
struct VariableStash
{
    unsigned int   count = 0;
    std::vector<T> values;
};

/**
*/
template <typename T>
struct GroupedDataStash
{
    bool isValid() const
    {
        //check if array sizes still match
        size_t n = selected_values.size();

        for (const auto& s : variable_stashes)
            if (s.values.size() != n)
                return false;

        if (record_numbers.size() != n)
            return false;

        return true;
    }

    void updateCounts()
    {
        traced_assert(isValid());

        size_t n  = selected_values.size(); 
        size_t nv = variable_stashes.size();

        nan_values.assign(n, false);

        valid_count = 0;
        nan_count   = 0;
        sel_count   = 0;
        unsel_count = 0;

        //update counts and nan-array
        for (size_t i = 0; i < n; ++i)
        {
            bool is_nan = false;
            for (size_t j = 0; j < nv; ++j)
                is_nan |= isNan(j, i);

            nan_values[ i ] = is_nan;

            if (is_nan)
                ++nan_count;
            else if (selected_values[ i ])
                ++sel_count;
            else
                ++unsel_count;
        }

        valid_count = sel_count + unsel_count;
    }

    void freeStashData()
    {
        for (auto& s : variable_stashes)
            s.values.resize(0);

        selected_values.resize(0);
        nan_values.resize(0);
        record_numbers.resize(0);
    }

    size_t size() const
    {
        return selected_values.size();
    }

    size_t numVariables() const
    {
        return variable_stashes.size();
    }

    bool isNan(size_t var_idx, size_t idx) const
    {
        return std::isnan(variable_stashes[ var_idx ].values[ idx ]);
    }

    bool isNan(size_t idx) const
    {
        size_t nv = numVariables();
        bool is_nan = false;
        for (size_t j = 0; j < nv; ++j)
            is_nan |= isNan(j, idx);

        return is_nan;
    }

    std::vector<VariableStash<T>> variable_stashes;
    std::vector<bool>             selected_values;
    std::vector<bool>             nan_values;
    std::vector<unsigned long>    record_numbers;

    unsigned int                  valid_count = 0;
    unsigned int                  nan_count   = 0;
    unsigned int                  sel_count   = 0;
    unsigned int                  unsel_count = 0;
};

/**
*/
template <typename T>
class VariableViewStash
{
public:
    typedef std::pair<double, double>  DataRange;
    typedef boost::optional<DataRange> OptionalDataRange;

    VariableViewStash(size_t num_variables)
    :   num_variables_(num_variables)
    {
        traced_assert(num_variables_ > 0);

        data_ranges_.resize(num_variables_);
    }

    const std::vector<OptionalDataRange>& dataRanges() const { return data_ranges_; }
    const std::map<std::string, GroupedDataStash<T>> groupedStashes() const { return grouped_stashes_; }

    bool hasData() const
    {
        return !grouped_stashes_.empty();
    }

    GroupedDataStash<T>& groupedDataStash(const std::string& group_name)
    {
        //stash already init?
        if (grouped_stashes_.count(group_name))
            return grouped_stashes_.at(group_name);

        //init new stash for dbcontent
        auto& s = grouped_stashes_[ group_name ];
        s.variable_stashes.resize(num_variables_);

        return s;
    }

    const GroupedDataStash<T>& groupStash(const std::string& group_name) const
    {
        return grouped_stashes_.at(group_name);
    }

    bool hasGroupStash(const std::string& group_name) const
    {
        return grouped_stashes_.count(group_name) != 0;
    }

    void reset()
    {
        grouped_stashes_.clear();

        nan_value_count_  = 0;
        valid_count_      = 0;
        selected_count_   = 0;
        unselected_count_ = 0;

        for (auto& dr : data_ranges_)
            dr.reset();
    }

    bool isValid() const
    {
        //check if each dbcontent stash is still valid
        for (const auto& dbc_stash : grouped_stashes_)
            if (!dbc_stash.second.isValid())
                return false;

        return true;
    }

    void updateDataRanges()
    {
        for (size_t i = 0; i < num_variables_; ++i)
        {
            //reset variable range
            auto& data_range = data_ranges_[ i ];
            data_range.reset();

            //update variable range
            for (const auto& dbc : grouped_stashes_)
            {
                const auto& values = dbc.second.variable_stashes[ i ].values;
                for (const auto& v : values)
                {
                    if (!std::isfinite(v))
                        continue;

                    if (!data_range.has_value())
                    {
                        data_range = DataRange(v, v);
                        continue;
                    }
                    if (v < data_range.value().first)
                        data_range.value().first = v;
                    if (v > data_range.value().second)
                        data_range.value().second = v;
                }
            }
        }
    }

    void updateCounts()
    {
        //update dbcontent counts
        for (auto& dbc_stash : grouped_stashes_)
            dbc_stash.second.updateCounts();

        //accumulate individual counts
        nan_value_count_  = 0;
        valid_count_      = 0;
        selected_count_   = 0;
        unselected_count_ = 0;

        for (const auto& dbc : grouped_stashes_)
        {
            nan_value_count_  += dbc.second.nan_count;
            valid_count_      += dbc.second.valid_count;
            selected_count_   += dbc.second.sel_count;
            unselected_count_ += dbc.second.unsel_count;
        }
    }

    void update()
    {
        updateDataRanges();
        updateCounts();
    }

    void freeStashData()
    {
        for (auto& dbc_stash : grouped_stashes_)
            dbc_stash.second.freeStashData();
    }

    unsigned int nan_value_count_   = 0;
    unsigned int valid_count_       = 0;
    unsigned int selected_count_    = 0;
    unsigned int unselected_count_  = 0;

private:
    std::map<std::string, GroupedDataStash<T>> grouped_stashes_;
    std::vector<OptionalDataRange>             data_ranges_;

    size_t num_variables_ = 0;
};
