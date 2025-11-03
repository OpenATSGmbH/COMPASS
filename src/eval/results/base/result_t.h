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

#include "eval/results/evaluationdetail.h"
#include "eval/results/base/base.h"
#include "traced_assert.h"

namespace EvaluationRequirementResult
{

/**
 * Retrieves a value from a detail,
 * either by a value id or custom functional.
*/
template <typename T>
struct ValueSource
{
    typedef std::function<boost::optional<T>(const EvaluationDetail& detail)> GetValueFunc;
    typedef std::function<T(const T&)> ValueTransformFunc;

    ValueSource() = default;
    ValueSource(int id) : value_id(id) {}
    ValueSource(const GetValueFunc& func) : value_func(func) {}

    bool isValid() const
    {
        return (value_id >= 0 || value_func);
    }

    /**
     * Obtains the value from the given detail.
     * The value will be casted to T, which might fail.
    */
    boost::optional<T> valueFromDetail(const EvaluationDetail& detail) const
    {
        boost::optional<T> v = value_func ? value_func(detail) : valueFromID(detail);
        if (!v.has_value())
            return {};

        if (value_tr_func)
            v = value_tr_func(v.value());

        return v;
    }

    int                value_id = -1;  // id of a detail value
    GetValueFunc       value_func;     // functional for custom retrieval
    ValueTransformFunc value_tr_func;  // optional transformation for retrieved value

private:
    /**
     * Get value from detail's QVariant.
     * The value will be casted to T, which might fail.
    */
    boost::optional<T> valueFromID(const EvaluationDetail& detail) const
    {
        //default: just try to cast
        return detail.getValueAs<T>(value_id);
    }
};

/**
 * Get std::string from detail's QVariant.
*/
template<>
inline boost::optional<std::string> ValueSource<std::string>::valueFromID(const EvaluationDetail& detail) const
{
    //obtain as QString
    auto v = detail.getValueAs<QString>(value_id);
    if (!v)
        return {};

    return v.value().toStdString();
}

/**
 * Lat/lon position plus value.
*/
template <typename T>
struct PosValue
{
    PosValue() = default;
    PosValue(double lat, double lon, const T& v) : latitude(lat), longitude(lon), value(v) {}

    double latitude;
    double longitude;
    T      value;
};

/**
 * Time plus value.
*/
template <typename T>
struct TimedValue
{
    TimedValue() = default;
    TimedValue(const boost::posix_time::ptime& t, const T& v) : timestamp(t), value(v) {}

    boost::posix_time::ptime timestamp;
    T                        value;
};

/**
 * Time in seconds plus value.
*/
template <typename T>
struct MSecTimedValue
{
    MSecTimedValue() = default;
    MSecTimedValue(double time_msecs, const T& v) : t_msecs(time_msecs), value(v) {}

    double t_msecs;
    T      value;
};

/**
 * Collection of templated data retrieval functions for an evaluation result.
*/
class EvaluationResultTemplates
{
public:
    EvaluationResultTemplates(const Base* result) 
    :   result_ (result )
    { 
        traced_assert(result_); 
    }
    virtual ~EvaluationResultTemplates() = default;

    /**
     * Obtains values from all details using the passed value source.
     * The values will be casted to T, which might fail.
    */
    template <typename T>
    std::vector<T> getValues(const ValueSource<T>& source) const
    {
        traced_assert(source.isValid());

        std::vector<T> values;
        values.reserve(result_->totalNumDetails());

        auto func = [ & ] (const EvaluationDetail& detail, 
                           const EvaluationDetail* parent_detail, 
                           int didx0, 
                           int didx1,
                           int evt_pos_idx, 
                           int evt_ref_pos_idx)
        {
            //get detail value from source
            auto v = source.valueFromDetail(detail);

            //value might not be set => skip
            if (!v.has_value())
                return;

            //collect converted value
            values.push_back(v.value());
        };

        result_->iterateDetails(func, {});

        values.shrink_to_fit();

        return values;
    }

    /**
     * Obtains values from all details using the passed value source.
     * The values will be casted to T, which might fail.
    */
    template <typename T>
    std::vector<boost::optional<T>> getOptionalValues(const ValueSource<T>& source) const
    {
        traced_assert(source.isValid());

        std::vector<boost::optional<T>> values;
        values.reserve(result_->totalNumDetails());

        auto func = [ & ] (const EvaluationDetail& detail, 
                           const EvaluationDetail* parent_detail, 
                           int didx0, 
                           int didx1,
                           int evt_pos_idx, 
                           int evt_ref_pos_idx)
        {
            //get detail value from source
            auto v = source.valueFromDetail(detail);

            //collect converted value
            values.push_back(v);
        };

        result_->iterateDetails(func, {});

        values.shrink_to_fit();

        return values;
    }

    /**
     * Obtains values plus their timestamp from all details using the passed value source.
     * The values will be casted to T, which might fail.
    */
    template <typename T>
    std::vector<TimedValue<T>> getTimedValues(const ValueSource<T>& source) const
    {
        traced_assert(source.isValid());

        std::vector<TimedValue<T>> values;
        values.reserve(result_->totalNumDetails());

        auto func = [ & ] (const EvaluationDetail& detail, 
                           const EvaluationDetail* parent_detail, 
                           int didx0, 
                           int didx1,
                           int evt_pos_idx, 
                           int evt_ref_pos_idx)
        {
            //get detail value from source
            auto v = source.valueFromDetail(detail);

            //value might not be set => skip
            if (!v.has_value())
                return;

            //collect converted value
            values.emplace_back(detail.timestamp(), v.value());
        };

        result_->iterateDetails(func, {});

        values.shrink_to_fit();

        return values;
    }

    template <typename T>
    std::vector<MSecTimedValue<T>> getMSecTimedValues(const ValueSource<T>& source) const
    {
        auto tvalues = getTimedValues<T>(source);

        std::sort(tvalues.begin(), tvalues.end(), [ & ] (const TimedValue<T>& v0, const TimedValue<T>& v1) { return v0.timestamp < v1.timestamp; });

        size_t n = tvalues.size();

        std::vector<MSecTimedValue<T>> values(n);

        for (size_t i = 0; i < n; ++i)
            values[ i ] = MSecTimedValue<T>((double)Utils::Time::toLong(tvalues[ i ].timestamp), tvalues[ i ].value);

        return values;
    }

    /**
     * Obtains values plus their respective positions from all details using the passed value source.
     * The values will be casted to T, which might fail.
    */
    template <typename T>
    std::vector<PosValue<T>> getValuesPlusPosition(const ValueSource<T>& source,
                                                   DetailValuePositionMode detail_pos_mode = DetailValuePositionMode::EventPosition,
                                                   std::vector<std::pair<size_t,size_t>>* detail_ranges = nullptr) const
    {
        traced_assert(source.isValid());

        if (detail_ranges)
            detail_ranges->clear();

        bool single_pos = detail_pos_mode == DetailValuePositionMode::EventPosition || 
                          detail_pos_mode == DetailValuePositionMode::EventRefPosition;

        size_t n_details   = result_->totalNumDetails();
        size_t n_positions = single_pos ? n_details : result_->totalNumPositions();
        
        if (n_positions == 0)
            return {};

        std::vector<PosValue<T>> pos_values;
        pos_values.reserve(n_positions);

        if (detail_ranges)
            detail_ranges->reserve(n_details);

        auto func = [ & ] (const EvaluationDetail& detail, 
                           const EvaluationDetail* parent_detail,
                           int didx0, 
                           int didx1,
                           int evt_pos_idx, 
                           int evt_ref_pos_idx)
        {
            int num_pos = (int)detail.numPositions();
            traced_assert(num_pos >= 1);

            //get detail value from source
            auto value = source.valueFromDetail(detail);

            //value might not be set => skip
            if (!value.has_value())
                return;

            //collect positions + value
            if (detail_pos_mode == DetailValuePositionMode::EventPosition)
            {
                if (detail_ranges)
                    detail_ranges->emplace_back(pos_values.size(), 1);

                //!event position always needs to be available!
                traced_assert(evt_pos_idx <= num_pos);

                const auto& pos = evt_pos_idx >= 0 ? detail.position(evt_pos_idx) : detail.lastPos();
                
                pos_values.emplace_back(pos.latitude_, pos.longitude_, value.value());
            }
            else if (detail_pos_mode == DetailValuePositionMode::EventRefPosition)
            {
                if (detail_ranges)
                    detail_ranges->emplace_back(pos_values.size(), 1);

                //no ref pos available => skip
                if (evt_ref_pos_idx >= num_pos)
                    return;

                const auto& pos = evt_ref_pos_idx >= 0 ? detail.position(evt_ref_pos_idx) : detail.lastPos();
                
                pos_values.emplace_back(pos.latitude_, pos.longitude_, value.value());
            }
            else if (detail_pos_mode == DetailValuePositionMode::AllPositions)
            {
                if (detail_ranges)
                    detail_ranges->emplace_back(pos_values.size(), detail.numPositions());

                for (const auto& pos : detail.positions())
                    pos_values.emplace_back(pos.latitude_, pos.longitude_, value.value());
            }
        };

        result_->iterateDetails(func, {});

        pos_values.shrink_to_fit();

        if (detail_ranges)
            detail_ranges->shrink_to_fit();

        return pos_values;
    }

private:
    const Base* result_  = nullptr;
};

}
