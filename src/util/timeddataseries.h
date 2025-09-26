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
#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "logger.h"
#include "stringconv.h"
#include "timeconv.h"

/**
 * @brief A time‐indexed series storing unsigned long values along with an integer confidence
 *        and the original insert timestamp.
 *
 * Template parameter T should be a callable type (e.g., a lambda or functor) that,
 * given an unsigned long, returns an int representing the confidence of that value.
 *
 * Internally, timestamps (boost::posix_time::ptime) are mapped into a zero‐based
 * std::vector index based on a minimum and maximum ptime, with 1‐second resolution.
 * Each element of the vector is a boost::optional<Entry>, where Entry holds:
 *   - the stored unsigned long value
 *   - the computed confidence (int) for that value
 *   - the original ptime at which this entry was inserted
 *
 * On insertion, you supply:
 *   1) a timestamp
 *   2) an unsigned long value
 *   3) a maxSeconds window
 *
 * The class computes:
 *   int conf = confidenceFunc(value);
 *
 * Then, for each second‐offset i in [ -maxSeconds, +maxSeconds ], it checks the slot at:
 *   index = convertTimestampToIndex(timestamp) + i
 * If index is in‐bounds, it will write { value, conf, timestamp } into data_[index]
 * under these rules:
 *   1. If the slot is empty → accept immediately.
 *   2. If existing.confidence < conf    → override.
 *   3. If existing.confidence == conf → compare distances:
 *         abs((existing.insert_time) – (slot_timestamp))
 *       vs
 *         abs((timestamp) – (slot_timestamp)).
 *       If the new timestamp is strictly closer, override.
 *
 */
template <typename T>
class TimedDataSeries
{
public:
    /// Internal storage type for each slot: (value + confidence + insert_time)
    struct Entry
    {
        unsigned long                value_index;
        int                          confidence;
        boost::posix_time::ptime     insert_time;
    };

    using OptionalEntry = boost::optional<Entry>;
    using ptime         = boost::posix_time::ptime;
    using time_duration = boost::posix_time::time_duration;

    /**
     * @brief Construct a TimedDataSeries with a given [min_ptime, max_ptime] range and a confidence functor.
     *
     * @param min_ptime     The earliest timestamp (inclusive) that the vector will cover.
     * @param max_ptime     The latest timestamp (inclusive) that the vector will cover.
     * @param confidence_func A callable (e.g., lambda) that takes an unsigned long and returns an int confidence.
     *
     * Initializes an internal std::vector of size ((max_ptime - min_ptime).total_seconds() + 1).
     * All slots start out empty (boost::none).  Timestamps outside this range will be ignored by insert().
     */
    TimedDataSeries(const ptime&                 min_ptime,
               const ptime&                      max_ptime,
               unsigned int                      max_seconds,
               const std::function<int(unsigned long)>& confidence_func,
               const std::function<boost::optional<T>(unsigned long)>& value_func)
        : min_ptime_(min_ptime)
        , max_ptime_(max_ptime)
        , max_seconds_(max_seconds)
        , confidence_func_(confidence_func)
        , value_func_(value_func)
    {
        traced_assert(!min_ptime_.is_not_a_date_time());
        traced_assert(!max_ptime_.is_not_a_date_time());
        traced_assert(min_ptime_ <= max_ptime_);

        initializeDataVector();
    }

    /**
     * @brief Reset the covered time range to a new [min_ptime, max_ptime], clearing all stored data.
     *
     * After calling resetRange(), any previously stored samples are discarded.  A new internal vector
     * is allocated with 1‐second resolution from min_ptime to max_ptime (inclusive).
     *
     * @param new_min_ptime   The new earliest timestamp (inclusive).
     * @param new_max_ptime   The new latest timestamp (inclusive).
     */
    void resetRange(const ptime& new_min_ptime,
                    const ptime& new_max_ptime)
    {
        traced_assert(!new_min_ptime.is_not_a_date_time());
        traced_assert(!new_max_ptime.is_not_a_date_time());
        traced_assert(new_min_ptime <= new_max_ptime);

        min_ptime_ = new_min_ptime;
        max_ptime_ = new_max_ptime;

        initializeDataVector();
    }

    /**
     * @brief Insert a (timestamp, value_index) pair, with propagation to ±max_seconds, using the stored confidence functor.
     *
     * This computes:
     *   int conf = confidence_func_(value_index);
     *
     * Then, for each second‐offset i in [ -max_seconds, +max_seconds ], it checks the slot at:
     *   index = convertTimestampToIndex(timestamp) + i
     * If index is in‐bounds, it will write { value_index, conf, timestamp } into data_[index]
     * under these rules:
     *   1) If the slot is empty → accept immediately.
     *   2) If existing.confidence < conf → override.
     *   3) If existing.confidence == conf → compare:
     *         distance(existing.insert_time, slot_ptime)
     *       vs
     *         distance(timestamp, slot_ptime).
     *       If new timestamp is strictly closer, override.
     *
     * Any timestamp outside [min_ptime_, max_ptime_] is ignored entirely.
     *
     * @param timestamp   The ptime at which the value_index was measured.
     * @param value_index       The unsigned long measurement.
     * @param max_seconds  How many seconds around `timestamp` to propagate this value_index.
     */
    void insert(const ptime& timestamp,
                unsigned long value_index, bool debug=false)
    {
        // Reject if outside the defined range
        traced_assert(timestamp >= min_ptime_ && timestamp <= max_ptime_);

        // Compute confidence for this value
        int conf = confidence_func_(value_index);

        // Base index for this exact timestamp
        std::size_t base_index = convertTimestampToIndex(timestamp);

        // The insertion timestamp (to store in each Entry)
        const ptime insertion_time = timestamp;

        // Propagate to all indices in [base_index - maxSeconds, base_index + maxSeconds]
        int window = static_cast<int>(max_seconds_);

        for (int delta = -window; delta <= window; ++delta)
        {
            int idx = static_cast<int>(base_index) + delta;
            if (idx < 0 || idx >= static_cast<int>(data_.size())) {
                continue;
            }
            std::size_t uidx = static_cast<std::size_t>(idx);

            // Compute the ptime corresponding to this slot index:
            //   slot_ptime = min_ptime_ + seconds(idx)
            ptime slot_ptime = min_ptime_ + boost::posix_time::seconds(uidx);

            // If slot is empty, accept immediately
            if (!data_[uidx].is_initialized()) {
                data_[uidx] = Entry{ value_index, conf, insertion_time };
                continue;
            }

            // Slot already has an entry: compare confidences
            Entry const& existing = *data_[uidx];

            if (existing.confidence < conf)
            {
                // New sample has strictly higher confidence → override
                data_[uidx] = Entry{ value_index, conf, insertion_time };

                if (debug)
                    loginf << "TDS: new conf " << conf
                           << " slot " << Utils::Time::toString(slot_ptime)
                           << " insert " << Utils::Time::toString(insertion_time);
            }
            else if (existing.confidence == conf)
            {
                // Same confidence tier → compare which insertion time is closer to slot_ptime
                time_duration existing_diff = existing.insert_time - slot_ptime;
                float existing_abs_secs = std::abs(existing_diff.total_milliseconds()) / 1000.0;

                time_duration new_diff = insertion_time - slot_ptime;
                float new_abs_secs = std::abs(new_diff.total_milliseconds()) / 1000.0;

                if (new_abs_secs < existing_abs_secs) {
                    // The new insertion timestamp is strictly closer → override
                    data_[uidx] = Entry{ value_index, conf, insertion_time };

                    if (debug)
                        loginf << "TDS: override conf " << conf
                               << " slot " << Utils::Time::toString(slot_ptime)
                               << " insert " << Utils::Time::toString(insertion_time)
                               << " existing_abs_secs " << Utils::String::doubleToStringPrecision(existing_abs_secs, 2)
                               << " new_abs_secs " << Utils::String::doubleToStringPrecision(new_abs_secs, 2);
                }
                // Otherwise, keep the existing entry
            }
            // else (existing.confidence > conf): do nothing
        }
    }

    /**
     * @brief Retrieve the stored Entry (value + confidence + insert_time) at a given timestamp, if any.
     *
     * @param timestamp   The ptime to look up.
     * @return boost::optional<Entry>  The stored entry, or boost::none if empty/out-of-range.
     */
    OptionalEntry getAt(const ptime& timestamp) const
    {
        if (timestamp < min_ptime_ || timestamp > max_ptime_)
            return boost::none;

        std::size_t idx = convertTimestampToIndex(timestamp);
        return data_[idx];
    }

    bool hasValueAt(const ptime& timestamp) const
    {
        OptionalEntry entry = getAt(timestamp); // boost::none if outside of time

        if (!entry || entry->confidence == -1)
            return false;

        return (value_func_(entry->value_index)).has_value();
    }

    T getValueAt(const ptime& timestamp) const
    {
        OptionalEntry entry = getAt(timestamp);

        traced_assert(entry && entry->confidence != -1);

        auto opt_val = value_func_(entry->value_index);
        traced_assert(opt_val.has_value());

        return *opt_val;
    }

    /**
     * @brief Return the minimum and maximum ptime currently covered.
     */
    ptime getMinTime() const { return min_ptime_; }
    ptime getMaxTime() const { return max_ptime_; }

    /**
     * @brief Return the total number of seconds (i.e., vector size) covered.
     */
    std::size_t getTotalSeconds() const { return data_.size(); }

private:

    ptime                         min_ptime_;
    ptime                         max_ptime_;
    unsigned int                  max_seconds_{0};

    std::vector<OptionalEntry>    data_;
    std::function<int(unsigned long)> confidence_func_;
    std::function<boost::optional<T>(unsigned long)> value_func_;

    /**
     * @brief (Re)initialize the internal data vector based on min_ptime_ and max_ptime_.
     *
     * Computes:
     *   std::size_t num_secs = (max_ptime_ - min_ptime_).total_seconds() + 1;
     * and resizes `data_` to that length, setting all elements to boost::none.
     */
    void initializeDataVector()
    {
        time_duration span = max_ptime_ - min_ptime_;
        traced_assert(!span.is_negative());

        long total_secs = span.total_seconds() + 1;
        data_.clear();
        data_.resize(static_cast<std::size_t>(total_secs), boost::none);
    }

    /**
     * @brief Convert a timestamp into a zero‐based index within [0, data_.size()-1].
     *
     * Assumes `timestamp` ∈ [min_ptime_, max_ptime_].  The calculation is:
     *   index = (timestamp - min_ptime_).total_seconds()
     *
     * @param timestamp   The ptime to convert.
     * @return std::size_t  The index into `data_`.
     */
    std::size_t convertTimestampToIndex(const ptime& timestamp) const
    {
        time_duration diff = timestamp - min_ptime_;

        long seconds       = diff.total_seconds();
        traced_assert(seconds >= 0);

        return static_cast<std::size_t>(seconds);
    }


};

