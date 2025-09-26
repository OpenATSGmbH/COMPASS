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
#include "eval/results/base/result_t.h"

#include <string>

#include <QColor>

namespace EvaluationRequirementResult
{

/**
 * A data series that can be part of a feature. Reusable base.
 */
template <typename T>
struct FeatureDefinitionDataSeries
{
    virtual bool isValid() const 
    {
        return series_value_source.isValid();
    }

    /**
     * Get valid series values.
    */
    std::vector<T> getValues(const Base* base) const
    {
        traced_assert(isValid());
        return EvaluationResultTemplates(base).getValues<T>(series_value_source);
    }

    /**
     * Get optional series values for each detail.
    */
    std::vector<boost::optional<T>> getOptionalValues(const Base* base) const
    {
        traced_assert(isValid());
        return EvaluationResultTemplates(base).getOptionalValues<T>(series_value_source);
    }

    /**
     * Get series values + time.
    */
    std::vector<TimedValue<T>> getTimedValues(const Base* base) const
    {
        traced_assert(isValid());
        return EvaluationResultTemplates(base).getTimedValues<T>(series_value_source);
    }

    /**
     * Get series values + time in seconds.
    */
    std::vector<MSecTimedValue<T>> getMSecTimedValues(const Base* base) const
    {
        traced_assert(isValid());
        return EvaluationResultTemplates(base).getMSecTimedValues<T>(series_value_source);
    }

    /**
     * Get series values + positions.
    */
    std::vector<PosValue<T>> getValuesPlusPosition(const Base* base,
                                                   DetailValuePositionMode detail_pos_mode,
                                                   std::vector<std::pair<size_t,size_t>>* detail_ranges = nullptr) const
    {
        return EvaluationResultTemplates(base).getValuesPlusPosition<T>(series_value_source, 
                                                                        detail_pos_mode, 
                                                                        detail_ranges);
    }

    std::string    series_name;         // name of the series used in a plot legend
    ValueSource<T> series_value_source; // source of the series value in an eval detail
    QColor         series_color;        // color of the series (may be ignored by some features)
};

}
