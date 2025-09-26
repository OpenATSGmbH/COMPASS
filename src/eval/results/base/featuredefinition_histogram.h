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

#include "eval/results/base/featuredefinition.h"
#include "eval/results/base/featuredefinition_t.h"

#include "view/points/viewpointgenerator.h"
#include "view/histogramview/histogram.h"
#include "view/histogramview/histograminitializer.h"

#include "evaluationcalculator.h"

namespace EvaluationRequirementResult
{

/**
 * General histogram feature definition.
*/
template<typename T>
class FeatureDefinitionHistogram : public FeatureDefinition
{
public:
    typedef FeatureDefinitionDataSeries<T> HistogramDataSeries;

    FeatureDefinitionHistogram(const EvaluationCalculator& calculator,
                               const std::string& feature_description,
                               const std::string& x_axis_label,
                               const boost::optional<unsigned int>& num_bins = boost::optional<unsigned int>(),
                               const boost::optional<unsigned int>& num_distinct_values_min = boost::optional<unsigned int>()) 
    :   FeatureDefinition       (calculator, "histogram", feature_description, x_axis_label, "") 
    ,   num_bins_               (num_bins)
    ,   num_distinct_values_min_(num_distinct_values_min) {}

    virtual ~FeatureDefinitionHistogram() = default;

    /**
    */
    FeatureDefinitionHistogram& addDataSeries(const std::string& name, 
                                              const ValueSource<T>& value_source, 
                                              const QColor& color = DataSeriesColorDefault)
    {
        HistogramDataSeries ds;
        ds.series_name         = name;
        ds.series_color        = color;
        ds.series_value_source = value_source;

        data_series_.push_back(ds);
        
        return *this;
    }

    /**
    */
    bool isValid() const override final
    {
        //check if all data series are valid
        for (const auto& ds : data_series_)
            if (!ds.isValid())
                return false;

        return true;
    }

    /**
    */
    std::unique_ptr<ViewPointGenFeature> createFeature_impl(const Base* result) const override final
    {
        traced_assert(isValid());

        RawHistogramCollection hcollection;

        boost::optional<unsigned int> num_bins = num_bins_;
        if (!num_bins.has_value())
            num_bins = calculator().settings().histogram_num_bins;

        //handle data series
        for (const auto& ds : data_series_)
        {
            RawHistogram h;
            HistogramInitializerT<T> hinit;

            bool ok = hinit.createRAW(h, ds.getValues(result), false, num_bins, num_distinct_values_min_);
            if (!ok)
                continue;

            hcollection.addDataSeries(h, ds.series_name, ds.series_color);
        }

        auto feat = new ViewPointGenFeatureHistogram(hcollection);

        return std::unique_ptr<ViewPointGenFeature>(feat);
    }

private:
    boost::optional<unsigned int> num_bins_;
    boost::optional<unsigned int> num_distinct_values_min_;

    std::vector<HistogramDataSeries> data_series_;
};

/**
 * Category histogram feature definition.
*/
template<typename T>
class FeatureDefinitionCategoryHistogram : public FeatureDefinition
{
public:
    FeatureDefinitionCategoryHistogram(const EvaluationCalculator& calculator,
                                       const std::string& feature_description,
                                       const std::string& x_axis_label)
    :   FeatureDefinition(calculator, "histogram_category", feature_description, x_axis_label, "") 
    {
    }
    virtual ~FeatureDefinitionCategoryHistogram() = default;

    /**
     * Add static counts to histogram.
    */
    FeatureDefinitionCategoryHistogram& addDataSeries(const std::string& name, 
                                                      const std::vector<T>& category_values,
                                                      const std::vector<uint32_t> category_counts,
                                                      const QColor& color = Qt::blue)
    {
        HistogramDataSeries ds;
        ds.series_name     = name;
        ds.series_color    = color;
        ds.category_values = category_values;
        ds.category_counts = category_counts;

        data_series_.push_back(ds);
        
        return *this;
    }

    /**
    */
    bool isValid() const override final
    {
        //check if all data series are valid
        for (const auto& ds : data_series_)
            if (!ds.isValid())
                return false;

        return true;
    }

    /**
    */
    std::unique_ptr<ViewPointGenFeature> createFeature_impl(const Base* result) const override final
    {
        traced_assert(isValid());

        RawHistogramCollection hcollection;

        //handle data series
        for (const auto& ds : data_series_)
        {
            RawHistogram h;
            HistogramInitializerT<T> hinit;

            bool ok = hinit.createRAW(h, ds.category_values, ds.category_counts, false);
            if (!ok)
                continue;

            hcollection.addDataSeries(h, ds.series_name, ds.series_color);
        }

        auto feat = new ViewPointGenFeatureHistogram(hcollection);

        return std::unique_ptr<ViewPointGenFeature>(feat);
    }

private:
    /**
    */
    struct HistogramDataSeries : public FeatureDefinitionDataSeries<T>
    {
        bool isValid() const override
        {
            return (category_values.size() > 0 && category_values.size() == category_counts.size());
        }

        std::vector<T>        category_values;
        std::vector<uint32_t> category_counts;
    };

    std::vector<HistogramDataSeries> data_series_;
};

/**
 * String category histogram feature definition.
*/
class FeatureDefinitionStringCategoryHistogram : public FeatureDefinitionCategoryHistogram<std::string>
{
public:
    FeatureDefinitionStringCategoryHistogram(const EvaluationCalculator& calculator,
                                             const std::string& feature_description,
                                             const std::string& x_axis_label)
    :   FeatureDefinitionCategoryHistogram<std::string>(calculator, feature_description, x_axis_label) 
    {
    }
    virtual ~FeatureDefinitionStringCategoryHistogram() = default;
};

}
