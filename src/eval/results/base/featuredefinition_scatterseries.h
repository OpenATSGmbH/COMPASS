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

#include "results/base/featuredefinition.h"
#include "results/base/featuredefinition_t.h"

#include "points/viewpointgenerator.h"
#include "scatterplotview/scatterseries.h"

#include "dbcontent/dbcontent.h"

#include "evaluationcalculator.h"

namespace EvaluationRequirementResult
{

/**
 * Scatter series feature definition.
*/
class FeatureDefinitionScatterSeries : public FeatureDefinition
{
public:
    typedef FeatureDefinitionDataSeries<double> ScatterDataSeries;

    FeatureDefinitionScatterSeries(const EvaluationCalculator& calculator,
                                   const std::string& feature_description,
                                   const std::string& x_axis_label,
                                   const std::string& y_axis_label);

    virtual ~FeatureDefinitionScatterSeries() = default;

    /**
    */
    FeatureDefinitionScatterSeries& addDataSeries(const std::string& name, 
                                                  const ValueSource<double>& value_source_x, 
                                                  const ValueSource<double>& value_source_y,
                                                  const QColor& color = DataSeriesColorDefault);

    /**
    */
    bool isValid() const override final;

    /**
    */
    std::unique_ptr<ViewPointGenFeature> createFeature_impl(const Base* result) const override final;

private:
    std::vector<ScatterDataSeries> data_series_x_;
    std::vector<ScatterDataSeries> data_series_y_;
};

/**
 * Custom scatter series feature definition.
*/
class FeatureDefinitionCustomScatterSeries : public FeatureDefinition
{
public:
    FeatureDefinitionCustomScatterSeries(const EvaluationCalculator& calculator,
                                         const std::string& feature_description,
                                         const std::string& x_axis_label,
                                       const std::string& y_axis_label);
    virtual ~FeatureDefinitionCustomScatterSeries() = default;

    /**
     * Add static counts to histogram.
    */
    FeatureDefinitionCustomScatterSeries& addDataSeries(const std::string& name, 
                                                        const std::vector<Eigen::Vector2d>& positions,
                                                        const QColor& color = Qt::blue);

    /**
    */
    bool isValid() const override final;

    /**
    */
    std::unique_ptr<ViewPointGenFeature> createFeature_impl(const Base* result) const override final;

private:
    /**
    */
    struct ScatterDataSeries
    {
        std::string                  name;
        QColor                       color;
        std::vector<Eigen::Vector2d> positions;
    };

    std::vector<ScatterDataSeries> data_series_;
};

/**
 * Scatter series feature definition.
*/
class FeatureDefinitionTimedScatterSeries : public FeatureDefinition
{
public:
    typedef FeatureDefinitionDataSeries<double> ScatterDataSeries;

    FeatureDefinitionTimedScatterSeries(const EvaluationCalculator& calculator,
                                        const std::string& feature_description,
                                        const std::string& y_axis_label) ;

    virtual ~FeatureDefinitionTimedScatterSeries() = default;

    /**
    */
    FeatureDefinitionTimedScatterSeries& addDataSeries(const std::string& name, 
                                                       const ValueSource<double>& value_source,
                                                       const QColor& color = DataSeriesColorDefault);

    /**
    */
    bool isValid() const override final;

    /**
    */
    std::unique_ptr<ViewPointGenFeature> createFeature_impl(const Base* result) const override final;

private:
    std::vector<ScatterDataSeries> data_series_;
};

}
