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

#include "json_fwd.hpp"

#include <QSize>

#include <string>

namespace report_figure
{

/// An exported image has the layout of this size, with ExportScale * ExportScale pixels per layout pixel.
/// More pixels alone make the chart texts smaller, their font size is fixed.
extern const QSize ExportLayoutSize;
extern const double ExportScale;

}

/**
 * Options of the report figures as a task (evaluation, data source analysis) stores them in its configuration.
 * The producers of the figures write them into the figures: the log scale into each histogram of one series,
 * the export layout size into each figure view point.
 */
struct ReportFigureOptions
{
    bool  histogram_log_scale = true;
    QSize export_layout_size  = report_figure::ExportLayoutSize; // a smaller layout gives larger texts

    bool operator==(const ReportFigureOptions& other) const;
    bool operator!=(const ReportFigureOptions& other) const { return !(*this == other); }

    nlohmann::json toJSON() const;
    /// null is the same as an empty object, a missing field keeps its default. Returns false and keeps the values
    /// if obj is not readable or a size is outside [MinExportLayoutSize, MaxExportLayoutSize].
    bool fromJSON(const nlohmann::json& obj);

    /// value of the view point field of the export layout size
    nlohmann::json exportLayoutSizeJSON() const;
    /// the export layout size of a figure view point, ExportLayoutSize without a readable one
    static QSize exportLayoutSizeFromViewPoint(const nlohmann::json& view_point);

    /// Checks the value of the configuration parameter key in the parameters, if they hold it. Returns an error text
    /// if the value is not readable, else an empty string.
    static std::string checkParameter(const nlohmann::json& params, const std::string& key = ParameterName);

    static const int MinExportLayoutSize;
    static const int MaxExportLayoutSize;

    /// name of the configuration parameter of the tasks
    static const std::string ParameterName;

    static const std::string TagHistogramLogScale;
    static const std::string TagExportWidth;
    static const std::string TagExportHeight;

    /// view point field of the export layout size, an object with TagWidth and TagHeight
    static const std::string ViewPointFieldExportLayoutSize;
    static const std::string TagWidth;
    static const std::string TagHeight;
};
