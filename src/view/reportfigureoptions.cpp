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

#include "reportfigureoptions.h"
#include "logger.h"

#include "json.hpp"

namespace report_figure
{

const QSize  ExportLayoutSize = QSize(1200, 800);
const double ExportScale      = 2.0;

}

const int ReportFigureOptions::MinExportLayoutSize = 300;
const int ReportFigureOptions::MaxExportLayoutSize = 2400;

const std::string ReportFigureOptions::ParameterName = "report_figure_options";

const std::string ReportFigureOptions::TagHistogramLogScale = "histogram_log_scale";
const std::string ReportFigureOptions::TagExportWidth       = "export_width";
const std::string ReportFigureOptions::TagExportHeight      = "export_height";

const std::string ReportFigureOptions::ViewPointFieldExportLayoutSize = "export_layout_size";
const std::string ReportFigureOptions::TagWidth                       = "width";
const std::string ReportFigureOptions::TagHeight                      = "height";

namespace
{
    bool validLayoutLength(int length)
    {
        return length >= ReportFigureOptions::MinExportLayoutSize && length <= ReportFigureOptions::MaxExportLayoutSize;
    }
}

/**
 */
bool ReportFigureOptions::operator==(const ReportFigureOptions& other) const
{
    return histogram_log_scale == other.histogram_log_scale &&
           export_layout_size  == other.export_layout_size;
}

/**
 */
nlohmann::json ReportFigureOptions::toJSON() const
{
    nlohmann::json obj;

    obj[ TagHistogramLogScale ] = histogram_log_scale;
    obj[ TagExportWidth       ] = export_layout_size.width();
    obj[ TagExportHeight      ] = export_layout_size.height();

    return obj;
}

/**
 */
bool ReportFigureOptions::fromJSON(const nlohmann::json& obj)
{
    if (obj.is_null())
        return fromJSON(nlohmann::json::object());

    if (!obj.is_object())
    {
        logerr << "report figure options are no JSON object";
        return false;
    }

    ReportFigureOptions read;

    try
    {
        int width  = read.export_layout_size.width();
        int height = read.export_layout_size.height();

        auto readField = [ & ] (const std::string& tag, auto& value)
        {
            auto it = obj.find(tag);
            if (it != obj.end())
                value = it->template get<std::remove_reference_t<decltype(value)>>();
        };

        readField(TagHistogramLogScale, read.histogram_log_scale);
        readField(TagExportWidth      , width                   );
        readField(TagExportHeight     , height                  );

        if (!validLayoutLength(width) || !validLayoutLength(height))
        {
            logerr << "report figure export size " << width << " x " << height << " outside of ["
                   << MinExportLayoutSize << ", " << MaxExportLayoutSize << "]";
            return false;
        }

        read.export_layout_size = QSize(width, height);
    }
    catch (const nlohmann::json::exception& e)
    {
        logerr << "report figure options not readable: " << e.what();
        return false;
    }

    *this = read;

    return true;
}

/**
 */
nlohmann::json ReportFigureOptions::exportLayoutSizeJSON() const
{
    return { { TagWidth , export_layout_size.width()  },
             { TagHeight, export_layout_size.height() } };
}

/**
 */
QSize ReportFigureOptions::exportLayoutSizeFromViewPoint(const nlohmann::json& view_point)
{
    if (!view_point.is_object())
        return report_figure::ExportLayoutSize;

    auto it = view_point.find(ViewPointFieldExportLayoutSize);
    if (it == view_point.end())
        return report_figure::ExportLayoutSize;

    const nlohmann::json& size = *it;

    const bool readable = size.is_object()
                       && size.contains(TagWidth)  && size.at(TagWidth).is_number_integer()
                       && size.contains(TagHeight) && size.at(TagHeight).is_number_integer()
                       && validLayoutLength(size.at(TagWidth).get<int>())
                       && validLayoutLength(size.at(TagHeight).get<int>());
    if (!readable)
    {
        logwrn << "export layout size of the view point not readable, using the default";
        return report_figure::ExportLayoutSize;
    }

    return QSize(size.at(TagWidth).get<int>(), size.at(TagHeight).get<int>());
}

/**
 */
std::string ReportFigureOptions::checkParameter(const nlohmann::json& params, const std::string& key)
{
    if (!params.is_object() || !params.contains(key))
        return "";

    ReportFigureOptions options;
    if (!options.fromJSON(params.at(key)))
        return "'" + key + "' are no valid report figure options";

    return "";
}
