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

#include "scatterseries.h"
#include "traced_assert.h"

#include "json.hpp"

/*******************************************************************************************
 * ScatterSeries
 *******************************************************************************************/

/**
*/
bool ScatterSeries::fromJSON(const nlohmann::json& data, bool binary)
{
    *this = {};

    if (binary)
    {
        if (!data.is_string())
            return false;

        //read binary
        std::string str = data;

        QByteArray ba_base64(str.data(), str.size());
        QByteArray ba = QByteArray::fromBase64(ba_base64);

        unsigned int* n        = (unsigned int*)(ba.data());
        const double* data     = (const double*)(ba.data() + sizeof(unsigned int));
        unsigned int  n_points = *n;

        Eigen::MatrixXd points_mat(n_points, 2);
        memcpy(points_mat.data(), data, points_mat.size() * sizeof(double));

        points.resize(n_points);

        for (unsigned int i = 0; i < n_points; ++i)
            points[ i ] = Eigen::Vector2d(points_mat(i, 0), points_mat(i, 1));
    }
    else
    {
        //read as json array
        if (!data.is_array())
            return false;

        points.reserve(data.size());

        for (size_t i = 0; i < data.size(); ++i)
        {
            const auto& jpoint = data[ i ];
            if (!jpoint.is_array() || jpoint.size() != 2)
                return false;

            double x = jpoint[ 0 ];
            double y = jpoint[ 1 ];

            points.emplace_back(x, y);
        }
    }

    return true;
}

/**
*/
nlohmann::json ScatterSeries::toJSON(bool binary) const
{
    nlohmann::json jpoints;

    if (binary)
    {
        //write binary
        unsigned int n = points.size();

        QByteArray ba;

        //add size
        ba.append((const char*)&n, sizeof(unsigned int));

        Eigen::MatrixXd points_mat(n, 2);
        for (unsigned int i = 0; i < n; ++i)
        {
            points_mat(i, 0) = points[ i ].x();
            points_mat(i, 1) = points[ i ].y();
        }
        
        //add points
        ba.append((const char*)points_mat.data(), points_mat.size() * sizeof(double));

        //code base 64
        QString byte_str(ba.toBase64());

        jpoints = byte_str.toStdString();
    }
    else
    {
        //write as json array
        jpoints = nlohmann::json::array();

        for (const auto& pos : points)
        {
            nlohmann::json jpoint = nlohmann::json::array();
            jpoint.push_back(pos.x());
            jpoint.push_back(pos.y());

            jpoints.push_back(jpoint);
        }
    }

    return jpoints;
}

/**
*/
boost::optional<QRectF> ScatterSeries::getDataBounds() const
{
    bool empty = true;

    double x_min = std::numeric_limits<double>::max();    // everything is <= this
    double x_max = std::numeric_limits<double>::lowest(); // everything is >= this
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();

    for (const auto& p : points)
    {
        x_min = std::min(x_min, p.x());
        x_max = std::max(x_max, p.x());
        y_min = std::min(y_min, p.y());
        y_max = std::max(y_max, p.y());

        empty = false;
    }

    if (empty)
        return {};
    else
        return QRectF(QPointF(x_min, y_min), QPointF(x_max, y_max));
}

/**
*/
size_t ScatterSeries::numPoints() const
{
    return points.size();
}

/*******************************************************************************************
 * ScatterSeriesCollection
 *******************************************************************************************/

const std::string ScatterSeriesCollection::TagDataSeries      = "dataseries";
const std::string ScatterSeriesCollection::TagConnectionLines = "connection_lines";
const std::string ScatterSeriesCollection::TagData            = "data";
const std::string ScatterSeriesCollection::TagDataRaw         = "data_raw";
const std::string ScatterSeriesCollection::TagDataTypeX       = "data_type_x";
const std::string ScatterSeriesCollection::TagDataTypeY       = "data_type_y";
const std::string ScatterSeriesCollection::TagName            = "name";
const std::string ScatterSeriesCollection::TagColor           = "color";
const std::string ScatterSeriesCollection::TagMarkerSize      = "marker_size";

/**
*/
ScatterSeriesCollection::ScatterSeriesCollection() = default;

/**
*/
ScatterSeriesCollection::~ScatterSeriesCollection() = default;

/**
*/
void ScatterSeriesCollection::clear()
{
    data_series_.clear();
}

/**
*/
void ScatterSeriesCollection::reset()
{
    clear();

    connection_lines_.reset();
}

/**
*/
// void ScatterSeriesCollection::addDataSeries(const DataSeries& series)
// {
//     data_series_.push_back(series);
// }

/**
*/
void ScatterSeriesCollection::addDataSeries(const ScatterSeries& scatter_series,
                                            const std::string& name,
                                            const QColor& color,
                                            double marker_size)
{
    DataSeries series;
    series.scatter_series = scatter_series;
    series.name           = name;
    series.color          = color;
    series.marker_size    = marker_size;

    traced_assert(!data_series_.count(name));
    data_series_[name] = series;
}

/**
*/
size_t ScatterSeriesCollection::numDataSeries() const
{
    return data_series_.size();
}

/**
*/
size_t ScatterSeriesCollection::numPoints(bool visible_series_only) const
{
    size_t count = 0;

    for (const auto& series_it : data_series_)
    {
        if (visible_series_only && !series_it.second.visible)
            continue;

        count += series_it.second.scatter_series.numPoints();
    }

    return count;
}

/**
*/
const std::map<std::string, ScatterSeriesCollection::DataSeries>& ScatterSeriesCollection::dataSeries() const
{
    return data_series_;
}

/**
*/
std::map<std::string, ScatterSeriesCollection::DataSeries>& ScatterSeriesCollection::dataSeries()
{
    return data_series_;
}

/**
*/
ScatterSeries::DataType ScatterSeriesCollection::commonDataTypeX() const
{
    if (data_series_.empty())
        return ScatterSeries::DataTypeFloatingPoint;

    ScatterSeries::DataType data_type = data_series_.begin()->second.scatter_series.data_type_x;

    // for (size_t i = 1; i < data_series_.size(); ++i)
    //     if (data_series_[ i ].scatter_series.data_type_x != data_type)
    //         return ScatterSeries::DataTypeFloatingPoint;

    for (auto& series_it : data_series_)
        if (series_it.second.scatter_series.data_type_x != data_type)
            return ScatterSeries::DataTypeFloatingPoint;

    return data_type;
}

/**
*/
ScatterSeries::DataType ScatterSeriesCollection::commonDataTypeY() const
{
    if (data_series_.empty())
        return ScatterSeries::DataTypeFloatingPoint;

    ScatterSeries::DataType data_type = data_series_.begin()->second.scatter_series.data_type_y;

    for (auto& series_it : data_series_)
        if (series_it.second.scatter_series.data_type_y != data_type)
            return ScatterSeries::DataTypeFloatingPoint;

    return data_type;
}

/**
*/
void ScatterSeriesCollection::setUseConnectionLines(bool ok)
{
    connection_lines_ = ok;
}

/**
*/
const boost::optional<bool>& ScatterSeriesCollection::useConnectionLines() const
{
    return connection_lines_;
}

/**
*/
bool ScatterSeriesCollection::fromJSON(const nlohmann::json& data)
{
    reset();

    if (!data.is_object() || !data.contains(TagDataSeries))
        return false;

    const auto& jseries = data[ TagDataSeries ];
    if (!jseries.is_array())
        return false;

    for (const auto& js : jseries)
    {
        if (!js.is_object() || (!js.contains(TagData) && !js.contains(TagDataRaw)))
            return false;

        DataSeries dseries;

        bool read_raw = js.contains(TagDataRaw);

        if (!dseries.scatter_series.fromJSON(js[ read_raw ? TagDataRaw : TagData ], read_raw))
            return false;

        if (js.contains(TagDataTypeX))
            dseries.scatter_series.data_type_x = js[ TagDataTypeX ];

        if (js.contains(TagDataTypeY))
            dseries.scatter_series.data_type_y = js[ TagDataTypeY ];

        if (js.contains(TagName))
            dseries.name = js[ TagName ];

        if (js.contains(TagColor))
        {
            std::string cname = js[ TagColor ];
            dseries.color = QColor(QString::fromStdString(cname));
        }

        if (js.contains(TagMarkerSize))
            dseries.marker_size = js[ TagMarkerSize ];

        traced_assert(!data_series_.count(dseries.name));
        data_series_[dseries.name] = dseries;
    }

    if (data.contains(TagConnectionLines))
    {
        bool cl = data[ TagConnectionLines ];
        connection_lines_ = cl;
    }

    return true;
}

/**
*/
nlohmann::json ScatterSeriesCollection::toJSON(bool binary) const
{
    nlohmann::json obj;
    nlohmann::json jseries = nlohmann::json::array();

    for (const auto& dseries : data_series_)
    {
        nlohmann::json js;

        js[ binary ? TagDataRaw : TagData ] = dseries.second.scatter_series.toJSON(binary);

        js[ TagDataTypeX  ] = dseries.second.scatter_series.data_type_x;
        js[ TagDataTypeY  ] = dseries.second.scatter_series.data_type_y;
        js[ TagName       ] = dseries.second.name;
        js[ TagColor      ] = dseries.second.color.name().toStdString();
        js[ TagMarkerSize ] = dseries.second.marker_size;

        jseries.push_back(js);
    }

    obj[ TagDataSeries ] = jseries;

    if (connection_lines_.has_value())
        obj[ TagConnectionLines ] = connection_lines_.value();

    return obj;
}

/**
*/
boost::optional<QRectF> ScatterSeriesCollection::getDataBounds(bool visible_series_only) const
{
    QRectF bounds;
    bool empty = true;

    for (const auto& dseries : data_series_)
    {
        if (visible_series_only && !dseries.second.visible)
            continue;

        auto b = dseries.second.scatter_series.getDataBounds();
        if (!b.has_value())
            continue;

        bounds = bounds.united(b.value());
        empty = false;
    }

    if (empty)
        return {};

    return bounds;
}
