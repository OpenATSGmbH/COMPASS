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

#include "viewpointgenerator.h"
#include "viewpoint.h"

#include "geotiff.h"

#include "global.h"

#include "util/stringconv.h"

#include <QBuffer>
#include <QImageReader>

/********************************************************************************
 * ViewPointGenFilters
 ********************************************************************************/

/**
*/
void ViewPointGenFilters::addFilter(std::unique_ptr<ViewPointGenFilter>&& filter)
{
    filters_[filter->name()] = std::move(filter);
}

/**
*/
size_t ViewPointGenFilters::size() const
{
    return filters_.size();
}

/**
*/
void ViewPointGenFilters::toJSON(nlohmann::json& j) const
{
    for (const auto& f : filters_)
    {
        nlohmann::json j_filter;
        f.second->toJSON(j_filter);

        j[ f.first ] = j_filter;
    }
}

/********************************************************************************
 * ViewPointGenFilterUTN
 ********************************************************************************/

const std::string ViewPointGenFilterUTN::UTNFilterNodeName  = "UTNs";
const std::string ViewPointGenFilterUTN::UTNFilterFieldName = "utns";

/**
*/
ViewPointGenFilterUTN::ViewPointGenFilterUTN(const std::vector<uint32_t>& utns)
:   utns_(utns)
{
}

/**
*/
ViewPointGenFilterUTN::ViewPointGenFilterUTN(uint32_t utn)
{
    utns_.push_back(utn);
}

/**
*/
void ViewPointGenFilterUTN::toJSON(nlohmann::json& j) const
{
    if (utns_.empty())
        return;

    j[UTNFilterFieldName] = Utils::String::compress(utns_, ',');
}

/********************************************************************************
 * ViewPointGenFeature
 ********************************************************************************/

const std::string ViewPointGenFeature::FeatureTypeFieldName         = "name";
const std::string ViewPointGenFeature::FeatureTypeFieldType         = "type";
const std::string ViewPointGenFeature::FeatureFieldNameProps        = "properties";
const std::string ViewPointGenFeature::FeatureFieldNamePropColor    = "color";
const std::string ViewPointGenFeature::FeatureFieldNamePlotMetadata = "plot_metadata";

/**
*/
ViewPointGenFeature::ViewPointGenFeature(const std::string& type)
:   type_(type)
{
}

/**
*/
void ViewPointGenFeature::toJSON(nlohmann::json& j) const
{
    j[FeatureTypeFieldType   ] = type_;
    j[FeatureTypeFieldName   ] = name_;

    toJSON_impl(j, write_binary_if_possible_);

    if (plot_metadata_.has_value())
        j[ FeatureFieldNamePlotMetadata ] = plot_metadata_.value().toJSON();
}

/**
*/
void ViewPointGenFeature::print(std::ostream& strm, const std::string& prefix) const
{
    strm << prefix << "[Feature]" << std::endl;
    strm << prefix << "name: " << name() << std::endl;
    strm << prefix << "type: " << type_ << std::endl;
    strm << prefix << "size: " << size() << std::endl;
}

/********************************************************************************
 * ViewPointGenFeaturePointGeometry
 ********************************************************************************/

const std::string ViewPointGenFeaturePointGeometry::FeatureFieldNameGeom      = "geometry";
const std::string ViewPointGenFeaturePointGeometry::FeatureFieldNameCoords    = "coordinates";
const std::string ViewPointGenFeaturePointGeometry::FeatureFieldNameColors    = "colors";


/**
*/
ViewPointGenFeaturePointGeometry::ViewPointGenFeaturePointGeometry(const std::string& type,
                                                                   const std::vector<Eigen::Vector2d>& positions,
                                                                   const std::vector<QColor>& colors,
                                                                   bool enable_color_vector)
:   ViewPointGenFeature (type               )
,   positions_          (positions          )
,   colors_             (colors             )
,   enable_color_vector_(enable_color_vector)
{
}

/**
*/
void ViewPointGenFeaturePointGeometry::reserve(size_t n, bool reserve_cols)
{
    positions_.reserve(n);
    if (enable_color_vector_ && reserve_cols)
        colors_.reserve(n);
}

/**
*/
void ViewPointGenFeaturePointGeometry::addPoint(const Eigen::Vector2d& pos, 
                                                const boost::optional<QColor>& color)
{
    positions_.push_back(pos);

    if (enable_color_vector_ && color.has_value())
        colors_.push_back(color.value());
}

/**
*/
void ViewPointGenFeaturePointGeometry::addPoints(const std::vector<Eigen::Vector2d>& positions,
                                                 const boost::optional<std::vector<QColor>>& colors)
{
    positions_.insert(positions_.end(), positions.begin(), positions.end());

    if (enable_color_vector_ && colors.has_value())
        colors_.insert(colors_.end(), colors->begin(), colors->end());
}

/**
*/
void ViewPointGenFeaturePointGeometry::toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const
{
    nlohmann::json geom;
    nlohmann::json props;

    auto coords = nlohmann::json::array();

    for (const auto& pos : positions_)
    {
        auto coord = nlohmann::json::array();
        coord.push_back(pos.x());
        coord.push_back(pos.y());

        coords.push_back(coord);
    }

    geom[FeatureFieldNameCoords] = coords; 
    
    if (enable_color_vector_ && positions_.size() == colors_.size())
    {
        auto colors = nlohmann::json::array();

        for (const auto& c : colors_)
        {
            nlohmann::json color = c.name().toStdString();
            colors.push_back(color);
        }

        geom[FeatureFieldNameColors] = colors;
    }
    else
    {
        props[FeatureFieldNamePropColor] = color_.name().toStdString();
    }

    writeGeometry(geom);
    writeProperties(props);

    j[FeatureFieldNameGeom ] = geom;
    j[FeatureFieldNameProps] = props; 
}

/**
*/
nlohmann::json& ViewPointGenFeaturePointGeometry::getCoordinatesJSON(nlohmann::json& feature_json)
{
    //check validity first
    traced_assert(feature_json.count(FeatureFieldNameGeom));
    traced_assert(feature_json[ FeatureFieldNameGeom ].count(FeatureFieldNameCoords));

    return feature_json.at(FeatureFieldNameGeom).at(FeatureFieldNameCoords);
}

/**
*/
QRectF ViewPointGenFeaturePointGeometry::roi() const
{
    if (positions_.empty())
        return QRectF();

    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::lowest();
    double ymax = std::numeric_limits<double>::lowest();

    for (const auto& pos : positions_)
    {
        if (pos.x() < xmin) xmin = pos.x();
        if (pos.y() < ymin) ymin = pos.y();
        if (pos.x() > xmax) xmax = pos.x();
        if (pos.y() > ymax) ymax = pos.y();
    }

    return QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
}

/********************************************************************************
 * ViewPointGenFeaturePoints
 ********************************************************************************/

const std::string ViewPointGenFeaturePoints::FeatureName                      = "points";
const std::string ViewPointGenFeaturePoints::FeaturePointsFieldNameSymbol     = "symbol";
const std::string ViewPointGenFeaturePoints::FeaturePointsFieldNameSymbolSize = "symbol_size";

const std::string ViewPointGenFeaturePoints::SymbolNameCircle      = "circle";
const std::string ViewPointGenFeaturePoints::SymbolNameTriangle    = "triangle";
const std::string ViewPointGenFeaturePoints::SymbolNameSquare      = "square";
const std::string ViewPointGenFeaturePoints::SymbolNameCross       = "cross";
const std::string ViewPointGenFeaturePoints::SymbolNameBorder      = "border";
const std::string ViewPointGenFeaturePoints::SymbolNameBorderThick = "border_thick";

/**
*/
ViewPointGenFeaturePoints::ViewPointGenFeaturePoints(Symbol symbol,
                                                     float symbol_size,
                                                     const std::vector<Eigen::Vector2d>& positions,
                                                     const std::vector<QColor>& colors,
                                                     bool enable_color_vector)
:   ViewPointGenFeaturePointGeometry(FeatureName, positions, colors, enable_color_vector)
,   symbol_     (symbol     )
,   symbol_size_(symbol_size)
{
}

/**
*/
std::string ViewPointGenFeaturePoints::symbolString() const
{
    if (symbol_ == Symbol::Circle)
        return SymbolNameCircle;
    if (symbol_ == Symbol::Triangle)
        return SymbolNameTriangle;
    if (symbol_ == Symbol::Square)
        return SymbolNameSquare;
    if (symbol_ == Symbol::Cross)
        return SymbolNameCross;
    if (symbol_ == Symbol::Border)
        return SymbolNameBorder;
    if (symbol_ == Symbol::BorderThick)
        return SymbolNameBorderThick;
    return "";
}

/**
*/
void ViewPointGenFeaturePoints::writeProperties(nlohmann::json& j) const
{
    j[FeaturePointsFieldNameSymbol    ] = symbolString();
    j[FeaturePointsFieldNameSymbolSize] = symbol_size_;
}

/********************************************************************************
 * ViewPointGenFeatureStyledLine
 ********************************************************************************/

const std::string ViewPointGenFeatureStyledLine::FeatureLineStringFieldNameLineWidth = "line_width";
const std::string ViewPointGenFeatureStyledLine::FeatureLineStringFieldNameLineStyle = "line_style";

/**
*/
ViewPointGenFeatureStyledLine::ViewPointGenFeatureStyledLine(const std::string& type,
                                  float line_width,
                                  LineStyle line_style,
                                  const std::vector<Eigen::Vector2d>& positions,
                                  const std::vector<QColor>& colors,
                                  bool enable_color_vector)
:   ViewPointGenFeaturePointGeometry(type, positions, colors, enable_color_vector)
,   line_width_(line_width)
,   line_style_(line_style)
{
}

/**
*/
std::string ViewPointGenFeatureStyledLine::styleString() const
{
    if (line_style_ == LineStyle::Dotted)
        return "dotted";

    return "solid";
}

/**
*/
void ViewPointGenFeatureStyledLine::writeProperties(nlohmann::json& j) const
{
    j[FeatureLineStringFieldNameLineWidth] = line_width_;
    j[FeatureLineStringFieldNameLineStyle] = styleString();
}

/********************************************************************************
 * ViewPointGenFeatureLineString
 ********************************************************************************/

const std::string ViewPointGenFeatureLineString::FeatureName       = "line_string";
const std::string ViewPointGenFeatureLineString::FeatureNameInterp = "line_string_interpolated";

/**
*/
ViewPointGenFeatureLineString::ViewPointGenFeatureLineString(bool interpolated,
                                                             float line_width,
                                                             LineStyle line_style,
                                                             const std::vector<Eigen::Vector2d>& positions,
                                                             const std::vector<QColor>& colors,
                                                             bool enable_color_vector)
:   ViewPointGenFeatureStyledLine(interpolated ? FeatureNameInterp : FeatureName, line_width, line_style, positions, colors, enable_color_vector)
{
}

/********************************************************************************
 * ViewPointGenFeatureLines
 ********************************************************************************/

const std::string ViewPointGenFeatureLines::FeatureName = "lines";

/**
*/
ViewPointGenFeatureLines::ViewPointGenFeatureLines(float line_width,
                                                   LineStyle line_style,
                                                   const std::vector<Eigen::Vector2d>& positions,
                                                   const std::vector<QColor>& colors,
                                                   bool enable_color_vector)
:   ViewPointGenFeatureStyledLine(FeatureName, line_width, line_style, positions, colors, enable_color_vector)
{
}

/********************************************************************************
 * ViewPointGenFeatureErrorEllipses
 ********************************************************************************/

const std::string ViewPointGenFeatureErrEllipses::FeatureName                          = "ellipses";
const std::string ViewPointGenFeatureErrEllipses::FeatureErrEllipsesFieldNameLineWidth = "line_width";
const std::string ViewPointGenFeatureErrEllipses::FeatureErrEllipsesFieldNameNumPoints = "num_points";
const std::string ViewPointGenFeatureErrEllipses::FeatureErrEllipsesFieldNameSizes     = "sizes";

/**
*/
ViewPointGenFeatureErrEllipses::ViewPointGenFeatureErrEllipses(float line_width,
                                                               size_t num_points,
                                                               const std::vector<Eigen::Vector2d>& positions,
                                                               const std::vector<QColor>& colors,
                                                               const std::vector<Eigen::Vector3d>& sizes,
                                                               bool enable_color_vector)
:   ViewPointGenFeaturePointGeometry(FeatureName, positions, colors, enable_color_vector)
,   line_width_(line_width)
,   num_points_(num_points)
,   sizes_     (sizes     )
{
}

/**
*/
void ViewPointGenFeatureErrEllipses::reserve(size_t n, bool reserve_cols)
{
    ViewPointGenFeaturePointGeometry::reserve(n, reserve_cols);

    sizes_.reserve(n);
}

/**
*/
void ViewPointGenFeatureErrEllipses::addSize(const Eigen::Vector3d& size)
{
    sizes_.push_back(size);
}

/**
*/
void ViewPointGenFeatureErrEllipses::addSizes(const std::vector<Eigen::Vector3d>& sizes)
{
    sizes_.insert(sizes_.end(), sizes.begin(), sizes.end());
}

/**
*/
void ViewPointGenFeatureErrEllipses::writeGeometry(nlohmann::json& j) const
{
    nlohmann::json sizes = nlohmann::json::array();
    for (const auto& s : sizes_)
    {
        nlohmann::json size = nlohmann::json::array();
        size.push_back(s.x());
        size.push_back(s.y());
        size.push_back(s.z());

        sizes.push_back(size);
    }

    j[FeatureErrEllipsesFieldNameSizes] = sizes;
}

/**
*/
void ViewPointGenFeatureErrEllipses::writeProperties(nlohmann::json& j) const
{
    j[FeatureErrEllipsesFieldNameLineWidth] = line_width_;
    j[FeatureErrEllipsesFieldNameNumPoints] = num_points_;
}

/********************************************************************************
 * ViewPointGenFeatureText
 ********************************************************************************/

const std::string ViewPointGenFeatureText::FeatureName                  = "text";
const std::string ViewPointGenFeatureText::FeatureTextFieldNameText     = "text";
const std::string ViewPointGenFeatureText::FeatureTextFieldNamePos      = "position";
const std::string ViewPointGenFeatureText::FeatureTextFieldNameDir      = "direction";
const std::string ViewPointGenFeatureText::FeatureTextFieldNameFontSize = "font_size";

const std::string ViewPointGenFeatureText::TextDirNameRightUp   = "right-up";
const std::string ViewPointGenFeatureText::TextDirNameRightDown = "right-down";
const std::string ViewPointGenFeatureText::TextDirNameLeftUp    = "left-up";
const std::string ViewPointGenFeatureText::TextDirNameLeftDown  = "left-down";

/**
*/
ViewPointGenFeatureText::ViewPointGenFeatureText(const std::string& text,
                                                 double x,
                                                 double y,
                                                 float font_size,
                                                 TextDirection text_dir)
:   ViewPointGenFeature(FeatureName)
,   text_           (text     )
,   x_              (x        )
,   y_              (y        )
,   font_size_      (font_size)
,   text_dir_       (text_dir )
{
}

/**
*/
std::string ViewPointGenFeatureText::textDirString() const
{
    if (text_dir_ == TextDirection::RightUp)
        return TextDirNameRightUp;
    if (text_dir_ == TextDirection::RightDown)
        return TextDirNameRightDown;
    if (text_dir_ == TextDirection::LeftUp)
        return TextDirNameLeftUp;
    if (text_dir_ == TextDirection::LeftDown)
        return TextDirNameLeftDown;
    return "";
}

/**
*/
void ViewPointGenFeatureText::toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const
{
    j[FeatureTextFieldNameText] = text_;

    nlohmann::json pos = nlohmann::json::array();
    pos.push_back(x_);
    pos.push_back(y_);
    j[FeatureTextFieldNamePos] = pos;

    nlohmann::json props;
    props[FeatureTextFieldNameFontSize] = font_size_;
    props[FeatureFieldNamePropColor   ] = color_.name().toStdString();
    props[FeatureTextFieldNameDir     ] = textDirString();

    j[FeatureFieldNameProps] = props;
}

/**
*/
QRectF ViewPointGenFeatureText::roi() const
{
    return QRectF(x_ - 1e-06, y_ - 1e-06, 2e-06, 2e-06);
}

/********************************************************************************
 * ViewPointGenFeatureGeoImage
 ********************************************************************************/

const std::string ViewPointGenFeatureGeoImage::FeatureName                         = "geoimage";
const std::string ViewPointGenFeatureGeoImage::FeatureGeoImageFieldNameSource      = "source";
const std::string ViewPointGenFeatureGeoImage::FeatureGeoImageFieldNameFn          = "fn";
const std::string ViewPointGenFeatureGeoImage::FeatureGeoImageFieldNameData        = "data";
const std::string ViewPointGenFeatureGeoImage::FeatureGeoImageFieldNameReference   = "reference";
const std::string ViewPointGenFeatureGeoImage::FeatureGeoImageFieldNameLegend      = "legend";
const std::string ViewPointGenFeatureGeoImage::FeatureGeoImageFieldNameSubsample   = "subsample";
const std::string ViewPointGenFeatureGeoImage::FeatureGeoImageFieldNameSubsampling = "subsampling";

/**
*/
ViewPointGenFeatureGeoImage::ViewPointGenFeatureGeoImage(const std::string& fn,
                                                         const RasterReference& ref,
                                                         const ColorLegend& legend,
                                                         bool subsample,
                                                         int subsampling)
:   ViewPointGenFeature(FeatureName)
,   fn_         (fn         )
,   ref_        (ref        )
,   legend_     (legend     )
,   subsample_  (subsample  )
,   subsampling_(subsampling)
{
}

/**
*/
ViewPointGenFeatureGeoImage::ViewPointGenFeatureGeoImage(const QImage& data,
                                                         const RasterReference& ref,
                                                         const ColorLegend& legend,
                                                         bool subsample,
                                                         int subsampling)
:   ViewPointGenFeature(FeatureName)
,   data_       (data       )
,   ref_        (ref        )
,   legend_     (legend     )
,   subsample_  (subsample  )
,   subsampling_(subsampling)
{
}

/**
*/
void ViewPointGenFeatureGeoImage::toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const
{
    //source
    j[FeatureGeoImageFieldNameSource] = nlohmann::json::object();
    auto& source = j[FeatureGeoImageFieldNameSource];

    if (!data_.isNull())
    {
        source[ FeatureGeoImageFieldNameData ] = ViewPointGenFeatureGeoImage::imageToByteStringWithMetadata(data_);
    }
    else
    {
        source[ FeatureGeoImageFieldNameFn ] = fn_;
    }

    //reference
    j[FeatureGeoImageFieldNameReference] = ref_.toJSON();

    //legend
    if (!legend_.empty())
        j[FeatureGeoImageFieldNameLegend] = legend_.toJSON();

    //subsampling
    j[FeatureGeoImageFieldNameSubsample  ] = subsample_;
    j[FeatureGeoImageFieldNameSubsampling] = subsampling_;
}

/**
 * Encodes an image to a base64 string representation of a certain image format (e.g. 'PNG').
 * This method can be used for embedding an image for external use.
*/
std::string ViewPointGenFeatureGeoImage::imageToByteString(const QImage& img, const std::string& format)
{
    //prepare buffer
    QByteArray byteArray;
    QBuffer buffer(&byteArray);
    buffer.open(QIODevice::WriteOnly);

    //encode image to given format
    img.save(&buffer, format.c_str(), 0);

    QByteArray base64Data = byteArray.toBase64();
    return QString::fromLatin1(base64Data).toStdString();
}

/**
 * Encodes an image to a base64 string and adds some metadata to it for decoding.
 * This method is meant for internal use, as no standard format is used for encoding/decoding.
*/
std::string ViewPointGenFeatureGeoImage::imageToByteStringWithMetadata(const QImage& img)
{
    int w      = img.width();
    int h      = img.height();
    int stride = img.bytesPerLine();
    int format = (int)img.format();

    QByteArray ba;

    //add meta-data
    ba.append((const char*)&w     , sizeof(int));
    ba.append((const char*)&h     , sizeof(int));
    ba.append((const char*)&stride, sizeof(int));
    ba.append((const char*)&format, sizeof(int));
    
    //add image data
    ba.append((const char*)img.bits(), img.sizeInBytes());

    //code base 64
    QString byte_str(ba.toBase64());

    return byte_str.toStdString();
}

/**
 * Decodes a base64 string to an image, using its metadata (see imageToByteStringWithMetadata()).
 * This method is meant for internal use, as no standard format is used for encoding/decoding.
*/
QImage ViewPointGenFeatureGeoImage::byteStringWithMetadataToImage(const std::string& str)
{
    QByteArray ba_base64(str.data(), str.size());
    QByteArray ba = QByteArray::fromBase64(ba_base64);

    //get meta-data
    int* w      = (int*)(ba.data() + 0 * sizeof(int));
    int* h      = (int*)(ba.data() + 1 * sizeof(int));
    int* stride = (int*)(ba.data() + 2 * sizeof(int));
    int* format = (int*)(ba.data() + 3 * sizeof(int));

    //std::cout << "byte image - w: " << *w << ", h: " << *h << ", stride: " << *stride << ", format: " << *format << std::endl;

    const char* data = ba.data() + 4 * sizeof(int);

    //data pointer has to live over the whole QImage's lifetime, so we copy the
    //image, which should do a deep copy of the data
    return QImage((const uchar*)data, *w, *h, *stride, (QImage::Format)(*format)).copy();
}

/**
*/
QRectF ViewPointGenFeatureGeoImage::roi() const
{
    QSize size = data_.isNull() ? (!fn_.empty() ? QImageReader(QString::fromStdString(fn_)).size() : QSize()) : data_.size();
    if (size.isNull())
        return QRectF();

    if (!ref_.srs.empty())
    {
        //get extents from reference + image size
        return ref_.getROI(size.width(), size.height());
    }
    else if (!fn_.empty())
    {
        //get extents from image file (if possible)
        auto info = GeoTIFF::getInfo(fn_);
        if (info.isValid())
            return info.roiAsWGS84();
    }

    return QRectF();
}

/********************************************************************************
 * ViewPointGenFeatureGrid
 ********************************************************************************/

const std::string ViewPointGenFeatureGrid::FeatureName                        = "grid";
const std::string ViewPointGenFeatureGrid::FeatureGridFieldNameGrid           = "grid";
const std::string ViewPointGenFeatureGrid::FeatureGridFieldNameRenderSettings = "render_settings";

/**
*/
ViewPointGenFeatureGrid::ViewPointGenFeatureGrid(const Grid2DLayer& grid,
                                                 const boost::optional<Grid2DRenderSettings>& render_settings,
                                                 const boost::optional<PlotMetadata>& metadata)
:   ViewPointGenFeature(FeatureName)
,   grid_           (grid           )
,   render_settings_(render_settings)
{
    plot_metadata_ = metadata;
}

/**
*/
void ViewPointGenFeatureGrid::toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const
{
    j[FeatureGridFieldNameGrid] = grid_.toJSON(write_binary_if_possible);

    if (render_settings_.has_value())
        j[FeatureGridFieldNameRenderSettings] = render_settings_.value().toJSON();
}

/**
*/
QRectF ViewPointGenFeatureGrid::roi() const
{
    return grid_.ref.getROI(grid_.data.cols(), grid_.data.rows());
}

/********************************************************************************
 * ViewPointGenFeatureHistogram
 ********************************************************************************/

const std::string ViewPointGenFeatureHistogram::FeatureName                        = "histogram";
const std::string ViewPointGenFeatureHistogram::FeatureHistogramFieldNameHistogram = "histogram";

/**
*/
ViewPointGenFeatureHistogram::ViewPointGenFeatureHistogram(const RawHistogram& histogram,
                                                           const std::string& series_name,
                                                           const QColor& series_color,
                                                           const boost::optional<bool>& use_log_scale,
                                                           const PlotMetadata& metadata)
:   ViewPointGenFeature(FeatureName)
{
    histogram_.addDataSeries(histogram, series_name, series_color);

    if (use_log_scale.has_value())
        histogram_.setUseLogScale(use_log_scale.value());

    plot_metadata_ = metadata;
}

/**
*/
ViewPointGenFeatureHistogram::ViewPointGenFeatureHistogram(const RawHistogramCollection& histogram_collection,
                                                           const PlotMetadata& metadata)
:   ViewPointGenFeature(FeatureName)
,   histogram_(histogram_collection)
{
    plot_metadata_ = metadata;
}

/**
*/
size_t ViewPointGenFeatureHistogram::size() const 
{ 
    return histogram_.numDataSeries();
}

/**
*/
void ViewPointGenFeatureHistogram::toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const
{
    j[ FeatureHistogramFieldNameHistogram ] = histogram_.toJSON();
}

/********************************************************************************
 * ViewPointGenFeatureScatterSeries
 ********************************************************************************/

const std::string ViewPointGenFeatureScatterSeries::FeatureName                            = "scatterseries";
const std::string ViewPointGenFeatureScatterSeries::FeatureHistogramFieldNameScatterSeries = "scatterseries";

/**
*/
ViewPointGenFeatureScatterSeries::ViewPointGenFeatureScatterSeries(const ScatterSeries& scatter_series,
                                                                   const std::string& series_name,
                                                                   const QColor& series_color,
                                                                   double marker_size,
                                                                   const boost::optional<bool>& use_connection_lines,
                                                                   const PlotMetadata& metadata)
:   ViewPointGenFeature(FeatureName)
{
    scatter_series_.addDataSeries(scatter_series, series_name, series_color, marker_size);

    if (use_connection_lines.has_value())
        scatter_series_.setUseConnectionLines(use_connection_lines.value());

    plot_metadata_ = metadata;
}

/**
*/
ViewPointGenFeatureScatterSeries::ViewPointGenFeatureScatterSeries(const ScatterSeriesCollection& scatter_series_collection,
                                                                   const PlotMetadata& metadata)
:   ViewPointGenFeature(FeatureName)
,   scatter_series_(scatter_series_collection)
{
    plot_metadata_ = metadata;
}

/**
*/
size_t ViewPointGenFeatureScatterSeries::size() const 
{ 
    return scatter_series_.numDataSeries();
}

/**
*/
void ViewPointGenFeatureScatterSeries::toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const
{
    j[ FeatureHistogramFieldNameScatterSeries ] = scatter_series_.toJSON(write_binary_if_possible);
}

/********************************************************************************
 * ViewPointGenAnnotation
 ********************************************************************************/

const std::string ViewPointGenAnnotation::AnnotationFieldName        = "name";
const std::string ViewPointGenAnnotation::AnnotationFieldHidden      = "hidden";
const std::string ViewPointGenAnnotation::AnnotationFieldSymbolColor = "symbol_color";
const std::string ViewPointGenAnnotation::AnnotationFieldFeatures    = "features";
const std::string ViewPointGenAnnotation::AnnotationFieldAnnotations = "annotations";

/**
*/
ViewPointGenAnnotation::ViewPointGenAnnotation(const std::string& name,
                                               bool hidden)
:   name_  (name  )
,   hidden_(hidden)
{
}

/**
*/
void ViewPointGenAnnotation::setSymbolColor(const QColor& color)
{
    symbol_color_ = color;
}

/**
*/
void ViewPointGenAnnotation::addFeature(std::unique_ptr<ViewPointGenFeature>&& feat)
{
    traced_assert(feat);

    feat_map_[ feat->name() ] = features_.size();
    features_.push_back(std::move(feat));
}

/**
*/
void ViewPointGenAnnotation::addFeature(ViewPointGenFeature* feat)
{
    traced_assert(feat);

    feat_map_[ feat->name() ] = features_.size();
    features_.emplace_back(feat);
}

/**
*/
bool ViewPointGenAnnotation::hasFeature(const std::string& name) const
{
    return feat_map_.count(name) > 0;
}

/**
*/
ViewPointGenFeature* ViewPointGenAnnotation::getFeature(const std::string& name)
{
    auto it = feat_map_.find(name);

    if (it == feat_map_.end())
        return nullptr;

    return features_.at(it->second).get();
}

/**
*/
const ViewPointGenFeature* ViewPointGenAnnotation::getFeature(const std::string& name) const
{
    auto it = feat_map_.find(name);

    if (it == feat_map_.end())
        return nullptr;

    return features_.at(it->second).get();
}

/**
*/
ViewPointGenAnnotation* ViewPointGenAnnotation::addAnnotation(const std::string& name, bool hidden)
{
    anno_map_[ name ] = annotations_.size();
    annotations_.emplace_back(new ViewPointGenAnnotation(name, hidden));
    return annotations_.back().get();
}

/**
*/
void ViewPointGenAnnotation::addAnnotation(std::unique_ptr<ViewPointGenAnnotation>&& a)
{
    traced_assert(a);

    anno_map_[ a->name() ] = annotations_.size();
    annotations_.push_back(std::move(a));
}

/**
*/
ViewPointGenAnnotation* ViewPointGenAnnotation::getOrCreateAnnotation(const std::string& name, bool hidden)
{
    if (hasAnnotation(name))
        return annotation(name);

    return addAnnotation(name, hidden);
}

/**
*/
bool ViewPointGenAnnotation::hasAnnotation(const std::string& name) const
{
    return anno_map_.count(name) > 0;
}

/**
*/
ViewPointGenAnnotation* ViewPointGenAnnotation::annotation(const std::string& name) const
{
    auto it = anno_map_.find(name);
    traced_assert(it != anno_map_.end());

    return annotations_.at(it->second).get();
}

/**
*/
void ViewPointGenAnnotation::toJSON(nlohmann::json& j) const
{
    j[AnnotationFieldName  ] = name_;
    j[AnnotationFieldHidden] = hidden_;

    if (symbol_color_.has_value())
        j[AnnotationFieldSymbolColor] = symbol_color_.value().name().toStdString();

    nlohmann::json feats = nlohmann::json::array();

    for (const auto& f : features_)
    {
        nlohmann::json feat;
        f->toJSON(feat);

        feats.push_back(feat);
    }

    j[AnnotationFieldFeatures] = feats;

    nlohmann::json annos = nlohmann::json::array();

    for (const auto& a : annotations_)
    {
        nlohmann::json anno;
        a->toJSON(anno);

        annos.push_back(anno);
    }

    j[AnnotationFieldAnnotations] = annos;
}

/**
*/
void ViewPointGenAnnotation::print(std::ostream& strm, const std::string& prefix) const
{
    strm << prefix << "[Annotation]" << std::endl;
    strm << prefix << "name:   " << name_ << std::endl;
    strm << prefix << "hidden: " << hidden_ << std::endl;

    std::string p2 = prefix + "   ";

    for (const auto& f : features_)
        f->print(strm, p2);

    for (const auto& a : annotations_)
        a->print(strm, p2);
}

/**
*/
nlohmann::json& ViewPointGenAnnotation::getFeaturesJSON(nlohmann::json& annotation_json)
{
    traced_assert(annotation_json.contains(AnnotationFieldFeatures));
    traced_assert(annotation_json.at(AnnotationFieldFeatures).is_array());

    return annotation_json.at(AnnotationFieldFeatures);
}

/**
*/
nlohmann::json& ViewPointGenAnnotation::getFeatureJSON(nlohmann::json& annotation_json, size_t idx)
{
    auto& feat_arr = ViewPointGenAnnotation::getFeaturesJSON(annotation_json);

    traced_assert(feat_arr.is_array());
    traced_assert(idx < feat_arr.size());
    traced_assert(feat_arr.at(idx).is_object());

    return feat_arr.at(idx);
}

/**
*/
nlohmann::json& ViewPointGenAnnotation::getChildrenJSON(nlohmann::json& annotation_json)
{
    traced_assert(annotation_json.contains(AnnotationFieldAnnotations));
    traced_assert(annotation_json.at(AnnotationFieldAnnotations).is_array());

    return annotation_json.at(AnnotationFieldAnnotations);
}

/**
*/
QRectF ViewPointGenAnnotation::roi() const
{
    QRectF r;

    for (const auto& a : annotations_)
        r = r.united(a->roi());

    for (const auto& f : features_)
        r = r.united(f->roi());

    return r;
}

/********************************************************************************
 * ViewPointGenAnnotations
 ********************************************************************************/

/**
*/
ViewPointGenAnnotation* ViewPointGenAnnotations::addAnnotation(const std::string& name, bool hidden)
{
    anno_map_[ name ] = annotations_.size();
    annotations_.emplace_back(new ViewPointGenAnnotation(name, hidden));
    return annotations_.back().get();
}

/**
*/
void ViewPointGenAnnotations::addAnnotation(std::unique_ptr<ViewPointGenAnnotation>&& a)
{
    traced_assert(a);

    anno_map_[ a->name() ] = annotations_.size();
    annotations_.push_back(std::move(a));
}

/**
*/
ViewPointGenAnnotation* ViewPointGenAnnotations::getOrCreateAnnotation(const std::string& name, bool hidden)
{
    if (hasAnnotation(name))
        return annotation(name);

    return addAnnotation(name, hidden);
}

/**
*/
bool ViewPointGenAnnotations::hasAnnotation(const std::string& name) const
{
    return anno_map_.count(name) > 0;
}

/**
*/
ViewPointGenAnnotation* ViewPointGenAnnotations::annotation(const std::string& name) const
{
    auto it = anno_map_.find(name);
    traced_assert(it != anno_map_.end());

    return annotations_.at(it->second).get();
}

/**
*/
void ViewPointGenAnnotations::toJSON(nlohmann::json& j) const
{
    j = nlohmann::json::array();

    for (const auto& a : annotations_)
    {
        nlohmann::json anno;
        a->toJSON(anno);

        j.push_back(anno);
    }
}

/**
*/
void ViewPointGenAnnotations::print(std::ostream& strm, const std::string& prefix) const
{
    for (const auto& a : annotations_)
        a->print(strm, prefix);
}

/**
*/
QRectF ViewPointGenAnnotations::roi() const
{
    QRectF r;
    for (const auto& a : annotations_)
        r = r.united(a->roi());

    return r;
}

/********************************************************************************
 * ViewPointGenVP
 ********************************************************************************/

const std::string ViewPointGenVP::ViewPointFieldName        = ViewPoint::VP_NAME_KEY;
const std::string ViewPointGenVP::ViewPointFieldID          = ViewPoint::VP_ID_KEY;
const std::string ViewPointGenVP::ViewPointFieldType        = ViewPoint::VP_TYPE_KEY;
const std::string ViewPointGenVP::ViewPointFieldStatus      = ViewPoint::VP_STATUS_KEY;
const std::string ViewPointGenVP::ViewPointFieldAnnotations = ViewPoint::VP_ANNOTATION_KEY;
const std::string ViewPointGenVP::ViewPointFieldFilters     = ViewPoint::VP_FILTERS_KEY;
const std::string ViewPointGenVP::ViewPointFieldPosLat      = ViewPoint::VP_POS_LAT_KEY;
const std::string ViewPointGenVP::ViewPointFieldPosLon      = ViewPoint::VP_POS_LON_KEY;
const std::string ViewPointGenVP::ViewPointFieldWinLat      = ViewPoint::VP_POS_WIN_LAT_KEY;
const std::string ViewPointGenVP::ViewPointFieldWinLon      = ViewPoint::VP_POS_WIN_LON_KEY;

const std::string ViewPointGenVP::StatusNameOpen   = "open";
const std::string ViewPointGenVP::StatusNameClosed = "closed";
const std::string ViewPointGenVP::StatusNameTodo   = "todo";

/**
*/
ViewPointGenVP::ViewPointGenVP(const std::string& name,
                               unsigned int id,
                               const std::string& type,
                               const QRectF& roi,
                               Status status)
:   name_  (name)
,   id_    (id)
,   type_  (type)
,   roi_   (roi)
,   status_(status)
{
}

/**
*/
void ViewPointGenVP::addCustomField(const std::string& name, const QVariant& value)
{
    custom_fields_[ name ] = value;
}

/**
*/
std::string ViewPointGenVP::statusString() const
{
    if (status_ == Status::Open)
        return StatusNameOpen;
    if (status_ == Status::Closed)
        return StatusNameClosed;
    if (status_ == Status::Todo)
        return StatusNameTodo;
    return "";
}

void ViewPointGenVP::appendToDescription(const std::string& text)
{
    description_ += text;
}

/**
*/
void ViewPointGenVP::toJSON(nlohmann::json& j) const
{
    j[ViewPointFieldName  ] = name_;
    j[ViewPointFieldID    ] = id_;
    j[ViewPointFieldType  ] = type_;
    j[ViewPointFieldStatus] = statusString();

    if (description_.size())
        j[ViewPoint::VP_DESCRIPTION_KEY] = description_;

    if (!roi_.isEmpty())
    {
        j[ViewPointFieldPosLat] = roi_.center().x();
        j[ViewPointFieldPosLon] = roi_.center().y();
        j[ViewPointFieldWinLat] = roi_.width();
        j[ViewPointFieldWinLon] = roi_.height();
    }

    nlohmann::json annotations;
    annotations_.toJSON(annotations);
    j[ViewPointFieldAnnotations] = annotations;

    nlohmann::json filters;
    filters_.toJSON(filters);
    if (!filters.is_null())
        j[ViewPointFieldFilters] = filters;

    for (const auto& cf : custom_fields_)
    {
        if (cf.second.type() == QVariant::Type::Int)
            j[cf.first] = cf.second.toInt();
        else if (cf.second.type() == QVariant::Type::UInt)
            j[cf.first] = cf.second.toUInt();
        else if (cf.second.type() == QVariant::Type::Double)
            j[cf.first] = cf.second.toDouble();
        else if (cf.second.type() == QVariant::Type::Bool)
            j[cf.first] = cf.second.toBool();
        else if (cf.second.type() == QVariant::Type::String)
            j[cf.first] = cf.second.toString().toStdString();
    }

    if (no_data_loaded_)
        j[ViewPoint::VP_DS_TYPES_KEY] = nlohmann::json::array();
}

/**
*/
void ViewPointGenVP::print(std::ostream& strm, const std::string& prefix) const
{
    strm << prefix << "[Viewpoint]" << std::endl;
    strm << prefix << "id:     " << id_ << std::endl;
    strm << prefix << "name:   " << name_ << std::endl;
    strm << prefix << "type:   " << type_ << std::endl;
    strm << prefix << "roi:    " << roi_.x() << "," << roi_.y() << " " << roi_.width() << "x" << roi_.height() << std::endl;
    strm << prefix << "status: " << statusString() << std::endl;

    std::string p2 = prefix + "   ";

    annotations_.print(strm, p2);
}

/**
*/
bool ViewPointGenVP::hasAnnotations(const nlohmann::json& vp_json)
{
    if (!vp_json.is_object())
        return false;

    if (vp_json.count(ViewPointFieldAnnotations) == 0)
        return false;

    auto node = vp_json[ViewPointFieldAnnotations];
    if (!node.is_array() || node.size() < 1)
        return false;

    return true;
}

namespace
{
    /**
    */
    // void appendID(std::string& id, const std::string& id_to_add, const std::string& sep)
    // {
    //     if (id_to_add.empty())
    //         return;

    //     if (id.empty())
    //     {
    //         id = id_to_add;
    //         return;
    //     }

    //     id += sep + id_to_add;
    // }

    /**
    */
    std::vector<ViewPointGenVP::JSONFeature> scanForFeaturesRecursive(const nlohmann::json& anno_json, 
                                                                      std::vector<std::string> path, 
                                                                      size_t idx,
                                                                      const std::set<std::string>& feature_types)
    {
        if (!anno_json.is_object() || !anno_json.contains(ViewPointGenAnnotation::AnnotationFieldFeatures) || 
                                      !anno_json.contains(ViewPointGenAnnotation::AnnotationFieldName))
            return {};

        std::string anno_name = anno_json[ ViewPointGenAnnotation::AnnotationFieldName ];

        path.push_back(anno_name);

        const auto& features_json = anno_json[ ViewPointGenAnnotation::AnnotationFieldFeatures ];
        if (!features_json.is_array())
            return {};

        std::vector<ViewPointGenVP::JSONFeature> features;

        //add annotation features
        size_t feat_idx = 0;
        for (const auto& f : features_json)
        {
            ++feat_idx;

            if (!f.is_object() || !f.contains(ViewPointGenFeature::FeatureTypeFieldType))
                return {};

            std::string type = f[ ViewPointGenFeature::FeatureTypeFieldType ];

            if (!feature_types.empty() && feature_types.count(type) == 0)
                continue;

            std::string feat_name;
            if (f.contains(ViewPointGenFeature::FeatureTypeFieldName))
                feat_name = f[ ViewPointGenFeature::FeatureTypeFieldName ];

            ViewPointGenVP::JSONFeature entry;
            entry.annotations  = path;
            entry.name         = feat_name;
            entry.feature_json = f;

            features.push_back(entry);
        }

        //add child annotation features?
        if (anno_json.contains(ViewPointGenAnnotation::AnnotationFieldAnnotations))
        {
            const auto& children_json = anno_json[ ViewPointGenAnnotation::AnnotationFieldAnnotations ];
            if (!children_json.is_array())
                return {};

            size_t child_idx = 0;
            for (const auto& c : children_json)
            {
                //obtain child feats
                auto child_feats = scanForFeaturesRecursive(c, path, child_idx++, feature_types);

                features.insert(features.begin(), child_feats.begin(), child_feats.end());
            }
        }

        return features;
    };
}

/**
*/
std::vector<ViewPointGenVP::JSONFeature> ViewPointGenVP::scanForFeatures(const nlohmann::json& vp_json,
                                                                         const std::set<std::string>& feature_types)
{
    if (!vp_json.is_object() || !vp_json.contains(ViewPointFieldAnnotations))
        return {};

    const auto& annos_json = vp_json[ ViewPointFieldAnnotations ];
    if (!annos_json.is_array())
        return {};

    std::vector<ViewPointGenVP::JSONFeature> features;

    size_t child_idx = 0;
    for (const auto& anno_json : annos_json)
    {
        if (!anno_json.is_object())
            return {};

        auto f = scanForFeaturesRecursive(anno_json, {}, child_idx++, feature_types);

        features.insert(features.begin(), f.begin(), f.end());
    }

    return features;
}

/**
*/
void ViewPointGenVP::autoDetectROI()
{
    roi_ = QRectF();

    auto anno_roi = annotations_.roi();
    if (anno_roi.isEmpty() || anno_roi.isNull())
        return;

    double x0 = anno_roi.left();
    double y0 = anno_roi.top();

    double dx = anno_roi.width()  * 0.05;
    double dy = anno_roi.height() * 0.05;

    roi_ = QRectF(x0 - dx, y0 - dy, anno_roi.width() + dx * 2, anno_roi.height() + dy * 2);
}

/********************************************************************************
 * ViewPointGenerator
 ********************************************************************************/

const std::string ViewPointGenerator::ViewPointsFieldVersion     = "content_version";
const std::string ViewPointGenerator::ViewPointsFieldContentType = "content_type";
const std::string ViewPointGenerator::ViewPointsFieldViewPoints  = "view_points";

/**
*/
ViewPointGenVP* ViewPointGenerator::addViewPoint(const std::string& name,
                                                 unsigned int id,
                                                 const std::string& type,
                                                 const QRectF& roi,
                                                 ViewPointGenVP::Status status)
{
    view_points_.emplace_back(new ViewPointGenVP(name, id, type, roi, status));
    return view_points_.back().get();
}

/**
*/
void ViewPointGenerator::addViewPoint(std::unique_ptr<ViewPointGenVP>&& vp)
{
    view_points_.push_back(std::move(vp));
}

/**
*/
void ViewPointGenerator::toJSON(nlohmann::json& j, 
                                bool with_viewpoints_only,
                                bool with_annotations_only) const
{
    nlohmann::json vp_content;

    vp_content[ViewPointsFieldContentType] = "view_points";
    vp_content[ViewPointsFieldVersion    ] = ViewPoint::VP_COLLECTION_CONTENT_VERSION;

    nlohmann::json viewpoints = nlohmann::json::array();

    size_t added = 0;

    for (const auto& vp : view_points_)
    {
        if (with_annotations_only && !vp->hasAnnotations())
            continue;

        nlohmann::json viewpoint;
        vp->toJSON(viewpoint);

        viewpoints.push_back(viewpoint);

        ++added;
    }

    vp_content[ViewPointsFieldViewPoints] = viewpoints;

    if (with_viewpoints_only && added == 0)
        return;

    j = vp_content;
}

/**
*/
nlohmann::json ViewPointGenerator::toJSON(bool with_viewpoints_only,
                                          bool with_annotations_only) const
{
    nlohmann::json j;
    toJSON(j, with_viewpoints_only, with_annotations_only);

    return j;
}

/**
*/
boost::optional<nlohmann::json> ViewPointGenerator::viewPointJSON(const nlohmann::json& vps_json, 
                                                                  size_t idx, 
                                                                  bool with_annotations_only)
{
    if (!vps_json.is_object())
        return {};

    if (vps_json.count(ViewPointsFieldViewPoints) == 0)
        return {};

    auto vps_node = vps_json[ViewPointsFieldViewPoints];
    if (!vps_node.is_array() || vps_node.size() <= idx)
        return {};

    auto vp_node = vps_node.at(idx);
    if (!vp_node.is_object())
        return {};

    if (with_annotations_only && !ViewPointGenVP::hasAnnotations(vp_node))
        return {};

    return vp_node;
}
