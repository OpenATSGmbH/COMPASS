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

#include "rasterreference.h"
#include "histogram_raw.h"
#include "scatterseries.h"
#include "grid2dlayer.h"
#include "plotmetadata.h"
#include "colorlegend.h"
#include "grid2drendersettings.h"

#include <memory>
#include <vector>
#include <map>
#include <set>

#include <Eigen/Core>

#include <QColor>
#include <QRectF>
#include <QVariant>
#include <QImage>

#include <boost/optional.hpp>

/**
*/
class ViewPointGenFilter
{
public:
    ViewPointGenFilter() = default;
    virtual ~ViewPointGenFilter() = default;

    virtual std::string name() const = 0;
    virtual void toJSON(nlohmann::json& j) const = 0;
};

/**
*/
class ViewPointGenFilters
{
public:
    ViewPointGenFilters() = default;
    virtual ~ViewPointGenFilters() = default;

    void addFilter(std::unique_ptr<ViewPointGenFilter>&& filter);
    size_t size() const;

    void toJSON(nlohmann::json& j) const;

private:
    std::map<std::string, std::unique_ptr<ViewPointGenFilter>> filters_;
};

/**
*/
class ViewPointGenFilterUTN : public ViewPointGenFilter
{
public:
    ViewPointGenFilterUTN(const std::vector<uint32_t>& utns);
    ViewPointGenFilterUTN(uint32_t utn);
    virtual ~ViewPointGenFilterUTN() = default;

    virtual std::string name() const override { return UTNFilterNodeName; }
    virtual void toJSON(nlohmann::json& j) const override;

    static const std::string UTNFilterNodeName;
    static const std::string UTNFilterFieldName;

private:
    std::vector<uint32_t> utns_;
};

/**
*/
class ViewPointGenFeature
{
public:
    ViewPointGenFeature(const std::string& type);
    virtual ~ViewPointGenFeature() = default;

    void setName(const std::string& name) { name_ = name; }
    void setColor(const QColor& c) { color_ = c; }
    void setPlotMetadata(const PlotMetadata& metadata) { plot_metadata_ = metadata; }

    void writeBinaryIfPossible(bool ok) { write_binary_if_possible_ = ok; }

    void toJSON(nlohmann::json& j) const;
    void print(std::ostream& strm, const std::string& prefix = "") const;

    const std::string& name() const { return name_; } 
    const std::string& type() const { return type_; }

    virtual size_t size() const = 0;
    virtual QRectF roi() const { return QRectF(); }
    
    static const std::string FeatureTypeFieldName;
    static const std::string FeatureTypeFieldType;
    static const std::string FeatureFieldNameProps;
    static const std::string FeatureFieldNamePropColor;
    static const std::string FeatureFieldNamePlotMetadata;

protected:
    virtual void toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const = 0;

    QColor color_;
    boost::optional<PlotMetadata> plot_metadata_;

private:
    std::string name_;
    std::string type_;

    bool write_binary_if_possible_ = false;
};

/**
*/
class ViewPointGenFeaturePointGeometry : public ViewPointGenFeature
{
public:
    ViewPointGenFeaturePointGeometry(const std::string& type,
                                     const std::vector<Eigen::Vector2d>& positions = std::vector<Eigen::Vector2d>(),
                                     const std::vector<QColor>& colors = std::vector<QColor>(),
                                     bool enable_color_vector = true);
    virtual ~ViewPointGenFeaturePointGeometry() = default;

    virtual void reserve(size_t n, bool reserve_cols);
    virtual size_t size() const override { return positions_.size(); }
    virtual QRectF roi() const override;

    void addPoint(const Eigen::Vector2d& pos, 
                  const boost::optional<QColor>& color = boost::optional<QColor>());
    void addPoints(const std::vector<Eigen::Vector2d>& positions,
                   const boost::optional<std::vector<QColor>>& colors = boost::optional<std::vector<QColor>>());

    static nlohmann::json& getCoordinatesJSON(nlohmann::json& feature_json);

    static const std::string FeatureFieldNameGeom;
    static const std::string FeatureFieldNameCoords;
    static const std::string FeatureFieldNameColors;

protected:
    virtual void toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const override;

    virtual void writeGeometry(nlohmann::json& j) const {};
    virtual void writeProperties(nlohmann::json& j) const {};

private:
    std::vector<Eigen::Vector2d> positions_;
    std::vector<QColor>          colors_;
    bool                         enable_color_vector_ = true;
};

/**
*/
class ViewPointGenFeaturePoints : public ViewPointGenFeaturePointGeometry
{
public:
    enum class Symbol
    {
        Circle = 0,
        Triangle,
        Square,
        Cross,
        Border,
        BorderThick
    };

    ViewPointGenFeaturePoints(Symbol symbol = Symbol::Square,
                              float symbol_size = 6.0f,
                              const std::vector<Eigen::Vector2d>& positions = std::vector<Eigen::Vector2d>(),
                              const std::vector<QColor>& colors = std::vector<QColor>(),
                              bool enable_color_vector = true);
    virtual ~ViewPointGenFeaturePoints() = default;

    static const std::string FeatureName;
    static const std::string FeaturePointsFieldNameSymbol;
    static const std::string FeaturePointsFieldNameSymbolSize;

    static const std::string SymbolNameCircle;
    static const std::string SymbolNameTriangle;
    static const std::string SymbolNameSquare;
    static const std::string SymbolNameCross;
    static const std::string SymbolNameBorder;
    static const std::string SymbolNameBorderThick;

protected:
    virtual void writeProperties(nlohmann::json& j) const override;

private:
    std::string symbolString() const;

    Symbol symbol_;
    float  symbol_size_;
};

/**
*/
class ViewPointGenFeatureStyledLine : public ViewPointGenFeaturePointGeometry
{
public:
    enum class LineStyle
    {
        Solid = 0,
        Dotted
    };

    ViewPointGenFeatureStyledLine(const std::string& type,
                                  float line_width,
                                  LineStyle line_style,
                                  const std::vector<Eigen::Vector2d>& positions,
                                  const std::vector<QColor>& colors,
                                  bool enable_color_vector);
    virtual ~ViewPointGenFeatureStyledLine() = default;

    static const std::string FeatureLineStringFieldNameLineWidth;
    static const std::string FeatureLineStringFieldNameLineStyle;

protected:
    virtual void writeProperties(nlohmann::json& j) const override;

    std::string styleString() const;

    float     line_width_;
    LineStyle line_style_;
};

/**
*/
class ViewPointGenFeatureLineString : public ViewPointGenFeatureStyledLine
{
public:
    ViewPointGenFeatureLineString(bool interpolated,
                                  float line_width = 1.0f,
                                  LineStyle line_style = LineStyle::Solid,
                                  const std::vector<Eigen::Vector2d>& positions = std::vector<Eigen::Vector2d>(),
                                  const std::vector<QColor>& colors = std::vector<QColor>(),
                                  bool enable_color_vector = true);
    virtual ~ViewPointGenFeatureLineString() = default;

    static const std::string FeatureName;
    static const std::string FeatureNameInterp;
    
};

/**
*/
class ViewPointGenFeatureLines : public ViewPointGenFeatureStyledLine
{
public:
    ViewPointGenFeatureLines(float line_width = 1.0f,
                             LineStyle line_style = LineStyle::Solid,
                             const std::vector<Eigen::Vector2d>& positions = std::vector<Eigen::Vector2d>(),
                             const std::vector<QColor>& colors = std::vector<QColor>(),
                             bool enable_color_vector = true);
    virtual ~ViewPointGenFeatureLines() = default;

    static const std::string FeatureName;
};

/**
*/
class ViewPointGenFeatureErrEllipses : public ViewPointGenFeaturePointGeometry
{
public:
    ViewPointGenFeatureErrEllipses(float line_width = 1.0f,
                                   size_t num_points = 32,
                                   const std::vector<Eigen::Vector2d>& positions = std::vector<Eigen::Vector2d>(),
                                   const std::vector<QColor>& colors = std::vector<QColor>(),
                                   const std::vector<Eigen::Vector3d>& sizes = std::vector<Eigen::Vector3d>(),
                                   bool enable_color_vector = true);
    virtual ~ViewPointGenFeatureErrEllipses() = default;

    virtual void reserve(size_t n, bool reserve_cols) override;

    void addSize(const Eigen::Vector3d& size);
    void addSizes(const std::vector<Eigen::Vector3d>& sizes);

    static const std::string FeatureName;
    static const std::string FeatureErrEllipsesFieldNameLineWidth;
    static const std::string FeatureErrEllipsesFieldNameNumPoints;
    static const std::string FeatureErrEllipsesFieldNameSizes;

protected:
    virtual void writeGeometry(nlohmann::json& j) const override;
    virtual void writeProperties(nlohmann::json& j) const override;

private:
    float  line_width_;
    size_t num_points_;

    std::vector<Eigen::Vector3d> sizes_;
};

/**
*/
class ViewPointGenFeatureText : public ViewPointGenFeature
{
public:
    enum class TextDirection
    {
        RightUp = 0,
        RightDown,
        LeftUp,
        LeftDown
    };

    ViewPointGenFeatureText(const std::string& text,
                            double x,
                            double y,
                            float font_size = 10.0,
                            TextDirection text_dir = TextDirection::RightUp);
    virtual ~ViewPointGenFeatureText() = default;

    virtual size_t size() const { return 1; }
    virtual QRectF roi() const override;

    static const std::string FeatureName;
    static const std::string FeatureTextFieldNameText;
    static const std::string FeatureTextFieldNamePos;
    static const std::string FeatureTextFieldNameDir;
    static const std::string FeatureTextFieldNameFontSize;

    static const std::string TextDirNameRightUp;
    static const std::string TextDirNameRightDown;
    static const std::string TextDirNameLeftUp;
    static const std::string TextDirNameLeftDown;

protected:
    virtual void toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const override;

private:
    std::string textDirString() const;

    std::string   text_;
    double        x_;
    double        y_;
    float         font_size_;
    TextDirection text_dir_;
};

/**
*/
class ViewPointGenFeatureGeoImage : public ViewPointGenFeature
{
public:
    ViewPointGenFeatureGeoImage(const std::string& fn,
                                const RasterReference& ref,
                                const ColorLegend& legend = ColorLegend(),
                                bool subsample = true,
                                int subsampling = -1);
    ViewPointGenFeatureGeoImage(const QImage& data,
                                const RasterReference& ref,
                                const ColorLegend& legend = ColorLegend(),
                                bool subsample = true,
                                int subsampling = -1);
    virtual ~ViewPointGenFeatureGeoImage() = default;

    static std::string imageToByteString(const QImage& img, const std::string& format);
    static std::string imageToByteStringWithMetadata(const QImage& img);
    static QImage byteStringWithMetadataToImage(const std::string& str);

    virtual size_t size() const { return 1; }
    virtual QRectF roi() const override;

    static const std::string FeatureName;
    static const std::string FeatureGeoImageFieldNameSource;
    static const std::string FeatureGeoImageFieldNameFn;
    static const std::string FeatureGeoImageFieldNameData;
    static const std::string FeatureGeoImageFieldNameReference;
    static const std::string FeatureGeoImageFieldNameLegend;
    static const std::string FeatureGeoImageFieldNameSubsample;
    static const std::string FeatureGeoImageFieldNameSubsampling;

protected:
    virtual void toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const override;

private:
    std::string     fn_;
    QImage          data_;
    RasterReference ref_;
    ColorLegend     legend_;
    bool            subsample_ = false;
    int             subsampling_;
};

/**
*/
class ViewPointGenFeatureGrid : public ViewPointGenFeature
{
public:
    ViewPointGenFeatureGrid(const Grid2DLayer& grid, 
                            const boost::optional<Grid2DRenderSettings>& render_settings = boost::optional<Grid2DRenderSettings>(),
                            const boost::optional<PlotMetadata>& metadata = boost::optional<PlotMetadata>());
    virtual ~ViewPointGenFeatureGrid() = default;

    virtual size_t size() const { return 1; }
    virtual QRectF roi() const override;

    static const std::string FeatureName;
    static const std::string FeatureGridFieldNameGrid;
    static const std::string FeatureGridFieldNameRenderSettings;

protected:
    virtual void toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const override;

private:
    Grid2DLayer grid_;
    boost::optional<Grid2DRenderSettings> render_settings_;
};

/**
*/
class ViewPointGenFeatureHistogram : public ViewPointGenFeature
{
public:
    ViewPointGenFeatureHistogram(const RawHistogram& histogram,
                                 const std::string& series_name = "",
                                 const QColor& series_color = Qt::blue,
                                 const boost::optional<bool>& use_log_scale = boost::optional<bool>(),
                                 const PlotMetadata& metadata = PlotMetadata());
    ViewPointGenFeatureHistogram(const RawHistogramCollection& histogram_collection,
                                 const PlotMetadata& metadata = PlotMetadata());
    virtual ~ViewPointGenFeatureHistogram() = default;

    RawHistogramCollection& histograms() { return histogram_; }

    virtual size_t size() const override;

    static const std::string FeatureName;
    static const std::string FeatureHistogramFieldNameHistogram;

protected:
    virtual void toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const override;

private:
    RawHistogramCollection histogram_;
};

/**
*/
class ViewPointGenFeatureScatterSeries : public ViewPointGenFeature
{
public:
    ViewPointGenFeatureScatterSeries(const ScatterSeries& scatter_series,
                                     const std::string& series_name = "",
                                     const QColor& series_color = Qt::blue,
                                     double marker_size = 8.0,
                                     const boost::optional<bool>& use_connection_lines = boost::optional<bool>(),
                                     const PlotMetadata& metadata = PlotMetadata());
    ViewPointGenFeatureScatterSeries(const ScatterSeriesCollection& scatter_series_collection,
                                     const PlotMetadata& metadata = PlotMetadata());
    virtual ~ViewPointGenFeatureScatterSeries() = default;

    ScatterSeriesCollection& scatterSeries() { return scatter_series_; }

    virtual size_t size() const override;

    static const std::string FeatureName;
    static const std::string FeatureHistogramFieldNameScatterSeries;

protected:
    virtual void toJSON_impl(nlohmann::json& j, bool write_binary_if_possible) const override;

private:
    ScatterSeriesCollection scatter_series_;
};

/**
*/
class ViewPointGenAnnotation
{
public:
    ViewPointGenAnnotation(const std::string& name,
                           bool hidden = false);
    virtual ~ViewPointGenAnnotation() = default;

    void setSymbolColor(const QColor& color);

    void addFeature(std::unique_ptr<ViewPointGenFeature>&& feat);
    void addFeature(ViewPointGenFeature* feat);

    template <class T, typename... Targs>
    T* addFeature(const std::string& name, Targs&&... args)
    {
        T* feat = new T(std::forward<Targs>(args)...);
        feat->setName(name);

        feat_map_[ name ] = features_.size();
        features_.push_back(std::unique_ptr<T>(feat));

        return feat;
    }

    bool hasFeature(const std::string& name) const;

    ViewPointGenFeature* getFeature(const std::string& name);
    const ViewPointGenFeature* getFeature(const std::string& name) const;

    template <class T>
    T* getFeatureAs(const std::string& name)
    {
        auto f = getFeature(name);
        return dynamic_cast<T*>(f);
    }

    template <class T>
    const T* getFeatureAs(const std::string& name) const
    {
        auto f = getFeature(name);
        return dynamic_cast<const T*>(f);
    }

    ViewPointGenAnnotation* addAnnotation(const std::string& name, bool hidden = false);
    void addAnnotation(std::unique_ptr<ViewPointGenAnnotation>&& a);

    ViewPointGenAnnotation* getOrCreateAnnotation(const std::string& name, bool hidden = false);

    bool hasAnnotation(const std::string& name) const;
    ViewPointGenAnnotation* annotation(const std::string& name) const;

    const std::string& name() const { return name_; }

    size_t size() const { return annotations_.size(); }
    size_t numFeatures() const { return features_.size(); }

    void toJSON(nlohmann::json& j) const;
    void print(std::ostream& strm, const std::string& prefix = "") const;

    QRectF roi() const;

    static const std::string AnnotationFieldName;
    static const std::string AnnotationFieldHidden;
    static const std::string AnnotationFieldSymbolColor;
    static const std::string AnnotationFieldFeatures;
    static const std::string AnnotationFieldAnnotations;

    static nlohmann::json& getFeaturesJSON(nlohmann::json& annotation_json);
    static nlohmann::json& getFeatureJSON(nlohmann::json& annotation_json, size_t idx);
    static nlohmann::json& getChildrenJSON(nlohmann::json& annotation_json);

private:
    std::string             name_;
    bool                    hidden_;
    boost::optional<QColor> symbol_color_;

    std::vector<std::unique_ptr<ViewPointGenFeature>>    features_;
    std::vector<std::unique_ptr<ViewPointGenAnnotation>> annotations_;
    std::map<std::string, size_t>                        feat_map_;
    std::map<std::string, size_t>                        anno_map_;
};

/**
*/
class ViewPointGenAnnotations
{
public:
    ViewPointGenAnnotations() = default;
    virtual ~ViewPointGenAnnotations() = default;

    ViewPointGenAnnotation* addAnnotation(const std::string& name, bool hidden = false);
    void addAnnotation(std::unique_ptr<ViewPointGenAnnotation>&& a);

    ViewPointGenAnnotation* getOrCreateAnnotation(const std::string& name, bool hidden = false);

    size_t size() const { return annotations_.size(); }
    ViewPointGenAnnotation* annotation(size_t idx) const { return annotations_.at(idx).get(); }

    bool hasAnnotation(const std::string& name) const;
    ViewPointGenAnnotation* annotation(const std::string& name) const;

    void toJSON(nlohmann::json& j) const;
    void print(std::ostream& strm, const std::string& prefix = "") const;

    QRectF roi() const;

private:
    std::vector<std::unique_ptr<ViewPointGenAnnotation>> annotations_;
    std::map<std::string, size_t>                        anno_map_;
};

/**
*/
class ViewPointGenVP
{
public:
    enum class Status
    {
        Open = 0, 
        Closed,
        Todo
    };

    ViewPointGenVP(const std::string& name,
                   unsigned int id,
                   const std::string& type,
                   const QRectF& roi = QRectF(),
                   Status status = Status::Open);
    virtual ~ViewPointGenVP() = default;

    bool hasAnnotations() const { return (annotations_.size() > 0); }
    static bool hasAnnotations(const nlohmann::json& vp_json);

    void setROI(const QRectF& roi) { roi_ = roi; }
    void autoDetectROI();

    void addCustomField(const std::string& name, const QVariant& value);

    ViewPointGenAnnotations& annotations() { return annotations_; }
    ViewPointGenFilters& filters() { return filters_; }

    void appendToDescription(const std::string& text);

    void toJSON(nlohmann::json& j) const;
    void print(std::ostream& strm, const std::string& prefix = "") const;

    std::string statusString() const;

    struct JSONFeature
    {
        std::vector<std::string> annotations;
        std::string              name;
        nlohmann::json           feature_json;
    };

    static std::vector<JSONFeature> scanForFeatures(const nlohmann::json& vp_json,
                                                    const std::set<std::string>& feature_types = std::set<std::string>());

    static const std::string ViewPointFieldName;
    static const std::string ViewPointFieldID;
    static const std::string ViewPointFieldType;
    static const std::string ViewPointFieldStatus;
    static const std::string ViewPointFieldAnnotations;
    static const std::string ViewPointFieldFilters;
    static const std::string ViewPointFieldPosLat;
    static const std::string ViewPointFieldPosLon;
    static const std::string ViewPointFieldWinLat;
    static const std::string ViewPointFieldWinLon;

    static const std::string StatusNameOpen;
    static const std::string StatusNameClosed;
    static const std::string StatusNameTodo;

    bool noDataLoaded() const { return no_data_loaded_; }
    void noDataLoaded(bool value) { no_data_loaded_ = value; }

  private:
    std::string  name_;
    unsigned int id_;
    std::string  type_;
    QRectF       roi_;
    Status       status_;
    std::string  description_;

    ViewPointGenAnnotations annotations_;
    ViewPointGenFilters     filters_;
    bool no_data_loaded_ {false}; // set if no dbcontent data should be loaded

    std::map<std::string, QVariant> custom_fields_;
};

/**
*/
class ViewPointGenerator
{
public:
    ViewPointGenerator() = default;
    virtual ~ViewPointGenerator() = default;

    ViewPointGenVP* addViewPoint(const std::string& name,
                                 unsigned int id,
                                 const std::string& type,
                                 const QRectF& roi = QRectF(),
                                 ViewPointGenVP::Status status = ViewPointGenVP::Status::Open);
    void addViewPoint(std::unique_ptr<ViewPointGenVP>&& vp);

    size_t size() const { return view_points_.size(); }

    static const std::string Version;

    static const std::string ViewPointsFieldVersion;
    static const std::string ViewPointsFieldContentType;
    static const std::string ViewPointsFieldViewPoints;

    nlohmann::json toJSON(bool with_viewpoints_only = false,
                          bool with_annotations_only = false) const;

    static boost::optional<nlohmann::json> viewPointJSON(const nlohmann::json& vps_json, 
                                                         size_t idx, 
                                                         bool with_annotations_only = false);
private:
    void toJSON(nlohmann::json& j, 
                bool with_viewpoints_only,
                bool with_annotations_only) const;

    std::vector<std::unique_ptr<ViewPointGenVP>> view_points_;
};
