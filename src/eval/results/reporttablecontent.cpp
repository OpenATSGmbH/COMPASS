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

#include "results/reporttablecontent.h"
#include "results/base/featuredefinition_grid.h"

#include "buffer.h"
#include "logger.h"
#include "stringconv.h"
#include "timeconv.h"
#include "traced_assert.h"
#include "viewpointgenerator.h"
#include "viewpointkeys.h"

#include "json.hpp"

#include <algorithm>

using namespace EvaluationRequirementResult;

namespace
{
    const double BoundsEps = 1e-12;

    /**
     * Reads a column of a row as a json value, null when the column is missing or the row is null.
     */
    nlohmann::json columnValue(const ReportTableColumn& column,
                               const Buffer& buffer,
                               unsigned int row)
    {
        if (!buffer.properties().hasProperty(column.name))
            return nlohmann::json();

        switch (column.data_type)
        {
            case PropertyDataType::BOOL:
                if (buffer.has<bool>(column.name) && !buffer.get<bool>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<bool>(column.name).get(row));
                break;
            case PropertyDataType::CHAR:
                if (buffer.has<char>(column.name) && !buffer.get<char>(column.name).isNull(row))
                    return nlohmann::json((int)buffer.get<char>(column.name).get(row));
                break;
            case PropertyDataType::UCHAR:
                if (buffer.has<unsigned char>(column.name) && !buffer.get<unsigned char>(column.name).isNull(row))
                    return nlohmann::json((unsigned int)buffer.get<unsigned char>(column.name).get(row));
                break;
            case PropertyDataType::INT:
                if (buffer.has<int>(column.name) && !buffer.get<int>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<int>(column.name).get(row));
                break;
            case PropertyDataType::UINT:
                if (buffer.has<unsigned int>(column.name) && !buffer.get<unsigned int>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<unsigned int>(column.name).get(row));
                break;
            case PropertyDataType::LONGINT:
                if (buffer.has<std::int64_t>(column.name) && !buffer.get<std::int64_t>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<std::int64_t>(column.name).get(row));
                break;
            case PropertyDataType::ULONGINT:
                if (buffer.has<std::uint64_t>(column.name) && !buffer.get<std::uint64_t>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<std::uint64_t>(column.name).get(row));
                break;
            case PropertyDataType::FLOAT:
                if (buffer.has<float>(column.name) && !buffer.get<float>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<float>(column.name).get(row));
                break;
            case PropertyDataType::DOUBLE:
                if (buffer.has<double>(column.name) && !buffer.get<double>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<double>(column.name).get(row));
                break;
            case PropertyDataType::STRING:
            case PropertyDataType::JSON:
                if (buffer.has<std::string>(column.name) && !buffer.get<std::string>(column.name).isNull(row))
                    return nlohmann::json(buffer.get<std::string>(column.name).get(row));
                break;
            case PropertyDataType::TIMESTAMP:
                if (buffer.has<boost::posix_time::ptime>(column.name) &&
                    !buffer.get<boost::posix_time::ptime>(column.name).isNull(row))
                    return nlohmann::json(Utils::Time::toString(
                        buffer.get<boost::posix_time::ptime>(column.name).get(row)));
                break;
            default:
                break;
        }

        return nlohmann::json();
    }

    /**
     * Reads a double column of a row, none when the column is missing or the row is null.
     */
    boost::optional<double> doubleValue(const Buffer& buffer,
                                        const std::string& name,
                                        unsigned int row)
    {
        if (!buffer.has<double>(name) || buffer.get<double>(name).isNull(row))
            return boost::none;

        return buffer.get<double>(name).get(row);
    }

    /**
     * Reads a bool column of a row, none when the column is missing or the row is null.
     */
    boost::optional<bool> boolValue(const Buffer& buffer,
                                    const std::string& name,
                                    unsigned int row)
    {
        if (!buffer.has<bool>(name) || buffer.get<bool>(name).isNull(row))
            return boost::none;

        return buffer.get<bool>(name).get(row);
    }
}

/************************************************************************************************
 * ReportTableContent
 ************************************************************************************************/

namespace ReportTableContent
{

const std::string ColumnDtPrev           = "dt_prev_s";
const std::string ColumnTstLatitude      = "tst_lat";
const std::string ColumnTstLongitude     = "tst_lon";
const std::string ColumnRefLatitude      = "ref_lat";
const std::string ColumnRefLongitude     = "ref_lon";
const std::string ColumnRefLatitudeEnd   = "ref_lat_end";
const std::string ColumnRefLongitudeEnd  = "ref_lon_end";
const std::string ColumnCommentHeader    = "Comment";

/**
 */
bool isGapTable(const ReportTableDefinition& definition)
{
    return !definition.hasColumn(ColumnTstLatitude);
}

/**
 * The record number and the UTN are the join keys, they carry no information for the reader.
 */
std::vector<std::string> detailsTableHeaders(const ReportTableDefinition& definition)
{
    std::vector<std::string> headers;

    for (const auto& column : definition.columns)
    {
        if (column.name == ReportTableDefinition::KeyColumnName ||
            column.name == ReportTableDefinition::UTNColumnName)
            continue;

        headers.push_back(column.display_name);
    }

    headers.push_back(ColumnCommentHeader);

    return headers;
}

/**
 */
nlohmann::json::array_t detailsTableValues(const ReportTableDefinition& definition,
                                           const Buffer& buffer,
                                           unsigned int row)
{
    nlohmann::json::array_t values;

    for (const auto& column : definition.columns)
    {
        if (column.name == ReportTableDefinition::KeyColumnName ||
            column.name == ReportTableDefinition::UTNColumnName)
            continue;

        values.push_back(columnValue(column, buffer, row));
    }

    values.push_back(rowComment(definition, buffer, row));

    return values;
}

/**
 * The check outcome of a row, read from the flag columns the families write.
 */
bool rowIsOk(const ReportTableDefinition& definition,
             const Buffer& buffer,
             unsigned int row)
{
    auto check_passed = boolValue(buffer, "check_passed", row);
    if (check_passed.has_value() && !check_passed.value())
        return false;

    auto ok = boolValue(buffer, "ok", row);
    if (ok.has_value() && !ok.value())
        return false;

    auto correct = boolValue(buffer, "correct", row);
    if (correct.has_value() && !correct.value())
        return false;

    auto is_dubious = boolValue(buffer, "is_dubious", row);
    if (is_dubious.has_value() && is_dubious.value())
        return false;

    auto extra = boolValue(buffer, "extra", row);
    if (extra.has_value() && extra.value())
        return false;

    auto missed = doubleValue(buffer, "missed_updates", row);
    if (missed.has_value() && missed.value() > 0.0)
        return false;

    if (buffer.has<unsigned int>("missed_updates") &&
        !buffer.get<unsigned int>("missed_updates").isNull(row) &&
        buffer.get<unsigned int>("missed_updates").get(row) > 0)
        return false;

    return true;
}

/**
 * The comment is not stored, it is built from the flag columns of the row. It names what went
 * wrong, a row without a problem has no comment.
 */
std::string rowComment(const ReportTableDefinition& definition,
                       const Buffer& buffer,
                       unsigned int row)
{
    std::vector<std::string> parts;

    auto ref_exists = boolValue(buffer, "ref_exists", row);
    if (ref_exists.has_value() && !ref_exists.value())
        parts.push_back("No reference");

    auto pos_inside = boolValue(buffer, "pos_inside", row);
    if (pos_inside.has_value() && !pos_inside.value())
        parts.push_back("Outside sector");

    auto inside = boolValue(buffer, "inside", row);
    if (inside.has_value() && !inside.value())
        parts.push_back("Outside sector");

    auto check_passed = boolValue(buffer, "check_passed", row);
    if (check_passed.has_value() && !check_passed.value())
        parts.push_back("Check failed");

    auto ok = boolValue(buffer, "ok", row);
    if (ok.has_value() && !ok.value())
        parts.push_back("Not OK");

    auto correct = boolValue(buffer, "correct", row);
    if (correct.has_value() && !correct.value())
        parts.push_back("Not correct");

    auto extra = boolValue(buffer, "extra", row);
    if (extra.has_value() && extra.value())
        parts.push_back("Extra");

    auto is_dubious = boolValue(buffer, "is_dubious", row);
    if (is_dubious.has_value() && is_dubious.value())
        parts.push_back("Dubious");

    if (buffer.has<std::string>("dubious_reasons") &&
        !buffer.get<std::string>("dubious_reasons").isNull(row))
    {
        auto reasons = buffer.get<std::string>("dubious_reasons").get(row);
        if (!reasons.empty())
            parts.push_back(reasons);
    }

    boost::optional<double> missed = doubleValue(buffer, "missed_updates", row);
    if (!missed.has_value() &&
        buffer.has<unsigned int>("missed_updates") &&
        !buffer.get<unsigned int>("missed_updates").isNull(row))
        missed = (double)buffer.get<unsigned int>("missed_updates").get(row);

    if (missed.has_value() && missed.value() > 0.0)
        parts.push_back(Utils::String::doubleToStringPrecision(missed.value(), 2) + " missed updates");

    std::string comment;
    for (const auto& part : parts)
        comment += (comment.empty() ? "" : ", ") + part;

    return comment;
}

/**
 */
boost::optional<boost::posix_time::ptime> rowTimestamp(const Buffer& buffer,
                                                       unsigned int row)
{
    const std::string& name = ReportTableDefinition::TimestampColumnName;

    if (!buffer.has<boost::posix_time::ptime>(name) ||
        buffer.get<boost::posix_time::ptime>(name).isNull(row))
        return boost::none;

    return buffer.get<boost::posix_time::ptime>(name).get(row);
}

/**
 */
QRectF RowPositions::bounds(double eps) const
{
    QRectF r;

    auto add = [ & ] (const std::pair<double, double>& p)
    {
        QRectF p_rect(p.first - eps, p.second - eps, 2 * eps, 2 * eps);
        r = r.isNull() ? p_rect : r.united(p_rect);
    };

    if (event.has_value())
        add(event.value());
    if (reference.has_value())
        add(reference.value());

    return r;
}

/**
 * A test-keyed row points at the test position and measures against the reference position,
 * a gap row spans the first and the last reference sample inside the gap.
 */
RowPositions rowPositions(const ReportTableDefinition& definition,
                          const Buffer& buffer,
                          unsigned int row)
{
    RowPositions positions;

    auto readPair = [ & ] (const std::string& lat_name, const std::string& lon_name)
        -> boost::optional<std::pair<double, double>>
    {
        auto lat = doubleValue(buffer, lat_name, row);
        auto lon = doubleValue(buffer, lon_name, row);

        if (!lat.has_value() || !lon.has_value())
            return boost::none;

        return std::make_pair(lat.value(), lon.value());
    };

    if (isGapTable(definition))
    {
        positions.event     = readPair(ColumnRefLatitude, ColumnRefLongitude);
        positions.reference = readPair(ColumnRefLatitudeEnd, ColumnRefLongitudeEnd);
    }
    else
    {
        positions.event     = readPair(ColumnTstLatitude, ColumnTstLongitude);
        positions.reference = readPair(ColumnRefLatitude, ColumnRefLongitude);
    }

    return positions;
}

/**
 */
QRectF failedRowBounds(const ReportTableDefinition& definition,
                       const Buffer& buffer)
{
    QRectF failed_bounds;
    QRectF all_bounds;

    for (unsigned int row = 0; row < buffer.size(); ++row)
    {
        auto positions = rowPositions(definition, buffer, row);
        if (!positions.valid())
            continue;

        auto b = positions.bounds(BoundsEps);

        all_bounds = all_bounds.isNull() ? b : all_bounds.united(b);

        if (!rowIsOk(definition, buffer, row))
            failed_bounds = failed_bounds.isNull() ? b : failed_bounds.united(b);
    }

    return failed_bounds.isNull() ? all_bounds : failed_bounds;
}

/**
 */
void createOverviewAnnotations(nlohmann::json& annotations_json,
                               const ReportTableDefinition& definition,
                               const Buffer& buffer)
{
    const bool gaps = isGapTable(definition);

    for (unsigned int row = 0; row < buffer.size(); ++row)
    {
        auto positions = rowPositions(definition, buffer, row);
        if (!positions.valid())
            continue;

        auto type = rowIsOk(definition, buffer, row) ? EvaluationAnnotations::ArrayType::TypeOk
                                                     : EvaluationAnnotations::ArrayType::TypeError;

        EvaluationAnnotations::addPosition(annotations_json,
                                           positions.event->first,
                                           positions.event->second, type);

        if (positions.reference.has_value())
        {
            EvaluationAnnotations::addPosition(annotations_json,
                                               positions.reference->first,
                                               positions.reference->second, type);

            //a gap is drawn as its span, a measured value as the offset to its reference
            if (gaps || !rowIsOk(definition, buffer, row))
                EvaluationAnnotations::addLine(annotations_json,
                                               positions.event->first,
                                               positions.event->second,
                                               positions.reference->first,
                                               positions.reference->second, type);
        }
    }
}

/**
 */
void createHighlightAnnotations(nlohmann::json& annotations_json,
                                const ReportTableDefinition& definition,
                                const Buffer& buffer,
                                unsigned int row)
{
    auto positions = rowPositions(definition, buffer, row);
    if (!positions.valid())
        return;

    const auto type = EvaluationAnnotations::ArrayType::TypeHighlight;

    EvaluationAnnotations::addPosition(annotations_json,
                                       positions.event->first,
                                       positions.event->second, type);

    if (positions.reference.has_value())
    {
        EvaluationAnnotations::addPosition(annotations_json,
                                           positions.reference->first,
                                           positions.reference->second, type);
        EvaluationAnnotations::addLine(annotations_json,
                                       positions.event->first,
                                       positions.event->second,
                                       positions.reference->first,
                                       positions.reference->second, type);
    }
}

/**
 */
nlohmann::json& addRootAnnotation(nlohmann::json& view_point, const std::string& name)
{
    view_point[ ViewPointKeys::VP_ANNOTATION_KEY ] = nlohmann::json::array();
    auto& annotations = view_point[ ViewPointKeys::VP_ANNOTATION_KEY ];

    ViewPointGenAnnotation root_annotation(name);

    nlohmann::json root_annotation_json;
    root_annotation.toJSON(root_annotation_json);

    annotations.push_back(root_annotation_json);

    return ViewPointGenAnnotation::getChildrenJSON(annotations.at(0));
}

/**
 */
QRectF overviewRegion(const ReportTableDefinition& definition,
                      const Buffer& buffer,
                      double min_size)
{
    //x = latitude, y = longitude
    const QRectF bounds = failedRowBounds(definition, buffer);
    if (bounds.isNull())
        return QRectF();

    const double lat_size = std::max(bounds.width() , min_size);
    const double lon_size = std::max(bounds.height(), min_size);

    return QRectF(bounds.center().y() - lon_size / 2.0,
                  bounds.center().x() - lat_size / 2.0,
                  lon_size,
                  lat_size);
}

/**
 */
std::unique_ptr<ViewPointGenFeatureGrid> createOverviewGrid(const ReportTableDefinition& definition,
                                                            const Buffer& buffer,
                                                            const QRectF& region,
                                                            unsigned int num_cells_x,
                                                            unsigned int num_cells_y)
{
    if (region.isEmpty())
        return nullptr;

    Grid2D grid;
    std::string error;

    if (!grid.create(region, grid2d::GridResolution().setCellCount(num_cells_x, num_cells_y), "wgs84", true, &error))
    {
        logerr << "could not create the target overview grid: " << error;
        return nullptr;
    }

    const bool gaps = isGapTable(definition);

    for (unsigned int row = 0; row < buffer.size(); ++row)
    {
        auto positions = rowPositions(definition, buffer, row);
        if (!positions.valid())
            continue;

        const double v = rowIsOk(definition, buffer, row) ? 1.0 : 0.0;

        if (gaps && positions.reference.has_value())
        {
            grid.addLine(positions.event->second,
                         positions.event->first,
                         positions.reference->second,
                         positions.reference->first, v);
        }
        else
        {
            const auto& pos = positions.reference.has_value() ? positions.reference.value()
                                                              : positions.event.value();
            grid.addValue(pos.second, pos.first, v);
        }
    }

    auto layer = grid.createLayer("Passed", FeatureDefinitionBinaryGrid::CellValueType);

    return std::make_unique<ViewPointGenFeatureGrid>(*layer, FeatureDefinitionBinaryGrid::renderSettings());
}

/**
 * The grid comes first in the annotations, so the points and lines are drawn on top of it.
 */
nlohmann::json createOverviewViewPoint(const ReportTableDefinition& definition,
                                       const Buffer& buffer,
                                       const OverviewFigureSettings& settings)
{
    nlohmann::json view_point = nlohmann::json::object();

    const QRectF region = overviewRegion(definition, buffer, settings.min_region_size);

    if (!region.isEmpty())
    {
        view_point[ ViewPointKeys::VP_POS_LAT_KEY     ] = region.center().y();
        view_point[ ViewPointKeys::VP_POS_LON_KEY     ] = region.center().x();
        view_point[ ViewPointKeys::VP_POS_WIN_LAT_KEY ] = region.height();
        view_point[ ViewPointKeys::VP_POS_WIN_LON_KEY ] = region.width();
    }

    auto& annotations = addRootAnnotation(view_point, settings.annotation_name);

    auto grid = createOverviewGrid(definition, buffer, region, settings.num_cells_x, settings.num_cells_y);

    if (grid)
    {
        const std::string description = "Passed";

        PlotMetadata metadata;
        metadata.plot_group_   = settings.annotation_name;
        metadata.title_        = settings.title;
        metadata.subtitle_     = description;
        metadata.x_axis_label_ = "Longitude (deg)";
        metadata.y_axis_label_ = "Latitude (deg)";

        grid->setName(description);
        grid->setPlotMetadata(metadata);
        grid->writeBinaryIfPossible(true);

        auto feature_annotation = std::make_unique<ViewPointGenAnnotation>(description);
        feature_annotation->addFeature(std::move(grid));

        ViewPointGenAnnotation grid_annotation(settings.title);
        grid_annotation.addAnnotation(std::move(feature_annotation));

        nlohmann::json grid_annotation_json;
        grid_annotation.toJSON(grid_annotation_json);

        annotations.push_back(grid_annotation_json);

        view_point[ ViewPointGenFeatureGrid::ViewPointFieldNameBackground ] = settings.background.toJSON();
    }

    //the row annotations are looked up by their type field, so they are built in an array of their own
    nlohmann::json row_annotations = nlohmann::json::array();
    createOverviewAnnotations(row_annotations, definition, buffer);

    for (auto& a : row_annotations)
        annotations.push_back(std::move(a));

    return view_point;
}

}

/************************************************************************************************
 * EvaluationAnnotations
 ************************************************************************************************/

namespace EvaluationAnnotations
{

namespace
{
    const std::string AnnotationArrayTypeField = "eval_annotation_array_type";

    const std::string NameOk        = "OK";
    const std::string NameError     = "Errors";
    const std::string NameHighlight = "Selected";

    struct Style
    {
        QColor                            color;
        ViewPointGenFeaturePoints::Symbol point_symbol;
        int                               point_size;
        int                               line_width;
    };

    Style styleFor(ArrayType type, bool overview)
    {
        Style s;

        if (type == ArrayType::TypeHighlight)
        {
            s.color        = Single::AnnotationColorHighlight;
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::Border;
            s.point_size   = Single::AnnotationPointSizeHighlight;
            s.line_width   = Single::AnnotationLineWidthHighlight;
        }
        else if (type == ArrayType::TypeError)
        {
            s.color        = Single::AnnotationColorError;
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::BorderThick;
            s.point_size   = Single::AnnotationPointSizeError;
            s.line_width   = Single::AnnotationLineWidthError;
        }
        else
        {
            s.color        = Single::AnnotationColorOk;
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::Border;
            s.point_size   = Single::AnnotationPointSizeOk;
            s.line_width   = Single::AnnotationLineWidthOk;
        }

        if (overview)
        {
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::Circle;
            s.point_size   = Single::AnnotationPointSizeOverview;
        }

        return s;
    }
}

/**
 */
const std::string& typeName(ArrayType type)
{
    if (type == ArrayType::TypeHighlight)
        return NameHighlight;
    if (type == ArrayType::TypeError)
        return NameError;

    return NameOk;
}

/**
 * Inserts the annotation of the given type in render order, or returns the existing one.
 */
nlohmann::json& getOrCreate(nlohmann::json& annotations_json,
                            ArrayType type,
                            bool overview)
{
    traced_assert(annotations_json.is_array());

    const std::string& anno_name = typeName(type);
    const std::string& field_name = ViewPointGenAnnotation::AnnotationFieldName;

    //find insertion index
    auto comp = [ ] (const nlohmann::json& j, ArrayType t)
    {
        int anno_type = j[ AnnotationArrayTypeField ];
        return anno_type < (int)t;
    };

    auto it = std::lower_bound(annotations_json.begin(), annotations_json.end(), type, comp);

    unsigned int insert_idx;

    if (it == annotations_json.end())
    {
        insert_idx = annotations_json.size();
    }
    else
    {
        insert_idx = it - annotations_json.begin();

        int anno_type = (*it)[ AnnotationArrayTypeField ];
        if (anno_type == (int)type)
        {
            auto& j = annotations_json.at(insert_idx);
            traced_assert(j.at(field_name) == anno_name);
            return j;
        }
    }

    auto style = styleFor(type, overview);

    annotations_json.insert(annotations_json.begin() + insert_idx, nlohmann::json::object());
    traced_assert(insert_idx < annotations_json.size());

    nlohmann::json& annotation_json = annotations_json.at(insert_idx);

    ViewPointGenAnnotation annotation(anno_name);
    annotation.setSymbolColor(style.color);

    //ATTENTION: !ORDER IMPORTANT!

    // lines
    std::unique_ptr<ViewPointGenFeatureLines> feature_lines;
    feature_lines.reset(new ViewPointGenFeatureLines(style.line_width,
                                                     ViewPointGenFeatureLineString::LineStyle::Solid,
                                                     {}, {}, false));
    feature_lines->setColor(style.color);

    annotation.addFeature(std::move(feature_lines));

    // symbols
    std::unique_ptr<ViewPointGenFeaturePoints> feature_points;
    feature_points.reset(new ViewPointGenFeaturePoints(style.point_symbol, style.point_size, {}, {}, false));
    feature_points->setColor(style.color);

    annotation.addFeature(std::move(feature_points));

    annotation.toJSON(annotation_json);

    //add annotation type for finding the annotation again later on
    annotation_json[ AnnotationArrayTypeField ] = (int)type;

    traced_assert(annotation_json.at(field_name) == anno_name);

    return annotation_json;
}

/**
 */
nlohmann::json& pointCoordinates(nlohmann::json& annotations_json,
                                 ArrayType type,
                                 bool overview)
{
    nlohmann::json& annotation = getOrCreate(annotations_json, type, overview);

    auto& feat_json = ViewPointGenAnnotation::getFeatureJSON(annotation, 1);

    return ViewPointGenFeaturePointGeometry::getCoordinatesJSON(feat_json);
}

/**
 */
nlohmann::json& lineCoordinates(nlohmann::json& annotations_json,
                                ArrayType type,
                                bool overview)
{
    nlohmann::json& annotation = getOrCreate(annotations_json, type, overview);

    auto& feat_json = ViewPointGenAnnotation::getFeatureJSON(annotation, 0);

    return ViewPointGenFeaturePointGeometry::getCoordinatesJSON(feat_json);
}

/**
 */
void addPosition(nlohmann::json& annotations_json,
                 double latitude,
                 double longitude,
                 ArrayType type,
                 bool overview)
{
    auto& coords = pointCoordinates(annotations_json, type, overview);
    coords.push_back(std::vector<double>({ latitude, longitude, 0 }));
}

/**
 */
void addLine(nlohmann::json& annotations_json,
             double latitude0,
             double longitude0,
             double latitude1,
             double longitude1,
             ArrayType type,
             bool overview)
{
    auto& coords = lineCoordinates(annotations_json, type, overview);
    coords.push_back(std::vector<double>({ latitude0, longitude0, 0 }));
    coords.push_back(std::vector<double>({ latitude1, longitude1, 0 }));
}

}
