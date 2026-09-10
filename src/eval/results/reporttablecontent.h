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

#include "eval/results/base/single.h"

#include <string>
#include <vector>

#include <boost/optional.hpp>

#include <QRectF>

#include "json_fwd.hpp"

class Buffer;

/**
 * Report content built from the rows of a Report Table, without a result object.
 *
 * The "Target Report Details" table, the "Target Errors Overview" figure and the highlight
 * viewable of one row are rendered from the catalog and the rows alone. They work on a report
 * of the current session and on a reopened or locked report, see readme_dynamic_dbcontent.md
 * Section 4.5.
 */
namespace ReportTableContent
{
    /// names of the columns the renderer reads by itself
    extern const std::string ColumnDtPrev;
    extern const std::string ColumnTstLatitude;
    extern const std::string ColumnTstLongitude;
    extern const std::string ColumnRefLatitude;
    extern const std::string ColumnRefLongitude;
    extern const std::string ColumnRefLatitudeEnd;
    extern const std::string ColumnRefLongitudeEnd;
    extern const std::string ColumnCommentHeader;

    /// true if the rows of the table describe gaps, keyed by a reference sample
    bool isGapTable(const ReportTableDefinition& definition);

    /// headings of the "Target Report Details" table, the display names plus the comment
    std::vector<std::string> detailsTableHeaders(const ReportTableDefinition& definition);
    /// values of one "Target Report Details" row, in the order of detailsTableHeaders()
    nlohmann::json::array_t detailsTableValues(const ReportTableDefinition& definition,
                                               const Buffer& buffer,
                                               unsigned int row);

    /// true if the row passed its check
    bool rowIsOk(const ReportTableDefinition& definition,
                 const Buffer& buffer,
                 unsigned int row);
    /// comment text of a row, built from its flag columns and not stored
    std::string rowComment(const ReportTableDefinition& definition,
                           const Buffer& buffer,
                           unsigned int row);

    /// timestamp of a row
    boost::optional<boost::posix_time::ptime> rowTimestamp(const Buffer& buffer,
                                                           unsigned int row);

    /**
     * The two positions of a row: the position the row is assigned to, and the position it is
     * measured against. A gap row uses the first and the last reference sample inside the gap.
     */
    struct RowPositions
    {
        boost::optional<std::pair<double, double>> event;     // latitude, longitude
        boost::optional<std::pair<double, double>> reference; // latitude, longitude

        QRectF bounds(double eps) const;
        bool valid() const { return event.has_value(); }
    };

    RowPositions rowPositions(const ReportTableDefinition& definition,
                              const Buffer& buffer,
                              unsigned int row);

    /// bounds over the failed rows, over all rows when every row passed
    QRectF failedRowBounds(const ReportTableDefinition& definition,
                           const Buffer& buffer);

    /// annotations of the "Target Errors Overview", one entry per row
    void createOverviewAnnotations(nlohmann::json& annotations_json,
                                   const ReportTableDefinition& definition,
                                   const Buffer& buffer);
    /// annotations of one highlighted row, added on top of the overview
    void createHighlightAnnotations(nlohmann::json& annotations_json,
                                    const ReportTableDefinition& definition,
                                    const Buffer& buffer,
                                    unsigned int row);
}

/**
 * The annotation arrays of an evaluation viewable, shared by the result objects and the
 * report content built from the Report Tables.
 */
namespace EvaluationAnnotations
{
    typedef EvaluationRequirementResult::Single::AnnotationArrayType ArrayType;

    /// name of an annotation array, "OK", "Errors" or "Selected"
    const std::string& typeName(ArrayType type);

    /// creates the annotation of the given type if it does not exist yet
    nlohmann::json& getOrCreate(nlohmann::json& annotations_json,
                                ArrayType type,
                                bool overview);

    nlohmann::json& pointCoordinates(nlohmann::json& annotations_json,
                                     ArrayType type,
                                     bool overview = false);
    nlohmann::json& lineCoordinates(nlohmann::json& annotations_json,
                                    ArrayType type,
                                    bool overview = false);

    void addPosition(nlohmann::json& annotations_json,
                     double latitude,
                     double longitude,
                     ArrayType type,
                     bool overview = false);
    void addLine(nlohmann::json& annotations_json,
                 double latitude0,
                 double longitude0,
                 double latitude1,
                 double longitude1,
                 ArrayType type,
                 bool overview = false);
}
