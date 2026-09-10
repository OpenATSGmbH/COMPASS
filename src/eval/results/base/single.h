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

#include "eval/results/base/base.h"

#include "task/result/reporttable.h"

#include "view/gridview/grid2d.h"
#include "view/gridview/grid2dlayer.h"
#include "view/gridview/grid2drendersettings.h"

#include <vector>

#include <boost/optional.hpp>

#include <Eigen/Core>

#include <QColor>

#include "json_fwd.hpp"

const double OSGVIEW_POS_WINDOW_SCALE {1.8};

namespace ResultReport
{
    class SectionContentTable;
    class SectionContentFigure;
    class Report;

    struct SectionContentViewable;
}

class EvaluationTaskResult;

namespace EvaluationRequirementResult
{

class Joined;

template <typename T>
struct ValueSource;

/**
*/
class Single : public Base
{
public:
    /**
    */
    class TemporaryDetails
    {
    public:
        TemporaryDetails() = default;
        TemporaryDetails(TemporaryDetails&& other) 
        :   single_ (other.single_ )
        ,   created_(other.created_)
        {
            other.single_  = nullptr;
            other.created_ = false;
        }
        TemporaryDetails(const Single* single) 
        :   single_(single) 
        { 
            traced_assert(single_); 

            //load details from the report table if not available
            if (!single_->details_.has_value())
            {
                single_->details_ = single_->loadDetailsFromReportTable();
                created_ = true;
            }
        }
        ~TemporaryDetails()
        { 
            //purge temporary details if they were created by this class
            if (single_ && created_)
                single_->details_.reset();
        }

        TemporaryDetails& operator=(TemporaryDetails&& other)
        {
            single_  = other.single_;
            created_ = other.created_;

            other.single_  = nullptr;
            other.created_ = false;

            return *this;
        }

    private:
        const Single* single_  = nullptr;
        bool          created_ = false;
    };

    /**
    */
    struct LayerDefinition
    {
        grid2d::ValueType    value_type;
        Grid2DRenderSettings render_settings;
    };

    enum class TargetAnnotationType
    {
        SumOverview = 0,
        TargetOverview,
        Highlight
    };
    
    //!be careful: reflects order in render hierarchy!
    enum class AnnotationArrayType
    {
        TypeOk = 0,
        TypeError,
        TypeHighlight
    };

    enum class DetailNestingMode
    {
        Vector,      // details without child details
        Nested,      // details with child details
        SingleNested // single detail child details
    };

    typedef Info TargetInfo;

    Single(const std::string& type, 
           const std::string& result_id,
           std::shared_ptr<EvaluationRequirement::Base> requirement, 
           const SectorLayer& sector_layer,
           unsigned int utn, 
           const EvaluationTargetData* target, 
           EvaluationCalculator& calculator,
           const EvaluationDetails& details);
    virtual ~Single();

    BaseType baseType() const override final { return BaseType::Single; }

    unsigned int utn() const;
    const EvaluationTargetData* target() const;

    void updateUseFromTarget();

    void setInterestFactor(double factor, bool reset_in_target = false);

    bool hasViewableData (const ResultReport::SectionContentTable& table, 
                          const QVariant& annotation) const override final;
    bool viewableDataReady() const override final;
    std::shared_ptr<nlohmann::json::object_t> viewableData(const ResultReport::SectionContentTable& table, 
                                                           const QVariant& annotation) const override final;
    
    void createSumOverviewAnnotations(nlohmann::json& annotations_json,
                                      bool add_ok_details_to_overview = true) const;

    bool hasReference (const ResultReport::SectionContentTable& table, 
                       const QVariant& annotation) const override final;
    std::string reference(const ResultReport::SectionContentTable& table, 
                          const QVariant& annotation) const override final;

    void addToReport (std::shared_ptr<ResultReport::Report> report) override final;

    bool hasStoredDetails() const;
    size_t numStoredDetails() const;
    void purgeStoredDetails();
    TemporaryDetails temporaryDetails() const;

    void iterateDetails(const DetailFunc& func,
                        const DetailSkipFunc& skip_func = DetailSkipFunc()) const override final;

    /*report table related*/
    enum class ReportTableKeyKind
    {
        TestReport = 0, // one row per test report, keyed by its record number
        ReferenceGap    // one row per gap, keyed by the first reference sample inside it
    };

    /// definition of the report table of this result's requirement and sector layer
    ReportTableDefinition reportTableDefinition() const;
    /// writes the rows of the stored details into the report table
    void addReportTableRows(ReportTableRows& rows) const;
    /// key of the report table the rows of this result went into, empty before they were written
    const std::string& reportTableKey() const { return report_table_key_; }

    /// sets a column from a detail value, leaves it null if the value is not set
    template <typename T>
    static void setReportTableValue(ReportTableRows& rows,
                                    const std::string& column,
                                    const EvaluationDetail& detail,
                                    EvaluationDetail::Key key)
    {
        auto value = detail.getValueAs<T>(key);
        if (value.has_value())
            rows.set<T>(column, value.value());
    }

    /// sets a detail value from a report table column, leaves it unset if the column is null
    template <typename T>
    static void setDetailValue(EvaluationDetail& detail,
                               EvaluationDetail::Key key,
                               const Buffer& buffer,
                               const std::string& column,
                               unsigned int row)
    {
        if (!buffer.has<T>(column) || buffer.get<T>(column).isNull(row))
            return;

        detail.setValue(key, QVariant(buffer.get<T>(column).get(row)));
    }

    std::vector<double> getValues(const ValueSource<double>& source) const;
    std::vector<double> getValues(int value_id) const;

    /// create empty joined result
    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) = 0;

    /**
     * Result information stored on an on-demand content of a single result.
     */
    struct ContentInfo
    {
        unsigned int                    utn = 0;
        Evaluation::RequirementResultID id;
        std::string                     report_table_key; // empty for a report written before the tables
    };

    static void setSingleContentProperties(ResultReport::SectionContent& content,
                                           const Evaluation::RequirementResultID& id,
                                           unsigned int utn,
                                           const std::string& report_table_key);
    static boost::optional<ContentInfo>
    singleContentProperties(const ResultReport::SectionContent& content);

    std::string sumSectionName() const override final;

    static const std::string TRDetailsTableName;
    static const std::string TRDetailsOverviewTableName;
    static const std::string TargetOverviewID;

    static const int AnnotationPointSizeOverview;
    static const int AnnotationPointSizeHighlight;
    static const int AnnotationPointSizeError;
    static const int AnnotationPointSizeOk;

    static const int AnnotationLineWidthHighlight;
    static const int AnnotationLineWidthError;
    static const int AnnotationLineWidthOk;

    static const QColor AnnotationColorHighlight;
    static const QColor AnnotationColorError;
    static const QColor AnnotationColorOk;

    static const std::string ContentPropertyUTN;
    static const std::string ContentPropertyReportTable;

protected:
    friend class EvaluationTaskResult; // for loading on-demand content

    std::string getTargetSectionID();
    std::string getTargetRequirementSectionID();

    /// compute result value
    virtual void updateResult();
    virtual boost::optional<double> computeResult() const;
    virtual boost::optional<double> computeResult_impl() const = 0;

    virtual std::string getRequirementSectionID () const override;
    virtual std::string getRequirementAnnotationID_impl() const override;

    /*report contents related*/

    virtual std::vector<std::string> targetTableHeadersCommon() const;
    virtual std::vector<std::string> targetTableHeadersOptional() const;
    std::vector<std::string> targetTableHeaders(unsigned int* sort_column = nullptr) const;
    virtual nlohmann::json::array_t targetTableValuesCommon() const;
    virtual nlohmann::json::array_t targetTableValuesOptional() const;
    nlohmann::json::array_t targetTableValues() const;

    virtual std::vector<TargetInfo> targetInfosCommon() const;
    std::vector<TargetInfo> targetConditionInfos(bool& failed) const;
    
    /// derive to obtain custom target table header strings
    virtual std::vector<std::string> targetTableHeadersCustom() const = 0;
    /// derive to obtain custom target table row values (size must match targetTableHeadersDerived())
    virtual nlohmann::json::array_t targetTableValuesCustom() const = 0;
    /// derive to obtain a custom sort column for the target table (!index relative to custom values!)
    virtual std::string targetTableCustomSortColumn() const { return ""; }
    /// derive to obtain a custom sort order for the target table
    virtual Qt::SortOrder targetTableSortOrder() const;
    /// derive to obtain items for the target details overview table
    virtual std::vector<TargetInfo> targetInfos() const = 0;
    virtual void addTargetToOverviewTable(std::shared_ptr<ResultReport::Report> report);
    virtual void addTargetToOverviewTable(ResultReport::Section& section, 
                                          const std::string& table_name);
    virtual void addTargetDetailsToReport(std::shared_ptr<ResultReport::Report> report);
    virtual void generateDetailsTable(ResultReport::Section& utn_req_section);

    /*details related*/
    const EvaluationDetails& getDetails() const;
    
    bool detailIndexValid(const DetailIndex& index) const; 

    /// detail nesting mode
    virtual DetailNestingMode detailNestingMode() const { return DetailNestingMode::Vector; }

    boost::optional<DetailIndex> detailIndex(const QVariant& annotation) const;

    /*report table related*/
    /// kind of record number the rows of this result family reference
    virtual ReportTableKeyKind reportTableKeyKind() const { return ReportTableKeyKind::TestReport; }
    /// derive to add the family columns to the table definition
    virtual void addReportTableColumns(ReportTableDefinition& def) const {}
    /// derive to fill the family columns of a row, the base fills the common columns
    virtual void fillReportTableRow(ReportTableRows& rows,
                                    const EvaluationDetail& detail,
                                    const EvaluationDetail* parent_detail,
                                    const EvaluationDetail* prev_detail) const {}
    /// derive to skip details which give no row
    virtual bool reportTableRowWanted(const EvaluationDetail& detail,
                                      const EvaluationDetail* parent_detail) const { return true; }
    /// derive for reference gap tables: begin and end of the gap of a detail, none to skip it
    virtual boost::optional<std::pair<boost::posix_time::ptime, boost::posix_time::ptime>>
        reportTableGapBounds(const EvaluationDetail& detail) const { return boost::none; }
    /// derive to fill the family values of a detail from a row, inverse of fillReportTableRow()
    virtual void fillDetailFromReportTableRow(EvaluationDetail& detail,
                                              const Buffer& buffer,
                                              unsigned int row,
                                              const EvaluationDetail* prev_detail) const {}
    /// derive for families with nested details to fill the period detail from a row of its period
    virtual void fillPeriodDetailFromReportTableRow(EvaluationDetail& period,
                                                    const Buffer& buffer,
                                                    unsigned int row) const {}
    /// column grouping the rows into periods, empty for families without nested details
    virtual std::string reportTablePeriodColumn() const { return ""; }

    /*viewable + annotation related*/
    std::shared_ptr<nlohmann::json::object_t> viewableOverviewData() const override final;
    std::unique_ptr<nlohmann::json::object_t> createBaseViewable() const override final;
    ViewableInfo createViewableInfo(const AnnotationOptions& options) const override final;
    void createAnnotations(nlohmann::json& annotations_json, 
                           const AnnotationOptions& options) const override final;

    /// if yes the overview annotations are added if a detail is highlighted (default)
    virtual bool addOverviewAnnotationsToDetail() const { return true; }
    /// derive to implement detail validity checks
    virtual bool detailIsOk(const EvaluationDetail& detail) const = 0;
    /// derive to implement detail annotation creation 
    virtual void addAnnotationForDetail(nlohmann::json& annotations_json, 
                                        const EvaluationDetail& detail, 
                                        TargetAnnotationType type,
                                        bool is_ok) const = 0;
    /// returns the index of the event position (where the event occurs)
    /// first position is default
    virtual int eventPositionIndex() const { return 0; }
    /// returns the index of the event reference position (the position the event is measured against, e.g. a ref trajectory position, the interval begin, etc.)
    /// -1 means the last position (default)
    virtual int eventRefPositionIndex() const { return -1; }

    void createTargetAnnotations(const EvaluationDetails& details,
                                 nlohmann::json& annotations_json,
                                 TargetAnnotationType type,
                                 const boost::optional<DetailIndex>& detail_index = boost::optional<DetailIndex>(),
                                 bool add_ok_details_to_overview = true) const;
    
    void addAnnotationPos(nlohmann::json& annotations_json,
                          const EvaluationDetail::Position& pos, 
                          AnnotationArrayType type) const;
    void addAnnotationLine(nlohmann::json& annotations_json,
                           const EvaluationDetail::Position& pos0, 
                           const EvaluationDetail::Position& pos1,
                           AnnotationArrayType type) const;
    void addAnnotationDistance(nlohmann::json& annotations_json,
                               const EvaluationDetail& detail,
                               AnnotationArrayType type,
                               bool add_line = true,
                               bool add_ref = true) const;

    /// creates if not existing
    nlohmann::json& annotationPointCoords(nlohmann::json& annotations_json, AnnotationArrayType type, bool overview = false) const;
    nlohmann::json& annotationLineCoords(nlohmann::json& annotations_json, AnnotationArrayType type, bool overview = false) const;
    nlohmann::json& getOrCreateAnnotation(nlohmann::json& annotations_json, AnnotationArrayType type, bool overview) const;

    unsigned int                utn_;    // used to generate result
    const EvaluationTargetData* target_; // used to generate result

    double interest_factor_ {0};

private:
    void iterateDetails(const EvaluationDetails& details,
                        const DetailFunc& func,
                        const DetailSkipFunc& skip_func = DetailSkipFunc()) const;

    const EvaluationDetail& getDetail(const EvaluationDetails& details,
                                      const DetailIndex& index) const;
    void clearDetails();

    EvaluationDetails loadDetailsFromReportTable() const;
    EvaluationDetails detailsFromReportTableRows(const ReportTableDefinition& definition,
                                                 const Buffer& buffer) const;

    mutable boost::optional<EvaluationDetails> details_;
    mutable std::string                        report_table_key_;

    bool show_overview_always_ = true;
};

}
