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

#include "eval/results/base/single.h"
#include "eval/results/base/result_t.h"
#include "eval/results/base/joined.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttext.h"

#include "eval/results/report/evalsectionid.h"

#include "eval/requirement/base/base.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "sectorlayer.h"

#include "viewpoint.h"
#include "viewpointgenerator.h"

using namespace std;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

const std::string Single::TRDetailsTableName         = "Target Report Details";
const std::string Single::TRDetailsOverviewTableName = "Target Overview";
const std::string Single::TargetOverviewID           = "Target Errors Overview";

const int Single::AnnotationPointSizeOverview  = 8;
const int Single::AnnotationPointSizeHighlight = 12;
const int Single::AnnotationPointSizeError     = 16;
const int Single::AnnotationPointSizeOk        = 10;

const int Single::AnnotationLineWidthHighlight = 4;
const int Single::AnnotationLineWidthError     = 4;
const int Single::AnnotationLineWidthOk        = 2;

const QColor Single::AnnotationColorHighlight = Qt::yellow;
const QColor Single::AnnotationColorError     = QColor("#FF6666");
const QColor Single::AnnotationColorOk        = QColor("#66FF66");

const std::string Single::ContentPropertyUTN  = "utn";

/**
*/
Single::Single(const std::string& type, 
               const std::string& result_id,
               std::shared_ptr<EvaluationRequirement::Base> requirement,
               const SectorLayer& sector_layer,
               unsigned int utn,
               const EvaluationTargetData* target,
               EvaluationCalculator& calculator,
               const EvaluationDetails& details)
:   Base    (type, result_id, requirement, sector_layer, calculator)
,   utn_    (utn   )
,   target_ (target)
,   details_(details)
{
    annotation_type_names_[AnnotationArrayType::TypeHighlight] = "Selected";
    annotation_type_names_[AnnotationArrayType::TypeError    ] = "Errors";
    annotation_type_names_[AnnotationArrayType::TypeOk       ] = "OK";
}

/**
*/
Single::~Single() {}

/**
*/
unsigned int Single::utn() const
{
    return utn_;
}

/**
*/
const EvaluationTargetData* Single::target() const
{
    traced_assert(target_);
    return target_;
}

/**
*/
void Single::setInterestFactor(double factor, bool reset_in_target)
{
    interest_factor_ = factor;

    traced_assert(target_);

    Evaluation::RequirementSumResultID id;
    id.fromResult(*this);

    target_->addInterestFactor(id, factor, reset_in_target);
}

/**
*/
void Single::updateUseFromTarget()
{
    use_ = (resultUsable() 
            && target_->use() 
            && !target_->excludedRequirements().count(requirement_->name()));
}

/**
*/
void Single::updateResult()
{
    Base::updateResult(computeResult());

    updateUseFromTarget();
}

/**
*/
boost::optional<double> Single::computeResult() const
{
    return computeResult_impl();
}

/**
*/
std::string Single::getTargetSectionID()
{
    return EvalSectionID::targetID(utn_);
}

/**
*/
std::string Single::getTargetRequirementSectionID ()
{
    return EvalSectionID::targetResultID(utn_, *this);
}

/**
*/
std::string Single::sumSectionName() const
{
    if (calculator_.settings().report_split_results_by_mops_)
    {
        string tmp = target()->mopsVersionStr();

        if (!tmp.size())
            tmp = "Unknown";

        tmp = "MOPS " + tmp;

        return tmp + " " + EvalSectionID::SectionSum;
    }
    else if (calculator_.settings().report_split_results_by_aconly_ms_)
    {
        string tmp = "Primary";

        if (target()->isModeS())
            tmp = "Mode S";
        else if (target()->isModeACOnly())
            tmp = "Mode A/C";
        else
            traced_assert(target()->isPrimaryOnly());

        return tmp + " " + EvalSectionID::SectionSum;
    }

    //default sum section name
    return Base::sumSectionName();
}

/**
*/
std::string Single::getRequirementSectionID () const // TODO hack
{
    return EvalSectionID::requirementResultSumID(*this);
}

/**
*/
std::string Single::getRequirementAnnotationID_impl() const
{
    return (requirement_->name() + ":UTN" + std::to_string(utn_));
}

/**
*/
bool Single::hasStoredDetails() const
{
    return details_.has_value();
}

/**
*/
size_t Single::numStoredDetails() const
{
    if (!hasStoredDetails())
        return 0;

    return details_.value().size();
}

/**
*/
void Single::purgeStoredDetails()
{
    clearDetails();
}

/**
*/
Single::TemporaryDetails Single::temporaryDetails() const
{
    return TemporaryDetails(this);
}

/**
*/
Single::EvaluationDetails Single::recomputeDetails() const
{
    traced_assert(requirement_);
    traced_assert(calculator_.data().hasTargetData(utn_));

    logdbg << "recomputing target details for requirement '" << requirement_->name() << "' UTN " << utn_ << "...";

    const auto& data = calculator_.data().targetData(utn_);

    auto result = requirement_->evaluate(data, requirement_, sector_layer_);
    traced_assert(result);

    logdbg << "target details recomputed!";

    return result->getDetails();
}

/**
*/
const EvaluationDetail& Single::getDetail(const EvaluationDetails& details,
                                          const DetailIndex& index) const
{
    const auto& d = details.at(index[ 0 ]);

    if (index[ 1 ] < 0)
        return d;

    return d.details().at(index[ 1 ]);
}

/**
*/
const Single::EvaluationDetails& Single::getDetails() const
{
    traced_assert(hasStoredDetails());

    return details_.value();
}

/**
*/
bool Single::detailIndexValid(const DetailIndex& index) const
{
    if (index[ 0 ] < 0 || (details_.has_value() && index[ 0 ] >= (int)details_.value().size()))
        return false;
    if (index[ 1 ] >= 0 && (details_.has_value() && index[ 1 ] >= (int)details_.value()[ index[ 0 ] ].numDetails()))
        return false;

    return true;
}

/**
*/
void Single::clearDetails()
{
    details_.reset();
}

/**
*/
void Single::addToReport(std::shared_ptr<ResultReport::Report> report)
{
    logdbg << "start" <<  requirement_->name();

    // add target to requirements->group->req
    addTargetToOverviewTable(report);

    // add requirement results to targets->utn->requirements->group->req
    addTargetDetailsToReport(report);

    // @TODO add requirement description, methods
}

/**
*/
void Single::addTargetToOverviewTable(shared_ptr<ResultReport::Report> report)
{
    auto& tgt_overview_section = getRequirementSection(report);

    addTargetToOverviewTable(tgt_overview_section, Joined::SectorTargetsTableName);

    if (calculator_.settings().report_split_results_by_mops_ || 
        calculator_.settings().report_split_results_by_aconly_ms_) // add to general sum table
    {
        auto& sum_section = report->getSection(getRequirementSumSectionID());

        addTargetToOverviewTable(sum_section, Joined::SectorTargetsTableName);
    }
}

/**
*/
void Single::addTargetToOverviewTable(ResultReport::Section& section, 
                                      const std::string& table_name)
{
    if (!section.hasTable(table_name))
    {
        unsigned int sort_column;
        auto headers = targetTableHeaders(&sort_column);

        auto sort_order = targetTableSortOrder();

        auto& table = section.addTable(table_name, headers.size(), headers, true, sort_column, sort_order);

        Joined::setJoinedContentProperties(table, Evaluation::RequirementResultID(sector_layer_.name(),
                                                                                  requirement_->groupName(),
                                                                                  requirement_->name()));
    }

    auto& target_table = section.getTable(table_name);

    auto values = targetTableValues();
    traced_assert(values.size() == target_table.numColumns());

    std::string link = getTargetRequirementSectionID();
    std::string fig  = hasIssues() ? TargetOverviewID : "";

    bool is_active = use() && resultUsable();
    target_table.addRow(values, ResultReport::SectionContentViewable(), link, fig, QVariant(), is_active ? 0u : ResultReport::CellStyleInactive);
}

/**
*/
void Single::addTargetDetailsToReport(std::shared_ptr<ResultReport::Report> report)
{
    auto& utn_section     = report->getSection(getTargetSectionID());
    auto& utn_req_section = report->getSection(getTargetRequirementSectionID());

    utn_section.perTargetSection(true); // mark utn section per target

    

    //generate details overview table
    if (!utn_req_section.hasTable(TRDetailsOverviewTableName))
        utn_req_section.addTable(TRDetailsOverviewTableName, 3, {"Name", "Comment", "Value"}, false);

    auto& utn_req_table = utn_req_section.getTable(TRDetailsOverviewTableName);

    //add common infos
    auto common_infos = targetInfosCommon();

    for (const auto& info : common_infos)
        utn_req_table.addRow({ info.info_name, info.info_comment, info.info_value });

    //add custom infos
    auto infos = targetInfos();

    for (const auto& info : infos)
        utn_req_table.addRow({ info.info_name, info.info_comment, info.info_value });

    //add condition result
    bool failed;
    auto infos_condition = targetConditionInfos(failed);

    for (const auto& info : infos_condition)
        utn_req_table.addRow({ info.info_name, info.info_comment, info.info_value });

    if (failed)
    {
        // mark utn section as with issue
        utn_section.perTargetWithIssues(true); 
        utn_req_section.perTargetWithIssues(true);
    }

    //generate overview figure?
    if (hasIssues())
    {
        auto& fig = utn_req_section.addFigure(TargetOverviewID, ResultReport::SectionContentViewable().setCaption("Target Errors Overview"));

        //setup on-demand and add result info
        fig.setOnDemand();

        Single::setSingleContentProperties(fig, Evaluation::RequirementResultID(sector_layer_.name(),
                                                                                requirement_->groupName(),
                                                                                requirement_->name()), utn_);
    }
    else
    {
        utn_req_section.addText("target_errors_overview_no_figure");
        utn_req_section.getText("target_errors_overview_no_figure").
            addText("No target errors found, therefore no figure was generated.");
    }

    //generate details table
    generateDetailsTable(utn_req_section);
}

/**
*/
void Single::generateDetailsTable(ResultReport::Section& utn_req_section)
{
    //init table if needed
    if (!utn_req_section.hasTable(TRDetailsTableName))
    {
        auto headers = detailHeaders();

        auto& table = utn_req_section.addTable(TRDetailsTableName, headers.size(), headers);

        //setup on-demand and add result info
        table.setOnDemand();

        Single::setSingleContentProperties(table, Evaluation::RequirementResultID(sector_layer_.name(),
                                                                                  requirement_->groupName(),
                                                                                  requirement_->name()), utn_);
    }
}

/**
*/
bool Single::addDetailsToTable(ResultReport::SectionContentTable& table)
{
    //create details on demand
    auto temp_details = temporaryDetails();
    
    //detail => table row functor
    auto func = [ & ] (const EvaluationDetail& detail, 
                       const EvaluationDetail* parent_detail, 
                       int didx0, 
                       int didx1,
                       int evt_pos_idx, 
                       int evt_ref_pos_idx)
    {
        auto values = detailValues(detail, parent_detail);

        traced_assert(values.size() == table.numColumns());

        table.addRow(values, ResultReport::SectionContentViewable().setOnDemand(), "", "", QPoint(didx0, didx1));
    };

    //iterate over temporary details
    iterateDetails(func);

    return true;
}

/**
*/
bool Single::addOverviewToFigure(ResultReport::SectionContentFigure& figure)
{
    auto viewable = viewableOverviewData();

    auto viewable_func = [viewable]() { return viewable; };
    figure.setViewableFunc(viewable_func);

    return true;
}

/**
*/
bool Single::addHighlightToViewable(ResultReport::SectionContentViewable& viewable, const QVariant& annotation)
{
    //obtain detail key from annotation
    auto detail_key = detailIndex(annotation);
    if (!detail_key.has_value())
        return false;

    //generate temporary details
    auto temp_details = temporaryDetails();

    //create highlight viewable for detail
    auto v = createViewable(AnnotationOptions().highlight(detail_key.value()));
    if (!v)
        return false;

    //set callback
    nlohmann::json::object_t j = *v;
    v.reset();
    viewable.setCallback(j);

    return true;
}

/**
*/
std::vector<std::string> Single::targetTableHeadersCommon() const
{
    return { "UTN", "Begin", "End", "ACIDs", "ACADs", "M3/A" }; // , "MC Min", "MC Max"
}

/**
*/
std::vector<std::string> Single::targetTableHeadersOptional() const
{
    return {};
}

/**
*/
std::vector<std::string> Single::targetTableHeaders(unsigned int* sort_column) const
{
    auto headers          = targetTableHeadersCommon();
    auto headers_custom   = targetTableHeadersCustom();
    auto headers_optional = targetTableHeadersOptional();

    size_t result_value_idx = headers.size() + headers_custom.size();

    //add custom headers
    headers.insert(headers.end(), headers_custom.begin(), headers_custom.end());

    //add result value name
    headers.push_back(requirement_->getConditionResultNameShort(false));

    //add optional headers
    headers.insert(headers.end(), headers_optional.begin(), headers_optional.end());

    //set sort column
    if (sort_column)
    {
        std::string sort_column_custom_name = targetTableCustomSortColumn();

        if (sort_column_custom_name.size())
        {
            int index = -1;
            for (size_t i = 0; i < headers.size(); ++i)
            {
                if (headers[i] == sort_column_custom_name)
                {
                    index = static_cast<int>(i);
                    break;
                }
            }
            // index is -1 if not found, otherwise contains the index
            if (index >= 0)
                *sort_column = (unsigned int)index;
            else
            {
                logerr << "undefined header '" << sort_column_custom_name;
                *sort_column = result_value_idx;
            }
        }
    }

    return headers;
}

/**
*/
Qt::SortOrder Single::targetTableSortOrder() const
{
    //per default infer sort order from requirement
    return requirement_->resultSortOrder();
}

/**
*/
nlohmann::json::array_t Single::targetTableValuesCommon() const
{
    return { utn_, 
             target_->timeBeginStr(), 
             target_->timeEndStr(),
             target_->acidsStr(), 
             target_->acadsStr(),
             target_->modeACodesStr() };
            //  target_->modeCMinStr(), 
            //  target_->modeCMaxStr() 
}

/**
*/
nlohmann::json::array_t Single::targetTableValuesOptional() const
{
    return {};
}

/**
*/
nlohmann::json::array_t Single::targetTableValues() const
{
    auto values          = targetTableValuesCommon();
    auto values_custom   = targetTableValuesCustom();
    auto values_optional = targetTableValuesOptional();

    //add custom values
    values.insert(values.end(), values_custom.begin(), values_custom.end());

    //add result value
    values.push_back(resultValue());

    //add optional values
    values.insert(values.end(), values_optional.begin(), values_optional.end());

    return values;
}

/**
*/
std::vector<Single::TargetInfo> Single::targetInfosCommon() const
{
    return { { "UTN"         , "Unique Target Number"            , utn_                     },
             { "Begin"       , "Begin time of target"            , target_->timeBeginStr()  },
             { "End"         , "End time of target"              , target_->timeEndStr()    },
             { "ACIDs"       , "Mode S aicraft identification(s)", target_->acidsStr()      },
             { "ACADs"       , "Mode S aircraft address(es)"     , target_->acadsStr()      },
             { "Mode 3/A"    , "Mode 3/A code(s)"                , target_->modeACodesStr() },
             { "Mode C Min"  , "Minimum Mode C code [ft]"        , target_->modeCMinStr()   },
             { "Mode C Max"  , "Maximum Mode C code [ft]"        , target_->modeCMaxStr()   },
             { "Use"         , "To be used in results"           , use_                     }};
}

/**
*/
std::vector<Single::TargetInfo> Single::targetConditionInfos(bool& failed) const
{
    std::vector<TargetInfo> infos;

    auto result_val = resultValue();

    infos.push_back({ requirement_->getConditionResultNameShort(true), 
                       requirement_->getConditionResultName(), 
                       result_val });

    infos.push_back({"Condition", "", requirement_->getConditionStr() });

    string result {"Unknown"};

    if (!result_val.is_null())
        result = conditionResultString();

    infos.push_back({"Condition Fulfilled", "", result});

    if (requirement_->mustHoldForAnyTarget().has_value())
        infos.emplace_back("Must hold for any target ", "", requirement_->mustHoldForAnyTarget().value());

    failed = (result == "Failed");

    return infos;
}

/**
*/
bool Single::hasReference (const ResultReport::SectionContentTable& table, 
                           const QVariant& annotation) const
{
    if (table.name() == Joined::SectorTargetsTableName && annotation.toUInt() == utn_)
        return true;
    else
        return false;
}

/**
*/
std::string Single::reference(const ResultReport::SectionContentTable& table, 
                              const QVariant& annotation) const
{
    traced_assert(hasReference(table, annotation));
    return EvalSectionID::createForTargetResult(utn_, *this);
}

/**
*/
std::vector<double> Single::getValues(const ValueSource<double>& source) const
{
    return EvaluationResultTemplates(this).getValues<double>(source);
}

/**
*/
std::vector<double> Single::getValues(int value_id) const
{
    return getValues(ValueSource<double>(value_id));
}

/**
*/
bool Single::hasViewableData (const ResultReport::SectionContentTable& table, 
                              const QVariant& annotation) const
{
    //check validity of annotation index
    if (table.name() == Joined::SectorTargetsTableName && annotation.toUInt() == utn_)
        return true;
    else if (table.name() == TRDetailsTableName && annotation.isValid() && detailIndex(annotation).has_value())
        return true;
    
    return false;
}

/**
*/
bool Single::viewableDataReady() const
{
    //viewable data for single results is always computed on the fly, so return false in any case
    return false;
}

/**
 * Creates viewable data for this target for the given section.
 * BEWARE: !will always recompute the details!
*/
std::shared_ptr<nlohmann::json::object_t> Single::viewableData(const ResultReport::SectionContentTable& table, 
                                                               const QVariant& annotation) const
{
    traced_assert(hasViewableData(table, annotation));

    auto temp_details = temporaryDetails();

    if (table.name() == Joined::SectorTargetsTableName)
    {
        //return target overview viewable
        return createViewable(AnnotationOptions().overview());
    }
    else if (table.name() == TRDetailsTableName && annotation.isValid())
    {
        //obtain detail key from annotation
        auto detail_key = detailIndex(annotation);
        traced_assert(detail_key.has_value());

        //return target detail viewable
        return createViewable(AnnotationOptions().highlight(detail_key.value()));
    }

    return nullptr;
}

/**
 * Creates viewable overview data for this target.
 * BEWARE: !will always recompute the details!
*/
std::shared_ptr<nlohmann::json::object_t> Single::viewableOverviewData() const
{
    auto temp_details = temporaryDetails();

    //create overview viewable
    auto viewable = createViewable(AnnotationOptions().overview());

    return std::shared_ptr<nlohmann::json::object_t>(viewable.release());
}

/**
 * Retrieves (and possibly creates) a json point coordinate array for the given annotation type.
*/
nlohmann::json& Single::annotationPointCoords(nlohmann::json& annotations_json, AnnotationArrayType type, bool overview) const
{
    nlohmann::json& annotation = getOrCreateAnnotation(annotations_json, type, overview);

    auto& feat_json = ViewPointGenAnnotation::getFeatureJSON(annotation, 1);

    return ViewPointGenFeaturePointGeometry::getCoordinatesJSON(feat_json);
}

/**
 * Retrieves (and possibly creates) a json line coordinate array for the given annotation type.
*/
nlohmann::json& Single::annotationLineCoords(nlohmann::json& annotations_json, AnnotationArrayType type, bool overview) const
{
    nlohmann::json& annotation = getOrCreateAnnotation(annotations_json, type, overview);
    
    auto& feat_json = ViewPointGenAnnotation::getFeatureJSON(annotation, 0);

    return ViewPointGenFeaturePointGeometry::getCoordinatesJSON(feat_json);
}

/**
 * Retrieves (and possibly creates) a json annotation for the given annotation type.
*/
nlohmann::json& Single::getOrCreateAnnotation(nlohmann::json& annotations_json, 
                                              AnnotationArrayType type, 
                                              bool overview) const
{
    traced_assert(annotations_json.is_array());

    string anno_name = annotation_type_names_.at(type);

    logdbg << "anno_name '" << anno_name << "' overview " << overview;

    const std::string AnnotationArrayTypeField = "eval_annotation_array_type";
    const std::string FieldName                = ViewPointGenAnnotation::AnnotationFieldName;
    
    //creates a new annotation at the given array position
    auto insertAnnotation = [ & ] (const std::string& name,
                                   unsigned int position,
                                   const QColor& symbol_color,
                                   const ViewPointGenFeaturePoints::Symbol& point_symbol,
                                   const QColor& point_color,
                                   int point_size,
                                   const QColor& line_color,
                                   int line_width)
    {
        logdbg << "size " << annotations_json.size()
               << " creating '" << name << "' at pos " << position;

        for (unsigned int cnt=0; cnt < annotations_json.size(); ++cnt)
            logdbg << "start: index " << cnt <<" '" << annotations_json.at(cnt).at("name") << "'";

        annotations_json.insert(annotations_json.begin() + position, json::object()); // errors
        traced_assert(position < annotations_json.size());

        nlohmann::json& annotation_json = annotations_json.at(position);

        ViewPointGenAnnotation annotation(name);
        annotation.setSymbolColor(symbol_color);

        //ATTENTION: !ORDER IMPORTANT!

        // lines
        std::unique_ptr<ViewPointGenFeatureLines> feature_lines;
        feature_lines.reset(new ViewPointGenFeatureLines(line_width, ViewPointGenFeatureLineString::LineStyle::Solid, {}, {}, false));
        feature_lines->setColor(line_color);

        annotation.addFeature(std::move(feature_lines));

        // symbols
        std::unique_ptr<ViewPointGenFeaturePoints> feature_points;
        feature_points.reset(new ViewPointGenFeaturePoints(point_symbol, point_size, {}, {}, false));
        feature_points->setColor(point_color);

        annotation.addFeature(std::move(feature_points));

        //convert to json
        annotation.toJSON(annotation_json);

        //add annotation type for finding the annotation again later on
        annotation_json[ AnnotationArrayTypeField ] = (int)type;

        for (unsigned int cnt=0; cnt < annotations_json.size(); ++cnt)
            logdbg << "end: index " << cnt <<" '" << annotations_json.at(cnt).at("name") << "'";
    };

    struct Style
    {
        QColor                            color;
        ViewPointGenFeaturePoints::Symbol point_symbol;
        int                               point_size;
        int                               line_width;
    };

    //creates a style for the given annotation type
    auto getStyle = [ & ] (AnnotationArrayType type)
    {
        Style s;
        if (type == AnnotationArrayType::TypeHighlight)
        {
            s.color        = AnnotationColorHighlight;
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::Border;
            s.point_size   = AnnotationPointSizeHighlight;
            s.line_width   = AnnotationLineWidthHighlight;
        }
        else if (type == AnnotationArrayType::TypeError)
        {
            s.color        = AnnotationColorError;
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::BorderThick;
            s.point_size   = AnnotationPointSizeError;
            s.line_width   = AnnotationLineWidthError;
        }
        else if (type == AnnotationArrayType::TypeOk)
        {
            s.color        = AnnotationColorOk;
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::Border;
            s.point_size   = AnnotationPointSizeOk;
            s.line_width   = AnnotationLineWidthOk;
        }

        if (overview)
        {
            s.point_symbol = ViewPointGenFeaturePoints::Symbol::Circle;
            s.point_size   = AnnotationPointSizeOverview;
        }

        return s;
    };

    //find insertion index
    unsigned int insert_idx;

    auto comp = [ & ] (const nlohmann::json& j, AnnotationArrayType type) 
    { 
        int anno_type = j[ AnnotationArrayTypeField ];
        return anno_type < (int)type;
    };

    auto it = std::lower_bound(annotations_json.begin(), annotations_json.end(), type, comp);

    if (it == annotations_json.end()) 
    {
        // no element >= type -> insert at end
        insert_idx = annotations_json.size();
    }
    else // element >= type found
    {
        insert_idx = it - annotations_json.begin();

        // type already present? -> return annotation
        int anno_type = (*it)[ AnnotationArrayTypeField ];
        if (anno_type == (int)type)
        {
            auto& j = annotations_json.at(insert_idx);
            traced_assert(j.at(FieldName) == anno_name);
            return j;
        }

        // element > type found -> insert there
    }

    //insert new annotation of given type
    auto style = getStyle(type);

    insertAnnotation(anno_name, 
                     insert_idx,
                     style.color,
                     style.point_symbol,
                     style.color,
                     style.point_size,
                     style.color,
                     style.line_width);
    
    auto& j = annotations_json.at(insert_idx);
    traced_assert(j.at(FieldName) == anno_name);
    
    return j;
}

/**
 * Adds a new position to the position annotation.
*/
void Single::addAnnotationPos(nlohmann::json& annotations_json, 
                              const EvaluationDetail::Position& pos,
                              AnnotationArrayType type) const
{
    auto& coords = annotationPointCoords(annotations_json, type);
    coords.push_back(pos.asVector());
}

/**
 * Adds a new line to the line annotation.
*/
void Single::addAnnotationLine(nlohmann::json& annotations_json,
                               const EvaluationDetail::Position& pos0,
                               const EvaluationDetail::Position& pos1,
                               AnnotationArrayType type) const
{
    auto& coords = annotationLineCoords(annotations_json, type);
    coords.push_back(pos0.asVector());
    coords.push_back(pos1.asVector());
}

/**
*/
void Single::addAnnotationDistance(nlohmann::json& annotations_json,
                                   const EvaluationDetail& detail,
                                   AnnotationArrayType type,
                                   bool add_line,
                                   bool add_ref) const
{
    traced_assert(detail.numPositions() >= 1);

    addAnnotationPos(annotations_json, detail.position(0), type);

    if (add_ref  && detail.numPositions() >= 2) addAnnotationPos(annotations_json, detail.lastPos(), type);
    if (add_line && detail.numPositions() >= 2) addAnnotationLine(annotations_json, detail.position(0), detail.lastPos(), type);
}

/**
 * Returns a detail key given an annotation id.
 * Default behaviour will return a first level detail index.
*/
boost::optional<Base::DetailIndex> Single::detailIndex(const QVariant& annotation) const
{
    if (!annotation.isValid())
        return {};

    QPoint index = annotation.toPoint();
    if (index.isNull())
        return {};

    DetailIndex dindex = { index.x(), index.y() };
    if (!detailIndexValid(dindex))
        return {};

    return dindex;
}

/**
 * Iterates over all details for which skip_func does not return true, and applies the given functor func.
 * Default behaviour will iterate over all first level details.
*/
void Single::iterateDetails(const DetailFunc& func,
                            const DetailSkipFunc& skip_func) const
{
    traced_assert(details_.has_value());

    iterateDetails(details_.value(), func, skip_func);
}

/**
*/
void Single::iterateDetails(const EvaluationDetails& details,
                            const DetailFunc& func,
                            const DetailSkipFunc& skip_func) const
{
    traced_assert(func);

    auto nesting_mode = detailNestingMode();

    const auto& details0 = details;

    int evt_pos_idx     = eventPositionIndex();
    int evt_ref_pos_idx = eventRefPositionIndex();

    if (nesting_mode == DetailNestingMode::Vector)
    {
        for (size_t i = 0; i < details0.size(); ++i)
            if (!skip_func || !skip_func(details0[ i ]))
                func(details0[ i ], nullptr, (int)i, -1, evt_pos_idx, evt_ref_pos_idx);
    }
    else if (nesting_mode == DetailNestingMode::Nested)
    {
        for (size_t i = 0; i < details0.size(); ++i)
        {
            if (!details0[ i ].hasDetails())
                continue;

            const auto& details1 = details0[ i ].details();

            for (size_t j = 0; j < details1.size(); ++j)
                if (!skip_func || !skip_func(details1[ j ]))
                    func(details1[ j ], &details0[ i ], (int)i, (int)j, evt_pos_idx, evt_ref_pos_idx);
        }
    }
    else if (nesting_mode == DetailNestingMode::SingleNested)
    {
        traced_assert(details0.size() == 1);

        if (details0[ 0 ].hasDetails())
        {
            const auto& details1 = details0[ 0 ].details();

            for (size_t i = 0; i < details1.size(); ++i)
                if (!skip_func || !skip_func(details1[ i ]))
                    func(details1[ i ], &details0[ 0 ], 0, (int)i, evt_pos_idx, evt_ref_pos_idx);
        }
    }
}

/**
*/
std::unique_ptr<nlohmann::json::object_t> Single::createBaseViewable() const
{
    return calculator_.getViewableForEvaluation(utn_, req_grp_id_, result_id_);
}

/**
*/
Base::ViewableInfo Single::createViewableInfo(const AnnotationOptions& options) const
{
    traced_assert(options.valid());
    traced_assert(details_.has_value());

    Base::ViewableInfo info;
    info.viewable_type = options.viewable_type;

    const double BoundsEps = 1e-12;

    if (options.viewable_type == ViewableType::Overview)
    {
        //overview => iterate over failed details and compute bounds
        auto skip_func = [ this ] (const EvaluationDetail& detail)
        {
            return this->detailIsOk(detail);
        };

        QRectF bounds;

        auto func = [ & ] (const EvaluationDetail& detail, 
                           const EvaluationDetail* parent_detail, 
                           int didx0, 
                           int didx1,
                           int evt_pos_idx, 
                           int evt_ref_pos_idx)
        {
            traced_assert(detail.numPositions() >= 1);

            bounds |= detail.bounds(BoundsEps);
        };

        iterateDetails(func, skip_func);

        info.bounds = bounds;
    }
    else if (options.viewable_type == ViewableType::Highlight)
    {
        //highlight single detail
        const auto& d = getDetail(details_.value(), options.detail_index.value());

        traced_assert(d.numPositions() >= 1);

        info.bounds    = d.bounds(BoundsEps);
        info.timestamp = d.timestamp();
    }
    else
    {
        bool valid_viewable_type = false;
        traced_assert(valid_viewable_type);
    }

    return info;
}

/**
*/
void Single::createSumOverviewAnnotations(nlohmann::json& annotations_json,
                                          bool add_ok_details_to_overview) const
{
    traced_assert(details_.has_value());
    createTargetAnnotations(details_.value(), annotations_json, TargetAnnotationType::SumOverview, {}, add_ok_details_to_overview);
}

/**
*/
void Single::createTargetAnnotations(const EvaluationDetails& details,
                                     nlohmann::json& annotations_json,
                                     TargetAnnotationType type,
                                     const boost::optional<DetailIndex>& detail_index,
                                     bool add_ok_details_to_overview) const
{
    if (type == TargetAnnotationType::SumOverview)
    {
        //add sum overview annotations
        auto skip_func = [ & ] (const EvaluationDetail& detail)
        {
            //skip none?
            if (add_ok_details_to_overview)
                return false;

            //skip ok details
            return detailIsOk(detail);
        };

        auto func = [ & ] (const EvaluationDetail& detail, 
                           const EvaluationDetail* parent_detail, 
                           int didx0, 
                           int didx1,
                           int evt_pos_idx, 
                           int evt_ref_pos_idx)
        {
            addAnnotationForDetail(annotations_json, detail, TargetAnnotationType::SumOverview, detailIsOk(detail));
        };

        iterateDetails(details, func, skip_func);

        return;
    }
    else if (type == TargetAnnotationType::TargetOverview)
    {
        //add target overview annotations
        auto func = [ & ] (const EvaluationDetail& detail, 
                           const EvaluationDetail* parent_detail, 
                           int didx0, 
                           int didx1,
                           int evt_pos_idx, 
                           int evt_ref_pos_idx)
        {
            addAnnotationForDetail(annotations_json, detail, TargetAnnotationType::TargetOverview, detailIsOk(detail));
        };

        iterateDetails(details, func);
    }
    else if (type == TargetAnnotationType::Highlight)
    {
        traced_assert(detail_index.has_value());

        //add target overview annotations?
        if (addOverviewAnnotationsToDetail())
        {
            auto func = [ & ] (const EvaluationDetail& detail, 
                               const EvaluationDetail* parent_detail, 
                               int didx0, 
                               int didx1,
                               int evt_pos_idx, 
                               int evt_ref_pos_idx)
            {
                addAnnotationForDetail(annotations_json, detail, TargetAnnotationType::TargetOverview, detailIsOk(detail));
            };

            iterateDetails(details, func);
        }

        //add detail annotation
        const auto& d = getDetail(details, detail_index.value());
        addAnnotationForDetail(annotations_json, d, TargetAnnotationType::Highlight, detailIsOk(d));
    }
}

/**
*/
void Single::createAnnotations(nlohmann::json& annotations_json, 
                               const AnnotationOptions& options) const
{
    traced_assert(options.valid());
    traced_assert(details_.has_value());

    if (options.viewable_type == ViewableType::Overview)
    {
        //create target overview annotations
        createTargetAnnotations(details_.value(), annotations_json, TargetAnnotationType::TargetOverview);

        //add custom annotations to target overview
        addCustomAnnotations(annotations_json);
    }
    else if (options.viewable_type == ViewableType::Highlight)
    {
        createTargetAnnotations(details_.value(), annotations_json, TargetAnnotationType::Highlight, options.detail_index);
    }
}

/**
*/
void Single::setSingleContentProperties(ResultReport::SectionContent& content,
                                        const Evaluation::RequirementResultID& id,
                                        unsigned int utn)
{
    Base::setContentProperties(content, id);

    content.setJSONProperty(ContentPropertyUTN, utn);
}

/**
*/
boost::optional<std::pair<unsigned int, Evaluation::RequirementResultID>> Single::singleContentProperties(const ResultReport::SectionContent& content)
{
    if (!content.hasJSONProperty(ContentPropertyUTN))
        return boost::optional<std::pair<unsigned int, Evaluation::RequirementResultID>>();

    auto id = Base::contentProperties(content);
    if (!id.has_value())
        return boost::optional<std::pair<unsigned int, Evaluation::RequirementResultID>>();

    unsigned int utn;
    
    try
    {
        utn = content.jsonProperty(ContentPropertyUTN);
    }
    catch(...)
    {
        return boost::optional<std::pair<unsigned int, Evaluation::RequirementResultID>>();
    }

    return std::make_pair(utn, id.value());
}

}
