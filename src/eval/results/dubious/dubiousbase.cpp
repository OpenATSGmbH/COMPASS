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

#include "eval/results/dubious/dubiousbase.h"
#include "evaluationmanager.h"

namespace EvaluationRequirementResult
{

/************************************************************************************
 * DubiousBase
 ************************************************************************************/

/**
*/
DubiousBase::DubiousBase() = default;

/**
*/
DubiousBase::DubiousBase(unsigned int num_updates,
                         unsigned int num_pos_outside,
                         unsigned int num_pos_inside,
                         unsigned int num_pos_inside_dubious)
:   num_updates_           (num_updates)
,   num_pos_outside_       (num_pos_outside)
,   num_pos_inside_        (num_pos_inside)
,   num_pos_inside_dubious_(num_pos_inside_dubious)
{
}

/**
*/
unsigned int DubiousBase::numPosOutside() const
{
    return num_pos_outside_;
}

/**
*/
unsigned int DubiousBase::numPosInside() const
{
    return num_pos_inside_;
}

/**
*/
unsigned int DubiousBase::numPosInsideDubious() const
{
    return num_pos_inside_dubious_;
}

/**
*/
unsigned int DubiousBase::numUpdates() const
{
    return num_updates_;
}

/************************************************************************************
 * SingleDubiousBase::DetailData
 ************************************************************************************/

/**
*/
SingleDubiousBase::DetailData::DetailData(unsigned int utn_or_track_number, 
                                          boost::posix_time::ptime ts_begin)
:   utn_or_tracknum(utn_or_track_number)
,   tod_begin      (ts_begin           )
,   tod_end        (ts_begin           )
{
}

/**
*/
void SingleDubiousBase::DetailData::assignTo(EvaluationDetail& d) const
{
    d.setValue(DetailKey::UTNOrTrackNum, utn_or_tracknum)
     .setValue(DetailKey::FirstInside, first_inside)
     .setValue(DetailKey::TODBegin, tod_begin)
     .setValue(DetailKey::TODEnd, tod_end)
     .setValue(DetailKey::Duration, duration)
     .setValue(DetailKey::NumPosInside, num_pos_inside)
     .setValue(DetailKey::NumPosInsideDub, num_pos_inside_dubious)
     .setValue(DetailKey::HasModeAC, has_mode_ac)
     .setValue(DetailKey::HasModeS, has_mode_s)
     .setValue(DetailKey::LeftSector, left_sector)
     .setValue(DetailKey::IsDubious, is_dubious)
     .addPosition(pos_begin)
     .addPosition(pos_last)
     .setDetails(details);

    SingleDubiousBase::logComments(d, dubious_reasons);
}

/**
*/
unsigned int SingleDubiousBase::DetailData::numDubious() const
{
    unsigned int cnt = 0;

    for (auto& dd : details)
        if (dd.comments().hasComments(DetailCommentGroupDubious))
            ++cnt;

    return cnt;
}

/************************************************************************************
 * SingleDubiousBase
 ************************************************************************************/

const std::string SingleDubiousBase::DetailCommentGroupDubious = "CommentsDubious";

/**
*/
SingleDubiousBase::SingleDubiousBase(const std::string& result_type,
                                     const std::string& result_id, 
                                     std::shared_ptr<EvaluationRequirement::Base> requirement,
                                     const SectorLayer& sector_layer,
                                     unsigned int utn, 
                                     const EvaluationTargetData* target, 
                                     EvaluationCalculator& calculator,
                                     const EvaluationDetails& details,
                                     unsigned int num_updates,
                                     unsigned int num_pos_outside, 
                                     unsigned int num_pos_inside, 
                                     unsigned int num_pos_inside_dubious)
:   DubiousBase(num_updates, num_pos_outside, num_pos_inside, num_pos_inside_dubious)
,   SingleProbabilityBase(result_type, result_id, requirement, sector_layer, utn, target, calculator, details)
{
}

/**
*/
SingleDubiousBase::~SingleDubiousBase() = default;

/**
*/
std::string SingleDubiousBase::dubiousReasonsString(const EvaluationDetailComments& comments)
{
    if (!comments.hasComments(DetailCommentGroupDubious))
        return "OK";
    
    std::string str;

    auto cmts = comments.group(DetailCommentGroupDubious);

    for (const auto& c : cmts.value())
    {
        if (str.size())
            str += ", ";

        str += c.first;
        if (c.second.size())
            str += "(" + c.second + ")";
    }
    
    return str;
}

/**
*/
void SingleDubiousBase::logComment(EvaluationDetail& d, const std::string& id, const std::string& comment)
{
    d.comments().comment(DetailCommentGroupDubious, id, comment);
}

/**
*/
void SingleDubiousBase::logComments(EvaluationDetail& d, const EvaluationDetailComments::CommentGroup& group)
{
    d.comments().group(DetailCommentGroupDubious, group);
}

/**
*/
SingleDubiousBase::EvaluationDetails SingleDubiousBase::generateDetails(const std::vector<DetailData>& detail_data)
{
    if (detail_data.empty())
        return {};

    size_t n = detail_data.size();

    EvaluationDetails details(n);

    for (size_t i = 0; i < n; ++i)
        detail_data[ i ].assignTo(details[ i ]);

    return details;
}

/************************************************************************************
 * JoinedDubiousBase
 ************************************************************************************/

/**
*/
JoinedDubiousBase::JoinedDubiousBase(const std::string& result_type,
                                     const std::string& result_id, 
                                     std::shared_ptr<EvaluationRequirement::Base> requirement,
                                     const SectorLayer& sector_layer, 
                                     EvaluationCalculator& calculator)
:   DubiousBase()
,   JoinedProbabilityBase(result_type, result_id, requirement, sector_layer, calculator)
{
}

/**
 */
void SingleDubiousBase::addReportTableColumns(ReportTableDefinition& def) const
{
    def.addColumn("period_begin", PropertyDataType::TIMESTAMP, "Period Begin",
                  "Begin of the period the test report belongs to");
    def.addColumn("period_end", PropertyDataType::TIMESTAMP, "Period End",
                  "End of the period the test report belongs to");
    def.addColumn("is_dubious", PropertyDataType::BOOL, "Dubious",
                  "Test report assessed as dubious");
    def.addColumn("dubious_reasons", PropertyDataType::STRING, "Dubious Reasons",
                  "Reasons the test report is dubious");
    def.addColumn("period_dubious", PropertyDataType::BOOL, "Period Dubious",
                  "Period of the test report assessed as dubious");
    def.addColumn("first_inside", PropertyDataType::BOOL, "First Inside",
                  "First position of the period inside the sector layer");
    def.addColumn("left_sector", PropertyDataType::BOOL, "Left Sector",
                  "Period left the sector layer");
    def.addColumn("has_mode_ac", PropertyDataType::BOOL, "Has Mode A/C",
                  "Period has Mode A or Mode C data");
    def.addColumn("has_mode_s", PropertyDataType::BOOL, "Has Mode S",
                  "Period has Mode S data");
}

/**
 */
void SingleDubiousBase::fillReportTableRow(ReportTableRows& rows,
                                  const EvaluationDetail& detail,
                                  const EvaluationDetail* parent_detail,
                                  const EvaluationDetail* prev_detail) const
{
    // the period flags sit on the parent detail, the rows are its test reports
    const EvaluationDetail& period = parent_detail ? *parent_detail : detail;

    //the period detail carries no timestamp of its own, its bounds are detail values
    auto period_begin = period.getValueAs<boost::posix_time::ptime>(DetailKey::TODBegin);
    auto period_end   = period.getValueAs<boost::posix_time::ptime>(DetailKey::TODEnd);

    if (period_begin.has_value() && !period_begin->is_not_a_date_time())
        rows.set<boost::posix_time::ptime>("period_begin", period_begin.value());
    else if (!period.timestamp().is_not_a_date_time())
        rows.set<boost::posix_time::ptime>("period_begin", period.timestamp());

    if (period_end.has_value() && !period_end->is_not_a_date_time())
        rows.set<boost::posix_time::ptime>("period_end", period_end.value());

    // the dubious state of the row is the one of its test report, the period reasons are
    // copied onto every test report of a dubious period during the evaluation
    const bool row_dubious = detail.comments().hasComments(DetailCommentGroupDubious);

    rows.set<bool>("is_dubious", row_dubious);

    if (row_dubious)
        rows.set<std::string>("dubious_reasons", dubiousReasonsString(detail.comments()));

    setReportTableValue<bool>(rows, "period_dubious", period, DetailKey::IsDubious);
    setReportTableValue<bool>(rows, "first_inside"  , period, DetailKey::FirstInside);
    setReportTableValue<bool>(rows, "left_sector"   , period, DetailKey::LeftSector);
    setReportTableValue<bool>(rows, "has_mode_ac"   , period, DetailKey::HasModeAC);
    setReportTableValue<bool>(rows, "has_mode_s"    , period, DetailKey::HasModeS);
}

/**
 */
void SingleDubiousBase::fillDetailFromReportTableRow(EvaluationDetail& detail,
                                  const Buffer& buffer,
                                  unsigned int row,
                                  const EvaluationDetail* prev_detail) const
{
    //the comment group is what detailIsOk() reads, so the reasons go back into it
    if (buffer.has<std::string>("dubious_reasons") && !buffer.get<std::string>("dubious_reasons").isNull(row))
    {
        auto reasons = buffer.get<std::string>("dubious_reasons").get(row);

        if (!reasons.empty())
            logComment(detail, reasons, "");
    }
}

/**
 */
void SingleDubiousBase::fillPeriodDetailFromReportTableRow(EvaluationDetail& period,
                                  const Buffer& buffer,
                                  unsigned int row) const
{
    const std::string begin_column = "period_begin";
    const std::string end_column   = "period_end";

    boost::optional<boost::posix_time::ptime> begin;
    boost::optional<boost::posix_time::ptime> end;

    if (buffer.has<boost::posix_time::ptime>(begin_column) &&
        !buffer.get<boost::posix_time::ptime>(begin_column).isNull(row))
        begin = buffer.get<boost::posix_time::ptime>(begin_column).get(row);

    if (buffer.has<boost::posix_time::ptime>(end_column) &&
        !buffer.get<boost::posix_time::ptime>(end_column).isNull(row))
        end = buffer.get<boost::posix_time::ptime>(end_column).get(row);

    if (begin.has_value())
        period.setValue(DetailKey::TODBegin, begin.value());
    if (end.has_value())
        period.setValue(DetailKey::TODEnd, end.value());
    if (begin.has_value() && end.has_value())
        period.setValue(DetailKey::Duration, end.value() - begin.value());

    setDetailValue<bool>(period, DetailKey::IsDubious  , buffer, "period_dubious", row);
    setDetailValue<bool>(period, DetailKey::FirstInside, buffer, "first_inside"  , row);
    setDetailValue<bool>(period, DetailKey::LeftSector , buffer, "left_sector"   , row);
    setDetailValue<bool>(period, DetailKey::HasModeAC  , buffer, "has_mode_ac"   , row);
    setDetailValue<bool>(period, DetailKey::HasModeS   , buffer, "has_mode_s"    , row);
}

}
