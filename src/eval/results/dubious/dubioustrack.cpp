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

#include "eval/results/dubious/dubioustrack.h"

#include "eval/results/base/featuredefinitions.h"

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/dubious/dubioustrack.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "logger.h"
#include "util/stringconv.h"
#include "util/timeconv.h"
#include "viewpoint.h"

#include "traced_assert.h"
#include <algorithm>

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

/**********************************************************************************************
 * DubiousTrackBase
 **********************************************************************************************/

/**
*/
DubiousTrackBase::DubiousTrackBase() = default;

/**
*/
DubiousTrackBase::DubiousTrackBase(unsigned int num_tracks, 
                                   unsigned int num_tracks_dubious)
:   num_tracks_        (num_tracks)
,   num_tracks_dubious_(num_tracks_dubious)
{
}

/**
*/
unsigned int DubiousTrackBase::numTracks() const
{
    return num_tracks_;
}

/**
*/
unsigned int DubiousTrackBase::numTracksDubious() const
{
    return num_tracks_dubious_;
}

/**********************************************************************************************
 * SingleDubiousTrack
 **********************************************************************************************/

/**
*/
SingleDubiousTrack::SingleDubiousTrack(const std::string& result_id, 
                                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                                       const SectorLayer& sector_layer,
                                       unsigned int utn,
                                       const EvaluationTargetData* target,
                                       EvaluationCalculator& calculator,
                                       const EvaluationDetails& details,
                                       unsigned int num_updates,
                                       unsigned int num_pos_outside,
                                       unsigned int num_pos_inside,
                                       unsigned int num_pos_inside_dubious,
                                       unsigned int num_tracks,
                                       unsigned int num_tracks_dubious)
:   DubiousTrackBase(num_tracks, num_tracks_dubious)
,   SingleDubiousBase ("SingleDubiousTrack", result_id, requirement, sector_layer, utn, target, calculator, details,
                       num_updates, num_pos_outside, num_pos_inside, num_pos_inside_dubious)
{
    for (const auto& detail_it : details)
    {
        if (dubious_reasons_.size())
            dubious_reasons_ += "\n";

        auto dub_reasons_str = dubiousReasonsString(detail_it.comments());

        dubious_reasons_ += to_string(detail_it.getValue(DetailKey::UTNOrTrackNum).toUInt()) + ":" + dub_reasons_str;
    }

    updateResult();
}

/**
*/
SingleDubiousTrack::~SingleDubiousTrack() = default;

/**
*/
float SingleDubiousTrack::trackDurationAll() const
{
    return track_duration_all_;
}

/**
*/
float SingleDubiousTrack::trackDurationNondub() const
{
    return track_duration_nondub_;
}

/**
*/
float SingleDubiousTrack::trackDurationDubious() const
{
    return track_duration_dubious_;
}


/**
*/
std::shared_ptr<Joined> SingleDubiousTrack::createEmptyJoined(const std::string& result_id)
{
    return make_shared<JoinedDubiousTrack> (result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
EvaluationRequirement::DubiousTrack* SingleDubiousTrack::req ()
{
    EvaluationRequirement::DubiousTrack* req =
            dynamic_cast<EvaluationRequirement::DubiousTrack*>(requirement_.get());
    traced_assert(req);
    return req;
}

/**
*/
boost::optional<double> SingleDubiousTrack::computeResult_impl() const
{
    traced_assert(num_updates_ == num_pos_inside_ + num_pos_outside_);
    traced_assert(num_tracks_ >= num_tracks_dubious_);

    p_dubious_update_.reset();

    boost::optional<double> result;

    if (num_tracks_)
    {
        result = (double)num_tracks_dubious_ / (double)num_tracks_;

        for (const auto& detail_it : getDetails())
        {
            auto duration   = detail_it.getValue(DetailKey::Duration).toDouble();
            auto is_dubious = detail_it.getValue(DetailKey::IsDubious).toBool();

            track_duration_all_ += duration;

            if (is_dubious)
                track_duration_dubious_ += duration;
            else
                track_duration_nondub_ += duration;
        }
    }
    
    if (num_pos_inside_)
    {
        p_dubious_update_ = (double)num_pos_inside_dubious_ / (double)num_pos_inside_;
    }

    logdbg << "'" << requirement_->name() << "' " << target_->utn_
           << " has_p_dubious_update_ "   << p_dubious_update_.has_value()
           << " num_pos_inside_dubious_ " << num_pos_inside_dubious_
           << " num_pos_inside_ "         << num_pos_inside_
           << " p_dubious_update_ "       << (p_dubious_update_.has_value() ? p_dubious_update_.value() : 0);

    return result;
}

/**
*/
unsigned int SingleDubiousTrack::numIssues() const
{
    return num_tracks_dubious_;
}

/**
*/
std::vector<std::string> SingleDubiousTrack::targetTableHeadersCustom() const
{
    return { "#PosInside", "#DU", "PDU", "#T", "#DT", "Reasons" };
}

/**
*/
nlohmann::json::array_t SingleDubiousTrack::targetTableValuesCustom() const
{
    return { num_pos_inside_,                        // "#IU"
             num_pos_inside_dubious_,                // "#DU"
             resultValueOptional(p_dubious_update_), // "PDU"
             num_tracks_,                            // "#T"
             num_tracks_dubious_,                    // "#DT"
             dubious_reasons_ };                     // "Reasons"
}

/**
*/
std::vector<Single::TargetInfo> SingleDubiousTrack::targetInfos() const
{
    auto p_dubious_up_var          = SingleProbabilityBase::formatProbabilityOptional(p_dubious_update_);
    auto track_duration_var        = Utils::String::doubleToStringPrecision(track_duration_all_,2).c_str();
    auto track_duration_dub_var    = Utils::String::doubleToStringPrecision(track_duration_dubious_,2).c_str();
    auto track_duration_nondub_var = Utils::String::doubleToStringPrecision(track_duration_nondub_,2).c_str();
    auto dubious_t_avg_var         = num_tracks_dubious_ > 0 ? nlohmann::json(roundf(track_duration_dubious_/(float)num_tracks_dubious_ * 100.0) / 100.0) : nlohmann::json();

    nlohmann::json p_dubious_t_var, p_nondub_t_var;

    if (track_duration_all_)
    {
        p_dubious_t_var = roundf(track_duration_dubious_ / track_duration_all_ * 10000.0) / 100.0;
        p_nondub_t_var  = roundf(track_duration_nondub_  / track_duration_all_ * 10000.0) / 100.0;
    }

    return { TargetInfo("#Up [1]"                        , "Number of updates"                      , num_updates_             ),
             TargetInfo("#PosOutside [1]"                , "Number of updates outside sector"       , num_pos_outside_         ),
             TargetInfo("#PosInside [1]"                 , "Number of updates inside sector"        , num_pos_inside_          ),
             TargetInfo("#DU [1]"                        , "Number of dubious updates inside sector", num_pos_inside_dubious_  ),
             TargetInfo("PDU [%]"                        , "Probability of dubious update"          , p_dubious_up_var         ),
             TargetInfo("#T [1]"                         , "Number of tracks"                       , num_tracks_              ),
             TargetInfo("#DT [1]"                        , "Number of dubious tracks"               , num_tracks_dubious_      ),
             TargetInfo("Duration [s]"                   , "Duration of all tracks"                 , track_duration_var       ),
             TargetInfo("Duration Dubious [s]"           , "Duration of dubious tracks"             , track_duration_dub_var   ),
             TargetInfo("Duration Non-Dubious [s]"       , "Duration of non-dubious tracks"         , track_duration_nondub_var),
             TargetInfo("Average Duration Dubious [s]"   , "Average duration of dubious tracks"     , dubious_t_avg_var        ),
             TargetInfo("Duration Dubious Ratio [%]"     , "Duration ratio of dubious tracks"       , p_dubious_t_var          ),
             TargetInfo("Duration Non-Dubious Ration [%]", "Duration ratio of non-dubious tracks"   , p_nondub_t_var           ) };
}

/**
*/
std::vector<std::string> SingleDubiousTrack::detailHeaders() const
{
    return {"ToD", "TN", "Dubious Comment"};
}

/**
*/
nlohmann::json::array_t SingleDubiousTrack::detailValues(const EvaluationDetail& detail,
                                                         const EvaluationDetail* parent_detail) const
{
    traced_assert(parent_detail);

    return { Utils::Time::toString(detail.timestamp()),
             parent_detail->getValue(DetailKey::UTNOrTrackNum).toUInt(),
             dubiousReasonsString(detail.comments()) }; // "Comment"
}

/**
*/
bool SingleDubiousTrack::detailIsOkStatic(const EvaluationDetail& detail)
{
    auto comments = detail.comments().group(DetailCommentGroupDubious);
    bool is_dub   = (comments.has_value() && !comments->empty());

    return !is_dub;
}

/**
*/
bool SingleDubiousTrack::detailIsOk(const EvaluationDetail& detail) const
{
    return SingleDubiousTrack::detailIsOkStatic(detail);
}

/**
*/
void SingleDubiousTrack::addAnnotationForDetail(nlohmann::json& annotations_json, 
                                                const EvaluationDetail& detail, 
                                                TargetAnnotationType type,
                                                bool is_ok) const
{
    traced_assert(detail.numPositions() >= 1);

    if (type == TargetAnnotationType::Highlight)
    {
        addAnnotationPos(annotations_json, detail.position(0), AnnotationArrayType::TypeHighlight);
    }
    else if (type == TargetAnnotationType::TargetOverview)
    {
        addAnnotationPos(annotations_json, detail.position(0), is_ok ? AnnotationArrayType::TypeOk : AnnotationArrayType::TypeError);
    }
}

/**********************************************************************************************
 * JoinedDubiousTrack
 **********************************************************************************************/

/**
*/
JoinedDubiousTrack::JoinedDubiousTrack(const std::string& result_id, 
                                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                                       const SectorLayer& sector_layer, 
                                       EvaluationCalculator& calculator)
:   DubiousTrackBase()
,   JoinedDubiousBase("JoinedDubiousTrack", result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedDubiousTrack::numIssues() const
{
    return num_pos_inside_dubious_;
}

/**
*/
unsigned int JoinedDubiousTrack::numUpdates() const
{
    return num_updates_;
}

/**
*/
void JoinedDubiousTrack::clearResults_impl() 
{
    num_updates_ = 0;
    num_pos_outside_ = 0;
    num_pos_inside_ = 0;
    num_pos_inside_dubious_ = 0;

    num_tracks_ = 0;
    num_tracks_dubious_ = 0;

    duration_all_ = 0;
    duration_nondub_ = 0;
    duration_dubious_ = 0;
}

/**
*/
void JoinedDubiousTrack::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleDubiousTrack> single = std::static_pointer_cast<SingleDubiousTrack>(single_result);

    num_updates_            += single->numUpdates();
    num_pos_outside_        += single->numPosOutside();
    num_pos_inside_         += single->numPosInside();
    num_pos_inside_dubious_ += single->numPosInsideDubious();

    num_tracks_             += single->numTracks();
    num_tracks_dubious_     += single->numTracksDubious();

    duration_all_     += single->trackDurationAll();
    duration_nondub_  += single->trackDurationNondub();
    duration_dubious_ += single->trackDurationDubious();
}

/**
*/
boost::optional<double> JoinedDubiousTrack::computeResult_impl() const
{
    loginf << "start"
            << " num_updates " << num_updates_
            << " num_tracks " << num_tracks_
            << " num_tracks_dubious " << num_tracks_dubious_
            << " num_pos_inside " << num_pos_inside_
            << " num_pos_inside_dubious " << num_pos_inside_dubious_;

    traced_assert(num_tracks_ >= num_tracks_dubious_);

    p_dubious_update_.reset();

    if (num_pos_inside_)
        p_dubious_update_ = (double)num_pos_inside_dubious_ / (double)num_pos_inside_;

    if (num_tracks_ == 0)
        return {};
    
    return (double)num_tracks_dubious_ / (double)num_tracks_;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedDubiousTrack::sectorInfos() const
{
    auto p_dubious_up_var = SingleProbabilityBase::formatProbabilityOptional(p_dubious_update_);

    std::vector<Joined::SectorInfo> infos = 
        { { "#Up [1]"             , "Number of updates"                      , num_updates_                   },
          { "#PosOutside [1]"     , "Number of updates outside sector"       , num_pos_outside_               },
          { "#PosInside [1]"      , "Number of updates inside sector"        , num_pos_inside_                },
          { "#DU [1]"             , "Number of dubious updates inside sector", num_pos_inside_dubious_        },
          { "PDU [%]"             , "Probability of dubious update"          , p_dubious_up_var               },
          { "#T [1]"              , "Number of tracks"                       , num_tracks_                    },
          { "#DT [1]"             , "Number of dubious tracks"               , num_tracks_dubious_            },
          { "Duration [s]"        , "Duration of all tracks"                 , formatValue(duration_all_)     },
          { "Duration Dubious [s]", "Duration of dubious tracks"             , formatValue(duration_dubious_) } };

    nlohmann::json dubious_t_avg_var;

    if (num_tracks_dubious_)
        dubious_t_avg_var = roundf(duration_dubious_/(float)num_tracks_dubious_ * 100.0) / 100.0;

    infos.push_back({ "Duration Non-Dubious [s]", "Duration of non-dubious tracks", formatValue(duration_nondub_) });
    infos.push_back({ "Average Duration Dubious [s]", "Average duration of dubious tracks", dubious_t_avg_var });

    nlohmann::json p_dubious_t_var, p_nondub_t_var;

    if (duration_all_)
    {
        p_dubious_t_var = std::roundf(duration_dubious_/ duration_all_ * 10000.0) / 100.0;
        p_nondub_t_var  = std::roundf(duration_nondub_ / duration_all_ * 10000.0) / 100.0;
    }

    infos.push_back({ "Duration Ratio Dubious [%]", "Duration ratio of dubious tracks", p_dubious_t_var });
    infos.push_back({ "Duration Ratio Non-Dubious [%]", "Duration ratio of non-dubious tracks", p_nondub_t_var });

    return infos;
}

/**
*/
FeatureDefinitions JoinedDubiousTrack::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    auto getValue = [ = ] (const EvaluationDetail& detail)
    {
        return boost::optional<bool>(SingleDubiousTrack::detailIsOkStatic(detail));
    };

    defs.addDefinition<FeatureDefinitionBinaryGrid>(requirement()->name(), calculator_, "Passed")
        .addDataSeries(ValueSource<bool>(getValue), 
                       GridAddDetailMode::AddEvtPosition, 
                       false);

    return defs;
}

}
