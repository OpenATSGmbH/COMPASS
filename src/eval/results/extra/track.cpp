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

#include "eval/results/extra/track.h"

#include "eval/results/base/featuredefinitions.h"

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/extra/track.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "logger.h"
#include "util/timeconv.h"
#include "viewpoint.h"

#include "traced_assert.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

/**********************************************************************************************
 * ExtraTrackBase
 **********************************************************************************************/

/**
*/
ExtraTrackBase::ExtraTrackBase() = default;

/**
*/
ExtraTrackBase::ExtraTrackBase(unsigned int num_inside, 
                               unsigned int num_extra,  
                               unsigned int num_ok)
:   num_inside_(num_inside)
,   num_extra_ (num_extra)
,   num_ok_    (num_ok)
{
}

/**
*/
unsigned int ExtraTrackBase::numInside() const
{
    return num_inside_;
}

/**
*/
unsigned int ExtraTrackBase::numExtra() const
{
    return num_extra_;
}

/**
*/
unsigned int ExtraTrackBase::numOK() const
{
    return num_ok_;
}

/**********************************************************************************************
 * SingleExtraTrack
 **********************************************************************************************/

/**
*/
SingleExtraTrack::SingleExtraTrack(const std::string& result_id, 
                                   std::shared_ptr<EvaluationRequirement::Base> requirement,
                                   const SectorLayer& sector_layer,
                                   unsigned int utn,
                                   const EvaluationTargetData* target,
                                   EvaluationCalculator& calculator,
                                   const EvaluationDetails& details,
                                   unsigned int num_inside,
                                   unsigned int num_extra,
                                   unsigned int num_ok)
:   ExtraTrackBase(num_inside, num_extra, num_ok)
,   SingleProbabilityBase("SingleExtraTrack", result_id, requirement, sector_layer, utn, target, calculator, details)
{
    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleExtraTrack::createEmptyJoined(const std::string& result_id)
{
    return make_shared<JoinedExtraTrack> (result_id, requirement_, sector_layer_, calculator_);
}


/**
*/
boost::optional<double> SingleExtraTrack::computeResult_impl() const
{
    logdbg << "result_id " << result_id_ << " num_extra " << num_extra_ << " num_ok " << num_ok_;

    traced_assert(num_inside_ >= num_extra_ + num_ok_);

    unsigned int num_total = num_extra_ + num_ok_;

    if (num_total == 0)
        return {};

    return (double)num_extra_ / (double)num_total;
}

/**
*/
unsigned int SingleExtraTrack::numIssues() const
{
    return num_extra_;
}

/**
*/
std::vector<std::string> SingleExtraTrack::targetTableHeadersCustom() const
{
    return { "#Tst", "Ign.", "#Check", "#OK", "#Extra" };
}

/**
*/
nlohmann::json::array_t SingleExtraTrack::targetTableValuesCustom() const
{
    return { target_->numTstUpdates(), ignoreResult(), num_extra_ + num_ok_, num_ok_, num_extra_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleExtraTrack::targetInfos() const
{
    return { TargetInfo("#Tst [1]", "Number of test updates"              , target_->numTstUpdates()),
             TargetInfo("Ign."    , "Ignore target"                       , ignoreResult()          ),
             TargetInfo("#Check." , "Number of checked test track updates", num_extra_ + num_ok_    ),
             TargetInfo("#OK."    , "Number of OK test track updates"     , num_ok_                 ),
             TargetInfo("#Extra"  , "Number of extra test track updates"  , num_extra_              ) };
}

/**
*/
std::vector<std::string> SingleExtraTrack::detailHeaders() const
{
    return {"ToD", "Inside", "TN", "Extra", "Comment"};
}

/**
*/
nlohmann::json::array_t SingleExtraTrack::detailValues(const EvaluationDetail& detail,
                                                       const EvaluationDetail* parent_detail) const
{
    return { Utils::Time::toString(detail.timestamp()),
             detail.getValue(DetailKey::Inside).toBool(),
             detail.getValue(DetailKey::TrackNum).toUInt(),
             detail.getValue(DetailKey::Extra).toBool(),
             detail.comments().generalComment() };
}

/**
*/
bool SingleExtraTrack::detailIsOk(const EvaluationDetail& detail) const
{
    auto is_extra = detail.getValueAs<bool>(DetailKey::Extra);
    traced_assert(is_extra.has_value());

    return !is_extra.value();
}

/**
*/
void SingleExtraTrack::addAnnotationForDetail(nlohmann::json& annotations_json, 
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
 * JoinedExtraTrack
 **********************************************************************************************/

/**
*/
JoinedExtraTrack::JoinedExtraTrack(const std::string& result_id,
                                   std::shared_ptr<EvaluationRequirement::Base> requirement,
                                   const SectorLayer& sector_layer,
                                   EvaluationCalculator& calculator)
:   ExtraTrackBase()
,   JoinedProbabilityBase("JoinedExtraTrack", result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedExtraTrack::numIssues() const
{
    return num_extra_;
}

/**
*/
unsigned int JoinedExtraTrack::numUpdates() const
{
    return num_extra_ + num_ok_;
}

/**
*/
void JoinedExtraTrack::clearResults_impl() 
{
    num_inside_ = 0;
    num_extra_  = 0;
    num_ok_     = 0;
}

/**
*/
void JoinedExtraTrack::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleExtraTrack> single = std::static_pointer_cast<SingleExtraTrack>(single_result);

    num_inside_ += single->numInside();
    num_extra_  += single->numExtra();
    num_ok_     += single->numOK();
}

/**
*/
boost::optional<double> JoinedExtraTrack::computeResult_impl() const
{
    loginf << "start"
            << " num_inside " << num_inside_
            << " num_extra " << num_extra_
            << " num_ok " << num_ok_;

    traced_assert(num_inside_ >= num_extra_ + num_ok_);

    unsigned int total = num_extra_ + num_ok_;

    if (total == 0)
        return {};
    
    return (double)num_extra_ / (double)total;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedExtraTrack::sectorInfos() const
{
    return { { "#Check." , "Number of checked test track updates", num_extra_ + num_ok_ },
             { "#OK."    , "Number of OK test track updates"     , num_ok_              },
             { "#Extra"  , "Number of extra test track updates"  , num_extra_           } };
}

/**
*/
FeatureDefinitions JoinedExtraTrack::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    defs.addDefinition<FeatureDefinitionBinaryGrid>(requirement()->name(), calculator_, "Passed")
        .addDataSeries(SingleExtraTrack::DetailKey::Extra, 
                       GridAddDetailMode::AddEvtPosition, 
                       true);

    return defs;
}

}
