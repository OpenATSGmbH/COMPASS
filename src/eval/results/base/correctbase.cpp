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

#include "eval/results/base/correctbase.h"
#include "eval/results/base/featuredefinitions.h"

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"

#include "eval/requirement/base/base.h"

#include "evaluationtargetdata.h"
#include "evaluationcalculator.h"

#include "logger.h"
#include "util/timeconv.h"
#include "viewpoint.h"

#include "traced_assert.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

/********************************************************************************************************
 * CorrectBase
 ********************************************************************************************************/

/**
*/
CorrectBase::CorrectBase(const std::string& correct_value_name,
                         const std::string& correct_short_name,
                         const std::string& not_correct_short_name)
:   correct_value_name_    (correct_value_name)
,   correct_short_name_    (correct_short_name)
,   not_correct_short_name_(not_correct_short_name)
{
}

/**
*/
CorrectBase::CorrectBase(unsigned int num_updates, 
                         unsigned int num_no_ref_pos, 
                         unsigned int num_no_ref_id,
                         unsigned int num_pos_outside, 
                         unsigned int num_pos_inside,
                         unsigned int num_correct, 
                         unsigned int num_not_correct,
                         const std::string& correct_value_name,
                         const std::string& correct_short_name,
                         const std::string& not_correct_short_name)
:   num_updates_           (num_updates)
,   num_no_ref_pos_        (num_no_ref_pos)
,   num_no_ref_id_         (num_no_ref_id)
,   num_pos_outside_       (num_pos_outside)
,   num_pos_inside_        (num_pos_inside)
,   num_correct_           (num_correct)
,   num_not_correct_       (num_not_correct)
,   correct_value_name_    (correct_value_name)
,   correct_short_name_    (correct_short_name)
,   not_correct_short_name_(not_correct_short_name)
{
}

/**
*/
unsigned int CorrectBase::numNoRefPos() const
{
    return num_no_ref_pos_;
}

/**
*/
unsigned int CorrectBase::numNoRefId() const
{
    return num_no_ref_id_;
}

/**
*/
unsigned int CorrectBase::numPosOutside() const
{
    return num_pos_outside_;
}

/**
*/
unsigned int CorrectBase::numPosInside() const
{
    return num_pos_inside_;
}

/**
*/
unsigned int CorrectBase::numUpdates() const
{
    return num_updates_;
}

/**
*/
unsigned int CorrectBase::numCorrect() const
{
    return num_correct_;
}

/**
*/
unsigned int CorrectBase::numNotCorrect() const
{
    return num_not_correct_;
}

/********************************************************************************************************
 * SingleCorrectBase
 ********************************************************************************************************/

/**
*/
SingleCorrectBase::SingleCorrectBase(const std::string& result_type,
                                     const std::string& result_id, 
                                     std::shared_ptr<EvaluationRequirement::Base> requirement,
                                     const SectorLayer& sector_layer,
                                     unsigned int utn,
                                     const EvaluationTargetData* target,
                                     EvaluationCalculator& calculator,
                                     const EvaluationDetails& details,
                                     unsigned int num_updates,
                                     unsigned int num_no_ref_pos,
                                     unsigned int num_no_ref_id,
                                     unsigned int num_pos_outside,
                                     unsigned int num_pos_inside,
                                     unsigned int num_correct,
                                     unsigned int num_not_correct,
                                     const std::string& correct_value_name,
                                     const std::string& correct_short_name,
                                     const std::string& not_correct_short_name)
:   CorrectBase(num_updates, num_no_ref_pos, num_no_ref_id, num_pos_outside, num_pos_inside, num_correct, num_not_correct,
                correct_value_name, correct_short_name, not_correct_short_name)
,   SingleProbabilityBase(result_type, result_id, requirement, sector_layer, utn, target, calculator, details)
{
}

/**
*/
boost::optional<double> SingleCorrectBase::computeResult_impl() const
{
    traced_assert(num_updates_ - num_no_ref_pos_ == num_pos_inside_ + num_pos_outside_);
    traced_assert(num_pos_inside_ == num_no_ref_id_ + num_correct_ + num_not_correct_);

    unsigned int num_total = num_correct_ + num_not_correct_;

    if (num_total == 0)
        return {};
    
    return (double)num_correct_ / (double)num_total;
}

/**
*/
unsigned int SingleCorrectBase::numIssues() const
{
    return num_not_correct_;
}

/**
*/
std::vector<std::string> SingleCorrectBase::targetTableHeadersCustom() const
{
    return { "#Up", "#NoRef", correct_short_name_, not_correct_short_name_ };
}

/**
*/
nlohmann::json::array_t SingleCorrectBase::targetTableValuesCustom() const
{
    return { num_updates_, num_no_ref_pos_ + num_no_ref_id_, num_correct_, num_not_correct_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleCorrectBase::targetInfos() const
{
    std::string sn_c  = correct_short_name_;
    std::string sn_nc = not_correct_short_name_;
    std::string cvn   = correct_value_name_;

    std::vector<Single::TargetInfo> infos = 
        { { "#Up [1]"        , "Number of updates"                                 , num_updates_                     },
          { "#NoRef [1]"     , "Number of updates w/o reference position or " + cvn, num_no_ref_pos_ + num_no_ref_id_ },
          { "#NoRefPos [1]"  , "Number of updates w/o reference position "         , num_no_ref_pos_                  },
          { "#NoRef [1]"     , "Number of updates w/o reference " + cvn            , num_no_ref_id_                   },
          { "#PosOutside [1]", "Number of updates outside sector"                  , num_pos_outside_                 },
          { "#PosInside [1]" , "Number of updates inside sector"                   , num_pos_inside_                  },
          { sn_c + " [1]"    , "Number of updates with correct " + cvn             , num_correct_                     },
          { sn_nc + " [1]"   , "Number of updates with no correct " + cvn          , num_not_correct_                 } };

    auto additional = additionalTargetInfos();

    infos.insert(infos.end(), additional.begin(), additional.end());

    return infos;
}

/**
*/
std::vector<std::string> SingleCorrectBase::detailHeaders() const
{
    return { "ToD", "Ref", "Ok", "#Up", "#NoRef", "#PosInside", "#PosOutside", correct_short_name_, not_correct_short_name_, "Comment" };
}

/**
*/
nlohmann::json::array_t SingleCorrectBase::detailValues(const EvaluationDetail& detail,
                                                        const EvaluationDetail* parent_detail) const
{
    return { Utils::Time::toString(detail.timestamp()),
             detail.getValue(DetailKey::RefExists).toBool(),
            !detail.getValue(DetailKey::IsNotCorrect).toBool(),
             detail.getValue(DetailKey::NumUpdates).toUInt(),
             detail.getValue(DetailKey::NumNoRef).toUInt(),
             detail.getValue(DetailKey::NumInside).toUInt(),
             detail.getValue(DetailKey::NumOutside).toUInt(),
             detail.getValue(DetailKey::NumCorrect).toUInt(),
             detail.getValue(DetailKey::NumNotCorrect).toUInt(),
             detail.comments().generalComment() };
}

/**
*/
bool SingleCorrectBase::detailIsOk(const EvaluationDetail& detail) const
{
    auto is_not_correct = detail.getValueAs<bool>(DetailKey::IsNotCorrect);
    traced_assert(is_not_correct.has_value());

    return !is_not_correct.value();
}

/**
*/
void SingleCorrectBase::addAnnotationForDetail(nlohmann::json& annotations_json, 
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

/***************************************************************************
 * JoinedFalseBase
 ***************************************************************************/

/**
*/
JoinedCorrectBase::JoinedCorrectBase(const std::string& result_type,
                                     const std::string& result_id, 
                                     std::shared_ptr<EvaluationRequirement::Base> requirement,
                                     const SectorLayer& sector_layer, 
                                     EvaluationCalculator& calculator,
                                     const std::string& correct_value_name,
                                     const std::string& correct_short_name,
                                     const std::string& not_correct_short_name)
:   CorrectBase(correct_value_name, correct_short_name, not_correct_short_name)
,   JoinedProbabilityBase(result_type, result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedCorrectBase::numIssues() const
{
    return num_not_correct_;
}

/**
*/
unsigned int JoinedCorrectBase::numUpdates() const
{
    return num_correct_ + num_not_correct_;
}

/**
*/
void JoinedCorrectBase::clearResults_impl() 
{
    num_updates_     = 0;
    num_no_ref_pos_  = 0;
    num_no_ref_id_   = 0;
    num_pos_outside_ = 0;
    num_pos_inside_  = 0;
    num_correct_     = 0;
    num_not_correct_ = 0;
}

/**
*/
void JoinedCorrectBase::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleCorrectBase> single_correct = std::static_pointer_cast<SingleCorrectBase>(single_result);

    num_updates_     += single_correct->numUpdates();
    num_no_ref_pos_  += single_correct->numNoRefPos();
    num_no_ref_id_   += single_correct->numNoRefId();
    num_pos_outside_ += single_correct->numPosOutside();
    num_pos_inside_  += single_correct->numPosInside();
    num_correct_     += single_correct->numCorrect();
    num_not_correct_ += single_correct->numNotCorrect();
}

/**
*/
boost::optional<double> JoinedCorrectBase::computeResult_impl() const
{
    loginf << "start" << type()
            << " num_updates " << num_updates_
            << " num_no_ref_pos " << num_no_ref_pos_
            << " num_no_ref_id " << num_no_ref_id_
            << " num_correct " << num_correct_
            << " num_not_correct " << num_not_correct_;

    traced_assert(num_updates_ - num_no_ref_pos_ == num_pos_inside_ + num_pos_outside_);
    traced_assert(num_pos_inside_ == num_no_ref_id_+ num_correct_ + num_not_correct_);

    unsigned int total = num_correct_ + num_not_correct_;

    if (total == 0)
        return {};

    return (double)num_correct_ / (double)total;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedCorrectBase::sectorInfos() const
{
    std::string sn_c  = correct_short_name_;
    std::string sn_nc = not_correct_short_name_;
    std::string cvn   = correct_value_name_;

    return { { "#Updates [1]"   , "Total number target reports"                       , num_updates_                     },
             { "#NoRef [1]"     , "Number of updates w/o reference position or " + cvn, num_no_ref_pos_ + num_no_ref_id_ },
             { "#NoRefPos [1]"  , "Number of updates w/o reference position "         , num_no_ref_pos_                  },
             { "#NoRef [1]"     , "Number of updates w/o reference " + cvn            , num_no_ref_id_                   },
             { "#PosOutside [1]", "Number of updates outside sector"                  , num_pos_outside_                 },
             { "#PosInside [1]" , "Number of updates inside sector"                   , num_pos_inside_                  },
             { sn_c + " [1]"    , "Number of updates with correct " + cvn             , num_correct_                     },
             { sn_nc + " [1]"   , "Number of updates with no correct " + cvn          , num_not_correct_                 } };
}

/**
*/
FeatureDefinitions JoinedCorrectBase::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    defs.addDefinition<FeatureDefinitionBinaryGrid>(requirement()->name(), calculator_, "Passed")
        .addDataSeries(SingleCorrectBase::DetailKey::IsNotCorrect, 
                       GridAddDetailMode::AddEvtRefPosition, 
                       true);
    return defs;
}

}
