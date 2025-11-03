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

#include "eval/results/generic/generic.h"

#include "eval/results/base/featuredefinitions.h"

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/generic/generic.h"

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

/****************************************************************************************
 * GenericBase
 ****************************************************************************************/

/**
*/
GenericBase::GenericBase() = default;

/**
*/
GenericBase::GenericBase(unsigned int num_updates,
                         unsigned int num_no_ref_pos,
                         unsigned int num_no_ref_val,
                         unsigned int num_pos_outside,
                         unsigned int num_pos_inside,
                         unsigned int num_unknown,
                         unsigned int num_correct,
                         unsigned int num_false)
:   num_updates_    (num_updates)
,   num_no_ref_pos_ (num_no_ref_pos)
,   num_no_ref_val_ (num_no_ref_val)
,   num_pos_outside_(num_pos_outside)
,   num_pos_inside_ (num_pos_inside)
,   num_unknown_    (num_unknown)
,   num_correct_    (num_correct)
,   num_false_      (num_false)
{
}

/**
*/
unsigned int GenericBase::numNoRefPos() const
{
    return num_no_ref_pos_;
}

/**
*/
unsigned int GenericBase::numNoRefValue() const
{
    return num_no_ref_val_;
}

/**
*/
unsigned int GenericBase::numPosOutside() const
{
    return num_pos_outside_;
}

/**
*/
unsigned int GenericBase::numPosInside() const
{
    return num_pos_inside_;
}

/**
*/
unsigned int GenericBase::numUpdates() const
{
    return num_updates_;
}

/**
*/
unsigned int GenericBase::numUnknown() const
{
    return num_unknown_;
}

/**
*/
unsigned int GenericBase::numCorrect() const
{
    return num_correct_;
}

/**
*/
unsigned int GenericBase::numFalse() const
{
    return num_false_;
}

/****************************************************************************************
 * SingleGeneric
 ****************************************************************************************/

/**
*/
SingleGeneric::SingleGeneric(const std::string& result_type, const std::string& result_id,
                                   std::shared_ptr<EvaluationRequirement::Base> requirement,
                                   const SectorLayer& sector_layer,
                                   unsigned int utn,
                                   const EvaluationTargetData* target,
                                   EvaluationCalculator& calculator,
                                   const EvaluationDetails& details,
                                   unsigned int num_updates,
                                   unsigned int num_no_ref_pos,
                                   unsigned int num_no_ref_val,
                                   unsigned int num_pos_outside,
                                   unsigned int num_pos_inside,
                                   unsigned int num_unknown,
                                   unsigned int num_correct,
                                   unsigned int num_false)
:   GenericBase(num_updates, num_no_ref_pos, num_no_ref_val, num_pos_outside,
                num_pos_inside, num_unknown, num_correct, num_false)
,   SingleProbabilityBase(result_type, result_id, requirement, sector_layer, utn, target, calculator, details)
{
    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleGeneric::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedGeneric> (type_, result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
EvaluationRequirement::GenericBase& SingleGeneric::genericRequirement() const
{
    traced_assert(requirement_);
    EvaluationRequirement::GenericBase* req_ptr = dynamic_cast<EvaluationRequirement::GenericBase*>(requirement_.get());
    traced_assert(req_ptr);

    return *req_ptr;
}

/**
*/
boost::optional<double> SingleGeneric::computeResult_impl() const
{
    traced_assert(num_updates_ - num_no_ref_pos_ == num_pos_inside_ + num_pos_outside_);
    traced_assert(num_pos_inside_ == num_no_ref_val_ + num_unknown_ + num_correct_+num_false_);

    unsigned int num_total = num_correct_ + num_false_;

    if (num_total == 0)
        return {};

    boost::optional<double> result = (double)(num_correct_) / (double)num_total;
    
    return result;
}

/**
*/
unsigned int SingleGeneric::numIssues() const
{
    return num_false_;
}

/**
*/
std::vector<std::string> SingleGeneric::targetTableHeadersCustom() const
{
    return { "#Up", "#NoRef", "#Unknown", "#Correct", "#False" };
}

/**
*/
nlohmann::json::array_t SingleGeneric::targetTableValuesCustom() const
{
    return { num_updates_, num_no_ref_pos_ + num_no_ref_val_, num_unknown_, num_correct_, num_false_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleGeneric::targetInfos() const
{
    EvaluationRequirement::GenericBase& req = genericRequirement();

    unsigned int no_ref = num_no_ref_pos_ + num_no_ref_val_;

    std::string valname = req.valueName();

    std::vector<Single::TargetInfo> infos = 
        { { "#Up [1]"        , "Number of updates"                                     , num_updates_     },
          { "#NoRef [1]"     , "Number of updates w/o reference position or " + valname, no_ref           },
          { "#NoRefPos [1]"  , "Number of updates w/o reference position "             , num_no_ref_pos_  },
          { "#NoRef [1]"     , "Number of updates w/o reference " + valname            , num_no_ref_val_  },
          { "#PosOutside [1]", "Number of updates outside sector"                      , num_pos_outside_ }, 
          { "#PosInside [1]" , "Number of updates inside sector"                       , num_pos_inside_  },
          { "#Unknown [1]"   , "Number of updates unknown " + valname                  , num_unknown_     },
          { "#Correct [1]"   , "Number of updates with correct " + valname             , num_correct_     }, 
          { "#False [1]"     , "Number of updates with incorrect " + valname           , num_false_       } };

    return infos;
}

/**
*/
std::vector<std::string> SingleGeneric::detailHeaders() const
{
    return { "ToD", "Ref", "Ok", "#Up", "#NoRef", "#PosInside", "#PosOutside", "#Unknown", "#Correct", "#False", "Comment" };
}

/**
*/
nlohmann::json::array_t SingleGeneric::detailValues(const EvaluationDetail& detail,
                                                    const EvaluationDetail* parent_detail) const
{
    return { Utils::Time::toString(detail.timestamp()),
             detail.getValue(DetailKey::RefExists).toBool(),
            !detail.getValue(DetailKey::IsNotOk).toBool(),
             detail.getValue(DetailKey::NumUpdates).toUInt(),
             detail.getValue(DetailKey::NumNoRef).toUInt(),
             detail.getValue(DetailKey::NumInside).toUInt(),
             detail.getValue(DetailKey::NumOutside).toUInt(),
             detail.getValue(DetailKey::NumUnknownID).toUInt(),
             detail.getValue(DetailKey::NumCorrectID).toUInt(),
             detail.getValue(DetailKey::NumFalseID).toUInt(),
             detail.comments().generalComment() };
}

/**
*/
bool SingleGeneric::detailIsOk(const EvaluationDetail& detail) const
{
    auto is_not_ok = detail.getValueAs<bool>(DetailKey::IsNotOk);
    traced_assert(is_not_ok.has_value());

    return !is_not_ok.value();
}

/**
*/
void SingleGeneric::addAnnotationForDetail(nlohmann::json& annotations_json, 
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

/****************************************************************************************
 * JoinedGeneric
 ****************************************************************************************/

/**
*/
JoinedGeneric::JoinedGeneric(const std::string& result_type, 
                             const std::string& result_id,
                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                             const SectorLayer& sector_layer, 
                             EvaluationCalculator& calculator)
:   JoinedProbabilityBase(result_type, result_id, requirement, sector_layer, calculator)
{
}

/**
*/
EvaluationRequirement::GenericBase& JoinedGeneric::genericRequirement() const
{
    traced_assert(requirement_);

    EvaluationRequirement::GenericBase* req_ptr = dynamic_cast<EvaluationRequirement::GenericBase*>(requirement_.get());
    traced_assert(req_ptr);

    return *req_ptr;
}

/**
*/
unsigned int JoinedGeneric::numIssues() const
{
    return num_false_;
}

/**
*/
unsigned int JoinedGeneric::numUpdates() const
{
    return num_correct_ + num_false_;
}

/**
*/
void JoinedGeneric::clearResults_impl() 
{
    num_updates_     = 0;
    num_no_ref_pos_  = 0;
    num_no_ref_val_  = 0;
    num_pos_outside_ = 0;
    num_pos_inside_  = 0;
    num_unknown_     = 0;
    num_correct_     = 0;
    num_false_       = 0;
}

/**
*/
void JoinedGeneric::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleGeneric> single = std::static_pointer_cast<SingleGeneric>(single_result);

    num_updates_     += single->numUpdates();
    num_no_ref_pos_  += single->numNoRefPos();
    num_no_ref_val_  += single->numNoRefValue();
    num_pos_outside_ += single->numPosOutside();
    num_pos_inside_  += single->numPosInside();
    num_unknown_     += single->numUnknown();
    num_correct_     += single->numCorrect();
    num_false_       += single->numFalse();
}

/**
*/
boost::optional<double> JoinedGeneric::computeResult_impl() const
{
    loginf << "start"
            << " num_updates " << num_updates_
            << " num_no_ref_pos " << num_no_ref_pos_
            << " num_no_ref_val " << num_no_ref_val_
            << " num_unknown " << num_unknown_
            << " num_correct " << num_correct_
            << " num_false " << num_false_;

    traced_assert(num_updates_ - num_no_ref_pos_ == num_pos_inside_ + num_pos_outside_);
    traced_assert(num_pos_inside_ == num_no_ref_val_+num_unknown_+num_correct_+num_false_);

    unsigned int total = num_correct_ + num_false_;

    if (total == 0)
        return {};

    return (double)num_correct_ / (double)total;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedGeneric::sectorInfos() const
{
    EvaluationRequirement::GenericBase& req = genericRequirement();

    std::string name = req.valueName();

    unsigned int no_ref = num_no_ref_pos_ + num_no_ref_val_;

    return { { "#Up [1]"        , "Number of updates", num_updates_ },
             { "#NoRef [1]"     , "Number of updates w/o reference position or "+name, no_ref           }, 
             { "#NoRefPos [1]"  , "Number of updates w/o reference position"         , num_no_ref_pos_  }, 
             { "#NoRef [1]"     , "Number of updates w/o reference " + name          , num_no_ref_val_  }, 
             { "#PosOutside [1]", "Number of updates outside sector"                 , num_pos_outside_ },
             { "#PosInside [1]" , "Number of updates inside sector"                  , num_pos_inside_  }, 
             { "#Unknown [1]"   , "Number of updates unknown " + name                , num_unknown_     },
             { "#Correct [1]"   , "Number of updates with correct " + name           , num_correct_     }, 
             { "#False [1]"     , "Number of updates with incorrect " + name         , num_false_       } };
}

/**
*/
FeatureDefinitions JoinedGeneric::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    defs.addDefinition<FeatureDefinitionBinaryGrid>(requirement()->name(), calculator_, "Passed")
        .addDataSeries(SingleGeneric::DetailKey::IsNotOk, 
                       GridAddDetailMode::AddEvtRefPosition, 
                       true);

    return defs;
}

}
