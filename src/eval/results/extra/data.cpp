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

#include "eval/results/extra/data.h"

#include "eval/results/base/featuredefinitions.h"

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/extra/data.h"

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
 * ExtraDataBase
 **********************************************************************************************/

/**
*/
ExtraDataBase::ExtraDataBase() = default;

/**
*/
ExtraDataBase::ExtraDataBase(unsigned int num_extra, 
                             unsigned int num_ok)
:   num_extra_(num_extra)
,   num_ok_   (num_ok   )
{
}

/**
*/
unsigned int ExtraDataBase::numExtra() const
{
    return num_extra_;
}

/**
*/
unsigned int ExtraDataBase::numOK() const
{
    return num_ok_;
}

/**********************************************************************************************
 * SingleExtraData
 **********************************************************************************************/

/**
*/
SingleExtraData::SingleExtraData(const std::string& result_id, 
                                 std::shared_ptr<EvaluationRequirement::Base> requirement,
                                 const SectorLayer& sector_layer,
                                 unsigned int utn,
                                 const EvaluationTargetData* target,
                                 EvaluationCalculator& calculator,
                                 const EvaluationDetails& details,
                                 unsigned int num_extra,
                                 unsigned int num_ok,
                                 bool has_extra_test_data)
:   ExtraDataBase(num_extra, num_ok)
,   SingleProbabilityBase("SingleExtraData", result_id, requirement, sector_layer, utn, target, calculator, details)
,   has_extra_test_data_(has_extra_test_data)
{
    updateResult();
}

/**
*/
bool SingleExtraData::hasExtraTestData() const
{
    return has_extra_test_data_;
}

/**
*/
std::shared_ptr<Joined> SingleExtraData::createEmptyJoined(const std::string& result_id)
{
    return make_shared<JoinedExtraData> (result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
boost::optional<double> SingleExtraData::computeResult_impl() const
{
    unsigned int num_total = num_extra_ + num_ok_;

    if (num_total == 0)
        return {};

    boost::optional<double> result = (double)num_extra_ / (double)num_total;

    logdbg << "result_id " << result_id_ << " num_extra " << num_extra_ << " num_ok " << num_ok_;

    return result;
}

/**
*/
unsigned int SingleExtraData::numIssues() const
{
    return num_extra_;
}

/**
*/
std::vector<std::string> SingleExtraData::targetTableHeadersCustom() const
{
    return { "#Tst", "Ign.", "#Check", "#OK", "#Extra" };
}

/**
*/
nlohmann::json::array_t SingleExtraData::targetTableValuesCustom() const
{
    return { target_->numTstUpdates(), ignoreResult(), num_extra_ + num_ok_, num_ok_, num_extra_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleExtraData::targetInfos() const
{
    return { TargetInfo("#Ref [1]", "Number of reference updates"   , target_->numRefUpdates()),
             TargetInfo("#Tst [1]", "Number of test updates"        , target_->numTstUpdates()),
             TargetInfo("Ign."    , "Ignore target"                 , ignoreResult()          ),
             TargetInfo("#Check." , "Number of checked test updates", num_extra_ + num_ok_    ),
             TargetInfo("#OK."    , "Number of OK test updates"     , num_ok_                 ),
             TargetInfo("#Extra"  , "Number of extra test updates"  , num_extra_              ) };
}

/**
*/
std::vector<std::string> SingleExtraData::detailHeaders() const
{
    return {"ToD", "Inside", "Extra", "Ref.", "Comment"};
}

/**
*/
nlohmann::json::array_t SingleExtraData::detailValues(const EvaluationDetail& detail,
                                                      const EvaluationDetail* parent_detail) const
{
    return { Utils::Time::toString(detail.timestamp()),
             detail.getValue(DetailKey::Inside).toBool(),
             detail.getValue(DetailKey::Extra).toBool(),
             detail.getValue(DetailKey::RefExists).toBool(),
             detail.comments().generalComment() };
}

/**
*/
bool SingleExtraData::detailIsOk(const EvaluationDetail& detail) const
{
    auto is_extra = detail.getValueAs<bool>(DetailKey::Extra);
    traced_assert(is_extra.has_value());

    return !is_extra.value();
}

/**
*/
void SingleExtraData::addAnnotationForDetail(nlohmann::json& annotations_json, 
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
 * JoinedExtraData
 **********************************************************************************************/

/**
*/
JoinedExtraData::JoinedExtraData(const std::string& result_id, 
                                 std::shared_ptr<EvaluationRequirement::Base> requirement,
                                 const SectorLayer& sector_layer,
                                 EvaluationCalculator& calculator)
:   JoinedProbabilityBase("JoinedExtraData", result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedExtraData::numIssues() const
{
    return num_extra_;
}

/**
*/
unsigned int JoinedExtraData::numUpdates() const
{
    return num_extra_ + num_ok_;
}

/**
*/
void JoinedExtraData::clearResults_impl() 
{
    num_extra_ = 0;
    num_ok_    = 0;
}

/**
*/
void JoinedExtraData::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleExtraData> single = std::static_pointer_cast<SingleExtraData>(single_result);

    num_extra_ += single->numExtra();
    num_ok_    += single->numOK();
}

/**
*/
boost::optional<double> JoinedExtraData::computeResult_impl() const
{
    loginf << "start"
            << " num_extra " << num_extra_
            << " num_ok " << num_ok_;

    unsigned int total = num_extra_ + num_ok_;

    if (total == 0)
        return {};
    
    return (double)num_extra_ / (double)total;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedExtraData::sectorInfos() const
{
    return { { "#Check." , "Number of checked test updates", num_extra_ + num_ok_ },
             { "#OK."    , "Number of OK test updates"     , num_ok_              },
             { "#Extra"  , "Number of extra test updates"  , num_extra_           } };
}

/**
*/
FeatureDefinitions JoinedExtraData::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    defs.addDefinition<FeatureDefinitionBinaryGrid>(requirement()->name(), calculator_, "Passed")
        .addDataSeries(SingleExtraData::DetailKey::Extra, 
                       GridAddDetailMode::AddEvtPosition, 
                       true);

    return defs;
}

}
