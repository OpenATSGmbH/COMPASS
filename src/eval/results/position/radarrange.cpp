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

#include "results/position/radarrange.h"
#include "requirement/position/radar_helpers.h"
#include "requirement/position/radarrange.h"
#include "stringconv.h"

#include "logger.h"


#include "traced_assert.h"

using namespace Utils;

namespace EvaluationRequirementResult
{

namespace
{

double rangeStatisticValue(const std::shared_ptr<EvaluationRequirement::Base>& requirement,
                           double stddev, const EvaluationRequirement::RadarHelpers::RangeErrorModel& model)
{
    auto req = std::dynamic_pointer_cast<EvaluationRequirement::PositionRadarRange>(requirement);
    traced_assert(req);

    return EvaluationRequirement::RadarHelpers::statisticValue(req->statistic(), stddev, model.bias_m, model.gain_m_per_nm);
}

}

/**********************************************************************************************
 * SinglePositionRadarRange
 **********************************************************************************************/

/**
*/
SinglePositionRadarRange::SinglePositionRadarRange(const std::string& result_id,
                                                   std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                   const SectorLayer& sector_layer,
                                                   unsigned int utn,
                                                   const EvaluationTargetData* target,
                                                   EvaluationCalculator& calculator,
                                                   const EvaluationDetails& details,
                                                   unsigned int num_pos,
                                                   unsigned int num_no_ref,
                                                   unsigned int num_pos_outside,
                                                   unsigned int num_pos_inside,
                                                   unsigned int num_ref_inaccurate,
                                                   unsigned int num_comp_passed,
                                                   unsigned int num_comp_failed,
                                                   const std::vector<double>& range_values_ref,
                                                   const std::vector<double>& range_values_tst)
:   SinglePositionValueBase("SinglePositionRadarRange", result_id, requirement, sector_layer, utn, target, calculator, details,
                            num_pos, num_no_ref,num_pos_outside, num_pos_inside, num_ref_inaccurate, num_comp_passed, num_comp_failed)
,   range_values_ref_(range_values_ref)
,   range_values_tst_(range_values_tst)
{
    traced_assert(range_values_ref_.size() == range_values_tst_.size());

    updateResult();
}

/**
 */
void SinglePositionRadarRange::addDetailCodeTexts(ReportTableDefinition& def) const
{
    DetailCode::addTypeText(def, NoDataSourceInfo, "No data source position");
    DetailCode::addTypeText(def, RefTransformationError, "Reference position transformation error");
    DetailCode::addTypeText(def, TstTransformationError, "Test position transformation error");
}

/**
*/
std::shared_ptr<Joined> SinglePositionRadarRange::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedPositionRadarRange> (result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
std::vector<std::string> SinglePositionRadarRange::targetTableHeadersCustom() const
{
    return { "DMin", "DMax", "DAvg", "DSDev", "Bias", "Gain", "#CF", "#CP" };
}

/**
*/
nlohmann::json::array_t SinglePositionRadarRange::targetTableValuesCustom() const
{
    return { formatValue(accumulator_.min()), 
             formatValue(accumulator_.max()),
             formatValue(accumulator_.mean()),
             formatValue(accumulator_.stddev()),
             range_bias_.is_null() ? range_bias_ : nlohmann::json(formatValue(range_bias_.get<double>())),
             range_gain_.is_null() ? range_gain_ : nlohmann::json(formatValue(range_gain_.get<double>())),
             num_failed_, 
             num_passed_ }; 
}

/**
*/
std::vector<Single::TargetInfo> SinglePositionRadarRange::targetInfos() const
{
    std::vector<Single::TargetInfo> infos = 
        { { "#Pos [1]"       , "Number of updates"                        , num_pos_                           }, 
          { "#NoRef [1]"     , "Number of updates w/o reference positions", num_no_ref_                        },
          { "#PosOutside [1]", "Number of updates outside sector"         , num_pos_outside_                   }, 
          { "#PosInside [1]" , "Number of updates inside sector"          , num_pos_inside_                    },
          { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_   },
          { "#RefPosIn [%]"  , "Percentage of updates with inaccurate reference position"  , String::percentToStringProtected(num_ref_inaccurate_, num_pos_inside_, 2).c_str()},
          { "DMin [m]"       , "Minimum of distance"                      , formatValue(accumulator_.min())    }, 
          { "DMax [m]"       , "Maximum of distance"                      , formatValue(accumulator_.max())    },
          { "DAvg [m]"       , "Average of distance"                      , formatValue(accumulator_.mean())   }, 
          { "DSDev [m]"      , "Standard Deviation of distance"           , formatValue(accumulator_.stddev()) }, 
          { "DVar [m^2]"     , "Variance of distance"                     , formatValue(accumulator_.var())    } };

    if (!range_bias_.is_null())
        infos.push_back({ "Range Bias [m]", "Bias of the range error model (test = (1 + gain) * reference + bias)", formatValue(range_bias_.get<double>()) });

    if (!range_gain_.is_null())
        infos.push_back({ "Range Gain [m/NM]", "Gain of the range error model (test = (1 + gain) * reference + bias)", formatValue(range_gain_.get<double>()) });

    infos.push_back({ "#CF [1]", "Number of updates with failed comparison" , num_failed_});
    infos.push_back({ "#CP [1]", "Number of updates with passed comparison" , num_passed_});

    return infos;
}

/**
*/
boost::optional<double> SinglePositionRadarRange::computeFinalResultValue() const
{
    range_gain_ = nlohmann::json();
    range_bias_ = nlohmann::json();

    if (accumulator_.numValues() == 0)
        return {};

    traced_assert(accumulator_.numValues() == range_values_ref_.size());

    auto model = EvaluationRequirement::RadarHelpers::rangeErrorModel(range_values_ref_, range_values_tst_);

    range_gain_ = model.gain_m_per_nm;
    range_bias_ = model.bias_m;

    return rangeStatisticValue(requirement_, accumulator_.stddev(), model);
}

/**********************************************************************************************
 * JoinedPositionRadarRange
 **********************************************************************************************/

/**
*/
JoinedPositionRadarRange::JoinedPositionRadarRange(const std::string& result_id,
                                                   std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                   const SectorLayer& sector_layer,
                                                   EvaluationCalculator& calculator)
:   JoinedPositionValueBase("JoinedPositionRadarRange", result_id, requirement, sector_layer, calculator, "distance")
{
}

/**
*/
std::vector<Joined::SectorInfo> JoinedPositionRadarRange::sectorInfos() const
{
    std::vector<Joined::SectorInfo> infos = 
        { { "#Pos [1]"       , "Number of updates"                        , num_pos_                           }, 
          { "#NoRef [1]"     , "Number of updates w/o reference positions", num_no_ref_                        },
          { "#PosOutside [1]", "Number of updates outside sector"         , num_pos_outside_                   }, 
          { "#PosInside [1]" , "Number of updates inside sector"          , num_pos_inside_                    },
          { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_   },
          { "#RefPosIn [%]"  , "Percentage of updates with inaccurate reference position"  , String::percentToStringProtected(num_ref_inaccurate_, num_pos_inside_, 2).c_str()},
          { "DMin [m]"       , "Minimum of distance"                      , formatValue(accumulator_.min())    }, 
          { "DMax [m]"       , "Maximum of distance"                      , formatValue(accumulator_.max())    },
          { "DAvg [m]"       , "Average of distance"                      , formatValue(accumulator_.mean())   }, 
          { "DSDev [m]"      , "Standard Deviation of distance"           , formatValue(accumulator_.stddev()) }, 
          { "DVar [m^2]"     , "Variance of distance"                     , formatValue(accumulator_.var())    } };

    if (range_bias_.isValid())
        infos.push_back({ "Range Bias [m]", "Bias of the range error model (test = (1 + gain) * reference + bias)", formatValue(range_bias_.toDouble()) });

    if (range_gain_.isValid())
        infos.push_back({ "Range Gain [m/NM]", "Gain of the range error model (test = (1 + gain) * reference + bias)", formatValue(range_gain_.toDouble()) });

    infos.push_back({ "#CF [1]", "Number of updates with failed comparison" , num_failed_});
    infos.push_back({ "#CP [1]", "Number of updates with passed comparison" , num_passed_});

    return infos;
}

/**
*/
boost::optional<double> JoinedPositionRadarRange::computeFinalResultValue() const
{
    range_gain_ = QVariant();
    range_bias_ = QVariant();

    if (accumulator_.numValues() == 0)
        return {};

    auto single_results = usedSingleResults();

    std::vector<double> range_values_ref;
    std::vector<double> range_values_tst;

    for (const auto& single_result : single_results)
    {
        const SinglePositionRadarRange* single_radar_range = dynamic_cast<const SinglePositionRadarRange*>(single_result.get());
        traced_assert(single_radar_range);

        const auto& values_ref = single_radar_range->rangeValuesRef();
        const auto& values_tst = single_radar_range->rangeValuesTst();

        range_values_ref.insert(range_values_ref.end(), values_ref.begin(), values_ref.end());
        range_values_tst.insert(range_values_tst.end(), values_tst.begin(), values_tst.end());
    }

    traced_assert(accumulator_.numValues() == range_values_ref.size() && range_values_ref.size() == range_values_tst.size());

    auto model = EvaluationRequirement::RadarHelpers::rangeErrorModel(range_values_ref, range_values_tst);

    range_gain_ = model.gain_m_per_nm;
    range_bias_ = model.bias_m;

    return rangeStatisticValue(requirement_, accumulator_.stddev(), model);
}

}
