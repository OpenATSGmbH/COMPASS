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


#include "results/position/radartimestamp.h"
#include "requirement/position/radar_helpers.h"
#include "stringconv.h"
#include "logger.h"
#include "traced_assert.h"

#include <cmath>

using namespace Utils;

namespace EvaluationRequirementResult
{

/**********************************************************************************************
 * SinglePositionRadarTimeStamp
 **********************************************************************************************/

/**
*/
SinglePositionRadarTimeStamp::SinglePositionRadarTimeStamp(const std::string& result_id,
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
                                                           const std::vector<double>& along_values,
                                                           const std::vector<double>& speed_values)
:   SinglePositionValueBase("SinglePositionRadarTimeStamp", result_id, requirement, sector_layer, utn, target, calculator, details,
                            num_pos, num_no_ref, num_pos_outside, num_pos_inside, num_ref_inaccurate, num_comp_passed, num_comp_failed)
,   along_values_(along_values)
,   speed_values_(speed_values)
{
    traced_assert(along_values_.size() == speed_values_.size());

    updateResult();
}

/**
 */
void SinglePositionRadarTimeStamp::addDetailCodeTexts(ReportTableDefinition& def) const
{
    DetailCode::addTypeText(def, OffsetInvalid, "Offset not calculable");
    DetailCode::addTypeText(def, SpeedInvalid, "Reference speed not usable");
}

/**
*/
std::shared_ptr<Joined> SinglePositionRadarTimeStamp::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedPositionRadarTimeStamp> (result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
std::vector<std::string> SinglePositionRadarTimeStamp::targetTableHeadersCustom() const
{
    return { "DMin", "DMax", "DAvg", "DSDev" };
}

/**
*/
nlohmann::json::array_t SinglePositionRadarTimeStamp::targetTableValuesCustom() const
{
    return { formatValue(accumulator_.min()),
             formatValue(accumulator_.max()),
             formatValue(accumulator_.mean()),
             formatValue(accumulator_.stddev()) };
}

/**
*/
std::vector<Single::TargetInfo> SinglePositionRadarTimeStamp::targetInfos() const
{
    std::vector<Single::TargetInfo> infos =
        { { "#Pos [1]"       , "Number of updates"                        , num_pos_                           },
          { "#NoRef [1]"     , "Number of updates w/o reference positions", num_no_ref_                        },
          { "#PosOutside [1]", "Number of updates outside sector"         , num_pos_outside_                   },
          { "#PosInside [1]" , "Number of updates inside sector"          , num_pos_inside_                    },
          { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_   },
          { "#RefPosIn [%]"  , "Percentage of updates with inaccurate reference position"  , String::percentToStringProtected(num_ref_inaccurate_, num_pos_inside_, 2).c_str()},
          { "#Upd [1]"       , "Number of updates used for the time stamp error", num_passed_                  },
          { "DMin [s]"       , "Minimum of time offset"                   , formatValue(accumulator_.min())    },
          { "DMax [s]"       , "Maximum of time offset"                   , formatValue(accumulator_.max())    },
          { "DAvg [s]"       , "Average of time offset"                   , formatValue(accumulator_.mean())   },
          { "DSDev [s]"      , "Standard Deviation of time offset"        , formatValue(accumulator_.stddev()) } };

    if (!time_stamp_error_.is_null())
        infos.push_back({ "Time Stamp Error [s]", "Time stamp error (least squares estimation, positive = late)", formatValue(time_stamp_error_.get<double>(), 3) });

    return infos;
}

/**
*/
boost::optional<double> SinglePositionRadarTimeStamp::computeFinalResultValue() const
{
    time_stamp_error_ = nlohmann::json();

    if (along_values_.empty())
        return {};

    double error = EvaluationRequirement::RadarHelpers::timeStampError(along_values_, speed_values_);

    time_stamp_error_ = error;

    return std::fabs(error);
}

/**********************************************************************************************
 * JoinedPositionRadarTimeStamp
 **********************************************************************************************/

/**
*/
JoinedPositionRadarTimeStamp::JoinedPositionRadarTimeStamp(const std::string& result_id,
                                                           std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                           const SectorLayer& sector_layer,
                                                           EvaluationCalculator& calculator)
:   JoinedPositionValueBase("JoinedPositionRadarTimeStamp", result_id, requirement, sector_layer, calculator, "time offset")
{
}

/**
*/
std::vector<Joined::SectorInfo> JoinedPositionRadarTimeStamp::sectorInfos() const
{
    std::vector<Joined::SectorInfo> infos =
        { { "#Pos [1]"       , "Number of updates"                        , num_pos_                           },
          { "#NoRef [1]"     , "Number of updates w/o reference positions", num_no_ref_                        },
          { "#PosOutside [1]", "Number of updates outside sector"         , num_pos_outside_                   },
          { "#PosInside [1]" , "Number of updates inside sector"          , num_pos_inside_                    },
          { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_   },
          { "#RefPosIn [%]"  , "Percentage of updates with inaccurate reference position"  , String::percentToStringProtected(num_ref_inaccurate_, num_pos_inside_, 2).c_str()},
          { "#Upd [1]"       , "Number of updates used for the time stamp error", num_passed_                  },
          { "DMin [s]"       , "Minimum of time offset"                   , formatValue(accumulator_.min())    },
          { "DMax [s]"       , "Maximum of time offset"                   , formatValue(accumulator_.max())    },
          { "DAvg [s]"       , "Average of time offset"                   , formatValue(accumulator_.mean())   },
          { "DSDev [s]"      , "Standard Deviation of time offset"        , formatValue(accumulator_.stddev()) } };

    if (time_stamp_error_.isValid())
        infos.push_back({ "Time Stamp Error [s]", "Time stamp error (least squares estimation, positive = late)", formatValue(time_stamp_error_.toDouble(), 3) });

    return infos;
}

/**
*/
boost::optional<double> JoinedPositionRadarTimeStamp::computeFinalResultValue() const
{
    time_stamp_error_ = QVariant();

    std::vector<double> along_values;
    std::vector<double> speed_values;

    for (const auto& single_result : usedSingleResults())
    {
        const SinglePositionRadarTimeStamp* single = dynamic_cast<const SinglePositionRadarTimeStamp*>(single_result.get());
        traced_assert(single);

        along_values.insert(along_values.end(), single->alongValues().begin(), single->alongValues().end());
        speed_values.insert(speed_values.end(), single->speedValues().begin(), single->speedValues().end());
    }

    if (along_values.empty())
        return {};

    double error = EvaluationRequirement::RadarHelpers::timeStampError(along_values, speed_values);

    time_stamp_error_ = error;

    return std::fabs(error);
}

}
