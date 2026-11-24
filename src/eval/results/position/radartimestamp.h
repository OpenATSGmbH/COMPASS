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

#include "results/position/positionbase.h"

namespace EvaluationRequirementResult
{

/**
*/
class SinglePositionRadarTimeStamp : public SinglePositionValueBase
{
public:
    SinglePositionRadarTimeStamp(const std::string& result_id,
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
                                 const std::vector<double>& speed_values);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;

    const std::vector<double>& alongValues() const { return along_values_; }
    const std::vector<double>& speedValues() const { return speed_values_; }

protected:
    virtual OverviewLabels overviewLabels() const override { return { "Time Stamp Error", "s", false }; }

    virtual boost::optional<double> computeFinalResultValue() const override;

    virtual std::vector<std::string> targetTableHeadersCustom() const override;
    virtual nlohmann::json::array_t targetTableValuesCustom() const override;
    virtual std::string targetTableCustomSortColumn() const override { return "TSE"; }
    virtual Qt::SortOrder targetTableSortOrder() const override { return Qt::SortOrder::DescendingOrder; }
    virtual std::vector<TargetInfo> targetInfos() const override;

private:
    mutable nlohmann::json time_stamp_error_;

    std::vector<double> along_values_;
    std::vector<double> speed_values_;
};

/**
*/
class JoinedPositionRadarTimeStamp : public JoinedPositionValueBase
{
public:
    JoinedPositionRadarTimeStamp(const std::string& result_id,
                                 std::shared_ptr<EvaluationRequirement::Base> requirement,
                                 const SectorLayer& sector_layer,
                                 EvaluationCalculator& calculator);
protected:
    virtual OverviewLabels overviewLabels() const override { return { "Time Stamp Error", "s", false }; }

    virtual boost::optional<double> computeFinalResultValue() const override;

    virtual std::vector<SectorInfo> sectorInfos() const override;

private:
    mutable QVariant time_stamp_error_;
};

}
