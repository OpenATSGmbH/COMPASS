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

#include "eval/requirement/detection/detectionconfig.h"
#include "eval/requirement/detection/detection.h"
#include "eval/requirement/detection/detectionconfigwidget.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "comparisontype.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;

namespace EvaluationRequirement
{

DetectionConfig::DetectionConfig(
        const std::string& class_id, const std::string& instance_id,
        Group& group, EvaluationStandard& standard, EvaluationCalculator& calculator)
    : ProbabilityBaseConfig(class_id, instance_id, group, standard, calculator)
{
    registerParameter("update_interval", &update_interval_s_, 1.0f);

    registerParameter("use_min_gap_length", &use_min_gap_length_, false);
    registerParameter("min_gap_length_s", &min_gap_length_s_, 3.0f);

    registerParameter("use_max_gap_length", &use_max_gap_length_, false);
    registerParameter("max_gap_length_s", &max_gap_length_s_, 5.0f);

    registerParameter("invert_prob", &invert_prob_, false);

    registerParameter("use_miss_tolerance", &use_miss_tolerance_, false);
    registerParameter("miss_tolerance", &miss_tolerance_s_, 0.01f);

    registerParameter("hold_for_any_target", &hold_for_any_target_, false);

    registerParameter("ignore_primary_only", &ignore_primary_only_, false);
}

DetectionConfig::~DetectionConfig()
{
}

std::shared_ptr<Base> DetectionConfig::createRequirement()
{
    shared_ptr<Detection> req = make_shared<Detection>(
                name_, short_name_, group_.name(), prob_, prob_check_type_, calculator_, update_interval_s_,
                use_min_gap_length_, min_gap_length_s_, use_max_gap_length_, max_gap_length_s_, invert_prob_,
                use_miss_tolerance_, miss_tolerance_s_, hold_for_any_target_, ignore_primary_only_);

    return req;
}

float DetectionConfig::updateInterval() const
{
    return update_interval_s_;
}

void DetectionConfig::updateInterval(float value)
{
    update_interval_s_ = value;
}

bool DetectionConfig::invertProb() const
{
    return invert_prob_;
}

void DetectionConfig::invertProb(bool value)
{
    invert_prob_ = value;
}

bool DetectionConfig::useMissTolerance() const
{
    return use_miss_tolerance_;
}

void DetectionConfig::useMissTolerance(bool value)
{
    use_miss_tolerance_ = value;
}

float DetectionConfig::missTolerance() const
{
    return miss_tolerance_s_;
}

void DetectionConfig::missTolerance(float value)
{
    miss_tolerance_s_ = value;
}

bool DetectionConfig::useMinGapLength() const
{
    return use_min_gap_length_;
}

void DetectionConfig::useMinGapLength(bool value)
{
    use_min_gap_length_ = value;
}

float DetectionConfig::minGapLength() const
{
    return min_gap_length_s_;
}

void DetectionConfig::minGapLength(float value)
{
    min_gap_length_s_ = value;
}

bool DetectionConfig::useMaxGapLength() const
{
    return use_max_gap_length_;
}

void DetectionConfig::useMaxGapLength(bool value)
{
    use_max_gap_length_ = value;
}

float DetectionConfig::maxGapLength() const
{
    return max_gap_length_s_;
}

void DetectionConfig::maxGapLength(float value)
{
    max_gap_length_s_ = value;
}

void DetectionConfig::ignorePrimaryOnly(bool value)
{
    ignore_primary_only_ = value;
}

bool DetectionConfig::ignorePrimaryOnly() const
{
    return ignore_primary_only_;;
}

BaseConfigWidget* DetectionConfig::createWidget()
{
    return new DetectionConfigWidget(*this);
}

void DetectionConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:"+group_.name()+":"+name_);

    auto& table = section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    table.addRow({"Probability [1]", "Probability of detection or miss (inverted probability)",
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Update Interval [s]", "",
                  update_interval_s_});

    table.addRow({"Use Minimum Gap Length", "If minimum gap length should be used",
                  String::boolToString(use_min_gap_length_)});
    table.addRow({"Minimum Gap Length [s]", "Minimum gap length to be considered",
                  min_gap_length_s_});

    table.addRow({"Use Maximum Gap Length", "If maximum gap length should be used",
                  String::boolToString(use_max_gap_length_)});
    table.addRow({"Maximum Gap Length [s]", "Maximum gap length to be considered",
                  max_gap_length_s_});

    table.addRow({"Invert Probability", "If calculated probability should be inverted",
                  comment_.c_str()});

    table.addRow({"Use Miss Tolerance", "If miss tolerance should be used",
                  String::boolToString(use_miss_tolerance_)});
    table.addRow({"Miss Tolerance [s]", "Acceptable time delta for miss detection",
                  miss_tolerance_s_});

    table.addRow({"Hold for any Target", "Must hold for any target (every single targets)",
                  String::boolToString(hold_for_any_target_)});

    table.addRow({"Ignore Primary Only",
                  "Requirement result is ignored if target is primary only (has no"
                  " secondary attributes)",
                  String::boolToString(ignore_primary_only_)});                  
}

bool DetectionConfig::holdForAnyTarget() const
{
    return hold_for_any_target_;
}

void DetectionConfig::holdForAnyTarget(bool value)
{
    hold_for_any_target_ = value;
}

}
