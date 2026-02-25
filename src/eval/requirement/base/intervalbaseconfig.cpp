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

#include "intervalbaseconfig.h"

#include "eval/requirement/group.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "util/stringconv.h"

namespace EvaluationRequirement 
{

/**
*/
IntervalBaseConfig::IntervalBaseConfig(nlohmann::json& config,
                                       Group* parent)
:   ProbabilityBaseConfig(config, parent)
{
    registerParameter("update_interval", &update_interval_s_, 1.0f);

    registerParameter("use_min_gap_length", &use_min_gap_length_, false);
    registerParameter("min_gap_length_s", &min_gap_length_s_, 3.0f);

    registerParameter("use_max_gap_length", &use_max_gap_length_, false);
    registerParameter("max_gap_length_s", &max_gap_length_s_, 5.0f);

    registerParameter("use_miss_tolerance", &use_miss_tolerance_, false);
    registerParameter("miss_tolerance", &miss_tolerance_s_, 0.01f);

    registerParameter("hold_for_any_target", &hold_for_any_target_, false);
}

/**
*/
void IntervalBaseConfig::configure(uint32_t flags)
{
    config_flags_ = flags;
}

/**
*/
uint32_t IntervalBaseConfig::configFlags() const
{
    return config_flags_;
}

/**
*/
BaseConfigWidget* IntervalBaseConfig::createWidget()
{
    return createWidget_impl();
}

/**
*/
void IntervalBaseConfig::addToReport (std::shared_ptr<ResultReport::Report> report)
{
    auto& section = report->getSection("Appendix:Requirements:" + group_.name() + ":" + name_);

    section.addTable("req_table", 3, {"Name", "Comment", "Value"}, false);

    auto& table = section.getTable("req_table");

    table.addRow({"Probability [%]", probabilityDescription(),
                  roundf(prob_ * 10000.0) / 100.0});
    table.addRow({"Probability Check Type", "",
                  comparisonTypeString(prob_check_type_)});

    table.addRow({"Update Interval [s]", "",
                  update_interval_s_});

    if (config_flags_ & ConfigFlags::UseMinGapLen)
    {
        table.addRow({"Use Minimum Gap Length", "If minimum gap length should be used",
                      Utils::String::boolToString(use_min_gap_length_)});
        table.addRow({"Minimum Gap Length [s]", "Minimum gap length to be considered",
                      min_gap_length_s_});
    }

    if (config_flags_ & ConfigFlags::UseMaxGapLen)
    {
        table.addRow({"Use Maximum Gap Length", "If maximum gap length should be used",
                    Utils::String::boolToString(use_max_gap_length_)});
        table.addRow({"Maximum Gap Length [s]", "Maximum gap length to be considered",
                    max_gap_length_s_});
    }

    if (config_flags_ & ConfigFlags::UseMissTol)
    {
        table.addRow({"Use Miss Tolerance", "If miss tolerance should be used",
                      Utils::String::boolToString(use_miss_tolerance_)});
        table.addRow({"Miss Tolerance [s]", "Acceptable time delta for miss detection",
                      miss_tolerance_s_});
    }

    if (config_flags_ & ConfigFlags::UseAnyTarget)
    {
        table.addRow({"Must hold for any Target", "Must hold for any target (every single target)",
                    Utils::String::boolToString(hold_for_any_target_)});
    }

    addCustomTableEntries(table);
}

/**
*/
float IntervalBaseConfig::updateInterval() const
{
    return update_interval_s_;
}

/**
*/
void IntervalBaseConfig::updateInterval(float value)
{
    update_interval_s_ = value;
}

/**
*/
bool IntervalBaseConfig::useMissTolerance() const
{
    return use_miss_tolerance_;
}

/**
*/
void IntervalBaseConfig::useMissTolerance(bool value)
{
    use_miss_tolerance_ = value;
}

/**
*/
bool IntervalBaseConfig::useMinGapLength() const
{
    return use_min_gap_length_;
}

/**
*/
void IntervalBaseConfig::useMinGapLength(bool value)
{
    use_min_gap_length_ = value;
}

/**
*/
float IntervalBaseConfig::minGapLength() const
{
    return min_gap_length_s_;
}

/**
*/
void IntervalBaseConfig::minGapLength(float value)
{
    min_gap_length_s_ = value;
}

/**
*/
bool IntervalBaseConfig::useMaxGapLength() const
{
    return use_max_gap_length_;
}

/**
*/
void IntervalBaseConfig::useMaxGapLength(bool value)
{
    use_max_gap_length_ = value;
}

/**
*/
float IntervalBaseConfig::maxGapLength() const
{
    return max_gap_length_s_;
}

/**
*/
void IntervalBaseConfig::maxGapLength(float value)
{
    max_gap_length_s_ = value;
}

/**
*/
float IntervalBaseConfig::missTolerance() const
{
    return miss_tolerance_s_;
}

/**
*/
void IntervalBaseConfig::missTolerance(float value)
{
    miss_tolerance_s_ = value;
}

/**
*/
bool IntervalBaseConfig::holdForAnyTarget() const
{
    return hold_for_any_target_;
}

/**
*/
void IntervalBaseConfig::holdForAnyTarget(bool value)
{
    hold_for_any_target_ = value;
}

} // namespace EvaluationRequirement
