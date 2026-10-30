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

#include "evaluationconstraints.h"
#include "evaluationmanager.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/target/targetmodel.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"
#include "util/timeconv.h"
#include "traced_assert.h"

#include "json.hpp"

#include <QStringList>

const std::string EvaluationConstraints::ExcludedDataSectionHeading    = "Excluded Data";
const std::string EvaluationConstraints::ExcludedTimeWindowsTableName  = "Excluded Time Windows";
const std::string EvaluationConstraints::ExcludedRequirementsTableName = "Excluded Requirements";
const std::string EvaluationConstraints::NoExcludedDataText            = "No excluded data defined.";

/**
 */
void EvaluationConstraints::clear()
{
    global_time_filter_enabled_    = false;
    global_time_window_            = Utils::TimeWindow();
    global_exclusion_time_windows_ = Utils::TimeWindowCollection();
    target_constraints_.clear();
}

/**
 * Takes the current exclusions of the Evaluation manager and the target model.
 */
void EvaluationConstraints::update(EvaluationManager& eval_man)
{
    clear();

    bool load_filter_valid = !eval_man.loadTimestampBegin().is_not_a_date_time() &&
                             !eval_man.loadTimestampEnd().is_not_a_date_time();

    global_time_filter_enabled_    = eval_man.useTimestampFilter();
    global_time_window_            = load_filter_valid ? Utils::TimeWindow(eval_man.loadTimestampBegin(), eval_man.loadTimestampEnd()) : Utils::TimeWindow();
    global_exclusion_time_windows_ = eval_man.excludedTimeWindows();
    target_constraints_            = eval_man.dbContentManager().targetModel()->evaluationConstraints(true);
}

/**
 */
bool EvaluationConstraints::useTarget(unsigned int utn) const
{
    auto c = targetConstraint(utn);

    return !c || c->use_in_eval_;
}

/**
 * Global rule: a timestamp is excluded when the global time filter is on and the timestamp
 * is outside the global time window or inside a global exclusion window.
 */
bool EvaluationConstraints::isTimeStampNotExcluded(const boost::posix_time::ptime& ts) const
{
    // global time filters disabled? => not excluded
    if (!global_time_filter_enabled_)
        return true;

    //check if ts outside of global load window => excluded
    if (global_time_window_.valid() && !global_time_window_.contains(ts))
        return false;

    // check if ts inside global load exclusion windows => excluded
    if (global_exclusion_time_windows_.contains(ts))
        return false;

    // not excluded
    return true;
}

/**
 * Global rule plus the excluded time windows of the target.
 */
bool EvaluationConstraints::isTimeStampNotExcluded(unsigned int utn, const boost::posix_time::ptime& ts) const
{
    if (!isTimeStampNotExcluded(ts))
        return false;

    auto c = targetConstraint(utn);

    return !c || !c->excluded_time_windows_.contains(ts);
}

/**
 */
const dbContent::TargetEvalConstraints* EvaluationConstraints::targetConstraint(unsigned int utn) const
{
    if (target_constraints_.empty())
        return nullptr;

    auto it = target_constraints_.find(utn);
    if (it == target_constraints_.end())
        return nullptr;

    return &it->second;
}

/**
 */
size_t EvaluationConstraints::numExcludedTargets() const
{
    size_t num = 0;

    for (const auto& tc : target_constraints_)
        if (!tc.second.use_in_eval_)
            ++num;

    return num;
}

/**
 */
nlohmann::json EvaluationConstraints::globalTimeWindowAsJSON() const
{
    return global_time_window_.valid() ? global_time_window_.getAsJson() : nlohmann::json();
}

/**
 */
nlohmann::json EvaluationConstraints::globalExclusionTimeWindowsAsJSON() const
{
    return global_exclusion_time_windows_.asJSON();
}

/**
 */
nlohmann::json EvaluationConstraints::targetConstraintsAsJSON() const
{
    nlohmann::json j = nlohmann::json::object();

    for (const auto& tc : target_constraints_)
    {
        if (!tc.second.hasActiveConstraint())
            continue;

        j[ std::to_string(tc.first) ] = tc.second.toJSON();
    }

    return j;
}

/**
 */
void EvaluationConstraints::globalTimeWindowFromJSON(const nlohmann::json& j)
{
    global_time_window_ = Utils::TimeWindow();

    if (j.is_null())
        return;

    global_time_window_.setFrom(j);
}

/**
 */
void EvaluationConstraints::globalExclusionTimeWindowsFromJSON(const nlohmann::json& j)
{
    global_exclusion_time_windows_ = Utils::TimeWindowCollection();

    if (j.is_null())
        return;

    global_exclusion_time_windows_.setFrom(j);
}

/**
 */
void EvaluationConstraints::targetConstraintsFromJSON(const nlohmann::json& j)
{
    target_constraints_.clear();

    if (j.is_null())
        return;

    traced_assert(j.is_object());

    for (const auto& tc_json : j.items())
    {
        QString s = QString::fromStdString(tc_json.key());

        bool ok;
        auto utn = s.toUInt(&ok);
        traced_assert(ok);

        dbContent::TargetEvalConstraints tc;
        ok = tc.fromJSON(tc_json.value());
        traced_assert(ok);

        if (!tc.hasActiveConstraint())
            continue;

        target_constraints_[ utn ] = tc;
    }
}

/**
 */
void EvaluationConstraints::addReportSection(ResultReport::Section& section,
                                             const std::set<unsigned int>* utns,
                                             bool with_excluded_requirements) const
{
    auto targetUsed = [ & ] (unsigned int utn) { return !utns || utns->count(utn) > 0; };

    //check needed number of rows
    size_t num_excluded_tw  = 0;
    size_t num_excluded_req = 0;

    if (global_time_filter_enabled_)
    {
        if (global_time_window_.valid())
            num_excluded_tw += 2;

        for (const auto& w : global_exclusion_time_windows_)
            if (w.valid())
                ++num_excluded_tw;
    }

    for (const auto& tc : target_constraints_)
    {
        if (!targetUsed(tc.first))
            continue;

        for (const auto& w : tc.second.excluded_time_windows_)
            if (w.valid())
                ++num_excluded_tw;

        if (with_excluded_requirements && !tc.second.excluded_requirements_.empty())
            ++num_excluded_req;
    }

    //no content? => add placeholder text
    if (num_excluded_tw == 0 &&
        num_excluded_req == 0)
    {
        auto& txt = section.addText(NoExcludedDataText);
        txt.addText(NoExcludedDataText);

        return;
    }

    //create excluded time window table
    if (num_excluded_tw > 0)
    {
        std::vector<std::string> headings_constraints_ts;
        headings_constraints_ts.push_back("Target");
        headings_constraints_ts.push_back("Type");
        headings_constraints_ts.push_back("Timestamp");
        headings_constraints_ts.push_back("Comment");

        auto& table_constraints_ts = section.addTable(ExcludedTimeWindowsTableName,
                                                      headings_constraints_ts.size(),
                                                      headings_constraints_ts);

        if (global_time_filter_enabled_)
        {
            if (global_time_window_.valid())
            {
                {
                    auto row = nlohmann::json::array();
                    row.push_back("All");
                    row.push_back("Timestamp Begin");
                    row.push_back(Utils::Time::toString(global_time_window_.begin()));
                    row.push_back("");

                    table_constraints_ts.addRow(row);
                }
                {
                    auto row = nlohmann::json::array();
                    row.push_back("All");
                    row.push_back("Timestamp End");
                    row.push_back(Utils::Time::toString(global_time_window_.end()));
                    row.push_back("");

                    table_constraints_ts.addRow(row);
                }
            }

            for (const auto& w : global_exclusion_time_windows_)
            {
                if (!w.valid())
                    continue;

                auto row = nlohmann::json::array();
                row.push_back("All");
                row.push_back("Exclusion Window");
                row.push_back(w.asStr());
                row.push_back(w.comment());

                table_constraints_ts.addRow(row);
            }
        }

        for (const auto& tc : target_constraints_)
        {
            if (!targetUsed(tc.first))
                continue;

            for (const auto& w : tc.second.excluded_time_windows_)
            {
                if (!w.valid())
                    continue;

                auto row = nlohmann::json::array();
                row.push_back("UTN " + std::to_string(tc.first));
                row.push_back("Exclusion Window");
                row.push_back(w.asStr());
                row.push_back(w.comment());

                table_constraints_ts.addRow(row);
            }
        }
    }

    //create excluded requirements table
    if (num_excluded_req > 0)
    {
        std::vector<std::string> headings_constraints_req;
        headings_constraints_req.push_back("Target");
        headings_constraints_req.push_back("Excluded Requirements");

        auto& table_constraints_req = section.addTable(ExcludedRequirementsTableName,
                                                       headings_constraints_req.size(),
                                                       headings_constraints_req);

        for (const auto& tc : target_constraints_)
        {
            if (!targetUsed(tc.first) || tc.second.excluded_requirements_.empty())
                continue;

            QStringList strings;
            for (const auto& s : tc.second.excluded_requirements_)
                strings.push_back(QString::fromStdString(s));

            auto s = strings.join(", ").toStdString();

            auto row = nlohmann::json::array();
            row.push_back("UTN " + std::to_string(tc.first));
            row.push_back(s);

            table_constraints_req.addRow(row);
        }
    }
}
