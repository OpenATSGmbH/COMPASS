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

#include "evaluationtargetfilter.h"
#include "evaluationmanager.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "util/timeconv.h"

#include "json.hpp"

using namespace nlohmann;
using namespace std;
using namespace Utils;

EvaluationTargetFilter::EvaluationTargetFilter(const std::string& class_id, const std::string& instance_id, EvaluationManager& eval_manager)
    : Configurable(class_id, instance_id, &eval_manager), eval_manager_(eval_manager)
{
    // shorts
    registerParameter("remove_short_targets", &remove_short_targets_, true);
    registerParameter("remove_short_targets_min_updates", &remove_short_targets_min_updates_, 10u);
    registerParameter("remove_short_targets_min_duration", &remove_short_targets_min_duration_, 60.0);
    // psr
    registerParameter("remove_psr_only_targets", &remove_psr_only_targets_, true);
    // ma
    registerParameter("remove_modeac_onlys", &remove_modeac_onlys_, false);
    registerParameter("filter_mode_a_codes", &filter_mode_a_codes_, false);
    registerParameter("filter_mode_a_code_blacklist", &filter_mode_a_code_blacklist_, true);
    registerParameter("filter_mode_a_code_values", &filter_mode_a_code_values_, std::string("7000,7777"));
    // mc
    registerParameter("remove_mode_c_values", &remove_mode_c_values_, false);
    registerParameter("remove_mode_c_min_value", &remove_mode_c_min_value_, 11000.0f);
    // ta
    registerParameter("filter_target_addresses", &filter_target_addresses_, false);
    registerParameter("filter_target_addresses_blacklist", &filter_target_addresses_blacklist_, true);
    registerParameter("filter_target_address_values", &filter_target_address_values_, std::string());
    // dbcont
    registerParameter("remove_not_detected_dbconts", &remove_not_detected_dbconts_, false);
    registerParameter("remove_not_detected_dbcont_values", &remove_not_detected_dbcont_values_, json::object());

    createSubConfigurables();

}

void EvaluationTargetFilter::setUse(dbContent::TargetCache& target_data)
{
    using namespace boost::posix_time;

    bool use;
    string comment;

    std::set<std::pair<int,int>> remove_mode_as = filterModeACodeData();
    std::set<unsigned int> remove_tas = filterTargetAddressData();

    bool tmp_match;

    time_duration short_duration = Time::partialSeconds(remove_short_targets_min_duration_);

    auto& dbcont_man = COMPASS::instance().dbContentManager();

    for (auto target_it = target_data.begin(); target_it != target_data.end(); ++target_it)
    {
        if (!target_it->useInEval())
            continue;

        use = true; // must be true here
        comment = "";

        if (remove_short_targets_
            && (target_it->numUpdates() < remove_short_targets_min_updates_
                || target_it->timeDuration() < short_duration))
        {
            use = false;
            comment = "Short duration";
        }

        if (use && remove_psr_only_targets_ && target_it->isPrimaryOnly())
        {
            use = false;
            comment = "Primary only";
        }

        if (use && remove_modeac_onlys_ && target_it->isModeACOnly())
        {
            use = false;
            comment = "Mode A/C only";
        }

        if (use && filter_mode_a_codes_)
        {
            tmp_match = false;

            for (auto t_ma : target_it->modeACodes())
            {
                for (auto& r_ma_p : remove_mode_as)
                {
                    traced_assert(r_ma_p.first >= 0);
                    if (r_ma_p.second == -1) // single
                    {
                        tmp_match |= (t_ma == static_cast<unsigned int>(r_ma_p.first));
                    }
                    else // pair
                    {
                        traced_assert(r_ma_p.second >= 0);
                        tmp_match |= ((t_ma >= static_cast<unsigned int>(r_ma_p.first) 
                        && t_ma <= static_cast<unsigned int>(r_ma_p.second)));
                    }
                }

                if (tmp_match)
                    break;
            }

            if (filter_mode_a_code_blacklist_)
            {
                if (tmp_match) // disable if match
                {
                    use = false;
                    comment = "Mode A";
                }
            }
            else // whitelist
            {
                if (!tmp_match) // disable if not match
                {
                    use = false;
                    comment = "Mode A";
                }
            }
        }

        if (use && remove_mode_c_values_)
        {
            if (!target_it->hasModeC())
            {
                use = false;
                comment = "Mode C not existing";
            }
            else if (target_it->modeCMax() < remove_mode_c_min_value_)
            {
                use = false;
                comment = "Max Mode C too low";
            }
        }

        if (use && filter_target_addresses_)
        {
            tmp_match = false;

            for (auto ta_it : target_it->aircraftAddresses())
            {
                tmp_match = remove_tas.count(ta_it);

                if (tmp_match)
                    break;
            }

            if (filter_target_addresses_blacklist_)
            {
                if (tmp_match) // disable if match
                {
                    use = false;
                    comment = "Target Address";
                }
            }
            else // whitelist
            {
                if (!tmp_match) // disable if not match
                {
                    use = false;
                    comment = "Target Address";
                }
            }
        }

        if (use && remove_not_detected_dbconts_) // prepare associations
        {

            for (auto& dbcont_it : dbcont_man)
            {
                if (remove_not_detected_dbcont_values_.contains(dbcont_it.first)
                    && remove_not_detected_dbcont_values_.at(dbcont_it.first) == true // removed if not detected
                    && target_it->dbContentCount(dbcont_it.first) == 0) // not detected
                {
                    use = false; // remove it
                    comment = "Not Detected in "+dbcont_it.first;
                    break;
                }
            }
        }

        if (!use)
        {
            logdbg << "removing " << target_it->utn_ << " comment '" << comment << "'";

            target_data.modify(target_it, [use](dbContent::Target& p) { p.useInEval(use); });
            target_data.modify(target_it, [comment](dbContent::Target& p) { p.comment(comment); });
        }
    }

    // updates done in TargetModel::setUseByFilter
}

/**
 */
bool EvaluationTargetFilter::removeShortTargets() const
{
    return remove_short_targets_;
}

/**
 */
void EvaluationTargetFilter::removeShortTargets(bool value)
{
    loginf << "value " << value;

    remove_short_targets_ = value;
}

/**
 */
unsigned int EvaluationTargetFilter::removeShortTargetsMinUpdates() const
{
    return remove_short_targets_min_updates_;
}

/**
 */
void EvaluationTargetFilter::removeShortTargetsMinUpdates(unsigned int value)
{
    loginf << "value " << value;

    remove_short_targets_min_updates_ = value;
}

/**
 */
double EvaluationTargetFilter::removeShortTargetsMinDuration() const
{
    return remove_short_targets_min_duration_;
}

/**
 */
void EvaluationTargetFilter::removeShortTargetsMinDuration(double value)
{
    loginf << "value " << value;

    remove_short_targets_min_duration_ = value;
}

/**
 */
bool EvaluationTargetFilter::removePsrOnlyTargets() const
{
    return remove_psr_only_targets_;
}

/**
 */
void EvaluationTargetFilter::removePsrOnlyTargets(bool value)
{
    loginf << "value " << value;

    remove_psr_only_targets_ = value;
}

/**
 */
std::string EvaluationTargetFilter::filterModeACodeValues() const
{
    return filter_mode_a_code_values_;
}

/**
 */
std::set<std::pair<int,int>> EvaluationTargetFilter::filterModeACodeData() const // single ma,-1 or range ma1,ma2
{
    std::set<std::pair<int,int>> data;

    vector<string> parts = String::split(filter_mode_a_code_values_, ',');

    for (auto& part_it : parts)
    {
        if (part_it.find("-") != std::string::npos) // range
        {
            vector<string> sub_parts = String::split(part_it, '-');

            if (sub_parts.size() != 2)
            {
                logwrn << "not able to parse range '" << part_it << "'";
                continue;
            }

            int val1 = String::intFromOctalString(sub_parts.at(0));
            int val2 = String::intFromOctalString(sub_parts.at(1));

            data.insert({val1, val2});
        }
        else // single value
        {
            int val1 = String::intFromOctalString(part_it);
            data.insert({val1, -1});
        }
    }

    return data;
}

/**
 */
void EvaluationTargetFilter::filterModeACodeValues(const std::string& value)
{
    loginf << "value '" << value << "'";

    filter_mode_a_code_values_ = value;
}

/**
 */
std::string EvaluationTargetFilter::filterTargetAddressValues() const
{
    return filter_target_address_values_;
}

/**
 */
std::set<unsigned int> EvaluationTargetFilter::filterTargetAddressData() const
{
    std::set<unsigned int>  data;

    vector<string> parts = String::split(filter_target_address_values_, ',');

    for (auto& part_it : parts)
    {
        int val1 = String::intFromHexString(part_it);
        data.insert(val1);
    }

    return data;
}

/**
 */
void EvaluationTargetFilter::filterTargetAddressValues(const std::string& value)
{
    loginf << "value '" << value << "'";

    filter_target_address_values_ = value;
}

/**
 */
bool EvaluationTargetFilter::removeModeACOnlys() const
{
    return remove_modeac_onlys_;
}

/**
 */
void EvaluationTargetFilter::removeModeACOnlys(bool value)
{
    loginf << "value " << value;
    remove_modeac_onlys_ = value;
}

/**
 */
bool EvaluationTargetFilter::removeNotDetectedDBContents() const
{
    return remove_not_detected_dbconts_;
}

/**
 */
void EvaluationTargetFilter::removeNotDetectedDBContents(bool value)
{
    loginf << "value " << value;

    remove_not_detected_dbconts_ = value;
}

/**
 */
bool EvaluationTargetFilter::removeNotDetectedDBContent(const std::string& dbcontent_name) const
{
    if (!remove_not_detected_dbcont_values_.contains(dbcontent_name))
        return false;

    return remove_not_detected_dbcont_values_.at(dbcontent_name);
}

/**
 */
void EvaluationTargetFilter::removeNotDetectedDBContents(const std::string& dbcontent_name, bool value)
{
    loginf << "dbcont " << dbcontent_name << " value " << value;

    remove_not_detected_dbcont_values_[dbcontent_name] = value;
}

/**
 */
bool EvaluationTargetFilter::filterTargetAddressesBlacklist() const
{
    return filter_target_addresses_blacklist_;
}

/**
 */
void EvaluationTargetFilter::filterTargetAddressesBlacklist(bool value)
{
    loginf << "value " << value;

    filter_target_addresses_blacklist_ = value;
}

/**
 */
bool EvaluationTargetFilter::filterModeACodeBlacklist() const
{
    return filter_mode_a_code_blacklist_;
}

/**
 */
void EvaluationTargetFilter::filterModeACodeBlacklist(bool value)
{
    loginf << "value " << value;

    filter_mode_a_code_blacklist_ = value;
}

/**
 */
bool EvaluationTargetFilter::removeModeCValues() const
{
    return remove_mode_c_values_;
}

/**
 */
void EvaluationTargetFilter::removeModeCValues(bool value)
{
    loginf << "value " << value;

    remove_mode_c_values_ = value;
}

/**
 */
float EvaluationTargetFilter::removeModeCMinValue() const
{
    return remove_mode_c_min_value_;
}

/**
 */
void EvaluationTargetFilter::removeModeCMinValue(float value)
{
    loginf << "value " << value;
    remove_mode_c_min_value_ = value;
}

/**
 */
bool EvaluationTargetFilter::filterTargetAddresses() const
{
    return filter_target_addresses_;
}

/**
 */
void EvaluationTargetFilter::filterTargetAddresses(bool value)
{
    loginf << "value " << value;

    filter_target_addresses_ = value;
}

/**
 */
bool EvaluationTargetFilter::filterModeACodes() const
{
    return filter_mode_a_codes_;
}

/**
 */
void EvaluationTargetFilter::filterModeACodes(bool value)
{
    loginf << "value " << value;

    filter_mode_a_codes_ = value;
}
