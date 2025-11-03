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

#include "configurable.h"
#include "targetmodel.h"

class EvaluationManager;


class EvaluationTargetFilter : public Configurable
{
  public:
    EvaluationTargetFilter(const std::string& class_id, const std::string& instance_id, EvaluationManager& eval_manager);
    virtual ~EvaluationTargetFilter() {}

    virtual void generateSubConfigurable(const std::string& class_id, const std::string& instance_id) override {};

    void setUse(dbContent::TargetCache& target_data);

    bool removeShortTargets() const;
    void removeShortTargets(bool value);

    unsigned int removeShortTargetsMinUpdates() const;
    void removeShortTargetsMinUpdates(unsigned int value);

    double removeShortTargetsMinDuration() const;
    void removeShortTargetsMinDuration(double value);

    bool removePsrOnlyTargets() const;
    void removePsrOnlyTargets(bool value);

    bool filterModeACodes() const;
    void filterModeACodes(bool value);
    bool filterModeACodeBlacklist() const;
    void filterModeACodeBlacklist(bool value);

    bool removeModeCValues() const;
    void removeModeCValues(bool value);

    float removeModeCMinValue() const;
    void removeModeCMinValue(float value);

    std::string filterModeACodeValues() const;
    std::set<std::pair<int,int>> filterModeACodeData() const; // single ma,-1 or range ma1,ma2
    void filterModeACodeValues(const std::string& value);

    bool filterTargetAddresses() const;
    void filterTargetAddresses(bool value);
    bool filterTargetAddressesBlacklist() const;
    void filterTargetAddressesBlacklist(bool value);

    std::string filterTargetAddressValues() const;
    std::set<unsigned int> filterTargetAddressData() const;
    void filterTargetAddressValues(const std::string& value);

    bool removeModeACOnlys() const;
    void removeModeACOnlys(bool value);

    bool removeNotDetectedDBContents() const;
    void removeNotDetectedDBContents(bool value);

    bool removeNotDetectedDBContent(const std::string& dbcontent_name) const;
    void removeNotDetectedDBContents(const std::string& dbcontent_name, bool value);

  protected:
    EvaluationManager& eval_manager_;

    // utn use filter stuff

    bool remove_short_targets_ {true};
    unsigned int remove_short_targets_min_updates_ {10};
    double remove_short_targets_min_duration_ {60.0};

    bool remove_psr_only_targets_ {true};
    bool remove_modeac_onlys_ {false};

    bool filter_mode_a_codes_{false};
    bool filter_mode_a_code_blacklist_{true};
    std::string filter_mode_a_code_values_;

    bool remove_mode_c_values_{false};
    float remove_mode_c_min_value_;

    bool filter_target_addresses_{false};
    bool filter_target_addresses_blacklist_{true};
    std::string filter_target_address_values_;

    bool remove_not_detected_dbconts_{false};
    nlohmann::json remove_not_detected_dbcont_values_;

    virtual void checkSubConfigurables() override {};

};

