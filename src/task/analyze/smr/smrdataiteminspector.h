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

#include "datasourceinspectorbase.h"
#include "inspectorsettingsbase.h"

#include "json_fwd.hpp"

/**
 * Settings of the SMR data-item inspector.
 */
class SMRDataItemInspectorSettings : public InspectorSettingsBase
{
public:
    SMRDataItemInspectorSettings(nlohmann::json& config_json, Configurable* parent);
    ~SMRDataItemInspectorSettings() override = default;

    std::string inspectorClassName() const override { return "SMRDataItemInspector"; }

    /// Per-CAT enable flag. CATs not present in the map default to enabled.
    nlohmann::json included_cats_;  // cat (uint, e.g. "10") -> bool

    bool catIncluded(unsigned int cat) const;
    void setCatIncluded(unsigned int cat, bool value);
};

/**
 * SMR Feature 1: cumulative per-(DS, CAT, item) data-item tables for CAT010,
 * plus a check of the target report fields EUROCAE ED-116 section 3.4.10
 * requires (message type, data source identifier, target report descriptor,
 * time of day, position, target size and orientation, system status).
 *
 * Report-only: works from the cumulative ASTERIX import info held by the
 * DBContextManager, no dataset load and no Reference Trajectory needed. See
 * experimental_src/analysis/readme_analysis_smr.md (Feature 1).
 */
class SMRDataItemInspector : public DataSourceInspectorBase
{
public:
    SMRDataItemInspector(AnalyzeDataSourceTask& task, SMRDataItemInspectorSettings& settings);
    ~SMRDataItemInspector() override = default;

    std::string className() const override { return "SMRDataItemInspector"; }
    std::string name()      const override { return "Data Item Analysis"; }
    std::string dsType()    const override { return "SMR"; }
    std::string description() const override
    {
        return "Per-data-source ASTERIX data-item summary for SMR (CAT010), drawn from "
               "the cumulative import probe info, with a check of the target report "
               "fields EUROCAE ED-116 requires.";
    }

    bool requiresReferenceTrajectory() const override { return false; }
    bool requiresLoadedDataset()       const override { return false; }

    void writeReport(ResultReport::Section& root) override;
};
