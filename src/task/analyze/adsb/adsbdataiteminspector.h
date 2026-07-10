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

class ADSBDataItemInspectorSettings : public InspectorSettingsBase
{
public:
    ADSBDataItemInspectorSettings(nlohmann::json& config_json, Configurable* parent);
    ~ADSBDataItemInspectorSettings() override = default;

    std::string inspectorClassName() const override { return "ADSBDataItemInspector"; }

    /// Per-CAT enable flag. CATs not present in the map default to enabled.
    nlohmann::json included_cats_;  // cat (uint, e.g. "021") -> bool

    bool catIncluded(unsigned int cat) const;
    void setCatIncluded(unsigned int cat, bool value);
};

class ADSBDataItemInspector : public DataSourceInspectorBase
{
public:
    ADSBDataItemInspector(AnalyzeDataSourceTask& task, ADSBDataItemInspectorSettings& settings);
    ~ADSBDataItemInspector() override = default;

    std::string className() const override { return "ADSBDataItemInspector"; }
    std::string name()      const override { return "Data Item Analysis"; }
    std::string dsType()    const override { return "ADSB"; }
    std::string description() const override
    {
        return "Per-data-source ASTERIX data-item summary for ADS-B (CAT021), drawn "
               "from the cumulative import probe info (count, min, max).";
    }

    bool requiresReferenceTrajectory() const override { return false; }
    bool requiresLoadedDataset()       const override { return false; }

    void writeReport(ResultReport::Section& root) override;
};
