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

#include "analyzedatasourcetask.h"
#include "dialogs.h"
#include "analyzedatasourcedialog.h"
#include "analyze_commands.h"
#include "analysisdataset.h"
#include "datasourceinspectorbase.h"
#include "inspectorsettingsbase.h"
#include "mlatdataiteminspector.h"
#include "mlatcoverageinspector.h"
#include "adsbdataiteminspector.h"
#include "adsbcoverageinspector.h"
#include "smrdataiteminspector.h"
#include "smrcoverageinspector.h"

#include <array>
#include <cmath>

#if USE_EXPERIMENTAL_SOURCE == true
#include "mlataccuracyinspector.h"
#include "mlatrucoverageinspector.h"
#include "mlatrueffectinspector.h"
#include "adsbaccuracyinspector.h"
#include "smraccuracyinspector.h"
#include "smrunassociatedinspector.h"
#endif

#include "compass.h"
#include "mainwindow.h"
#include "taskmanager.h"
#include "db_context_manager.h"
#include "sectorlayer.h"
#include "data_source.h"
#include "license.h"
#include "licensemanager.h"
#include "logger.h"
#include "configuration.h"
#include "report.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "stringconv.h"
#include "system.h"
#include "taskresult.h"
#include "traced_assert.h"

#include "json.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QCursor>
#include <QLabel>
#include <QMessageBox>
#include <QProgressDialog>
#include <QThread>

#include <atomic>
#include <chrono>
#include <future>

AnalyzeDataSourceTask::AnalyzeDataSourceTask(nlohmann::json& config, TaskManager* parent,
                                             const std::string& ds_type_default,
                                             const std::string& object_name)
    : Task(*parent), Configurable(config, parent)
{
    setObjectName(QString::fromStdString(object_name));

    registerParameter("ds_type", &ds_type_, ds_type_default);
    registerParameter("use_data_sources",           &use_data_sources_,           nlohmann::json::object());
    registerParameter("use_data_sources_lines",     &use_data_sources_lines_,     nlohmann::json::object());
    registerParameter("use_reference_data_sources", &use_reference_data_sources_, nlohmann::json::object());
    registerParameter("inspector_enabled",          &inspector_enabled_,          nlohmann::json::object());
    registerParameter("line_id_tst",                &line_id_tst_,                (unsigned int)0);
    registerParameter("line_id_ref",                &line_id_ref_,                (unsigned int)0);
    registerParameter("custom_report_name",         &custom_report_name_,         std::string());
    registerParameter("cell_size_m",                &cell_size_m_,                cell_size_m_);
    registerParameter("cell_size_ft",               &cell_size_ft_,               cell_size_ft_);
    registerParameter("max_cells_per_axis",         &max_cells_per_axis_,         max_cells_per_axis_);

    registerParameter("use_ground_only",            &use_ground_only_,            use_ground_only_);
    registerParameter("use_min_fl",                 &use_min_fl_,                 use_min_fl_);
    registerParameter("min_fl",                     &min_fl_,                     min_fl_);
    registerParameter("use_max_fl",                 &use_max_fl_,                 use_max_fl_);
    registerParameter("max_fl",                     &max_fl_,                     max_fl_);

    registerParameter("limit_by_sectors",           &limit_by_sectors_,           limit_by_sectors_);
    registerParameter("used_sectors",               &used_sectors_,               nlohmann::json::object());

    tooltip_ = "Analyze a chosen data source from multiple angles "
               "(data items, sensor coverage / PD, position accuracy).";

    createSubConfigurables();

    // The runtime-command registry is process-global; initialize it once from
    // the primary (MLAT) instance to avoid a double registration.
    if (ds_type_ == "MLAT")
        init_analyze_commands(compass());
}

AnalyzeDataSourceTask::~AnalyzeDataSourceTask() = default;

void AnalyzeDataSourceTask::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);

    if (class_name == "MLATDataItemInspectorSettings")
    {
        traced_assert(!data_item_settings_);
        data_item_settings_.reset(new MLATDataItemInspectorSettings(child_json, this));
        traced_assert(data_item_settings_);
    }
    else if (class_name == "MLATCoverageInspectorSettings")
    {
        traced_assert(!coverage_settings_);
        coverage_settings_.reset(new MLATCoverageInspectorSettings(child_json, this));
        traced_assert(coverage_settings_);
    }
    else if (class_name == "ADSBDataItemInspectorSettings")
    {
        traced_assert(!adsb_data_item_settings_);
        adsb_data_item_settings_.reset(new ADSBDataItemInspectorSettings(child_json, this));
        traced_assert(adsb_data_item_settings_);
    }
    else if (class_name == "ADSBCoverageInspectorSettings")
    {
        traced_assert(!adsb_coverage_settings_);
        adsb_coverage_settings_.reset(new ADSBCoverageInspectorSettings(child_json, this));
        traced_assert(adsb_coverage_settings_);
    }
    else if (class_name == "SMRDataItemInspectorSettings")
    {
        traced_assert(!smr_data_item_settings_);
        smr_data_item_settings_.reset(new SMRDataItemInspectorSettings(child_json, this));
        traced_assert(smr_data_item_settings_);
    }
    else if (class_name == "SMRCoverageInspectorSettings")
    {
        traced_assert(!smr_coverage_settings_);
        smr_coverage_settings_.reset(new SMRCoverageInspectorSettings(child_json, this));
        traced_assert(smr_coverage_settings_);
    }
#if USE_EXPERIMENTAL_SOURCE == true
    else if (class_name == "SMRAccuracyInspectorSettings")
    {
        traced_assert(!smr_accuracy_settings_);
        smr_accuracy_settings_.reset(new SMRAccuracyInspectorSettings(child_json, this));
        traced_assert(smr_accuracy_settings_);
    }
    else if (class_name == "SMRUnassociatedInspectorSettings")
    {
        traced_assert(!smr_unassociated_settings_);
        smr_unassociated_settings_.reset(new SMRUnassociatedInspectorSettings(child_json, this));
        traced_assert(smr_unassociated_settings_);
    }
    else if (class_name == "MLATAccuracyInspectorSettings")
    {
        traced_assert(!accuracy_settings_);
        accuracy_settings_.reset(new MLATAccuracyInspectorSettings(child_json, this));
        traced_assert(accuracy_settings_);
    }
    else if (class_name == "MLATRUCoverageInspectorSettings")
    {
        traced_assert(!ru_coverage_settings_);
        ru_coverage_settings_.reset(new MLATRUCoverageInspectorSettings(child_json, this));
        traced_assert(ru_coverage_settings_);
    }
    else if (class_name == "MLATRUEffectInspectorSettings")
    {
        traced_assert(!ru_effect_settings_);
        ru_effect_settings_.reset(new MLATRUEffectInspectorSettings(child_json, this));
        traced_assert(ru_effect_settings_);
    }
    else if (class_name == "ADSBAccuracyInspectorSettings")
    {
        traced_assert(!adsb_accuracy_settings_);
        adsb_accuracy_settings_.reset(new ADSBAccuracyInspectorSettings(child_json, this));
        traced_assert(adsb_accuracy_settings_);
    }
#endif
    else
    {
        throw std::runtime_error("AnalyzeDataSourceTask: generateSubConfigurable: "
                                 "unknown class_name " + class_name);
    }
}

void AnalyzeDataSourceTask::checkSubConfigurables()
{
    // Each task instance is bound to one DSType and only creates that DSType's
    // inspector-settings sub-configs. generateSubConfigurable() above still
    // accepts any known class name so a persisted child of either set loads.
    if (ds_type_ == "ADSB")
    {
        if (!adsb_data_item_settings_)
            generateSubConfigurableFromConfig("ADSBDataItemInspectorSettings",
                                              "ADSBDataItemInspectorSettings0");
        traced_assert(adsb_data_item_settings_);

        if (!adsb_coverage_settings_)
            generateSubConfigurableFromConfig("ADSBCoverageInspectorSettings",
                                              "ADSBCoverageInspectorSettings0");
        traced_assert(adsb_coverage_settings_);

#if USE_EXPERIMENTAL_SOURCE == true
        if (!adsb_accuracy_settings_)
            generateSubConfigurableFromConfig("ADSBAccuracyInspectorSettings",
                                              "ADSBAccuracyInspectorSettings0");
        traced_assert(adsb_accuracy_settings_);
#endif
        return;
    }

    if (ds_type_ == "SMR")
    {
        if (!smr_data_item_settings_)
            generateSubConfigurableFromConfig("SMRDataItemInspectorSettings",
                                              "SMRDataItemInspectorSettings0");
        traced_assert(smr_data_item_settings_);

        if (!smr_coverage_settings_)
            generateSubConfigurableFromConfig("SMRCoverageInspectorSettings",
                                              "SMRCoverageInspectorSettings0");
        traced_assert(smr_coverage_settings_);

#if USE_EXPERIMENTAL_SOURCE == true
        if (!smr_accuracy_settings_)
            generateSubConfigurableFromConfig("SMRAccuracyInspectorSettings",
                                              "SMRAccuracyInspectorSettings0");
        traced_assert(smr_accuracy_settings_);

        if (!smr_unassociated_settings_)
            generateSubConfigurableFromConfig("SMRUnassociatedInspectorSettings",
                                              "SMRUnassociatedInspectorSettings0");
        traced_assert(smr_unassociated_settings_);
#endif
        return;
    }

    if (!data_item_settings_)
        generateSubConfigurableFromConfig("MLATDataItemInspectorSettings",
                                          "MLATDataItemInspectorSettings0");
    traced_assert(data_item_settings_);

    if (!coverage_settings_)
        generateSubConfigurableFromConfig("MLATCoverageInspectorSettings",
                                          "MLATCoverageInspectorSettings0");
    traced_assert(coverage_settings_);

#if USE_EXPERIMENTAL_SOURCE == true
    if (!accuracy_settings_)
        generateSubConfigurableFromConfig("MLATAccuracyInspectorSettings",
                                          "MLATAccuracyInspectorSettings0");
    traced_assert(accuracy_settings_);

    if (!ru_coverage_settings_)
        generateSubConfigurableFromConfig("MLATRUCoverageInspectorSettings",
                                          "MLATRUCoverageInspectorSettings0");
    traced_assert(ru_coverage_settings_);

    if (!ru_effect_settings_)
        generateSubConfigurableFromConfig("MLATRUEffectInspectorSettings",
                                          "MLATRUEffectInspectorSettings0");
    traced_assert(ru_effect_settings_);
#endif
}

void AnalyzeDataSourceTask::initTask()
{
    registerInspectors();

    // A new active context exposes a different data-source set, so any
    // previously-saved custom report name will no longer match - drop it.
    connect(&compass().dbContextManager(),
            &context::DBContextManager::activeContextChangedSignal,
            this, [this]() { resetCustomReportName(); });
}

Result AnalyzeDataSourceTask::applyJSONParameters(const nlohmann::json& params_json)
{
    static const std::string kInspectorSettingsKey = "inspector_settings";

    if (!params_json.is_object())
        return Configurable::applyJSONParameters(params_json);

    if (!params_json.contains(kInspectorSettingsKey))
        return Configurable::applyJSONParameters(params_json);

    const auto& ins_json = params_json.at(kInspectorSettingsKey);
    if (!ins_json.is_object())
        return Result::failed("'" + kInspectorSettingsKey + "' must be a JSON object");

    for (auto it = ins_json.begin(); it != ins_json.end(); ++it)
    {
        const std::string& inspector_class = it.key();
        DataSourceInspectorBase* ins = inspector(inspector_class);
        if (!ins)
            return Result::failed("unknown inspector class '" + inspector_class + "'");
        if (!it.value().is_object())
            return Result::failed("'" + kInspectorSettingsKey + "['" + inspector_class
                                  + "']' must be a JSON object");

        auto res = ins->settings().applyJSONParameters(it.value());
        if (!res.ok())
            return Result::failed("inspector '" + inspector_class + "': " + res.error());
    }

    nlohmann::json rest = params_json;
    rest.erase(kInspectorSettingsKey);
    if (rest.empty())
        return Result::succeeded();

    return Configurable::applyJSONParameters(rest);
}

void AnalyzeDataSourceTask::registerInspectors()
{
    inspectors_.clear();

    if (ds_type_ == "ADSB")
    {
        inspectors_.emplace_back(new ADSBDataItemInspector(*this, *adsb_data_item_settings_));
        inspectors_.emplace_back(new ADSBCoverageInspector(*this, *adsb_coverage_settings_));

#if USE_EXPERIMENTAL_SOURCE == true
        inspectors_.emplace_back(new ADSBAccuracyInspector(*this, *adsb_accuracy_settings_));
#endif
        return;
    }

    if (ds_type_ == "SMR")
    {
        inspectors_.emplace_back(new SMRDataItemInspector(*this, *smr_data_item_settings_));
        inspectors_.emplace_back(new SMRCoverageInspector(*this, *smr_coverage_settings_));

#if USE_EXPERIMENTAL_SOURCE == true
        inspectors_.emplace_back(new SMRAccuracyInspector(*this, *smr_accuracy_settings_));
        inspectors_.emplace_back(new SMRUnassociatedInspector(*this, *smr_unassociated_settings_));
#endif
        return;
    }

    inspectors_.emplace_back(new MLATDataItemInspector(*this, *data_item_settings_));
    inspectors_.emplace_back(new MLATCoverageInspector(*this, *coverage_settings_));

#if USE_EXPERIMENTAL_SOURCE == true
    inspectors_.emplace_back(new MLATAccuracyInspector(*this, *accuracy_settings_));
    inspectors_.emplace_back(new MLATRUCoverageInspector(*this, *ru_coverage_settings_));
    inspectors_.emplace_back(new MLATRUEffectInspector(*this, *ru_effect_settings_));
#endif
}

bool AnalyzeDataSourceTask::useDataSource(unsigned int ds_id) const
{
    auto key = std::to_string(ds_id);
    if (use_data_sources_.contains(key))
        return use_data_sources_.at(key).get<bool>();
    return true;
}

void AnalyzeDataSourceTask::useDataSource(unsigned int ds_id, bool value)
{
    bool changed = useDataSource(ds_id) != value;
    use_data_sources_[std::to_string(ds_id)] = value;
    if (changed)
        resetCustomReportName();
}

bool AnalyzeDataSourceTask::useDataSourceLine(unsigned int ds_id, unsigned int line_id) const
{
    auto ds_key   = std::to_string(ds_id);
    auto line_key = std::to_string(line_id);
    if (use_data_sources_lines_.contains(ds_key)
        && use_data_sources_lines_.at(ds_key).contains(line_key))
        return use_data_sources_lines_.at(ds_key).at(line_key).get<bool>();
    return true;
}

void AnalyzeDataSourceTask::useDataSourceLine(unsigned int ds_id, unsigned int line_id, bool value)
{
    use_data_sources_lines_[std::to_string(ds_id)][std::to_string(line_id)] = value;
}

std::set<unsigned int> AnalyzeDataSourceTask::selectedDataSourceIDs() const
{
    std::set<unsigned int> ids;
    for (auto ds_id : testDataSourceCandidateIDs())
    {
        if (useDataSource(ds_id))
            ids.insert(ds_id);
    }
    return ids;
}

bool AnalyzeDataSourceTask::dataSourceHasCategory(unsigned int ds_id, unsigned int cat) const
{
    const auto& info_map = compass().dbContextManager().asterixInfo();
    auto it = info_map.find(ds_id);
    if (it == info_map.end())
        return false;
    return it->second.count(cat) > 0;
}

bool AnalyzeDataSourceTask::dataSourceMatches(unsigned int ds_id) const
{
    auto& ctx = compass().dbContextManager();
    const auto* ds = ctx.dataSource(ds_id);
    if (!ds)
        return false;

    // An SMR is always a Radar data source in the Data Context. There is no
    // "SMR" DSType string, so the rule is DSType Radar with CAT010 data.
    if (ds_type_ == "SMR")
        return ds->dsType() == "Radar" && dataSourceHasCategory(ds_id, 10);

    return ds->dsType() == ds_type_;
}

std::set<unsigned int> AnalyzeDataSourceTask::testDataSourceCandidateIDs() const
{
    std::set<unsigned int> ids;
    auto& ctx = compass().dbContextManager();
    for (auto ds_id : ctx.allDataSourceIds())
    {
        if (dataSourceMatches(ds_id))
            ids.insert(ds_id);
    }
    return ids;
}

std::string AnalyzeDataSourceTask::testDataSourceRequirementText() const
{
    if (ds_type_ == "SMR")
        return "Radar data sources with CAT010 data";
    return "data sources of type " + ds_type_;
}

void AnalyzeDataSourceTask::setLineIDTst(unsigned int line_id)
{
    line_id_tst_ = line_id;
}

void AnalyzeDataSourceTask::setLineIDRef(unsigned int line_id)
{
    line_id_ref_ = line_id;
}

bool AnalyzeDataSourceTask::useReferenceDataSource(unsigned int ds_id) const
{
    auto key = std::to_string(ds_id);
    if (use_reference_data_sources_.contains(key))
        return use_reference_data_sources_.at(key).get<bool>();
    return true;
}

void AnalyzeDataSourceTask::useReferenceDataSource(unsigned int ds_id, bool value)
{
    use_reference_data_sources_[std::to_string(ds_id)] = value;
}

std::set<unsigned int> AnalyzeDataSourceTask::selectedReferenceDataSourceIDs() const
{
    std::set<unsigned int> ids;
    for (auto ds_id : referenceDataSourceCandidateIDs())
    {
        if (useReferenceDataSource(ds_id))
            ids.insert(ds_id);
    }
    return ids;
}

std::set<unsigned int> AnalyzeDataSourceTask::referenceDataSourceCandidateIDs() const
{
    std::set<unsigned int> ids;
    auto& ctx = compass().dbContextManager();
    auto types = ctx.dsTypes();

    for (auto ds_id : ctx.allDataSourceIds())
    {
        auto it = types.find(ds_id);
        if (it != types.end() && it->second == "RefTraj")
            ids.insert(ds_id);
    }
    return ids;
}

std::string AnalyzeDataSourceTask::reportName() const
{
    if (!custom_report_name_.empty())
        return custom_report_name_;
    return suggestReportName();
}

std::string AnalyzeDataSourceTask::suggestReportName() const
{
    auto ids = selectedDataSourceIDs();
    if (ids.empty())
        return "Analyze " + ds_type_ + " Data Source";

    auto& ctx = compass().dbContextManager();
    const auto* ds = ctx.dataSource(*ids.begin());
    std::string name = ds ? ds->name() : std::to_string(*ids.begin());

    return name + " L" + std::to_string(line_id_tst_ + 1) + " Analysis";
}

void AnalyzeDataSourceTask::setCustomReportName(const std::string& name)
{
    custom_report_name_ = name;
}

void AnalyzeDataSourceTask::resetCustomReportName()
{
    custom_report_name_.clear();
}

void AnalyzeDataSourceTask::setCellSizeMeters(float v)
{
    if (v > 0.0f)
        cell_size_m_ = v;
}

void AnalyzeDataSourceTask::setCellSizeFeet(float v)
{
    if (v > 0.0f)
        cell_size_ft_ = v;
}

void AnalyzeDataSourceTask::setMaxCellsPerAxis(unsigned int v)
{
    if (v >= 10 && v <= 2000)
        max_cells_per_axis_ = v;
}

namespace
{
/// Smallest 1-2-5-ladder integer value (1, 2, 5, 10, 20, 50, 100, ...) that is
/// >= the requested minimum. Used to scale a configured cell size up to fit a
/// per-axis cell-count budget while keeping the result an integer multiple of
/// the original.
unsigned int next125(double min_value)
{
    if (min_value <= 1.0) return 1;
    static const std::array<unsigned int, 3> ladder = { 1, 2, 5 };
    unsigned int decade = 1;
    for (int p = 0; p < 12; ++p, decade *= 10)
    {
        for (unsigned int m : ladder)
        {
            unsigned int candidate = m * decade;
            if (static_cast<double>(candidate) >= min_value)
                return candidate;
        }
    }
    return 1000000000u;
}

constexpr double kEarthRadiusM = 6371000.0;

/// Required cell-size multiplier on a single axis: smallest integer (on the
/// 1-2-5 ladder) such that `extent / (base * mult) <= max_cells`.
unsigned int axisMultiplier(double extent, double base_cell, unsigned int max_cells)
{
    if (extent <= 0.0 || base_cell <= 0.0 || max_cells == 0)
        return 1;
    double min_mult = extent / (base_cell * static_cast<double>(max_cells));
    return next125(min_mult);
}
}

AnalyzeDataSourceTask::CellSizing
AnalyzeDataSourceTask::clampedCellSizes(const AnalysisDataset& dataset) const
{
    CellSizing out{};
    out.cell_size_m            = cell_size_m_;
    out.cell_size_ft           = cell_size_ft_;
    out.horizontal_multiplier  = 1;
    out.vertical_multiplier    = 1;
    out.horizontal_clamped     = false;
    out.vertical_clamped       = false;

    // Horizontal extent: use the larger of the lat/lon extents (in meters).
    double horizontal_extent_m = 0.0;
    if (dataset.minLatitudeDeg() != dataset.maxLatitudeDeg()
        || dataset.minLongitudeDeg() != dataset.maxLongitudeDeg())
    {
        const double dlat_deg = dataset.maxLatitudeDeg() - dataset.minLatitudeDeg();
        const double dlon_deg = dataset.maxLongitudeDeg() - dataset.minLongitudeDeg();
        const double ref_lat  = dataset.centerLatitudeDeg();
        const double cos_lat  = std::cos(ref_lat * M_PI / 180.0);
        const double safe_cos = (std::abs(cos_lat) > 1e-6) ? cos_lat : 1.0;
        const double lat_m = dlat_deg * (M_PI / 180.0) * kEarthRadiusM;
        const double lon_m = dlon_deg * (M_PI / 180.0) * kEarthRadiusM * safe_cos;
        horizontal_extent_m = std::max(lat_m, lon_m);
    }

    out.horizontal_multiplier = axisMultiplier(horizontal_extent_m,
                                               static_cast<double>(cell_size_m_),
                                               max_cells_per_axis_);
    out.cell_size_m           = cell_size_m_ * static_cast<float>(out.horizontal_multiplier);
    out.horizontal_clamped    = (out.horizontal_multiplier > 1);

    // Vertical extent: from the dataset, otherwise default to 50000 ft.
    double vertical_extent_ft = 50000.0;
    if (dataset.hasAltitudeExtent())
        vertical_extent_ft = std::max(0.0, dataset.maxAltitudeFt() - dataset.minAltitudeFt());

    out.vertical_multiplier   = axisMultiplier(vertical_extent_ft,
                                               static_cast<double>(cell_size_ft_),
                                               max_cells_per_axis_);
    out.cell_size_ft          = cell_size_ft_ * static_cast<float>(out.vertical_multiplier);
    out.vertical_clamped      = (out.vertical_multiplier > 1);

    return out;
}

bool AnalyzeDataSourceTask::selectionContainsCAT020() const
{
    auto& ctx = compass().dbContextManager();
    const auto& info_map = ctx.asterixInfo();

    for (auto ds_id : selectedDataSourceIDs())
    {
        auto it = info_map.find(ds_id);
        if (it == info_map.end())
            continue;
        if (it->second.count(20))
            return true;
    }
    return false;
}

bool AnalyzeDataSourceTask::inspectorEnabled(const std::string& inspector_class) const
{
    if (inspector_enabled_.contains(inspector_class))
        return inspector_enabled_.at(inspector_class).get<bool>();
    return true;
}

void AnalyzeDataSourceTask::inspectorEnabled(const std::string& inspector_class, bool value)
{
    inspector_enabled_[inspector_class] = value;
}

DataSourceInspectorBase* AnalyzeDataSourceTask::inspector(const std::string& class_name) const
{
    for (const auto& ins : inspectors_)
        if (ins->className() == class_name)
            return ins.get();
    return nullptr;
}

MLATDataItemInspectorSettings& AnalyzeDataSourceTask::dataItemSettings() const
{
    traced_assert(data_item_settings_);
    return *data_item_settings_;
}

MLATCoverageInspectorSettings& AnalyzeDataSourceTask::coverageSettings() const
{
    traced_assert(coverage_settings_);
    return *coverage_settings_;
}

ADSBDataItemInspectorSettings& AnalyzeDataSourceTask::adsbDataItemSettings() const
{
    traced_assert(adsb_data_item_settings_);
    return *adsb_data_item_settings_;
}

ADSBCoverageInspectorSettings& AnalyzeDataSourceTask::adsbCoverageSettings() const
{
    traced_assert(adsb_coverage_settings_);
    return *adsb_coverage_settings_;
}

SMRDataItemInspectorSettings& AnalyzeDataSourceTask::smrDataItemSettings() const
{
    traced_assert(smr_data_item_settings_);
    return *smr_data_item_settings_;
}

SMRCoverageInspectorSettings& AnalyzeDataSourceTask::smrCoverageSettings() const
{
    traced_assert(smr_coverage_settings_);
    return *smr_coverage_settings_;
}

#if USE_EXPERIMENTAL_SOURCE == true
SMRAccuracyInspectorSettings& AnalyzeDataSourceTask::smrAccuracySettings() const
{
    traced_assert(smr_accuracy_settings_);
    return *smr_accuracy_settings_;
}

SMRUnassociatedInspectorSettings& AnalyzeDataSourceTask::smrUnassociatedSettings() const
{
    traced_assert(smr_unassociated_settings_);
    return *smr_unassociated_settings_;
}

MLATAccuracyInspectorSettings& AnalyzeDataSourceTask::accuracySettings() const
{
    traced_assert(accuracy_settings_);
    return *accuracy_settings_;
}

MLATRUCoverageInspectorSettings& AnalyzeDataSourceTask::ruCoverageSettings() const
{
    traced_assert(ru_coverage_settings_);
    return *ru_coverage_settings_;
}

MLATRUEffectInspectorSettings& AnalyzeDataSourceTask::ruEffectSettings() const
{
    traced_assert(ru_effect_settings_);
    return *ru_effect_settings_;
}

ADSBAccuracyInspectorSettings& AnalyzeDataSourceTask::adsbAccuracySettings() const
{
    traced_assert(adsb_accuracy_settings_);
    return *adsb_accuracy_settings_;
}
#endif

bool AnalyzeDataSourceTask::professionalLicenseEnabled() const
{
    return compass().licenseManager().componentEnabled(
        license::License::Component::ComponentProbIMMReconstructor);
}

bool AnalyzeDataSourceTask::useSector(const std::string& layer_name) const
{
    if (used_sectors_.contains(layer_name))
        return used_sectors_.at(layer_name).get<bool>();
    return true;  // unlisted layers default to used
}

void AnalyzeDataSourceTask::setUseSector(const std::string& layer_name, bool value)
{
    used_sectors_[layer_name] = value;
}

std::vector<std::string> AnalyzeDataSourceTask::selectedSectorLayers() const
{
    std::vector<std::string> out;
    auto& ctx = compass().dbContextManager();
    for (const auto& layer : ctx.sectorLayers())
    {
        if (layer && useSector(layer->name()))
            out.push_back(layer->name());
    }
    return out;
}

std::vector<std::shared_ptr<SectorLayer>> AnalyzeDataSourceTask::scopeSectorLayers() const
{
    std::vector<std::shared_ptr<SectorLayer>> out;
    auto& ctx = compass().dbContextManager();
    for (const auto& layer : ctx.sectorLayers())
    {
        if (layer && useSector(layer->name()))
            out.push_back(layer);
    }
    return out;
}

COMPASS& AnalyzeDataSourceTask::compass() const
{
    return task_manager_.compass();
}

void AnalyzeDataSourceTask::showDialog()
{
    AnalyzeDataSourceDialog dlg(*this, QApplication::activeWindow());
    if (dlg.exec() == QDialog::Rejected)
        return;

    if (!canRun())
    {
        QMessageBox::warning(QApplication::activeWindow(), "Analyze Data Source",
                             "The task cannot run with the current selection.");
        return;
    }

    run();
}

bool AnalyzeDataSourceTask::canRun()
{
    if (selectedDataSourceIDs().empty())
    {
        loginf << "no test data source selected";
        return false;
    }

    if (selectedReferenceDataSourceIDs().empty())
    {
        loginf << "no reference data source selected";
        return false;
    }

    bool any_enabled = false;
    for (const auto& ins : inspectors_)
    {
        if (!inspectorEnabled(ins->className()))
        {
            loginf << "inspector '" << ins->className() << "' not enabled";
            continue;
        }

        std::string reason;
        if (!ins->prerequisitesMet(reason))
        {
            loginf << "inspector '" << ins->className()
                   << "' prerequisites not met: " << reason;
            return false;
        }

        loginf << "inspector '" << ins->className() << "' OK";
        any_enabled = true;
    }
    if (!any_enabled)
        loginf << "no inspector enabled";
    return any_enabled;
}

void AnalyzeDataSourceTask::run()
{
    loginf << "running with ds_type " << ds_type_
           << " selected " << selectedDataSourceIDs().size() << " data sources";

    auto& tm = compass().taskManager();

    const std::string window_title = "Analyze " + ds_type_ + " Data Source";
    const std::string result_name  = reportName();

    // Determine which inspectors will actually run, and what test dbcontents they need.
    std::vector<DataSourceInspectorBase*> active_inspectors;
    std::set<std::string> tst_dbcontents;
    bool any_needs_dataset   = false;
    bool any_needs_reference = false;

    for (const auto& ins : inspectors_)
    {
        if (!inspectorEnabled(ins->className()))
            continue;

        std::string reason;
        if (!ins->prerequisitesMet(reason))
        {
            logwrn << "skipping inspector " << ins->name()
                   << " (prerequisites not met " << reason << ")";
            continue;
        }

        active_inspectors.push_back(ins.get());

        if (ins->requiresLoadedDataset())
        {
            any_needs_dataset = true;
            if (ins->requiresReferenceTrajectory())
                any_needs_reference = true;
            for (const auto& dbc : ins->testDBContentNames())
                tst_dbcontents.insert(dbc);
        }
    }

    // Status dialog (modal, no cancel) - same pattern as the evaluation results
    // generator. Shown unconditionally: a status dialog is informational, not a
    // user prompt, so `allowUserInteractions()` does not gate it.
    //
    // Progress steps: preparing (1) + load (1 if any inspector needs the dataset)
    //                 + per active inspector (1 each) + saving (1).
    const int total_steps = 1
                            + (any_needs_dataset ? 1 : 0)
                            + static_cast<int>(active_inspectors.size())
                            + 1;
    QWidget* parent_w = Dialogs::statusDialogParent();

    auto status_dialog = std::make_unique<QProgressDialog>("", "", 0, total_steps, parent_w);
    status_dialog->setWindowTitle(QString::fromStdString(window_title));
    status_dialog->setCancelButton(nullptr);
    status_dialog->setWindowModality(Qt::ApplicationModal);
    status_dialog->setMinimumDuration(0);
    status_dialog->setMinimumWidth(420);
    status_dialog->setAutoClose(false);
    status_dialog->setAutoReset(false);
    // do not steal os focus from other applications when popping up
    status_dialog->setAttribute(Qt::WA_ShowWithoutActivating, true);

    QLabel* status_label = new QLabel("", status_dialog.get());
    status_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    status_dialog->setLabel(status_label);
    status_dialog->setValue(0);
    status_dialog->show();

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    // Pump events so the dialog actually paints before we start the work.
    QCoreApplication::processEvents();

    int step = 0;
    auto setStatus = [&](const std::string& msg)
    {
        status_label->setText(QString::fromStdString(msg));
        status_dialog->setValue(step);
        QCoreApplication::processEvents();
    };

    auto advanceStep = [&](const std::string& msg)
    {
        ++step;
        setStatus(msg);
    };

    setStatus("Preparing result...");

    tm.beginTaskResultWriting(result_name, task::TaskResultType::DataSourceAnalysis);

    auto& report = tm.currentReport();
    auto& root   = *report->rootSection();
    auto& overview = root.addSubSection("Overview");

    auto& info = overview.addTable("Run Configuration", 2, {"Property", "Value"}, false);

    auto formatDSList = [&](const std::set<unsigned int>& ids) {
        std::string out;
        for (auto ds_id : ids)
        {
            const auto* ds = compass().dbContextManager().dataSource(ds_id);
            if (!ds)
                continue;
            if (!out.empty()) out += "\n";
            out += ds->name() + " (" + std::to_string(ds->sac())
                   + "/" + std::to_string(ds->sic()) + ")";
        }
        return out;
    };

    info.addRow({"DSType", ds_type_});
    info.addRow({"Selected data sources", std::to_string(selectedDataSourceIDs().size())});
    {
        std::string sel = formatDSList(selectedDataSourceIDs());
        if (!sel.empty())
            info.addRow({"Names", sel});
    }
    info.addRow({"Test line", "L" + std::to_string(line_id_tst_ + 1)});
    {
        auto ref_ids = selectedReferenceDataSourceIDs();
        info.addRow({"Selected reference data sources", std::to_string(ref_ids.size())});
        std::string sel = formatDSList(ref_ids);
        if (!sel.empty())
            info.addRow({"Reference", sel});
    }
    info.addRow({"Reference line", "L" + std::to_string(line_id_ref_ + 1)});
    info.addRow({"Professional license",
                 professionalLicenseEnabled() ? std::string("enabled")
                                              : std::string("disabled")});
    info.addRow({"Use ground only", use_ground_only_ ? "yes" : "no"});
    {
        std::string fl;
        if (use_min_fl_) fl += "min FL " + std::to_string(static_cast<int>(min_fl_));
        if (use_max_fl_) fl += (fl.empty() ? "" : ", ") + std::string("max FL ")
                               + std::to_string(static_cast<int>(max_fl_));
        if (fl.empty()) fl = "none";
        info.addRow({"Flight level filter", fl});
    }
    {
        std::string sec = "no";
        if (limit_by_sectors_)
        {
            sec.clear();
            for (const auto& name : selectedSectorLayers())
                sec += (sec.empty() ? "" : ", ") + name;
            if (sec.empty()) sec = "yes (none selected)";
        }
        info.addRow({"Limit by sectors", sec});
    }

    advanceStep("Preparing result...");

    // Load the combined dataset once, before iterating the inspectors that need it.
    std::unique_ptr<AnalysisDataset> dataset;
    if (any_needs_dataset)
    {
        advanceStep("Loading combined dataset...");

        dataset.reset(new AnalysisDataset(compass()));

        AnalysisDataset::ScopeFilter scope_filter;
        scope_filter.ground_only = use_ground_only_;
        scope_filter.use_min_fl  = use_min_fl_;
        scope_filter.min_fl      = min_fl_;
        scope_filter.use_max_fl  = use_max_fl_;
        scope_filter.max_fl      = max_fl_;
        scope_filter.limit_by_sectors = limit_by_sectors_;
        {
            auto& ctx = compass().dbContextManager();

            if (limit_by_sectors_)
            {
                for (const auto& name : selectedSectorLayers())
                    if (ctx.hasSectorLayer(name))
                        scope_filter.sectors.push_back(ctx.sectorLayer(name));
            }

            // Reports of ground-only data sources (SMR) carry no ground bit
            // and count as on ground.
            for (auto ds_id : selectedDataSourceIDs())
            {
                const auto* ds = ctx.dataSource(ds_id);
                if (ds && ds->groundOnly())
                    scope_filter.ground_only_ds_ids.insert(ds_id);
            }
        }
        // SMR: keep only PSR reports of CAT010 sources that also carry MLAT reports.
        scope_filter.smr_only = (ds_type_ == "SMR");

        dataset->setScopeFilter(scope_filter);
        dataset->setRequireReference(any_needs_reference);

        std::string error;
        if (!dataset->load(selectedDataSourceIDs(), line_id_tst_,
                           selectedReferenceDataSourceIDs(), line_id_ref_,
                           tst_dbcontents, error))
        {
            logwrn << "AnalyzeDataSourceTask: dataset load failed: " << error;

            auto& warn_section = root.addSubSection("Dataset");
            auto& note = warn_section.addText("Note");
            note.addText("Combined dataset could not be built: " + error
                         + " Inspectors that depend on the loaded dataset will be skipped.");
            dataset.reset();
        }
        else
        {
            auto& info = overview.addTable("Loaded Dataset", 2, {"Property", "Value"}, false);
            info.addRow({"UTNs",                 std::to_string(dataset->utns().size())});
            info.addRow({"Reference records",    std::to_string(dataset->numReferenceRecordsTotal())});
            info.addRow({"Test records",         std::to_string(dataset->numTestRecordsTotal())});
            info.addRow({"Test chains (UTN x DBC)", std::to_string(dataset->numTestChains())});
            std::string dbcs;
            for (const auto& dbc : dataset->testDbContentsPresent())
            {
                if (!dbcs.empty()) dbcs += ", ";
                dbcs += dbc;
            }
            info.addRow({"Test dbcontents present", dbcs});
            info.addRow({"Unassociated test records",
                         std::to_string(dataset->numUnassociatedRecordsTotal())});
            if (dataset->numNonPSRRecordsSkipped() > 0)
                info.addRow({"Non-PSR CAT010 records skipped",
                             std::to_string(dataset->numNonPSRRecordsSkipped())});
            if (!dataset->statusCyclesByDS().empty())
            {
                std::string cycles;
                for (const auto& kv : dataset->statusCyclesByDS())
                {
                    const auto* ds = compass().dbContextManager().dataSource(kv.first);
                    if (!cycles.empty()) cycles += "\n";
                    cycles += (ds ? ds->name() : std::to_string(kv.first))
                              + ": " + std::to_string(kv.second.size());
                }
                info.addRow({"Scan cycles per source", cycles});
            }
        }
    }

    auto logRAM = [](const std::string& tag) {
        loginf << "RAM [" << tag << "] process "
               << Utils::String::doubleToStringPrecision(
                      Utils::System::getProcessRAMinGB(), 2)
               << " GB free "
               << Utils::String::doubleToStringPrecision(
                      Utils::System::getFreeRAMinGB(), 2)
               << " GB";
    };

    for (auto* ins : active_inspectors)
    {
        loginf << "running inspector " << ins->name();
        logRAM("inspector " + ins->name() + " before compute");
        advanceStep("Running inspector: " + ins->name() + "...");
        AnalysisDataset* ds = ins->requiresLoadedDataset() ? dataset.get() : nullptr;

        if (ins->requiresLoadedDataset())
        {
            // Heavy compute on a worker thread; keep the Qt event loop free on
            // the main thread so the status dialog paints. Mirrors
            // EvaluationResultsGenerator's std::async + polling pattern.
            std::atomic<bool> done{false};
            auto fut = std::async(std::launch::async, [ins, ds, &done]() {
                ins->compute(ds);
                done.store(true, std::memory_order_release);
            });

            using namespace std::chrono_literals;
            while (!done.load(std::memory_order_acquire))
            {
                QCoreApplication::processEvents();
                QThread::msleep(50);
            }
            fut.get();
        }
        else
        {
            ins->compute(ds);
        }

        logRAM("inspector " + ins->name() + " after compute");

        // Report writing is always on the main thread.
        ins->writeReport(root);

        logRAM("inspector " + ins->name() + " after writeReport");
    }

    advanceStep("Saving result...");
    tm.endTaskResultWriting(true, false);

    ++step;
    status_dialog->setValue(step);
    QCoreApplication::processEvents();
    status_dialog->close();
    QApplication::restoreOverrideCursor();

    done_ = true;
    emit doneSignal();

    // Mirror the evaluation flow: switch to the Task Results tool and show
    // the freshly written report so the user lands on it without an extra
    // click. See MainWindow::showResult / showEvaluationResult.
    compass().mainWindow().showResult(result_name);
}
