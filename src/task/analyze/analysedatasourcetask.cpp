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

#include "analysedatasourcetask.h"
#include "analysedatasourcedialog.h"
#include "analysisdataset.h"
#include "datasourceinspectorbase.h"
#include "inspectorsettingsbase.h"
#include "mlatdataiteminspector.h"
#include "mlatcoverageinspector.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "mlataccuracyinspector.h"
#endif

#include "compass.h"
#include "taskmanager.h"
#include "db_context_manager.h"
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

AnalyseDataSourceTask::AnalyseDataSourceTask(nlohmann::json& config, TaskManager* parent)
    : Task(*parent), Configurable(config, parent)
{
    setObjectName("AnalyseDataSourceTask");

    registerParameter("ds_type", &ds_type_, std::string("MLAT"));
    registerParameter("use_data_sources",       &use_data_sources_,       nlohmann::json::object());
    registerParameter("use_data_sources_lines", &use_data_sources_lines_, nlohmann::json::object());
    registerParameter("inspector_enabled",      &inspector_enabled_,      nlohmann::json::object());

    tooltip_ = "Analyse a chosen data source from multiple angles "
               "(data items, sensor coverage / PD, position accuracy).";

    createSubConfigurables();
}

AnalyseDataSourceTask::~AnalyseDataSourceTask() = default;

void AnalyseDataSourceTask::generateSubConfigurable(nlohmann::json& child_json)
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
#if USE_EXPERIMENTAL_SOURCE == true
    else if (class_name == "MLATAccuracyInspectorSettings")
    {
        traced_assert(!accuracy_settings_);
        accuracy_settings_.reset(new MLATAccuracyInspectorSettings(child_json, this));
        traced_assert(accuracy_settings_);
    }
#endif
    else
    {
        throw std::runtime_error("AnalyseDataSourceTask: generateSubConfigurable: "
                                 "unknown class_name " + class_name);
    }
}

void AnalyseDataSourceTask::checkSubConfigurables()
{
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
#endif
}

void AnalyseDataSourceTask::initTask()
{
    registerInspectors();
}

void AnalyseDataSourceTask::registerInspectors()
{
    inspectors_.clear();

    inspectors_.emplace_back(new MLATDataItemInspector(*this, *data_item_settings_));
    inspectors_.emplace_back(new MLATCoverageInspector(*this, *coverage_settings_));

#if USE_EXPERIMENTAL_SOURCE == true
    inspectors_.emplace_back(new MLATAccuracyInspector(*this, *accuracy_settings_));
#endif
}

bool AnalyseDataSourceTask::useDataSource(unsigned int ds_id) const
{
    auto key = std::to_string(ds_id);
    if (use_data_sources_.contains(key))
        return use_data_sources_.at(key).get<bool>();
    return true;
}

void AnalyseDataSourceTask::useDataSource(unsigned int ds_id, bool value)
{
    use_data_sources_[std::to_string(ds_id)] = value;
}

bool AnalyseDataSourceTask::useDataSourceLine(unsigned int ds_id, unsigned int line_id) const
{
    auto ds_key   = std::to_string(ds_id);
    auto line_key = std::to_string(line_id);
    if (use_data_sources_lines_.contains(ds_key)
        && use_data_sources_lines_.at(ds_key).contains(line_key))
        return use_data_sources_lines_.at(ds_key).at(line_key).get<bool>();
    return true;
}

void AnalyseDataSourceTask::useDataSourceLine(unsigned int ds_id, unsigned int line_id, bool value)
{
    use_data_sources_lines_[std::to_string(ds_id)][std::to_string(line_id)] = value;
}

std::set<unsigned int> AnalyseDataSourceTask::selectedDataSourceIDs() const
{
    std::set<unsigned int> ids;
    auto& ctx = compass().dbContextManager();
    auto types = ctx.dsTypes();

    for (auto ds_id : ctx.allDataSourceIds())
    {
        auto it = types.find(ds_id);
        if (it == types.end() || it->second != ds_type_)
            continue;
        if (!useDataSource(ds_id))
            continue;
        ids.insert(ds_id);
    }
    return ids;
}

bool AnalyseDataSourceTask::selectionContainsCAT020() const
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

bool AnalyseDataSourceTask::inspectorEnabled(const std::string& inspector_class) const
{
    if (inspector_enabled_.contains(inspector_class))
        return inspector_enabled_.at(inspector_class).get<bool>();
    return true;
}

void AnalyseDataSourceTask::inspectorEnabled(const std::string& inspector_class, bool value)
{
    inspector_enabled_[inspector_class] = value;
}

DataSourceInspectorBase* AnalyseDataSourceTask::inspector(const std::string& class_name) const
{
    for (const auto& ins : inspectors_)
        if (ins->className() == class_name)
            return ins.get();
    return nullptr;
}

MLATDataItemInspectorSettings& AnalyseDataSourceTask::dataItemSettings() const
{
    traced_assert(data_item_settings_);
    return *data_item_settings_;
}

MLATCoverageInspectorSettings& AnalyseDataSourceTask::coverageSettings() const
{
    traced_assert(coverage_settings_);
    return *coverage_settings_;
}

#if USE_EXPERIMENTAL_SOURCE == true
MLATAccuracyInspectorSettings& AnalyseDataSourceTask::accuracySettings() const
{
    traced_assert(accuracy_settings_);
    return *accuracy_settings_;
}
#endif

bool AnalyseDataSourceTask::professionalLicenseEnabled() const
{
    return compass().licenseManager().componentEnabled(
        license::License::Component::ComponentProbIMMReconstructor);
}

COMPASS& AnalyseDataSourceTask::compass() const
{
    return task_manager_.compass();
}

void AnalyseDataSourceTask::showDialog()
{
    AnalyseDataSourceDialog dlg(*this, QApplication::activeWindow());
    if (dlg.exec() == QDialog::Rejected)
        return;

    if (!canRun())
    {
        QMessageBox::warning(QApplication::activeWindow(), "Analyse Data Source",
                             "The task cannot run with the current selection.");
        return;
    }

    run();
}

bool AnalyseDataSourceTask::canRun()
{
    if (selectedDataSourceIDs().empty())
        return false;

    bool any_enabled = false;
    for (const auto& ins : inspectors_)
    {
        if (!inspectorEnabled(ins->className()))
            continue;

        std::string reason;
        if (!ins->prerequisitesMet(reason))
            return false;

        any_enabled = true;
    }
    return any_enabled;
}

void AnalyseDataSourceTask::run()
{
    loginf << "running with ds_type " << ds_type_
           << " selected " << selectedDataSourceIDs().size() << " data sources";

    auto& tm = compass().taskManager();

    const std::string title = "Analyse " + ds_type_ + " Data Source";

    // Determine which inspectors will actually run, and what test dbcontents they need.
    std::vector<DataSourceInspectorBase*> active_inspectors;
    std::set<std::string> tst_dbcontents;
    bool any_needs_dataset = false;

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
    QWidget* parent_w = QApplication::activeWindow();

    auto status_dialog = std::make_unique<QProgressDialog>("", "", 0, total_steps, parent_w);
    status_dialog->setWindowTitle(QString::fromStdString(title));
    status_dialog->setCancelButton(nullptr);
    status_dialog->setWindowModality(Qt::ApplicationModal);
    status_dialog->setMinimumDuration(0);
    status_dialog->setMinimumWidth(420);
    status_dialog->setAutoClose(false);
    status_dialog->setAutoReset(false);

    QLabel* status_label = new QLabel("", status_dialog.get());
    status_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    status_dialog->setLabel(status_label);
    status_dialog->setValue(0);
    status_dialog->show();
    status_dialog->raise();
    status_dialog->activateWindow();

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    // Pump events so the dialog actually paints before we start the work.
    QCoreApplication::processEvents();

    int step = 0;
    auto setStatus = [&](const std::string& msg)
    {
        status_label->setText(QString::fromStdString(msg));
        status_dialog->setValue(step);
        status_dialog->show();
        status_dialog->raise();
        QCoreApplication::processEvents();
    };

    auto advanceStep = [&](const std::string& msg)
    {
        ++step;
        setStatus(msg);
    };

    setStatus("Preparing result...");

    tm.beginTaskResultWriting(title, task::TaskResultType::Generic);

    auto& report = tm.currentReport();
    auto& root   = report->getSection("Results");
    auto& overview = root.addSubSection("Overview");

    auto& info = overview.addTable("Run Configuration", 2, {"Property", "Value"}, false);
    info.addRow({"DSType", ds_type_});
    info.addRow({"Selected data sources", std::to_string(selectedDataSourceIDs().size())});
    {
        std::string sel;
        for (auto ds_id : selectedDataSourceIDs())
        {
            const auto* ds = compass().dbContextManager().dataSource(ds_id);
            if (!ds)
                continue;
            if (!sel.empty()) sel += ", ";
            sel += ds->name() + " (" + std::to_string(ds_id) + ")";
        }
        if (!sel.empty())
            info.addRow({"Names", sel});
    }
    info.addRow({"Professional license",
                 professionalLicenseEnabled() ? std::string("enabled")
                                              : std::string("disabled")});

    advanceStep("Preparing result...");

    // Load the combined dataset once, before iterating the inspectors that need it.
    std::unique_ptr<AnalysisDataset> dataset;
    if (any_needs_dataset)
    {
        advanceStep("Loading combined dataset...");

        dataset.reset(new AnalysisDataset(compass()));
        std::string error;
        if (!dataset->load(selectedDataSourceIDs(), tst_dbcontents, error))
        {
            logwrn << "AnalyseDataSourceTask: dataset load failed: " << error;

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
        }
    }

    for (auto* ins : active_inspectors)
    {
        loginf << "running inspector " << ins->name();
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

        // Report writing is always on the main thread.
        ins->writeReport(root);
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
}
