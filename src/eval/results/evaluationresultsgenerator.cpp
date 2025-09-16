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

#include "evaluationresultsgenerator.h"
#include "evaluationmanager.h"
#include "evaluationcalculator.h"
#include "evaluationdata.h"
#include "evaluationstandard.h"
#include "evaluationtaskresult.h"

#include "eval/requirement/group.h"
#include "eval/requirement/base/baseconfig.h"
#include "eval/requirement/base/base.h"

#include "eval/results/base/single.h"
#include "eval/results/base/joined.h"

#include "eval/results/report/evalsectionid.h"

#include "dbcontentmanager.h"
#include "dbinterface.h"

#include "taskmanager.h"
#include "taskresult.h"
#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"

#include "compass.h"
#include "logger.h"
#include "stringconv.h"
#include "global.h"
#include "sectorlayer.h"
#include "async.h"

#include <QProgressDialog>
#include <QApplication>
#include <QThread>
#include <QLabel>
#include <QMessageBox>

#include "util/tbbhack.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <future>

using namespace std;
using namespace EvaluationRequirementResult;
using namespace EvaluationResultsReport;
using namespace Utils;

const std::string EvaluationResultsGenerator::EvalResultName = "Evaluation";

/**
 */
EvaluationResultsGenerator::EvaluationResultsGenerator(EvaluationCalculator& calculator)
:   calculator_(calculator)
{
}

/**
 */
EvaluationResultsGenerator::~EvaluationResultsGenerator()
{
    clear();
}

/**
 */
void EvaluationResultsGenerator::evaluate(EvaluationStandard& standard,
                                          const std::vector<unsigned int>& utns,
                                          const std::vector<Evaluation::RequirementResultID>& requirements,
                                          bool update_report)
{
    traced_assert(calculator_.dataLoaded());
    traced_assert(calculator_.sectorsLoaded());

    const auto& eval_settings = calculator_.settings();
    auto&       data          = calculator_.data();

    loginf << "start"
           << " skip_no_data_details " << eval_settings.report_skip_no_data_details_
           << " split_results_by_mops " << eval_settings.report_split_results_by_mops_
           << " report_split_results_by_aconly_ms " << eval_settings.report_split_results_by_aconly_ms_;

    boost::posix_time::ptime start_time;
    boost::posix_time::ptime elapsed_time;

    start_time = boost::posix_time::microsec_clock::local_time();

    std::vector<std::shared_ptr<SectorLayer>>& sector_layers = calculator_.sectorLayers();

    unsigned int num_req_evals = 0;
    for (auto& sec_it : sector_layers)
    {
        const string& sector_layer_name = sec_it->name();

        for (auto& req_group_it : standard)
        {
            if (!req_group_it->used())
                continue;

            const string& requirement_group_name = req_group_it->name();

            if (!calculator_.useGroupInSectorLayer(sector_layer_name, requirement_group_name))
                continue; // skip if not used

            num_req_evals += req_group_it->numUsedRequirements() * data.size(); // num reqs * num target
            num_req_evals += req_group_it->numUsedRequirements(); // num reqs for sector sum
        }
    }

    QProgressDialog postprocess_dialog ("", "", 0, num_req_evals);
    postprocess_dialog.setWindowTitle("Evaluating");
    postprocess_dialog.setCancelButton(nullptr);
    postprocess_dialog.setWindowModality(Qt::ApplicationModal);

    QLabel* progress_label = new QLabel("", &postprocess_dialog);
    progress_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    postprocess_dialog.setLabel(progress_label);

    postprocess_dialog.show();

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    clear();

    result_name_ = standard.name() + " " + EvalResultName;

    vector<unsigned int> used_utns;
    std::set<unsigned int> utn_set(utns.begin(), utns.end());

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    for (auto& target_data_it : data)
    {
        if (!dbcont_man.existsTarget(target_data_it.utn_))
            logerr << "unknown utn " << target_data_it.utn_;

        traced_assert(dbcont_man.existsTarget(target_data_it.utn_));
        traced_assert(data.hasTargetData(target_data_it.utn_));

        //utn list is provided => skip utns not in list
        if (!utn_set.empty() && !utn_set.count(target_data_it.utn_))
            continue;

        //if (target_data_it.use())
        used_utns.push_back(target_data_it.utn_);
    }

    unsigned int num_utns = used_utns.size();
    unsigned int eval_cnt = 0;

    boost::posix_time::time_duration time_diff;
    double elapsed_time_s;
    double time_per_eval, remaining_time_s;

    string remaining_time_str;

    string subresult_str;

    for (auto& sec_it : sector_layers)
    {
        const string& sector_layer_name = sec_it->name();

        loginf << "sector layer " << sector_layer_name;

        for (auto& req_group_it : standard)
        {
            if (!req_group_it->used())
                continue;

            const string& requirement_group_name = req_group_it->name();

            if (!calculator_.useGroupInSectorLayer(sector_layer_name, requirement_group_name))
                continue; // skip if not used

            loginf << "sector layer " << sector_layer_name
                   << " group " << requirement_group_name;

            for (auto& req_cfg_it : *req_group_it)
            {
                if (!req_cfg_it->used())
                    continue;

                //check list of requirements if provided
                if (!requirements.empty())
                {
                    auto it = std::find_if(requirements.begin(), requirements.end(), 
                        [ & ] (const Evaluation::RequirementResultID& id) 
                        {
                            return id.sec_layer_name == sector_layer_name &&
                                   id.req_group_name == requirement_group_name &&
                                   id.req_name == req_cfg_it->name();
                        });
                    
                        if (it == requirements.end())
                            continue;
                }

                loginf << "start"
                       << " sector layer " << sector_layer_name
                       << " group " << requirement_group_name
                       << " req '" << req_cfg_it->name() << "'";

                std::shared_ptr<EvaluationRequirement::Base> req = req_cfg_it->createRequirement();
                std::shared_ptr<Joined> result_sum;
                map<string, std::shared_ptr<Joined>> extra_results_sums;

                vector<shared_ptr<Single>> results;
                results.resize(num_utns);

                vector<bool> done_flags;
                done_flags.resize(num_utns, false);
                bool task_done = false;

                // generate results
//                EvaluateTask* t = new (tbb::task::allocate_root()) EvaluateTask(
//                            results, used_utns, data, req, *sec_it, done_flags, task_done, false);
//                tbb::task::enqueue(*t);

                const SectorLayer& sector_layer = *sec_it;
                bool single_thread = false;

//                int num_threads = oneapi::tbb::info::default_concurrency();
//                loginf << "starting, num_threads " << num_threads;

                std::future<void> pending_future = std::async(std::launch::async, [&] {

                    try
                    {
                        unsigned int num_utns = used_utns.size();
                        traced_assert(done_flags.size() == num_utns);

                        if (single_thread)
                        {
                            for(unsigned int utn_cnt=0; utn_cnt < num_utns; ++utn_cnt)
                            {
                                results[utn_cnt] = req->evaluate(data.targetData(used_utns.at(utn_cnt)), req, sector_layer);
                                done_flags[utn_cnt] = true;
                            }
                        }
                        else
                        {
                            tbb::parallel_for(uint(0), num_utns, [&](unsigned int utn_cnt)
                                              {
                                                  //assert(num_threads == oneapi::tbb::this_task_arena::max_concurrency());
                                                  results[utn_cnt] = req->evaluate(data.targetData(used_utns.at(utn_cnt)), req, sector_layer);
                                                  done_flags[utn_cnt] = true;
                                              });
                        }

                        for(unsigned int utn_cnt=0; utn_cnt < num_utns; ++utn_cnt)
                            traced_assert(results[utn_cnt]);

                        task_done = true;
                    }
                    catch (const std::exception& e)
                    {
                        logerr << "exception '" << e.what() << "'";
                        throw e;
                    }

                });

                unsigned int tmp_done_cnt;

                postprocess_dialog.setLabelText(
                            ("Sector Layer "+sector_layer_name
                             +":\n Requirement: "+req_group_it->name()+":\n    "+req_cfg_it->name()+"\n\n\n").c_str());
                postprocess_dialog.setValue(eval_cnt);

                Async::waitAndProcessEventsFor(50);

                logdbg << "waiting on group " << req_group_it->name()
                       << " req '" << req_cfg_it->name() << "'";

                while (!task_done)
                {
                    tmp_done_cnt = 0;

                    for (auto done_it : done_flags)
                    {
                        if (done_it)
                            tmp_done_cnt++;
                    }

                    //assert (eval_cnt+tmp_done_cnt <= num_req_evals);
                    // hack
                    if (eval_cnt+tmp_done_cnt <= num_req_evals && tmp_done_cnt)
                    {

                        elapsed_time = boost::posix_time::microsec_clock::local_time();

                        time_diff = elapsed_time - start_time;
                        elapsed_time_s = time_diff.total_milliseconds() / 1000.0;

                        time_per_eval = elapsed_time_s/(double)(eval_cnt+tmp_done_cnt);
                        remaining_time_s = (double)(num_req_evals-eval_cnt-tmp_done_cnt)*time_per_eval;

                        postprocess_dialog.setLabelText(
                                    ("Sector Layer "+sector_layer_name
                                     +":\n  "+req_group_it->name()+":\n    "+req_cfg_it->name()
                                     +"\n\nElapsed: "+String::timeStringFromDouble(elapsed_time_s, false)
                                     +"\nRemaining: "+String::timeStringFromDouble(remaining_time_s, false)
                                     +" (estimated)").c_str());

                        postprocess_dialog.setValue(eval_cnt+tmp_done_cnt);

                        Async::waitAndProcessEventsFor(50);
                    }

                    if (!task_done)
                    {
                        QCoreApplication::processEvents();
                        QThread::msleep(100);
                    }
                }

                postprocess_dialog.setLabelText(("Sector Layer "+sector_layer_name+":\nAggregating results").c_str());

                for (auto& result_it : results)
                {
                    results_[result_it->reqGrpId()][result_it->resultId()] = result_it;
                    results_vec_.push_back(result_it);

                    if (!result_sum)
                        result_sum = result_it->createEmptyJoined("Sum");

                    result_sum->addSingleResult(result_it);

                    if (eval_settings.report_split_results_by_mops_)
                    {
                        subresult_str = result_it->target()->mopsVersionStr();

                        if (subresult_str == "?")
                            subresult_str = "Unknown";

                        subresult_str = "MOPS "+subresult_str;

                        if (!extra_results_sums.count(subresult_str+" Sum"))
                            extra_results_sums[subresult_str+" Sum"] =
                                    result_it->createEmptyJoined(subresult_str+" Sum");

                        extra_results_sums.at(subresult_str+" Sum")->addSingleResult(result_it);
                    }

                    if (eval_settings.report_split_results_by_aconly_ms_)
                    {
                        subresult_str = "Primary";

                        if (result_it->target()->isModeS())
                            subresult_str = "Mode S";
                        else if (result_it->target()->isModeACOnly())
                            subresult_str = "Mode A/C";
                        else
                            traced_assert(result_it->target()->isPrimaryOnly());

                        if (!extra_results_sums.count(subresult_str+" Sum"))
                            extra_results_sums[subresult_str+" Sum"] =
                                    result_it->createEmptyJoined(subresult_str+" Sum");

                        extra_results_sums.at(subresult_str+" Sum")->addSingleResult(result_it);
                    }
                }

                if (result_sum)
                {
                    loginf << "adding result '" << result_sum->reqGrpId()
                           << "' id '" << result_sum->resultId() << "'";
                    traced_assert(!results_[result_sum->reqGrpId()].count(result_sum->resultId()));

                    //update now => here we still have all details for the joined viewable
                    result_sum->updateToChanges(true);

                    results_[result_sum->reqGrpId()][result_sum->resultId()] = result_sum;
                    results_vec_.push_back(result_sum); // has to be added after all singles
                }

                for (auto& mops_res_it : extra_results_sums) // add extra results generated by splits
                {
                    loginf << "adding extra result '"
                           << mops_res_it.second->reqGrpId()
                           << "' id '" << mops_res_it.second->resultId() << "'";

                    traced_assert(!results_[mops_res_it.second->reqGrpId()].count(mops_res_it.second->resultId()));

                    //update now => here we still have all details for the joined viewable
                    mops_res_it.second->updateToChanges(true);

                    results_[mops_res_it.second->reqGrpId()][mops_res_it.second->resultId()] = mops_res_it.second;
                    results_vec_.push_back(mops_res_it.second); // has to be added after all singles
                }

                //purge stored single result details
                for (auto& result_it : results)
                    result_it->purgeStoredDetails();

                traced_assert(eval_cnt <= num_req_evals);
                eval_cnt = results_vec_.size();
            }
        }
    }

    postprocess_dialog.close();

    //viewables are up-to-date => do not reset them
    updateToChanges(false, update_report);

    elapsed_time   = boost::posix_time::microsec_clock::local_time();
    time_diff      = elapsed_time - start_time;
    elapsed_time_s = time_diff.total_milliseconds() / 1000.0;

    loginf << "data done " << String::timeStringFromDouble(elapsed_time_s, true);

    // 00:06:22.852 with no parallel

    //@TODO?
    //emit eval_calc_.resultsChangedSignal();

    loginf << "generating results";

    // generating results GUI
    //generateResultsReportGUI();

    loginf << "done " << String::timeStringFromDouble(elapsed_time_s, true);

    QApplication::restoreOverrideCursor();
}

/**
 */
void EvaluationResultsGenerator::clear()
{
    // clear everything
    results_.clear();
    results_vec_.clear();

    result_name_ = "";
}

/**
 */
void EvaluationResultsGenerator::generateResultsReportGUI()
{
    loginf;

    boost::posix_time::ptime loading_start_time;
    boost::posix_time::ptime loading_stop_time;

    loading_start_time = boost::posix_time::microsec_clock::local_time();

    QProgressDialog dlg; // QApplication::topLevelWidgets().first()
    dlg.setWindowTitle("Updating Results");
    dlg.setLabelText( "Please wait...");
    dlg.setRange(0, 0);
    dlg.setCancelButton(nullptr);
    dlg.setWindowModality(Qt::ApplicationModal);
    dlg.setMinimumWidth(500);
    dlg.show();

    auto& task_manager = COMPASS::instance().taskManager();
    task_manager.beginTaskResultWriting(result_name_, task::TaskResultType::Evaluation);

    auto& result = task_manager.currentResult();
    auto& report = task_manager.currentReport();

    EvaluationTaskResult* eval_result = dynamic_cast<EvaluationTaskResult*>(result.get());
    traced_assert(eval_result);

    //store eval config
    nlohmann::json config;
    calculator_.generateJSON(config, Configurable::JSONExportType::General);
    result->setJSONConfiguration(config);

    auto& gen_sec = report->getSection("Overview:General");

    gen_sec.addText("This section contains information about the used application, database and data sources.");

    // add dataset stuff

    gen_sec.addTable("gen_overview_table", 3, {"Name", "Comment", "Value"}, false);

    auto& gen_table = gen_sec.getTable("gen_overview_table");

    gen_table.addRow({"Application", "Application Filename", APP_FILENAME});
    gen_table.addRow({"Application Version", "Application Version", VERSION});
    gen_table.addRow({"DB", "Database Name", COMPASS::instance().lastDbFilename()});

    traced_assert(calculator_.hasCurrentStandard());
    gen_table.addRow({"Standard", "Standard name", calculator_.currentStandardName()});

    // add used sensors

    auto data_source_ref = calculator_.activeDataSourceInfoRef();
    auto data_source_tst = calculator_.activeDataSourceInfoTst();

    std::string sensors_ref;
    std::string sensors_tst;

    for (const auto& elem : data_source_ref.data_sources)
        sensors_ref += (sensors_ref.empty() ? "" : ", ") + elem.name;
    
    for (const auto& elem : data_source_tst.data_sources)
        sensors_tst += (sensors_tst.empty() ? "" : ", ") + elem.name;

    sensors_ref = data_source_ref.dbcontent + ": " + sensors_ref;
    sensors_tst = data_source_tst.dbcontent + ": " + sensors_tst;
    
    gen_table.addRow({ "Reference Sensors", "Used reference sensors", sensors_ref });
    gen_table.addRow({ "Test Sensors", "Used test sensors", sensors_tst });

    // generate results

    loginf << "Adding joined results...";

    // first add all joined
    for (auto& result_it : results_vec_)
    {
        if (result_it->isJoined())
        {
            QCoreApplication::processEvents();
            result_it->addToReport(report);
        }
    }

    loginf << "Adding single results...";

    // then all singles
    unsigned int cnt = 0;
    for (auto& result_it : results_vec_)
    {
        if (result_it->isSingle())
        {
            if (cnt % 100 == 0)
                QCoreApplication::processEvents();

            result_it->addToReport(report);

            ++cnt;
        }
    }

    loginf << "Adding targets...";

    // generate target section
    addTargetSection(report);

    loginf << "Adding non-results content...";

    // generate non-result details
    addNonResultsContent(report);

    // disable per target result section for all report exports (too detailed, too much overhead)
    // @TODO: additionally we could also deactivate all subitems separately.
    // advantage of only activating the parent node:
    // - whole node skipped in standard exports
    // - node deactivated when trying to fetch it via get_result rtcommand (could take forever to fetch)
    // - fetching subnodes (=single targets) via get_result rtcommand still possible if really desired
    auto& target_sec = report->getSection(EvalSectionID::targetID());
    target_sec.enableExports(false);

    loginf << "Storing targets...";

    // store targets to result
    eval_result->setTargets(calculator_.data().toTargets());

    loginf << "storing results...";

    task_manager.endTaskResultWriting(true);

    loginf << "results stored";

    loading_stop_time = boost::posix_time::microsec_clock::local_time();

    double load_time;
    boost::posix_time::time_duration diff = loading_stop_time - loading_start_time;
    load_time = diff.total_milliseconds() / 1000.0;

    dlg.close();

    loginf << "done "
           << String::timeStringFromDouble(load_time, true);
}

/**
 */
void EvaluationResultsGenerator::updateToChanges(bool reset_viewable, 
                                                 bool update_report)
{
    loginf << "reset_viewable " << reset_viewable;

    //update data to changes (target usage, reset interest factors etc.)
    calculator_.data().updateToChanges();

    // first check all single results if should be used
    for (auto& result_it : results_vec_)
    {
        if (result_it->isSingle()) // single result
        {
            // s starts with prefix

            shared_ptr<EvaluationRequirementResult::Single> result =
                static_pointer_cast<EvaluationRequirementResult::Single>(result_it);

            traced_assert(result);
            result->updateUseFromTarget();
        }
    }

    // then do the joined ones
    for (auto& result_it : results_vec_)
    {
        if (result_it->isJoined()) // single result
        {
            shared_ptr<EvaluationRequirementResult::Joined> result =
                static_pointer_cast<EvaluationRequirementResult::Joined>(result_it);

            traced_assert(result);
            result->updateToChanges(reset_viewable);
        }
    }

    if (update_report)
        generateResultsReportGUI();
}

/**
 */
void EvaluationResultsGenerator::updateToChanges ()
{
    //always reset the viewable on standard result updates
    updateToChanges(true);
}

/**
 */
void EvaluationResultsGenerator::addTargetSection(const std::shared_ptr<ResultReport::Report>& report)
{
    calculator_.data().addToReport(report);
}

/**
 */
void EvaluationResultsGenerator::addNonResultsContent (const std::shared_ptr<ResultReport::Report>& report)
{
    // standard
    traced_assert(calculator_.hasCurrentStandard());
    calculator_.currentStandard().addToReport(report);
}
