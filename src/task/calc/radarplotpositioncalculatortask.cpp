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

#include "radarplotpositioncalculatortask.h"
#include "compass.h"
#include "buffer.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "logger.h"
#include "projection.h"
#include "projectionmanager.h"
#include "radarplotpositioncalculatortaskdialog.h"
#include "stringconv.h"
#include "taskmanager.h"
#include "viewmanager.h"
#include "datasourcemanager.h"

#include <QApplication>
#include <QCoreApplication>
#include <QMessageBox>

using namespace std;
using namespace Utils;
using namespace dbContent;

RadarPlotPositionCalculatorTask::RadarPlotPositionCalculatorTask(const std::string& class_id,
                                                                 const std::string& instance_id,
                                                                 TaskManager& task_manager)
    : Task(task_manager),
      Configurable(class_id, instance_id, &task_manager, "task_calc_radar_pos.json")
{
    tooltip_ =
            "Allows calculation of Radar plot position information based on the defined data sources.";

    qRegisterMetaType<std::shared_ptr<Buffer>>("std::shared_ptr<Buffer>");
    // qRegisterMetaType<DBContent>("DBContent");

}

RadarPlotPositionCalculatorTask::~RadarPlotPositionCalculatorTask() {}

void RadarPlotPositionCalculatorTask::showDialog()
{
    RadarPlotPositionCalculatorTaskDialog dialog (*this);

    if (dialog.exec() == QDialog::Rejected)
        return;

    traced_assert(canRun());
    run();
}

bool RadarPlotPositionCalculatorTask::canRun()
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (!dbcont_man.existsDBContent("CAT001")
            && !dbcont_man.existsDBContent("CAT010")
            && !dbcont_man.existsDBContent("CAT048"))
        return false;

    if (!dbcont_man.dbContent("CAT001").loadable()
            && !dbcont_man.dbContent("CAT010").loadable()
            && !dbcont_man.dbContent("CAT048").loadable())
        return false;

    const std::vector<std::unique_ptr<dbContent::DBDataSource>>& db_srcs =
            COMPASS::instance().dataSourceManager().dbDataSources();

    bool found_radar_with_data = false;
    for (const auto& src : db_srcs)
    {
        if (src->dsType() == "Radar" && src->hasNumInserted())
        {
            found_radar_with_data = true;
            break;
        }

    }

    return found_radar_with_data;
}

void RadarPlotPositionCalculatorTask::run()
{
    loginf;

    traced_assert(canRun());

    start_time_ = boost::posix_time::microsec_clock::local_time();

    COMPASS::instance().logInfo("Radar Plot Position Calculation")
        << "started";

    // set up projections
    ProjectionManager& proj_man = ProjectionManager::instance();

    traced_assert(proj_man.hasCurrentProjection());
    Projection& projection = proj_man.currentProjection();
    projection.clearCoordinateSystems(); // to rebuild from data sources
    projection.addAllCoordinateSystems();

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    dbcontent_man.clearData();
    dbcontent_done_.clear();

    COMPASS::instance().viewManager().disableDataDistribution(true);

    connect(&dbcontent_man, &DBContentManager::loadedDataSignal,
            this, &RadarPlotPositionCalculatorTask::loadedDataSlot);
    connect(&dbcontent_man, &DBContentManager::loadingDoneSignal,
            this, &RadarPlotPositionCalculatorTask::loadingDoneSlot);

    calculating_ = true;
    done_ = false;
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    std::string msg = "Loading object data";
    msg_box_ = new QMessageBox;
    traced_assert(msg_box_);
    msg_box_->setWindowTitle("Calculating Radar Plot Positions");
    msg_box_->setText(msg.c_str());
    msg_box_->setStandardButtons(QMessageBox::NoButton);
    msg_box_->show();

    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

    for (auto& dbcont_it : dbcontent_man)
    {
        if (dbcont_it.first != "CAT001" && dbcont_it.first != "CAT010" && dbcont_it.first != "CAT048")
            continue;

        if (!dbcont_it.second->hasData())
            continue;

        VariableSet read_set = getReadSetFor(dbcont_it.first);

        dbcont_it.second->load(read_set, false, false);
    }
}

void RadarPlotPositionCalculatorTask::loadedDataSlot(
        const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset)
{
    data_ = data;
}

void RadarPlotPositionCalculatorTask::loadingDoneSlot()
{
    loginf << "starting calculation";

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    disconnect(&dbcontent_man, &DBContentManager::loadedDataSignal,
               this, &RadarPlotPositionCalculatorTask::loadedDataSlot);
    disconnect(&dbcontent_man, &DBContentManager::loadingDoneSignal,
               this, &RadarPlotPositionCalculatorTask::loadingDoneSlot);

    dbcontent_man.clearData();

    COMPASS::instance().viewManager().disableDataDistribution(false);

    ProjectionManager& proj_man = ProjectionManager::instance();

    std::pair<unsigned int, std::map<std::string, std::shared_ptr<Buffer>>> result_buffers =
            proj_man.doUpdateRadarPlotPositionCalculations (data_);

    unsigned int transformation_errors = result_buffers.first;
    std::map<std::string, std::shared_ptr<Buffer>> update_buffers = result_buffers.second;

    unsigned int buffers_size = 0;

    for (auto& buf_it : update_buffers)
        buffers_size += buf_it.second->size();

    traced_assert(msg_box_);
    delete msg_box_;

    COMPASS::instance().logInfo("Radar Plot Position Calculation")
        << transformation_errors << " transformation errors";

    if (transformation_errors)
    {
        QApplication::restoreOverrideCursor();

        QMessageBox::StandardButton reply;

        std::string question =
                "There were " + std::to_string(transformation_errors) +
                " skipped coordinates with transformation errors, " +
                std::to_string(buffers_size) +
                " coordinates were projected correctly. Do you want to insert the data?";

        reply = QMessageBox::question(nullptr, "Insert Data", question.c_str(),
                                      QMessageBox::Yes | QMessageBox::No);

        if (reply == QMessageBox::No)
        {
            loginf << "aborted by user because of "
                      "transformation errors";

            COMPASS::instance().logInfo("Radar Plot Position Calculation") << "save declined";

            return;
        }
    }

    if (buffers_size)
    {
        std::string msg;
        msg_box_ = new QMessageBox;
        traced_assert(msg_box_);
        msg_box_->setWindowTitle("Calculating Radar Plot Positions");
        msg = "Writing object data";
        msg_box_->setText(msg.c_str());
        msg_box_->setStandardButtons(QMessageBox::NoButton);
        msg_box_->show();

        logdbg << "writing size " << buffers_size;

        for (auto& buf_it : update_buffers)
        {
            DBContent& dbcontent = dbcontent_man.dbContent(buf_it.first);

            dbcontent.updateData(dbcontent_man.metaGetVariable(
                                     buf_it.first, DBContent::meta_var_rec_num_), buf_it.second);

            connect(&dbcontent, &DBContent::updateDoneSignal, this, &RadarPlotPositionCalculatorTask::updateDoneSlot);
        }
    }

    loginf << "end";
}

void RadarPlotPositionCalculatorTask::updateDoneSlot(DBContent& db_content)
{
    loginf;

    disconnect(&db_content, &DBContent::updateDoneSignal,
               this, &RadarPlotPositionCalculatorTask::updateDoneSlot);

    traced_assert(data_.count(db_content.name()));

    traced_assert(!dbcontent_done_.count(db_content.name()));
    dbcontent_done_.insert(db_content.name());

    if (dbcontent_done_.size() == data_.size())
    {
        loginf << "fully done";

        data_.clear();

        traced_assert(msg_box_);
        msg_box_->close();
        delete msg_box_;
        msg_box_ = nullptr;

        stop_time_ = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration time_diff = stop_time_ - start_time_;
        std::string elapsed_time_str =
                String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, false);

        done_ = true;

        QApplication::restoreOverrideCursor();

        msg_box_ = new QMessageBox;
        traced_assert(msg_box_);
        msg_box_->setWindowTitle("Calculating Radar Plot Positions");
        msg_box_->setText("Writing of object data done.");
        msg_box_->setStandardButtons(QMessageBox::Ok);

        if (allow_user_interactions_)
            msg_box_->exec();

        delete msg_box_;
        msg_box_ = nullptr;

        COMPASS::instance().logInfo("Radar Plot Position Calculation")
            << "finished after "
            << String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, false);

        emit doneSignal();
    }
    else
        loginf << "not yet done";
}


bool RadarPlotPositionCalculatorTask::isCalculating() { return calculating_; }

dbContent::VariableSet RadarPlotPositionCalculatorTask::getReadSetFor(const std::string& dbcontent_name)
{
    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    VariableSet read_set;

    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_rec_num_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rec_num_));

    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));

    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_));

    traced_assert(dbcontent_man.canGetVariable(dbcontent_name, DBContent::var_radar_range_));
    read_set.add(dbcontent_man.getVariable(dbcontent_name, DBContent::var_radar_range_));

    traced_assert(dbcontent_man.canGetVariable(dbcontent_name, DBContent::var_radar_azimuth_));
    read_set.add(dbcontent_man.getVariable(dbcontent_name, DBContent::var_radar_azimuth_));

    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_));

    // optionals
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_));

    return read_set;
}
