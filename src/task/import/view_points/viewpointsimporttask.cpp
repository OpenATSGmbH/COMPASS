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

#include "viewpointsimporttask.h"
#include "viewpointsimporttaskdialog.h"
#include "taskmanager.h"
#include "compass.h"
#include "dbinterface.h"
#include "files.h"
#include "logger.h"
#include "viewmanager.h"
#include "viewpointsimporttask.h"
#include "viewpoint.h"

#include "asteriximporttask.h"
#include "util/timeconv.h"
#include "util/async.h"

#include <fstream>

#include <QCoreApplication>
#include <QApplication>
#include <QThread>
#include <QMessageBox>

#include "boost/date_time/posix_time/posix_time.hpp"

using namespace std;
using namespace nlohmann;
using namespace Utils;

ViewPointsImportTask::ViewPointsImportTask(const std::string& class_id, const std::string& instance_id,
                                           TaskManager& task_manager)
    : Task(task_manager),
      Configurable(class_id, instance_id, &task_manager, "task_import_view_points.json")
{
    tooltip_ =
            "Allows import of view points and associated datasets.";

    createSubConfigurables(); // no thing

    current_error_ = "No filename set";

    setObjectName("ViewPointsImportTask");
}

ViewPointsImportTask::~ViewPointsImportTask()
{
}

void ViewPointsImportTask::showDialog()
{
    ViewPointsImportTaskDialog dialog (*this);

    if (dialog.exec() == QDialog::Rejected)
        return;
    
    traced_assert(canRun());
    run();
}

// void ViewPointsImportTask::dialogImportSlot()
// {
//     traced_assert(canRun());

//     run();
// }

void ViewPointsImportTask::generateSubConfigurable(const std::string& class_id,
                                                   const std::string& instance_id)
{
    throw std::runtime_error("ViewPointsImportTask: generateSubConfigurable: unknown class_id " +
                             class_id);
}

void ViewPointsImportTask::importFilename(const std::string& filename)
{
    current_filename_ = filename;

    parseCurrentFile();
}

std::string ViewPointsImportTask::currentError() const
{
    return current_error_;
}

void ViewPointsImportTask::parseCurrentFile ()
{
    loginf << "file '" << current_filename_ << "'";

    current_error_ = "";

    current_data_.clear();

    if (!Files::fileExists(current_filename_))
    {
        current_error_ = "file '" + current_filename_ + "' does not exist";
        logerr << "start" << current_error_;
        return;
    }

    std::ifstream ifs(current_filename_);

    try
    {
        current_data_ = json::parse(ifs);

        checkParsedData();
    }
    catch (exception& e)
    {
        current_error_ = "parsing file '" + current_filename_ + "' resulted in error '" + e.what() + "'";
        logerr << "start" << current_error_;
    }

    loginf << "done";
}

void ViewPointsImportTask::checkParsedData ()
{
    loginf;

    std::string err;
    if (!ViewPoint::isValidJSON(current_data_, current_filename_, &err, true))
        throw std::runtime_error(err);
}

bool ViewPointsImportTask::canImport ()
{
    return current_error_.size() == 0;
}

bool ViewPointsImportTask::canRun()
{
    return canImport();
}

void ViewPointsImportTask::run()
{
    loginf;

    traced_assert(canImport()); // checked file content, version etc
    done_ = false;
    stopped_ = false;

    DBInterface& db_interface = COMPASS::instance().dbInterface();

    // check and clear existing ones
    if(db_interface.existsViewPointsTable() && db_interface.viewPoints().size() && allow_user_interactions_)
    {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(
                    nullptr, "Clear Existing View Points",
                    "There are already view points defined in the database.\n\n"
                    "Do you agree to delete all view points?",
                    QMessageBox::Yes | QMessageBox::No);

        if (reply == QMessageBox::Yes)
        {
            loginf << "deleting all view points";
        }
        else
        {
            loginf << "aborted";

            done_ = true;
            return;
        }
    }

    COMPASS::instance().logInfo("ViewPoints Import") << "started, clearing previous viewpoints";

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    COMPASS::instance().viewManager().loadViewPoints(current_data_);

    COMPASS::instance().logInfo("ViewPoints Import") << "imported";

    QApplication::restoreOverrideCursor();

    // datasets
    if (current_data_.contains(ViewPoint::VP_CONTEXT_KEY))
    {
        json& context = current_data_.at(ViewPoint::VP_CONTEXT_KEY);

        if (context.contains(ViewPoint::VP_CONTEXT_DATASETS_KEY))
        {
            COMPASS::instance().logInfo("ViewPoints Import") << "importing datasets";

            for (auto& ds_it : context.at(ViewPoint::VP_CONTEXT_DATASETS_KEY).get<json::array_t>())
            {
//                std::string name;

//                if (ds_it.contains("name"))
//                    name = ds_it.at("name");

                std::string filename = ds_it.at(ViewPoint::VP_CONTEXT_DATASET_FILENAME_KEY);

                loginf << "importing dataset file '" << filename << "'";

                if (!Files::fileExists(filename))
                {
                    std::string file = Files::getFilenameFromPath(filename);
                    std::string dir = Files::getDirectoryFromPath(current_filename_);

                    filename = dir+"/"+file;

                    traced_assert(Files::fileExists(filename));
                }

                ASTERIXImportTask& task = task_manager_.asterixImporterTask();

                unsigned int line_id {0};

                // line
                if (ds_it.contains("line_id"))
                {
                    traced_assert(ds_it.at("line_id").is_number_unsigned());
                    line_id = ds_it.at("line_id");

                    task.settings().file_line_id_ = line_id;

                    loginf << "line_id " << line_id;
                }


                if (ds_it.contains("time_offset"))
                {
                    traced_assert(ds_it.at("time_offset").is_number());

                    float tod_offset = ds_it.at("time_offset");

                    task.settings().override_tod_active_ = true;
                    task.settings().override_tod_offset_ = -tod_offset;
                }
                else
                {
                    loginf << "override information not set";
                    task.settings().override_tod_active_ = false;
                    task.settings().override_tod_offset_ = 0;
                }

                if (ds_it.contains("date"))
                {
                    traced_assert(ds_it.at("date").is_string());
                    string date_str = ds_it.at("date");

                    loginf << "date " << date_str;

                    boost::posix_time::ptime date = Time::fromDateString(date_str);

                    task.settings().date_ = date;
                }

                task.source().setSourceType(ASTERIXImportSource::SourceType::FileASTERIX, {filename}); //line_id);

                traced_assert(task.canRun());
                task.allowUserInteractions(false);

                //widget->runCurrentTaskSlot();
                loginf << "running task";
                task.run();

                while (!task.done())
                {
                    QCoreApplication::processEvents();
                    QThread::msleep(1);
                }

                loginf << "importing dataset file '" << filename << "' done";

            }

            Async::waitAndProcessEventsFor(50);

            COMPASS::instance().logInfo("ViewPoints Import") << "importing datasets done";
        }
    }

    COMPASS::instance().logInfo("ViewPoints Import") << "done";

    done_ = true;

    emit doneSignal();

    loginf;
}
void ViewPointsImportTask::stop()
{

}

const nlohmann::json& ViewPointsImportTask::currentData() const
{
    return current_data_;
}
