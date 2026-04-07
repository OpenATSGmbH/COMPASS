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

#include "managesectorstask.h"
#include "compass.h"
#include "db_context_manager.h"
#include "logger.h"
#include "managesectorstaskdialog.h"
#include "sector_import_utils.h"
#include "taskmanager.h"
#include "files.h"

#include "json.hpp"

#include <QMessageBox>

using namespace Utils;
using namespace nlohmann;
using namespace std;

ManageSectorsTask::ManageSectorsTask(nlohmann::json& config, TaskManager* parent)
    : Task(*parent),
      Configurable(config, parent)
{
    registerParameter("db_file_list", &file_list_, json::array());
    registerParameter("current_filename", &current_filename_, std::string());

    createSubConfigurables();

    vector<string> cleaned_file_list;
    // clean missing files

    for (auto& filename : file_list_.get<std::vector<string>>())
    {
        if (Files::fileExists(filename))
            cleaned_file_list.push_back(filename);
    }
    file_list_ = cleaned_file_list;

    if (!hasFile(current_filename_))
        current_filename_ = "";

    tooltip_ =
            "Allows management of sectors stored in the database. "
            "This task can not be run, but is performed using the GUI elements.";

    if (canImportFile())
        parseCurrentFile(false);
}

ManageSectorsTask::~ManageSectorsTask()
{
    file_list_.clear();
}

ManageSectorsTaskDialog* ManageSectorsTask::dialog()
{
    if (!dialog_)
    {
        dialog_.reset(new ManageSectorsTaskDialog(*this));

        connect(dialog_.get(), &ManageSectorsTaskDialog::doneSignal,
                this, &ManageSectorsTask::dialogDoneSlot);
    }

    traced_assert(dialog_);
    return dialog_.get();
}

void ManageSectorsTask::dialogDoneSlot()
{
    traced_assert(dialog_);
    dialog_->hide();

    emit manager().compass().dbContextManager().sectorsChangedSignal();
}

bool ManageSectorsTask::hasFile(const std::string& filename) const
{
    logdbg << "filename '" << filename
           << "' file_list_ '" << file_list_.dump(2) << "'";

    vector<string> tmp_list = file_list_.get<std::vector<string>>();

    return find(tmp_list.begin(), tmp_list.end(), filename) != tmp_list.end();
}

std::vector<std::string> ManageSectorsTask::fileList() const
{
    return file_list_.get<std::vector<string>>();
}

void ManageSectorsTask::addFile(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    if (file_list_.count(filename) != 0)
        throw std::invalid_argument("ManageSectorsTask: addFile: name '" + filename +
                                    "' already in use");

    vector<string> tmp_list = file_list_.get<std::vector<string>>();
    if (find(tmp_list.begin(), tmp_list.end(), filename) == tmp_list.end())
    {
        loginf << "adding filename '" << filename << "'";

        tmp_list.push_back(filename);

        sort(tmp_list.begin(), tmp_list.end());

        file_list_ = tmp_list;
    }

    current_filename_ = filename;
    parseCurrentFile(false);

    if (dialog_)
        dialog_->updateFileList();

    loginf << "filenames '" << file_list_.dump(2) << "'";
    loginf << "current filename '" << current_filename_ << "'";
}

void ManageSectorsTask::removeCurrentFilename()
{
    loginf << "filenames '" << file_list_.dump(2) << "'";
    loginf << "current filename '" << current_filename_ << "'";

    traced_assert(current_filename_.size());
    traced_assert(hasFile(current_filename_));

    auto it = std::find(file_list_.begin(), file_list_.end(), current_filename_);

    if (it == file_list_.end())
    {
        throw std::invalid_argument("ManageSectorsTask: removeCurrentFilename: name '" +
                                    current_filename_ + "' not in use");
    }

    file_list_.erase(it);
    current_filename_ = "";

    parse_message_ = "";
    found_sectors_num_ = 0;

    if (dialog_)
    {
        dialog_->updateFileList();
        dialog_->updateParseMessage();
    }
}

void ManageSectorsTask::removeAllFiles ()
{
    loginf;

    file_list_.clear();
    current_filename_ = "";

    parse_message_ = "";
    found_sectors_num_ = 0;

    if (dialog_)
    {
        dialog_->updateFileList();
        dialog_->updateParseMessage();
    }
}

void ManageSectorsTask::currentFilename(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    current_filename_ = filename;

    if (canImportFile())
        parseCurrentFile(false);
    else
    {
        parse_message_ = "";
        found_sectors_num_ = 0;
    }
}

std::string ManageSectorsTask::parseMessage() const
{
    return parse_message_;
}

//const map<string, map<string, vector<pair<double, double>>>>& ManageSectorsTask::parsedData() const
//{
//    return data_;
//}

bool ManageSectorsTask::canImportFile()
{
    if (!current_filename_.size())
        return false;

    if (!Files::fileExists(current_filename_))
    {
        loginf << "not possible since file '"
               << current_filename_ << "'does not exist";
        return false;
    }

    return true;
}

void ManageSectorsTask::importFile (const std::string& layer_name, bool exclude, QColor color)
{
    traced_assert(canImportFile());

    if (!found_sectors_num_)
    {
        loginf << "not possible since no data found";

        QMessageBox msgBox;
        msgBox.setText("Import not possible since no data was found.");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();

        return;
    }

    layer_name_ = layer_name;
    exclude_ = exclude;
    color_ = color;

    parseCurrentFile(true);

    QMessageBox msgBox;
    msgBox.setText(QString("Import of ")+QString::number(found_sectors_num_)+" sectors done");
    msgBox.setIcon(QMessageBox::Information);

    if (allow_user_interactions_)
        msgBox.exec();

    loginf << "done";
}


void ManageSectorsTask::parseCurrentFile (bool import)
{
    loginf << "file '" << current_filename_ << "' import " << import;

    found_sectors_num_ = 0;
    parse_message_ = "";

    auto sectors = sector_utils::parseGDALFile(current_filename_);

    if (sectors.empty())
    {
        parse_message_ = "file '" + current_filename_ + "' open failed or contains no sectors.";

        if (dialog_)
            dialog_->updateParseMessage();
        return;
    }

    for (const auto& sec : sectors)
    {
        ++found_sectors_num_;

        if (layer_name_.size())
            parse_message_ += "Found layer '" + layer_name_ + "' sector name '" + sec.name
                    + "' num points " + to_string(sec.points.size()) + "\n";
        else
            parse_message_ += "Found sector name '" + sec.name
                    + "' num points " + to_string(sec.points.size()) + "\n";

        if (import)
            addSector(sec.name, sec.points);
    }

    if (import)
        emit manager().compass().dbContextManager().sectorsChangedSignal();

    if (dialog_)
        dialog_->updateParseMessage();
}

void ManageSectorsTask::addSector (const std::string& sector_name, std::vector<std::pair<double,double>> points)
{
    loginf << "layer '" << layer_name_ << "' name '" << sector_name
           << "' num points " << points.size();

    auto& ctx = manager().compass().dbContextManager();

    traced_assert(!ctx.hasSector(sector_name, layer_name_));

    loginf << "adding layer '" << layer_name_ << "' name '" << sector_name;
    ctx.createSector(sector_name, layer_name_, exclude_, color_, points);
}

