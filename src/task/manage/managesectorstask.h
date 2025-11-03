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
#include "task.h"

#include <QObject>
#include <QColor>

#include <memory>

class TaskManager;
class ManageSectorsTaskDialog;
class Sector;
class OGRPolygon;
class OGRLinearRing;

class ManageSectorsTask : public Task, public Configurable
{
    Q_OBJECT

public slots:
    void dialogDoneSlot();

public:
    ManageSectorsTask(const std::string& class_id, const std::string& instance_id,
                      TaskManager& task_manager);
    virtual ~ManageSectorsTask();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    ManageSectorsTaskDialog* dialog();

    bool canImportFile();
    void importFile (const std::string& layer_name, bool exclude, QColor color);

    virtual void run() {} // TODO doesnt fit

    bool hasFile(const std::string& filename) const;
    std::vector<std::string> fileList() const;
    void addFile(const std::string& filename);
    void removeCurrentFilename();
    void removeAllFiles ();
    void currentFilename(const std::string& filename);
    const std::string& currentFilename() { return current_filename_; }

    std::string parseMessage() const;
    //std::vector<std::shared_ptr<Sector>>& parsedData() const;

protected:
    nlohmann::json file_list_;
    std::string current_filename_;

    std::unique_ptr<ManageSectorsTaskDialog> dialog_;

    unsigned int found_sectors_num_{0};
    std::string parse_message_;

    std::string layer_name_;
    bool exclude_;
    QColor color_;

    virtual void checkSubConfigurables() {}

    void parseCurrentFile (bool import);

    void addPolygon (const std::string& sector_name, OGRPolygon& polygon, bool import);
    void addLinearRing (const std::string& sector_name, OGRLinearRing& ring, bool import);

    void addSector (const std::string& sector_name, std::vector<std::pair<double,double>> points);
};
