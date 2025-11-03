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
#include "json_fwd.hpp"

#include <QObject>

#include <memory>

class ViewPointsImportTaskDialog;
class TaskManager;

class ViewPointsImportTask : public Task, public Configurable
{
    Q_OBJECT

// public slots:
//     void dialogImportSlot();

public:
    ViewPointsImportTask(const std::string& class_id, const std::string& instance_id,
                         TaskManager& task_manager);
    virtual ~ViewPointsImportTask();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    void showDialog();

    void importFilename(const std::string& filename);
    const std::string& importFilename() { return current_filename_; }

    std::string currentError() const;

    bool canImport ();

    virtual bool canRun() override;
    virtual void run() override;
    virtual void stop() override;

    const nlohmann::json& currentData() const;

protected:
    std::string current_filename_;
    nlohmann::json current_data_;

    std::string current_error_;

    virtual void checkSubConfigurables() override {}

    void parseCurrentFile ();
    void checkParsedData (); // throws exceptions for errors
};
