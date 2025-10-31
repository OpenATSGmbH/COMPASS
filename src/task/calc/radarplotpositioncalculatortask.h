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
#include "dbcontent/variable/variableset.h"
#include "radarplotpositioncalculatortaskdialog.h"

#include <QObject>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <memory>
#include <set>

class Buffer;
class DBContent;

class TaskManager;
class UpdateBufferDBJob;

namespace dbContent {
class Variable;
}

class QMessageBox;

class RadarPlotPositionCalculatorTask : public Task, public Configurable
{
    Q_OBJECT

public slots:
    void loadedDataSlot(const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset);
    void loadingDoneSlot();

    void updateDoneSlot(DBContent& db_content);

public:
    RadarPlotPositionCalculatorTask(const std::string& class_id, const std::string& instance_id,
                                    TaskManager& task_manager);
    virtual ~RadarPlotPositionCalculatorTask();

    bool isCalculating();

    void showDialog();

    virtual bool canRun() override;
    virtual void run() override;

protected:
    boost::posix_time::ptime start_time_;
    boost::posix_time::ptime stop_time_;

    bool calculating_{false};

    std::map<std::string, std::shared_ptr<Buffer>> data_;
    std::set<std::string> dbcontent_done_;

    QMessageBox* msg_box_{nullptr};

    dbContent::VariableSet getReadSetFor(const std::string& dbcontent_name);
};

