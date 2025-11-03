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
#include "projection/transformation.h"
#include "task.h"

#include <QObject>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <memory>

class TaskManager;
class GPSImportCSVTaskDialog;
class DBContent;
class Buffer;

struct GPSPosition
{
    boost::posix_time::ptime timestamp_;
    double latitude_{0}; // deg
    double longitude_{0};  // deg
    double altitude_{0}; // m msl
    bool has_speed_{false};
    double vx_{0}, vy_{0}; // m/s
    double track_angle_{0}; // deg
    double speed_{0}; // m/s
};

class GPSImportCSVTask : public Task, public Configurable
{
    Q_OBJECT

public slots:
    void insertDoneSlot();

    void dialogImportSlot();
    void dialogCancelSlot();

public:
    GPSImportCSVTask(const std::string& class_id, const std::string& instance_id,
                       TaskManager& task_manager);
    virtual ~GPSImportCSVTask();

    GPSImportCSVTaskDialog* dialog();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    bool canImportFile();

    virtual bool canRun() override;
    virtual void run() override;

    void importFilename(const std::string& filename);
    const std::string& importFilename() { return current_filename_; }

    virtual bool checkPrerequisites();
    virtual bool isRecommended();
    virtual bool isRequired();

    std::string currentText() const;
    std::string currentError() const;

    std::string dsName() const;
    void dsName(const std::string& ds_name);

    unsigned int dsSAC() const;
    void dsSAC(unsigned int sac);

    unsigned int dsSIC() const;
    void dsSIC(unsigned int sic);

    float todOffset() const;
    void todOffset(float value);

    bool setMode3aCode() const;
    void setMode3aCode(bool value);

    unsigned int mode3aCode() const;
    void mode3aCode(unsigned int value);

    bool setTargetAddress() const;
    void setTargetAddress(bool value);

    unsigned int targetAddress() const;
    void targetAddress(unsigned int value);

    bool setCallsign() const;
    void setCallsign(bool value);

    std::string callsign() const;
    void callsign(const std::string& callsign);

    unsigned int lineID() const;
    void lineID(unsigned int line_id);

protected:
    std::string current_filename_;

    std::string timestamp_format_;

    std::string ds_name_;
    unsigned int ds_sac_ {0};
    unsigned int ds_sic_ {0};

    float tod_offset_ {0};

    bool set_mode_3a_code_;
    unsigned int mode_3a_code_; // decimal
    bool set_target_address_;
    unsigned int target_address_; // decimal
    bool set_callsign_;
    std::string callsign_;

    unsigned int line_id_ {0};

    std::unique_ptr<GPSImportCSVTaskDialog> dialog_;

    Transformation trafo_;

    std::string current_error_;
    std::string current_text_;

    std::vector<GPSPosition> gps_positions_;

    std::shared_ptr<Buffer> buffer_;

    virtual void checkSubConfigurables() override {}

    void parseCurrentFile ();
    //void checkParsedData (); // throws exceptions for errors
};
