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
#include "result.h"

#include <QObject>

#include <nmeaparse/nmea.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <memory>

class TaskManager;
class GPSTrailImportTaskDialog;
class DBContent;
class Buffer;

struct AsyncTaskState;
class AsyncTaskProgressWrapper;

/**
*/
class GPSTrailImportTask : public Task, public Configurable
{
    Q_OBJECT

public:
    struct Settings
    {
        Settings();

        std::string  ds_name;
        unsigned int ds_sac;
        unsigned int ds_sic;

        bool         use_tod_offset;
        double       tod_offset;

        bool         use_override_date;
        std::string  override_date_str;
        
        bool         set_mode_3a_code;
        unsigned int mode_3a_code;        // decimal
        bool         set_target_address;
        unsigned int target_address;      // decimal
        bool         set_callsign;
        std::string  callsign;
    };

    GPSTrailImportTask(nlohmann::json& config, TaskManager* parent);
    virtual ~GPSTrailImportTask();

    GPSTrailImportTaskDialog* dialog();

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

    bool useTodOffset() const;
    void useTodOffset(bool value);

    double todOffset() const;
    void todOffset(double value);

    bool useOverrideDate() const;
    void useOverrideDate(bool value);

    const boost::gregorian::date& overrideDate() const;
    void overrideDate(const boost::gregorian::date& date);

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

public slots:
    void insertDoneSlot();

    void dialogImportSlot();
    void dialogCancelSlot();

signals:
    void configChanged();
    void fileChanged();

protected:
    virtual void onConfigurationChanged(const std::vector<std::string>& changed_params) override;

    void parseCurrentFile ();
    Result parseCurrentFileImpl (const AsyncTaskState& state, AsyncTaskProgressWrapper& progress);
    //void checkParsedData (); // throws exceptions for errors

    void updateDateFromString();

    std::string            current_filename_;
    boost::gregorian::date override_date_;
    unsigned int           line_id_ = 0;

    Settings settings_;

    std::unique_ptr<GPSTrailImportTaskDialog> dialog_;

    std::string current_error_;
    std::string current_text_;

    std::vector<nmea::GPSFix> gps_fixes_;

    std::map<unsigned int, unsigned int> quality_counts_;
    unsigned int gps_fixes_zero_datetime_ {0};
    unsigned int gps_fixes_cnt_ {0};
    //unsigned int gps_fixes_skipped_lost_lock_ {0}; removed since rtk does not set GPS lock
    unsigned int gps_fixes_skipped_quality_cnt_ {0};
    unsigned int gps_fixes_skipped_time_cnt_ {0};
    unsigned int gps_fixes_without_speedvec_ {0};

    const std::map<unsigned int, std::string> quality_labels {
        {0, "Invalid"},
        {1, "Standard"},
        {2, "DGPS"},
        {3, "PPS fix"},
        {4, "Real Time Kinetic"},
        {5, "Real Time Kinetic (float)"},
        {6, "Estimate"},
        {7, "Manual input"},
        {8, "Simulation mode"}
    };

    std::shared_ptr<Buffer> buffer_;
};

