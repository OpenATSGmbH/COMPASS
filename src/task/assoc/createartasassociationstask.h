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
#include "createartasassociationsjob.h"
#include "createartasassociationsstatusdialog.h"
#include "dbcontent/variable/variableset.h"
#include "task.h"

#include <QObject>
#include <memory>

#include "boost/date_time/posix_time/posix_time.hpp"

class TaskManager;
class CreateARTASAssociationsTaskDialog;

class DBContent;
class Buffer;

namespace dbContent {
class Variable;
class MetaVariable;
}

class CreateARTASAssociationsTask : public Task, public Configurable
{
    Q_OBJECT

public slots:
    void dialogRunSlot();

    void createDoneSlot();
    void createObsoleteSlot();

    void loadedDataDataSlot(const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset);
    void loadingDoneSlot();

    void associationStatusSlot(QString status);
    void saveAssociationsQuestionSlot(QString question_str);

    void closeStatusDialogSlot();

signals:
    void dataSourceChanged();

public:
    enum class Error
    {
        NoError = 0,
        NoDataSource,
        NoDataForLineID
    };

    struct Settings
    {
        std::string current_data_source_name_;
        unsigned int current_data_source_line_id_{0};

        float end_track_time_{300.0f};  // time-delta after which begin a new track

        float association_time_past_{60.0f};  // time_delta for which associations are considered into past time
        float association_time_future_{2.0f};  // time_delta for which associations are considered into future time

        float misses_acceptable_time_{60.0f};  // time delta at beginning/end of recording where misses are acceptable

        float associations_dubious_distant_time_{30.0f};
        // time delta of tou where association is dubious bc too distant in time
        float association_dubious_close_time_past_{20.0f};
        // time delta of tou where association is dubious when multible hashes exist
        float association_dubious_close_time_future_{1.0f};
        // time delta of tou where association is dubious when multible hashes exist

        bool ignore_track_end_associations_{true};
        bool mark_track_end_associations_dubious_{false};

        bool ignore_track_coasting_associations_{true};
        bool mark_track_coasting_associations_dubious_{false};
    };

    CreateARTASAssociationsTask(const std::string& class_id, const std::string& instance_id,
                                TaskManager& task_manager);
    virtual ~CreateARTASAssociationsTask();

    void showDialog();

    std::string currentDataSourceName() const;
    void currentDataSourceName(const std::string& currentDataSourceName);

    unsigned int currentDataSourceLineID() const;
    void currentDataSourceLineID(unsigned int line_id);

    float endTrackTime() const;
    void endTrackTime(float end_track_time);

    float associationTimePast() const;
    void associationTimePast(float association_time_past);

    float associationTimeFuture() const;
    void associationTimeFuture(float association_time_future);

    float missesAcceptableTime() const;
    void missesAcceptableTime(float misses_acceptable_time);

    float associationsDubiousDistantTime() const;
    void associationsDubiousDistantTime(float associations_dubious_distant_time);

    float associationDubiousCloseTimePast() const;
    void associationDubiousCloseTimePast(float association_dubious_close_time_past);

    float associationDubiousCloseTimeFuture() const;
    void associationDubiousCloseTimeFuture(float association_dubious_close_time_future);

    bool ignoreTrackEndAssociations() const;
    void ignoreTrackEndAssociations(bool value);

    bool markTrackEndAssociationsDubious() const;
    void markTrackEndAssociationsDubious(bool value);

    bool ignoreTrackCoastingAssociations() const;
    void ignoreTrackCoastingAssociations(bool value);

    bool markTrackCoastingAssociationsDubious() const;
    void markTrackCoastingAssociationsDubious(bool value);

    virtual bool canRun() override;
    virtual void run() override;

    bool wasRun();

    Error checkError() const;

    static const std::string DONE_PROPERTY_NAME;

protected:
    Settings settings_;

    boost::posix_time::ptime start_time_;
    boost::posix_time::ptime stop_time_;

    bool save_associations_{true};

    std::map<std::string, std::shared_ptr<Buffer>> data_;

    std::unique_ptr<CreateARTASAssociationsStatusDialog> status_dialog_{nullptr};

    bool dbcont_loading_done_{false};

    std::shared_ptr<CreateARTASAssociationsJob> create_job_;
    bool create_job_done_{false};

    dbContent::VariableSet getReadSetFor(const std::string& dbcontent_name);
};
