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
#include "jsonparsingschema.h"
#include "task.h"
#include "asterixpostprocess.h"
#include "asterixpostprocessjob.h"
#include "asterixtimestampcalculator.h"

#include <QObject>

#include <memory>
#include <set>


class TaskManager;
class JSONImportTaskDialog;
class JSONParseJob;
class JSONMappingJob;
class ReadJSONFileJob;
class DBContent;
class Buffer;

namespace dbContent
{
    class VariableSet;
}

class QMessageBox;

class JSONImportTask : public Task, public Configurable
{
    Q_OBJECT

    using JSONParsingSchemaIterator = std::map<std::string, std::shared_ptr<JSONParsingSchema>>::iterator;

  public slots:
    void dialogImportSlot();
    void dialogTestImportSlot();
    void dialogCancelSlot();

    void addReadJSONSlot();
    void readJSONFileDoneSlot();
    void readJSONFileObsoleteSlot();

    void parseJSONDoneSlot();
    void parseJSONObsoleteSlot();

    void mapJSONDoneSlot();
    void mapJSONObsoleteSlot();

    void postprocessDoneSlot();
    void postprocessObsoleteSlot();

    void insertDoneSlot();

  public:
    JSONImportTask(const std::string& class_id, const std::string& instance_id,
                   TaskManager& task_manager);
    virtual ~JSONImportTask();

    JSONImportTaskDialog* dialog();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    bool canImportFile();

    virtual bool canRun() override;
    virtual void run() override;

    void importFilename(const std::string& filename);
    const std::string& importFilename() { return import_filename_; }

    JSONParsingSchemaIterator begin() { return schemas_.begin(); }
    JSONParsingSchemaIterator end() { return schemas_.end(); }
    bool hasSchema(const std::string& name);
    bool hasCurrentSchema() const;
    bool hasJSONSchema() const;
    std::shared_ptr<JSONParsingSchema> currentJSONSchema();
    void removeCurrentSchema();

    std::string currentSchemaName() const;
    void currentSchemaName(const std::string& currentSchema);

    void test(bool test);

    size_t objectsInserted() const;

    unsigned int fileLineID() const;
    void fileLineID(unsigned int value);

    const boost::posix_time::ptime &date() const;
    void date(const boost::posix_time::ptime& date);

  protected:
    std::string import_filename_;

    std::unique_ptr<JSONImportTaskDialog> dialog_;

    std::string current_schema_name_;
    std::map<std::string, std::shared_ptr<JSONParsingSchema>> schemas_;

    unsigned int file_line_id_ {0};
    boost::posix_time::ptime date_;

    ASTERIXPostProcess post_process_;

    size_t insert_active_{0};

    std::map<std::string, std::tuple<std::string, dbContent::VariableSet>> dbcont_variable_sets_;
    std::set<int> added_data_sources_;

    std::shared_ptr<ReadJSONFileJob> read_json_job_;
    std::shared_ptr<JSONParseJob> json_parse_job_;
    std::vector<std::shared_ptr<JSONMappingJob>> json_map_jobs_;
    ASTERIXTimestampCalculator ts_calculator_;
    std::vector<std::shared_ptr<ASTERIXPostprocessJob>> postprocess_jobs_;

    bool test_{false};

    boost::posix_time::ptime start_time_;
    boost::posix_time::ptime stop_time_;

    size_t bytes_read_{0};
    size_t bytes_to_read_{0};
    float read_status_percent_{0.0};

    size_t objects_read_{0};
    size_t objects_parsed_{0};
    size_t objects_parse_errors_{0};

    size_t objects_mapped_{0};
    size_t objects_not_mapped_{0};

    size_t objects_created_{0};
    size_t records_inserted_{0};

    //unsigned int num_radar_inserted_ {0};
    bool insert_slot_connected_ {false};
    bool all_done_{false};

    size_t statistics_calc_objects_inserted_{0};
    std::string object_rate_str_;
    std::string remaining_time_str_;

    std::unique_ptr<QMessageBox> msg_box_;

    void insertData(std::map<std::string, std::shared_ptr<Buffer>> job_buffers);

    void checkAllDone();

    void updateMsgBox();

    bool maxLoadReached();

    virtual void checkSubConfigurables() override {}
};

