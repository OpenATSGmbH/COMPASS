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

#include "asterixdecodejob.h"
#include "asterixpostprocess.h"
#include "configurable.h"
#include "json_fwd.hpp"
#include "asterixjsonmappingjob.h"
#include "asterixpostprocessjob.h"
#include "asterixjsonparsingschema.h"
#include "asteriximportsource.h"
#include "asterixtimestampcalculator.h"
#include "task.h"
#include "appmode.h"

#include <QObject>
#include <QMessageBox>

#include <memory>


class TaskManager;

class ASTERIXCategoryConfig;
class ASTERIXStatusDialog;

class QProgressDialog;

namespace jASTERIX
{
class jASTERIX;
}

/**
*/
struct ASTERIXFileInfo
{
    std::string filename_;
    //unsigned int line_id_ {0}; // TODO rework

    bool decoding_tried_ {false};
    std::string decoding_info_str_;

    nlohmann::json analysis_info_;
    bool errors_found_{false};
};

/**
*/
class ASTERIXImportTaskSettings
{
public:
    ASTERIXImportTaskSettings(); // defines default param values

    // registered
    bool reset_date_between_files_;
    bool ignore_time_jumps_;
    bool debug_jasterix_;
    std::string current_file_framing_;

    unsigned int num_packets_overload_;

    bool override_tod_active_; // not saved
    double override_tod_offset_;

    bool filter_tod_active_; // not saved
    float filter_tod_min_;
    float filter_tod_max_;

    bool filter_position_active_; // not saved
    double filter_latitude_min_;
    double filter_latitude_max_;
    double filter_longitude_min_;
    double filter_longitude_max_;

    bool filter_modec_active_; // not saved
    float filter_modec_min_;
    float filter_modec_max_;

    // not saved
    unsigned int file_line_id_;
    std::string date_str_;

    bool network_ignore_future_ts_;

    bool obfuscate_secondary_info_;

    // not in config
    boost::posix_time::ptime date_;
    unsigned int max_network_lines_;

    //import chunk sizes
    unsigned int chunk_size_jasterix;
    unsigned int chunk_size_insert;

    unsigned int max_packets_in_processing_{5};
};

/**
*/
class ASTERIXImportTask : public Task, public Configurable
{
    Q_OBJECT

signals:
    void configChanged();
    void decodingStateChanged();
    void sourceUsageChanged();

public slots:
    void decodeASTERIXDoneSlot();
    void decodeASTERIXObsoleteSlot();
    void addDecodedASTERIXSlot();

    void mapJSONDoneSlot();
    void mapJSONObsoleteSlot();

    void timestampCalculationDoneSlot();

    void postprocessDoneSlot();
    void postprocessObsoleteSlot();

    void insertDoneSlot();

    void appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current);

public:
    ASTERIXImportTask(const std::string& class_id, 
                      const std::string& instance_id,
                      TaskManager& task_manager);
    virtual ~ASTERIXImportTask();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    void asterixFileFraming(const std::string& asterix_framing);
    void asterixDecoderConfig(const std::string& asterix_decoder_cfg);

    virtual bool canRun() override;
    virtual void run() override;

    virtual void stop() override;
    bool isRunning() const;

    void runDialog(QWidget* parent = nullptr);

    const ASTERIXImportSource& source() const { return source_; }
    ASTERIXImportSource& source() { return source_; }

    bool requiresFixedFraming() const;

    std::shared_ptr<jASTERIX::jASTERIX> jASTERIX(bool refresh = false) const;
    
    bool hasConfiguratonFor(unsigned int category);
    bool decodeCategory(unsigned int category);
    void decodeCategory(unsigned int category, bool decode);
    std::string editionForCategory(unsigned int category);
    void editionForCategory(unsigned int category, const std::string& edition);
    std::string refEditionForCategory(unsigned int category);
    void refEditionForCategory(unsigned int category, const std::string& ref);
    std::string spfEditionForCategory(unsigned int category);
    void spfEditionForCategory(unsigned int category, const std::string& spf);

    std::shared_ptr<ASTERIXJSONParsingSchema> schema() const;

    unsigned int numPacketsInProcessing() const;

    ASTERIXImportTaskSettings& settings();

    ASTERIXDecoderBase* decoder() { return decoder_.get(); }
    const ASTERIXDecoderBase* decoder() const { return decoder_.get(); }

    bool hasError() const { return error_; }
    const std::string& error() const { return error_message_; }

    void testFileDecoding();

protected:
    virtual void checkSubConfigurables() override;

    void reset();

    void insertData(); // inserts queued job buffers
    void checkAllDone();

    bool maxLoadReached();
    void updateFileProgressDialog(bool force=false);

    void onConfigurationChanged(const std::vector<std::string>& changed_params) override;

    void refreshjASTERIX() const;

    void sourceChanged();

    ASTERIXImportTaskSettings settings_;
    ASTERIXImportSource       source_;

    std::unique_ptr<QProgressDialog> file_progress_dialog_;

    mutable std::shared_ptr<jASTERIX::jASTERIX> jasterix_;
    ASTERIXPostProcess post_process_;

    //sub-configurables
    std::map<unsigned int, ASTERIXCategoryConfig> category_configs_;
    std::shared_ptr<ASTERIXJSONParsingSchema>     schema_;

    std::unique_ptr<ASTERIXDecoderBase> decoder_;
    std::shared_ptr<ASTERIXDecodeJob>   decode_job_;
    
    std::string current_data_source_name_; // used to check for decode file changes

    bool file_decoding_tested_ {false};

    bool running_ {false};

    unsigned int num_packets_in_processing_{0};
    unsigned int num_packets_total_{0};
    unsigned int num_records_ {0};

    std::vector<std::shared_ptr<ASTERIXJSONMappingJob>>         json_map_jobs_;
    ASTERIXTimestampCalculator ts_calculator_;
    //std::future<void> ts_calc_future_;
    std::vector<std::shared_ptr<ASTERIXPostprocessJob>>         postprocess_jobs_;
    std::map<std::string, std::shared_ptr<Buffer>>              accumulated_buffers_;
    std::vector<std::map<std::string, std::shared_ptr<Buffer>>> queued_insert_buffers_;

    boost::posix_time::ptime start_time_;
    boost::posix_time::ptime last_insert_time_;
    boost::posix_time::ptime last_file_progress_time_;

    bool error_{false};
    std::string error_message_;

    bool insert_active_{false};

    std::set<int> added_data_sources_;

    bool insert_slot_connected_ {false};
    bool all_done_{false};
};
