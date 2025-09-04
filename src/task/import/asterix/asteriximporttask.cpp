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

#include "asteriximporttask.h"
#include "asterixcategoryconfig.h"
#include "compass.h"
#include "buffer.h"
#include "configurable.h"
#include "dbinterface.h"
#include "dbcontent/dbcontentmanager.h"
#include "datasourcemanager.h"
#include "files.h"
#include "jobmanager.h"
#include "logger.h"
#include "asteriximporttaskdialog.h"
#include "util/stringconv.h"
#include "system.h"
#include "taskmanager.h"
#include "mainwindow.h"
#include "util/timeconv.h"
#include "projection.h"
#include "projectionmanager.h"
#include "asynctask.h"
#include "dbcontent.h"

#include <jasterix/category.h>
#include <jasterix/edition.h>
#include <jasterix/jasterix.h>
#include <jasterix/refedition.h>

#include <QApplication>
#include <QCoreApplication>
#include <QMessageBox>
#include <QThread>
#include <QProgressDialog>
#include <QMessageBox>
#include <QPushButton>

#include <algorithm>
#include <malloc.h>

using namespace Utils;
using namespace nlohmann;
using namespace std;

//const float ram_threshold = 4.0;

/**
*/
ASTERIXImportTaskSettings::ASTERIXImportTaskSettings()
    :   reset_date_between_files_ (true)
    ,   ignore_time_jumps_        (false)
    ,   debug_jasterix_           (false)
    ,   current_file_framing_     ("")
    ,   num_packets_overload_     (60)
    ,   override_tod_active_      (false)
    ,   override_tod_offset_      (0.0f)
    ,   filter_tod_active_        (false)
    ,   filter_tod_min_           (0.0f)
    ,   filter_tod_max_           (24*3600.0 - 1)
    ,   filter_position_active_   (false)
    ,   filter_latitude_min_      (-90.0)
    ,   filter_latitude_max_      ( 00.0)
    ,   filter_longitude_min_     (-180.0)
    ,   filter_longitude_max_     ( 180.0)
    ,   filter_modec_active_      (false)
    ,   filter_modec_min_         (-10000.0f)
    ,   filter_modec_max_         ( 50000.0f)
    ,   file_line_id_             (0)
    ,   date_str_                 ()
    ,   network_ignore_future_ts_ (false)
    ,   obfuscate_secondary_info_ (false)
    ,   date_                     ()
    ,   max_network_lines_        (4)
    ,   chunk_size_jasterix       (2000)
    ,   chunk_size_insert         (50000)
{
}

/**
*/
ASTERIXImportTask::ASTERIXImportTask(const std::string& class_id, 
                                     const std::string& instance_id,
                                     TaskManager& task_manager)
    : Task(task_manager),
    Configurable(class_id, instance_id, &task_manager, "task_import_asterix.json")
{
    tooltip_ = "Allows importing of ASTERIX data recording files into the opened database.";

    registerParameter("reset_date_between_files", &settings_.reset_date_between_files_,
                      ASTERIXImportTaskSettings().reset_date_between_files_);
    registerParameter("debug_jasterix", &settings_.debug_jasterix_, ASTERIXImportTaskSettings().debug_jasterix_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "debug_jasterix");

    registerParameter("current_file_framing", &settings_.current_file_framing_, ASTERIXImportTaskSettings().current_file_framing_);

    registerParameter("num_packets_overload", &settings_.num_packets_overload_, ASTERIXImportTaskSettings().num_packets_overload_);

    registerParameter("override_tod_active", &settings_.override_tod_active_,
                      ASTERIXImportTaskSettings().override_tod_active_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "override_tod_active");
    registerParameter("override_tod_offset", &settings_.override_tod_offset_,
                      ASTERIXImportTaskSettings().override_tod_offset_);

    registerParameter("filter_tod_active", &settings_.filter_tod_active_,
                      ASTERIXImportTaskSettings().filter_tod_active_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_tod_active");
    registerParameter("filter_tod_min", &settings_.filter_tod_min_, ASTERIXImportTaskSettings().filter_tod_min_);
    registerParameter("filter_tod_max", &settings_.filter_tod_max_, ASTERIXImportTaskSettings().filter_tod_max_);

    registerParameter("filter_position_active", &settings_.filter_position_active_,
                      ASTERIXImportTaskSettings().filter_position_active_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_position_active");
    registerParameter("filter_latitude_min", &settings_.filter_latitude_min_,
                      ASTERIXImportTaskSettings().filter_latitude_min_);
    registerParameter("filter_latitude_max", &settings_.filter_latitude_max_,
                      ASTERIXImportTaskSettings().filter_latitude_max_);
    registerParameter("filter_longitude_min", &settings_.filter_longitude_min_,
                      ASTERIXImportTaskSettings().filter_longitude_min_);
    registerParameter("filter_longitude_max", &settings_.filter_longitude_max_,
                      ASTERIXImportTaskSettings().filter_longitude_max_);

    registerParameter("filter_modec_active", &settings_.filter_modec_active_,
                      ASTERIXImportTaskSettings().filter_modec_active_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_modec_active");
    registerParameter("filter_modec_min", &settings_.filter_modec_min_, ASTERIXImportTaskSettings().filter_modec_min_);
    registerParameter("filter_modec_max", &settings_.filter_modec_max_, ASTERIXImportTaskSettings().filter_modec_max_);

    registerParameter("file_line_id", &settings_.file_line_id_, ASTERIXImportTaskSettings().file_line_id_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "file_line_id");
    registerParameter("date_str", &settings_.date_str_, ASTERIXImportTaskSettings().date_str_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "date_str");

    if (settings_.date_str_.size())
        settings_.date_ = Time::fromDateString(settings_.date_str_);
    if (settings_.date_.is_not_a_date_time())
        settings_.date_ = boost::posix_time::ptime(boost::gregorian::day_clock::universal_day());

    registerParameter("network_ignore_future_ts", &settings_.network_ignore_future_ts_,
                      ASTERIXImportTaskSettings().network_ignore_future_ts_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "network_ignore_future_ts");
    registerParameter("obfuscate_secondary_info", &settings_.obfuscate_secondary_info_,
                      ASTERIXImportTaskSettings().obfuscate_secondary_info_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "obfuscate_secondary_info");

    registerParameter("chunk_size_jasterix", &settings_.chunk_size_jasterix, ASTERIXImportTaskSettings().chunk_size_jasterix);
    registerParameter("chunk_size_insert", &settings_.chunk_size_insert, ASTERIXImportTaskSettings().chunk_size_insert);

    std::string jasterix_definition_path = HOME_DATA_DIRECTORY + "jasterix_definitions";

    loginf << "jasterix definition path '"
           << jasterix_definition_path << "'";
    traced_assert(Files::directoryExists(jasterix_definition_path));

    jASTERIX::frame_chunk_size      = settings_.chunk_size_jasterix;
    jASTERIX::data_block_chunk_size = settings_.chunk_size_jasterix;

    refreshjASTERIX(); // needed for available framings check etc.

    createSubConfigurables();

    connect(&source_, &ASTERIXImportSource::changed, this, &ASTERIXImportTask::sourceChanged);
    connect(&source_, &ASTERIXImportSource::fileUsageChanged, this, &ASTERIXImportTask::sourceUsageChanged);

    registerParameter("max_packets_in_processing", &settings_.max_packets_in_processing_,
                      settings_.max_packets_in_processing_);

    logdbg << "thread " << QThread::currentThreadId()
           << " main " << QApplication::instance()->thread()->currentThreadId();
}

/**
*/
ASTERIXImportTask::~ASTERIXImportTask()
{
    logdbg << "start";
}

/**
*/
void ASTERIXImportTask::generateSubConfigurable(const std::string& class_id,
                                                const std::string& instance_id)
{
    if (class_id == "ASTERIXCategoryConfig")
    {
        unsigned int category = getSubConfiguration(class_id, instance_id).getParameterConfigValue<unsigned int>("category");

        traced_assert(category_configs_.find(category) == category_configs_.end());

        logdbg << "generating asterix config "
               << instance_id << " with cat " << category;

        category_configs_.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(category),                                // args for key
            std::forward_as_tuple(category, class_id, instance_id, this));  // args for mapped value

        logdbg << "cat " << category << " decode "
               << category_configs_.at(category).decode() << " edition '"
               << category_configs_.at(category).edition() << "' ref '"
               << category_configs_.at(category).ref() << "'";
    }
    else if (class_id == "ASTERIXJSONParsingSchema")
    {
        std::string name = getSubConfiguration(class_id, instance_id).getParameterConfigValue<std::string>("name");

        traced_assert(schema_ == nullptr);
        traced_assert(name == "jASTERIX");

        logdbg << "generating schema " << instance_id
               << " with name " << name;

        schema_.reset(new ASTERIXJSONParsingSchema(class_id, instance_id, *this));
    }
    else
    {
        throw std::runtime_error("ASTERIXImportTask: generateSubConfigurable: unknown class_id " + class_id);
    }
}

/**
*/
void ASTERIXImportTask::asterixFileFraming(const std::string& asterix_framing)
{
    loginf << "framing '" << asterix_framing << "'";

    traced_assert(jasterix_);
    std::vector<std::string> framings = jasterix_->framings();

    if (asterix_framing != ""
        && std::find(framings.begin(), framings.end(), asterix_framing) == framings.end())
        throw runtime_error ("ASTERIXImportTask: unknown framing '"+asterix_framing+"'");

    settings_.current_file_framing_ = asterix_framing;
}

/**
*/
void ASTERIXImportTask::asterixDecoderConfig(const std::string& asterix_decoder_cfg)
{
    loginf << "config string '" << asterix_decoder_cfg << "'";

    traced_assert(jasterix_);

    json config = json::parse(asterix_decoder_cfg);

    if (!config.is_object())
        throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: json config is not an object");

    for (auto& cat_it : config.items())
    {
        std::string cat_str = cat_it.key();

        unsigned int cat = stoi (cat_str);

        if (!hasConfiguratonFor(cat))
            throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: unknown cat "+to_string(cat)
                                +" from '" + cat_str + "'");

        json& cat_cfg = cat_it.value();
        if (!cat_cfg.is_object())
            throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                +" config is not an object");

        if (cat_cfg.contains("edition"))
        {
            if (!cat_cfg.at("edition").is_string())
                throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                    +" edition is not a string");

            string edition = cat_cfg.at("edition");

            if (!jasterix_->category(cat)->hasEdition(edition))
                throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                    +" has no edition '"+edition+"'");

            loginf << "setting cat " << cat
                   << " edition " << edition;

            category_configs_.at(cat).edition(edition);
        }

        if (cat_cfg.contains("ref_edition"))
        {
            if (!cat_cfg.at("ref_edition").is_string())
                throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                    +" ref edition is not a string");

            string ref_ed = cat_cfg.at("ref_edition");

            if (!jasterix_->category(cat)->hasREFEdition(ref_ed))
                throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                    +" has no ref edition '"+ref_ed+"'");

            loginf << "setting cat " << cat
                   << " ref edition " << ref_ed;

            category_configs_.at(cat).ref(ref_ed);
        }

        if (cat_cfg.contains("spf_edition"))
        {
            if (!cat_cfg.at("spf_edition").is_string())
                throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                    +" spf edition is not a string");

            string spf_ed = cat_cfg.at("spf_edition");

            if (!jasterix_->category(cat)->hasSPFEdition(spf_ed))
                throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                    +" has no spf edition '"+spf_ed+"'");

            loginf << "setting cat " << cat
                   << " spf edition " << spf_ed;

            category_configs_.at(cat).spf(spf_ed);
        }
    }
}

/**
*/
void ASTERIXImportTask::checkSubConfigurables()
{
    if (schema_ == nullptr)
    {
        auto config = Configuration::create("JSONParsingSchema", "JSONParsingSchemajASTERIX0");
        config->addParameter<std::string>("name", "jASTERIX");
        generateSubConfigurableFromConfig(std::move(config));
    }
}

/**
*/
void ASTERIXImportTask::sourceChanged()
{
    loginf << "new type = " << source_.sourceTypeAsString();

    //update to suitable decoder
    decoder_ = ASTERIXDecoderBase::createDecoder(source_);
    traced_assert(decoder_);

    loginf << "created new decoder "  << decoder_->name();

    //switch to framing required by the decoder?
    auto required_framing = decoder_->requiredASTERIXFraming();
    if (required_framing.has_value())
        settings_.current_file_framing_ = required_framing.value();

    //test decoding
    testFileDecoding();
}

/**
*/
bool ASTERIXImportTask::requiresFixedFraming() const
{
    return (decoder_ && decoder_->requiredASTERIXFraming().has_value());
}

/**
*/
std::shared_ptr<jASTERIX::jASTERIX> ASTERIXImportTask::jASTERIX(bool refresh) const
{
    if (refresh)
        refreshjASTERIX();

    traced_assert(jasterix_);

    return jasterix_;
}

/**
*/
void ASTERIXImportTask::refreshjASTERIX() const
{
    std::string jasterix_definition_path = HOME_DATA_DIRECTORY + "jasterix_definitions";

    logdbg << "jasterix definition path '"
           << jasterix_definition_path << "'";
    traced_assert(Files::directoryExists(jasterix_definition_path));

    jasterix_ = std::make_shared<jASTERIX::jASTERIX>(jasterix_definition_path, false,
                                                     settings_.debug_jasterix_, true);

    std::vector<std::string> framings = jasterix_->framings();
    if (std::find(framings.begin(), framings.end(), settings_.current_file_framing_) == framings.end())
    {
        logdbg << "resetting to no framing";

        //@TODO: thats not nice...
        ASTERIXImportTaskSettings& settings = const_cast<ASTERIXImportTaskSettings&>(settings_);
        settings.current_file_framing_ = "";
    }

    // set category configs
    jasterix_->decodeNoCategories();

    for (auto& cat_it : category_configs_)
    {
        // loginf << "setting category " << cat_it.first;

        logdbg << "setting cat " << cat_it.first << " decode "
               << cat_it.second.decode() << " edition '" << cat_it.second.edition() << "' ref '"
               << cat_it.second.ref() << "'";

        if (!jasterix_->hasCategory(cat_it.first))
        {
            logwrn << "cat '" << cat_it.first
                   << "' not defined in decoder";
            continue;
        }

        if (!jasterix_->category(cat_it.first)->hasEdition(cat_it.second.edition()))
        {
            logwrn << "cat " << cat_it.first << " edition '"
                   << cat_it.second.edition() << "' not defined in decoder";
            continue;
        }

        if (cat_it.second.ref().size() &&  // only if value set
            !jasterix_->category(cat_it.first)->hasREFEdition(cat_it.second.ref()))
        {
            logwrn << "cat " << cat_it.first << " ref '"
                   << cat_it.second.ref() << "' not defined in decoder";
            continue;
        }

        if (cat_it.second.spf().size() &&  // only if value set
            !jasterix_->category(cat_it.first)->hasSPFEdition(cat_it.second.spf()))
        {
            logwrn << "cat " << cat_it.first << " spf '"
                   << cat_it.second.spf() << "' not defined in decoder";
            continue;
        }

        //        loginf << "setting cat " <<  cat_it.first
        //               << " decode flag " << cat_it.second.decode();
        jasterix_->setDecodeCategory(cat_it.first, cat_it.second.decode());
        logdbg << "setting cat " <<  cat_it.first
               << " edition " << cat_it.second.edition();
        jasterix_->category(cat_it.first)->setCurrentEdition(cat_it.second.edition());
        jasterix_->category(cat_it.first)->setCurrentREFEdition(cat_it.second.ref());
        jasterix_->category(cat_it.first)->setCurrentSPFEdition(cat_it.second.spf());
    }
}

/**
*/
bool ASTERIXImportTask::hasConfiguratonFor(unsigned int category)
{
    return category_configs_.count(category) > 0;
}

/**
*/
bool ASTERIXImportTask::decodeCategory(unsigned int category)
{
    traced_assert(hasConfiguratonFor(category));
    return category_configs_.at(category).decode();
}

/**
*/
void ASTERIXImportTask::decodeCategory(unsigned int category, bool decode)
{
    traced_assert(jasterix_->hasCategory(category));

    loginf << "cat " << category << " decode " << decode;

    if (!hasConfiguratonFor(category))
    {
        auto new_cfg = Configuration::create("ASTERIXCategoryConfig");
        new_cfg->addParameter<unsigned int>("category", category);
        new_cfg->addParameter<bool>("decode", decode);
        new_cfg->addParameter<std::string>("edition", jasterix_->category(category)->defaultEdition());
        new_cfg->addParameter<std::string>("ref", jasterix_->category(category)->defaultREFEdition());

        generateSubConfigurableFromConfig(std::move(new_cfg));
        traced_assert(hasConfiguratonFor(category));
    }
    else
        category_configs_.at(category).decode(decode);

    testFileDecoding();
}

/**
*/
std::string ASTERIXImportTask::editionForCategory(unsigned int category)
{
    traced_assert(hasConfiguratonFor(category));

    // check if edition exists, otherwise rest to default
    if (jasterix_->category(category)->editions().count(category_configs_.at(category).edition()) ==
        0)
    {
        loginf << "cat " << category
               << " reset to default edition";
        category_configs_.at(category).edition(jasterix_->category(category)->defaultEdition());
    }

    return category_configs_.at(category).edition();
}

/**
*/
void ASTERIXImportTask::editionForCategory(unsigned int category, const std::string& edition)
{
    traced_assert(jasterix_->hasCategory(category));

    loginf << "cat " << category << " edition " << edition;

    if (!hasConfiguratonFor(category))
    {
        auto new_cfg = Configuration::create("ASTERIXCategoryConfig");
        new_cfg->addParameter<unsigned int>("category", category);
        new_cfg->addParameter<bool>("decode", false);
        new_cfg->addParameter<std::string>("edition", edition);
        new_cfg->addParameter<std::string>("ref", jasterix_->category(category)->defaultREFEdition());

        generateSubConfigurableFromConfig(std::move(new_cfg));
        traced_assert(hasConfiguratonFor(category));
    }
    else
        category_configs_.at(category).edition(edition);

    testFileDecoding();
}

/**
*/
std::string ASTERIXImportTask::refEditionForCategory(unsigned int category)
{
    traced_assert(hasConfiguratonFor(category));

    // check if edition exists, otherwise rest to default
    if (category_configs_.at(category).ref().size() &&  // if value set and not exist in jASTERIX
        jasterix_->category(category)->refEditions().count(category_configs_.at(category).ref()) ==
            0)
    {
        loginf << "cat " << category
               << " reset to default ref";
        category_configs_.at(category).ref(jasterix_->category(category)->defaultREFEdition());
    }

    return category_configs_.at(category).ref();
}

/**
*/
void ASTERIXImportTask::refEditionForCategory(unsigned int category, const std::string& ref)
{
    traced_assert(jasterix_->hasCategory(category));

    loginf << "cat " << category << " ref '" << ref << "'";

    if (!hasConfiguratonFor(category))
    {
        auto new_cfg = Configuration::create("ASTERIXCategoryConfig");
        new_cfg->addParameter<unsigned int>("category", category);
        new_cfg->addParameter<bool>("decode", false);
        new_cfg->addParameter<std::string>("edition", jasterix_->category(category)->defaultEdition());
        new_cfg->addParameter<std::string>("ref", ref);

        generateSubConfigurableFromConfig(std::move(new_cfg));
        traced_assert(hasConfiguratonFor(category));
    }
    else
        category_configs_.at(category).ref(ref);

    testFileDecoding();
}

/**
*/
std::string ASTERIXImportTask::spfEditionForCategory(unsigned int category)
{
    traced_assert(hasConfiguratonFor(category));

    // check if edition exists, otherwise rest to default
    if (category_configs_.at(category).spf().size() &&  // if value set and not exist in jASTERIX
        jasterix_->category(category)->spfEditions().count(category_configs_.at(category).spf()) ==
            0)
    {
        loginf << "cat " << category
               << " reset to default spf";
        category_configs_.at(category).spf(jasterix_->category(category)->defaultSPFEdition());
    }

    return category_configs_.at(category).spf();
}

/**
*/
void ASTERIXImportTask::spfEditionForCategory(unsigned int category, const std::string& spf)
{
    traced_assert(jasterix_->hasCategory(category));

    loginf << "cat " << category << " spf '" << spf
           << "'";

    if (!hasConfiguratonFor(category))
    {
        auto new_cfg = Configuration::create("ASTERIXCategoryConfig");
        new_cfg->addParameter<unsigned int>("category", category);
        new_cfg->addParameter<bool>("decode", false);
        new_cfg->addParameter<std::string>("edition", jasterix_->category(category)->defaultEdition());
        new_cfg->addParameter<std::string>("spf", spf);

        generateSubConfigurableFromConfig(std::move(new_cfg));
        traced_assert(hasConfiguratonFor(category));
    }
    else
        category_configs_.at(category).spf(spf);

    testFileDecoding();
}

/**
*/
std::shared_ptr<ASTERIXJSONParsingSchema> ASTERIXImportTask::schema() const 
{ 
    return schema_; 
}

/**
*/
unsigned int ASTERIXImportTask::numPacketsInProcessing() const
{
    return num_packets_in_processing_;
}

/**
*/
ASTERIXImportTaskSettings& ASTERIXImportTask::settings()
{
    return settings_;
}

/**
*/
void ASTERIXImportTask::testFileDecoding()
{
    //no decoder yet?
    if (!decoder_)
        return;

    loginf << "checking decoding with decoder " << decoder_->name();

    file_decoding_tested_ = false;

    auto check_decoding = [ this ] (const AsyncTaskState& state, AsyncTaskProgressWrapper& progress)
    {
        //refresh decoder check
        traced_assert(this->decoder_);
        this->decoder_->canDecode(true);

        return Result::succeeded();
    };

    AsyncFuncTask task(check_decoding, "Testing decoding", "Please wait...", false);
    task.runAsyncDialog(true, nullptr);

    file_decoding_tested_ = true;

    //notify that the decoding has been checked
    emit decodingStateChanged();
}

/**
*/
bool ASTERIXImportTask::isRunning() const
{
    return running_;
}

/**
*/
bool ASTERIXImportTask::canRun()
{
    if (!decoder_)
        return false;

    if (file_decoding_tested_ && !decoder_->canDecode(false))
        return false;

    return true;
}

/**
*/
void ASTERIXImportTask::reset()
{
    decode_job_ = nullptr;

    json_map_jobs_.clear();
    postprocess_jobs_.clear();
    accumulated_buffers_.clear();
    queued_insert_buffers_.clear();

    running_ = true;
    stopped_ = false;
    done_    = false; // since can be run multiple times

    traced_assert(!insert_active_);
    insert_active_ = false;

    traced_assert(!insert_slot_connected_);
    insert_slot_connected_ = false;

    all_done_ = false;

    num_packets_in_processing_ = 0;
    num_packets_total_         = 0;
    num_records_               = 0;

    current_data_source_name_ = "";

    start_time_              = boost::posix_time::microsec_clock::local_time();
    last_insert_time_        = boost::posix_time::microsec_clock::local_time();
    last_file_progress_time_ = {};

    error_         = false;
    error_message_ = "";

    added_data_sources_.clear();

    ts_calculator_.reset();
}

/**
*/
void ASTERIXImportTask::stop()
{
    logdbg << "start";

    stopped_ = true;

    if (decode_job_)
        decode_job_->setObsolete();

    for (auto& job_it : json_map_jobs_)
        job_it->setObsolete();

    //json_map_futures_.clear();

    for (auto& job_it : postprocess_jobs_)
        job_it->setObsolete();

    accumulated_buffers_.clear();
    queued_insert_buffers_.clear();

    while(decode_job_ && !decode_job_->done())
    {
        logdbg << "waiting for decode job to finish";

        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

        QThread::msleep(1);
    }

    while(json_map_jobs_.size())
    {
        logdbg << "waiting for map job to finish";

        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

        QThread::msleep(1);
    }

    while(postprocess_jobs_.size())
    {
        logdbg << "waiting for post-process job to finish";

        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

        QThread::msleep(1);
    }

    logdbg << "done";
}

/**
*/
void ASTERIXImportTask::run() // , bool create_mapping_stubs
{
    loginf;

    traced_assert(!running_);
    traced_assert(decoder_);
    traced_assert(schema_);

    traced_assert(decode_job_ == nullptr);

    traced_assert(json_map_jobs_.empty());
    traced_assert(postprocess_jobs_.empty());
    traced_assert(accumulated_buffers_.empty());
    traced_assert(queued_insert_buffers_.empty());

    traced_assert(canRun());

    if (source_.isNetworkType())
    {
        COMPASS::instance().appMode(AppMode::LiveRunning); // set live mode

        COMPASS::instance().logInfo("ASTERIX Import") << "started: network";
    }
    else
        COMPASS::instance().logInfo("ASTERIX Import") << "started: files";


    //reset state before new run
    reset();

    float free_ram = System::getFreeRAMinGB();

    loginf << "filenames " << source_.filesAsString() << " free RAM " << free_ram << " GB";

    if (source_.isFileType())
    {
        last_file_progress_time_ = boost::posix_time::microsec_clock::local_time();

        updateFileProgressDialog(true);

        file_progress_dialog_->show();

        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

        while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds() < 50)
        {
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        }
    }

    for (auto& map_it : *schema_)
        if (!map_it.second->initialized())
            map_it.second->initialize();

    loginf << "setting categories";

    refreshjASTERIX();

    jASTERIX::add_artas_md5_hash = true;

    // set up projections
    ProjectionManager& proj_man = ProjectionManager::instance();

    traced_assert(proj_man.hasCurrentProjection());
    Projection& projection = proj_man.currentProjection();
    projection.clearCoordinateSystems(); // to rebuild from data sources
    projection.addAllCoordinateSystems();

    loginf << "starting decode job";

    if (source_.isNetworkType())
        COMPASS::instance().dataSourceManager().createNetworkDBDataSources();

    decode_job_ = make_shared<ASTERIXDecodeJob>(*this, post_process_);

    connect(decode_job_.get(), &ASTERIXDecodeJob::obsoleteSignal, this,
            &ASTERIXImportTask::decodeASTERIXObsoleteSlot, Qt::QueuedConnection);
    connect(decode_job_.get(), &ASTERIXDecodeJob::doneSignal, this,
            &ASTERIXImportTask::decodeASTERIXDoneSlot, Qt::QueuedConnection);
    connect(decode_job_.get(), &ASTERIXDecodeJob::decodedASTERIXSignal, this,
            &ASTERIXImportTask::addDecodedASTERIXSlot, Qt::QueuedConnection);

    JobManager::instance().addBlockingJob(decode_job_);

    loginf << "done";

    return;
}

/**
*/
void ASTERIXImportTask::decodeASTERIXDoneSlot()
{
    logdbg << "start";

    if (!decode_job_) // called twice?
        return;

    traced_assert(decode_job_);

    if (!stopped_ && decode_job_->error())
    {
        loginf << "error";

        error_         = decode_job_->error();
        error_message_ = decode_job_->errorMessage();

        if (allow_user_interactions_)
        {
            QMessageBox msgBox;
            msgBox.setText(("Decoding error: " + error_message_ + "\n\nPlease check the decoder settings.").c_str());
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.exec();
        }
    }

    decode_job_ = nullptr;

    checkAllDone();
}

/**
*/
void ASTERIXImportTask::decodeASTERIXObsoleteSlot()
{
    logdbg << "start";

    decode_job_ = nullptr;
}

/**
*/
void ASTERIXImportTask::addDecodedASTERIXSlot()
{
    logdbg << "start";
    //int cpu = sched_getcpu();
    //loginf << "running on cpu " << cpu;

    if (stopped_)
    {
        checkAllDone();
        return;
    }

    traced_assert(decode_job_);
    std::string source_name = decode_job_->currentDataSourceName();

    logdbg << "errors " << decode_job_->numErrors()
           << " num records " << jasterix_->numRecords();

    if (source_.isFileType())
    {
        if (file_progress_dialog_->wasCanceled())
        {
            stop();
            return;
        }

        if (maxLoadReached())
        // break if too many packets in process, this slot is called again from insertDoneSlot or postProcessDone
        {
            logdbg << "returning since max load reached, map futures "
                   << json_map_jobs_.size() << " queued insert buffers " << queued_insert_buffers_.size();
            return;
        }
    }

    if (stopped_)
        return;

    if (num_packets_in_processing_ > settings_.num_packets_overload_) // network special case
    {
        logwrn << "overload detected, packets in processing "
               << num_packets_in_processing_ << " skipping data";

        std::vector<std::unique_ptr<nlohmann::json>> extracted_data {decode_job_->extractedData()};

        // issue: if all packets are already in queued_job_buffers_, no insert will be started
        // try to resume after being in overload for too long
        if (!insert_active_ && queued_insert_buffers_.size())
            insertData(); // will call itself again if required

        return;
    }

    logdbg << "processing data total cnt " << num_packets_total_;

    std::vector<std::unique_ptr<nlohmann::json>> extracted_data {decode_job_->extractedData()};

    if (!extracted_data.size())
    {
        logdbg << "processing data empty";
        return;
    }

    ++num_packets_in_processing_;
    ++num_packets_total_;

    logdbg << "processing data,"
           << " num_packets_in_processing_ " << num_packets_in_processing_
           << " num_packets_total_ " << num_packets_total_;

    if (stopped_)
        return;

    traced_assert(schema_);

    std::vector<std::string> keys;

    if (settings_.current_file_framing_ == "" || source_.isNetworkType()) // force netto when doing network import
        keys = {"data_blocks", "content", "records"};
    else
        keys = {"frames", "content", "data_blocks", "content", "records"};

    std::shared_ptr<ASTERIXJSONMappingJob> json_map_job =
        make_shared<ASTERIXJSONMappingJob>(std::move(extracted_data), source_name, keys, schema_->parsers());

    json_map_jobs_.push_back(json_map_job);

    traced_assert(!extracted_data.size());

    connect(json_map_job.get(), &ASTERIXJSONMappingJob::obsoleteSignal, this,
            &ASTERIXImportTask::mapJSONObsoleteSlot, Qt::QueuedConnection);
    connect(json_map_job.get(), &ASTERIXJSONMappingJob::doneSignal, this,
            &ASTERIXImportTask::mapJSONDoneSlot, Qt::QueuedConnection);

    //loginf << "queueing in new mapping job, main thread is " << QThread::currentThreadId();

    JobManager::instance().addNonBlockingJob(json_map_job);
}

/**
*/
void ASTERIXImportTask::mapJSONDoneSlot()
{
    logdbg << "start";

    if (stopped_)
    {
        logdbg << "stopping";

        json_map_jobs_.clear();

        checkAllDone();

        return;
    }

    ASTERIXJSONMappingJob* map_job = dynamic_cast<ASTERIXJSONMappingJob*>(QObject::sender());
    traced_assert(map_job);
    std::string source_name = map_job->sourceName();

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers {map_job->buffers()};

    traced_assert(json_map_jobs_.size());
    traced_assert(json_map_jobs_.begin()->get() == map_job);
    map_job = nullptr;
    json_map_jobs_.erase(json_map_jobs_.begin()); // remove

    logdbg << "processing, num buffers " << job_buffers.size();

    if (!job_buffers.size())
    {
        traced_assert(num_packets_in_processing_);
        num_packets_in_processing_--;
        return;
    }

    traced_assert(!ts_calculator_.processing());
    ts_calculator_.setBuffers(std::move(job_buffers));

    bool check_future_ts = source_.isNetworkType();

    if (settings_.network_ignore_future_ts_)
        check_future_ts = false;

    ts_calculator_.calculate(source_name,
                             settings_.date_, settings_.reset_date_between_files_,
                             settings_.override_tod_active_, settings_.override_tod_offset_,
                             settings_.ignore_time_jumps_, check_future_ts);

    timestampCalculationDoneSlot();

    // while (ts_calculator_.processing())
    // {
    //     QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

    //     loginf << "waiting on ts calc";
    //     QThread::msleep(100);
    // }

    // traced_assert(!ts_calculator_.processing());
    // ts_calculator_.setBuffers(std::move(job_buffers));

    // // start calculate timestamp job
    // bool check_future_ts = source_.isNetworkType();

    // if (settings_.network_ignore_future_ts_)
    //     check_future_ts = false;

    // ts_calc_future_ = std::async(std::launch::async,
    //                              [this, current_source_name=source_name, settings=settings_,check_future_ts] {
    //     {
    //         try
    //         {
    //             //loginf << "time stamp calc lambda: start";

    //             ts_calculator_.calculate(current_source_name,
    //                                      settings.date_, settings.reset_date_between_files_,
    //                                      settings.override_tod_active_, settings.override_tod_offset_,
    //                                      settings.ignore_time_jumps_, check_future_ts);

    //             QMetaObject::invokeMethod(this, "timestampCalculationDoneSlot", Qt::QueuedConnection);
    //         }
    //         catch (const std::exception& e)
    //         {
    //             loginf << "calc ts threw exception '" << e.what() << "'";
    //             traced_assert(false);
    //         }
    //     }});

    logdbg << "done";
}

/**
*/
void ASTERIXImportTask::mapJSONObsoleteSlot()
{
    logdbg << "start";

    ASTERIXJSONMappingJob* map_job = dynamic_cast<ASTERIXJSONMappingJob*>(QObject::sender());
    traced_assert(map_job);

    traced_assert(json_map_jobs_.size());
    traced_assert(json_map_jobs_.begin()->get() == map_job);
    map_job = nullptr;
    json_map_jobs_.erase(json_map_jobs_.begin()); // remove

    checkAllDone();
}

void ASTERIXImportTask::timestampCalculationDoneSlot()
{
    logdbg << "start";

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers {ts_calculator_.buffers()};
    ts_calculator_.setProcessingDone();

    std::shared_ptr<ASTERIXPostprocessJob> postprocess_job =
        make_shared<ASTERIXPostprocessJob>(
            std::move(job_buffers),
            settings_.filter_tod_active_,
            settings_.filter_tod_min_, settings_.filter_tod_max_,
            settings_.filter_position_active_,
            settings_.filter_latitude_min_, settings_.filter_latitude_max_,
            settings_.filter_longitude_min_, settings_.filter_longitude_max_,
            settings_.filter_modec_active_,
            settings_.filter_modec_min_, settings_.filter_modec_max_,
            settings_.obfuscate_secondary_info_
            );

    postprocess_jobs_.push_back(postprocess_job);

    // check for future when net import

    connect(postprocess_job.get(), &ASTERIXPostprocessJob::obsoleteSignal, this,
            &ASTERIXImportTask::postprocessObsoleteSlot, Qt::QueuedConnection);
    connect(postprocess_job.get(), &ASTERIXPostprocessJob::doneSignal, this,
            &ASTERIXImportTask::postprocessDoneSlot, Qt::QueuedConnection);

    JobManager::instance().addNonBlockingJob(postprocess_job);
}

/**
*/
void ASTERIXImportTask::postprocessDoneSlot()
{
    logdbg << "start";

    if (stopped_)
    {
        postprocess_jobs_.clear();

        checkAllDone();

        return;
    }

    ASTERIXPostprocessJob* post_job = dynamic_cast<ASTERIXPostprocessJob*>(QObject::sender());
    traced_assert(post_job);

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers {post_job->buffers()};

    traced_assert(postprocess_jobs_.size());
    traced_assert(postprocess_jobs_.begin()->get() == post_job);
    post_job = nullptr;
    postprocess_jobs_.erase(postprocess_jobs_.begin()); // remove

    // check if still data in buffers, could be empty

    unsigned int buffer_cnt {0};

    for (auto& buf_it : job_buffers)
    {
        logdbg << "buffer " << buf_it.second->dbContentName()
               << " size " << buf_it.second->size();

        traced_assert(buf_it.second->hasProperty(DBContent::meta_var_timestamp_));

        buffer_cnt += buf_it.second->size();
    }

    logdbg << "buffer cnt " << buffer_cnt;

    if (buffer_cnt == 0)
    {
        if (accumulated_buffers_.size())
        {
            // --num_packets_in_processing_; not required since queued and decrement will be done by insert

            //queued buffers exist => add to queue since 0 data after
            queued_insert_buffers_.push_back(std::move(accumulated_buffers_));
            accumulated_buffers_.clear();

            if (!insert_active_ &&
                !queued_insert_buffers_.empty() &&
                !COMPASS::instance().dbExportInProgress() &&
                !COMPASS::instance().dbContentManager().loadInProgress())
            {
                logdbg << "inserting";
                traced_assert(!COMPASS::instance().dbContentManager().insertInProgress());

                insertData();
            }
        }
        else
        {
            traced_assert(num_packets_in_processing_);
            --num_packets_in_processing_;

            if (source_.isFileType())
                updateFileProgressDialog();
        }

        logdbg << "no data,"
               << " num_packets_in_processing_ " << num_packets_in_processing_
               << " num_packets_total_ " << num_packets_total_;

        checkAllDone();

        if (decode_job_ && decode_job_->hasData())
        {
            logdbg << "starting decoding of next chunk";
            addDecodedASTERIXSlot(); // load next chunk
        }

        return;
    }

    // queue data
    if (!stopped_)
    {
        if (source_.isNetworkType())
        {
            logdbg << "live - adding for insert...";

            // live - just add buffers for insert
            queued_insert_buffers_.push_back(std::move(job_buffers));
        }
        else
        {
            logdbg << "accumulating...";

            // offline - move buffers to accumulated buffers
            auto insert_buffers = std::move(job_buffers);

            for (auto& b : insert_buffers)
            {
                if (!b.second->size())
                    continue;

                auto it = accumulated_buffers_.find(b.first);
                if (it == accumulated_buffers_.end())
                    accumulated_buffers_[ b.first ] = std::move(b.second);
                else
                    it->second->seizeBuffer(*b.second);
            }
            insert_buffers.clear();

            // check if ready to queue in
            size_t size_max = 0;
            for (const auto& b : accumulated_buffers_)
            {
                logdbg << "buffer " << b.second->dbContentName()
                       << " size " << b.second->size();

                traced_assert(b.second->hasProperty(DBContent::meta_var_timestamp_));

                if (b.second->size() > size_max)
                    size_max = b.second->size();
            }

            logdbg << "accumulated buffers, maximum size "
                   << size_max << " / " << settings_.chunk_size_insert;

            if (!decode_job_ || size_max >= settings_.chunk_size_insert)
            {
                logdbg << "adding accumulated buffers to queue";

                //queued buffers full => add to queue
                queued_insert_buffers_.push_back(std::move(accumulated_buffers_));
                accumulated_buffers_.clear();
            }
            else
            {
                logdbg << "accumulated buffers not yet full";

                //processing is postponed, so to keep the whole thing running decrease packet count...
                --num_packets_in_processing_;

                //...and restart decoding
                if (decode_job_ && decode_job_->hasData())
                    QMetaObject::invokeMethod(this, "addDecodedASTERIXSlot", Qt::QueuedConnection);
            }
        }

        if (!insert_active_ && 
            !queued_insert_buffers_.empty() && 
            !COMPASS::instance().dbExportInProgress() && 
            !COMPASS::instance().dbContentManager().loadInProgress())
        {
            logdbg << "inserting";
            traced_assert(!COMPASS::instance().dbContentManager().insertInProgress());

            insertData();
        }
    }
}

/**
*/
void ASTERIXImportTask::postprocessObsoleteSlot()
{
    ASTERIXPostprocessJob* post_job = dynamic_cast<ASTERIXPostprocessJob*>(QObject::sender());
    traced_assert(post_job);

    traced_assert(postprocess_jobs_.size());
    traced_assert(postprocess_jobs_.begin()->get() == post_job);
    post_job = nullptr;
    postprocess_jobs_.erase(postprocess_jobs_.begin()); // remove
}

/**
*/
void ASTERIXImportTask::insertData()
{
    logdbg << "thread " << QThread::currentThreadId();

    traced_assert(!insert_active_);
    insert_active_ = true;

    traced_assert(queued_insert_buffers_.size());

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers = *queued_insert_buffers_.begin();
    queued_insert_buffers_.erase(queued_insert_buffers_.begin());

    unsigned cnt=0;

    for (auto& buf_it : job_buffers)
        cnt += buf_it.second->size();

    logdbg << "inserting " << job_buffers.size() << " into database, cnt " << cnt;

    if (stopped_)
    {
        checkAllDone();
        return;
    }

    DBContentManager& dbcont_manager = COMPASS::instance().dbContentManager();

    traced_assert(schema_);

    unsigned int current_num_records = 0;

    for (auto& job_it : job_buffers)
    {
        current_num_records += job_it.second->size();
        num_records_ += job_it.second->size();

        if (COMPASS::instance().appMode() != AppMode::LiveRunning) // is cleaned special there
            job_it.second->deleteEmptyProperties();
    }

    logdbg << "inserting " << current_num_records << " records";

    if (!insert_slot_connected_)
    {
        loginf << "connecting slot";

        connect(&dbcont_manager, &DBContentManager::insertDoneSignal,
                this, &ASTERIXImportTask::insertDoneSlot, Qt::QueuedConnection);
        insert_slot_connected_ = true;
    }

    //insert_start_time_ = boost::posix_time::microsec_clock::local_time();

    dbcont_manager.insertData(job_buffers);

    checkAllDone();

    logdbg << "done";
}

/**
*/
void ASTERIXImportTask::insertDoneSlot()
{
    logdbg << "start";

    traced_assert(insert_slot_connected_);

    if (source_.isFileType())
    {
        logdbg << "num_packets_in_processing " << num_packets_in_processing_;

        updateFileProgressDialog();
    }

    // has to be after file progress dialog update since calls processEvents and thus creates race condition
    traced_assert(insert_active_);
    insert_active_ = false;
    traced_assert(!COMPASS::instance().dbContentManager().insertInProgress());

    --num_packets_in_processing_;

    //    double insert_time_ms = (double)(
    //                boost::posix_time::microsec_clock::local_time() - insert_start_time_).total_microseconds() / 1000.0;

    //    total_insert_time_ms_ += insert_time_ms;

    //    loginf << "UGA insert time " << insert_time_ms
    //           << " ms total " << total_insert_time_ms_/1000.0 << " s" ;

    if (queued_insert_buffers_.size())
    {
        logdbg << "inserting, thread " << QThread::currentThreadId()
               << " main " << QApplication::instance()->thread()->currentThreadId();
        insertData();
    }

    logdbg << "processed " << num_records_ << " records";

    if (decode_job_ && decode_job_->hasData())
    {
        logdbg << "starting decoding of next chunk";
        QMetaObject::invokeMethod(this, "addDecodedASTERIXSlot", Qt::QueuedConnection); // load next chunk
    }

    checkAllDone();

    logdbg << "done";
}

/**
*/
void ASTERIXImportTask::appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current)
{
    loginf << "current " << toString(app_mode_current)
           << " new " << toString(app_mode_previous) << " running " << running_;

    if (!running_) // then nothing to do
        return;

    traced_assert(decode_job_);

    if (app_mode_current == AppMode::LiveRunning)
    {
        traced_assert(app_mode_previous == AppMode::LivePaused || app_mode_previous == AppMode::Offline);
    }
    else if (app_mode_current == AppMode::LivePaused)
    {
        traced_assert(app_mode_previous == AppMode::LiveRunning); // can only happen from running
    }
    else if (app_mode_current == AppMode::Offline)
    {
        traced_assert(app_mode_previous == AppMode::LiveRunning || app_mode_previous == AppMode::LivePaused);

        stop();
    }
}

/**
*/
void ASTERIXImportTask::checkAllDone()
{
    logdbg << "all done " << all_done_
           << " num_packets_in_processing_ " << num_packets_in_processing_
           << " decode " << (decode_job_ != nullptr)
           << " map jobs " << json_map_jobs_.size()
           << " ts calc " << ts_calculator_.processing()
           << " post jobs " << postprocess_jobs_.size()
           << " accumulated for insert " << accumulated_buffers_.size()
           << " queued insert " << queued_insert_buffers_.size()
           << " insert active " << insert_active_;

    if (!all_done_
        && decode_job_ == nullptr
        && !json_map_jobs_.size()
        && !ts_calculator_.processing()
        && !postprocess_jobs_.size()
        && !accumulated_buffers_.size()
        && !queued_insert_buffers_.size()
        && !insert_active_)
    {
        loginf << "setting all done: total packets " << num_packets_total_;

        ts_calculator_.logLastTimestamp();

        all_done_ = true;
        done_     = true; // why was this not set?
        running_  = false;

        boost::posix_time::time_duration time_diff = boost::posix_time::microsec_clock::local_time() - start_time_;
        loginf << "import done after "
               << String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, false);

        int records_per_second = num_records_ / std::max(1.0, time_diff.total_milliseconds() / 1000.0);

        COMPASS::instance().logInfo("ASTERIX Import")
            << " finished after "
            << String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, false)
            << ", inserted " << num_records_ << " rec"
            << " with " << records_per_second << " rec/s";

        COMPASS::instance().mainWindow().updateMenus(); // re-enable import menu

        logdbg << "refresh";

        refreshjASTERIX();

        logdbg << "db content";

        //emit COMPASS::instance().interface().databaseContentChangedSignal();

        logdbg << "status logging";

        if (!allow_user_interactions_)
        {
            logdbg << "deleting status widget";
        }

        COMPASS::instance().dataSourceManager().saveDBDataSources();
        emit COMPASS::instance().dataSourceManager().dataSourcesChangedSignal();
        emit COMPASS::instance().dbContentManager().dbContentStatusChanged();
        COMPASS::instance().dbInterface().saveProperties();

        malloc_trim(0); // release unused memory

        if (insert_slot_connected_) // moved here from insertDoneSlot
        {
            disconnect(&COMPASS::instance().dbContentManager(), &DBContentManager::insertDoneSignal,
                       this, &ASTERIXImportTask::insertDoneSlot);
            insert_slot_connected_ = false;
        }

        //close dialog
        if (source_.isFileType() && file_progress_dialog_)
        {
            file_progress_dialog_ = nullptr;
        }

        QApplication::restoreOverrideCursor();

        //cleanup db after import
        if (source_.isFileType())
            COMPASS::instance().dbInterface().cleanupDB(true);

        emit doneSignal();
    }

    logdbg << "done";
}

/**
*/
bool ASTERIXImportTask::maxLoadReached()
{
    return json_map_jobs_.size() > settings_.max_packets_in_processing_
        || queued_insert_buffers_.size() > settings_.max_packets_in_processing_;
}

/**
*/
void ASTERIXImportTask::updateFileProgressDialog(bool force)
{
    if (stopped_)
        return;

    traced_assert(source_.isFileType());

    if (!file_progress_dialog_)
    {
        file_progress_dialog_.reset(
            new QProgressDialog(("Files '" + source_.filesAsString() + "'").c_str(), "Abort", 0, 100));
        file_progress_dialog_->setWindowTitle("Importing ASTERIX Recording(s)");
        file_progress_dialog_->setWindowModality(Qt::ApplicationModal);

        force = true;
    }

    if (!force
        && (boost::posix_time::microsec_clock::local_time() - last_file_progress_time_).total_milliseconds() < 500)
    {
        return;
    }

    last_file_progress_time_ = boost::posix_time::microsec_clock::local_time();

    if (decode_job_)
    {
        traced_assert(decode_job_->hasStatusInfo());
        file_progress_dialog_->setLabelText(decode_job_->statusInfoString().c_str());
        file_progress_dialog_->setValue(decode_job_->statusInfoProgress());
    }
    else
    {
        file_progress_dialog_->setLabelText("Please wait...");
    }
}

/**
*/
void ASTERIXImportTask::onConfigurationChanged(const std::vector<std::string>& changed_params)
{
    emit configChanged();
}

/**
*/
void ASTERIXImportTask::runDialog(QWidget* parent)
{
    //show dialog
    ASTERIXImportTaskDialog dlg(*this, parent);

    //cancelled?
    if (dlg.exec() != QDialog::Accepted)
        return;

    //enable user interactions
    allowUserInteractions(true);

    //otherwise run import
    run();
}
