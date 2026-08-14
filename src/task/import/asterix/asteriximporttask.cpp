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
#include "asteriximportprobeaggregator.h"
#include "asterixnetworkreplaysender.h"
#include "asterixreporthelpers.h"
#include "asterix_decoding_config.h"

#include "task/result/report/report.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"

#include "data_source.h"
#include "number.h"
#include "compass.h"
#include "viewmanager.h"
#include "buffer.h"
#include "configurable.h"
#include "dbinterface.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentdataengine.h"
#include "db_context_manager.h"
#include "files.h"
#include "logger.h"
#include "asteriximporttaskdialog.h"
#include "traced_assert.h"
#include "util/stringconv.h"
#include "system.h"
#include "taskmanager.h"
#include "mainwindow.h"
#include "util/timeconv.h"
#include "projection.h"
#include "projectionmanager.h"
#include "jobmanager.h"
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
#include <QLabel>
#include <QProgressDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>

#include <algorithm>
#include <malloc.h>

using namespace Utils;
using namespace nlohmann;
using namespace std;

/**
*/
// (old constructor removed)
// {
//     tooltip_ = "Allows importing of ASTERIX data recording files into the opened database.";

//     registerParameter("reset_date_between_files", &settings_.reset_date_between_files_,
//                       ASTERIXImportTaskSettings().reset_date_between_files_);
//     registerParameter("debug_jasterix", &settings_.debug_jasterix_, ASTERIXImportTaskSettings().debug_jasterix_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "debug_jasterix");

//     registerParameter("current_file_framing", &settings_.current_file_framing_, ASTERIXImportTaskSettings().current_file_framing_);

//     registerParameter("num_packets_overload", &settings_.num_packets_overload_, ASTERIXImportTaskSettings().num_packets_overload_);

//     registerParameter("override_tod_active", &settings_.override_tod_active_,
//                       ASTERIXImportTaskSettings().override_tod_active_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "override_tod_active");
//     registerParameter("override_tod_offset", &settings_.override_tod_offset_,
//                       ASTERIXImportTaskSettings().override_tod_offset_);

//     registerParameter("filter_tod_active", &settings_.filter_tod_active_,
//                       ASTERIXImportTaskSettings().filter_tod_active_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_tod_active");
//     registerParameter("filter_tod_min", &settings_.filter_tod_min_, ASTERIXImportTaskSettings().filter_tod_min_);
//     registerParameter("filter_tod_max", &settings_.filter_tod_max_, ASTERIXImportTaskSettings().filter_tod_max_);

//     registerParameter("filter_position_rec_active", &settings_.filter_position_rec_active_,
//                       ASTERIXImportTaskSettings().filter_position_rec_active_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_position_rec_active");
//     registerParameter("filter_rec_latitude_min", &settings_.filter_rec_latitude_min_,
//                       ASTERIXImportTaskSettings().filter_rec_latitude_min_);
//     registerParameter("filter_rec_latitude_max", &settings_.filter_rec_latitude_max_,
//                       ASTERIXImportTaskSettings().filter_rec_latitude_max_);
//     registerParameter("filter_rec_longitude_min", &settings_.filter_rec_longitude_min_,
//                       ASTERIXImportTaskSettings().filter_rec_longitude_min_);
//     registerParameter("filter_rec_longitude_max", &settings_.filter_rec_longitude_max_,
//                       ASTERIXImportTaskSettings().filter_rec_longitude_max_);

//     registerParameter("filter_position_circ_active", &settings_.filter_position_circ_active_,
//                       ASTERIXImportTaskSettings().filter_position_circ_active_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_position_circ_active");
//     registerParameter("filter_circ_latitude", &settings_.filter_circ_latitude_,
//                       ASTERIXImportTaskSettings().filter_circ_latitude_);
//     registerParameter("filter_circ_longitude", &settings_.filter_circ_longitude_,
//                       ASTERIXImportTaskSettings().filter_circ_longitude_);
//     registerParameter("filter_circ_range", &settings_.filter_circ_range_,
//                       ASTERIXImportTaskSettings().filter_circ_range_);

//     registerParameter("filter_modec_active", &settings_.filter_modec_active_,
//                       ASTERIXImportTaskSettings().filter_modec_active_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_modec_active");
//     registerParameter("filter_modec_min", &settings_.filter_modec_min_, ASTERIXImportTaskSettings().filter_modec_min_);
//     registerParameter("filter_modec_max", &settings_.filter_modec_max_, ASTERIXImportTaskSettings().filter_modec_max_);

//     registerParameter("file_line_id", &settings_.file_line_id_, ASTERIXImportTaskSettings().file_line_id_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "file_line_id");
//     registerParameter("date_str", &settings_.date_str_, ASTERIXImportTaskSettings().date_str_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "date_str");

//     if (settings_.date_str_.size())
//         settings_.date_ = Time::fromDateString(settings_.date_str_);
//     if (settings_.date_.is_not_a_date_time())
//         settings_.date_ = boost::posix_time::ptime(boost::gregorian::day_clock::universal_day());

//     registerParameter("network_ignore_future_ts", &settings_.network_ignore_future_ts_,
//                       ASTERIXImportTaskSettings().network_ignore_future_ts_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "network_ignore_future_ts");
//     registerParameter("obfuscate_secondary_info", &settings_.obfuscate_secondary_info_,
//                       ASTERIXImportTaskSettings().obfuscate_secondary_info_);
//     addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "obfuscate_secondary_info");

//     registerParameter("chunk_size_jasterix", &settings_.chunk_size_jasterix, ASTERIXImportTaskSettings().chunk_size_jasterix);
//     registerParameter("chunk_size_insert", &settings_.chunk_size_insert, ASTERIXImportTaskSettings().chunk_size_insert);

//     std::string jasterix_definition_path = HOME_DATA_DIRECTORY + "jasterix_definitions";

//     loginf << "jasterix definition path '"
//            << jasterix_definition_path << "'";
//     traced_assert(Files::directoryExists(jasterix_definition_path));

//     jASTERIX::frame_chunk_size      = settings_.chunk_size_jasterix;
//     jASTERIX::data_block_chunk_size = settings_.chunk_size_jasterix;

//     refreshjASTERIX(); // needed for available framings check etc.

//     createSubConfigurables();

//     connect(&source_, &ASTERIXImportSource::changed, this, &ASTERIXImportTask::sourceChanged);
//     connect(&source_, &ASTERIXImportSource::fileUsageChanged, this, &ASTERIXImportTask::sourceUsageChanged);

//     registerParameter("max_packets_in_processing", &settings_.max_packets_in_processing_,
//                       settings_.max_packets_in_processing_);

//     logdbg << "thread " << QThread::currentThreadId()
//            << " main " << QApplication::instance()->thread()->currentThreadId();
// }

ASTERIXImportTask::ASTERIXImportTask(nlohmann::json& config, TaskManager* parent)
    : Task(*parent),
    Configurable(config, parent),
    compass_(parent->compass()),
    dbcontent_man_(parent->compass().dbContentManager())
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
    registerParameter("filter_position_rec_active", &settings_.filter_position_rec_active_,
                      ASTERIXImportTaskSettings().filter_position_rec_active_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_position_rec_active");
    registerParameter("filter_rec_latitude_min", &settings_.filter_rec_latitude_min_,
                      ASTERIXImportTaskSettings().filter_rec_latitude_min_);
    registerParameter("filter_rec_latitude_max", &settings_.filter_rec_latitude_max_,
                      ASTERIXImportTaskSettings().filter_rec_latitude_max_);
    registerParameter("filter_rec_longitude_min", &settings_.filter_rec_longitude_min_,
                      ASTERIXImportTaskSettings().filter_rec_longitude_min_);
    registerParameter("filter_rec_longitude_max", &settings_.filter_rec_longitude_max_,
                      ASTERIXImportTaskSettings().filter_rec_longitude_max_);
    registerParameter("filter_position_circ_active", &settings_.filter_position_circ_active_,
                      ASTERIXImportTaskSettings().filter_position_circ_active_);
    addJSONExportFilter(JSONExportType::General, JSONExportFilterType::ParamID, "filter_position_circ_active");
    registerParameter("filter_circ_latitude", &settings_.filter_circ_latitude_,
                      ASTERIXImportTaskSettings().filter_circ_latitude_);
    registerParameter("filter_circ_longitude", &settings_.filter_circ_longitude_,
                      ASTERIXImportTaskSettings().filter_circ_longitude_);
    registerParameter("filter_circ_range", &settings_.filter_circ_range_,
                      ASTERIXImportTaskSettings().filter_circ_range_);
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
    registerParameter("max_packets_in_processing", &settings_.max_packets_in_processing_,
                      ASTERIXImportTaskSettings().max_packets_in_processing_);

    std::string jasterix_definition_path = HOME_DATA_DIRECTORY + "jasterix_definitions";

    loginf << "jasterix definition path '"
           << jasterix_definition_path << "'";
    traced_assert(Files::directoryExists(jasterix_definition_path));

    jASTERIX::frame_chunk_size      = settings_.chunk_size_jasterix;
    jASTERIX::data_block_chunk_size = settings_.chunk_size_jasterix;

    initjASTERIX(); // create decoder for ASTERIXJSONParser - no context needed

    createSubConfigurables();

    connect(&source_, &ASTERIXImportSource::changed, this, &ASTERIXImportTask::sourceChanged);
    connect(&source_, &ASTERIXImportSource::fileUsageChanged, this, &ASTERIXImportTask::sourceUsageChanged);
}

/**
*/
ASTERIXImportTask::~ASTERIXImportTask()
{
    logdbg;
}

/**
*/
void ASTERIXImportTask::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);
    const auto& instance_name = Configuration::getInstanceName(child_json);
    if (class_name == "ASTERIXCategoryConfig")
    {
        loginf << "skipping legacy ASTERIXCategoryConfig '"
               << instance_name << "' - now managed by DBContextManager";
    }
    else if (class_name == "ASTERIXJSONParsingSchema")
    {
        std::string name = child_json.at(Configuration::ParameterSection).at("name").get<std::string>();

        traced_assert(schema_ == nullptr);
        traced_assert(name == "jASTERIX");

        logdbg << "generating schema " << instance_name
               << " with name " << name;

        schema_.reset(new ASTERIXJSONParsingSchema(child_json, *this));
    }
    else
    {
        throw std::runtime_error("ASTERIXImportTask: generateSubConfigurable: unknown class_name " + class_name);
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

    settings_.setActiveFileFraming(asterix_framing);
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

    auto& ctx_mgr = compass_.dbContextManager();

    for (auto& cat_it : config.items())
    {
        std::string cat_str = cat_it.key();

        unsigned int cat = stoi(cat_str);

        if (!hasConfiguratonFor(cat))
            throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: unknown cat "+to_string(cat)
                                +" from '" + cat_str + "'");

        json& cat_cfg = cat_it.value();
        if (!cat_cfg.is_object())
            throw runtime_error("ASTERIXImportTask: asterixDecoderConfig: cat "+to_string(cat)
                                +" config is not an object");

        // create the config entry with jASTERIX defaults if the context does not have one yet
        auto* cfg = &ctx_mgr.getOrCreateAsterixConfig(cat,
                                                      jasterix_->category(cat)->defaultEdition(),
                                                      jasterix_->category(cat)->defaultREFEdition(),
                                                      jasterix_->category(cat)->defaultSPFEdition());

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

            cfg->edition(edition);
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

            cfg->ref(ref_ed);
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

            cfg->spf(spf_ed);
        }
    }

    ctx_mgr.saveContext(ctx_mgr.activeContextName());
}

/**
*/
void ASTERIXImportTask::checkSubConfigurables()
{
    if (schema_ == nullptr)
    {
        auto& child_json = addNewSubConfiguration("ASTERIXJSONParsingSchema", "JSONParsingSchemajASTERIX0");
        child_json[Configuration::ParameterSection]["name"] = "jASTERIX";
        generateSubConfigurable(child_json);
    }
}

/**
*/
void ASTERIXImportTask::sourceChanged()
{
    loginf << "new type = " << source_.sourceTypeAsString();

    //update to suitable decoder
    decoder_ = ASTERIXDecoderBase::createDecoder(*this, source_);
    traced_assert(decoder_);

    loginf << "created new decoder "  << decoder_->name();

    //switch to framing required by the decoder?
    settings_.setFileFramingOverride(decoder_->requiredASTERIXFraming());

    // skip the network probe (10s UDP listen) when driven non-interactively
    // (e.g. via the import_asterix_network rt-command) - the caller wants the
    // import to start immediately
    if (source_.isNetworkType() && !allow_user_interactions_)
    {
        loginf << "skipping network probe (non-interactive mode)";

        // reset stale state from earlier decoding tests, otherwise canRun()
        // would re-trigger the probe via canDecode() on empty probe results
        file_decoding_tested_ = false;

        return;
    }

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
void ASTERIXImportTask::initjASTERIX() const
{
    std::string jasterix_definition_path = HOME_DATA_DIRECTORY + "jasterix_definitions";

    logdbg << "jasterix definition path '"
           << jasterix_definition_path << "'";
    traced_assert(Files::directoryExists(jasterix_definition_path));

    jasterix_ = std::make_shared<jASTERIX::jASTERIX>(jasterix_definition_path, false,
                                                     settings_.debug_jasterix_, true);

    std::vector<std::string> framings = jasterix_->framings();
    if (std::find(framings.begin(), framings.end(), settings_.activeFileFraming()) == framings.end())
    {
        logdbg << "resetting to no framing";

        ASTERIXImportTaskSettings& settings = const_cast<ASTERIXImportTaskSettings&>(settings_);
        settings.setActiveFileFraming("");
    }
}

void ASTERIXImportTask::configurejASTERIX() const
{
    traced_assert(jasterix_);

    jasterix_->decodeNoCategories();

    if (!compass_.hasActiveContext())
    {
        logdbg << "no active context, no categories configured";
        return;
    }

    auto& ctx_mgr = compass_.dbContextManager();

    // Build a signature of what we're about to apply so we can decide whether to log
    // verbosely (first run, or config changed since last run) or quietly. Configure
    // is called on every refreshjASTERIX (i.e. once per analyze line) - full per-cat
    // logging there would be very noisy.
    std::string signature = ctx_mgr.activeContext().name() + "|";
    for (const auto& cfg : ctx_mgr.activeContext().asterixDecoding())
    {
        signature += std::to_string(cfg.category()) + ":"
                  + (settings_.decodeCategory(cfg.category()) ? "1" : "0") + ":"
                  + cfg.edition() + ":" + cfg.ref() + ":" + cfg.spf() + ";";
    }

    const bool config_changed = (signature != last_applied_asterix_config_);
    last_applied_asterix_config_ = signature;

    if (config_changed)
        loginf << "applying asterix decoding from context '"
               << ctx_mgr.activeContext().name() << "'";
    else
        logdbg << "applying asterix decoding from context '"
               << ctx_mgr.activeContext().name() << "' (unchanged)";

    unsigned int applied = 0, skipped = 0;

    for (const auto& cfg : ctx_mgr.activeContext().asterixDecoding())
    {
        unsigned int cat = cfg.category();

        bool decode = settings_.decodeCategory(cat);

        if (!jasterix_->hasCategory(cat))
        {
            logwrn << "cat " << cat << " not defined in decoder, skipping";
            ++skipped;
            continue;
        }

        if (!jasterix_->category(cat)->hasEdition(cfg.edition()))
        {
            logwrn << "cat " << cat << " edition '" << cfg.edition()
                   << "' not defined in decoder, skipping (parser will fall back to default edition)";
            ++skipped;
            continue;
        }

        if (cfg.ref().size() &&
            !jasterix_->category(cat)->hasREFEdition(cfg.ref()))
        {
            logwrn << "cat " << cat << " ref '" << cfg.ref()
                   << "' not defined in decoder, skipping";
            ++skipped;
            continue;
        }

        if (cfg.spf().size() &&
            !jasterix_->category(cat)->hasSPFEdition(cfg.spf()))
        {
            logwrn << "cat " << cat << " spf '" << cfg.spf()
                   << "' not defined in decoder, skipping";
            ++skipped;
            continue;
        }

        jasterix_->setDecodeCategory(cat, decode);
        jasterix_->category(cat)->setCurrentEdition(cfg.edition());
        jasterix_->category(cat)->setCurrentREFEdition(cfg.ref());
        jasterix_->category(cat)->setCurrentSPFEdition(cfg.spf());

        const std::string per_cat_msg = "cat " + std::to_string(cat) + " decode "
            + std::to_string(decode) + " edition '" + cfg.edition() + "'"
            + (cfg.ref().size() ? " ref '" + cfg.ref() + "'" : std::string())
            + (cfg.spf().size() ? " spf '" + cfg.spf() + "'" : std::string());

        if (config_changed)
            loginf << per_cat_msg;
        else
            logdbg << per_cat_msg;

        ++applied;
    }

    if (config_changed)
        loginf << "applied " << applied << " category configurations, " << skipped << " skipped";
    else
        logdbg << "applied " << applied << " category configurations, " << skipped << " skipped";
}

void ASTERIXImportTask::refreshjASTERIX() const
{
    initjASTERIX();
    configurejASTERIX();
}

/**
*/
bool ASTERIXImportTask::hasConfiguratonFor(unsigned int category)
{
    return compass_.dbContextManager().hasAsterixConfig(category);
}

/**
*/
bool ASTERIXImportTask::decodeCategory(unsigned int category)
{
    return settings_.decodeCategory(category);
}

/**
*/
void ASTERIXImportTask::decodeCategory(unsigned int category, bool decode)
{
    loginf << "cat " << category << " decode " << decode;

    settings_.decodeCategory(category, decode);
}

/**
*/
std::string ASTERIXImportTask::editionForCategory(unsigned int category)
{
    traced_assert(hasConfiguratonFor(category));

    auto* cfg = compass_.dbContextManager().asterixConfig(category);

    // check if edition exists, otherwise reset to default
    if (jasterix_->category(category)->editions().count(cfg->edition()) == 0)
    {
        loginf << "cat " << category << " reset to default edition";
        cfg->edition(jasterix_->category(category)->defaultEdition());
    }

    return cfg->edition();
}

/**
*/
std::string ASTERIXImportTask::refEditionForCategory(unsigned int category)
{
    traced_assert(hasConfiguratonFor(category));

    auto* cfg = compass_.dbContextManager().asterixConfig(category);

    // check if edition exists, otherwise reset to default
    if (cfg->ref().size() &&  // if value set and not exist in jASTERIX
        jasterix_->category(category)->refEditions().count(cfg->ref()) == 0)
    {
        loginf << "cat " << category << " reset to default ref";
        cfg->ref(jasterix_->category(category)->defaultREFEdition());
    }

    return cfg->ref();
}

/**
*/
std::string ASTERIXImportTask::spfEditionForCategory(unsigned int category)
{
    traced_assert(hasConfiguratonFor(category));

    auto* cfg = compass_.dbContextManager().asterixConfig(category);

    // check if edition exists, otherwise reset to default
    if (cfg->spf().size() &&  // if value set and not exist in jASTERIX
        jasterix_->category(category)->spfEditions().count(cfg->spf()) == 0)
    {
        loginf << "cat " << category << " reset to default spf";
        cfg->spf(jasterix_->category(category)->defaultSPFEdition());
    }

    return cfg->spf();
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

    // use small chunk size for decoding check (fast probe, not full import)
    static const unsigned int check_chunk_size = 2000;
    unsigned int saved_frame_chunk    = jASTERIX::frame_chunk_size;
    unsigned int saved_data_chunk     = jASTERIX::data_block_chunk_size;
    jASTERIX::frame_chunk_size      = check_chunk_size;
    jASTERIX::data_block_chunk_size = check_chunk_size;

    auto check_decoding = [ this ] (const AsyncTaskState& state, AsyncTaskProgressWrapper& progress)
    {
        //refresh decoder check
        traced_assert(this->decoder_);
        this->decoder_->canDecode(true, &progress);

        progress.setFinished(true);

        return Result::succeeded();
    };

    AsyncFuncTask task(check_decoding, "Testing Decoding", "Please wait...", false);
    task.runAsyncDialog(true, nullptr);

    // restore chunk sizes for actual import
    jASTERIX::frame_chunk_size      = saved_frame_chunk;
    jASTERIX::data_block_chunk_size = saved_data_chunk;

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
    logdbg;

    stopped_ = true;

    for (auto& sender : replay_senders_)
        sender->stop();
    replay_senders_.clear();

    if (data_received_timer_)
        data_received_timer_->stop();

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

    // If obfuscation is requested, restore the per-session mapping from
    // /tmp before any postprocess job starts. Load is gated to a single
    // successful invocation per process, so repeated imports reuse the
    // in-memory state without touching the file again.
    if (settings_.obfuscate_secondary_info_)
        ASTERIXPostprocessJob::loadObfuscationMaps();

    // re-push chunk sizes in case the RAM tier was changed after construction
    jASTERIX::frame_chunk_size      = settings_.chunk_size_jasterix;
    jASTERIX::data_block_chunk_size = settings_.chunk_size_jasterix;

    max_process_ram_gb_ = 0.0f;

    logRAMUsage("run");

    if (source_.isNetworkType())
    {
        compass_.appMode(AppMode::LiveRunning); // set live mode

        compass_.logInfo("ASTERIX Import") << "started: network";

        data_received_timer_.reset(new QTimer);
        connect(data_received_timer_.get(), &QTimer::timeout, this, &ASTERIXImportTask::checkDataReceivedSlot);
        data_received_timer_->setInterval(1000);
        data_received_timer_->start();
    }
    else
        compass_.logInfo("ASTERIX Import") << "started: files";


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
    ProjectionManager& proj_man = compass_.projectionManager();

    traced_assert(proj_man.hasCurrentProjection());
    Projection& projection = proj_man.currentProjection();
    projection.clearCoordinateSystems(); // to rebuild from data sources
    projection.addAllCoordinateSystems();

    loginf << "starting decode job";

    if (source_.isNetworkType())
        compass_.dbContextManager().createNetworkDBDataSources();

    decode_job_ = make_shared<ASTERIXDecodeJob>(*this, post_process_);

    connect(decode_job_.get(), &ASTERIXDecodeJob::doneSignal, this,
            &ASTERIXImportTask::decodeASTERIXDoneSlot, Qt::QueuedConnection);
    connect(decode_job_.get(), &ASTERIXDecodeJob::decodedASTERIXSignal, this,
            &ASTERIXImportTask::addDecodedASTERIXSlot, Qt::QueuedConnection);

    compass_.jobManager().addBlockingJob(decode_job_);

    loginf << "done";

    return;
}

/**
*/
void ASTERIXImportTask::decodeASTERIXDoneSlot()
{
    loginf << "called";

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
            QMessageBox msgBox(QApplication::activeWindow());
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
void ASTERIXImportTask::addDecodedASTERIXSlot()
{
    logdbg;
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

    logdbg << "ASTERIXImportTask: addDecodedASTERIXSlot:"
           << " num parsers " << schema_->parsers().size()
           << " num extracted_data " << extracted_data.size();

    std::shared_ptr<ASTERIXJSONMappingJob> json_map_job =
        make_shared<ASTERIXJSONMappingJob>(std::move(extracted_data), source_name,
                                           schema_->parsers());

    json_map_jobs_.push_back(json_map_job);

    traced_assert(!extracted_data.size());

    connect(json_map_job.get(), &ASTERIXJSONMappingJob::doneSignal, this,
            &ASTERIXImportTask::mapJSONDoneSlot, Qt::QueuedConnection);

    //loginf << "queueing in new mapping job, main thread is " << QThread::currentThreadId();

    compass_.jobManager().addNonBlockingJob(json_map_job);
}

/**
*/
void ASTERIXImportTask::mapJSONDoneSlot()
{
    logdbg << "called, stopped " << stopped_;

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

    logdbg << "num buffers " << job_buffers.size();

    if (!job_buffers.size())
    {
        logdbg << "empty buffers, returning early";
        traced_assert(num_packets_in_processing_);
        num_packets_in_processing_--;
        return;
    }

    logdbg << "starting ts calc";

    traced_assert(!ts_calculator_.processing());
    ts_calculator_.setBuffers(std::move(job_buffers));

    bool check_future_ts = source_.isNetworkType();

    if (settings_.network_ignore_future_ts_)
        check_future_ts = false;

    ts_calculator_.calculate(source_name,
                             settings_.date_, settings_.reset_date_between_files_,
                             settings_.override_tod_active_, settings_.override_tod_offset_,
                             settings_.ignore_time_jumps_, check_future_ts,
                             compass_);

    logdbg << "ts calc done, calling timestampCalculationDoneSlot";

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

void ASTERIXImportTask::timestampCalculationDoneSlot()
{
    logdbg;

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers {ts_calculator_.buffers()};
    ts_calculator_.setProcessingDone();

    std::shared_ptr<ASTERIXPostprocessJob> postprocess_job =
        make_shared<ASTERIXPostprocessJob>(std::move(job_buffers), settings_, compass_);

    postprocess_jobs_.push_back(postprocess_job);

    // check for future when net import
    connect(postprocess_job.get(), &ASTERIXPostprocessJob::doneSignal, this,
            &ASTERIXImportTask::postprocessDoneSlot, Qt::QueuedConnection);

    compass_.jobManager().addNonBlockingJob(postprocess_job);
}

/**
*/
void ASTERIXImportTask::postprocessDoneSlot()
{
    logdbg << "called";

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

        traced_assert(buf_it.second->hasProperty(dbcontent_vars::meta_var_timestamp_));

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
                !compass_.dbExportInProgress() &&
                !dbcontent_man_.dataEngine().isLoading())
            {
                logdbg << "inserting";
                traced_assert(!dbcontent_man_.insertInProgress());

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

                traced_assert(b.second->hasProperty(dbcontent_vars::meta_var_timestamp_));

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
            !compass_.dbExportInProgress() &&
            !dbcontent_man_.dataEngine().isLoading())
        {
            logdbg << "inserting";
            traced_assert(!dbcontent_man_.insertInProgress());

            insertData();
        }
    }
}

/**
*/
void ASTERIXImportTask::insertData()
{
    logdbg << "called";

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

    DBContentManager& dbcont_manager = dbcontent_man_;

    traced_assert(schema_);

    unsigned int current_num_records = 0;

    for (auto& job_it : job_buffers)
    {
        current_num_records += job_it.second->size();
        num_records_ += job_it.second->size();

        // paused still ingests live buffers, cleanup would e.g. strip an all-null
        // timestamp column of a chunk without any ToD (CAT001) before insert
        if (!isLiveSession(compass_.appMode())) // is cleaned special there
            job_it.second->deleteEmptyProperties();
    }

    logdbg << "inserting " << current_num_records << " records";

    logRAMUsage("before insert");

    if (!insert_slot_connected_)
    {
        loginf << "connecting slot";

        // must be queued: insertDoneSignal is emitted in DBContentManager::finishInserting()
        // before the min/max timestamps/positions are computed and set as DB properties.
        // A direct connection would run insertDoneSlot (and the finalization with
        // saveProperties) on stale values.
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
    logdbg << "called";

    traced_assert(insert_slot_connected_);

    if (source_.isFileType())
    {
        logdbg << "num_packets_in_processing " << num_packets_in_processing_;

        updateFileProgressDialog();
    }

    // has to be after file progress dialog update since calls processEvents and thus creates race condition
    traced_assert(insert_active_);
    insert_active_ = false;
    traced_assert(!dbcontent_man_.insertInProgress());

    --num_packets_in_processing_;

    last_live_update_time_ = boost::posix_time::microsec_clock::local_time();

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
void ASTERIXImportTask::checkDataReceivedSlot()
{
    loginf;

    traced_assert(compass_.appMode() == AppMode::LiveRunning);

    using namespace boost::posix_time;

    if (!num_packets_in_processing_
        && !compass_.dbInterface().cleanupInProgress()
        && (microsec_clock::local_time() - last_live_update_time_).total_seconds() > 5)
    {
        loginf << "forcing live update";
        QMetaObject::invokeMethod(&compass_.viewManager(), "forceLiveUpdate", Qt::QueuedConnection);
        last_live_update_time_ = microsec_clock::local_time();
    }
}

/**
 * Resolves the replay target endpoint and creates one replay sender per file,
 * sharing a common pacing base so the relative timing between the files is kept.
 * Fails without side effects, so it can be called before run().
*/
bool ASTERIXImportTask::prepareReplay(const std::vector<std::string>& filenames, float speed,
                                      const std::string& line_key, bool stop_at_end,
                                      std::string& error)
{
    traced_assert(replay_senders_.empty());

    if (filenames.empty())
    {
        error = "no replay files given";
        return false;
    }

    if (speed <= 0.0f)
    {
        error = "replay speed " + std::to_string(speed) + " invalid";
        return false;
    }

    // common pacing base: the earliest first frame time across all files
    boost::optional<double> reference_time;

    for (const auto& filename : filenames)
    {
        std::string parse_error;
        auto first_time = ASTERIXNetworkReplaySender::firstFrameTime(filename, &parse_error);

        if (!first_time.has_value())
        {
            error = parse_error;
            return false;
        }

        if (!reference_time.has_value() || first_time.value() < reference_time.value())
            reference_time = first_time;
    }

    // resolve the target endpoint: the first configured endpoint of the given line.
    // sending to a single endpoint is required, receivers of the same line merge
    // into one buffer and would store duplicate records otherwise
    std::string  configured_ip;
    std::string  target_ip;
    unsigned int target_port {0};

    for (auto& ds_it : compass_.dbContextManager().getNetworkLines())
    {
        if (!ds_it.second.count(line_key))
            continue;

        const nlohmann::json& line_cfg = ds_it.second.at(line_key);

        if (!line_cfg.contains("mcast_ip") || !line_cfg.contains("mcast_port"))
            continue;

        configured_ip = line_cfg.at("mcast_ip").get<std::string>();
        target_port   = line_cfg.at("mcast_port").get<unsigned int>();

        if (configured_ip.size() && target_port)
        {
            // the source address of locally sent datagrams cannot match a
            // configured sender filter, the receiver will drop the replayed data
            if (line_cfg.contains("sender_ip")
                && line_cfg.at("sender_ip").is_string()
                && line_cfg.at("sender_ip").get<std::string>().size())
                logwrn << "line " << line_key << " filters sender ip '"
                       << line_cfg.at("sender_ip").get<std::string>()
                       << "', replayed data will be dropped by the receiver";

            break;
        }
    }

    if (configured_ip.empty() || !target_port)
    {
        error = "no network line '" + line_key + "' with valid endpoint defined in active context";
        return false;
    }

    // multicast groups are usable as-is (sent host-local), listen addresses are not
    target_ip = ASTERIXNetworkReplaySender::effectiveTargetIP(configured_ip);

    if (target_ip != configured_ip)
        loginf << "using target ip " << target_ip << " for configured " << configured_ip;

    replay_stop_at_end_ = stop_at_end;

    for (const auto& filename : filenames)
    {
        replay_senders_.emplace_back(
            new ASTERIXNetworkReplaySender(filename, speed, target_ip, target_port, reference_time));

        connect(replay_senders_.back().get(), &ASTERIXNetworkReplaySender::doneSignal,
                this, &ASTERIXImportTask::replaySenderDoneSlot, Qt::QueuedConnection);

        loginf << "file '" << filename << "' speed " << speed << " line " << line_key
               << " target " << target_ip << ":" << target_port;
    }

    loginf << replay_senders_.size() << " files, reference time "
           << String::timeStringFromDouble(reference_time.value());

    return true;
}

/**
*/
void ASTERIXImportTask::startReplay()
{
    traced_assert(replay_senders_.size());

    for (auto& sender : replay_senders_)
        sender->start();
}

/**
*/
void ASTERIXImportTask::replaySenderDoneSlot()
{
    if (replay_senders_.empty()) // already torn down (e.g. manual stop)
        return;

    // act only once ALL senders finished
    for (const auto& sender : replay_senders_)
        if (sender->isRunning())
            return;

    loginf << "all " << replay_senders_.size() << " senders done, stop at end "
           << replay_stop_at_end_;

    for (auto& sender : replay_senders_)
        sender->stop(); // joins the finished threads
    replay_senders_.clear();

    if (replay_stop_at_end_ && running_ && compass_.appMode() != AppMode::Offline)
        compass_.mainWindow().liveStopSlot();
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

        if (data_received_timer_ && !data_received_timer_->isActive())
            data_received_timer_->start();
    }
    else if (app_mode_current == AppMode::LivePaused)
    {
        traced_assert(app_mode_previous == AppMode::LiveRunning); // can only happen from running

        if (data_received_timer_)
            data_received_timer_->stop();
    }
    else if (app_mode_current == AppMode::Offline)
    {
        traced_assert(app_mode_previous == AppMode::LiveRunning || app_mode_previous == AppMode::LivePaused);

        // data_received_timer_->stop() called inside stop()
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

    // drain: once decoding and the map/ts/postprocess pipeline are done, no future chunk
    // can grow the accumulation, so flush any remainder for insertion. required since the
    // final postprocessDoneSlot can run while decode_job_ is still set (its doneSignal
    // queued but not yet delivered) and then postpones the flush waiting for a next chunk
    // that never comes
    if (!stopped_
        && decode_job_ == nullptr
        && !json_map_jobs_.size()
        && !ts_calculator_.processing()
        && !postprocess_jobs_.size()
        && accumulated_buffers_.size())
    {
        loginf << "flushing remaining accumulated buffers after pipeline drain";

        // the postponed accumulation already decremented the packet count,
        // insertDoneSlot decrements once per queued insert
        ++num_packets_in_processing_;

        queued_insert_buffers_.push_back(std::move(accumulated_buffers_));
        accumulated_buffers_.clear();

        if (!insert_active_
            && !compass_.dbExportInProgress()
            && !dbcontent_man_.dataEngine().isLoading())
        {
            insertData(); // calls checkAllDone again when done via insertDoneSlot
            return;
        }

        // insert active: insertDoneSlot picks up the queued buffers and calls checkAllDone again
    }

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

        ts_calculator_.logLastTimestamp(compass_);

        all_done_ = true;
        done_     = true; // why was this not set?
        running_  = false;

        boost::posix_time::time_duration time_diff = boost::posix_time::microsec_clock::local_time() - start_time_;
        loginf << "import done after "
               << String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, false);

        int records_per_second = num_records_ / std::max(1.0, time_diff.total_milliseconds() / 1000.0);

        compass_.logInfo("ASTERIX Import")
            << " finished after "
            << String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, false)
            << ", inserted " << num_records_ << " rec"
            << " with " << records_per_second << " rec/s";

        auto errors   = decoder_->errors();
        auto warnings = decoder_->warnings();

        if (!errors.empty())
        {
            std::stringstream ss;
            for (const auto& e : errors)
                ss << e << "\n";

            compass_.logError("ASTERIX Import")
                << "Import finished with errors." << "\n\n"
                << ss.str();
        }
        if (!warnings.empty())
        {
            std::stringstream ss;
            for (const auto& w : warnings)
                ss << w << "\n";

            compass_.logWarn("ASTERIX Import")
                << "Import finished with warnings." << "\n\n"
                << ss.str();
        }

        compass_.mainWindow().updateMenus(); // re-enable import menu

        logdbg << "refresh";

        refreshjASTERIX();

        logdbg << "db content";

        //emit COMPASS::instance().interface().databaseContentChangedSignal();

        logdbg << "status logging";

        if (!allow_user_interactions_)
        {
            logdbg << "deleting status widget";
        }

        compass_.dbContextManager().saveCountsToDB();

        // merge per-DS / per-CAT / per-item probe stats into the cumulative
        // store and persist them for this DB
        {
            auto agg = ASTERIXImportProbeAggregator::aggregate(
                source_, compass_.dbContextManager());

            context::DBContextManager::AsterixInfoMap delta;
            for (const auto& [ds_id, ds_probe] : agg.probe_by_dsid)
            {
                for (const auto& [cat, cat_probe] : ds_probe.categories)
                {
                    auto& dst = delta[ds_id][cat];
                    dst.total_count = cat_probe.total_count;
                    for (const auto& [item, st] : cat_probe.items)
                    {
                        auto& di = dst.items[item];
                        di.count = st.count;
                        di.min   = st.min;
                        di.max   = st.max;
                    }
                }
            }

            compass_.dbContextManager().mergeAsterixInfo(delta);
            compass_.dbContextManager().saveAsterixInfoToDB();
        }

        // dbContentStatusChanged emitted by DBContentManager::finishInserting()
        compass_.dbInterface().saveProperties();

        logRAMUsage("finalize before malloc_trim");

        malloc_trim(0); // release unused memory

        logRAMUsage("finalize after malloc_trim");

        if (insert_slot_connected_) // moved here from insertDoneSlot
        {
            disconnect(&dbcontent_man_, &DBContentManager::insertDoneSignal,
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
            compass_.dbInterface().cleanupDB(true);

        // append this import to the persistent "ASTERIX Import" task result.
        // Skip on cancel/error: we never open the result so previous content
        // (from earlier imports) stays untouched.
        if (!stopped_ && !error_ && compass_.hasActiveContext())
        {
            beginResultReport();
            buildResultReport(boost::posix_time::microsec_clock::local_time());
            compass_.taskManager().endTaskResultWriting(true, allow_user_interactions_);
        }

        // Persist the obfuscation maps so the next import (this run or the
        // next compass session before reboot) reuses the same mappings.
        // Skip on cancel/error to avoid serialising a partial state.
        if (settings_.obfuscate_secondary_info_ && !stopped_ && !error_)
            ASTERIXPostprocessJob::saveObfuscationMaps();

        emit doneSignal();
    }

    logdbg << "done";
}

/**
*/
void ASTERIXImportTask::logRAMUsage(const std::string& context)
{
    float process_ram = Utils::System::getProcessRAMinGB();

    if (process_ram > max_process_ram_gb_)
        max_process_ram_gb_ = process_ram;

    double elapsed_s = (boost::posix_time::microsec_clock::local_time() - start_time_).total_milliseconds() / 1000.0;
    int records_per_second = num_records_ / std::max(1.0, elapsed_s);

    loginf << "RAM [" << context << "]"
           << " process " << String::doubleToStringPrecision(process_ram, 2) << " GB"
           << " max " << String::doubleToStringPrecision(max_process_ram_gb_, 2) << " GB"
           << " free " << String::doubleToStringPrecision(Utils::System::getFreeRAMinGB(), 2) << " GB"
           << " total " << String::doubleToStringPrecision(Utils::System::getTotalRAMinGB(), 2) << " GB"
           << " records " << num_records_ << " rec/s " << records_per_second;
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
            new QProgressDialog(("Files '" + source_.filesAsString() + "'").c_str(), "Abort", 0, 100,
                                QApplication::activeWindow()));
        file_progress_dialog_->setWindowTitle("Importing ASTERIX Recording(s)");
        file_progress_dialog_->setWindowModality(Qt::ApplicationModal);
        file_progress_dialog_->setAutoClose(false);
        file_progress_dialog_->setAutoReset(false);

        // wrapping label with a bounded width: long recording paths otherwise
        // stretch the dialog to the longest filename (the status table inserts break
        // opportunities after every path separator, see statusInfoString)
        auto* label = new QLabel;
        label->setTextFormat(Qt::RichText);
        label->setWordWrap(true);
        label->setMaximumWidth(800);
        file_progress_dialog_->setLabel(label);

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
        file_progress_dialog_->setLabelText("Writing to database...");
    }
}

/**
*/
void ASTERIXImportTask::onConfigurationChanged(const std::vector<std::string>& changed_params)
{
    // date_ is a derived value computed from date_str_ at construction; mirror that here
    // so runtime reconfiguration via applyJSONParameters takes effect
    if (std::find(changed_params.begin(), changed_params.end(), "date_str") != changed_params.end())
    {
        if (settings_.date_str_.size())
            settings_.date_ = Time::fromDateString(settings_.date_str_);
        if (settings_.date_.is_not_a_date_time())
            settings_.date_ = boost::posix_time::ptime(boost::gregorian::day_clock::universal_day());
    }

    emit configChanged();
}

/**
*/
void ASTERIXImportTask::runDialog(QWidget* parent)
{
    //show dialog
    ASTERIXImportTaskDialog dlg(*this, parent);

    //canceled?
    if (dlg.exec() != QDialog::Accepted)
        return;

    //enable user interactions
    allowUserInteractions(true);

    //otherwise run import
    run();
}

namespace
{
    /// Middle-ellipsis a string to fit `max_chars`. Preserves the file
    /// extension where possible (last 12 chars).
    std::string abbreviateForHeading(const std::string& s, std::size_t max_chars)
    {
        if (s.size() <= max_chars || max_chars < 12)
            return s;

        const std::size_t tail = std::min<std::size_t>(12, max_chars / 2);
        const std::size_t head = max_chars - tail - 1; // 1 char for ellipsis
        return s.substr(0, head) + "…" + s.substr(s.size() - tail);
    }

    /// Human-readable byte size (decimal SI prefixes), 2 decimals from kB up.
    std::string formatBytes(std::size_t bytes)
    {
        const double b = static_cast<double>(bytes);
        if (b >= 1e9)
            return Utils::String::doubleToStringPrecision(b * 1e-9, 2) + " GB";
        if (b >= 1e6)
            return Utils::String::doubleToStringPrecision(b * 1e-6, 2) + " MB";
        if (b >= 1e3)
            return Utils::String::doubleToStringPrecision(b * 1e-3, 2) + " kB";
        return std::to_string(bytes) + " B";
    }

}

/**
*/
void ASTERIXImportTask::beginResultReport()
{
    // Single, persistent task result accumulating across all ASTERIX imports
    // into the open DB. Pass clear_existing=false so prior imports' content
    // is not wiped on each call. Network imports share the same result.
    compass_.taskManager().beginTaskResultWriting(
        "ASTERIX Import", task::TaskResultType::Generic, /*clear_existing=*/false);
}

/**
 * Build/append this import's content to the persistent "ASTERIX Import" report.
 * Each imported file becomes a new section under "Files" and a new row in the
 * Overview "Imported Files" table that links to that section. Earlier imports'
 * sections are left untouched.
 */
void ASTERIXImportTask::buildResultReport(const boost::posix_time::ptime& end_time)
{
    auto& tm = compass_.taskManager();
    auto report = tm.currentReport();
    if (!report)
        return;

    const double elapsed_s =
        std::max(0.0, (end_time - start_time_).total_milliseconds() / 1000.0);
    const std::string elapsed_str = Utils::String::timeStringFromDouble(elapsed_s, false);
    const std::string source_type = source_.sourceTypeAsString();
    const std::string framing =
        settings_.activeFileFraming().empty() ? std::string("(none)")
                                              : settings_.activeFileFraming();

    // ---- Overview / Imported Files (one row per imported file) ----
    auto& overview = report->getSection("Overview");
    if (!overview.hasTable("Imported Files"))
        overview.addTable("Imported Files", 9,
                          {"#", "Begin", "Elapsed", "File",
                           "Size", "Source", "Framing",
                           "Errors", "Warnings"},
                          true, 0);
    auto& overview_t = overview.getTable("Imported Files");

    // existing row count = number of files previously listed → next index N
    const std::size_t prev_total = overview_t.numRows();

    // ---- Files tree: one new section per file ----
    auto& jasterix = jasterix_; // current shared instance

    int local_idx = 0;
    for (const auto& fi : source_.files())
    {
        if (!fi.used)
            continue;

        ++local_idx;
        const std::size_t global_idx = prev_total + local_idx; // 1-based

        const std::string basename = Utils::Files::getFilenameFromPath(fi.filename);
        const std::string heading_basename = abbreviateForHeading(basename, 60);

        std::string file_heading = "#" + std::to_string(global_idx) + " " + heading_basename;
        // colons are the section path separator - strip them defensively
        std::replace(file_heading.begin(), file_heading.end(), ':', '_');

        const std::string section_id = "Files:" + file_heading;

        // overview row (linked)
        const bool fi_has_error  = fi.hasError();
        const bool fi_has_warn   = fi.hasWarning();
        overview_t.addRow(
            {static_cast<long long>(global_idx),
             Utils::Time::toString(start_time_, 0),
             elapsed_str,
             basename,
             formatBytes(fi.sizeInBytes()),
             source_type,
             framing,
             fi_has_error ? std::string("yes") : std::string("no"),
             fi_has_warn  ? std::string("yes") : std::string("no")},
            ResultReport::SectionContentViewable(),
            section_id);

        // per-file section (always fresh - global_idx is unique)
        auto& file_section = report->getSection(section_id);

        // Info
        if (!file_section.hasTable("Info"))
            file_section.addTable("Info", 2, {"Property", "Value"}, false);
        {
            auto& info_t = file_section.getTable("Info");
            info_t.addRow({"Filename",      basename});
            info_t.addRow({"Path",          fi.filename});
            info_t.addRow({"Size",          formatBytes(fi.sizeInBytes())});
            if (!fi.contentinfo.empty())
                info_t.addRow({"Content info", fi.contentinfo});
            info_t.addRow({"Source",        source_type});
            info_t.addRow({"Framing",       framing});
            info_t.addRow({"Imported on",   Utils::Time::toString(start_time_, 0)});
            info_t.addRow({"Elapsed",       elapsed_str});
            info_t.addRow({"PCAP sections", static_cast<long long>(fi.sections.size())});
        }

        // Records per data source: one row per DS seen in the file; the
        // "Records" cell lists "<dbcontent>: <count>" per CAT, one per line.
        auto file_probe = ASTERIXImportProbeAggregator::aggregateFile(fi);
        if (file_probe.probe_available)
        {
            if (!file_section.hasTable("Records per Data Source"))
                file_section.addTable("Records per Data Source", 3,
                                      {"Data Source", "DS Type", "Records"}, true);
            auto& rec_t = file_section.getTable("Records per Data Source");

            for (const auto& [ds_id, ds_probe] : file_probe.probe_by_dsid)
            {
                const unsigned int sac = Utils::Number::sacFromDsId(ds_id);
                const unsigned int sic = Utils::Number::sicFromDsId(ds_id);

                std::string ds_label;
                if (auto* ds = compass_.dbContextManager().dataSource(ds_id))
                    ds_label = ds->name() + " (" + std::to_string(sac) + "/" + std::to_string(sic) + ")";
                else
                    ds_label = std::to_string(sac) + "/" + std::to_string(sic);

                std::string ds_type;
                if (auto* ds = compass_.dbContextManager().dataSource(ds_id))
                    ds_type = ds->dsType();
                if (ds_type.empty())
                    ds_type = ASTERIXImportProbeAggregator::inferDsType(ds_probe.categories);
                if (ds_type.empty())
                    ds_type = "Other";

                std::string records_text;
                bool first = true;
                for (const auto& [cat, cat_probe] : ds_probe.categories)
                {
                    std::string dbc_name;
                    if (schema_ && schema_->hasObjectParser(cat))
                        dbc_name = schema_->parser(cat).dbContentName();
                    if (dbc_name.empty())
                        dbc_name = "CAT" + Utils::String::categoryString(cat);

                    if (!first)
                        records_text += "\n";
                    first = false;
                    records_text += dbc_name + ": " + std::to_string(cat_probe.total_count);
                }

                // Build the section path matching what the DSType/DS-tree
                // block below creates: file_heading is already colon-stripped;
                // mirror the strip on ds_type and ds_label so the link
                // resolves correctly via Report::getSection.
                std::string ds_type_link  = ds_type;
                std::string ds_label_link = ds_label;
                std::replace(ds_type_link.begin(),  ds_type_link.end(),  ':', '_');
                std::replace(ds_label_link.begin(), ds_label_link.end(), ':', '_');
                const std::string ds_link =
                    section_id + ":" + ds_type_link + ":" + ds_label_link;

                rec_t.addRow({ds_label, ds_type, records_text},
                             ResultReport::SectionContentViewable(),
                             ds_link);
            }
        }

        // Skipped categories: found in the recording during the probe but not
        // decoded (no specification available or decoding disabled). Aggregated
        // over the file itself and its used sections.
        {
            std::map<unsigned int, ASTERIXSkippedCategoryInfo> skipped = fi.skipped_categories;
            for (const auto& sec : fi.sections)
            {
                if (!sec.used)
                    continue;
                for (const auto& cat_it : sec.skipped_categories)
                {
                    auto& info = skipped[cat_it.first];
                    info.data_blocks += cat_it.second.data_blocks;
                    info.bytes       += cat_it.second.bytes;
                    if (info.reason.empty())
                        info.reason = cat_it.second.reason;
                }
            }

            if (!skipped.empty())
            {
                if (!file_section.hasTable("Skipped Categories"))
                    file_section.addTable("Skipped Categories", 4,
                                          {"Category", "Data Blocks", "Size", "Reason"}, false);
                auto& skip_t = file_section.getTable("Skipped Categories");

                for (const auto& cat_it : skipped)
                    skip_t.addRow({"CAT" + Utils::String::categoryString(cat_it.first),
                                   static_cast<long long>(cat_it.second.data_blocks),
                                   formatBytes(cat_it.second.bytes),
                                   cat_it.second.reason});
            }
        }

        // Data sources: DSType group sub-section > DS sub-section > one table per CAT.
        if (file_probe.probe_available)
        {
            // Group ds_id -> ds_type
            std::map<std::string,
                     std::map<unsigned int,
                              const ASTERIXImportProbeAggregator::DSProbe*>> by_type;

            for (const auto& [ds_id, ds_probe] : file_probe.probe_by_dsid)
            {
                std::string ds_type;
                if (auto* ds = compass_.dbContextManager().dataSource(ds_id))
                    ds_type = ds->dsType();
                if (ds_type.empty())
                    ds_type = ASTERIXImportProbeAggregator::inferDsType(ds_probe.categories);
                if (ds_type.empty())
                    ds_type = "Other";

                by_type[ds_type][ds_id] = &ds_probe;
            }

            for (const auto& [ds_type, ds_map] : by_type)
            {
                auto& type_section = file_section.hasSubSection(ds_type)
                                         ? file_section.getSubSection(ds_type)
                                         : file_section.addSubSection(ds_type);

                for (const auto& [ds_id, ds_probe_ptr] : ds_map)
                {
                    const auto& ds_probe = *ds_probe_ptr;
                    const unsigned int sac = Utils::Number::sacFromDsId(ds_id);
                    const unsigned int sic = Utils::Number::sicFromDsId(ds_id);

                    std::string ds_label;
                    if (auto* ds = compass_.dbContextManager().dataSource(ds_id))
                        ds_label = ds->name() + " (" + std::to_string(sac) + "/" + std::to_string(sic) + ")";
                    else
                        ds_label = std::to_string(sac) + "/" + std::to_string(sic);
                    std::replace(ds_label.begin(), ds_label.end(), ':', '_');

                    auto& ds_section = type_section.hasSubSection(ds_label)
                                           ? type_section.getSubSection(ds_label)
                                           : type_section.addSubSection(ds_label);

                    // Build the renderer's view of this DS's per-CAT data
                    // and delegate to the shared helper.
                    std::map<unsigned int, ASTERIXReportHelpers::CategoryView> view_cats;
                    for (const auto& [cat, cat_probe] : ds_probe.categories)
                    {
                        auto& cv = view_cats[cat];
                        cv.total_count = cat_probe.total_count;
                        for (const auto& [key, stats] : cat_probe.items)
                        {
                            cv.items[key] = { stats.count, &stats.min, &stats.max };
                        }
                    }

                    ASTERIXReportHelpers::renderDataItemTablesForDS(
                        ds_section, view_cats, jasterix.get());
                }
            }
        }

        // Errors / Warnings (probe-time, per file + sections)
        if (fi.error.hasError() || !fi.warning.empty()
            || std::any_of(fi.sections.begin(), fi.sections.end(),
                           [](const auto& s){ return s.used && (s.error.hasError() || !s.warning.empty()); }))
        {
            auto& issues_text = file_section.addText("Errors / Warnings");
            if (fi.error.hasError())
                issues_text.addText(std::string("Error: ") + fi.error.errinfo);
            if (!fi.warning.empty())
                issues_text.addText(std::string("Warning: ") + fi.warning);
            for (const auto& sec : fi.sections)
            {
                if (!sec.used)
                    continue;
                if (sec.error.hasError())
                    issues_text.addText("Section " + sec.id + " error: " + sec.error.errinfo);
                if (!sec.warning.empty())
                    issues_text.addText("Section " + sec.id + " warning: " + sec.warning);
            }
        }
    }

    // For network mode, source_.files() carries the synthesized probe entry -
    // it's already covered by the loop above. No extra handling.

    // Sections cache their content widget after first display. We have just
    // appended new rows/sections to a report that may already be visible in
    // the Task Results panel; rebuild every section's UI so the additions
    // show up without the user having to re-open the result.
    report->updateContents();
}

