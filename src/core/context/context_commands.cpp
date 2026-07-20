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

#include "context_commands.h"
#include "rtcommand_registry.h"
#include "db_context_manager.h"
#include "db_context.h"
#include "db_context_serializer.h"
#include "sector_import_utils.h"
#include "compass.h"
#include "logger.h"
#include "util/files.h"
#include "json.hpp"

#include <boost/program_options.hpp>

using namespace std;

REGISTER_RTCOMMAND(context_cmd::RTCommandCreateContext)
REGISTER_RTCOMMAND(context_cmd::RTCommandSetContext)
REGISTER_RTCOMMAND(context_cmd::RTCommandDeleteContext)
REGISTER_RTCOMMAND(context_cmd::RTCommandListContexts)
REGISTER_RTCOMMAND(context_cmd::RTCommandGetContextInfo)
REGISTER_RTCOMMAND(context_cmd::RTCommandGetContext)
REGISTER_RTCOMMAND(context_cmd::RTCommandImportFFTsJSON)
REGISTER_RTCOMMAND(context_cmd::RTCommandDeleteAllSectors)
REGISTER_RTCOMMAND(context_cmd::RTCommandDeleteAllFFTs)
REGISTER_RTCOMMAND(context_cmd::RTCommandImportSectorsGDAL)
REGISTER_RTCOMMAND(context_cmd::RTCommandExportSectorsJSON)
REGISTER_RTCOMMAND(context_cmd::RTCommandExportDataSourcesJSON)

namespace context_cmd
{

void init_context_commands()
{
    RTCommandCreateContext::init();
    RTCommandSetContext::init();
    RTCommandDeleteContext::init();
    RTCommandListContexts::init();
    RTCommandGetContextInfo::init();
    RTCommandGetContext::init();
    RTCommandImportFFTsJSON::init();
    RTCommandDeleteAllSectors::init();
    RTCommandDeleteAllFFTs::init();
    RTCommandImportSectorsGDAL::init();
    RTCommandExportSectorsJSON::init();
    RTCommandExportDataSourcesJSON::init();
}

// ============================================================
// create_context
// ============================================================

rtcommand::IsValid RTCommandCreateContext::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(name_.empty(), "Context name empty")
    return RTCommand::valid();
}

bool RTCommandCreateContext::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    try
    {
        if (ctx_man.hasContext(name_))
        {
            loginf << "context '" << name_ << "' already exists, deleting first";
            ctx_man.deleteContext(name_);
        }

        ctx_man.createContext(name_);
    }
    catch (const exception& e)
    {
        setResultMessage(string("Create failed: ") + e.what());
        return false;
    }

    loginf << "created context '" << name_ << "'";

    return true;
}

void RTCommandCreateContext::collectOptions_impl(OptionsDescription& options,
                                                  PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("name,n", po::value<string>()->required(), "context name");
    ADD_RTCOMMAND_POS_OPTION(positional, "name")
}

void RTCommandCreateContext::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "name", string, name_)
}

// ============================================================
// set_context
// ============================================================

rtcommand::IsValid RTCommandSetContext::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(name_.empty(), "Context name empty")
    return RTCommand::valid();
}

bool RTCommandSetContext::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasContext(name_))
    {
        setResultMessage("Context '" + name_ + "' does not exist");
        return false;
    }

    if (compass_->dbOpened() && ctx_man.hasInsertedData())
    {
        setResultMessage("Cannot switch context while a database with imported data is open");
        return false;
    }

    ctx_man.setActiveContext(name_);

    loginf << "set active context to '" << name_ << "'";

    return true;
}

void RTCommandSetContext::collectOptions_impl(OptionsDescription& options,
                                               PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("name,n", po::value<string>()->required(), "context name");
    ADD_RTCOMMAND_POS_OPTION(positional, "name")
}

void RTCommandSetContext::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "name", string, name_)
}

// ============================================================
// delete_context
// ============================================================

rtcommand::IsValid RTCommandDeleteContext::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(name_.empty(), "Context name empty")
    return RTCommand::valid();
}

bool RTCommandDeleteContext::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasContext(name_))
    {
        setResultMessage("Context '" + name_ + "' does not exist");
        return false;
    }

    try
    {
        ctx_man.deleteContext(name_);
    }
    catch (const exception& e)
    {
        setResultMessage(string("Delete failed: ") + e.what());
        return false;
    }

    loginf << "deleted context '" << name_ << "'";

    return true;
}

void RTCommandDeleteContext::collectOptions_impl(OptionsDescription& options,
                                                  PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("name,n", po::value<string>()->required(), "context name");
    ADD_RTCOMMAND_POS_OPTION(positional, "name")
}

void RTCommandDeleteContext::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "name", string, name_)
}

// ============================================================
// list_contexts
// ============================================================

bool RTCommandListContexts::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    nlohmann::json j;
    j["contexts"] = nlohmann::json::array();

    for (const auto& name : ctx_man.contextNames())
        j["contexts"].push_back(name);

    j["active"] = ctx_man.hasActiveContext() ? ctx_man.activeContextName() : "";

    setJSONReply(j);

    return true;
}

// ============================================================
// get_context_info
// ============================================================

bool RTCommandGetContextInfo::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    const auto& ctx = ctx_man.activeContext();

    nlohmann::json j;
    j["name"]               = ctx.name();
    j["data_source_count"]  = ctx.dataSources().size();
    j["fft_count"]          = ctx.ffts().size();
    j["sector_count"]       = ctx.sectors().size();
    j["sector_layer_count"] = ctx_man.sectorLayers().size();

    setJSONReply(j);

    return true;
}

// ============================================================
// get_context
// ============================================================

rtcommand::IsValid RTCommandGetContext::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(name_.empty(), "Context name empty")
    return RTCommand::valid();
}

bool RTCommandGetContext::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasContext(name_))
    {
        setResultMessage("Context '" + name_ + "' does not exist");
        return false;
    }

    setJSONReply(context::DBContextSerializer::toJSON(ctx_man.context(name_)));

    return true;
}

void RTCommandGetContext::collectOptions_impl(OptionsDescription& options,
                                              PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("name,n", po::value<string>()->required(), "context name");
    ADD_RTCOMMAND_POS_OPTION(positional, "name")
}

void RTCommandGetContext::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "name", string, name_)
}

// ============================================================
// import_ffts_json
// ============================================================

rtcommand::IsValid RTCommandImportFFTsJSON::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(filename_.empty(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Utils::Files::fileExists(filename_),
                                      string("File '") + filename_ + "' does not exist")
    return RTCommand::valid();
}

bool RTCommandImportFFTsJSON::run_impl()
{
    if (!compass_->dbContextManager().hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    try
    {
        compass_->dbContextManager().importFFTs(filename_);
    }
    catch (const exception& e)
    {
        setResultMessage(string("Import failed: ") + e.what());
        return false;
    }

    return true;
}

void RTCommandImportFFTsJSON::collectOptions_impl(OptionsDescription& options,
                                                    PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("filename,f", po::value<string>()->required(), "JSON file path");
    ADD_RTCOMMAND_POS_OPTION(positional, "filename")
}

void RTCommandImportFFTsJSON::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", string, filename_)
}

// ============================================================
// delete_all_sectors
// ============================================================

bool RTCommandDeleteAllSectors::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    try
    {
        ctx_man.deleteAllSectors();
    }
    catch (const exception& e)
    {
        setResultMessage(string("Delete failed: ") + e.what());
        return false;
    }

    return true;
}

// ============================================================
// delete_all_ffts
// ============================================================

bool RTCommandDeleteAllFFTs::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    try
    {
        ctx_man.deleteAllFFTs();
    }
    catch (const exception& e)
    {
        setResultMessage(string("Delete failed: ") + e.what());
        return false;
    }

    return true;
}

// ============================================================
// import_sectors_gdal
// ============================================================

rtcommand::IsValid RTCommandImportSectorsGDAL::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(filename_.empty(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Utils::Files::fileExists(filename_),
                                      string("File '") + filename_ + "' does not exist")
    return RTCommand::valid();
}

bool RTCommandImportSectorsGDAL::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    try
    {
        auto sectors = sector_utils::parseGDALFile(filename_);
        if (sectors.empty())
        {
            setResultMessage("No sectors found in file '" + filename_ + "'");
            return false;
        }

        string layer = layer_name_;
        if (layer.empty())
        {
            // use filename without extension as layer name
            auto pos = filename_.find_last_of("/\\");
            string basename = (pos != string::npos) ? filename_.substr(pos + 1) : filename_;
            auto dot = basename.find_last_of('.');
            layer = (dot != string::npos) ? basename.substr(0, dot) : basename;
        }

        QColor color = color_str_.empty() ? QColor("#4c88ff") : QColor(QString::fromStdString(color_str_));

        unsigned int imported_cnt = 0;
        unsigned int replaced_cnt = 0;

        for (const auto& sec : sectors)
        {
            bool replaced = false;
            if (ctx_man.createOrReplaceSector(sec.name, layer, exclude_, color, sec.points, &replaced))
                ++imported_cnt;
            if (replaced)
                ++replaced_cnt;
        }

        loginf << "imported " << imported_cnt << " of " << sectors.size()
               << " sectors from GDAL file into layer '" << layer << "', " << replaced_cnt << " replaced";

        nlohmann::json j;
        j["imported_count"] = imported_cnt;
        j["replaced_count"] = replaced_cnt;
        j["layer"] = layer;
        setJSONReply(j);
    }
    catch (const exception& e)
    {
        setResultMessage(string("Import failed: ") + e.what());
        return false;
    }

    return true;
}

void RTCommandImportSectorsGDAL::collectOptions_impl(OptionsDescription& options,
                                                      PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("filename,f", po::value<string>()->required(), "GDAL-supported file path (SHP, GeoJSON, KML)")
        ("layer,l", po::value<string>()->default_value(""), "target layer name (default: filename)")
        ("color,c", po::value<string>()->default_value(""), "sector color as hex string, e.g. '#ff4c4c' (default: '#4c88ff')")
        ("exclude,e", po::value<bool>()->default_value(false), "mark sectors as exclusion sectors");
    ADD_RTCOMMAND_POS_OPTION(positional, "filename")
}

void RTCommandImportSectorsGDAL::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", string, filename_)
    RTCOMMAND_GET_VAR(variables, "layer", string, layer_name_)
    RTCOMMAND_GET_VAR(variables, "color", string, color_str_)
    RTCOMMAND_GET_VAR(variables, "exclude", bool, exclude_)
}

// ============================================================
// export_sectors_json
// ============================================================

rtcommand::IsValid RTCommandExportSectorsJSON::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(filename_.empty(), "Filename empty")
    return RTCommand::valid();
}

bool RTCommandExportSectorsJSON::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    try
    {
        ctx_man.exportSectors(filename_);
    }
    catch (const exception& e)
    {
        setResultMessage(string("Export failed: ") + e.what());
        return false;
    }

    return true;
}

void RTCommandExportSectorsJSON::collectOptions_impl(OptionsDescription& options,
                                                      PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("filename,f", po::value<string>()->required(), "output JSON file path");
    ADD_RTCOMMAND_POS_OPTION(positional, "filename")
}

void RTCommandExportSectorsJSON::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", string, filename_)
}

// ============================================================
// export_data_sources_json
// ============================================================

rtcommand::IsValid RTCommandExportDataSourcesJSON::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(filename_.empty(), "Filename empty")
    return RTCommand::valid();
}

bool RTCommandExportDataSourcesJSON::run_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    if (!ctx_man.hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    try
    {
        ctx_man.exportSensors(filename_);
    }
    catch (const exception& e)
    {
        setResultMessage(string("Export failed: ") + e.what());
        return false;
    }

    return true;
}

void RTCommandExportDataSourcesJSON::collectOptions_impl(OptionsDescription& options,
                                                          PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("filename,f", po::value<string>()->required(), "output JSON file path");
    ADD_RTCOMMAND_POS_OPTION(positional, "filename")
}

void RTCommandExportDataSourcesJSON::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", string, filename_)
}

} // namespace context_cmd
