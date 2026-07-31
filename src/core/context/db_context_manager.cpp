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

#include "db_context_manager.h"
#include "db_context_serializer.h"
#include "db_context_diff.h"

#include "asterix_decoding_config.h"
#include "color_provider.h"
#include "compass.h"

#include <jasterix/jasterix.h>
#include <jasterix/category.h>
#include "dbinterface.h"
#include "sector.h"
#include "sectorlayer.h"
#include "airspace.h"
#include "datasourcestoolwidget.h"
#include "datasourcesstatustoolwidget.h"
#include "db_context_conflict_dialog.h"
#include "db_context_merge_dialog.h"
#include "files.h"
#include "logger.h"
#include "number.h"
#include "traced_assert.h"

#include <json.hpp>

#include <boost/filesystem.hpp>

#include <QApplication>

#include <QPushButton>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

using namespace std;
using namespace nlohmann;
namespace fs = boost::filesystem;

namespace context
{

static const string ACTIVE_CONTEXT_FILENAME = "active_context.json";

DBContextManager::DBContextManager(COMPASS& compass)
    : QObject()
    , compass_(compass)
{
    // ensure base directory exists
    fs::create_directories(basePath());

    // load available contexts
    loadContextList();

    // load last active context name
    loadActiveContextName();

    // build sector layers if a context is already active
    if (hasActiveContext())
    {
        loginf << "constructor: rebuilding sector layers for active context '" << active_context_name_ << "'";
        rebuildSectorLayers();
        loginf << "constructor: sectors_loaded_=" << sectors_loaded_;
    }

    loginf << "initialized with " << contexts_.size() << " contexts"
           << (active_context_name_.empty() ? "" : ", active: " + active_context_name_);
}

DBContextManager::~DBContextManager() = default;

double DBContextManager::SensorConfig::sensorStatusMaxStatusAgeValue() const
{
    traced_assert(sensor_status_max_status_age_options.is_array());
    traced_assert(sensor_status_max_status_age_index < sensor_status_max_status_age_options.size());

    unsigned int value = sensor_status_max_status_age_options.at(sensor_status_max_status_age_index);

    return (double)value;
}

double DBContextManager::SensorConfig::sensorStatusMaxStatusAgeMaxValue() const
{
    traced_assert(sensor_status_max_status_age_options.is_array());
    traced_assert(sensor_status_max_status_age_options.size() > 0);

    unsigned int value = sensor_status_max_status_age_options.back();

    return (double)value;
}

// ============================================================
// CRUD
// ============================================================

vector<string> DBContextManager::contextNames() const
{
    vector<string> names;
    for (const auto& [name, ctx] : contexts_)
        names.push_back(name);
    return names;
}

bool DBContextManager::hasContext(const string& name) const
{
    return contexts_.count(name) > 0;
}

const DBContext& DBContextManager::context(const string& name) const
{
    traced_assert(hasContext(name));
    return contexts_.at(name);
}

DBContext& DBContextManager::context(const string& name)
{
    traced_assert(hasContext(name));
    return contexts_.at(name);
}

void DBContextManager::createContext(const string& name)
{
    traced_assert(!name.empty());
    traced_assert(!hasContext(name));

    DBContext ctx(name);

    // populate default ASTERIX decoding configs from jASTERIX definitions
    std::string jasterix_definition_path = HOME_DATA_DIRECTORY + "jasterix_definitions";

    if (Utils::Files::directoryExists(jasterix_definition_path))
    {
        auto jasterix = std::make_shared<jASTERIX::jASTERIX>(jasterix_definition_path, false, false, true);

        for (auto& cat_it : jasterix->categories())
        {
            unsigned int category = cat_it.first;
            const auto& cat = cat_it.second;

            ctx.asterixDecoding().emplace_back(
                category, cat->defaultEdition(), cat->defaultREFEdition(), cat->defaultSPFEdition());
        }

        loginf << "populated " << ctx.asterixDecoding().size() << " default ASTERIX decoding configs";
    }

    DBContextSerializer::save(ctx, basePath());

    contexts_[name] = std::move(ctx);

    loginf << "created context '" << name << "'";

    emit contextsChangedSignal();
}

void DBContextManager::deleteContext(const string& name)
{
    traced_assert(hasContext(name));

    try
    {
        DBContextSerializer::deleteContext(basePath(), name);
    }
    catch(const std::exception& e)
    {
        string msg = string("DBContextManager: deleteContext: failed to delete context '") + name
                     + "': " + e.what();
        traced_assert_msg(false, msg.c_str());
    }
    
    contexts_.erase(name);

    if (active_context_name_ == name)
    {
        invalidateDataSourceCache();
        active_context_name_.clear();
        saveActiveContextName();
        emit activeContextChangedSignal();
    }

    loginf << "deleted context '" << name << "'";

    emit contextsChangedSignal();
}

void DBContextManager::renameContext(const string& old_name, const string& new_name)
{
    traced_assert(!new_name.empty());
    traced_assert(hasContext(old_name));
    traced_assert(!hasContext(new_name));

    DBContextSerializer::renameContext(basePath(), old_name, new_name);

    auto ctx = std::move(contexts_.at(old_name));
    contexts_.erase(old_name);
    ctx.name(new_name);
    contexts_[new_name] = std::move(ctx);

    if (active_context_name_ == old_name)
    {
        invalidateDataSourceCache();
        active_context_name_ = new_name;
        saveActiveContextName();
        emit activeContextChangedSignal();
    }

    loginf << "renamed context '" << old_name << "' to '" << new_name << "'";

    emit contextsChangedSignal();
}

void DBContextManager::duplicateContext(const string& src, const string& dest)
{
    traced_assert(!dest.empty());
    traced_assert(hasContext(src));
    traced_assert(!hasContext(dest));

    DBContext copy = contexts_.at(src);
    copy.name(dest);
    copy.created(DBContext::currentTimestamp());
    copy.modified(copy.created());

    DBContextSerializer::save(copy, basePath());
    contexts_[dest] = std::move(copy);

    loginf << "duplicated context '" << src << "' as '" << dest << "'";

    emit contextsChangedSignal();
}

void DBContextManager::saveContext(const string& name)
{
    traced_assert(hasContext(name));

    invalidateDataSourceCache();

    auto& ctx = contexts_.at(name);
    ctx.modified(DBContext::currentTimestamp());

    // saveContext persists an active context that the caller has already mutated in memory, so a
    // failure here leaves in-memory state diverged from disk (and the DB) with no way for the
    // caller to repair it. treat it as a fatal invariant violation rather than a recoverable,
    // divergence-hiding exception.
    try
    {
        DBContextSerializer::save(ctx, basePath());

        if (compass_.dbOpened())
            writeContextToDB();
    }
    catch (const std::exception& e)
    {
        string msg = string("DBContextManager: saveContext: failed to persist context '") + name
                     + "': " + e.what();
        traced_assert_msg(false, msg.c_str());
    }

    loginf << "saved context '" << name << "'";
}

// ============================================================
// Active context
// ============================================================

bool DBContextManager::hasActiveContext() const
{
    return !active_context_name_.empty() && hasContext(active_context_name_);
}

string DBContextManager::activeContextName() const
{
    return active_context_name_;
}

const DBContext& DBContextManager::activeContext() const
{
    if (!hasActiveContext())
    {
        static const DBContext empty;
        return empty;
    }
    return contexts_.at(active_context_name_);
}

DBContext& DBContextManager::activeContext()
{
    if (!hasActiveContext())
    {
        static DBContext empty;
        empty = DBContext{}; // reset each time so no stale state accumulates
        return empty;
    }
    return contexts_.at(active_context_name_);
}

void DBContextManager::setActiveContext(const string& name)
{
    traced_assert(!name.empty());
    traced_assert(hasContext(name));

    if (active_context_name_ == name)
        return;

    invalidateDataSourceCache();

    // switching context while a DB with imported data is open is not allowed -
    // callers (GUI, RT commands) must prevent this
    traced_assert(!compass_.dbOpened() || !hasInsertedData());

    active_context_name_ = name;
    saveActiveContextName();

    loginf << "active context set to '" << name << "', rebuilding sector layers";

    rebuildSectorLayers();

    if (compass_.dbOpened())
        writeContextToDB();

    loginf << "sector layers rebuilt, sectors_loaded_=" << sectors_loaded_;

    emit activeContextChangedSignal();
}

// ============================================================
// Data source query/lookup
// ============================================================

bool DBContextManager::hasDataSource(unsigned int ds_id) const
{
    if (!hasActiveContext()) return false;
    return activeContext().dataSources().find(ds_id) != activeContext().dataSources().end();
}

const DataSource* DBContextManager::dataSource(unsigned int ds_id) const
{
    if (!hasActiveContext()) return nullptr;
    const auto& sources = activeContext().dataSources();
    auto it = sources.find(ds_id);
    return it == sources.end() ? nullptr : &it->second;
}

DataSource* DBContextManager::dataSource(unsigned int ds_id)
{
    if (!hasActiveContext()) return nullptr;
    auto& sources = activeContext().dataSources();
    auto it = sources.find(ds_id);
    return it == sources.end() ? nullptr : &it->second;
}

std::string DBContextManager::remoteUnitName(unsigned int ds_id, int ru_idx) const
{
    if (!hasActiveContext()) return std::to_string(ru_idx);
    ensureDataSourceCache();
    auto ds_it = ru_name_cache_.find(ds_id);
    if (ds_it == ru_name_cache_.end()) return std::to_string(ru_idx);
    auto ru_it = ds_it->second.find(ru_idx);
    return ru_it == ds_it->second.end() ? std::to_string(ru_idx) : ru_it->second;
}

void DBContextManager::ensureDataSourceCache() const
{
    if (ds_cache_valid_) return;

    ru_name_cache_.clear();

    if (hasActiveContext())
    {
        // ds_id lookup is served directly by activeContext().dataSources() (a
        // std::map) - no by-id cache needed. Only the RU-name cache is built
        // here, since it requires JSON parsing per-DS.
        for (const auto& [ds_id, ds] : activeContext().dataSources())
        {
            if (!ds.hasRemoteUnits()) continue;
            const auto& ru_obj = ds.info().at("remote_units");
            if (!ru_obj.is_object()) continue;
            auto& by_idx = ru_name_cache_[ds_id];
            for (auto it = ru_obj.begin(); it != ru_obj.end(); ++it)
            {
                int idx;
                try { idx = std::stoi(it.key()); } catch (...) { continue; }
                if (it.value().is_object() && it.value().contains("name") && it.value().at("name").is_string())
                    by_idx[idx] = it.value().at("name").get<std::string>();
            }
        }
    }

    ds_cache_valid_ = true;
}

void DBContextManager::invalidateDataSourceCache() const
{
    ds_cache_valid_ = false;
    ru_name_cache_.clear();
}

bool DBContextManager::hasDataSource(const string& name) const
{
    if (!hasActiveContext()) return false;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        if (ds.name() == name) return true;
    return false;
}

unsigned int DBContextManager::getDataSourceId(const string& name) const
{
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        if (ds.name() == name) return ds.id();
    traced_assert(false); // not found
    return 0;
}

vector<unsigned int> DBContextManager::allDataSourceIds() const
{
    vector<unsigned int> ids;
    if (!hasActiveContext()) return ids;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        ids.push_back(ds.id());
    return ids;
}

vector<IDataSourceProvider::DataSourceInfo> DBContextManager::dataSourceInfos() const
{
    vector<IDataSourceProvider::DataSourceInfo> infos;
    if (!hasActiveContext()) return infos;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        infos.push_back({ds.id(), ds.name(), ds.dsType()});
    return infos;
}

set<unsigned int> DBContextManager::groundOnlyDataSources() const
{
    set<unsigned int> result;
    if (!hasActiveContext()) return result;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
    {
        if (ds.info().contains("ground_only") && ds.info().at("ground_only") == true)
            result.insert(ds.id());
    }
    return result;
}

map<unsigned int, string> DBContextManager::dsTypes() const
{
    map<unsigned int, string> result;
    if (!hasActiveContext()) return result;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        result[ds.id()] = ds.dsType();
    return result;
}

DataSource& DBContextManager::createDataSource(unsigned int sac, unsigned int sic,
                                                const std::string& name,
                                                const std::string& ds_type)
{
    traced_assert(hasActiveContext());

    DataSource ds;
    ds.sac(sac);
    ds.sic(sic);
    ds.dsType(ds_type);
    ds.name(name.empty() ? "New " + to_string(sac) + "/" + to_string(sic) : name);

    autoAssignColors(ds);

    unsigned int new_id = ds.id();
    auto [it, inserted] = activeContext().dataSources().emplace(new_id, std::move(ds));
    traced_assert(inserted);
    saveContext(active_context_name_);

    if (compass_.dbOpened())
        writeContextToDB();

    emit dataSourcesChangedSignal();

    return it->second;
}

void DBContextManager::autoAssignColors(DataSource& ds) const
{
    if (ds.baseColor().isValid())
        return;

    ColorProvider::Band band = ColorProvider::Band::Light;

    std::vector<QColor> existing; // only same-DSType neighbours - cross-type
                                  // spacing is handled by the per-DSType hue range.
    if (hasActiveContext())
    {
        if (activeContext().colors().preference == ContextColors::Preference::Dark)
            band = ColorProvider::Band::Dark;

        for (const auto& [other_id, other] : activeContext().dataSources())
        {
            if (other_id == ds.id())
                continue;
            if (other.dsType() != ds.dsType())
                continue;
            if (other.baseColor().isValid())
                existing.push_back(other.baseColor());
        }
    }

    ds.baseColor(ColorProvider::generateBaseColor(existing, band, ds.dsType()));
}

void DBContextManager::deleteDataSource(unsigned int ds_id)
{
    traced_assert(hasActiveContext());

    activeContext().dataSources().erase(ds_id);

    saveContext(active_context_name_);
    emit dataSourcesChangedSignal();
}

// ============================================================
// Data source loading/filtering
// ============================================================

bool DBContextManager::dsTypeLoadingWanted(const string& ds_type) const
{
    auto it = ds_type_loading_wanted_.find(ds_type);
    return it == ds_type_loading_wanted_.end() || it->second; // default true
}

void DBContextManager::dsTypeLoadingWanted(const string& ds_type, bool wanted)
{
    ds_type_loading_wanted_[ds_type] = wanted;
}

bool DBContextManager::dsTypeFiltered() const
{
    for (const auto& [type, wanted] : ds_type_loading_wanted_)
        if (!wanted) return true;
    return false;
}

void DBContextManager::setLoadDSTypes(bool loading_wanted)
{
    ds_type_loading_wanted_.clear();
    if (!loading_wanted)
    {
        // set all known types to false
        if (hasActiveContext())
        {
            for (const auto& [ds_id, ds] : activeContext().dataSources())
                ds_type_loading_wanted_[ds.dsType()] = false;
        }
    }
}

void DBContextManager::setLoadOnlyDSTypes(set<string> ds_types)
{
    if (!hasActiveContext()) return;

    // set all to false, then enable only specified
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        ds_type_loading_wanted_[ds.dsType()] = false;

    for (const auto& t : ds_types)
        ds_type_loading_wanted_[t] = true;
}

bool DBContextManager::loadingWanted(unsigned int ds_id) const
{
    auto it = ds_loading_wanted_.find(ds_id);
    return it == ds_loading_wanted_.end() || it->second; // default true
}

void DBContextManager::loadingWanted(unsigned int ds_id, bool wanted)
{
    ds_loading_wanted_[ds_id] = wanted;
}

bool DBContextManager::loadingWanted(const string& dbcontent_name) const
{
    if (!hasActiveContext()) return false;

    for (const auto& [ds_id, ds] : activeContext().dataSources())
    {
        if (dsTypeLoadingWanted(ds.dsType()) && loadingWanted(ds.id()))
        {
            // check if this ds has data for the given dbcontent
            // for now, return true if any ds is wanted (full check needs DB info)
            return true;
        }
    }
    return false;
}

bool DBContextManager::hasDSFilter(const string& /*dbcontent_name*/) const
{
    return dsTypeFiltered() || !ds_loading_wanted_.empty();
}

bool DBContextManager::hasDataSourcesOfDBContent(const string& /*dbcontent_name*/) const
{
    // requires DB-side counts info; return true if any data sources exist
    return hasActiveContext() && !activeContext().dataSources().empty();
}

void DBContextManager::setLoadDataSources(bool loading_wanted)
{
    ds_loading_wanted_.clear();
    line_loading_wanted_.clear();

    if (!loading_wanted && hasActiveContext())
    {
        for (const auto& [ds_id, ds] : activeContext().dataSources())
            ds_loading_wanted_[ds.id()] = false;
    }
}

void DBContextManager::setLoadOnlyDataSources(map<unsigned int, set<unsigned int>> ds_ids)
{
    // set all to not wanted
    setLoadDataSources(false);

    // enable only specified
    for (const auto& [ds_id, lines] : ds_ids)
    {
        ds_loading_wanted_[ds_id] = true;

        // Also enable the DSType of this DS - otherwise an inconsistent
        // viewpoint (lists this DS but not its DSType) or a stale per-DSType
        // filter would silently override the explicit per-DS request and
        // skip loading data for DBContents whose DS lives in another DSType.
        if (auto* ds = dataSource(ds_id))
            ds_type_loading_wanted_[ds->dsType()] = true;

        if (!lines.empty())
        {
            for (const auto& line : lines)
                line_loading_wanted_[ds_id][line] = true;
        }
    }
}

void DBContextManager::setLoadAllDataSourceLines()
{
    line_loading_wanted_.clear();
}

map<unsigned int, set<unsigned int>> DBContextManager::getLoadDataSources() const
{
    map<unsigned int, set<unsigned int>> result;

    if (!hasActiveContext()) return result;

    for (const auto& [ds_id, ds] : activeContext().dataSources())
    {
        if (!dsTypeLoadingWanted(ds.dsType()))
            continue;
        if (!loadingWanted(ds.id()))
            continue;

        set<unsigned int> wanted_lines;
        auto line_it = line_loading_wanted_.find(ds.id());
        if (line_it != line_loading_wanted_.end())
        {
            for (const auto& [line_id, wanted] : line_it->second)
                if (wanted) wanted_lines.insert(line_id);
        }
        // empty set = all lines wanted

        result[ds.id()] = wanted_lines;
    }

    return result;
}

bool DBContextManager::loadDataSourcesFiltered() const
{
    if (!hasActiveContext()) return false;

    for (const auto& [ds_id, ds] : activeContext().dataSources())
        if (!loadingWanted(ds.id()))
            return true;

    return false;
}

set<string> DBContextManager::wantedDSTypes() const
{
    set<string> ret;

    if (!hasActiveContext()) return ret;

    // collect all known ds types
    set<string> all_types;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        all_types.insert(ds.dsType());

    for (const auto& t : all_types)
    {
        auto it = ds_type_loading_wanted_.find(t);
        if (it == ds_type_loading_wanted_.end() || it->second)
            ret.insert(t);
    }

    return ret;
}

bool DBContextManager::lineSpecificLoadingRequired(const string& /*dbcontent_name*/) const
{
    return !line_loading_wanted_.empty();
}

bool DBContextManager::lineLoadingWanted(unsigned int ds_id, unsigned int line_id) const
{
    auto ds_it = line_loading_wanted_.find(ds_id);
    if (ds_it == line_loading_wanted_.end())
        return true; // no line filter = all wanted

    auto line_it = ds_it->second.find(line_id);
    return line_it == ds_it->second.end() || line_it->second;
}

void DBContextManager::lineLoadingWanted(unsigned int ds_id, unsigned int line_id, bool wanted)
{
    line_loading_wanted_[ds_id][line_id] = wanted;
}

void DBContextManager::selectAllDSTypes()
{
    ds_type_loading_wanted_.clear(); // all default to true
}

void DBContextManager::deselectAllDSTypes()
{
    setLoadDSTypes(false);
}

void DBContextManager::selectAllDataSources()
{
    ds_loading_wanted_.clear(); // all default to true
    line_loading_wanted_.clear();
}

void DBContextManager::deselectAllDataSources()
{
    setLoadDataSources(false);
}

void DBContextManager::deselectAllLines()
{
    if (!hasActiveContext()) return;

    for (const auto& [ds_id, ds] : activeContext().dataSources())
    {
        // get network lines from info
        if (ds.info().contains("network_lines"))
        {
            for (auto it = ds.info().at("network_lines").begin();
                 it != ds.info().at("network_lines").end(); ++it)
            {
                // line IDs are typically 0,1,2,3 mapped from L1,L2,L3,L4
                // for now, mark all as not wanted
            }
        }
    }
    // simplified: clear all line state (callers will re-select what they want)
    line_loading_wanted_.clear();
}

void DBContextManager::selectSpecificLine(unsigned int line_id)
{
    if (!hasActiveContext()) return;

    for (const auto& [ds_id, ds] : activeContext().dataSources())
        line_loading_wanted_[ds.id()][line_id] = true;
}

void DBContextManager::selectDSTypeSpecificDataSources(const string& ds_type)
{
    if (!hasActiveContext()) return;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        if (ds.dsType() == ds_type)
            ds_loading_wanted_[ds.id()] = true;
}

void DBContextManager::deselectDSTypeSpecificDataSources(const string& ds_type)
{
    if (!hasActiveContext()) return;
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        if (ds.dsType() == ds_type)
            ds_loading_wanted_[ds.id()] = false;
}

vector<unsigned int> DBContextManager::unfilteredDS(const string& /*dbcontent_name*/) const
{
    set<unsigned int> result;

    // context-resident ds_ids that pass dstype + per-DS wanted checks
    // (preserves "no entry in ds_loading_wanted_ = default wanted" semantics)
    if (hasActiveContext())
    {
        for (const auto& [ds_id, ds] : activeContext().dataSources())
        {
            if (dsTypeLoadingWanted(ds.dsType()) && loadingWanted(ds.id()))
                result.insert(ds.id());
        }
    }

    // explicitly-wanted ds_ids that may not yet be registered in the context
    // (e.g. ref/tst sources supplied via the evaluate command's config)
    for (const auto& [ds_id, wanted] : ds_loading_wanted_)
    {
        if (!wanted) continue;

        if (dsTypeFiltered())
        {
            if (const DataSource* ds = dataSource(ds_id))
                if (!dsTypeLoadingWanted(ds->dsType()))
                    continue;
        }

        result.insert(ds_id);
    }

    return vector<unsigned int>(result.begin(), result.end());
}

// ============================================================
// Runtime counts
// ============================================================

void DBContextManager::setLoadedCounts(
    map<unsigned int, map<string, map<unsigned int, unsigned int>>> loaded_counts)
{
    loaded_counts_ = std::move(loaded_counts);
    emit countsChangedSignal();
}

void DBContextManager::clearInsertedCounts(const string& dbcontent_name)
{
    for (auto& [ds_id, dbcont_map] : inserted_counts_)
        dbcont_map.erase(dbcontent_name);

    emit countsChangedSignal();
}

void DBContextManager::clearInsertedCounts(unsigned int ds_id, const string& dbcontent_name,
                                           const vector<unsigned int>& line_ids)
{
    auto ds_it = inserted_counts_.find(ds_id);
    if (ds_it == inserted_counts_.end()) return;

    auto dbc_it = ds_it->second.find(dbcontent_name);
    if (dbc_it == ds_it->second.end()) return;

    if (line_ids.empty())
        ds_it->second.erase(dbc_it);
    else
    {
        for (auto lid : line_ids)
            dbc_it->second.erase(lid);
    }

    emit countsChangedSignal();
}

void DBContextManager::applyDeleteInfo(const json& delete_info)
{
    auto clearDbcontent = [&] (unsigned int ds_id, const string& dbcontent_name)
    {
        auto ds_it = inserted_counts_.find(ds_id);
        if (ds_it == inserted_counts_.end())
            return;

        if (dbcontent_name.empty())
        {
            loginf << "clearing all counts for ds_id " << ds_id;
            ds_it->second.clear();
            return;
        }

        auto dbc_it = ds_it->second.find(dbcontent_name);
        if (dbc_it == ds_it->second.end())
            return;

        loginf << "clearing counts for ds_id " << ds_id
               << " dbcontent '" << dbcontent_name << "'";
        ds_it->second.erase(dbc_it);
    };

    auto clearLine = [&] (unsigned int ds_id, const string& dbcontent_name, unsigned int line_id)
    {
        auto ds_it = inserted_counts_.find(ds_id);
        if (ds_it == inserted_counts_.end())
            return;

        if (dbcontent_name.empty())
        {
            loginf << "clearing counts for ds_id " << ds_id << " line " << line_id << " (all dbcontents)";
            for (auto& [dbc_name, line_map] : ds_it->second)
                line_map.erase(line_id);
            return;
        }

        auto dbc_it = ds_it->second.find(dbcontent_name);
        if (dbc_it == ds_it->second.end())
            return;

        loginf << "clearing counts for ds_id " << ds_id
               << " dbcontent '" << dbcontent_name << "'"
               << " line " << line_id;
        dbc_it->second.erase(line_id);
    };

    for (const auto& entry : delete_info)
    {
        string dbcontent_name = entry.contains("dbcontent")
                ? entry.at("dbcontent").get<string>()
                : string{};

        if (!entry.contains("data_sources"))
        {
            // no data_sources key → all data (for the given dbcontent, or all) was deleted
            if (dbcontent_name.empty())
            {
                loginf << "clearing all counts";
                inserted_counts_.clear();
            }
            else
            {
                loginf << "clearing all counts for dbcontent '" << dbcontent_name << "'";
                for (auto& [ds_id, dbcont_map] : inserted_counts_)
                    dbcont_map.erase(dbcontent_name);
            }
        }
        else
        {
            for (const auto& ds_entry : entry.at("data_sources"))
            {
                unsigned int ds_id = ds_entry.at("ds_id").get<unsigned int>();

                if (!ds_entry.contains("line_ids"))
                {
                    clearDbcontent(ds_id, dbcontent_name);
                }
                else
                {
                    for (const auto& lid : ds_entry.at("line_ids"))
                        clearLine(ds_id, dbcontent_name, lid.get<unsigned int>());
                }
            }
        }
    }

    emit countsChangedSignal();
}

void DBContextManager::addNumInserted(unsigned int ds_id, const string& dbcontent_name,
                                      unsigned int line_id, unsigned int count)
{
    inserted_counts_[ds_id][dbcontent_name][line_id] += count;

    logdbg << "ds_id " << ds_id << " dbc " << dbcontent_name
           << " line " << line_id << " cnt +" << count
           << " total " << inserted_counts_[ds_id][dbcontent_name][line_id];
}

void DBContextManager::maxTimestamp(unsigned int ds_id, unsigned int line_id,
                                    boost::posix_time::ptime value)
{
    auto& ts = max_timestamps_[ds_id][line_id];
    if (ts.is_not_a_date_time() || value > ts)
        ts = value;
}

boost::posix_time::ptime DBContextManager::maxTimestamp(unsigned int ds_id, unsigned int line_id) const
{
    auto ds_it = max_timestamps_.find(ds_id);
    if (ds_it == max_timestamps_.end()) return {};

    auto line_it = ds_it->second.find(line_id);
    if (line_it == ds_it->second.end()) return {};

    return line_it->second;
}

bool DBContextManager::hasNumInserted(unsigned int ds_id) const
{
    auto ds_it = inserted_counts_.find(ds_id);
    if (ds_it == inserted_counts_.end()) return false;

    for (const auto& [dbc, lines] : ds_it->second)
        for (const auto& [line, cnt] : lines)
            if (cnt > 0) return true;

    return false;
}

map<unsigned int, unsigned int> DBContextManager::numInsertedLinesMap(unsigned int ds_id) const
{
    map<unsigned int, unsigned int> line_cnts;

    auto ds_it = inserted_counts_.find(ds_id);
    if (ds_it == inserted_counts_.end()) return line_cnts;

    for (const auto& [dbc, lines] : ds_it->second)
        for (const auto& [line, cnt] : lines)
            line_cnts[line] += cnt;

    return line_cnts;
}

unsigned int DBContextManager::numInserted(unsigned int ds_id, const string& dbcontent_name) const
{
    auto ds_it = inserted_counts_.find(ds_id);
    if (ds_it == inserted_counts_.end()) return 0;

    auto dbc_it = ds_it->second.find(dbcontent_name);
    if (dbc_it == ds_it->second.end()) return 0;

    unsigned int total = 0;
    for (const auto& [line, cnt] : dbc_it->second)
        total += cnt;
    return total;
}

unsigned int DBContextManager::numLoaded(unsigned int ds_id, const string& dbcontent_name) const
{
    auto ds_it = loaded_counts_.find(ds_id);
    if (ds_it == loaded_counts_.end()) return 0;

    auto dbc_it = ds_it->second.find(dbcontent_name);
    if (dbc_it == ds_it->second.end()) return 0;

    unsigned int total = 0;
    for (const auto& [line, cnt] : dbc_it->second)
        total += cnt;
    return total;
}

map<unsigned int, unsigned int> DBContextManager::numInsertedPerLine(
    unsigned int ds_id, const string& dbcontent_name) const
{
    auto ds_it = inserted_counts_.find(ds_id);
    if (ds_it == inserted_counts_.end()) return {};

    auto dbc_it = ds_it->second.find(dbcontent_name);
    if (dbc_it == ds_it->second.end()) return {};

    return dbc_it->second;
}

// ============================================================
// Count persistence
// ============================================================

static const string DBInfoKeyInsertedCounts = "inserted_counts";

void DBContextManager::saveCountsToDB()
{
    assert (compass_.dbOpened());
    loginf << "saving " << inserted_counts_.size() << " data sources";

    // serialize: { "ds_id": { "dbcontent": { "line_id": count } } }
    json j;
    for (const auto& [ds_id, dbc_map] : inserted_counts_)
        for (const auto& [dbc, line_map] : dbc_map)
            for (const auto& [line_id, cnt] : line_map)
                j[to_string(ds_id)][dbc][to_string(line_id)] = cnt;

    loginf << "json: " << j.dump().substr(0, 500);

    compass_.dbInterface().saveDBInfo(DBInfoKeyInsertedCounts, j.dump());

    loginf << "saved inserted counts to db_info";
}

void DBContextManager::loadCountsFromDB()
{
    inserted_counts_.clear();

    if (!compass_.dbOpened())
    {
        loginf << "db not opened, skipping";
        return;
    }

    string json_str = compass_.dbInterface().loadDBInfo(DBInfoKeyInsertedCounts);

    if (json_str.empty())
    {
        loginf << "no inserted_counts in db_info";
        return;
    }

    loginf << "raw json: " << json_str.substr(0, 500);

    json j = json::parse(json_str);

    for (auto& [ds_id_str, dbc_obj] : j.items())
    {
        unsigned int ds_id = stoul(ds_id_str);

        for (auto& [dbc, line_obj] : dbc_obj.items())
            for (auto& [line_str, cnt_val] : line_obj.items())
                inserted_counts_[ds_id][dbc][stoul(line_str)] = cnt_val.get<unsigned int>();
    }

    loginf << "loaded inserted counts from db_info (" << inserted_counts_.size() << " data sources)";

    for (const auto& [ds_id, dbc_map] : inserted_counts_)
        for (const auto& [dbc, line_map] : dbc_map)
            for (const auto& [line_id, cnt] : line_map)
                loginf << "  ds_id " << ds_id << " dbc " << dbc << " line " << line_id << " cnt " << cnt;
}

bool DBContextManager::hasInsertedData() const
{
    for (const auto& [ds_id, dbc_map] : inserted_counts_)
        for (const auto& [dbc, line_map] : dbc_map)
            for (const auto& [line_id, cnt] : line_map)
                if (cnt > 0)
                    return true;

    return false;
}

// ============================================================
// ASTERIX info (cumulative per-DS / per-CAT / per-item probe stats)
// ============================================================

namespace
{
    // The probe emits min/max as raw JSON values in the item's native type
    // (number / string). For min/max merging only matching numeric or string
    // values are meaningful; otherwise the existing value wins.
    bool jsonLess(const nlohmann::json& a, const nlohmann::json& b)
    {
        if (a.is_number() && b.is_number())
            return a.get<double>() < b.get<double>();
        if (a.is_string() && b.is_string())
            return a.get<std::string>() < b.get<std::string>();
        return false;
    }

    void mergeMin(nlohmann::json& dst, const nlohmann::json& src)
    {
        if (src.is_null())
            return;
        if (dst.is_null() || jsonLess(src, dst))
            dst = src;
    }

    void mergeMax(nlohmann::json& dst, const nlohmann::json& src)
    {
        if (src.is_null())
            return;
        if (dst.is_null() || jsonLess(dst, src))
            dst = src;
    }

    std::size_t readSizeT(const nlohmann::json& v)
    {
        if (v.is_number_unsigned())
            return v.get<std::size_t>();
        if (v.is_number_integer())
            return static_cast<std::size_t>(v.get<long long>());
        return 0;
    }
}

static const string DBInfoKeyAsterixInfo = "asterix_info";

bool DBContextManager::hasAsterixInfo(unsigned int ds_id) const
{
    return asterix_info_.count(ds_id) > 0;
}

bool DBContextManager::hasAnyAsterixInfo() const
{
    return !asterix_info_.empty();
}

void DBContextManager::mergeAsterixInfoInto(AsterixInfoMap& dst,
                                            const AsterixInfoMap& delta)
{
    for (const auto& [ds_id, cat_map] : delta)
    {
        if (ds_id == 0) // unknown SAC/SIC bucket - skip
            continue;

        for (const auto& [cat, cat_stats] : cat_map)
        {
            auto& dst_cat = dst[ds_id][cat];
            dst_cat.total_count += cat_stats.total_count;

            for (const auto& [item_name, item_stats] : cat_stats.items)
            {
                auto& dst_item = dst_cat.items[item_name];
                dst_item.count += item_stats.count;
                mergeMin(dst_item.min, item_stats.min);
                mergeMax(dst_item.max, item_stats.max);
            }
        }
    }
}

void DBContextManager::mergeAsterixInfo(const AsterixInfoMap& delta)
{
    const std::size_t before = asterix_info_.size();
    mergeAsterixInfoInto(asterix_info_, delta);

    // Only emit when delta actually contained applicable entries (skipping
    // ds_id == 0). If size didn't grow and delta had no usable rows we still
    // emit defensively when there were entries beyond the unknown bucket.
    bool delta_had_entries = false;
    for (const auto& [ds_id, _] : delta)
        if (ds_id != 0) { delta_had_entries = true; break; }

    if (delta_had_entries || before != asterix_info_.size())
        emit asterixInfoChangedSignal();
}

nlohmann::json DBContextManager::asterixInfoToJSON(const AsterixInfoMap& src)
{
    json j = json::object();

    for (const auto& [ds_id, cat_map] : src)
    {
        json& ds_node = j[std::to_string(ds_id)];

        for (const auto& [cat, cat_stats] : cat_map)
        {
            json& cat_node = ds_node[std::to_string(cat)];
            cat_node["total_count"] = cat_stats.total_count;

            json items_node = json::object();
            for (const auto& [item_name, item_stats] : cat_stats.items)
            {
                json item_node = json::object();
                item_node["count"] = item_stats.count;
                if (!item_stats.min.is_null())
                    item_node["min"] = item_stats.min;
                if (!item_stats.max.is_null())
                    item_node["max"] = item_stats.max;
                items_node[item_name] = std::move(item_node);
            }
            cat_node["items"] = std::move(items_node);
        }
    }

    return j;
}

DBContextManager::AsterixInfoMap
DBContextManager::asterixInfoFromJSON(const nlohmann::json& j)
{
    AsterixInfoMap result;

    if (!j.is_object())
        return result;

    for (auto& [ds_id_str, cat_obj] : j.items())
    {
        unsigned int ds_id = 0;
        try { ds_id = std::stoul(ds_id_str); } catch (...) { continue; }
        if (!cat_obj.is_object())
            continue;

        auto& dst_ds = result[ds_id];

        for (auto& [cat_str, cat_node] : cat_obj.items())
        {
            unsigned int cat = 0;
            try { cat = std::stoul(cat_str); } catch (...) { continue; }
            if (!cat_node.is_object())
                continue;

            auto& dst_cat = dst_ds[cat];

            if (cat_node.contains("total_count"))
                dst_cat.total_count = readSizeT(cat_node.at("total_count"));

            if (cat_node.contains("items") && cat_node.at("items").is_object())
            {
                for (auto& [item_name, item_node] : cat_node.at("items").items())
                {
                    if (!item_node.is_object())
                        continue;

                    auto& st = dst_cat.items[item_name];
                    if (item_node.contains("count"))
                        st.count = readSizeT(item_node.at("count"));
                    if (item_node.contains("min"))
                        st.min = item_node.at("min");
                    if (item_node.contains("max"))
                        st.max = item_node.at("max");
                }
            }
        }
    }

    return result;
}

void DBContextManager::saveAsterixInfoToDB()
{
    assert (compass_.dbOpened());
    loginf << "saving " << asterix_info_.size() << " data sources";

    json j = asterixInfoToJSON(asterix_info_);

    compass_.dbInterface().saveDBInfo(DBInfoKeyAsterixInfo, j.dump());

    loginf << "saved asterix_info to db_info";
}

void DBContextManager::loadAsterixInfoFromDB()
{
    asterix_info_.clear();

    if (!compass_.dbOpened())
    {
        loginf << "db not opened, skipping";
        return;
    }

    string json_str = compass_.dbInterface().loadDBInfo(DBInfoKeyAsterixInfo);

    if (json_str.empty())
    {
        loginf << "no asterix_info in db_info";
        return;
    }

    json j;
    try
    {
        j = json::parse(json_str);
    }
    catch (const std::exception& ex)
    {
        logerr << "failed to parse asterix_info: " << ex.what();
        return;
    }

    asterix_info_ = asterixInfoFromJSON(j);

    loginf << "loaded asterix_info from db_info (" << asterix_info_.size() << " data sources)";
}

// ============================================================
// Network lines
// ============================================================

map<unsigned int, map<string, json>> DBContextManager::getNetworkLines() const
{
    map<unsigned int, map<string, json>> result;

    if (!hasActiveContext()) return result;

    for (const auto& [ds_id, ds] : activeContext().dataSources())
    {
        if (ds.info().contains("network_lines"))
            result[ds.id()] = ds.info().at("network_lines").get<map<string, json>>();
    }

    return result;
}

void DBContextManager::createNetworkDBDataSources()
{
    // TODO: implement when wiring network import - create data sources from network config
    loginf << "createNetworkDBDataSources not yet implemented";
}

// ============================================================
// FFT query/lookup
// ============================================================

bool DBContextManager::hasFFT(const string& name) const
{
    if (!hasActiveContext()) return false;
    for (const auto& f : activeContext().ffts())
        if (f.name() == name) return true;
    return false;
}

const FFT* DBContextManager::fft(const string& name) const
{
    if (!hasActiveContext()) return nullptr;
    for (const auto& f : activeContext().ffts())
        if (f.name() == name) return &f;
    return nullptr;
}

FFT* DBContextManager::fft(const string& name)
{
    if (!hasActiveContext()) return nullptr;
    for (auto& f : activeContext().ffts())
        if (f.name() == name) return &f;
    return nullptr;
}

vector<string> DBContextManager::allFFTNames() const
{
    vector<string> names;
    if (!hasActiveContext()) return names;
    for (const auto& f : activeContext().ffts())
        names.push_back(f.name());
    return names;
}

FFT& DBContextManager::createFFT(const string& name)
{
    traced_assert(hasActiveContext());
    traced_assert(!hasFFT(name));

    FFT f;
    f.name(name);
    activeContext().ffts().push_back(std::move(f));
    saveContext(active_context_name_);

    emit fftsChangedSignal();

    return activeContext().ffts().back();
}

void DBContextManager::deleteFFT(const string& name)
{
    traced_assert(hasActiveContext());

    auto& ffts = activeContext().ffts();
    ffts.erase(
        remove_if(ffts.begin(), ffts.end(),
                  [&name](const FFT& f) { return f.name() == name; }),
        ffts.end());

    saveContext(active_context_name_);
    emit fftsChangedSignal();
}

void DBContextManager::deleteAllFFTs()
{
    traced_assert(hasActiveContext());
    activeContext().ffts().clear();
    saveContext(active_context_name_);
    emit fftsChangedSignal();
}

pair<bool, float> DBContextManager::isFromFFT(double latitude_deg, double longitude_deg,
                                              boost::optional<unsigned int> mode_s_address,
                                              bool ignore_mode_s,
                                              boost::optional<unsigned int> mode_a_code,
                                              boost::optional<float> mode_c_code) const
{
    if (!hasActiveContext())
        return {false, 0};

    // special case: mode 3/A code 7777 (4095 octal) always treated as FFT
    if (mode_a_code && *mode_a_code == 4095)
        return {true, 0};

    for (const auto& f : activeContext().ffts())
    {
        bool match = true;

        // mode S address check
        if (!ignore_mode_s && f.info().contains("mode_s_address") && mode_s_address)
        {
            if (f.info().at("mode_s_address").get<unsigned int>() != *mode_s_address)
                match = false;
        }

        // mode 3/A check
        if (match && f.info().contains("mode_3a_code") && mode_a_code)
        {
            if (f.info().at("mode_3a_code").get<unsigned int>() != *mode_a_code)
                match = false;
        }

        // mode C check
        if (match && f.info().contains("mode_c_code") && mode_c_code)
        {
            if (f.info().at("mode_c_code").get<float>() != *mode_c_code)
                match = false;
        }

        // position check
        if (match && f.hasPosition())
        {
            // approximate distance check using GeographicLib or simple haversine
            double dlat = latitude_deg - f.latitude();
            double dlon = longitude_deg - f.longitude();

            // rough distance in meters (1 degree ~ 111km lat, cos(lat)*111km lon)
            double cos_lat = cos(latitude_deg * M_PI / 180.0);
            double dist_m = sqrt(dlat * dlat + dlon * dlon * cos_lat * cos_lat) * 111000.0;

            if (dist_m > max_fft_plot_distance_m_)
                match = false;
        }

        if (match)
        {
            float alt = 0;
            if (f.hasAltitude())
                alt = static_cast<float>(f.altitude());
            return {true, alt};
        }
    }

    return {false, 0};
}

// ============================================================
// DB sync
// ============================================================

void DBContextManager::writeContextToDB()
{
    traced_assert(compass_.dbOpened());
    traced_assert(hasActiveContext());

    auto& db = compass_.dbInterface();

    if (!db.existsDBContextTable())
        db.createDBContextTable();
    const auto& ctx = activeContext();

    // save each section
    json meta;
    meta["name"] = ctx.name();
    meta["description"] = ctx.description();
    meta["created"] = ctx.created();
    meta["modified"] = ctx.modified();
    db.saveDBContextSection("meta", meta.dump());

    // sensors
    json sensors;
    sensors["version"] = DBContextSerializer::CURRENT_VERSION;
    json arr = json::array();
    for (const auto& [ds_id, ds] : ctx.dataSources())
        arr.push_back(ds.toJSON());
    sensors["data"] = arr;
    db.saveDBContextSection("sensors", sensors.dump());

    // ffts
    json ffts;
    ffts["version"] = DBContextSerializer::CURRENT_VERSION;
    json fft_arr = json::array();
    for (const auto& fft : ctx.ffts())
        fft_arr.push_back(fft.toJSON());
    ffts["data"] = fft_arr;
    db.saveDBContextSection("ffts", ffts.dump());

    // asterix decoding
    json ast;
    ast["version"] = DBContextSerializer::CURRENT_VERSION;
    json ast_arr = json::array();
    for (const auto& cfg : ctx.asterixDecoding())
        ast_arr.push_back(cfg.toJSON());
    ast["data"] = ast_arr;
    db.saveDBContextSection("asterix_decoding", ast.dump());

    // sectors
    json sec;
    sec["version"] = DBContextSerializer::CURRENT_VERSION;
    json sec_arr = json::array();
    for (const auto& s : ctx.sectors())
        sec_arr.push_back(s->jsonData());
    sec["data"] = sec_arr;
    db.saveDBContextSection("sectors", sec.dump());

    // colors
    json colors;
    colors["version"] = DBContextSerializer::CURRENT_VERSION;
    colors["data"] = ctx.colors().toJSON();
    db.saveDBContextSection("colors", colors.dump());

    loginf << "wrote active context to DB";
}

DBContext DBContextManager::readContextFromDB() const
{
    traced_assert(compass_.dbOpened());

    auto& db = compass_.dbInterface();

    if (!db.existsDBContextTable())
        return {};

    auto sections = db.loadAllDBContextSections();

    DBContext ctx;

    // meta
    if (sections.count("meta"))
    {
        json meta = json::parse(sections.at("meta"));
        ctx.name(meta.value("name", ""));
        ctx.description(meta.value("description", ""));
        ctx.created(meta.value("created", ""));
        ctx.modified(meta.value("modified", ""));
    }

    // colors (read before sensors so the preference is available for legacy DBs
    // lacking per-data-source color fields)
    if (sections.count("colors"))
    {
        json j = json::parse(sections.at("colors"));
        if (j.contains("data"))
            ctx.colors(ContextColors::fromJSON(j.at("data")));
    }

    // sensors
    if (sections.count("sensors"))
    {
        json j = json::parse(sections.at("sensors"));
        if (j.contains("data"))
        {
            ColorProvider::Band band =
                (ctx.colors().preference == ContextColors::Preference::Dark)
                    ? ColorProvider::Band::Dark
                    : ColorProvider::Band::Light;

            set<unsigned int> seen_ids;
            map<std::string, vector<QColor>> existing_by_type;

            for (const auto& ds_j : j.at("data"))
            {
                auto ds = DataSource::fromJSON(ds_j);
                unsigned int ds_id = ds.id();

                if (seen_ids.count(ds_id))
                {
                    logerr << "duplicate data source SAC/SIC "
                           << ds.sac() << "/" << ds.sic()
                           << " in DB context '" << ctx.name() << "', skipping duplicate";
                    continue;
                }

                if (!ds.baseColor().isValid())
                {
                    const auto& same_type = existing_by_type[ds.dsType()];
                    ds.baseColor(ColorProvider::generateBaseColor(same_type, band, ds.dsType()));
                }
                existing_by_type[ds.dsType()].push_back(ds.baseColor());

                seen_ids.insert(ds_id);
                ctx.dataSources().emplace(ds_id, std::move(ds));
            }
        }
    }

    // ffts
    if (sections.count("ffts"))
    {
        json j = json::parse(sections.at("ffts"));
        if (j.contains("data"))
        {
            for (const auto& fft_j : j.at("data"))
                ctx.ffts().push_back(FFT::fromJSON(fft_j));
        }
    }

    // asterix decoding
    if (sections.count("asterix_decoding"))
    {
        json j = json::parse(sections.at("asterix_decoding"));
        if (j.contains("data"))
        {
            for (const auto& cfg_j : j.at("data"))
                ctx.asterixDecoding().push_back(ASTERIXDecodingConfig::fromJSON(cfg_j));
        }
    }

    // sectors
    if (sections.count("sectors"))
    {
        json j = json::parse(sections.at("sectors"));
        if (j.contains("data"))
        {
            for (const auto& sec_j : j.at("data"))
            {
                unsigned int id = sec_j.at("id");
                string name = sec_j.at("name");
                string layer = sec_j.at("layer_name");
                auto sector = make_shared<Sector>(id, name, layer, false);
                sector->readJSON(sec_j);
                ctx.sectors().push_back(sector);
            }
        }
    }

    return ctx;
}

DBContextDiff DBContextManager::diffWithDB() const
{
    traced_assert(compass_.dbOpened());
    traced_assert(hasActiveContext());

    DBContext db_ctx = readContextFromDB();
    return DBContextDiff::compute(activeContext(), db_ctx);
}

DBContextDiff DBContextManager::diff(const DBContext& a, const DBContext& b) const
{
    return DBContextDiff::compute(a, b);
}

// ============================================================
// DB open/close slots
// ============================================================

void DBContextManager::databaseOpenedSlot()
{
    loginf << "database opened";

    invalidateDataSourceCache();

    auto& db = compass_.dbInterface();

    if (!hasActiveContext())
    {
        if (!db.existsDBContextTable())
        {
            // no active context and DB has no stored context (legacy or new empty DB) - stay in "None" state
            logwrn << "no active context and DB has no stored context - remaining in 'None' state";
            return;
        }

        // no active context, but the DB carries one - adopt it silently
        DBContext db_ctx = readContextFromDB();
        const string db_name = db_ctx.name();

        loginf << "no active context - adopting DB context '" << db_name << "'";

        if (!hasContext(db_name))
        {
            db_ctx.modified(DBContext::currentTimestamp());
            DBContextSerializer::save(db_ctx, basePath());
            contexts_[db_name] = std::move(db_ctx);
            emit contextsChangedSignal();
        }

        setActiveContext(db_name);
        loadCountsFromDB();
        loadAsterixInfoFromDB();

        emit countsChangedSignal();
        emit asterixInfoChangedSignal();
        return;
    }

    if (!db.existsDBContextTable())
    {
        // new DB - write the active context
        loginf << "no db_context table - writing active context to DB";
        writeContextToDB();
        loadCountsFromDB();
        loadAsterixInfoFromDB();
    }
    else
    {
        DBContext db_ctx = readContextFromDB();

        if (db_ctx.name() != activeContext().name())
        {
            const string db_name = db_ctx.name();

            // DB was saved with a different context - align to it before loading counts
            loginf << "DB context '" << db_name << "' differs from active '"
                   << activeContext().name() << "' - switching to DB context";

            if (!hasContext(db_name))
            {
                // context doesn't exist on disk - create from DB data
                loginf << "context '" << db_name << "' not found on disk, creating from DB";
                db_ctx.modified(DBContext::currentTimestamp());
                DBContextSerializer::save(db_ctx, basePath());
                contexts_[db_name] = std::move(db_ctx);
            }

            setActiveContext(db_name);
            loadCountsFromDB();
            loadAsterixInfoFromDB();
        }
        else
        {
            // counts needed for conflict dialog (sensors with DB data)
            loadCountsFromDB();
            loadAsterixInfoFromDB();

            auto d = DBContextDiff::compute(activeContext(), db_ctx);

            if (d.hasDifferences())
            {
                logwrn << "context differs from DB:\n" << d.summary();

                QApplication::restoreOverrideCursor();

                // check if any DB-only sensor has data that would be lost
                bool db_has_sensor_data = false;
                for (const auto& sd : d.sensor_diffs)
                {
                    if (sd.type != ItemDiff::Added)
                        continue;
                    if (!sd.item_b.contains("sac") || !sd.item_b.contains("sic"))
                        continue;

                    unsigned int ds_id = Utils::Number::dsIdFrom(
                        sd.item_b.at("sac").get<unsigned int>(),
                        sd.item_b.at("sic").get<unsigned int>());

                    if (inserted_counts_.count(ds_id))
                    {
                        db_has_sensor_data = true;
                        break;
                    }
                }

                DBContextConflictDialog::Resolution resolution;

                if (!compass_.allowUserInteractions())
                {
                    // automated processing (CLI / runtime command): do not open a modal dialog,
                    // keep the configuration context and write it to the database
                    logwrn << "context '" << active_context_name_
                           << "' differs from database during automated processing, using configuration";
                    resolution = DBContextConflictDialog::UseFile;
                }
                else
                {
                    DBContextConflictDialog dlg(active_context_name_, d,
                                               activeContext().modified(), db_ctx.modified(),
                                               db_has_sensor_data,
                                               QApplication::activeWindow());
                    dlg.exec();
                    resolution = dlg.resolution();
                }

                switch (resolution)
                {
                case DBContextConflictDialog::UseFile:
                    loginf << "conflict resolved: using configuration";
                    writeContextToDB();
                    break;

                case DBContextConflictDialog::UseDatabase:
                    loginf << "conflict resolved: using database definition";
                    contexts_[active_context_name_] = db_ctx;
                    invalidateDataSourceCache();
                    DBContextSerializer::save(db_ctx, basePath());
                    rebuildSectorLayers();
                    break;

                case DBContextConflictDialog::Merge:
                {
                    loginf << "conflict resolved: opening merge dialog";

                    std::set<unsigned int> ds_ids_with_data;
                    for (const auto& ds_entry : inserted_counts_)
                        ds_ids_with_data.insert(ds_entry.first);

                    DBContextMergeDialog merge_dlg(activeContext(), db_ctx, d,
                                                   ds_ids_with_data,
                                                   QApplication::activeWindow());
                    merge_dlg.exec();

                    contexts_[active_context_name_] = merge_dlg.mergedContext();
                    saveContext(active_context_name_); // also writes to the DB when one is open
                    rebuildSectorLayers();
                    break;
                }
                }

                QApplication::setOverrideCursor(Qt::WaitCursor);
            }
            else
            {
                loginf << "context matches DB - no action needed";
            }
        }
    }

    loginf << "emitting activeContextChangedSignal and countsChangedSignal";

    emit activeContextChangedSignal();
    emit countsChangedSignal();
    emit asterixInfoChangedSignal();
}

void DBContextManager::databaseClosedSlot()
{
    loginf << "database closed";

    // counts and asterix_info already saved in COMPASS::closeDBInternal() before DB was closed

    inserted_counts_.clear();
    loaded_counts_.clear();
    asterix_info_.clear();

    loginf << "emitting countsChangedSignal";

    emit countsChangedSignal();
    emit asterixInfoChangedSignal();
}

// ============================================================
// Sector layer access
// ============================================================

bool DBContextManager::sectorsLoaded() const
{
    return sectors_loaded_;
}

vector<shared_ptr<SectorLayer>>& DBContextManager::sectorLayers()
{
    traced_assert(sectors_loaded_);
    return sector_layers_;
}

const vector<shared_ptr<SectorLayer>>& DBContextManager::sectorLayers() const
{
    traced_assert(sectors_loaded_);
    return sector_layers_;
}

bool DBContextManager::hasSectorLayer(const string& layer_name) const
{
    traced_assert(sectors_loaded_);

    auto iter = find_if(sector_layers_.begin(), sector_layers_.end(),
                        [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name; });
    return iter != sector_layers_.end();
}

shared_ptr<SectorLayer> DBContextManager::sectorLayer(const string& layer_name) const
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSectorLayer(layer_name));

    auto iter = find_if(sector_layers_.begin(), sector_layers_.end(),
                        [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name; });
    traced_assert(iter != sector_layers_.end());
    return *iter;
}

bool DBContextManager::hasSector(const string& name, const string& layer_name) const
{
    traced_assert(sectors_loaded_);

    if (!hasSectorLayer(layer_name))
        return false;

    return sectorLayer(layer_name)->hasSector(name);
}

bool DBContextManager::hasSector(unsigned int id) const
{
    traced_assert(sectors_loaded_);

    for (const auto& lay : sector_layers_)
    {
        const auto& sectors = lay->sectors();
        auto iter = find_if(sectors.begin(), sectors.end(),
                            [id](const shared_ptr<Sector>& x) { return x->id() == id; });
        if (iter != sectors.end())
            return true;
    }
    return false;
}

shared_ptr<Sector> DBContextManager::sector(const string& name, const string& layer_name) const
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(name, layer_name));

    return sectorLayer(layer_name)->sector(name);
}

shared_ptr<Sector> DBContextManager::sector(unsigned int id) const
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(id));

    for (const auto& lay : sector_layers_)
    {
        const auto& sectors = lay->sectors();
        auto iter = find_if(sectors.begin(), sectors.end(),
                            [id](const shared_ptr<Sector>& x) { return x->id() == id; });
        if (iter != sectors.end())
            return *iter;
    }

    logerr << "id " << id << " not found";
    traced_assert(false);
    return nullptr; // unreachable
}

unsigned int DBContextManager::maxSectorId() const
{
    traced_assert(sectors_loaded_);
    return max_sector_id_;
}

shared_ptr<Sector> DBContextManager::createSector(const string& name, const string& layer_name,
                                                   bool exclude, QColor color,
                                                   vector<pair<double,double>> points)
{
    loginf << "name " << name << " layer_name " << layer_name
           << " num points " << points.size();

    traced_assert(sectors_loaded_);

    if (hasSector(name, layer_name))
    {
        logwrn << "sector '" << name << "' already exists in layer '" << layer_name << "'";
        return nullptr;
    }

    ++max_sector_id_;

    auto new_sector = make_shared<Sector>(max_sector_id_, name, layer_name, true, exclude, color, points);
    new_sector->setSaveCallback([this](unsigned int id) { saveSector(id); });
    new_sector->setMoveCallback([this](unsigned int id, const string& ol, const string& nl) { moveSector(id, ol, nl); });

    // add to context
    activeContext().sectors().push_back(new_sector);

    // save context
    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

    // rebuild cached layers
    rebuildSectorLayers();

    emit sectorsChangedSignal();

    return new_sector;
}

shared_ptr<Sector> DBContextManager::createOrReplaceSector(const string& name, const string& layer_name,
                                                            bool exclude, QColor color,
                                                            vector<pair<double,double>> points,
                                                            bool* replaced_existing)
{
    traced_assert(sectors_loaded_);

    bool replaced = hasSector(name, layer_name);

    if (replaced)
    {
        loginf << "replacing sector '" << name << "' in layer '" << layer_name << "'";

        unsigned int old_id = sector(name, layer_name)->id();

        auto& ctx_sectors = activeContext().sectors();
        auto iter = find_if(ctx_sectors.begin(), ctx_sectors.end(),
                            [old_id](const shared_ptr<Sector>& x) { return x->id() == old_id; });
        traced_assert(iter != ctx_sectors.end());
        ctx_sectors.erase(iter);

        rebuildSectorLayers();
    }

    if (replaced_existing)
        *replaced_existing = replaced;

    return createSector(name, layer_name, exclude, color, points);
}

void DBContextManager::deleteSector(shared_ptr<Sector> sector)
{
    traced_assert(sectors_loaded_);
    traced_assert(sector);
    traced_assert(hasSector(sector->name(), sector->layerName()));

    // remove from context
    auto& ctx_sectors = activeContext().sectors();
    auto iter = find_if(ctx_sectors.begin(), ctx_sectors.end(),
                        [&sector](const shared_ptr<Sector>& x) { return x->id() == sector->id(); });
    traced_assert(iter != ctx_sectors.end());
    ctx_sectors.erase(iter);

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

    rebuildSectorLayers();

    emit sectorsChangedSignal();
}

void DBContextManager::deleteSectors(const vector<shared_ptr<Sector>>& sectors)
{
    traced_assert(sectors_loaded_);

    if (sectors.empty())
        return;

    auto& ctx_sectors = activeContext().sectors();

    for (const auto& sector : sectors)
    {
        traced_assert(sector);
        auto iter = find_if(ctx_sectors.begin(), ctx_sectors.end(),
                            [&sector](const shared_ptr<Sector>& x) { return x->id() == sector->id(); });
        if (iter != ctx_sectors.end())
            ctx_sectors.erase(iter);
    }

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

    rebuildSectorLayers();

    emit sectorsChangedSignal();
}

void DBContextManager::deleteSectorLayer(const std::string& layer_name)
{
    traced_assert(sectors_loaded_);

    auto& ctx_sectors = activeContext().sectors();

    auto new_end = std::remove_if(ctx_sectors.begin(), ctx_sectors.end(),
                                  [&layer_name](const shared_ptr<Sector>& s)
                                  { return s->layerName() == layer_name; });

    if (new_end == ctx_sectors.end())
        return;

    ctx_sectors.erase(new_end, ctx_sectors.end());

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

    rebuildSectorLayers();

    emit sectorsChangedSignal();
}

void DBContextManager::renameSectorLayer(const std::string& old_name, const std::string& new_name)
{
    traced_assert(sectors_loaded_);
    traced_assert(!new_name.empty());

    if (old_name == new_name)
        return;

    bool changed = false;

    for (auto& s : activeContext().sectors())
    {
        if (s->layerName() == old_name)
        {
            s->layerName(new_name, false); // no per-sector move callback, persisted once below
            changed = true;
        }
    }

    if (!changed)
        return;

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

    rebuildSectorLayers();

    emit sectorsChangedSignal();
}

void DBContextManager::deleteAllSectors()
{
    traced_assert(sectors_loaded_);

    activeContext().sectors().clear();

    saveContext(activeContextName()); // also writes to the DB when one is open

    rebuildSectorLayers();

    emit sectorsChangedSignal();
}

void DBContextManager::saveSector(unsigned int id)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(id));

    saveSector(sector(id));
}

void DBContextManager::saveSector(shared_ptr<Sector> sector)
{
    traced_assert(sectors_loaded_);
    traced_assert(sector);

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();
}

void DBContextManager::moveSector(unsigned int id, const string& old_layer, const string& new_layer)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(id));

    // layer_name_ is already set by Sector::layerName() before it invokes the
    // move callback - calling the setter here again would recurse endlessly

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

    rebuildSectorLayers();

    emit sectorsChangedSignal();
}

void DBContextManager::importAirSpace(const AirSpace& air_space,
                                       const map<string, bool>& sectors_to_import,
                                       const string& target_layer_name)
{
    auto layers = air_space.layers();
    if (layers.empty())
        return;

    auto& ctx_sectors = activeContext().sectors();
    bool added = false;

    for (const auto& l : layers)
    {
        for (const auto& s : l->sectors())
        {
            auto it = sectors_to_import.find(s->name());
            if (it == sectors_to_import.end() || !it->second)
                continue;

            s->serializeSector(true);

            if (!target_layer_name.empty())
                s->layerName(target_layer_name);

            ctx_sectors.push_back(s);
            added = true;
        }
    }

    if (!added)
        return;

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

    rebuildSectorLayers();

    emit sectorsChangedSignal();
}

void DBContextManager::rebuildSectorLayers()
{
    sector_layers_.clear();
    max_sector_id_ = 0;

    if (!hasActiveContext())
    {
        sectors_loaded_ = false;
        return;
    }

    for (auto& s : activeContext().sectors())
    {
        s->setSaveCallback([this](unsigned int id) { saveSector(id); });
        s->setMoveCallback([this](unsigned int id, const string& ol, const string& nl) { moveSector(id, ol, nl); });

        const string& layer_name = s->layerName();

        // find or create layer
        auto iter = find_if(sector_layers_.begin(), sector_layers_.end(),
                            [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name; });

        if (iter == sector_layers_.end())
        {
            sector_layers_.push_back(make_shared<SectorLayer>(layer_name));
            sector_layers_.back()->addSector(s);
        }
        else
        {
            // override a duplicate sector name in the same layer (last one wins) instead of
            // aborting - self-heals contexts that ended up with duplicate sectors
            if ((*iter)->hasSector(s->name()))
            {
                logwrn << "duplicate sector '" << s->name() << "' in layer '" << layer_name
                       << "', overriding previous";
                (*iter)->removeSector((*iter)->sector(s->name()));
            }
            (*iter)->addSector(s);
        }

        max_sector_id_ = max(max_sector_id_, s->id());
    }

    sectors_loaded_ = true;
}

// ============================================================
// ASTERIX decoding config access
// ============================================================

bool DBContextManager::hasAsterixConfig(unsigned int category) const
{
    traced_assert(hasActiveContext());

    for (const auto& cfg : activeContext().asterixDecoding())
    {
        if (cfg.category() == category)
            return true;
    }
    return false;
}

ASTERIXDecodingConfig* DBContextManager::asterixConfig(unsigned int category)
{
    traced_assert(hasActiveContext());

    for (auto& cfg : activeContext().asterixDecoding())
    {
        if (cfg.category() == category)
            return &cfg;
    }
    return nullptr;
}

const ASTERIXDecodingConfig* DBContextManager::asterixConfig(unsigned int category) const
{
    traced_assert(hasActiveContext());

    for (const auto& cfg : activeContext().asterixDecoding())
    {
        if (cfg.category() == category)
            return &cfg;
    }
    return nullptr;
}

ASTERIXDecodingConfig& DBContextManager::getOrCreateAsterixConfig(unsigned int category,
    const string& default_edition,
    const string& default_ref,
    const string& default_spf)
{
    traced_assert(hasActiveContext());

    auto* existing = asterixConfig(category);
    if (existing)
        return *existing;

    // create new entry
    activeContext().asterixDecoding().emplace_back(category, default_edition, default_ref, default_spf);

    saveContext(activeContextName());

    return activeContext().asterixDecoding().back();
}

void DBContextManager::setAsterixEdition(unsigned int category, const string& edition,
                                         const string& default_ref)
{
    auto& cfg = getOrCreateAsterixConfig(category, edition, default_ref);
    cfg.edition(edition);
    saveContext(activeContextName());
}

void DBContextManager::setAsterixRef(unsigned int category, const string& ref,
                                     const string& default_edition)
{
    auto& cfg = getOrCreateAsterixConfig(category, default_edition, ref);
    cfg.ref(ref);
    saveContext(activeContextName());
}

void DBContextManager::setAsterixSpf(unsigned int category, const string& spf,
                                     const string& default_edition)
{
    auto& cfg = getOrCreateAsterixConfig(category, default_edition, "", spf);
    cfg.spf(spf);
    saveContext(activeContextName());
}

// ============================================================
// Per-section import/export
// ============================================================

void DBContextManager::importSensors(const string& filepath)
{
    traced_assert(hasActiveContext());

    ifstream ifs(filepath);
    if (!ifs.is_open())
        throw runtime_error("DBContextManager: importSensors: cannot open '" + filepath + "'");

    json j;
    try
    {
        ifs >> j;
    }
    catch (const json::exception& e)
    {
        throw runtime_error("DBContextManager: importSensors: parse error in '" + filepath
                            + "': " + e.what());
    }

    // support legacy format (content_type: "data_sources", content_version: "0.2")
    json data_arr;
    if (j.contains("data"))
        data_arr = j.at("data");
    else if (j.contains("data_sources"))
        data_arr = j.at("data_sources");
    else
    {
        logerr << "unsupported sensor import format";
        throw runtime_error("DBContextManager: importSensors: unsupported import format in '"
                            + filepath + "'");
    }

    // parse everything first so a malformed entry cannot leave a half-imported context
    std::vector<DataSource> parsed;
    parsed.reserve(data_arr.size());
    for (const auto& ds_j : data_arr)
        parsed.push_back(DataSource::fromJSON(ds_j));

    // commit: color assignment is incremental (each source spaced against the ones already
    // present), so keep autoAssignColors + add together here
    auto& ctx = activeContext();
    for (auto& ds : parsed)
    {
        autoAssignColors(ds);
        ctx.addOrReplaceDataSource(std::move(ds));
    }

    saveContext(active_context_name_);

    loginf << "imported " << data_arr.size() << " sensors";

    emit dataSourcesChangedSignal();
}

void DBContextManager::importFFTs(const string& filepath)
{
    traced_assert(hasActiveContext());

    ifstream ifs(filepath);
    if (!ifs.is_open())
        throw runtime_error("DBContextManager: importFFTs: cannot open '" + filepath + "'");

    json j;
    try
    {
        ifs >> j;
    }
    catch (const json::exception& e)
    {
        throw runtime_error("DBContextManager: importFFTs: parse error in '" + filepath
                            + "': " + e.what());
    }

    json data_arr;
    if (j.contains("data"))
        data_arr = j.at("data");
    else if (j.contains("ffts"))
        data_arr = j.at("ffts");
    else
    {
        logerr << "unsupported FFT import format";
        throw runtime_error("DBContextManager: importFFTs: unsupported import format in '"
                            + filepath + "'");
    }

    // parse everything first so a malformed entry cannot leave a half-imported context
    std::vector<FFT> parsed;
    parsed.reserve(data_arr.size());
    for (const auto& fft_j : data_arr)
        parsed.push_back(FFT::fromJSON(fft_j));

    auto& ctx = activeContext();
    for (auto& fft : parsed)
    {
        // override an existing FFT with the same name instead of duplicating it
        auto& fs = ctx.ffts();
        fs.erase(std::remove_if(fs.begin(), fs.end(),
                                [&fft](const FFT& f){ return f.name() == fft.name(); }),
                 fs.end());
        fs.push_back(std::move(fft));
    }

    saveContext(active_context_name_);

    loginf << "imported " << data_arr.size() << " FFTs";

    emit fftsChangedSignal();
}

void DBContextManager::importSectors(const string& filepath)
{
    traced_assert(hasActiveContext());

    ifstream ifs(filepath);
    if (!ifs.is_open())
        throw runtime_error("DBContextManager: importSectors: cannot open '" + filepath + "'");

    json j;
    try
    {
        ifs >> j;
    }
    catch (const json::exception& e)
    {
        throw runtime_error("DBContextManager: importSectors: parse error in '" + filepath
                            + "': " + e.what());
    }

    json data_arr;
    if (j.contains("data"))
        data_arr = j.at("data");
    else if (j.contains("sectors"))
        data_arr = j.at("sectors");
    else
    {
        logerr << "unsupported sector import format";
        throw runtime_error("DBContextManager: importSectors: unsupported import format in '"
                            + filepath + "'");
    }

    // parse everything first so a malformed entry cannot leave a half-imported context
    std::vector<std::shared_ptr<Sector>> parsed;
    parsed.reserve(data_arr.size());
    for (const auto& sec_j : data_arr)
    {
        unsigned int id = sec_j.at("id");
        string name = sec_j.at("name");
        string layer = sec_j.at("layer_name");
        auto sector = make_shared<Sector>(id, name, layer, false);
        sector->readJSON(sec_j);
        parsed.push_back(sector);
    }

    auto& ctx = activeContext();
    for (auto& sector : parsed)
    {
        // override an existing sector with the same name in the same layer instead of duplicating it
        auto& secs = ctx.sectors();
        secs.erase(std::remove_if(secs.begin(), secs.end(),
                                  [&sector](const std::shared_ptr<Sector>& s)
                                  { return s->name() == sector->name()
                                        && s->layerName() == sector->layerName(); }),
                   secs.end());
        secs.push_back(sector);
    }

    saveContext(active_context_name_); // also writes to the DB when one is open

    rebuildSectorLayers();

    loginf << "imported " << data_arr.size() << " sectors";

    emit sectorsChangedSignal();
}

void DBContextManager::exportSensors(const string& filepath)
{
    traced_assert(hasActiveContext());

    json j;
    j["version"] = DBContextSerializer::CURRENT_VERSION;
    j["content_type"] = "sensors";
    json arr = json::array();
    for (const auto& [ds_id, ds] : activeContext().dataSources())
        arr.push_back(ds.toJSON());
    j["data"] = arr;

    ofstream ofs(filepath);
    if (!ofs.is_open())
        throw runtime_error("DBContextManager: exportSensors: cannot open '" + filepath
                            + "' for writing");
    ofs << j.dump(4);

    loginf << "exported " << arr.size() << " sensors to " << filepath;
}

void DBContextManager::exportFFTs(const string& filepath)
{
    traced_assert(hasActiveContext());

    json j;
    j["version"] = DBContextSerializer::CURRENT_VERSION;
    j["content_type"] = "ffts";
    json arr = json::array();
    for (const auto& fft : activeContext().ffts())
        arr.push_back(fft.toJSON());
    j["data"] = arr;

    ofstream ofs(filepath);
    if (!ofs.is_open())
        throw runtime_error("DBContextManager: exportFFTs: cannot open '" + filepath
                            + "' for writing");
    ofs << j.dump(4);

    loginf << "exported " << arr.size() << " FFTs to " << filepath;
}

void DBContextManager::exportSectors(const string& filepath)
{
    traced_assert(hasActiveContext());

    json j;
    j["version"] = DBContextSerializer::CURRENT_VERSION;
    j["content_type"] = "sectors";
    json arr = json::array();
    for (const auto& s : activeContext().sectors())
        arr.push_back(s->jsonData());
    j["data"] = arr;

    ofstream ofs(filepath);
    if (!ofs.is_open())
        throw runtime_error("DBContextManager: exportSectors: cannot open '" + filepath
                            + "' for writing");
    ofs << j.dump(4);

    loginf << "exported " << arr.size() << " sectors to " << filepath;
}

void DBContextManager::exportContext(const string& name, const string& filepath)
{
    traced_assert(hasContext(name));
    DBContextSerializer::save(contexts_.at(name), filepath);
}

void DBContextManager::importContext(const string& filepath)
{
    DBContext ctx = DBContextSerializer::load(filepath);

    if (ctx.name().empty())
        throw runtime_error("DBContextManager: importContext: imported context has no name in '"
                            + filepath + "'");

    if (hasContext(ctx.name()))
        throw runtime_error("DBContextManager: importContext: a context named '" + ctx.name()
                            + "' already exists");

    DBContextSerializer::save(ctx, basePath());
    string name = ctx.name();
    contexts_[name] = std::move(ctx);

    loginf << "imported context '" << name << "'";

    emit contextsChangedSignal();
}

void DBContextManager::exportContextZip(const string& name, const string& zip_filepath)
{
    traced_assert(hasContext(name));
    DBContextSerializer::exportContextZip(basePath(), name, zip_filepath);
}

string DBContextManager::importContextZip(const string& zip_filepath)
{
    string name = DBContextSerializer::importContextZip(basePath(), zip_filepath);

    // reload the context from disk
    string ctx_dir = basePath() + "/" + name;
    DBContext ctx = DBContextSerializer::load(ctx_dir);

    contexts_[name] = std::move(ctx);

    loginf << "imported context '" << name << "' from zip";

    emit contextsChangedSignal();

    return name;
}

// ============================================================
// Storage
// ============================================================

// ============================================================
// Widgets
// ============================================================

DataSourcesToolWidget* DBContextManager::loadWidget()
{
    if (!load_widget_)
        load_widget_.reset(new DataSourcesToolWidget(*this));
    return load_widget_.get();
}

DataSourcesStatusToolWidget* DBContextManager::statusWidget()
{
    if (!status_widget_)
        status_widget_.reset(new DataSourcesStatusToolWidget(*this));
    return status_widget_.get();
}

string DBContextManager::basePath()
{
    return DATA_CONTEXTS_SUBDIRECTORY;
}

void DBContextManager::loadContextList()
{
    contexts_.clear();

    auto names = DBContextSerializer::listContexts(basePath());
    for (const auto& name : names)
    {
        string dir = basePath() + "/" + name;
        contexts_[name] = DBContextSerializer::load(dir);
    }
}

void DBContextManager::saveActiveContextName()
{
    json j;
    j["active_context"] = active_context_name_;

    ofstream ofs(basePath() + "/" + ACTIVE_CONTEXT_FILENAME);
    if (ofs.is_open())
        ofs << j.dump(4);
}

void DBContextManager::loadActiveContextName()
{
    string path = basePath() + "/" + ACTIVE_CONTEXT_FILENAME;

    if (!fs::exists(path))
        return;

    ifstream ifs(path);
    if (!ifs.is_open())
        return;

    json j;
    ifs >> j;

    if (j.contains("active_context"))
    {
        string name = j.at("active_context");
        if (hasContext(name))
            active_context_name_ = name;
    }
}

} // namespace context
