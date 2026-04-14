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

#include "compass.h"
#include "dbinterface.h"
#include "sector.h"
#include "sectorlayer.h"
#include "airspace.h"
#include "datasourcestoolwidget.h"
#include "datasourcesstatustoolwidget.h"
#include "datasourcesconfigurationdialog.h"
#include "db_context_create_dialog.h"
#include "db_context_select_dialog.h"
#include "files.h"
#include "logger.h"
#include "number.h"
#include "traced_assert.h"

#include <json.hpp>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

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
    traced_assert(!hasContext(name));

    DBContext ctx(name);
    DBContextSerializer::save(ctx, basePath());

    contexts_[name] = std::move(ctx);

    loginf << "created context '" << name << "'";

    emit contextsChangedSignal();
}

void DBContextManager::deleteContext(const string& name)
{
    traced_assert(hasContext(name));

    DBContextSerializer::deleteContext(basePath(), name);
    contexts_.erase(name);

    if (active_context_name_ == name)
    {
        active_context_name_.clear();
        saveActiveContextName();
        emit activeContextChangedSignal();
    }

    loginf << "deleted context '" << name << "'";

    emit contextsChangedSignal();
}

void DBContextManager::renameContext(const string& old_name, const string& new_name)
{
    traced_assert(hasContext(old_name));
    traced_assert(!hasContext(new_name));

    DBContextSerializer::renameContext(basePath(), old_name, new_name);

    auto ctx = std::move(contexts_.at(old_name));
    contexts_.erase(old_name);
    ctx.name(new_name);
    contexts_[new_name] = std::move(ctx);

    if (active_context_name_ == old_name)
    {
        active_context_name_ = new_name;
        saveActiveContextName();
        emit activeContextChangedSignal();
    }

    loginf << "renamed context '" << old_name << "' to '" << new_name << "'";

    emit contextsChangedSignal();
}

void DBContextManager::duplicateContext(const string& src, const string& dest)
{
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

    auto& ctx = contexts_.at(name);
    ctx.modified(DBContext::currentTimestamp());
    DBContextSerializer::save(ctx, basePath());

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
    traced_assert(hasContext(name));

    if (active_context_name_ == name)
        return;

    active_context_name_ = name;
    saveActiveContextName();

    loginf << "active context set to '" << name << "', rebuilding sector layers";

    rebuildSectorLayers();

    loginf << "sector layers rebuilt, sectors_loaded_=" << sectors_loaded_;

    emit activeContextChangedSignal();
}

// ============================================================
// Data source query/lookup
// ============================================================

bool DBContextManager::hasDataSource(unsigned int ds_id) const
{
    if (!hasActiveContext()) return false;
    for (const auto& ds : activeContext().dataSources())
        if (ds.id() == ds_id) return true;
    return false;
}

const DataSource* DBContextManager::dataSource(unsigned int ds_id) const
{
    if (!hasActiveContext()) return nullptr;
    for (const auto& ds : activeContext().dataSources())
        if (ds.id() == ds_id) return &ds;
    return nullptr;
}

DataSource* DBContextManager::dataSource(unsigned int ds_id)
{
    if (!hasActiveContext()) return nullptr;
    for (auto& ds : activeContext().dataSources())
        if (ds.id() == ds_id) return &ds;
    return nullptr;
}

bool DBContextManager::hasDataSource(const string& name) const
{
    if (!hasActiveContext()) return false;
    for (const auto& ds : activeContext().dataSources())
        if (ds.name() == name) return true;
    return false;
}

unsigned int DBContextManager::getDataSourceId(const string& name) const
{
    for (const auto& ds : activeContext().dataSources())
        if (ds.name() == name) return ds.id();
    traced_assert(false); // not found
    return 0;
}

vector<unsigned int> DBContextManager::allDataSourceIds() const
{
    vector<unsigned int> ids;
    if (!hasActiveContext()) return ids;
    for (const auto& ds : activeContext().dataSources())
        ids.push_back(ds.id());
    return ids;
}

vector<IDataSourceProvider::DataSourceInfo> DBContextManager::dataSourceInfos() const
{
    vector<IDataSourceProvider::DataSourceInfo> infos;
    if (!hasActiveContext()) return infos;
    for (const auto& ds : activeContext().dataSources())
        infos.push_back({ds.id(), ds.name(), ds.dsType()});
    return infos;
}

set<unsigned int> DBContextManager::groundOnlyDataSources() const
{
    set<unsigned int> result;
    if (!hasActiveContext()) return result;
    for (const auto& ds : activeContext().dataSources())
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
    for (const auto& ds : activeContext().dataSources())
        result[ds.id()] = ds.dsType();
    return result;
}

DataSource& DBContextManager::createDataSource(unsigned int sac, unsigned int sic)
{
    traced_assert(hasActiveContext());

    DataSource ds;
    ds.sac(sac);
    ds.sic(sic);
    ds.dsType("Other");
    ds.name("New " + to_string(sac) + "/" + to_string(sic));

    activeContext().dataSources().push_back(std::move(ds));
    saveContext(active_context_name_);

    if (compass_.dbOpened())
        writeContextToDB();

    emit activeContextChangedSignal();

    return activeContext().dataSources().back();
}

void DBContextManager::deleteDataSource(unsigned int ds_id)
{
    traced_assert(hasActiveContext());

    auto& sources = activeContext().dataSources();
    sources.erase(
        remove_if(sources.begin(), sources.end(),
                  [ds_id](const DataSource& ds) { return ds.id() == ds_id; }),
        sources.end());

    saveContext(active_context_name_);
    emit activeContextChangedSignal();
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
            for (const auto& ds : activeContext().dataSources())
                ds_type_loading_wanted_[ds.dsType()] = false;
        }
    }
}

void DBContextManager::setLoadOnlyDSTypes(set<string> ds_types)
{
    if (!hasActiveContext()) return;

    // set all to false, then enable only specified
    for (const auto& ds : activeContext().dataSources())
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

    for (const auto& ds : activeContext().dataSources())
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
        for (const auto& ds : activeContext().dataSources())
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

    for (const auto& ds : activeContext().dataSources())
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

    for (const auto& ds : activeContext().dataSources())
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
    for (const auto& ds : activeContext().dataSources())
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

    for (const auto& ds : activeContext().dataSources())
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

    for (const auto& ds : activeContext().dataSources())
        line_loading_wanted_[ds.id()][line_id] = true;
}

void DBContextManager::selectDSTypeSpecificDataSources(const string& ds_type)
{
    if (!hasActiveContext()) return;
    for (const auto& ds : activeContext().dataSources())
        if (ds.dsType() == ds_type)
            ds_loading_wanted_[ds.id()] = true;
}

void DBContextManager::deselectDSTypeSpecificDataSources(const string& ds_type)
{
    if (!hasActiveContext()) return;
    for (const auto& ds : activeContext().dataSources())
        if (ds.dsType() == ds_type)
            ds_loading_wanted_[ds.id()] = false;
}

vector<unsigned int> DBContextManager::unfilteredDS(const string& /*dbcontent_name*/) const
{
    vector<unsigned int> result;
    if (!hasActiveContext()) return result;

    for (const auto& ds : activeContext().dataSources())
    {
        if (dsTypeLoadingWanted(ds.dsType()) && loadingWanted(ds.id()))
            result.push_back(ds.id());
    }
    return result;
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

void DBContextManager::applyDeleteInfo(const json& /*delete_info*/)
{
    // TODO: implement when wiring callers — adjust counts and remove empty DS
    loginf << "applyDeleteInfo not yet implemented";
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

// ============================================================
// Network lines
// ============================================================

map<unsigned int, map<string, json>> DBContextManager::getNetworkLines() const
{
    map<unsigned int, map<string, json>> result;

    if (!hasActiveContext()) return result;

    for (const auto& ds : activeContext().dataSources())
    {
        if (ds.info().contains("network_lines"))
            result[ds.id()] = ds.info().at("network_lines").get<map<string, json>>();
    }

    return result;
}

void DBContextManager::createNetworkDBDataSources()
{
    // TODO: implement when wiring network import — create data sources from network config
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

    emit activeContextChangedSignal();

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
    emit activeContextChangedSignal();
}

void DBContextManager::deleteAllFFTs()
{
    traced_assert(hasActiveContext());
    activeContext().ffts().clear();
    saveContext(active_context_name_);
    emit activeContextChangedSignal();
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
    for (const auto& ds : ctx.dataSources())
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

    // sensors
    if (sections.count("sensors"))
    {
        json j = json::parse(sections.at("sensors"));
        if (j.contains("data"))
        {
            for (const auto& ds_j : j.at("data"))
                ctx.dataSources().push_back(DataSource::fromJSON(ds_j));
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

    if (!hasActiveContext())
    {
        logwrn << "no active context set — context must be created first";
        return;
    }

    auto& db = compass_.dbInterface();

    if (!db.existsDBContextTable())
    {
        // first open — write the active context to DB
        loginf << "no db_context table — writing active context to DB";
        writeContextToDB();
    }
    else
    {
        DBContext db_ctx = readContextFromDB();

        if (db_ctx.name() != activeContext().name())
        {
            // DB was saved with a different context — switch to it
            loginf << "DB context '" << db_ctx.name() << "' differs from active '"
                   << activeContext().name() << "' — switching to DB context";

            if (!hasContext(db_ctx.name()))
            {
                // context doesn't exist on disk — create from DB data
                loginf << "context '" << db_ctx.name() << "' not found on disk, creating from DB";
                db_ctx.modified(DBContext::currentTimestamp());
                DBContextSerializer::save(db_ctx, basePath());
                contexts_[db_ctx.name()] = std::move(db_ctx);
            }

            setActiveContext(db_ctx.name());
        }
        else
        {
            auto d = DBContextDiff::compute(activeContext(), db_ctx);

            if (d.hasDifferences())
            {
                logwrn << "context differs from DB:\n" << d.summary()
                       << "using file context as source of truth";

                // same name, file context wins — overwrite DB
                writeContextToDB();
            }
            else
            {
                loginf << "context matches DB — no action needed";
            }
        }
    }

    loadCountsFromDB();

    loginf << "emitting activeContextChangedSignal and countsChangedSignal";

    emit activeContextChangedSignal();
    emit countsChangedSignal();
}

void DBContextManager::databaseClosedSlot()
{
    loginf << "database closed";

    // counts already saved in COMPASS::closeDBInternal() before DB was closed

    inserted_counts_.clear();
    loaded_counts_.clear();

    loginf << "emitting countsChangedSignal";

    emit countsChangedSignal();
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
    traced_assert(!hasSector(name, layer_name));

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

void DBContextManager::deleteAllSectors()
{
    traced_assert(sectors_loaded_);

    activeContext().sectors().clear();

    saveContext(activeContextName());
    if (compass_.dbOpened())
        writeContextToDB();

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

    // find the sector and change its layer name
    auto sec = sector(id);
    sec->layerName(new_layer);

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

// ============================================================
// Per-section import/export
// ============================================================

void DBContextManager::importSensors(const string& filepath)
{
    traced_assert(hasActiveContext());

    ifstream ifs(filepath);
    traced_assert(ifs.is_open());

    json j;
    ifs >> j;

    // support legacy format (content_type: "data_sources", content_version: "0.2")
    json data_arr;
    if (j.contains("data"))
        data_arr = j.at("data");
    else if (j.contains("data_sources"))
        data_arr = j.at("data_sources");
    else
    {
        logerr << "unsupported sensor import format";
        return;
    }

    auto& ctx = activeContext();
    for (const auto& ds_j : data_arr)
        ctx.dataSources().push_back(DataSource::fromJSON(ds_j));

    saveContext(active_context_name_);

    loginf << "imported " << data_arr.size() << " sensors";

    emit activeContextChangedSignal();
}

void DBContextManager::importFFTs(const string& filepath)
{
    traced_assert(hasActiveContext());

    ifstream ifs(filepath);
    traced_assert(ifs.is_open());

    json j;
    ifs >> j;

    json data_arr;
    if (j.contains("data"))
        data_arr = j.at("data");
    else if (j.contains("ffts"))
        data_arr = j.at("ffts");
    else
    {
        logerr << "unsupported FFT import format";
        return;
    }

    auto& ctx = activeContext();
    for (const auto& fft_j : data_arr)
        ctx.ffts().push_back(FFT::fromJSON(fft_j));

    saveContext(active_context_name_);

    loginf << "imported " << data_arr.size() << " FFTs";

    emit activeContextChangedSignal();
}

void DBContextManager::importSectors(const string& filepath)
{
    traced_assert(hasActiveContext());

    ifstream ifs(filepath);
    traced_assert(ifs.is_open());

    json j;
    ifs >> j;

    json data_arr;
    if (j.contains("data"))
        data_arr = j.at("data");
    else if (j.contains("sectors"))
        data_arr = j.at("sectors");
    else
    {
        logerr << "unsupported sector import format";
        return;
    }

    auto& ctx = activeContext();
    for (const auto& sec_j : data_arr)
    {
        unsigned int id = sec_j.at("id");
        string name = sec_j.at("name");
        string layer = sec_j.at("layer_name");
        auto sector = make_shared<Sector>(id, name, layer, false);
        sector->readJSON(sec_j);
        ctx.sectors().push_back(sector);
    }

    saveContext(active_context_name_);
    if (compass_.dbOpened())
        writeContextToDB();

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
    for (const auto& ds : activeContext().dataSources())
        arr.push_back(ds.toJSON());
    j["data"] = arr;

    ofstream ofs(filepath);
    traced_assert(ofs.is_open());
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
    traced_assert(ofs.is_open());
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
    traced_assert(ofs.is_open());
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
    traced_assert(!ctx.name().empty());
    traced_assert(!hasContext(ctx.name()));

    DBContextSerializer::save(ctx, basePath());
    string name = ctx.name();
    contexts_[name] = std::move(ctx);

    loginf << "imported context '" << name << "'";

    emit contextsChangedSignal();
}

// ============================================================
// Storage
// ============================================================

// ============================================================
// Startup context check
// ============================================================

bool DBContextManager::ensureActiveContext(QWidget* parent)
{
    if (hasActiveContext())
        return true;

    if (contexts_.empty())
    {
        // no contexts exist — force creation
        DBContextCreateDialog dialog(*this, parent);
        if (dialog.exec() != QDialog::Accepted)
            return false;
    }
    else
    {
        // contexts exist but none is active — force selection
        DBContextSelectDialog dialog(*this, parent);
        if (dialog.exec() != QDialog::Accepted)
            return false;
    }

    return hasActiveContext();
}

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

DataSourcesConfigurationDialog* DBContextManager::configurationDialog()
{
    if (!config_dialog_)
        config_dialog_.reset(new DataSourcesConfigurationDialog(*this));
    return config_dialog_.get();
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
