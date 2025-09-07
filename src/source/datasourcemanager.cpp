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


#include "datasourcemanager.h"
#include "datasourceswidget.h"
#include "datasourcesconfigurationdialog.h"
#include "compass.h"
#include "dbinterface.h"
#include "stringconv.h"
#include "number.h"
#include "files.h"
#include "json.hpp"
#include "datasource_commands.h"
#include "logger.h"

#include <QMessageBox>

#include <fstream>

using namespace std;
using namespace Utils;
using namespace dbContent;
using namespace nlohmann;

const std::vector<std::string> DataSourceManager::data_source_types_ 
    {"Radar", "MLAT", "ADSB", "Tracker", "RefTraj", "Other"};

dbContent::DataSourceType DataSourceManager::typeFromString(const std::string& type_str)
{
    if (type_str == "ADSB")
        return DataSourceType::ADSB;
    else if (type_str == "MLAT")
        return DataSourceType::MLAT;
    else if (type_str == "Radar")
        return DataSourceType::Radar;
    else if (type_str == "Tracker")
        return DataSourceType::Tracker;
    else if (type_str == "RefTraj")
        return DataSourceType::RefTraj;
    else if (type_str == "Other")
        return DataSourceType::Other;

    traced_assert(false);
}

std::string DataSourceManager::stringFromType(dbContent::DataSourceType type)
{
    if (type == DataSourceType::ADSB)
        return "ADSB";
    else if (type == DataSourceType::MLAT)
        return "MLAT";
    else if (type == DataSourceType::Radar)
        return "Radar";
    else if (type == DataSourceType::Tracker)
        return "Tracker";
    else if (type == DataSourceType::RefTraj)
        return "RefTraj";
    else if (type ==  DataSourceType::Other)
        return "Other";

    traced_assert(false);
}

DataSourceManager::Config::Config()
:   load_widget_show_counts_ {true}
,   load_widget_show_lines_  {true}
,   ds_font_size_            {10}
,   primary_azimuth_stddev_  (0.05)
,   primary_range_stddev_    (120.0)
,   secondary_azimuth_stddev_(0.025)
,   secondary_range_stddev_  (70.0)
,   mode_s_azimuth_stddev_   (0.02) // 70m in 200km, 0.02 * 2 * pi * 200000 /360
,   mode_s_range_stddev_     (50)
    //,   use_radar_min_stddev_                 (false)
{
}

DataSourceManager::DataSourceManager(const std::string& class_id, const std::string& instance_id,
                                     COMPASS* compass)
    : Configurable(class_id, instance_id, compass, "data_sources.json"), compass_(*compass)

{
    registerParameter("load_widget_show_counts", &config_.load_widget_show_counts_, Config().load_widget_show_counts_);
    registerParameter("load_widget_show_lines", &config_.load_widget_show_lines_, Config().load_widget_show_lines_);

    registerParameter("ds_font_size", &config_.ds_font_size_, Config().ds_font_size_);

    registerParameter("primary_azimuth_stddev", &config_.primary_azimuth_stddev_, Config().primary_azimuth_stddev_);
    registerParameter("primary_range_stddev", &config_.primary_range_stddev_, Config().primary_range_stddev_);

    registerParameter("secondary_azimuth_stddev", &config_.secondary_azimuth_stddev_, Config().secondary_azimuth_stddev_);
    registerParameter("secondary_range_stddev", &config_.secondary_range_stddev_, Config().secondary_range_stddev_);

    registerParameter("mode_s_azimuth_stddev", &config_.mode_s_azimuth_stddev_, Config().mode_s_azimuth_stddev_);
    registerParameter("mode_s_range_stddev", &config_.mode_s_range_stddev_, Config().mode_s_range_stddev_);

    //registerParameter("use_radar_min_stddev", &config_.use_radar_min_stddev_, Config().use_radar_min_stddev_);

    createSubConfigurables();

    updateDSIdsAll();


    dbContent::init_data_source_commands();
}

DataSourceManager::~DataSourceManager()
{
    logdbg;

    config_data_sources_.clear();
    db_data_sources_.clear(); // delete their widgets, which removes them from load_widget_

    load_widget_ = nullptr; // deleted by qt
}

void DataSourceManager::generateSubConfigurable(const std::string& class_id,
                                                const std::string& instance_id)
{
    if (class_id == "ConfigurationDataSource")
    {
        unique_ptr<dbContent::ConfigurationDataSource> ds {
            new dbContent::ConfigurationDataSource(class_id, instance_id, *this)};
        logdbg << "adding config ds "
                   << ds->name() << " sac/sic " <<  ds->sac() << "/" << ds->sic();

        traced_assert(!hasConfigDataSource(Number::dsIdFrom(ds->sac(), ds->sic())));
        config_data_sources_.emplace_back(move(ds));
    }
    else
        throw std::runtime_error("DataSourceManager: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

const std::vector<unsigned int>& DataSourceManager::getAllDsIDs()
{
    return ds_ids_all_;
}

DataSourcesWidget* DataSourceManager::loadWidget()
{
    if (!load_widget_)
    {
        load_widget_ = new DataSourcesWidget(*this);
    }


    traced_assert(load_widget_);
    
    return load_widget_;

}

void DataSourceManager::updateWidget()
{
    if (load_widget_)
        load_widget_->updateContent();
}

DataSourcesConfigurationDialog* DataSourceManager::configurationDialog()
{
    if (!config_dialog_)
    {
        config_dialog_.reset(new DataSourcesConfigurationDialog(*this));

        connect (config_dialog_.get(), &DataSourcesConfigurationDialog::doneSignal,
                 this, &DataSourceManager::configurationDialogDoneSlot);
    }

    return config_dialog_.get();
}

void DataSourceManager::importDataSources(const std::string& filename)
{
    loginf << "file '" << filename << "'";

    try
    {
        if (!Files::fileExists(filename))
            throw std::runtime_error ("File '"+filename+"' not found.");

        std::ifstream input_file(filename, std::ifstream::in);

        json j = json::parse(input_file);

        if (!j.contains("content_type"))
            importDataSourcesJSONDeprecated(j);
        else
            importDataSourcesJSON(j);
    }
    catch (json::exception& e)
    {
        logerr << "could not load file '"
                   << filename << "'";
        throw e;
    }

    updateDSIdsAll();
    updateWidget();

    emit dataSourcesChangedSignal();
}

void DataSourceManager::importDataSourcesJSONDeprecated(const nlohmann::json& j)
{
    loginf;

    for (auto& j_dbcont_it : j.items())
    {
        std::string dbcontent_name = j_dbcont_it.key();

        for (auto& j_ds_it : j_dbcont_it.value().get<json::array_t>())
        {
            loginf << "found dbcontent " << dbcontent_name
                   << " ds '" << j_ds_it.dump(4) << "'";

            traced_assert(j_ds_it.contains("dbcont_name"));
            traced_assert(j_ds_it.contains("name"));
            traced_assert(j_ds_it.contains("sac"));
            traced_assert(j_ds_it.contains("sic"));

            unsigned int sac = j_ds_it.at("sac");
            unsigned int sic = j_ds_it.at("sic");

            unsigned int ds_id = Number::dsIdFrom(sac, sic);

            if (!hasConfigDataSource(ds_id))
                createConfigDataSource(ds_id);

            configDataSource(ds_id).setFromJSONDeprecated(j_ds_it);

            if (hasDBDataSource(ds_id))
                dbDataSource(ds_id).setFromJSONDeprecated(j_ds_it);
        }
    }
}

void DataSourceManager::importDataSourcesJSON(const nlohmann::json& j)
{
    loginf;

    if (!j.contains("content_type")
            || !j.at("content_type").is_string()
            || j.at("content_type") != "data_sources")
        throw std::runtime_error("current data is not data sources content");

    if (!j.contains("content_version")
            || !j.at("content_version").is_string()
            || j.at("content_version") != "0.2")
        throw std::runtime_error("current data content version is not supported");

    if (!j.contains("data_sources")
            || !j.at("data_sources").is_array())
        throw std::runtime_error("current data contains no data sources");

    for (auto& j_ds_it : j.at("data_sources").get<json::array_t>())
    {
        traced_assert(j_ds_it.contains("ds_type"));
        traced_assert(j_ds_it.contains("name"));
        traced_assert(j_ds_it.contains("sac"));
        traced_assert(j_ds_it.contains("sic"));

        unsigned int sac = j_ds_it.at("sac");
        unsigned int sic = j_ds_it.at("sic");

        unsigned int ds_id = Number::dsIdFrom(sac, sic);

        if (!hasConfigDataSource(ds_id))
            createConfigDataSource(ds_id);

        configDataSource(ds_id).setFromJSON(j_ds_it);

        if (hasDBDataSource(ds_id))
            dbDataSource(ds_id).setFromJSON(j_ds_it);
    }

    if (COMPASS::instance().dbOpened())
        saveDBDataSources();
}

void DataSourceManager::deleteAllConfigDataSources()
{
    loginf;

    for(auto it = config_data_sources_.begin(); it != config_data_sources_.end();)
    {
        if(!hasDBDataSource((*it)->id())) // erase if not in db
        {
            it = config_data_sources_.erase(it); // erase and update it
        }
        else // next one
        {
            ++it;
        }
    }

    updateDSIdsAll();

    emit dataSourcesChangedSignal();
}

void DataSourceManager::exportDataSources(const std::string& filename)
{
    loginf << "file '" << filename << "'";

    json data = getDataSourcesAsJSON();

    std::ofstream file(filename);
    file << data.dump(4);

    QMessageBox m_info(QMessageBox::Information, "Export Data Sources",
                       "File export: '"+QString(filename.c_str())+"' done.\n"
                       +QString::number(config_data_sources_.size())+" Data Sources saved.", QMessageBox::Ok);
    m_info.exec();
}

nlohmann::json DataSourceManager::getDataSourcesAsJSON()
{
    json data;

    data["content_type"] = "data_sources";
    data["content_version"] = "0.2";

    data["data_sources"] = json::array();
    json& data_sources = data.at("data_sources");

    set<unsigned int> joined_data_source_ids;

    for (auto& ds_it : db_data_sources_)
    {
        auto json_info = ds_it->getAsJSON();

        if (json_info.contains("counts"))
            json_info.erase("counts");

        data_sources[joined_data_source_ids.size()] = json_info;
        joined_data_source_ids.insert(ds_it->id());
    }

    for (auto& ds_it : config_data_sources_)
    {
        if (joined_data_source_ids.count(ds_it->id()))
            continue;

        data_sources[joined_data_source_ids.size()] = ds_it->getAsJSON();
        joined_data_source_ids.insert(ds_it->id());
    }

    return data;
}

nlohmann::json DataSourceManager::getDBDataSourcesAsJSON()
{
    json data;

    data["content_type"] = "data_sources";
    data["content_version"] = "0.2";

    data["data_sources"] = json::array();
    json& data_sources = data.at("data_sources");

    unsigned int cnt = 0;

    for (auto& ds_it : db_data_sources_)
    {
        data_sources[cnt] = ds_it->getAsJSON();
        ++cnt;
    }

    return data;
}

namespace
{
    void sortJSONDataSource(nlohmann::json& ds)
    {
        traced_assert(ds.contains("data_sources"));

        json& ds_array = ds.at("data_sources");

        std::sort(ds_array.begin(), ds_array.end(), 
            [ & ] (const nlohmann::json& j0, const nlohmann::json& j1) 
            { 
                traced_assert(j0.contains("name"));
                traced_assert(j1.contains("name"));

                std::string n0 = j0[ "name" ];
                std::string n1 = j1[ "name" ];

                return n0 < n1;
            });
    }
}

nlohmann::json DataSourceManager::getSortedConfigDataSourcesAsJSON()
{
    auto ds_json = getDataSourcesAsJSON();
    sortJSONDataSource(ds_json);
    return ds_json;
}

nlohmann::json DataSourceManager::getSortedDBDataSourcesAsJSON()
{
    auto ds_json = getDBDataSourcesAsJSON();
    sortJSONDataSource(ds_json);
    return ds_json;
}

// ds id->dbcont->line->cnt
void DataSourceManager::setLoadedCounts(std::map<unsigned int, std::map<std::string,
                                        std::map<unsigned int, unsigned int>>> loaded_counts)
{
    // clear num loaded

    for (auto& db_src_it : db_data_sources_)
        db_src_it->clearNumLoaded();

    for (auto ds_id_it : loaded_counts)
    {
        traced_assert(hasDBDataSource(ds_id_it.first));
        DBDataSource& src = dbDataSource(ds_id_it.first);

        for (auto dbcont_it : ds_id_it.second)
        {
            for (auto line_it : dbcont_it.second)
            {
                src.addNumLoaded(dbcont_it.first, line_it.first, line_it.second);
            }
        }
    }

    if (load_widget_ && config_.load_widget_show_counts_)
        load_widget_->updateContent();
}

void DataSourceManager::clearInsertedCounts(const std::string& dbcontent_name)
{
    for (auto& db_src_it : db_data_sources_)
        db_src_it->clearNumInserted(dbcontent_name);

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::selectAllDSTypes()
{
    ds_type_loading_wanted_.clear();

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::deselectAllDSTypes()
{
    for (const auto& ds_type_it : data_source_types_)
        ds_type_loading_wanted_[ds_type_it] = false;

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::selectAllDataSources()
{
    for (const auto& ds_it : db_data_sources_)
        ds_it->loadingWanted(true);

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::deselectAllDataSources()
{
    for (const auto& ds_it : db_data_sources_)
        ds_it->loadingWanted(false);

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::selectDSTypeSpecificDataSources (const std::string& ds_type)
{
    traced_assert(find(data_source_types_.begin(), data_source_types_.end(), ds_type) != data_source_types_.end());

    for (const auto& ds_it : db_data_sources_)
    {
        if (ds_it->dsType() == ds_type)
            ds_it->loadingWanted(true);
    }

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::deselectDSTypeSpecificDataSources (const std::string& ds_type)
{
    traced_assert(find(data_source_types_.begin(), data_source_types_.end(), ds_type) != data_source_types_.end());

    for (const auto& ds_it : db_data_sources_)
    {
        if (ds_it->dsType() == ds_type)
            ds_it->loadingWanted(false);
    }

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::deselectAllLines()
{
    for (const auto& ds_it : db_data_sources_)
    {
        ds_it->disableAllLines();
    }

    if (load_widget_)
        load_widget_->updateContent();
}
void DataSourceManager::selectSpecificLineSlot(unsigned int line_id)
{

    for (const auto& ds_it : db_data_sources_)
    {
        ds_it->disableAllLines();

        if (ds_it->hasNumInserted(line_id))
            ds_it->lineLoadingWanted(line_id, true);
    }

    if (load_widget_)
        load_widget_->updateContent();
}

DataSourceManager::Config& DataSourceManager::config()
{
    return config_;
}

bool DataSourceManager::loadingWanted (const std::string& dbcontent_name)
{
    for (auto& ds_it : db_data_sources_)
    {
        if (dsTypeLoadingWanted(ds_it->dsType()) && ds_it->loadingWanted() && ds_it->hasNumInserted(dbcontent_name))
            return true;
    }

    return false;
}

bool DataSourceManager::hasDSFilter (const std::string& dbcontent_name)
{
    for (auto& ds_it : db_data_sources_)
    {
        if (dsTypeLoadingWanted(ds_it->dsType()) && !ds_it->loadingWanted() && ds_it->hasNumInserted(dbcontent_name))
            return true;
    }

    return false;
}

std::vector<unsigned int> DataSourceManager::unfilteredDS (const std::string& dbcontent_name)
{
    //assert (hasDSFilter(dbcontent_name)); can also be used if no filter active

    std::vector<unsigned int> ds_ids;

    for (auto& ds_it : db_data_sources_)
    {
        if (dsTypeLoadingWanted(ds_it->dsType()) && ds_it->loadingWanted() && ds_it->hasNumInserted(dbcontent_name))
            ds_ids.push_back(ds_it->id());
    }

    traced_assert(ds_ids.size());

    return ds_ids;
}

bool DataSourceManager::lineSpecificLoadingRequired(const std::string& dbcontent_name)
{
    for (auto& ds_it : db_data_sources_)
        if (dsTypeLoadingWanted(ds_it->dsType()) && ds_it->loadingWanted() && ds_it->hasNumInserted(dbcontent_name)
                && ds_it->lineSpecificLoadingWanted())
            return true;

    return false;
}

void DataSourceManager::setLoadDataSources (bool loading_wanted)
{
    loginf << "wanted " << loading_wanted;

    for (auto& ds_it : db_data_sources_)
        ds_it->loadingWanted(loading_wanted);

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::setLoadAllDataSourceLines ()
{
    loginf;

    for (auto& ds_it : db_data_sources_)
        ds_it->enableAllLines();

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::setLoadOnlyDataSources (std::map<unsigned int, std::set<unsigned int>> ds_ids)
{
    loginf;

    // deactivate all loading
    setLoadDataSources(false);

    for (auto ds_id_it : ds_ids)
    {
        traced_assert(hasDBDataSource(ds_id_it.first));
        dbDataSource(ds_id_it.first).loadingWanted(true);
        dbDataSource(ds_id_it.first).disableAllLines();

        for (auto& line_str_it : ds_id_it.second)
            dbDataSource(ds_id_it.first).lineLoadingWanted(line_str_it, true);
    }

    if (load_widget_)
        load_widget_->updateContent();
}

bool DataSourceManager::loadDataSourcesFiltered()
{
    for (auto& ds_it : db_data_sources_)
        if (!ds_it->loadingWanted())
            return true;

    return false;
}

std::map<unsigned int, std::set<unsigned int>> DataSourceManager::getLoadDataSources ()
{
    std::map<unsigned int, std::set<unsigned int>> ds_to_load;

    for (auto& ds_it : db_data_sources_)
    {
        if (dsTypeLoadingWanted(ds_it->dsType()) && ds_it->loadingWanted())
            ds_to_load[ds_it->id()] = ds_it->getLoadingWantedLines();
    }

    return ds_to_load;
}

void DataSourceManager::resetToStartupConfiguration()
{
    loginf;

    selectAllDSTypes();
    selectAllDataSources();
    selectSpecificLineSlot(0);
}

void DataSourceManager::databaseOpenedSlot()
{

    loginf;

    loadDBDataSources();

    emit dataSourcesChangedSignal();

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::databaseClosedSlot()
{
    db_data_sources_.clear();

    updateDSIdsAll();

    emit dataSourcesChangedSignal();

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::configurationDialogDoneSlot()
{
    loginf;

    config_dialog_->hide();
    config_dialog_ = nullptr;

    saveDBDataSources();

    if (load_widget_)
        load_widget_->updateContent(true);

    emit dataSourcesChangedSignal();
}

bool DataSourceManager::hasConfigDataSource (unsigned int ds_id)
{
    return find_if(config_data_sources_.begin(), config_data_sources_.end(),
                   [ds_id] (const std::unique_ptr<dbContent::ConfigurationDataSource>& s)
    { return s->id() == ds_id; } ) != config_data_sources_.end();
}

void DataSourceManager::createConfigDataSource(unsigned int ds_id)
{
    traced_assert(!hasConfigDataSource(ds_id));

    auto new_cfg = Configuration::create("ConfigurationDataSource");

    new_cfg->addParameter<std::string>("ds_type", "Other");
    new_cfg->addParameter<unsigned int>("sac", Number::sacFromDsId(ds_id));
    new_cfg->addParameter<unsigned int>("sic", Number::sicFromDsId(ds_id));
    new_cfg->addParameter<std::string>("name", "Unknown ("+to_string(Number::sacFromDsId(ds_id))+"/"+to_string(Number::sicFromDsId(ds_id))+")");

    generateSubConfigurableFromConfig(std::move(new_cfg));

    updateDSIdsAll();
}

void DataSourceManager::deleteConfigDataSource(unsigned int ds_id)
{
    loginf << "ds_id " << ds_id;

    traced_assert(hasConfigDataSource(ds_id));
    traced_assert(!hasDBDataSource(ds_id)); // can not delete config data sources in existing db

    auto ds_it = find_if(config_data_sources_.begin(), config_data_sources_.end(),
                         [ds_id] (const std::unique_ptr<dbContent::ConfigurationDataSource>& s)
    { return s->id() == ds_id; } );

    config_data_sources_.erase(ds_it);

    updateDSIdsAll();

    emit dataSourcesChangedSignal();
}

dbContent::ConfigurationDataSource& DataSourceManager::configDataSource (unsigned int ds_id)
{
    traced_assert(hasConfigDataSource(ds_id));

    return *find_if(config_data_sources_.begin(), config_data_sources_.end(),
                    [ds_id] (const std::unique_ptr<dbContent::ConfigurationDataSource>& s)
    { return s->id() == ds_id; } )->get();
}

const std::vector<std::unique_ptr<dbContent::ConfigurationDataSource>>& DataSourceManager::configDataSources() const
{
    return config_data_sources_;
}

void DataSourceManager::checkSubConfigurables()
{

}

void DataSourceManager::loadDBDataSources()
{
    traced_assert(!db_data_sources_.size());

    DBInterface& db_interface = COMPASS::instance().dbInterface();

    if (db_interface.existsDataSourcesTable())
    {
        db_data_sources_ = db_interface.getDataSources();
        sortDBDataSources();
    }

    createConfigDataSourcesFromDB();
}

void DataSourceManager::sortDBDataSources()
{
    sort(db_data_sources_.begin(), db_data_sources_.end(),
         [](const std::unique_ptr<dbContent::DBDataSource>& a,
         const std::unique_ptr<dbContent::DBDataSource>& b) -> bool
    {
        return a->name() < b->name();
    });
}

void DataSourceManager::updateDSIdsAll()
{
    ds_ids_all_.clear();

    std::set<unsigned int> ds_ids_set;

    for (auto& ds_it : config_data_sources_) // add from cfg
    {
        if (!ds_ids_set.count(ds_it->id()))
            ds_ids_set.insert(ds_it->id());
    }

    for (auto& ds_it : db_data_sources_) // add from db
    {
        if (!ds_ids_set.count(ds_it->id()))
            ds_ids_set.insert(ds_it->id());
    }

    std::copy(ds_ids_set.begin(), ds_ids_set.end(), std::back_inserter(ds_ids_all_)); // copy to vec
}

void DataSourceManager::createConfigDataSourcesFromDB()
{
    for (auto& ds_it : db_data_sources_)
    {
        unsigned int ds_id = ds_it->id();

        if (!hasConfigDataSource(ds_id))
        {
            createConfigDataSource(ds_id);

            dbContent::ConfigurationDataSource& cfg_ds = configDataSource(ds_id);

            cfg_ds.dsType(ds_it->dsType());
            cfg_ds.sac(ds_it->sac());
            cfg_ds.sic(ds_it->sic());
            cfg_ds.name(ds_it->name());

            if (ds_it->hasShortName())
                cfg_ds.shortName(ds_it->shortName());

            if (!ds_it->info().is_null())
                cfg_ds.info(ds_it->info().dump());

            loginf << "added name " << cfg_ds.name()
                   << " sac/sic " << cfg_ds.sac() << "/" << cfg_ds.sic();
        }
        else
            logdbg << "ds " << ds_it->name()
                   << " sac/sic " << ds_it->sac() << "/" << ds_it->sic() << " already exists";
    }
}

void DataSourceManager::saveDBDataSources()
{
    loginf;

    DBInterface& db_interface = COMPASS::instance().dbInterface();

    traced_assert(db_interface.ready());
    db_interface.saveDataSources(db_data_sources_);
}

bool DataSourceManager::canAddNewDataSourceFromConfig (unsigned int ds_id)
{
    if (hasDBDataSource(ds_id))
        return false;

    return hasConfigDataSource(ds_id);
}

bool DataSourceManager::hasDBDataSource(unsigned int ds_id)
{
    return find_if(db_data_sources_.begin(), db_data_sources_.end(),
                   [ds_id] (const std::unique_ptr<dbContent::DBDataSource>& s)
    { return s->id() == ds_id; } ) != db_data_sources_.end();
}

bool DataSourceManager::hasDBDataSource(const std::string& ds_name)
{
    return find_if(db_data_sources_.begin(), db_data_sources_.end(),
                   [ds_name] (const std::unique_ptr<dbContent::DBDataSource>& s)
    { return (s->hasShortName() ? s->shortName() == ds_name : false)
                || s->name() == ds_name; } ) != db_data_sources_.end();
}

unsigned int DataSourceManager::getDBDataSourceDSID(const std::string& ds_name)
{
    auto ds_it = find_if(db_data_sources_.begin(), db_data_sources_.end(),
                         [ds_name] (const std::unique_ptr<dbContent::DBDataSource>& s)
    { return (s->hasShortName() ? s->shortName() == ds_name : false)
                || s->name() == ds_name; } );

    traced_assert(ds_it != db_data_sources_.end());

    return (*ds_it)->id();
}

bool DataSourceManager::hasDataSourcesOfDBContent(const std::string dbcontent_name)
{
    return find_if(db_data_sources_.begin(), db_data_sources_.end(),
                   [dbcontent_name] (const std::unique_ptr<dbContent::DBDataSource>& s)
    { return s->numInsertedMap().count(dbcontent_name) > 0; } ) != db_data_sources_.end();
}

void DataSourceManager::addNewDataSource (unsigned int ds_id)
{
    loginf << "ds_id " << ds_id;

    traced_assert(!hasDBDataSource(ds_id));

    if (hasConfigDataSource(ds_id))
    {
        loginf << "ds_id " << ds_id << " from config";

        dbContent::ConfigurationDataSource& cfg_ds = configDataSource(ds_id);

        db_data_sources_.emplace_back(cfg_ds.getAsNewDBDS());
        sortDBDataSources();
    }
    else
    {
        loginf << "ds_id " << ds_id << " create new";

        createConfigDataSource(ds_id);

        dbContent::DBDataSource* new_ds = new dbContent::DBDataSource();
        new_ds->id(ds_id);
        new_ds->sac(Number::sacFromDsId(ds_id));
        new_ds->sic(Number::sicFromDsId(ds_id));
        new_ds->dsType("Other");
        new_ds->name("Unknown ("+to_string(Number::sacFromDsId(ds_id))+"/"
                                +to_string(Number::sicFromDsId(ds_id))+")");

        db_data_sources_.emplace_back(new_ds);
        sortDBDataSources();
    }

    traced_assert(hasDBDataSource(ds_id));
    updateDSIdsAll();

    emit dataSourcesChangedSignal();

    loginf << "ds_id " << ds_id << " done";
}

dbContent::DBDataSource& DataSourceManager::dbDataSource(unsigned int ds_id)
{
    traced_assert(hasDBDataSource(ds_id));

    return *find_if(db_data_sources_.begin(), db_data_sources_.end(),
                    [ds_id] (const std::unique_ptr<dbContent::DBDataSource>& s)
    { return s->id() == ds_id; } )->get();
}

bool DataSourceManager::dsTypeFiltered()
{
    for (auto& ds_type_it : ds_type_loading_wanted_)
        if (!ds_type_it.second)
            return true;

    return false;
}

std::set<std::string> DataSourceManager::wantedDSTypes()
{
    std::set<std::string> ret;

    for (auto& ds_type_it : data_source_types_)
    {
        if (!ds_type_loading_wanted_.count(ds_type_it)) // not in means yes
            ret.insert(ds_type_it);
        else if (ds_type_loading_wanted_.at(ds_type_it))
            ret.insert(ds_type_it);
    }

    return ret;
}

void DataSourceManager::dsTypeLoadingWanted (const std::string& ds_type, bool wanted)
{
    loginf << "ds_type " << ds_type << " wanted " << wanted;

    ds_type_loading_wanted_[ds_type] = wanted;
}

bool DataSourceManager::dsTypeLoadingWanted (const std::string& ds_type)
{
    if (!ds_type_loading_wanted_.count(ds_type))
        return true;

    return ds_type_loading_wanted_.at(ds_type);
}

void DataSourceManager::setLoadDSTypes (bool loading_wanted)
{
    loginf << "wanted " << loading_wanted;

    for (auto& ds_type_it : data_source_types_)
    {
        logdbg << "wanted " << loading_wanted;
        ds_type_loading_wanted_[ds_type_it] = loading_wanted;
    }

    if (load_widget_)
        load_widget_->updateContent();
}

void DataSourceManager::setLoadOnlyDSTypes (std::set<std::string> ds_types)
{
    // deactivate all loading
    setLoadDSTypes(false);

    for (auto ds_type_it : ds_types)
    {
        dsTypeLoadingWanted(ds_type_it, true);
    }

    if (load_widget_)
        load_widget_->updateContent();
}

const std::vector<std::unique_ptr<dbContent::DBDataSource>>& DataSourceManager::dbDataSources() const
{
    return db_data_sources_;
}

std::set<unsigned int> DataSourceManager::groundOnlyDBDataSources() const
{
    std::set<unsigned int> ds_ids;

    for (auto& ds_it : db_data_sources_)
    {
        if (ds_it->detectionType() == DataSourceBase::DetectionType::PrimaryOnlyGround)
            ds_ids.insert(ds_it->id());
    }

    return ds_ids;
}

std::map<unsigned int, dbContent::DataSourceType> DataSourceManager::dsTypes() const
{
    std::map<unsigned int, dbContent::DataSourceType> ret;

    for (auto& ds_it : db_data_sources_)
        ret[ds_it->id()] = DataSourceManager::typeFromString(ds_it->dsType());

    return ret;
}

void DataSourceManager::createNetworkDBDataSources()
{
    unsigned int ds_id;

    for (auto& ds_it : config_data_sources_)
    {
        ds_id = ds_it->id();

        if (ds_it->hasNetworkLines())
        {
            if (!hasDBDataSource(ds_id))
            {
                loginf << "ds_id " << ds_id << " from config";

                db_data_sources_.emplace_back(ds_it->getAsNewDBDS());
                //addNewDataSource(ds_it->id());
            }

            unsigned int line_cnt;
            bool first = true;

            for (auto& line_it : ds_it->networkLines()) // lx -> ip, port
            {
                line_cnt = String::getAppendedInt(line_it.first);
                traced_assert(line_cnt >= 0 && line_cnt <= 4);

                dbDataSource(ds_it->id()).lineLoadingWanted(line_cnt - 1, first); // only load first one
                //dbDataSource(ds_it->id()).addNumInserted()

                first = false;
            }
        }
    }

    sortDBDataSources();
    updateDSIdsAll();

    emit dataSourcesChangedSignal();
}

std::map<unsigned int, std::map<std::string, std::shared_ptr<DataSourceLineInfo>>> DataSourceManager::getNetworkLines() const
{
    // ds_id -> line str ->(ip, port)
    std::map<unsigned int, std::map<std::string, std::shared_ptr<DataSourceLineInfo>>> lines;

    for (auto& ds_it : config_data_sources_)
    {
        if (ds_it->hasNetworkLines())
        {
            lines[ds_it->id()] = ds_it->networkLines();
        }
    }

    //db_data_sources_ // should be same

    return lines;
}
