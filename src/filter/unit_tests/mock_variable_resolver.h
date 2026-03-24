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

#include "idbvariableresolver.h"
#include "configuration.h"
#include "property.h"
#include "dbcontent/variable/variableset.h"
#include "json.hpp"
#include "dbcontent/dbcontent.h"

#include <map>
#include <set>
#include <string>
#include <vector>
#include <stdexcept>

class MockVariableResolver : public IDBVariableResolver
{
public:
    // --- Registration API ---

    void addMetaVariable(const Property& prop,
                         const std::map<std::string, std::string>& dbcontent_to_column,
                         const std::map<std::string, std::string>& dbcontent_to_varname = {})
    {
        MetaVarInfo info;
        info.dbcontent_to_column = dbcontent_to_column;
        info.dbcontent_to_varname = dbcontent_to_varname;
        info.data_type = prop.dataType();
        meta_vars_[prop.name()] = info;

        for (auto& p : dbcontent_to_column)
            known_dbcontents_.insert(p.first);
    }

    void addDirectVariable(const std::string& dbcontent_name, const Property& prop,
                           const std::string& db_column,
                           const std::string& var_name = "")
    {
        DirectVarInfo info;
        info.db_column = db_column;
        info.var_name = var_name.empty() ? prop.name() : var_name;
        info.data_type = prop.dataType();
        direct_vars_[{dbcontent_name, prop.name()}] = info;
        known_dbcontents_.insert(dbcontent_name);
    }

    // --- Property-based resolution ---

    bool metaCanGetVariable(const std::string& dbcontent_name,
                            const Property& meta_var) const override
    {
        auto it = meta_vars_.find(meta_var.name());
        if (it == meta_vars_.end()) return false;
        return it->second.dbcontent_to_column.count(dbcontent_name) > 0;
    }

    std::string metaGetVariableDBColumn(const std::string& dbcontent_name,
                                        const Property& meta_var) const override
    {
        return meta_vars_.at(meta_var.name()).dbcontent_to_column.at(dbcontent_name);
    }

    std::string metaGetVariableName(const std::string& dbcontent_name,
                                    const Property& meta_var) const override
    {
        auto& info = meta_vars_.at(meta_var.name());
        auto it = info.dbcontent_to_varname.find(dbcontent_name);
        if (it != info.dbcontent_to_varname.end())
            return it->second;
        return meta_var.name();
    }

    bool canGetVariable(const std::string& dbcontent_name,
                        const Property& var) const override
    {
        return direct_vars_.count({dbcontent_name, var.name()}) > 0;
    }

    std::string getVariableDBColumn(const std::string& dbcontent_name,
                                    const Property& var) const override
    {
        return direct_vars_.at({dbcontent_name, var.name()}).db_column;
    }

    std::string getVariableName(const std::string& dbcontent_name,
                                const Property& var) const override
    {
        auto it = direct_vars_.find({dbcontent_name, var.name()});
        if (it != direct_vars_.end())
            return it->second.var_name;
        return var.name();
    }

    bool variableHasDBContent(const std::string& dbcontent_name,
                              const Property& var) const override
    {
        return canGetVariable(dbcontent_name, var);
    }

    // --- Name-based resolution ---

    bool existsMetaVariable(const std::string& var_name) const override
    {
        return meta_vars_.count(var_name) > 0;
    }

    bool metaVariableExistsIn(const std::string& var_name,
                              const std::string& dbcontent_name) const override
    {
        auto it = meta_vars_.find(var_name);
        if (it == meta_vars_.end()) return false;
        return it->second.dbcontent_to_column.count(dbcontent_name) > 0;
    }

    bool existsDBContent(const std::string& dbcontent_name) const override
    {
        return known_dbcontents_.count(dbcontent_name) > 0;
    }

    bool dbContentHasVariable(const std::string& dbcontent_name,
                              const std::string& var_name) const override
    {
        // check direct vars
        for (auto& kv : direct_vars_)
        {
            if (kv.first.first == dbcontent_name && kv.second.var_name == var_name)
                return true;
        }
        // check meta vars
        auto it = meta_vars_.find(var_name);
        if (it != meta_vars_.end())
            return it->second.dbcontent_to_column.count(dbcontent_name) > 0;

        return false;
    }

    std::vector<std::string> metaVariableDBContentNames(
        const std::string& var_name) const override
    {
        std::vector<std::string> result;
        auto it = meta_vars_.find(var_name);
        if (it != meta_vars_.end())
        {
            for (auto& p : it->second.dbcontent_to_column)
                result.push_back(p.first);
        }
        return result;
    }

    std::string variableDBColumnName(const std::string& dbcontent_name,
                                     const std::string& var_name,
                                     const std::string& var_dbcontent_name) const override
    {
        if (var_dbcontent_name == "Meta")
        {
            auto it = meta_vars_.find(var_name);
            if (it != meta_vars_.end())
                return it->second.dbcontent_to_column.at(dbcontent_name);
        }
        // direct: find by var_name
        for (auto& kv : direct_vars_)
        {
            if (kv.first.first == var_dbcontent_name && kv.second.var_name == var_name)
                return kv.second.db_column;
        }
        throw std::runtime_error("MockVariableResolver: variableDBColumnName not found for "
                                 + var_name + " in " + dbcontent_name);
    }

    std::string variableDBTableName(const std::string& dbcontent_name,
                                    const std::string& var_name,
                                    const std::string& var_dbcontent_name) const override
    {
        return dbcontent_name;
    }

    std::string variableDBExpression(const std::string& dbcontent_name,
                                     const std::string& var_name,
                                     const std::string& var_dbcontent_name) const override
    {
        return variableDBColumnName(dbcontent_name, var_name, var_dbcontent_name);
    }

    PropertyDataType variableDataType(const std::string& dbcontent_name,
                                      const std::string& var_name,
                                      const std::string& var_dbcontent_name) const override
    {
        if (var_dbcontent_name == "Meta")
        {
            auto it = meta_vars_.find(var_name);
            if (it != meta_vars_.end())
                return it->second.data_type;
        }
        for (auto& kv : direct_vars_)
        {
            if (kv.first.first == var_dbcontent_name && kv.second.var_name == var_name)
                return kv.second.data_type;
        }
        return PropertyDataType::STRING;
    }

    bool variableHasNonStandardRepresentation(const std::string& dbcontent_name,
                                              const std::string& var_name,
                                              const std::string& var_dbcontent_name) const override
    {
        return false;
    }

    std::string variableValueFromRepresentation(const std::string& dbcontent_name,
                                                const std::string& var_name,
                                                const std::string& var_dbcontent_name,
                                                const std::string& value_str) const override
    {
        return value_str;
    }

    bool readSetHasVariable(const std::string& dbcontent_name,
                            const std::string& var_name,
                            const std::string& var_dbcontent_name,
                            const dbContent::VariableSet& read_set) const override
    {
        return false;
    }

    void addVariableToReadSet(const std::string& dbcontent_name,
                              const std::string& var_name,
                              const std::string& var_dbcontent_name,
                              dbContent::VariableSet& read_set) const override
    {
        // no-op
    }

private:
    struct MetaVarInfo {
        std::map<std::string, std::string> dbcontent_to_column;
        std::map<std::string, std::string> dbcontent_to_varname;
        PropertyDataType data_type{PropertyDataType::STRING};
    };

    struct DirectVarInfo {
        std::string db_column;
        std::string var_name;
        PropertyDataType data_type{PropertyDataType::STRING};
    };

    std::map<std::string, MetaVarInfo> meta_vars_;
    std::map<std::pair<std::string, std::string>, DirectVarInfo> direct_vars_;
    std::set<std::string> known_dbcontents_;
};

// --- Config helper ---

inline nlohmann::json makeFilterConfig(const std::string& class_name,
                                       const std::string& instance_name,
                                       nlohmann::json params = nlohmann::json::object())
{
    nlohmann::json cfg;
    cfg[Configuration::CLASS_NAME_KEY]    = class_name;
    cfg[Configuration::INSTANCE_NAME_KEY] = instance_name;
    if (!params.empty())
        cfg["parameters"] = params;
    return cfg;
}

// --- Standard mock pre-populated with all variables needed by the filter set ---

inline MockVariableResolver createStandardMock()
{
    MockVariableResolver mock;

    // Timestamp
    mock.addMetaVariable(DBContent::meta_var_timestamp_,
        {{"CAT001", "timestamp"}, {"CAT010", "timestamp"}, {"CAT020", "timestamp"},
         {"CAT021", "timestamp"}, {"CAT048", "timestamp"}, {"CAT062", "timestamp"},
         {"RefTraj", "timestamp"}});

    // Mode 3/A
    mock.addMetaVariable(DBContent::meta_var_m3a_,
        {{"CAT001", "mode3a_code"}, {"CAT010", "mode3a_code"}, {"CAT020", "mode3a_code"},
         {"CAT021", "mode3a_code"}, {"CAT048", "mode3a_code"}, {"CAT062", "mode3a_code"},
         {"RefTraj", "mode3a_code"}},
        {{"CAT001", "Mode 3/A Code"}, {"CAT010", "Mode 3/A Code"}, {"CAT020", "Mode 3/A Code"},
         {"CAT021", "Mode 3/A Code"}, {"CAT048", "Mode 3/A Code"}, {"CAT062", "Mode 3/A Code"},
         {"RefTraj", "Mode 3/A Code"}});

    // Mode C
    mock.addMetaVariable(DBContent::meta_var_mc_,
        {{"CAT001", "modec_code_ft"}, {"CAT010", "modec_code_ft"}, {"CAT020", "modec_code_ft"},
         {"CAT021", "modec_code_ft"}, {"CAT048", "modec_code_ft"}, {"CAT062", "modec_code_ft"},
         {"RefTraj", "modec_code_ft"}},
        {{"CAT001", "Mode C Code"}, {"CAT010", "Mode C Code"}, {"CAT020", "Mode C Code"},
         {"CAT021", "Mode C Code"}, {"CAT048", "Mode C Code"}, {"CAT062", "Mode C Code"},
         {"RefTraj", "Mode C Code"}});

    // Aircraft Address (ACAD)
    mock.addMetaVariable(DBContent::meta_var_acad_,
        {{"CAT010", "target_addr"}, {"CAT020", "target_addr"}, {"CAT021", "target_addr"},
         {"CAT048", "target_addr"}, {"CAT062", "target_addr"}, {"RefTraj", "target_addr"}},
        {{"CAT010", "Aircraft Address"}, {"CAT020", "Aircraft Address"}, {"CAT021", "Aircraft Address"},
         {"CAT048", "Aircraft Address"}, {"CAT062", "Aircraft Address"}, {"RefTraj", "Aircraft Address"}});

    // Aircraft Identification (ACID)
    mock.addMetaVariable(DBContent::meta_var_acid_,
        {{"CAT010", "target_id"}, {"CAT020", "target_id"}, {"CAT021", "target_id"},
         {"CAT048", "target_id"}, {"CAT062", "target_id"}, {"RefTraj", "target_id"}},
        {{"CAT010", "Aircraft Identification"}, {"CAT020", "Aircraft Identification"},
         {"CAT021", "Aircraft Identification"}, {"CAT048", "Aircraft Identification"},
         {"CAT062", "Aircraft Identification"}, {"RefTraj", "Aircraft Identification"}});

    // UTN
    mock.addMetaVariable(DBContent::meta_var_utn_,
        {{"CAT001", "utn"}, {"CAT010", "utn"}, {"CAT020", "utn"}, {"CAT021", "utn"},
         {"CAT048", "utn"}, {"CAT062", "utn"}, {"RefTraj", "utn"}});

    // DS ID
    mock.addMetaVariable(DBContent::meta_var_ds_id_,
        {{"CAT001", "ds_id"}, {"CAT010", "ds_id"}, {"CAT020", "ds_id"}, {"CAT021", "ds_id"},
         {"CAT048", "ds_id"}, {"CAT062", "ds_id"}, {"RefTraj", "ds_id"}});

    // Line ID
    mock.addMetaVariable(DBContent::meta_var_line_id_,
        {{"CAT001", "line_id"}, {"CAT010", "line_id"}, {"CAT020", "line_id"}, {"CAT021", "line_id"},
         {"CAT048", "line_id"}, {"CAT062", "line_id"}, {"RefTraj", "line_id"}});

    // Track Number
    mock.addMetaVariable(DBContent::meta_var_track_num_,
        {{"CAT001", "track_num"}, {"CAT010", "track_num"}, {"CAT020", "track_num"},
         {"CAT021", "track_num"}, {"CAT048", "track_num"}, {"CAT062", "track_num"},
         {"RefTraj", "track_num"}},
        {{"CAT001", "Track Number"}, {"CAT010", "Track Number"}, {"CAT020", "Track Number"},
         {"CAT021", "Track Number"}, {"CAT048", "Track Number"}, {"CAT062", "Track Number"},
         {"RefTraj", "Track Number"}});

    // Detection Type
    mock.addMetaVariable(DBContent::meta_var_detection_type_,
        {{"CAT001", "detection_type"}, {"CAT048", "detection_type"}, {"CAT062", "detection_type"}},
        {{"CAT001", "Type"}, {"CAT048", "Type"}, {"CAT062", "Type"}});

    // Ground Bit
    mock.addMetaVariable(DBContent::meta_var_ground_bit_,
        {{"CAT010", "ground_bit"}, {"CAT020", "ground_bit"}, {"CAT021", "ground_bit"},
         {"CAT062", "ground_bit"}, {"RefTraj", "ground_bit"}});

    // X/Y StdDev
    mock.addMetaVariable(DBContent::meta_var_x_stddev_,
        {{"RefTraj", "x_stddev"}, {"CAT062", "x_stddev"}});
    mock.addMetaVariable(DBContent::meta_var_y_stddev_,
        {{"RefTraj", "y_stddev"}, {"CAT062", "y_stddev"}});

    // Latitude / Longitude
    mock.addMetaVariable(DBContent::meta_var_latitude_,
        {{"CAT001", "latitude"}, {"CAT010", "latitude"}, {"CAT020", "latitude"},
         {"CAT021", "latitude"}, {"CAT048", "latitude"}, {"CAT062", "latitude"},
         {"RefTraj", "latitude"}});
    mock.addMetaVariable(DBContent::meta_var_longitude_,
        {{"CAT001", "longitude"}, {"CAT010", "longitude"}, {"CAT020", "longitude"},
         {"CAT021", "longitude"}, {"CAT048", "longitude"}, {"CAT062", "longitude"},
         {"RefTraj", "longitude"}});

    // Time of Day
    mock.addMetaVariable(DBContent::meta_var_time_of_day_,
        {{"CAT001", "tod"}, {"CAT010", "tod"}, {"CAT020", "tod"}, {"CAT021", "tod"},
         {"CAT048", "tod"}, {"CAT062", "tod"}, {"RefTraj", "tod"}});

    // --- Direct variables ---

    // CAT021 ADS-B quality
    mock.addDirectVariable("CAT021", DBContent::var_cat021_mops_version_, "mops_version");
    mock.addDirectVariable("CAT021", DBContent::var_cat021_nacp_, "nacp");
    mock.addDirectVariable("CAT021", DBContent::var_cat021_nucp_nic_, "nucp_nic");
    mock.addDirectVariable("CAT021", DBContent::var_cat021_sil_, "sil");

    // CAT020 MLAT contrib receivers
    mock.addDirectVariable("CAT020", DBContent::var_cat020_contrib_recv_, "contrib_receivers");

    // CAT062 direct variables
    mock.addDirectVariable("CAT062", DBContent::var_cat062_baro_alt_, "baro_alt");
    mock.addDirectVariable("CAT062", DBContent::var_cat062_fl_measured_, "fl_measured");
    mock.addDirectVariable("CAT062", DBContent::var_cat062_callsign_fpl_, "callsign_fpl",
                           "Callsign FPL");

    return mock;
}
