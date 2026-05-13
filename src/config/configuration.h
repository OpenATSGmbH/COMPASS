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

#include <string>
#include <vector>
#include <set>
#include <map>
#include <memory>

#include "configurableparameter.h"
#include "json_fwd.hpp"

#include <boost/signals2.hpp>

class Configurable;

class Configuration
{
public:
    /// Controls which parameters and sub-configs are included when serializing to JSON.
    /// General: full export. Preset: filtered export for user-facing presets.
    enum class JSONExportType
    {
        General = 0,
        Preset
    };

    /// Selects whether an export filter applies to sub-config class IDs or parameter IDs.
    enum class JSONExportFilterType
    {
        ClassID = 0,
        ParamID
    };

    enum class KeyType
    {
        Parameter = 0,
        SubConfig
    };

    enum class MissingKeyMode
    {
        MustExist = 0,    // key object must exist in config
        CreateIfMissing,  // key object can be created if missing
        SkipIfMissing     // key object can be skipped if missing
    };

    enum class MissingKeyType
    {
        Missing = 0,    // key should have existed but was missing
        Skipped,        // key was skipped
        CreationFailed  // key object should have been created but creation failed
    };

    enum class ReconfigureError
    {
        NoError = 0,
        PreCheckFailed,
        ApplyFailed,
        GeneralError,
        UnknownError
    };

    typedef std::pair<std::string, std::string>       Key;
    typedef Key                                       SubConfigKey;
    typedef std::vector<std::string>                  ParameterList;
    typedef std::pair<ReconfigureError, std::string>  ReconfigureResult;
    typedef std::pair<Key, MissingKeyType>            MissingKey;

    /// Parses parameters and sub-configs from backing_json. Sub-configs are moved into
    /// sub_config_storage_ (owned unique_ptrs); old nested format is converted on the fly.
    Configuration(nlohmann::json& backing_json);

    virtual ~Configuration();

    /// Binds a C++ variable to a config key. If the key exists in the JSON config, the
    /// variable is initialized from it; otherwise default_value is used. The pointer is
    /// stored so the parameter can be written back on save.
    template <typename T>
    void registerParameter(const std::string& parameter_id, T* pointer, const T& default_value);
    /// Adds a parameter entry without binding it to a C++ variable (pointer is null).
    template <typename T>
    void addParameter(const std::string& parameter_id, const T& default_value);
    /// Rebinds an already-registered parameter to a different C++ variable.
    template <typename T>
    void updateParameterPointer(const std::string& parameter_id, T* pointer);
    /// Reads the current value through the stored pointer (requires prior registerParameter).
    template <typename T>
    void getParameter(const std::string& parameter_id, T& value) const;

    bool hasParameter(const std::string& parameter_id) const;
    /// Returns true if a value for parameter_id exists either in the original JSON config
    /// or as a registered parameter - unlike hasParameter which only checks registered ones.
    bool hasParameterConfigValue(const std::string& parameter_id) const;

    template <typename T>
    bool hasParameterOfType(const std::string& parameter_id) const;

    /// Returns the config value for parameter_id: from the original JSON if not yet
    /// registered, or from the registered parameter's stored config value otherwise.
    template <typename T>
    T getParameterConfigValue(const std::string& parameter_id) const;

    void parseJSONConfig(const nlohmann::json& config);
    /// Appends this config as a sub_configs array element in parent_json.
    void writeJSON(nlohmann::json& parent_json, JSONExportType export_type) const;
    /// Builds a standalone JSON object with class_name, instance_name, parameters, and sub_configs.
    /// Sub_configs are shallow copies - call Configurable::writeBackConfigRecursive() first
    /// to ensure nested sub_configs are complete.
    void generateJSON(nlohmann::json& target, JSONExportType export_type) const;

    void resetToDefault();

    /// True once at least one parameter has been registered (i.e. a Configurable has claimed this config).
    bool getUsed() const { return used_; }

    bool hasSubConfiguration(const std::string& class_name,
                             const std::string& instance_name) const;
    bool hasSubConfiguration(const SubConfigKey& key) const;

    /// Creates a new sub-config entry in sub_config_storage_. The returned json& is stable
    /// (unique_ptr indirection) and can be passed to a child Configurable's constructor.
    nlohmann::json& addNewSubConfiguration(const std::string& class_name,
                                           const std::string& instance_name);
    nlohmann::json& addNewSubConfiguration(const std::string& class_name,
                                           const std::string& instance_name,
                                           const std::string& name);
    /// Auto-generates a unique instance_name by appending an incrementing number to class_name.
    nlohmann::json& addNewSubConfiguration(const std::string& class_name);

    void removeSubConfiguration(const std::string& class_name, const std::string& instance_name);
    void removeSubConfigurations(const std::string& class_name);

    nlohmann::json* findSubConfig(const std::string& class_name, const std::string& instance_name);
    const nlohmann::json* findSubConfig(const std::string& class_name, const std::string& instance_name) const;

    /// class_name -> vector of owned json objects, one per child config.
    typedef std::map<std::string, std::vector<std::unique_ptr<nlohmann::json>>> SubConfigStorage;

    const SubConfigStorage& subConfigStorage() const { return sub_config_storage_; }
    SubConfigStorage& subConfigStorage() { return sub_config_storage_; }

    /// Serializes sub_config_storage_ back into backing_json_ as a JSON array.
    /// Must be called bottom-up (children first) so nested sub_configs are complete.
    void rebuildSubConfigsToJson();

    const std::string& getInstanceId() const { return instance_name_; }
    const std::string& getClassId() const { return class_name_; }

    // only use in special case of configuration copy
    void setInstanceId(const std::string& instance_name) { instance_name_ = instance_name; }

    /// Overwrites specific parameter keys in the stored JSON config (org_config_parameters_).
    /// Used to patch config values before parameters are registered.
    void overrideJSONParameters(nlohmann::json& parameters_config);

    /// Generates a unique instance_name for a new child of the given class by finding the
    /// highest existing numeric suffix among siblings and incrementing it.
    std::string newInstanceID(const std::string& class_name) const;

    boost::signals2::connection connectListener(const std::function<void(const ParameterList&)>& changed_cb);

    /// Applies a JSON config to this configuration and its sub-configurations recursively.
    /// Runs a precheck pass first (dry run) to verify compatibility, then applies.
    /// Collects missing keys into the optional output vectors. Fires changed_signal_ on success.
    ReconfigureResult reconfigure(const nlohmann::json& config,
                                  Configurable* configurable = nullptr,
                                  std::vector<MissingKey>* missing_subconfig_keys = nullptr,
                                  std::vector<MissingKey>* missing_param_keys = nullptr,
                                  bool assert_on_error = false);

    /// Excludes specific class IDs or parameter IDs from JSON export for a given export type.
    void addJSONExportFilter(JSONExportType export_type,
                             JSONExportFilterType filter_type,
                             const std::string& id);
    void addJSONExportFilter(JSONExportType export_type,
                             JSONExportFilterType filter_type,
                             const std::vector<std::string>& ids);
    /// Returns the set of excluded IDs for the given export/filter type, or nullptr if none.
    const std::set<std::string>* jsonExportFilters(JSONExportType export_type,
                                                   JSONExportFilterType filter_type) const;

    /// Writes current registered parameter values back to backing_json_'s parameters section.
    void writeBackToJson();
    nlohmann::json& backingJson() { return backing_json_; }
    const nlohmann::json& backingJson() const { return backing_json_; }

    static void setClassName(nlohmann::json& j, const std::string& value);
    static void setInstanceName(nlohmann::json& j, const std::string& value);
    static const std::string& getClassName(const nlohmann::json& j);
    static const std::string& getInstanceName(const nlohmann::json& j);

    /// Raw JSON helpers that operate on any json node's sub_configs array
    /// (as opposed to the instance methods that use sub_config_storage_).
    static nlohmann::json& addSubConfigEntry(nlohmann::json& parent,
                                             const std::string& class_name,
                                             const std::string& instance_name);
    static nlohmann::json* findSubConfigEntry(nlohmann::json& parent,
                                              const std::string& class_name,
                                              const std::string& instance_name);
    static const nlohmann::json* findSubConfigEntry(const nlohmann::json& parent,
                                                    const std::string& class_name,
                                                    const std::string& instance_name);

    /// Converts old nested sub_configs format { class_name: { instance_name: config } }
    /// to flat array format, recursively. No-op if already converted.
    static void convertSubConfigsFormat(nlohmann::json& node);

    static const std::string ParameterName;
    static const std::string ParameterSection;
    static const std::string SubConfigSection;
    static const std::string InstanceID;
    static const std::string ClassID;
    static const std::string CLASS_NAME_KEY;
    static const std::string INSTANCE_NAME_KEY;

protected:
    void parseJSONParameters(const nlohmann::json& parameters_config);
    bool parameterInConfig(const std::string& parameter_id) const;

    template <typename T>
    T parameterValueFromConfig(const std::string& parameter_id) const;

    std::string class_name_;
    std::string instance_name_;
    bool used_{false};

    /// Snapshot of the parameter values as parsed from the original JSON config file.
    /// Retained so that unregistered parameters are preserved on save (round-trip fidelity).
    nlohmann::json org_config_parameters_;

    std::map<std::string, std::unique_ptr<ConfigurableParameter>> parameters_;

private:
    /// Callback-based config parser: dispatches parameters and sub_configs sections to callbacks.
    void parseJSONConfig(const nlohmann::json& config,
                         const std::function<void(const nlohmann::json&)>& parse_parameters_cb,
                         const std::function<void(const nlohmann::json&)>& parse_sub_configs_cb);
    void parseJSONParameters(const nlohmann::json& parameters_config,
                             const std::function<void(const std::string&, const nlohmann::json&)>& parse_param_cb);

    /// Handles both flat array and legacy nested object sub_configs formats transparently.
    static void iterateSubConfigs(const nlohmann::json& sub_configs_config,
                                  const std::function<void(const SubConfigKey&, const nlohmann::json&)>& cb);
    static bool isNestedSubConfigFormat(const nlohmann::json& sub_configs);

    void setParameterFromJSON(const std::string& parameter_id, const nlohmann::json& value);

    /// Single pass of reconfigure: when run_precheck=true, validates that all keys exist
    /// without modifying state; when false, applies values and fires change notifications.
    /// Returns {success, list_of_changed_parameter_paths}.
    std::pair<bool,std::vector<std::string>> reconfigure_internal(const nlohmann::json& config,
                                                                  Configurable* configurable,
                                                                  std::vector<MissingKey>* missing_subconfig_keys,
                                                                  std::vector<MissingKey>* missing_param_keys,
                                                                  bool assert_on_error,
                                                                  bool run_precheck);
    /// Reference to the json node that owns this config (e.g. an element of a parent's
    /// sub_configs array, or a root ConfigJSON). At runtime only parameters, class_name, and
    /// instance_name live here - sub_configs are moved to sub_config_storage_ at construction
    /// and rebuilt into this node on save via rebuildSubConfigsToJson().
    nlohmann::json& backing_json_;

    /// Owns each child sub-config json, grouped by class_name. References handed out by
    /// addNewSubConfiguration() are stable thanks to unique_ptr indirection.
    SubConfigStorage sub_config_storage_;

    boost::signals2::signal<void(const ParameterList&)> changed_signal_;

    /// IDs excluded from JSON export, keyed by export type.
    std::map<JSONExportType, std::set<std::string>> json_export_filters_class_name_;
    std::map<JSONExportType, std::set<std::string>> json_export_filters_param_id_;
};
