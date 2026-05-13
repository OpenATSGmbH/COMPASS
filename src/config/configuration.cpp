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

#include "configuration.h"
#include "configurable.h"
#include "logger.h"
#include "stringconv.h"
#include "traced_assert.h"

#include <algorithm>
#include <typeinfo>

using namespace Utils;

using namespace nlohmann;

const std::string Configuration::ParameterName    = "name";
const std::string Configuration::ParameterSection = "parameters";
const std::string Configuration::SubConfigSection = "sub_configs";
const std::string Configuration::InstanceID       = "instance_name";
const std::string Configuration::ClassID          = "class_name";
const std::string Configuration::CLASS_NAME_KEY   = "class_name";
const std::string Configuration::INSTANCE_NAME_KEY = "instance_name";

void Configuration::setClassName(nlohmann::json& j, const std::string& value)
{
    j[CLASS_NAME_KEY] = value;
}

void Configuration::setInstanceName(nlohmann::json& j, const std::string& value)
{
    j[INSTANCE_NAME_KEY] = value;
}

const std::string& Configuration::getClassName(const nlohmann::json& j)
{
    traced_assert(j.contains(CLASS_NAME_KEY));
    return j[CLASS_NAME_KEY].get_ref<const std::string&>();
}

const std::string& Configuration::getInstanceName(const nlohmann::json& j)
{
    traced_assert(j.contains(INSTANCE_NAME_KEY));
    return j[INSTANCE_NAME_KEY].get_ref<const std::string&>();
}

nlohmann::json& Configuration::addSubConfigEntry(nlohmann::json& parent,
                                                  const std::string& class_name,
                                                  const std::string& instance_name)
{
    if (!parent.contains(SubConfigSection))
        parent[SubConfigSection] = json::array();

    auto& arr = parent[SubConfigSection];
    traced_assert(arr.is_array());

    arr.push_back(json::object());
    auto& entry = arr.back();
    setClassName(entry, class_name);
    setInstanceName(entry, instance_name);
    return entry;
}

nlohmann::json* Configuration::findSubConfigEntry(nlohmann::json& parent,
                                                   const std::string& class_name,
                                                   const std::string& instance_name)
{
    if (!parent.contains(SubConfigSection))
        return nullptr;

    auto& arr = parent[SubConfigSection];
    if (!arr.is_array())
        return nullptr;

    for (auto& entry : arr)
    {
        if (entry.contains(CLASS_NAME_KEY) && entry[CLASS_NAME_KEY] == class_name &&
            entry.contains(INSTANCE_NAME_KEY) && entry[INSTANCE_NAME_KEY] == instance_name)
            return &entry;
    }
    return nullptr;
}

const nlohmann::json* Configuration::findSubConfigEntry(const nlohmann::json& parent,
                                                         const std::string& class_name,
                                                         const std::string& instance_name)
{
    if (!parent.contains(SubConfigSection))
        return nullptr;

    const auto& arr = parent[SubConfigSection];
    if (!arr.is_array())
        return nullptr;

    for (const auto& entry : arr)
    {
        if (entry.contains(CLASS_NAME_KEY) && entry[CLASS_NAME_KEY] == class_name &&
            entry.contains(INSTANCE_NAME_KEY) && entry[INSTANCE_NAME_KEY] == instance_name)
            return &entry;
    }
    return nullptr;
}

/**
 * Constructor backed by a json reference.
 * Parses parameters from the backing json. Sub-configs are moved out of the json
 * into sub_config_storage_ (one unique_ptr<json> per child). Old nested format
 * { class_name: { instance_name: config } } is converted to array on the fly.
 */
Configuration::Configuration(nlohmann::json& backing_json)
    : class_name_(getClassName(backing_json)),
      instance_name_(getInstanceName(backing_json)),
      backing_json_(backing_json)
{
    logdbg << "class '" << class_name_ << "' instance '" << instance_name_
           << "' backing_json type=" << backing_json.type_name()
           << " is_null=" << backing_json.is_null()
           << " is_object=" << backing_json.is_object();

    traced_assert(class_name_.size() != 0);
    traced_assert(instance_name_.size() != 0);

    // Parse only the parameters section from backing json
    if (!backing_json.is_null() && backing_json.is_object()
        && backing_json.contains(ParameterSection) && backing_json[ParameterSection].is_object())
    {
        parseJSONParameters(backing_json[ParameterSection]);
    }

    // Move sub-configs from backing json into sub_config_storage_
    if (!backing_json.is_null() && backing_json.is_object()
        && backing_json.contains(SubConfigSection))
    {
        auto& sc = backing_json[SubConfigSection];

        logdbg << "class '" << class_name_ << "' instance '" << instance_name_
               << "' sub_configs type=" << sc.type_name()
               << " size=" << (sc.is_array() ? std::to_string(sc.size())
                              : sc.is_object() ? std::to_string(sc.size()) : "N/A");

        // Convert old nested format to array if needed
        if (sc.is_object() && !sc.empty() && isNestedSubConfigFormat(sc))
        {
            logdbg << "class '" << class_name_ << "' instance '" << instance_name_
                   << "' converting nested sub_configs to array format";

            json arr = json::array();
            for (auto& [cid, instances] : sc.items())
            {
                if (!instances.is_object()) continue;
                for (auto& [iid, child] : instances.items())
                {
                    if (!child.contains(CLASS_NAME_KEY))
                        setClassName(child, cid);
                    if (!child.contains(INSTANCE_NAME_KEY))
                        setInstanceName(child, iid);
                    arr.push_back(std::move(child));
                }
            }
            sc = std::move(arr);
        }

        // Populate storage from array, grouped by class_name
        if (sc.is_array())
        {
            for (auto& entry : sc)
            {
                try
                {
                    const auto& cls = getClassName(entry);
                    logdbg << "class '" << class_name_ << "' instance '" << instance_name_
                           << "' populating sub_config: class_name='" << cls
                           << "' instance_name='" << getInstanceName(entry) << "'";
                    sub_config_storage_[cls].push_back(std::make_unique<json>(std::move(entry)));
                }
                catch (const std::exception& e)
                {
                    logerr << "class '" << class_name_ << "' instance '" << instance_name_
                           << "' failed to populate sub_config entry: " << e.what()
                           << " entry=" << entry.dump(2);
                    throw;
                }
            }
        }

        logdbg << "class '" << class_name_ << "' instance '" << instance_name_
               << "' populated " << sub_config_storage_.size() << " class buckets in sub_config_storage_";

        // Remove sub_configs from backing json - now owned by sub_config_storage_
        backing_json.erase(SubConfigSection);
    }

    logdbg << "class '" << class_name_ << "' instance '" << instance_name_ << "' construction complete";
}

Configuration::~Configuration()
{
    parameters_.clear();
}

/**
 * Generates a unique instance id by scanning sub_config_storage_ for entries of the given class.
 */
std::string Configuration::newInstanceID(const std::string& class_name) const
{
    int instance_number = -1;

    auto it = sub_config_storage_.find(class_name);
    if (it != sub_config_storage_.end())
    {
        for (const auto& ptr : it->second)
        {
            int num = String::getAppendedInt(getInstanceName(*ptr));
            if (num > instance_number)
                instance_number = num;
        }
    }

    instance_number++;
    return class_name + std::to_string(instance_number);
}

void Configuration::resetToDefault()
{
    logdbg << "start" << instance_name_;

    for (auto it = parameters_.begin(); it != parameters_.end(); it++)
        it->second->resetToDefault();
}

bool Configuration::hasParameter(const std::string& parameter_id) const
{
    return (parameters_.count(parameter_id) > 0);
}

template <typename T>
bool Configuration::hasParameterOfType(const std::string& parameter_id) const
{
    auto it = parameters_.find(parameter_id);
    if (it == parameters_.end())
        return false;

    return it->second->isType<T>();
}

/**
 * Checks if a config value for the given parameter_id exists either in the json config or in a registered parameter.
 */
bool Configuration::hasParameterConfigValue(const std::string& parameter_id) const
{
    bool has_param        = hasParameter(parameter_id);
    bool has_param_config = parameterInConfig(parameter_id);

    // exists in org json config and not yet stored in parameters?
    if (has_param_config && !has_param)
        return true;

    return has_param;
}

bool Configuration::parameterInConfig(const std::string& parameter_id) const
{
    return org_config_parameters_.contains(parameter_id);
}

template <typename T>
T Configuration::parameterValueFromConfig(const std::string& parameter_id) const
{
    //check parameterInConfig(parameter_id) beforehand in order to prevent this
    traced_assert(parameterInConfig(parameter_id));

    return ConfigurableParameterT<T>::valueFromJSON(org_config_parameters_.at(parameter_id));
}

template <typename T>
void Configuration::registerParameter(const std::string& parameter_id, T* pointer, const T& default_value)
{
    logdbg << instance_name_ << ": parameter_id " << parameter_id;

    //pointer should be valid
    traced_assert(pointer);

    //parameter not existing yet?
    if (!hasParameter(parameter_id))
    {
        bool in_config = parameterInConfig(parameter_id);

        //register new parameter
        //if no config value is available use the default value
        auto ptr = new ConfigurableParameterT<T>(parameter_id,
                                                 nullptr,
                                                 in_config ? parameterValueFromConfig<T>(parameter_id) : default_value,
                                                 default_value);
        parameters_.insert(std::make_pair(parameter_id, std::unique_ptr<ConfigurableParameter>(ptr)));
    }

    //get existing parameter as type
    ConfigurableParameterT<T>* param = parameters_.at(parameter_id)->as<T>();
    traced_assert(param);

    //update existing parameter
    param->update(pointer, default_value, true);

    used_ = true;
}

template <typename T>
void Configuration::addParameter(const std::string& parameter_id, const T& default_value)
{
    logdbg << "parameter " << parameter_id << " default " << default_value;

    //parameter already existing?
    if (hasParameter(parameter_id))
    {
        //@TODO: maybe assert on this?
        logwrn << instance_name_ << ": " << parameter_id << " already exists";
        return;
    }

    //create new parameter using default value
    auto ptr = new ConfigurableParameterT<T>(parameter_id, nullptr, default_value, default_value);

    parameters_.insert(std::make_pair(parameter_id, std::unique_ptr<ConfigurableParameter>(ptr)));
}

template <typename T>
void Configuration::updateParameterPointer(const std::string& parameter_id, T* pointer)
{
    logdbg << instance_name_ << ": parameter_id " << parameter_id;

    traced_assert(pointer);
    traced_assert(hasParameter(parameter_id));

    //get parameter as type
    auto param = parameters_.at(parameter_id)->as<T>();
    traced_assert(param);

    //update pointer
    param->update(pointer);

    used_ = true;
}

template <typename T>
void Configuration::getParameter(const std::string& parameter_id, T& value) const
{
    //check if parameter exists
    if (!hasParameter(parameter_id))
        throw std::runtime_error("Configuration: getParameter: unknown parameter id " + parameter_id);

    //get parameter as type
    auto param = parameters_.at(parameter_id)->as<T>();
    traced_assert(param);

    //different to ConfigurationParameterT::getParameterValue() we really retrieve the pointer value here,
    //so make sure it is set and throw if this is not the case.
    if (!param->hasStoredPointer())
        throw std::runtime_error("Configuration: getParameter: " + parameter_id + " not in use");

    value = *param->getStoredPointer();
}

/**
 * Retrieves the config value for the given parameter_id, either from the
 * json config or from a stored parameter.
 */
template <typename T>
T Configuration::getParameterConfigValue(const std::string& parameter_id) const
{
    bool has_param        = hasParameter(parameter_id);
    bool has_param_config = parameterInConfig(parameter_id);

    if (has_param_config && !has_param)
    {
        // only exists in org json config and not yet stored in parameters => obtain value from config
        return parameterValueFromConfig<T>(parameter_id);
    }

    traced_assert(has_param);

    //get stored parameter as type
    auto param = parameters_.at(parameter_id)->as<T>();
    traced_assert(param);

    //retrieve config value from registered parameter
    return param->getConfigValue();
}

/**
 * Parses a given json config struct.
 * Only the parameters section is extracted; sub_configs remain in the json tree.
 */
void Configuration::parseJSONConfig(const nlohmann::json& config)
{
    logdbg << "class_name " << class_name_ << " instance_name " << instance_name_;

    auto cb_params     = [this](const nlohmann::json& cfg) { this->parseJSONParameters(cfg); };
    auto cb_subconfigs = [](const nlohmann::json&) { /* sub_configs stay in json, nothing to do */ };

    parseJSONConfig(config, cb_params, cb_subconfigs);
}

/**
 * Parses a given json config struct and invokes section-specific callbacks.
 */
void Configuration::parseJSONConfig(const nlohmann::json& config,
                                    const std::function<void(const nlohmann::json&)>& parse_parameters_cb,
                                    const std::function<void(const nlohmann::json&)>& parse_sub_configs_cb)
{
    traced_assert(config.is_object());

    for (auto& it : config.items())
    {
        if (it.value() == nullptr)  // empty
            continue;

        if (it.key() == ParameterSection)
        {
            traced_assert(it.value().is_object());
            parse_parameters_cb(it.value());
        }
        else if (it.key() == SubConfigSection)
        {
            traced_assert(it.value().is_object() || it.value().is_array());
            parse_sub_configs_cb(it.value());
        }
        else if (it.key() == CLASS_NAME_KEY || it.key() == INSTANCE_NAME_KEY)
        {
            // metadata keys written by Configurable constructor, skip silently
        }
        else
        {
            logwrn << "Configuration class_name " << class_name_ << " instance_name "
                   << instance_name_ << ": parseJSONConfig: ignoring unknown key '" << it.key() << "'";
        }
    }
}

/**
 * Overwrites certain parameter keys of the stored json configuration.
 */
void Configuration::overrideJSONParameters(nlohmann::json& parameters_config)
{
    logdbg << "class_name " << class_name_ << " instance_name " << instance_name_;

    // is object
    traced_assert(parameters_config.is_object());

    // store parameters in local json config
    for (auto& it : parameters_config.items())
    {
        logdbg << "overriding '" << it.key()
               << "' with '" << it.value().dump(0) << "'";

        //overwrite key with new value
        org_config_parameters_[it.key()] = it.value();
    }
}

void Configuration::parseJSONParameters(const nlohmann::json& parameters_config)
{
    logdbg << "class_name " << class_name_ << " instance_name " << instance_name_;

    auto cb = [&](const std::string& key, const nlohmann::json& value)
    {
        traced_assert(!org_config_parameters_.contains(key));
        org_config_parameters_[key] = value;
    };

    parseJSONParameters(parameters_config, cb);
}

void Configuration::parseJSONParameters(const nlohmann::json& parameters_config,
                                        const std::function<void(const std::string&, const nlohmann::json&)>& parse_param_cb)
{
    // is object
    traced_assert(parameters_config.is_object());

    // store parameters in local json config
    for (auto& it : parameters_config.items())
        parse_param_cb(it.key(), it.value());
}

/**
 * Iterates a sub_configs json in either array or legacy nested object format.
 */
void Configuration::iterateSubConfigs(const nlohmann::json& sub_configs_config,
                                      const std::function<void(const SubConfigKey&, const nlohmann::json&)>& cb)
{
    if (sub_configs_config.is_array())
    {
        // New array format: [ { class_name, instance_name, parameters, ... }, ... ]
        for (const auto& entry : sub_configs_config)
        {
            traced_assert(entry.is_object());
            const auto& class_name    = getClassName(entry);
            const auto& instance_name = getInstanceName(entry);
            cb(SubConfigKey(class_name, instance_name), entry);
        }
    }
    else if (sub_configs_config.is_object())
    {
        // Legacy nested format: { class_name: { instance_name: config } }
        for (auto& sub_cfg_class_it : sub_configs_config.items())
        {
            traced_assert(sub_cfg_class_it.value().is_object());
            const std::string& class_name = sub_cfg_class_it.key();

            for (auto& sub_cfg_instance_it : sub_cfg_class_it.value().items())
            {
                traced_assert(sub_cfg_instance_it.value().is_object());
                const std::string& instance_name = sub_cfg_instance_it.key();
                cb(SubConfigKey(class_name, instance_name), sub_cfg_instance_it.value());
            }
        }
    }
}

/**
 * Writes full json config into parent's sub_configs array.
 */
void Configuration::writeJSON(nlohmann::json& parent_json,
                              JSONExportType export_type) const
{
    logdbg << "class_name " << class_name_ << " instance_name " << instance_name_;

    traced_assert(instance_name_.size() != 0);

    json config;
    generateJSON(config, export_type);

    if (!parent_json.contains(SubConfigSection))
        parent_json[SubConfigSection] = json::array();

    parent_json[SubConfigSection].push_back(std::move(config));
}

/**
 * Generates the full json config.
 * Parameters come from the cache; sub_configs come from sub_config_storage_.
 *
 * Important: sub_config entries are shallow copies - nested sub_configs within children
 * are only present if Configurable::writeBackConfigRecursive() was called bottom-up first.
 * Callers must ensure writeBackConfigRecursive() is called before generateJSON() whenever
 * the Configurable tree has runtime-created descendants (e.g. EvaluationStandard requirement groups).
 */
void Configuration::generateJSON(nlohmann::json& target,
                                 JSONExportType export_type) const
{
    logdbg << "class_name " << class_name_ << " instance_name " << instance_name_;

    setClassName(target, class_name_);
    setInstanceName(target, instance_name_);

    json& param_config = target[ParameterSection];

    auto export_filters_classid = jsonExportFilters(export_type, JSONExportFilterType::ClassID);
    auto export_filters_paramid = jsonExportFilters(export_type, JSONExportFilterType::ParamID);

    // original parameters, in case config was not used
    for (auto& par_it : org_config_parameters_.items())
    {
        if (export_filters_paramid && !export_filters_paramid->empty() && export_filters_paramid->count(par_it.key()) > 0)
            continue;

        param_config[par_it.key()] = par_it.value();
    }

    // overwrite new parameter values
    for (auto& par_it : parameters_)
    {
        if (export_filters_paramid && !export_filters_paramid->empty() && export_filters_paramid->count(par_it.second->getParameterId()) > 0)
            continue;

        logdbg << "class_name " << class_name_ << " instance_name " << instance_name_ << ": writing '" << par_it.second->getParameterId() << "'";
        par_it.second->toJSON(param_config);
    }

    // Build sub_configs array from storage, applying class_name filters
    if (!sub_config_storage_.empty())
    {
        target[SubConfigSection] = json::array();

        for (const auto& [cls, entries] : sub_config_storage_)
        {
            if (export_filters_classid && !export_filters_classid->empty()
                && export_filters_classid->count(cls) > 0)
                continue;

            for (const auto& ptr : entries)
                target[SubConfigSection].push_back(*ptr);
        }
    }
}

/**
 * Writes current parameter values back to the backing json's parameters section.
 *
 * Note: only the parameters section is updated. The sub_configs section is left
 * intact because json-backed children hold references into it.
 */
void Configuration::writeBackToJson()
{
    logdbg << "class '" << class_name_ << "' instance '" << instance_name_
           << "' backing_json keys before writeBack [";
    if (backing_json_.is_object())
        for (auto& [key, _] : backing_json_.items())
            logdbg << " " << key;
    logdbg << " ]";

    auto export_filters_paramid = jsonExportFilters(JSONExportType::General, JSONExportFilterType::ParamID);

    json& params = backing_json_[ParameterSection];
    params = json::object();

    // Write original config parameters (for params not yet registered)
    for (auto& par_it : org_config_parameters_.items())
    {
        if (export_filters_paramid && !export_filters_paramid->empty() &&
            export_filters_paramid->count(par_it.key()) > 0)
            continue;

        params[par_it.key()] = par_it.value();
    }

    // Overwrite with current registered parameter values
    for (auto& par_it : parameters_)
    {
        if (export_filters_paramid && !export_filters_paramid->empty() &&
            export_filters_paramid->count(par_it.second->getParameterId()) > 0)
            continue;

        par_it.second->toJSON(params);
    }

    logdbg << "class '" << class_name_ << "' instance '" << instance_name_
           << "' backing_json keys after writeBack [";
    if (backing_json_.is_object())
        for (auto& [key, _] : backing_json_.items())
            logdbg << " " << key;
    logdbg << " ] params_count " << params.size();
}

bool Configuration::hasSubConfiguration(const std::string& class_name, const std::string& instance_name) const
{
    return findSubConfig(class_name, instance_name) != nullptr;
}

bool Configuration::hasSubConfiguration(const SubConfigKey& key) const
{
    return hasSubConfiguration(key.first, key.second);
}

/**
 * Creates a new sub-configuration entry in sub_config_storage_ and returns a stable reference.
 */
nlohmann::json& Configuration::addNewSubConfiguration(const std::string& class_name,
                                                      const std::string& instance_name)
{
    traced_assert(instance_name.size() != 0);
    traced_assert(!hasSubConfiguration(class_name, instance_name));

    auto child = std::make_unique<json>(json::object());
    setClassName(*child, class_name);
    setInstanceName(*child, instance_name);

    auto& bucket = sub_config_storage_[class_name];
    bucket.push_back(std::move(child));
    return *bucket.back();
}

/**
 * Creates a sub-configuration entry with a name parameter.
 */
nlohmann::json& Configuration::addNewSubConfiguration(const std::string& class_name,
                                                      const std::string& instance_name,
                                                      const std::string& name)
{
    auto& child_json = addNewSubConfiguration(class_name, instance_name);
    child_json[ParameterSection][ParameterName] = name;

    return child_json;
}

/**
 * Creates a sub-configuration entry with an auto-generated unique instance id.
 */
nlohmann::json& Configuration::addNewSubConfiguration(const std::string& class_name)
{
    auto instance_name = newInstanceID(class_name);
    traced_assert(instance_name.size() != 0);

    return addNewSubConfiguration(class_name, instance_name);
}

void Configuration::removeSubConfiguration(const std::string& class_name,
                                           const std::string& instance_name)
{
    auto map_it = sub_config_storage_.find(class_name);
    if (map_it != sub_config_storage_.end())
    {
        auto& vec = map_it->second;
        for (auto it = vec.begin(); it != vec.end(); ++it)
        {
            if ((*it)->contains(INSTANCE_NAME_KEY) && (**it)[INSTANCE_NAME_KEY] == instance_name)
            {
                logdbg << "this " << class_name_ << " " << instance_name_ << " other " << class_name << " " << instance_name;

                vec.erase(it);
                if (vec.empty())
                    sub_config_storage_.erase(map_it);
                return;
            }
        }
    }

    logerr << "class_name_ " << class_name_
           << " instance_name_ " << instance_name_ << ": sub class_name " << class_name
           << " instance_name " << instance_name << " not found";
}

void Configuration::removeSubConfigurations(const std::string& class_name)
{
    logdbg << "this " << class_name_;

    sub_config_storage_.erase(class_name);
}

nlohmann::json* Configuration::findSubConfig(const std::string& class_name,
                                             const std::string& instance_name)
{
    auto map_it = sub_config_storage_.find(class_name);
    if (map_it == sub_config_storage_.end())
        return nullptr;

    for (auto& ptr : map_it->second)
    {
        if (ptr->contains(INSTANCE_NAME_KEY) && (*ptr)[INSTANCE_NAME_KEY] == instance_name)
            return ptr.get();
    }
    return nullptr;
}

const nlohmann::json* Configuration::findSubConfig(const std::string& class_name,
                                                    const std::string& instance_name) const
{
    auto map_it = sub_config_storage_.find(class_name);
    if (map_it == sub_config_storage_.end())
        return nullptr;

    for (const auto& ptr : map_it->second)
    {
        if (ptr->contains(INSTANCE_NAME_KEY) && (*ptr)[INSTANCE_NAME_KEY] == instance_name)
            return ptr.get();
    }
    return nullptr;
}

/**
 * Writes sub_config_storage_ back into backing_json_ as a JSON array.
 * Call on children before parents so nested sub_configs are complete.
 */
void Configuration::rebuildSubConfigsToJson()
{
    logdbg << "class '" << class_name_ << "' instance '" << instance_name_
           << "' storage buckets " << sub_config_storage_.size()
           << " backing_json type " << backing_json_.type_name();

    if (sub_config_storage_.empty())
    {
        backing_json_.erase(SubConfigSection);
        logdbg << "class '" << class_name_ << "' instance '" << instance_name_
               << "' no sub_configs to rebuild (storage empty)";
        return;
    }

    size_t total_entries = 0;
    backing_json_[SubConfigSection] = json::array();
    for (const auto& [cls, entries] : sub_config_storage_)
    {
        logdbg << "class '" << class_name_ << "' instance '" << instance_name_
               << "' rebuilding bucket '" << cls << "' with " << entries.size() << " entries";

        for (const auto& ptr : entries)
        {
            try
            {
                bool has_sub = ptr->contains(SubConfigSection);
                size_t sub_count = 0;
                if (has_sub && (*ptr)[SubConfigSection].is_array())
                    sub_count = (*ptr)[SubConfigSection].size();

                std::string iname = ptr->contains(INSTANCE_NAME_KEY) ?
                    (*ptr)[INSTANCE_NAME_KEY].get<std::string>() : "?";

                logdbg << "class '" << class_name_ << "' instance '" << instance_name_
                       << "' rebuilding entry: class '" << cls << "' instance '" << iname
                       << "' has_sub_configs " << has_sub
                       << " sub_count " << sub_count
                       << " keys [";
                for (auto& [key, _] : ptr->items())
                    logdbg << " " << key;
                logdbg << " ]";

                backing_json_[SubConfigSection].push_back(*ptr);
                ++total_entries;
            }
            catch (const std::exception& e)
            {
                logerr << "class '" << class_name_ << "' instance '" << instance_name_
                       << "' failed to rebuild sub_config entry for class '" << cls
                       << "': " << e.what();
                throw;
            }
        }
    }

    logdbg << "class '" << class_name_ << "' instance '" << instance_name_
           << "' rebuilt " << total_entries << " sub_config entries into backing_json";

    // Dump backing_json after rebuild (truncated)
    {
        std::string dumped = backing_json_.dump(2);
        const size_t max_dump = 2000;
        if (dumped.size() > max_dump)
            dumped = dumped.substr(0, max_dump) + "... [truncated, total " + std::to_string(dumped.size()) + " chars]";
        logdbg << "class '" << class_name_ << "' instance '" << instance_name_
               << "' backing_json after rebuild:\n" << dumped;
    }
}

/**
 * Detects old nested sub_configs format { class_name: { instance_name: config } }.
 * In old format, the values of sub_configs are instance maps (no known config keys).
 * In new format, the values ARE configs (have class_name, parameters, etc.).
 */
bool Configuration::isNestedSubConfigFormat(const nlohmann::json& sub_configs)
{
    if (!sub_configs.is_object() || sub_configs.empty())
        return false;

    const auto& first_val = sub_configs.begin().value();
    if (!first_val.is_object())
        return false;

    // If first value has known config keys, it's new (flat) format
    for (auto& [key, _] : first_val.items())
    {
        if (key == CLASS_NAME_KEY || key == INSTANCE_NAME_KEY ||
            key == ParameterSection || key == SubConfigSection)
            return false;
    }

    // No known config keys → old format (keys are instance_names)
    return true;
}

/**
 * Converts old nested sub_configs format to array format, recursively.
 * Safe to call on already-converted json (no-op).
 */
void Configuration::convertSubConfigsFormat(nlohmann::json& node)
{
    if (!node.is_object() || !node.contains(SubConfigSection))
        return;

    auto& sc = node[SubConfigSection];

    // Convert old nested object to array
    if (sc.is_object() && !sc.empty() && isNestedSubConfigFormat(sc))
    {
        json arr = json::array();
        for (auto& [class_name, instances] : sc.items())
        {
            if (!instances.is_object()) continue;
            for (auto& [inst_id, child] : instances.items())
            {
                if (!child.contains(CLASS_NAME_KEY))
                    setClassName(child, class_name);
                if (!child.contains(INSTANCE_NAME_KEY))
                    setInstanceName(child, inst_id);
                arr.push_back(std::move(child));
            }
        }
        sc = std::move(arr);
    }

    // Recurse into children
    if (sc.is_array())
    {
        for (auto& entry : sc)
        {
            if (entry.is_object())
                convertSubConfigsFormat(entry);
        }
    }
}

boost::signals2::connection Configuration::connectListener(const std::function<void(const ParameterList&)>& changed_cb)
{
    return changed_signal_.connect(changed_cb);
}

Configuration::ReconfigureResult Configuration::reconfigure(const nlohmann::json& config,
                                                            Configurable* configurable,
                                                            std::vector<MissingKey>* missing_subconfig_keys,
                                                            std::vector<MissingKey>* missing_param_keys,
                                                            bool assert_on_error)
{
    try
    {
        //run precheck first in order to verify compatibility of passed json config
        auto result_precheck = reconfigure_internal(config,
                                                    configurable,
                                                    missing_subconfig_keys,
                                                    missing_param_keys,
                                                    assert_on_error,
                                                    true);
        if (!result_precheck.first)
            return ReconfigureResult(ReconfigureError::PreCheckFailed, "");

        //check passed => reconfigure
        auto result_apply = reconfigure_internal(config,
                                                 configurable,
                                                 missing_subconfig_keys,
                                                 missing_param_keys,
                                                 assert_on_error,
                                                 false);
        if (!result_apply.first)
            return ReconfigureResult(ReconfigureError::ApplyFailed, "");
    }
    catch(const std::exception& e)
    {
        return ReconfigureResult(ReconfigureError::GeneralError, e.what());
    }
    catch(...)
    {
        return ReconfigureResult(ReconfigureError::UnknownError, "");
    }

    return ReconfigureResult(ReconfigureError::NoError, "");
}

std::pair<bool,std::vector<std::string>> Configuration::reconfigure_internal(const nlohmann::json& config,
                                                                             Configurable* configurable,
                                                                             std::vector<MissingKey>* missing_subconfig_keys,
                                                                             std::vector<MissingKey>* missing_param_keys,
                                                                             bool assert_on_error,
                                                                             bool run_precheck)
{
    logdbg << "class_name " << class_name_ << " instance_name " << instance_name_;

    if (missing_subconfig_keys)
        missing_subconfig_keys->clear();
    if (missing_param_keys)
        missing_param_keys->clear();

    std::set<std::string> param_set;

    auto mode_subconfig = configurable ? configurable->reconfigureSubConfigMode() : MissingKeyMode::MustExist;
    auto mode_param     = configurable ? configurable->reconfigureParameterMode() : MissingKeyMode::MustExist;

    auto logErrorSubConfig = [&](const SubConfigKey& key, bool creation_failed)
    {
        logerr << "sub-config " << key.first << "." << key.second
               << " not found in config " << this->class_name_ << "." << this->instance_name_
               << (creation_failed ? " and could not be created" : "");
    };

    auto logErrorParam = [&](const std::string& name, bool creation_failed)
    {
        logerr << "param " << name
               << " not found in config " << this->class_name_ << "." << this->instance_name_
               << (creation_failed ? " and could not be created" : "");
    };

    auto logWarningSubConfig = [&](const SubConfigKey& key)
    {
        logwrn << "sub-config " << key.first << "." << key.second
               << " not found in config " << this->class_name_ << "." << this->instance_name_ << ", skipping";
    };

    auto logWarningParam = [&](const std::string& name)
    {
        logwrn << "param " << name
               << " not found in config " << this->class_name_ << "." << this->instance_name_ << ", skipping";
    };

    bool subconfigs_ok = true;
    bool params_ok     = true;

    std::string class_name = getClassId();

    // Parameter callback
    auto cb_param = [&](const std::string& key, const nlohmann::json& value)
    {
        if (!hasParameter(key))
        {
            if (mode_param == MissingKeyMode::CreateIfMissing)
            {
                if (run_precheck)
                    return;
            }
            else if (mode_param == MissingKeyMode::SkipIfMissing)
            {
                logWarningParam(key);
                if (missing_param_keys)
                    missing_param_keys->push_back(MissingKey(Key(class_name, key), MissingKeyType::Skipped));
                return;
            }
        }

        bool has_param = hasParameter(key);

        if (!has_param)
        {
            bool creation_failed = (mode_param == MissingKeyMode::CreateIfMissing);
            logErrorParam(key, creation_failed);
            params_ok = false;

            if (missing_param_keys)
                missing_param_keys->push_back(MissingKey(Key(class_name, key), creation_failed ? MissingKeyType::CreationFailed :
                                                                                               MissingKeyType::Missing));
            if (assert_on_error)
                traced_assert(has_param);

            return;
        }

        if (!run_precheck)
        {
            setParameterFromJSON(key, value);
            param_set.insert(key);
        }
    };

    auto cb_params = [&](const nlohmann::json& cfg)
    {
        parseJSONParameters(cfg, cb_param);
    };

    // Sub-config callback
    auto cb_subconfig = [&](const SubConfigKey& key, const nlohmann::json& child_config)
    {
        // Check if child exists as a Configurable
        bool has_child = configurable && configurable->hasSubConfigurable(key.first, key.second);

        if (!has_child)
        {
            if (mode_subconfig == MissingKeyMode::CreateIfMissing && configurable)
            {
                if (run_precheck)
                    return;

                // Create sub-config entry in backing json and generate the child Configurable
                auto& new_child_json = addNewSubConfiguration(key.first, key.second);

                // Copy the incoming config into the new entry
                new_child_json = child_config;

                try
                {
                    configurable->generateSubConfigurable(new_child_json);
                    has_child = true;
                }
                catch(...)
                {
                    removeSubConfiguration(key.first, key.second);
                }
            }
            else if (mode_subconfig == MissingKeyMode::SkipIfMissing)
            {
                logWarningSubConfig(key);
                if (missing_subconfig_keys)
                    missing_subconfig_keys->push_back(MissingKey(key, MissingKeyType::Skipped));
                return;
            }
        }

        if (!has_child)
        {
            bool creation_failed = (mode_subconfig == MissingKeyMode::CreateIfMissing);
            logErrorSubConfig(key, creation_failed);
            subconfigs_ok = false;

            if (missing_subconfig_keys)
                missing_subconfig_keys->push_back(MissingKey(key, creation_failed ? MissingKeyType::CreationFailed :
                                                                                    MissingKeyType::Missing));
            if (assert_on_error)
                traced_assert(has_child);

            return;
        }

        // Recurse into the child's configuration via the Configurable hierarchy
        auto& child_configurable = configurable->getChild(key.first, key.second);
        auto result_subconfig = child_configurable.configuration_->reconfigure_internal(child_config,
                                                                                        &child_configurable,
                                                                                        missing_subconfig_keys,
                                                                                        missing_param_keys,
                                                                                        assert_on_error,
                                                                                        run_precheck);
        if (!run_precheck)
        {
            for (const auto& p : result_subconfig.second)
                param_set.insert(key.first + Configurable::ConfigurablePathSeparator + p);
        }

        if (!result_subconfig.first)
            subconfigs_ok = false;
    };

    auto cb_subconfigs = [&](const nlohmann::json& cfg)
    {
        iterateSubConfigs(cfg, cb_subconfig);
    };

    // Parse config struct using the specified callbacks
    parseJSONConfig(config, cb_params, cb_subconfigs);

    std::vector<std::string> changed_keys;

    if (!run_precheck)
    {
        changed_keys.assign(param_set.begin(), param_set.end());

        if (!changed_keys.empty())
            changed_signal_(changed_keys);
    }

    return std::make_pair(subconfigs_ok && params_ok, changed_keys);
}

void Configuration::setParameterFromJSON(const std::string& parameter_id, const nlohmann::json& value)
{
    //needs to be registered
    traced_assert(hasParameter(parameter_id));

    parameters_.at(parameter_id)->setValue(value);
}

void Configuration::addJSONExportFilter(JSONExportType export_type,
                                        JSONExportFilterType filter_type,
                                        const std::string& id)
{
    if (filter_type == JSONExportFilterType::ClassID)
        json_export_filters_class_name_[ export_type ].insert(id);
    else if (filter_type == JSONExportFilterType::ParamID)
        json_export_filters_param_id_[ export_type ].insert(id);
}

void Configuration::addJSONExportFilter(JSONExportType export_type,
                                        JSONExportFilterType filter_type,
                                        const std::vector<std::string>& ids)
{
    if (filter_type == JSONExportFilterType::ClassID)
        json_export_filters_class_name_[ export_type ].insert(ids.begin(), ids.end());
    else if (filter_type == JSONExportFilterType::ParamID)
        json_export_filters_param_id_[ export_type ].insert(ids.begin(), ids.end());
}

const std::set<std::string>* Configuration::jsonExportFilters(JSONExportType export_type,
                                                              JSONExportFilterType filter_type) const
{
    const std::map<JSONExportType, std::set<std::string>>* f = nullptr;

    if (filter_type == JSONExportFilterType::ClassID)
        f = &json_export_filters_class_name_;
    else if (filter_type == JSONExportFilterType::ParamID)
        f = &json_export_filters_param_id_;

    if (!f)
        return nullptr;

    if (f->empty() || f->count(export_type) == 0)
        return nullptr;

    return &f->at(export_type);
}

template void Configuration::registerParameter<bool>(const std::string& parameter_id, bool* pointer, const bool& default_value);
template void Configuration::registerParameter<int>(const std::string& parameter_id, int* pointer, const int& default_value);
template void Configuration::registerParameter<unsigned int>(const std::string& parameter_id, unsigned int* pointer, const unsigned int& default_value);
template void Configuration::registerParameter<float>(const std::string& parameter_id, float* pointer, const float& default_value);
template void Configuration::registerParameter<double>(const std::string& parameter_id, double* pointer, const double& default_value);
template void Configuration::registerParameter<std::string>(const std::string& parameter_id, std::string* pointer, const std::string& default_value);
template void Configuration::registerParameter<nlohmann::json>(const std::string& parameter_id, nlohmann::json* pointer, const nlohmann::json& default_value);

template void Configuration::addParameter<bool>(const std::string& parameter_id, const bool& default_value);
template void Configuration::addParameter<int>(const std::string& parameter_id, const int& default_value);
template void Configuration::addParameter<unsigned int>(const std::string& parameter_id, const unsigned int& default_value);
template void Configuration::addParameter<float>(const std::string& parameter_id, const float& default_value);
template void Configuration::addParameter<double>(const std::string& parameter_id, const double& default_value);
template void Configuration::addParameter<std::string>(const std::string& parameter_id, const std::string& default_value);
template void Configuration::addParameter<nlohmann::json>(const std::string& parameter_id, const nlohmann::json& default_value);

template void Configuration::updateParameterPointer<bool>(const std::string& parameter_id, bool* pointer);
template void Configuration::updateParameterPointer<int>(const std::string& parameter_id, int* pointer);
template void Configuration::updateParameterPointer<unsigned int>(const std::string& parameter_id, unsigned int* pointer);
template void Configuration::updateParameterPointer<float>(const std::string& parameter_id, float* pointer);
template void Configuration::updateParameterPointer<double>(const std::string& parameter_id, double* pointer);
template void Configuration::updateParameterPointer<std::string>(const std::string& parameter_id, std::string* pointer);
template void Configuration::updateParameterPointer<nlohmann::json>(const std::string& parameter_id, nlohmann::json* pointer);

template void Configuration::getParameter<bool>(const std::string& parameter_id, bool& value) const;
template void Configuration::getParameter<int>(const std::string& parameter_id, int& value) const;
template void Configuration::getParameter<unsigned int>(const std::string& parameter_id, unsigned int& value) const;
template void Configuration::getParameter<float>(const std::string& parameter_id, float& value) const;
template void Configuration::getParameter<double>(const std::string& parameter_id, double& value) const;
template void Configuration::getParameter<std::string>(const std::string& parameter_id, std::string& value) const;
template void Configuration::getParameter<nlohmann::json>(const std::string& parameter_id, nlohmann::json& value) const;

template bool Configuration::hasParameterOfType<bool>(const std::string& parameter_id) const;
template bool Configuration::hasParameterOfType<int>(const std::string& parameter_id) const;
template bool Configuration::hasParameterOfType<unsigned int>(const std::string& parameter_id) const;
template bool Configuration::hasParameterOfType<float>(const std::string& parameter_id) const;
template bool Configuration::hasParameterOfType<double>(const std::string& parameter_id) const;
template bool Configuration::hasParameterOfType<std::string>(const std::string& parameter_id) const;
template bool Configuration::hasParameterOfType<nlohmann::json>(const std::string& parameter_id) const;

template bool Configuration::getParameterConfigValue<bool>(const std::string& parameter_id) const;
template int Configuration::getParameterConfigValue<int>(const std::string& parameter_id) const;
template unsigned int Configuration::getParameterConfigValue<unsigned int>(const std::string& parameter_id) const;
template float Configuration::getParameterConfigValue<float>(const std::string& parameter_id) const;
template double Configuration::getParameterConfigValue<double>(const std::string& parameter_id) const;
template std::string Configuration::getParameterConfigValue<std::string>(const std::string& parameter_id) const;
template nlohmann::json Configuration::getParameterConfigValue<nlohmann::json>(const std::string& parameter_id) const;

template bool Configuration::parameterValueFromConfig<bool>(const std::string& parameter_id) const;
template int Configuration::parameterValueFromConfig<int>(const std::string& parameter_id) const;
template unsigned int Configuration::parameterValueFromConfig<unsigned int>(const std::string& parameter_id) const;
template float Configuration::parameterValueFromConfig<float>(const std::string& parameter_id) const;
template double Configuration::parameterValueFromConfig<double>(const std::string& parameter_id) const;
template std::string Configuration::parameterValueFromConfig<std::string>(const std::string& parameter_id) const;
template nlohmann::json Configuration::parameterValueFromConfig<nlohmann::json>(const std::string& parameter_id) const;
