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

#include "configurable.h"

#include "configuration.h"
#include "stringconv.h"
#include "logger.h"
#include "traced_assert.h"
#include "json.hpp"

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <stdexcept>

using namespace std;
using namespace Utils;

const char Configurable::ConfigurablePathSeparator = '.';

// Legacy constructor removed — all Configurables must now use the json-backed constructor.
// Configurable::Configurable(const std::string& class_name,
//                            const std::string& instance_name,
//                            Configurable* parent,
//                            const std::string& root_configuration_filename,
//                            const nlohmann::json* config)
// { ... }

Configurable::Configurable(nlohmann::json& config_json, Configurable* parent)
    : class_name_(Configuration::getClassName(config_json)),
      instance_name_(Configuration::getInstanceName(config_json)),
      key_id_(keyID(class_name_, instance_name_)),
      parent_(parent),
      path_str_(parent ? (parent->getPath() + ConfigurablePathSeparator + instance_name_) : instance_name_),
      is_transient_(false)
{
    loginf << "class_name '" << class_name_ << "' instance_name '" << instance_name_
           << "' path_str_ '" << path_str_ << "'"
           << " json type=" << config_json.type_name();

    // NOTE: Do NOT call setClassName/setInstanceName here. The keys are already present
    // in config_json (getClassName/getInstanceName above just read them). Calling set*
    // triggers nlohmann::json copy-and-swap which destroys the old string objects,
    // invalidating any const std::string& references the caller obtained via
    // getClassName/getInstanceName before passing config_json to this constructor.

    try
    {
        configuration_ = new Configuration(config_json);
    }
    catch (const std::exception& e)
    {
        logerr << "class_name '" << class_name_ << "' instance_name '" << instance_name_
               << "' Configuration construction failed: " << e.what();
        throw;
    }

    traced_assert(configuration_);

    changed_connection_ = configuration_->connectListener(
        [this](const std::vector<std::string>& params) { this->configurationChanged(params); });

    if (parent_)
        parent_->addChild(this);

    loginf << "class_name '" << class_name_ << "' instance_name '" << instance_name_ << "' construction complete";
}

Configurable::~Configurable()
{
    logdbg << "class_name " << class_name_ << " instance_name " << instance_name_;

    //@TODO: most likely destroying a connection will disconnect both parties automatically...
    changed_connection_.disconnect();

    // Remove from parent's children vector
    if (parent_)
        parent_->removeChild(this);

    // We own the Configuration, delete it
    delete configuration_;
    configuration_ = nullptr;
}

std::string Configurable::keyID(const std::string& class_name,
                                const std::string& instance_name)
{
    return class_name + instance_name;
}

void Configurable::addChild(Configurable* child)
{
    traced_assert(child);
    children_vec_.push_back(child);
}

void Configurable::removeChild(Configurable* child)
{
    auto it = std::find(children_vec_.begin(), children_vec_.end(), child);
    if (it != children_vec_.end())
        children_vec_.erase(it);
}

void Configurable::writeBackConfig()
{
    if (!configuration_)
        return;

    configuration_->writeBackToJson();
}

void Configurable::writeBackConfigRecursive()
{
    loginf << "class '" << class_name_ << "' instance '" << instance_name_
           << "' children=" << children_vec_.size();

    // Children first so nested sub_configs are complete
    for (auto* child : children_vec_)
    {
        if (child)
        {
            try
            {
                child->writeBackConfigRecursive();
            }
            catch (const std::exception& e)
            {
                logerr << "class '" << class_name_ << "' instance '" << instance_name_
                       << "' child '" << child->className() << "/" << child->instanceName()
                       << "' writeBackConfigRecursive failed: " << e.what();
                throw;
            }
        }
    }

    // Write own parameters
    writeBackConfig();

    // Rebuild sub_configs from storage into backing json
    if (configuration_)
    {
        try
        {
            configuration_->rebuildSubConfigsToJson();
        }
        catch (const std::exception& e)
        {
            logerr << "class '" << class_name_ << "' instance '" << instance_name_
                   << "' rebuildSubConfigsToJson failed: " << e.what();
            throw;
        }
    }

    loginf << "class '" << class_name_ << "' instance '" << instance_name_
           << "' writeBackConfigRecursive complete";
}

template <typename T>
void Configurable::registerParameter(const std::string& parameter_id, T* pointer, const T& default_value)
{
    logdbg << instance_name_ << ": parameter_id " << parameter_id;

    traced_assert(configuration_);
    traced_assert(pointer);

    configuration_->registerParameter<T>(parameter_id, pointer, default_value);
}

template <typename T>
T Configurable::getParameterConfigValue(const std::string& parameter_id) const
{
    traced_assert(configuration_);
    return configuration_->getParameterConfigValue<T>(parameter_id);
}

template <typename T>
void Configurable::setParameter(T& param, const T& value)
{
    if (param == value)
        return;

    param = value;

    notifyModifications();
}

void Configurable::resetToDefault()
{
    logdbg << instance_name_;

    traced_assert(configuration_);

    configuration_->resetToDefault();

    for (auto it : children_vec_)
    {
        // loginf  << instance_name_ << ": child " << it->first;
        it->resetToDefault();
    }
}

nlohmann::json& Configurable::addNewSubConfiguration(const std::string& class_name,
                                                     const std::string& instance_name)
{
    traced_assert(configuration_);
    return configuration_->addNewSubConfiguration(class_name, instance_name);
}

nlohmann::json& Configurable::addNewSubConfiguration(const std::string& class_name)
{
    traced_assert(configuration_);
    return configuration_->addNewSubConfiguration(class_name);
}

void Configurable::removeSubConfigurations(const std::string& class_name)
{
    traced_assert(configuration_);
    configuration_->removeSubConfigurations(class_name);
}

nlohmann::json& Configurable::ensureSubConfig(const std::string& class_name,
                                              const std::string& instance_name)
{
    traced_assert(configuration_);

    auto* existing = configuration_->findSubConfig(class_name, instance_name);
    if (existing)
        return *existing;

    return configuration_->addNewSubConfiguration(class_name, instance_name);
}

void Configurable::writeJSON(nlohmann::json& parent_json, JSONExportType export_type) const
{
    traced_assert(configuration_);
    configuration_->writeJSON(parent_json, export_type);
}

void Configurable::generateJSON(nlohmann::json& target, JSONExportType export_type) const
{
    traced_assert(configuration_);
    configuration_->generateJSON(target, export_type);
}

void Configurable::createSubConfigurables()
{
    traced_assert(configuration_);

    loginf << "class '" << class_name_ << "' instance '" << instance_name_
           << "' storage buckets=" << configuration_->subConfigStorage().size();

    for (auto& [class_name, entries] : configuration_->subConfigStorage())
    {
        loginf << "class '" << class_name_ << "' instance '" << instance_name_
               << "' creating sub_configs for class '" << class_name
               << "' (" << entries.size() << " entries)";

        for (auto& ptr : entries)
        {
            const std::string inst_id = Configuration::getInstanceName(*ptr);

            loginf << "class '" << class_name_ << "' instance '" << instance_name_
                   << "' creating: class_name '" << class_name
                   << "' instance_name '" << inst_id << "'";

            traced_assert(!hasSubConfigurable(class_name, inst_id));

            try
            {
                generateSubConfigurable(*ptr);
            }
            catch (const std::exception& e)
            {
                logerr << "class '" << class_name_ << "' instance '" << instance_name_
                       << "' generateSubConfigurable failed for class_name '" << class_name
                       << "' instance_name '" << inst_id << "': " << e.what();
                throw;
            }
        }
    }

    checkSubConfigurables();

    loginf << "class '" << class_name_ << "' instance '" << instance_name_
           << "' createSubConfigurables complete, children=" << children_vec_.size();
}

void Configurable::checkSubConfigurables()
{
}

// Old 2-param version removed — use the json-backed 3-param version
// void Configurable::generateSubConfigurable(const std::string& class_name,
//                                            const std::string& instance_name)
// {
//     loginf << "class " << class_name_ << " does not override ";
// }

void Configurable::generateSubConfigurable(nlohmann::json& child_json)
{
    throw std::runtime_error("class "+class_name_+": generateSubConfigurable: not implemented");
}


void Configurable::generateSubConfigurableFromConfig(const std::string& class_name,
                                                     const std::string& instance_name)
{
    auto& child_json = ensureSubConfig(class_name, instance_name);
    generateSubConfigurable(child_json);
}

void Configurable::generateSubConfigurableFromJSON(const Configurable& configurable,
                                                   const nlohmann::json& additional_data)
{
    traced_assert(configuration_);

    //create json config from configurable
    nlohmann::json json_cfg;
    configurable.generateJSON(json_cfg);

    //add external values if provided
    if (!additional_data.is_null())
        json_cfg.update(additional_data);

    //add new subconfig entry in backing json
    auto& child_json = addNewSubConfiguration(Configuration::getClassName(json_cfg));

    //merge the generated json into the new entry (parameters + sub_configs)
    child_json.update(json_cfg);

    generateSubConfigurable(child_json);
}

bool Configurable::hasSubConfigurable(const std::string& class_name,
                                      const std::string& instance_name) const
{
    traced_assert(configuration_);

    return std::any_of(children_vec_.begin(), children_vec_.end(),
                       [&](const Configurable* c) {
                           return c && c->className() == class_name && c->instanceName() == instance_name;
                       });
}

std::pair<rtcommand::FindObjectErrCode, Configurable*> Configurable::findSubConfigurablePath(const std::string& path)
{
    vector<string> parts = String::split(path, ConfigurablePathSeparator);

    if (!parts.size())
        return {rtcommand::FindObjectErrCode::NotFound, nullptr};

    Configurable* child {this};

    for (const auto& part : parts)
    {
        child = child->getApproximateChildNamed(part);

        if (!child)
            return {rtcommand::FindObjectErrCode::NotFound, nullptr};
    }

    return {rtcommand::FindObjectErrCode::NoError, child};
}

std::pair<rtcommand::FindObjectErrCode, Configurable*> Configurable::findSubConfigurableName(const std::string& name)
{
    auto child = getApproximateChildNamed(name);
    if (child)
        return {rtcommand::FindObjectErrCode::NoError, child};

    //not found, try in children
    for (auto* c : children_vec_)
    {
        if (!c)
            continue;
        auto res = c->findSubConfigurableName(name);
        if (res.first == rtcommand::FindObjectErrCode::NoError)
            return res;
    }

    return {rtcommand::FindObjectErrCode::NotFound, nullptr};
}

Configurable* Configurable::getApproximateChildNamed(const std::string& approx_name)
{
    // find exact instance id
    std::string approx_name_lower = boost::algorithm::to_lower_copy(approx_name);

    for (auto* c : children_vec_)
    {
        if (c && boost::algorithm::to_lower_copy(c->instanceName()) == approx_name_lower)
            return c;
    }

    // check if class_name, take first match
    for (auto* c : children_vec_)
    {
        if (c && boost::algorithm::to_lower_copy(c->className()) == approx_name_lower)
        {
            loginf << "key_id " << key_id_ << " found approximate name '"
                   << approx_name << "' with child instance_name " << c->instanceName();
            return c;
        }
    }

    return nullptr;
}

const Configurable& Configurable::getChild(const std::string& class_name,
                                           const std::string& instance_name) const
{
    for (const auto* c : children_vec_)
    {
        if (c && c->className() == class_name && c->instanceName() == instance_name)
            return *c;
    }
    throw std::runtime_error("Configurable::getChild: child '" + class_name + "/" + instance_name + "' not found");
}

Configurable& Configurable::getChild(const std::string& class_name,
                                     const std::string& instance_name)
{
    for (auto* c : children_vec_)
    {
        if (c && c->className() == class_name && c->instanceName() == instance_name)
            return *c;
    }
    throw std::runtime_error("Configurable::getChild: child '" + class_name + "/" + instance_name + "' not found");
}

void Configurable::setTmpDisableRemoveConfigOnDelete(bool value)
{
    logdbg << "value " << value;

    tmp_disable_remove_config_on_delete_ = value;

    for (auto* c : children_vec_)
    {
        if (c)
            c->setTmpDisableRemoveConfigOnDelete(value);
    }
}

Configurable::ReconfigureResult Configurable::reconfigure(const nlohmann::json& config,
                                                          std::vector<MissingKey>* missing_subconfig_keys,
                                                          std::vector<MissingKey>* missing_param_keys,
                                                          bool assert_on_error,
                                                          std::string* error)
{
    traced_assert(configuration_);
    return configuration_->reconfigure(config,
                                       this, 
                                       missing_subconfig_keys, 
                                       missing_param_keys, 
                                       assert_on_error);
}

void Configurable::configurationChanged(const std::vector<std::string>& changed_params)
{
    traced_assert(configuration_);

    //invoke deriveable method for specific behavior (e.g. widget updates)
    onConfigurationChanged(changed_params);
}

std::string Configurable::getPath() const
{
    return path_str_;
}

void Configurable::addJSONExportFilter(JSONExportType export_type,
                                       JSONExportFilterType filter_type,
                                       const std::string& id)
{
    traced_assert(configuration_);
    configuration_->addJSONExportFilter(export_type, filter_type, id);
}

void Configurable::addJSONExportFilter(JSONExportType export_type,
                                       JSONExportFilterType filter_type,
                                       const std::vector<std::string>& ids)
{
    traced_assert(configuration_);
    configuration_->addJSONExportFilter(export_type, filter_type, ids);
}

void Configurable::notifyModifications()
{
    //invoke my own modification callback
    onModified();

    //propagate to parent
    if (parent_)
        parent_->notifyModifications();
}

Configurable::MissingKeyMode Configurable::reconfigureSubConfigMode() const
{
    return MissingKeyMode::MustExist;
}

Configurable::MissingKeyMode Configurable::reconfigureParameterMode() const
{
    return MissingKeyMode::MustExist;
}

namespace
{
    ResultT<nlohmann::json> jsonFromString(const std::string& str)
    {
        nlohmann::json config;
        try
        {
            config = nlohmann::json::parse(str);

            if (!config.is_object())
                throw std::runtime_error("Configuration not a json object");
        }
        catch(const std::exception& e)
        {
            return ResultT<nlohmann::json>::failed("Could not parse configuration: " + std::string(e.what()));
        }
        catch(...)
        {
            return ResultT<nlohmann::json>::failed("Could not parse configuration: Unknown error");
            return false;
        }

        return ResultT<nlohmann::json>::succeeded(config);
    }
}

Result Configurable::applyJSONSettings(const nlohmann::json& settings_json)
{
    loginf << "CONFIG:\n" << settings_json.dump(4);

    std::string error;
    std::vector<Configuration::MissingKey> missing_subconfig_keys;
    std::vector<Configuration::MissingKey> missing_param_keys;
    auto res = reconfigure(settings_json, &missing_subconfig_keys, &missing_param_keys, false, &error);

    if (res.first == ReconfigureError::NoError)
    {
        loginf << "configuration successfully applied";
        return Result::succeeded();
    }

    if (error.empty())
    {
        if (!res.second.empty())
            error = res.second;
        else
            error = "Unknown error";
    }
    error += " (Code " + std::to_string((int)res.first) + ")\n";

    if (!missing_subconfig_keys.empty())
    {
        error += "missing subconfig keys:\n";
        for (const auto& key : missing_subconfig_keys)
            error += " - " + key.first.first + "." + key.first.second + "\n";
    }

    if (!missing_param_keys.empty())
    {
        error += "missing parameter keys:\n";
        for (const auto& key : missing_param_keys)
            error += " - " + key.first.first + "." + key.first.second + "\n";
    }

    return Result::failed(error);
}

Result Configurable::applyJSONStringSettings(const std::string& settings_json_str)
{
    auto r = jsonFromString(settings_json_str);
    if (!r.ok())
        return Result::failed(r.error());

    traced_assert(r.hasResult());

    loginf << "configuration successfully parsed";

    return applyJSONSettings(r.result());
}

Result Configurable::applyJSONParameters(const nlohmann::json& params_json)
{
    auto wrapper = nlohmann::json::object();
    wrapper[ Configuration::ParameterSection ] = params_json;

    return applyJSONSettings(wrapper);
}

Result Configurable::applyJSONStringParameters(const std::string& params_json_str)
{
    auto r = jsonFromString(params_json_str);
    if (!r.ok())
        return Result::failed(r.error());

    traced_assert(r.hasResult());

    loginf << "configuration successfully parsed";

    return applyJSONParameters(r.result());
}

// void Configurable::saveConfigurationAsTemplate (const std::string& template_name)
//{
//    traced_assert(parent_);
//    parent_->saveTemplateConfiguration(this, template_name);
//}

// void Configurable::saveTemplateConfiguration (Configurable *child, const std::string&
// template_name)
//{
//    traced_assert(configuration_.getSubTemplateNameFree(template_name));
//    configuration_.addSubTemplate(child->getConfiguration().clone(), template_name);
//}

template void Configurable::registerParameter<bool>(const std::string& parameter_id, bool* pointer, const bool& default_value);
template void Configurable::registerParameter<int>(const std::string& parameter_id, int* pointer, const int& default_value);
template void Configurable::registerParameter<unsigned int>(const std::string& parameter_id, unsigned int* pointer, const unsigned int& default_value);
template void Configurable::registerParameter<float>(const std::string& parameter_id, float* pointer, const float& default_value);
template void Configurable::registerParameter<double>(const std::string& parameter_id, double* pointer, const double& default_value);
template void Configurable::registerParameter<std::string>(const std::string& parameter_id, std::string* pointer, const std::string& default_value);
template void Configurable::registerParameter<nlohmann::json>(const std::string& parameter_id, nlohmann::json* pointer, const nlohmann::json& default_value);

template void Configurable::setParameter<bool>(bool& param, const bool& value);
template void Configurable::setParameter<int>(int& param, const int& value);
template void Configurable::setParameter<unsigned int>(unsigned int& param, const unsigned int& value);
template void Configurable::setParameter<float>(float& param, const float& value);
template void Configurable::setParameter<double>(double& param, const double& value);
template void Configurable::setParameter<std::string>(std::string& param, const std::string& value);
template void Configurable::setParameter<nlohmann::json>(nlohmann::json& param, const nlohmann::json& value);

template bool Configurable::getParameterConfigValue<bool>(const std::string& parameter_id) const;
template int Configurable::getParameterConfigValue<int>(const std::string& parameter_id) const;
template unsigned int Configurable::getParameterConfigValue<unsigned int>(const std::string& parameter_id) const;
template float Configurable::getParameterConfigValue<float>(const std::string& parameter_id) const;
template double Configurable::getParameterConfigValue<double>(const std::string& parameter_id) const;
template std::string Configurable::getParameterConfigValue<std::string>(const std::string& parameter_id) const;
template nlohmann::json Configurable::getParameterConfigValue<nlohmann::json>(const std::string& parameter_id) const;
