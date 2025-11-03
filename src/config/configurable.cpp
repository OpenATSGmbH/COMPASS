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
#include "configurationmanager.h"
#include "stringconv.h"
#include "logger.h"
#include "traced_assert.h"

#include <boost/algorithm/string.hpp>

using namespace std;
using namespace Utils;

const char Configurable::ConfigurablePathSeparator = '.';

/**
 * \param class_id Class identifier
 * \param instance_id Instance identifier
 * \param parent Parent, default null (if Singleton)
 * \param configuration_filename special XML configuration filename, default ""
 *
 * Initializes members, adds itself to parent, retrieves filename from parent (if set). Registers
 * itself as child on either the parent or as root configurable, which at the same time sets the
 * configuration reference.
 *
 * \todo Extend registerParameter to template function.
 */
Configurable::Configurable(const std::string& class_id, 
                           const std::string& instance_id,
                           Configurable* parent, 
                           const std::string& root_configuration_filename,
                           const nlohmann::json* config)
    : class_id_    (class_id),
      instance_id_ (instance_id),
      key_id_      (keyID(class_id, instance_id)),
      parent_      (parent),
      is_transient_(config != nullptr)
{
    logdbg << "class_id " << class_id_ << " instance_id " << instance_id_;

    if (config)
    {
        //init config from json => transient configurable
        configuration_ = new Configuration(class_id_, instance_id_);
        configuration_->parseJSONConfig(*config);
    }
    else
    {
        //init from configurable hierarchy
        if (parent)
        {
            parent_ = parent;
            configuration_ = &parent_->registerSubConfigurable(*this);
        }
        else
        {
            is_root_ = true;
            configuration_ = &ConfigurationManager::getInstance().registerRootConfigurable(*this);
        }
    }

    traced_assert(configuration_);

    //connect to configuration to receive changes
    changed_connection_ = configuration_->connectListener([ this ] (const std::vector<std::string>& params) { this->configurationChanged(params); });

    if (root_configuration_filename.size() != 0)
    {
        loginf << "got root filename " << root_configuration_filename;

        configuration_->setConfigurationFilename(root_configuration_filename);
    }

    logdbg << "class_id " << class_id_ << " instance_id " << instance_id_ << " end";
}

/**
 * If parent is set, unregisters from it using removeChildConfigurable, if not, calls
 * unregisterRootConfigurable.
 *
 * Displays warning message if undeleted sub-configurables exist.
 */
Configurable::~Configurable()
{
    logdbg << "class_id " << class_id_ << " instance_id " << instance_id_;

    //@TODO: most likely destroying a connection will disconnect both parties automatically...
    changed_connection_.disconnect();

    if (parent_)
    {
        logdbg << "class_id " << class_id_ << " instance_id "
               << instance_id_ << ": removal from parent";
        
        parent_->removeChildConfigurable(*this, !tmp_disable_remove_config_on_delete_);
        // in case cfg to be used later, see ViewManager::resetToStartupConfiguration
    }

    if (is_root_)
        ConfigurationManager::getInstance().unregisterRootConfigurable(*this);

    if (children_.size() != 0)
    {
        logwrn << "class_id " << class_id_ << " instance_id "
               << instance_id_ << " still " << children_.size() << " undeleted";

        for (auto& child_it : children_)
        {
            logwrn << "class_id " << class_id_ << " instance_id "
                   << instance_id_ << " undelete child ptr " << &child_it.second;
        }
    }

    if (is_transient_ && configuration_)
    {
        //manually delete configuration
        delete configuration_;
    }
}

/**
 * Generates a unique key id from a given class id and instance id.
*/
std::string Configurable::keyID(const std::string& class_id,
                                const std::string& instance_id)
{
    return class_id + instance_id;
}

/**
 * Registers a new parameter in the local config.
*/
template <typename T>
void Configurable::registerParameter(const std::string& parameter_id, T* pointer, const T& default_value)
{
    logdbg << instance_id_ << ": parameter_id " << parameter_id;

    traced_assert(configuration_);
    traced_assert(pointer);

    configuration_->registerParameter<T>(parameter_id, pointer, default_value);
}

/**
 * Returns a parameter config value stored in the local config.
*/
template <typename T>
T Configurable::getParameterConfigValue(const std::string& parameter_id) const
{
    traced_assert(configuration_);
    return configuration_->getParameterConfigValue<T>(parameter_id);
}

/**
 * Sets a parameter and handles notifications.
*/
template <typename T>
void Configurable::setParameter(T& param, const T& value)
{
    if (param == value)
        return;

    param = value;

    notifyModifications();
}

/**
*/
Configuration& Configurable::registerSubConfigurable(Configurable& child, bool config_must_exist)
{
    logdbg << instance_id_ << ": child " << child.instanceId();

    traced_assert(configuration_);

    const std::string& key = child.keyId();

    if (children_.find(key) != children_.end())
    {
        throw std::runtime_error("Configurable: registerSubConfigurable: child key '" + key +
                                 "' already in use");
    }

    logdbg << instance_id_ << ": " << key;

    children_.insert(std::pair<std::string, Configurable&>(key, child));

    if (config_must_exist)
        traced_assert(configuration_->hasSubConfiguration(child.classId(), child.instanceId()));

    return configuration_->getOrCreateSubConfiguration(child.classId(), child.instanceId());
}

/**
*/
void Configurable::removeChildConfigurable(Configurable& child, bool remove_config)
{
    logdbg << instance_id_ << ": child " << child.instanceId();

    traced_assert(configuration_);

    const std::string& key = child.keyId();
    traced_assert(children_.find(key) != children_.end());

    logdbg << instance_id_ << ": " << key;

    children_.erase(children_.find(key));

    if (remove_config)
        configuration_->removeSubConfiguration(child.classId(), child.instanceId());
}

/**
 * Also iteratively calls resetToDefault on all sub-configurables.
 */
void Configurable::resetToDefault()
{
    logdbg << instance_id_;

    traced_assert(configuration_);

    configuration_->resetToDefault();

    std::map<std::string, Configurable&>::iterator it;

    for (it = children_.begin(); it != children_.end(); it++)
    {
        // loginf  << instance_id_ << ": child " << it->first;
        it->second.resetToDefault();
    }
}

/**
*/
Configuration& Configurable::addNewSubConfiguration(const std::string& class_id,
                                                    const std::string& instance_id)
{
    traced_assert(configuration_);
    return configuration_->addNewSubConfiguration(class_id, instance_id);
}

/**
*/
Configuration& Configurable::addNewSubConfiguration(const std::string& class_id)
{
    traced_assert(configuration_);
    return configuration_->addNewSubConfiguration(class_id);
}

/**
*/
Configuration& Configurable::addNewSubConfiguration(std::unique_ptr<Configuration>&& configuration)
{
    traced_assert(configuration_);
    return configuration_->addNewSubConfiguration(std::move(configuration));
}

/**
 * Returns the given subconfiguration or asserts.
 */
const Configuration& Configurable::getSubConfiguration(const std::string& class_id,
                                                       const std::string& instance_id) const
{
    traced_assert(configuration_);
    return configuration_->getSubConfiguration(class_id, instance_id);
}

/**
*/
void Configurable::writeJSON(nlohmann::json& parent_json, JSONExportType export_type) const
{
    traced_assert(configuration_);
    configuration_->writeJSON(parent_json, export_type);
}

/**
*/
void Configurable::generateJSON(nlohmann::json& target, JSONExportType export_type) const
{
    traced_assert(configuration_);
    configuration_->generateJSON(target, export_type);
}

/**
*/
void Configurable::createSubConfigurables()
{
    traced_assert(configuration_);

    logdbg << "config instance " << configuration_->getInstanceId()
           << " configurable instance " << instanceId();

    const auto& sub_configs = configuration_->subConfigurations();

    //create from provided custom creation order first
    auto custom_order = subConfigurableCreationOrder();
    for (const auto& key : custom_order)
    {
        //must be part of sub configurations
        if (sub_configs.count(key) != 0)
        {
            logdbg << "class_id '" << key.first << "' instance_id '" << key.second << "' (custom order)";

            traced_assert(!hasSubConfigurable(key.first, key.second));
            generateSubConfigurable(key.first, key.second);
        }
    }
    
    // TODO what if map changed (deleting of config) during iteration
    for (auto it = sub_configs.begin(); it != sub_configs.end(); it++)
    {
        //not yet created from manual order?
        if (custom_order.empty() || std::find(custom_order.begin(), custom_order.end(), it->first) == custom_order.end())
        {
            logdbg << "class_id '"
                   << it->first.first << "' instance_id '" << it->first.second << "'";

            if (hasSubConfigurable(it->first.first, it->first.second))
                logerr << "class_id '" << it->first.first << "' instance_id '" << it->first.second << "' already exists";

            traced_assert(!hasSubConfigurable(it->first.first, it->first.second));
            generateSubConfigurable(it->first.first, it->first.second);
        }
    }

    //check if every needed subconfigurable has been created yet
    checkSubConfigurables();

    logdbg << "instance " << instance_id_ << " end";
}

/**
*/
void Configurable::checkSubConfigurables()
{
    logerr << "class " << class_id_ << " failed to override me";
}

/**
*/
void Configurable::generateSubConfigurable(const std::string& class_id,
                                           const std::string& instance_id)
{
    loginf << "class " << class_id_ << " does not override ";
}

/**
 * Adds the given config to the local configuration and generates the subconfigurable referenced by it.
*/
void Configurable::generateSubConfigurableFromConfig(std::unique_ptr<Configuration>&& config)
{
    traced_assert(config);

    //note: this might add a unique instance id if yet missing in the passed config
    const auto& added_config = addNewSubConfiguration(std::move(config));

    //get class and instance id from added sub config
    const auto& class_id    = added_config.getClassId();
    const auto& instance_id = added_config.getInstanceId();

    //generate sub configurable
    generateSubConfigurable(class_id, instance_id);
}

/**
 * Shortcut for adding an empty config using the given class and instance id, and generating the subconfigurable referenced by it.
 */
void Configurable::generateSubConfigurableFromConfig(const std::string& class_id,
                                                     const std::string& instance_id)
{
    generateSubConfigurableFromConfig(Configuration::create(class_id, instance_id));
}

/**
 * Generates a subconfigurable by cloning the given configurable via its json interface.
 */
void Configurable::generateSubConfigurableFromJSON(const Configurable& configurable,
                                                   const nlohmann::json& additional_data,
                                                   const std::string& class_id)
{
    //create json config from configurable
    nlohmann::json json_cfg;
    configurable.generateJSON(json_cfg);

    //add external values if provided
    if (!additional_data.is_null())
        json_cfg.update(additional_data);

    auto new_class_id = class_id.empty() ? configurable.classId() : class_id;

    //add new subconfig and parse json
    Configuration& config = addNewSubConfiguration(new_class_id);
    config.parseJSONConfig(json_cfg);

    //generate subconfigurable
    generateSubConfigurable(config.getClassId(), config.getInstanceId());
}

/**
*/
bool Configurable::hasSubConfigurable(const std::string& class_id, 
                                      const std::string& instance_id) const
{
    traced_assert(configuration_);

    return (children_.find(keyID(class_id, instance_id)) != children_.end());
}

/**
*/
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

/**
*/
std::pair<rtcommand::FindObjectErrCode, Configurable*> Configurable::findSubConfigurableName(const std::string& name)
{
    auto child = getApproximateChildNamed(name);
    if (child)
        return {rtcommand::FindObjectErrCode::NoError, child};

    //not found, try in children
    for (auto& c : children_)
    {
        auto res = c.second.findSubConfigurableName(name);
        if (res.first == rtcommand::FindObjectErrCode::NoError)
            return res;
    }

    return {rtcommand::FindObjectErrCode::NotFound, nullptr};
}

/**
*/
Configurable* Configurable::getApproximateChildNamed (const std::string& approx_name)
{
    // find exact instance id
    std::string approx_name_lower = boost::algorithm::to_lower_copy(approx_name);

    auto cb = [approx_name_lower](const std::pair<std::string, Configurable&>& child_iter)
    {
        return (boost::algorithm::to_lower_copy(child_iter.second.instanceId()) == approx_name_lower); 
    };

    auto exact_instance_iter = std::find_if(children_.begin(), children_.end(), cb);

    if (exact_instance_iter != children_.end())
        return &exact_instance_iter->second;

    // check if class_id, take first match

    auto class_id_match_iter = std::find_if(children_.begin(), children_.end(),
                            [approx_name_lower](const pair<std::string, Configurable&>& child_iter) -> bool {
        return boost::algorithm::to_lower_copy(child_iter.second.classId()) == approx_name_lower; });

    if (class_id_match_iter != children_.end())
    {
        loginf << "key_id " << key_id_ << " found approximate name '"
               << approx_name << "' with child instance_id " << class_id_match_iter->second.instanceId();
        return &class_id_match_iter->second;
    }

    return nullptr;
}

/**
*/
const Configurable& Configurable::getChild(const std::string& class_id,
                                           const std::string& instance_id) const
{
    traced_assert(hasSubConfigurable(class_id, instance_id));
    return children_.at(keyID(class_id, instance_id));
}

/**
*/
Configurable& Configurable::getChild(const std::string& class_id,
                                     const std::string& instance_id)
{
    traced_assert(hasSubConfigurable(class_id, instance_id));
    return children_.at(keyID(class_id, instance_id));
}

/**
*/
void Configurable::setTmpDisableRemoveConfigOnDelete(bool value)
{
    logdbg << "value " << value;

    tmp_disable_remove_config_on_delete_ = value;

    for (auto it = children_.begin(); it != children_.end(); it++)
    {
        it->second.setTmpDisableRemoveConfigOnDelete(value);
    }
}

/**
*/
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

/**
 * Reacts on reconfigured configurations.
*/
void Configurable::configurationChanged(const std::vector<std::string>& changed_params)
{
    traced_assert(configuration_);

    //invoke deriveable method for specific behavior (e.g. widget updates)
    onConfigurationChanged(changed_params);
}

/**
 * Returns a unique path that can be used to find the configurable inside the configurable hierarchy.
*/
std::string Configurable::getPath() const
{
    std::string prefix;
    if (parent_)
    {
        auto parent_path = parent_->getPath();
        if (!parent_path.empty())
            prefix = parent_path + ConfigurablePathSeparator;
    }

    return prefix + instance_id_;
}

/**
 * Adds a new filtered class id to the export filter for the given export type.
 */
void Configurable::addJSONExportFilter(JSONExportType export_type, 
                                       JSONExportFilterType filter_type,
                                       const std::string& id)
{
    traced_assert(configuration_);
    configuration_->addJSONExportFilter(export_type, filter_type, id);
}

/**
 * Adds new filtered class ids to the export filter for the given export type.
 */
void Configurable::addJSONExportFilter(JSONExportType export_type, 
                                       JSONExportFilterType filter_type,
                                       const std::vector<std::string>& ids)
{
    traced_assert(configuration_);
    configuration_->addJSONExportFilter(export_type, filter_type, ids);
}

/**
 * Signals changes in the configurable and propagates to its parents.
 */
void Configurable::notifyModifications()
{
    //invoke my own modification callback
    onModified();

    //propagate to parent
    if (parent_)
        parent_->notifyModifications();
}

/**
 * @brief Returns the mode describing how missing subconfigurables will be treated when calling reconfigure().
 * Standard value is MustExist, meaning that reconfigure() will fail on encountering a missing subconfigurable.
 */
Configurable::MissingKeyMode Configurable::reconfigureSubConfigMode() const 
{ 
    return MissingKeyMode::MustExist; 
}

/**
 * @brief Returns the mode describing how missing parameters will be treated when calling reconfigure().
 * Standard value is MustExist, meaning that reconfigure() will fail on encountering a missing parameter.
 */
Configurable::MissingKeyMode Configurable::reconfigureParameterMode() const 
{ 
    return MissingKeyMode::MustExist; 
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
