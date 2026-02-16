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
#include <typeinfo>
#include <vector>
#include <set>
#include <map>
#include <memory>

#include "configurableparameter.h"
#include "json_fwd.hpp"
//#include "string.h"
//#include "logger.h"

#include <boost/signals2.hpp>

/*
 *  @brief Configuration storage and retrieval container class
 *
 *  @details This class is used by a Configurable to store and retrieve parameters to/from an XML
 * configuration system.
 *
 *  Is held by a Configurable or can exist on its own.
 *
 *  \todo Extend registerParameter to template function.
 *  \todo Extend addParameter to template function.
 */
class Configuration
{
public:
    enum class JSONExportType
    {
        General = 0,
        Preset
    };

    enum class JSONExportFilterType
    {
        ClassID = 0,
        ParamID
    };

    enum class ConfigSource
    {
        File = 0,
        Preset
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
    typedef std::unique_ptr<Configuration>            Ptr;
    typedef std::vector<std::string>                  ParameterList;
    typedef std::pair<ReconfigureError, std::string>  ReconfigureResult;
    typedef std::pair<Key, MissingKeyType>            MissingKey;

    struct InstanceDescr
    {
        InstanceDescr() = default;
        InstanceDescr(const std::string& instance_name, 
                      const std::string& instance_id) : name(instance_name), id(instance_id) {}

        std::string name;
        std::string id;
    };

    /// @brief Constructor
    Configuration(const std::string& class_id, 
                  const std::string& instance_id,
                  const std::string& configuration_filename = "");

    /// @brief Destructor
    virtual ~Configuration();

    static Configuration::Ptr create(const std::string& class_id, 
                                     const std::string& instance_id);
    static Configuration::Ptr create(const std::string& class_id, 
                                     const std::string& instance_id,
                                     const std::string& name);
    static Configuration::Ptr create(const std::string& class_id);

    template <typename T>
    void registerParameter(const std::string& parameter_id, T* pointer, const T& default_value);
    template <typename T>
    void addParameter(const std::string& parameter_id, const T& default_value);
    template <typename T>
    void updateParameterPointer(const std::string& parameter_id, T* pointer);
    template <typename T>
    void getParameter(const std::string& parameter_id, T& value) const;

    bool hasParameter(const std::string& parameter_id) const;
    bool hasParameterConfigValue(const std::string& parameter_id) const;

    template <typename T>
    bool hasParameterOfType(const std::string& parameter_id) const;

    template <typename T>
    T getParameterConfigValue(const std::string& parameter_id) const;

    // parses the member config file
    void parseJSONConfigFile();
    void parseJSONConfig(const nlohmann::json& config);
    // writes full json config or sub-file to parent
    void writeJSON(nlohmann::json& parent_json, JSONExportType export_type) const;
    // generates the full json config
    void generateJSON(nlohmann::json& target, JSONExportType export_type) const;

    /// @brief Resets all values to their default values
    void resetToDefault();

    /// @brief Returns flag indicating if configuration has been used by a configurable
    bool getUsed() const { return used_; }

    /// @brief Sets special filename for XML configuration
    void setConfigurationFilename(const std::string& configuration_filename);
    /// @brief Returns flag if special filename has been set
    bool hasConfigurationFilename() const;
    /// @brief Return special filename
    const std::string& getConfigurationFilename() const;

    bool hasSubConfiguration(const std::string& class_id, 
                             const std::string& instance_id) const;
    bool hasSubConfiguration(const SubConfigKey& key) const;

    /// @brief Adds a new sub-configuration and returns reference
    Configuration& addNewSubConfiguration(const std::string& class_id,
                                          const std::string& instance_id);
    /// @brief Adds a new sub-configuration and returns reference
    Configuration& addNewSubConfiguration(const std::string& class_id,
                                          const std::string& instance_id,
                                          const std::string& name);
    /// @brief Adds a new sub-configuration and returns reference
    Configuration& addNewSubConfiguration(const std::string& class_id);
    /// @brief Returns a sub-configuration, creates new empty one if non-existing
    Configuration& addNewSubConfiguration(std::unique_ptr<Configuration>&& configuration);
    /// @brief Returns a specified sub-configuration
    Configuration& getSubConfiguration(const std::string& class_id, const std::string& instance_id) const;
    /// @brief Creates a specified sub-configuration if needed and returns it
    Configuration& getOrCreateSubConfiguration(const std::string& class_id, const std::string& instance_id);
    /// @brief Removes a sub-configuration
    void removeSubConfiguration(const std::string& class_id, const std::string& instance_id);
    void removeSubConfigurations(const std::string& class_id);
    /// @brief Returns all subconfigurations
    const std::map<SubConfigKey, std::unique_ptr<Configuration>>& subConfigurations() const { return sub_configurations_; }

    /// @brief Returns the instance identifier
    const std::string& getInstanceId() const { return instance_id_; }
    /// @brief Returns the class identifier
    const std::string& getClassId() const { return class_id_; }

    /// @brief Sets the template flag and name
    // void setTemplate (bool template_flag, const std::string& template_name_);
    /// @brief Returns the template flag
    // bool getTemplateFlag () const { return template_flag_; }
    /// @brief Returns the template name
    // const std::string& getTemplateName () { return template_name_; }

    /// @brief Checks if a specific template_name is already taken, true if free
    //bool getSubTemplateNameFree(const std::string& template_name) const;
    /// @brief Adds a template configuration with a name
    //void addSubTemplate(Configuration* configuration, const std::string& template_name);

    /// @brief Return contaienr with all configuration templates
    // std::map<std::string, Configuration>& getConfigurationTemplates () { return
    // configuration_templates_; }

    // only use in special case of configuration copy
    void setInstanceId(const std::string& instance_id) { instance_id_ = instance_id; }

    void overrideJSONParameters(nlohmann::json& parameters_config);

    std::string newInstanceID(const std::string& class_id) const;

    boost::signals2::connection connectListener(const std::function<void(const ParameterList&)>& changed_cb);
    ReconfigureResult reconfigure(const nlohmann::json& config, 
                                  Configurable* configurable = nullptr,
                                  std::vector<MissingKey>* missing_subconfig_keys = nullptr,
                                  std::vector<MissingKey>* missing_param_keys = nullptr,
                                  bool assert_on_error = false);

    void addJSONExportFilter(JSONExportType export_type, 
                             JSONExportFilterType filter_type,
                             const std::string& id);
    void addJSONExportFilter(JSONExportType export_type, 
                             JSONExportFilterType filter_type,
                             const std::vector<std::string>& ids);
    const std::set<std::string>* jsonExportFilters(JSONExportType export_type, 
                                                   JSONExportFilterType filter_type) const;

    static const std::string ParameterName;
    static const std::string ParameterSection;
    static const std::string SubConfigSection;
    static const std::string SubConfigFileSection;
    static const std::string InstanceID;
    static const std::string ClassID;
    static const std::string SubConfigFilePath;

protected:
    void parseJSONSubConfigFile(const std::string& class_id, 
                                const std::string& instance_id,
                                const std::string& path);
    void parseJSONParameters(const nlohmann::json& parameters_config);
    void parseJSONSubConfigs(const nlohmann::json& sub_configs_config);

    bool parameterInConfig(const std::string& parameter_id) const;

    template <typename T>
    T parameterValueFromConfig(const std::string& parameter_id) const;

    /// Class identifier
    std::string class_id_;
    /// Instance identifier
    std::string instance_id_;
    /// Flag indicating if configuration has been used by configurable
    bool used_{false};
    /// Special XML configuration filename
    std::string configuration_filename_;

    nlohmann::json org_config_parameters_;
    // nlohmann::json org_config_sub_files_;
    // nlohmann::json org_config_sub_configs_;

    /// Container for all parameters (parameter identifier -> ConfigurableParameter ptr)
    std::map<std::string, std::unique_ptr<ConfigurableParameter>> parameters_;

    /// Container for all added sub-configurables
    std::map<SubConfigKey, std::unique_ptr<Configuration>> sub_configurations_;

    /// Flag which indicates if instance is a template
    // bool template_flag_ {false};
    /// Template name, empty if no template
    // std::string template_name_;

    /// Container with all configuration templates
    // std::map<std::string, Configuration> configuration_templates_;
private:
    Configuration(const std::string& class_id);

    void parseJSONConfig(const nlohmann::json& config,
                         const std::function<void(const nlohmann::json&)>& parse_parameters_cb,
                         const std::function<void(const nlohmann::json&)>& parse_sub_configs_cb,
                         const std::function<void(const std::string&, const std::string&, const std::string&)>& parse_sub_config_files_cb);
    void parseJSONParameters(const nlohmann::json& parameters_config,
                             const std::function<void(const std::string&, const nlohmann::json&)>& parse_param_cb);
    void parseJSONSubConfigs(const nlohmann::json& sub_configs_config,
                             const std::function<void(const SubConfigKey&, const nlohmann::json&)>& parse_subconfig_cb);

    void setParameterFromJSON(const std::string& parameter_id, const nlohmann::json& value);

    std::pair<bool,std::vector<std::string>> reconfigure_internal(const nlohmann::json& config, 
                                                                  Configurable* configurable,
                                                                  std::vector<MissingKey>* missing_subconfig_keys,
                                                                  std::vector<MissingKey>* missing_param_keys,
                                                                  bool assert_on_error,
                                                                  bool run_precheck);
    bool create_instance_name_ = false;

    boost::signals2::signal<void(const ParameterList&)> changed_signal_;

    std::map<JSONExportType, std::set<std::string>> json_export_filters_class_id_;
    std::map<JSONExportType, std::set<std::string>> json_export_filters_param_id_;
};
