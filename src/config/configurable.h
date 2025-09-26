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

#include "traced_assert.h"

#include "configuration.h"
#include "rtcommand_defs.h"

#include <map>
#include <vector>

#include "json_fwd.hpp"

/**
 * @brief Configuration storage and retrieval functionality with sub-class generation
 *
 * @details A Configurable is a super-class that is used for configuration storage and retrieval for
 * any number of parameters. Also can have sub-configurables as children, which are automatically
 * generated at startup.
 *
 * A second constructor exists which should only be used if NO configuration will be saved or used.
 * The idea behind this is that, based on the configuration, sub-configurables are generated, which
 * can be cloned. Since the clones are not to be saved, they can use the simple constructor
 * Configurable (), which disables all Configurable functionality.
 *
 * Holds and uses a Configuration for managing the configuration parameters.
 *
 * If a class is derived from Configurable, it should override the following functions (if required)
 * @code
 * public:
 *   virtual void generateSubConfigurable (std::string class_id, std::string instance_id);
 * protected:
 *   virtual void checkSubConfigurables ();
 * @endcode
 * and call createSubConfigurables() in its constructor. Also, its constructor has to be adapted to
 * supply the required arguments for the Configurable constructor.
 *
 */
class Configurable
{
public:
    typedef Configuration::JSONExportType       JSONExportType;
    typedef Configuration::JSONExportFilterType JSONExportFilterType;
    typedef Configuration::Key                  Key;
    typedef Configuration::SubConfigKey         SubConfigKey;
    typedef Configuration::MissingKeyMode       MissingKeyMode;
    typedef Configuration::MissingKeyType       MissingKeyType;
    typedef Configuration::MissingKey           MissingKey;
    typedef Configuration::ReconfigureError     ReconfigureError;
    typedef Configuration::ReconfigureResult    ReconfigureResult;
    typedef Configuration::InstanceDescr        InstanceDescr;
    
    /// @brief Constructor
    Configurable(const std::string& class_id, 
                 const std::string& instance_id,
                 Configurable* parent = nullptr,
                 const std::string& root_configuration_filename = "",
                 const nlohmann::json* config = nullptr);

    /// @brief Default constructor, for STL containers
    Configurable() = default;

    Configurable(const Configurable&) = delete;
    Configurable(const Configurable&&) = delete;
    Configurable& operator=(const Configurable& other) = delete;
    Configurable& operator=(Configurable&& other) = delete;

    /// @brief Destructor
    virtual ~Configurable();

    bool isTransient() const { return is_transient_; }

    /// @brief Reset parameters to their reset values
    virtual void resetToDefault();

    virtual std::string getPath() const;

    /// @brief Override for creation of sub-configurables
    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);
    void generateSubConfigurableFromConfig(std::unique_ptr<Configuration>&& config);
    void generateSubConfigurableFromConfig(const std::string& class_id,
                                           const std::string& instance_id);
    void generateSubConfigurableFromJSON(const Configurable& configurable,
                                         const nlohmann::json& additional_data = nlohmann::json(),
                                         const std::string& class_id = std::string());
    /// @brief Returns if a specified sub-configurable exists
    bool hasSubConfigurable(const std::string& class_id, const std::string& instance_id) const;
    // finds by approx name, either exact instance id or first matching class id
    std::pair<rtcommand::FindObjectErrCode, Configurable*> findSubConfigurablePath(const std::string& path);
    std::pair<rtcommand::FindObjectErrCode, Configurable*> findSubConfigurableName(const std::string& name);
    // returns nullptr if not found
    Configurable* getApproximateChildNamed(const std::string& approx_name);
    const Configurable& getChild(const std::string& class_id,
                                 const std::string& instance_id) const;
    Configurable& getChild(const std::string& class_id,
                           const std::string& instance_id);

    virtual MissingKeyMode reconfigureSubConfigMode() const;
    virtual MissingKeyMode reconfigureParameterMode() const;

    /// @brief Saves the current configuration as template at its parent
    // void saveConfigurationAsTemplate (const std::string& template_name);

    /// @brief Returns instance identifier
    const std::string& instanceId() const { return instance_id_; }
    /// @brief Returns class identifier
    const std::string& classId() const { return class_id_; }
    /// @brief Returns key identifier (class_id + instance_id)
    const std::string& keyId() const { return key_id_; }

    void setTmpDisableRemoveConfigOnDelete(bool value); // disabled removal of cfg on delete of instance

    void writeJSON(nlohmann::json& parent_json, JSONExportType export_type = JSONExportType::General) const;
    void generateJSON(nlohmann::json& target, JSONExportType export_type = JSONExportType::General) const;

    ReconfigureResult reconfigure(const nlohmann::json& config,
                                  std::vector<MissingKey>* missing_subconfig_keys = nullptr,
                                  std::vector<MissingKey>* missing_param_keys = nullptr,
                                  bool assert_on_error = false,
                                  std::string* error = nullptr);

    static std::string keyID(const std::string& class_id,
                             const std::string& instance_id);

    static const char ConfigurablePathSeparator;

protected:
    /// @brief Creates sub-configurables according to configuration
    void createSubConfigurables();

    /// @brief Override to enforce a custom sub-configurable creation order inside createSubConfigurables()
    virtual std::vector<Configuration::Key> subConfigurableCreationOrder() const { return {}; }

    /// @brief Registers a parameter of given type
    template <typename T>
    void registerParameter(const std::string& parameter_id, T* pointer, const T& default_value);
    template <typename T>
    T getParameterConfigValue(const std::string& parameter_id) const;

    /// @brief Explicitely sets a parameter value
    template <typename T>
    void setParameter(T& param, const T& value);

    /// @brief Override to check if required sub-configurables exist
    virtual void checkSubConfigurables();

    /// @brief Reacts on configuration changes, override as needed
    virtual void onConfigurationChanged(const std::vector<std::string>& changed_params) {}
    
    /// @brief Reacts on modification of the configurable or one of its subconfigurables
    virtual void onModified() {}

    /// @brief Returns the given sub-configuration (e.g. in order to check certain parameter values in generateSubConfigurable())
    const Configuration& getSubConfiguration(const std::string& class_id,
                                             const std::string& instance_id) const;

    void addJSONExportFilter(JSONExportType export_type, 
                             JSONExportFilterType filter_type,
                             const std::string& id);
    void addJSONExportFilter(JSONExportType export_type, 
                             JSONExportFilterType filter_type,
                             const std::vector<std::string>& ids);

    void notifyModifications();

    /// @brief Saves the specified child's configuration as template
    // void saveTemplateConfiguration (Configurable *child, const std::string& template_name);

    /// @brief Returns configuration for this class (const version)
    const Configuration& getConfiguration() const
    {
        traced_assert(configuration_);
        return *configuration_;
    }

    /// @brief Returns the parent configurable for this class (const version)
    const Configurable& getParent() const
    {
        traced_assert(parent_);
        return *parent_;
    }

private:
    void configurationChanged(const std::vector<std::string>& changed_params);

    /// @brief Adds a configurable as a child
    Configuration& registerSubConfigurable(Configurable& child, bool config_must_exist = false);
    /// @brief Removes a child configurable
    void removeChildConfigurable(Configurable& child, bool remove_config = true);

    /// @brief Adds a new sub-configuration based on class id and instance id
    Configuration& addNewSubConfiguration(const std::string& class_id,
                                          const std::string& instance_id);
    /// @brief Adds a new sub-configuration based on class id, instance id is generated
    Configuration& addNewSubConfiguration(const std::string& class_id);
    /// @brief Adds a new sub-configuration by reference and copy constructor
    Configuration& addNewSubConfiguration(std::unique_ptr<Configuration>&& configuration);

    /// @brief Returns configuration for this class
    Configuration& configuration()
    {
        traced_assert(configuration_);
        return *configuration_;
    }

    /// @brief Returns the parent configurable for this class
    Configurable& parent()
    {
        traced_assert(parent_);
        return *parent_;
    }

    void parent(Configurable& parent) { parent_ = &parent; }

    /// Class identifier
    std::string class_id_;
    /// Instance identifier
    std::string instance_id_;
    /// Key identifier
    std::string key_id_;
    /// Parent pointer, null if Singleton or empty
    Configurable* parent_{nullptr};
    /// Configuration
    Configuration* configuration_{nullptr};
    bool is_root_{false};
    bool is_transient_{true};

    bool tmp_disable_remove_config_on_delete_ {false};

    /// Container for all sub-configurables (class id + instance id -> Configurable)
    std::map<std::string, Configurable&> children_;

    boost::signals2::connection changed_connection_;
};
