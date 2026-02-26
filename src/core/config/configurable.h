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
#include "result.h"

#include <map>
#include <vector>

#include "json_fwd.hpp"

class Configurable
{
    friend class Configuration;  // Configuration::reconfigure_internal needs access to configuration_

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

    /// Creates a Configuration from config_json, registers with parent (if non-null)
    /// or with ConfigurationManager as a root configurable.
    Configurable(nlohmann::json& config_json, Configurable* parent);

    /// Default constructor — leaves the object in transient (uninitialized) state.
    Configurable() = default;

    Configurable(const Configurable&) = delete;
    Configurable(const Configurable&&) = delete;
    Configurable& operator=(const Configurable& other) = delete;
    Configurable& operator=(Configurable&& other) = delete;

    virtual ~Configurable();

    /// True if this Configurable was default-constructed and has no backing Configuration.
    bool isTransient() const { return is_transient_; }

    Configurable* parentConfigurable() const { return parent_; }

    void addChild(Configurable* child);
    void removeChild(Configurable* child);

    void writeBackConfig();
    /// Writes back config bottom-up: children first, then self. This ensures nested
    /// sub_configs are complete before the parent rebuilds its own sub_configs array.
    void writeBackConfigRecursive();

    virtual void resetToDefault();

    virtual std::string getPath() const;

    nlohmann::json& addNewSubConfiguration(const std::string& class_id,
                                           const std::string& instance_id);
    nlohmann::json& addNewSubConfiguration(const std::string& class_id);
    void removeSubConfigurations(const std::string& class_id);

    /// Returns a reference to the sub-config entry, creating it if it doesn't exist yet.
    nlohmann::json& ensureSubConfig(const std::string& class_id,
                                    const std::string& instance_id);
    /// Creates an empty sub-config entry and calls generateSubConfigurable() on it.
    void generateSubConfigurableFromConfig(const std::string& class_id,
                                           const std::string& instance_id);
    /// Clones the given configurable's JSON representation, merges additional_data,
    /// adds it as a sub-config entry, and generates the child configurable from it.
    void generateSubConfigurableFromJSON(const Configurable& configurable,
                                         const nlohmann::json& additional_data = nlohmann::json());
    bool hasSubConfigurable(const std::string& class_id, const std::string& instance_id) const;
    /// Walks a dot-separated path of instance/class names to find a descendant.
    std::pair<rtcommand::FindObjectErrCode, Configurable*> findSubConfigurablePath(const std::string& path);
    /// Searches for a child by name, then recurses into children if not found directly.
    std::pair<rtcommand::FindObjectErrCode, Configurable*> findSubConfigurableName(const std::string& name);
    /// Case-insensitive lookup: tries exact instance_id match first, then first class_id match.
    Configurable* getApproximateChildNamed(const std::string& approx_name);
    const Configurable& getChild(const std::string& class_id,
                                 const std::string& instance_id) const;
    Configurable& getChild(const std::string& class_id,
                           const std::string& instance_id);

    /// Override to control how reconfigure() handles sub-configs missing from the target
    /// hierarchy. Default: MustExist (fail on missing). Alternatives: CreateIfMissing, SkipIfMissing.
    virtual MissingKeyMode reconfigureSubConfigMode() const;
    /// Same as reconfigureSubConfigMode() but for parameters.
    virtual MissingKeyMode reconfigureParameterMode() const;

    const std::string& instanceId() const { return instance_id_; }
    const std::string& classId() const { return class_id_; }
    const std::string& keyId() const { return key_id_; }

    /// Prevents config removal from the parent's sub_config_storage_ when this instance is
    /// destroyed. Propagates to all children. Used during bulk teardown/rebuild scenarios.
    void setTmpDisableRemoveConfigOnDelete(bool value);

    void writeJSON(nlohmann::json& parent_json, JSONExportType export_type = JSONExportType::General) const;
    void generateJSON(nlohmann::json& target, JSONExportType export_type = JSONExportType::General) const;

    /// Applies a full JSON config tree (parameters + sub_configs) to this configurable and its
    /// descendants. Delegates to Configuration::reconfigure() which does a precheck then apply.
    ReconfigureResult reconfigure(const nlohmann::json& config,
                                  std::vector<MissingKey>* missing_subconfig_keys = nullptr,
                                  std::vector<MissingKey>* missing_param_keys = nullptr,
                                  bool assert_on_error = false,
                                  std::string* error = nullptr);

    /// Convenience wrappers: apply a full settings JSON (parameters + sub_configs) or just
    /// parameters to this configurable. String variants parse from a JSON string first.
    virtual Result applyJSONSettings(const nlohmann::json& settings_json);
    virtual Result applyJSONStringSettings(const std::string& settings_json_str);
    virtual Result applyJSONParameters(const nlohmann::json& params_json);
    virtual Result applyJSONStringParameters(const std::string& params_json_str);

    static std::string keyID(const std::string& class_id,
                             const std::string& instance_id);

    static const char ConfigurablePathSeparator;

protected:
    /// Override to instantiate the correct subclass from a child's JSON config.
    /// Called by createSubConfigurables() for each entry in sub_config_storage_.
    virtual void generateSubConfigurable(nlohmann::json& child_json);
    /// Iterates sub_config_storage_ and calls generateSubConfigurable() for each entry.
    /// Finishes with checkSubConfigurables().
    void createSubConfigurables();

    template <typename T>
    void registerParameter(const std::string& parameter_id, T* pointer, const T& default_value);
    template <typename T>
    T getParameterConfigValue(const std::string& parameter_id) const;

    /// Assigns value to param and triggers notifyModifications() if the value changed.
    template <typename T>
    void setParameter(T& param, const T& value);

    /// Override to create any sub-configurables that must exist but may be absent from the config
    /// (e.g. newly added mandatory children). Called at the end of createSubConfigurables().
    virtual void checkSubConfigurables();

    /// Called after reconfigure() applies parameter changes. Override to react
    /// (e.g. update widgets). changed_params contains dot-separated parameter paths.
    virtual void onConfigurationChanged(const std::vector<std::string>& changed_params) {}

    /// Called when this configurable or any descendant is modified via setParameter().
    virtual void onModified() {}

    void addJSONExportFilter(JSONExportType export_type,
                             JSONExportFilterType filter_type,
                             const std::string& id);
    void addJSONExportFilter(JSONExportType export_type,
                             JSONExportFilterType filter_type,
                             const std::vector<std::string>& ids);

    /// Invokes onModified() on this configurable and propagates up to the root parent.
    void notifyModifications();

    const Configuration& getConfiguration() const
    {
        traced_assert(configuration_);
        return *configuration_;
    }

    const Configurable& getParent() const
    {
        traced_assert(parent_);
        return *parent_;
    }

private:
    void configurationChanged(const std::vector<std::string>& changed_params);

    Configuration& configuration()
    {
        traced_assert(configuration_);
        return *configuration_;
    }

    std::string class_id_;
    std::string instance_id_;
    std::string key_id_;                            ///< class_id + instance_id
    Configurable* parent_{nullptr};
    std::string path_str_;                          ///< Dot-separated path from root
    Configuration* configuration_{nullptr};         ///< Owned. Created in constructor, deleted in destructor.
    bool is_transient_{true};                       ///< True until json-backed constructor runs

    bool tmp_disable_remove_config_on_delete_ {false};

    std::vector<Configurable*> children_vec_;       ///< Non-owning pointers to json-backed children

    boost::signals2::connection changed_connection_;
};
