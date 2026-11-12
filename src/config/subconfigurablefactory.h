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

#include "configurable.h"

#include "traced_assert.h"

#include <functional>
#include <initializer_list>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "json_fwd.hpp"

/**
 * Declares a static sub-configurable factory in a parent configurable. Place it in the public part,
 * it leaves the access at public.
 * - declares the typedef NameType and the static accessor NameType& Name()
 * - the accessor is defined in the parent's .cpp with a function-local static holding the built-in types
 * - declares a member that attaches the factory to each instance before the constructor body runs
 *   (i.e. before createSubConfigurables())
 * - Defaults is a SubConfigurableDefaults value (None, OnePerClass)
 */
#define DECLARE_SUBCONFIGURABLE_FACTORY(Name, Base, Parent, Defaults)                                \
    typedef SubConfigurableFactory<Base, Parent> Name##Type;                                         \
    static Name##Type& Name();                                                                       \
private:                                                                                             \
    SubConfigurableFactoryAttacher Name##_attacher_ { this, Name(), SubConfigurableDefaults::Defaults }; \
public:

/**
 * Default instances a parent creates for the classes of an attached factory.
 */
enum class SubConfigurableDefaults
{
    None,       // only children from the config
    OnePerClass // additionally <class_name>0 for every factory class without a child (after creation from the config)
};

/**
 * Source of the creator of a factory class.
 * - a class without creator is known but cannot be created: BuiltIn => error, Plugin => plugin not present, skipped
 */
enum class CreationSource
{
    BuiltIn, // creator defined with the factory
    Plugin   // creator added by a plugin at runtime (missing if the plugin is not present)
};

/**
 * Type-erased part of a sub-configurable factory, as stored by Configurable.
 */
class SubConfigurableFactoryBase
{
public:
    virtual ~SubConfigurableFactoryBase() = default;

    virtual bool knows(const std::string& class_name) const = 0;     // class listed, with or without creator
    virtual bool canCreate(const std::string& class_name) const = 0; // class listed with creator
    virtual CreationSource creationSource(const std::string& class_name) const = 0; // of a known class
    virtual std::vector<std::string> classNames() const = 0;         // classes that can be created

    /// creates the child, ownership goes to the caller
    virtual Configurable* createChild(nlohmann::json& child_json, Configurable& parent) const = 0;
};

/**
 * Attaches a sub-configurable factory to its configurable on construction (used by DECLARE_SUBCONFIGURABLE_FACTORY).
 */
class SubConfigurableFactoryAttacher
{
public:
    SubConfigurableFactoryAttacher(Configurable* configurable,
                                   const SubConfigurableFactoryBase& factory,
                                   SubConfigurableDefaults defaults)
    {
        traced_assert(configurable);
        configurable->addSubConfigurableFactory(factory, defaults);
    }
};

/**
 * Creates sub-configurables derived from Base by class name.
 * - Parent is the configurable creating and owning the children
 * - the classes are listed on construction: built-in types with creator, plugin types without (CreationSource::Plugin)
 * - addPlugin() sets the creator of a listed plugin type (called by the plugin), add() adds a type not listed
 * - plugin types must be listed: a build without plugins skips them when reading a config instead of throwing
 * - class names are unique
 */
template<typename Base, typename Parent>
class SubConfigurableFactory : public SubConfigurableFactoryBase
{
public:
    typedef std::function<std::unique_ptr<Base>(nlohmann::json&, Parent&)> Creator;

    struct Entry
    {
        std::string    class_name;
        Creator        creator;                           // empty for a plugin type until the plugin adds it
        std::string    display_name;                      // name shown in menus (empty = class name)
        CreationSource source = CreationSource::BuiltIn;
    };

    SubConfigurableFactory() = default;
    SubConfigurableFactory(std::initializer_list<Entry> entries);
    virtual ~SubConfigurableFactory() = default;

    void add(const std::string& class_name,
             const Creator& creator,
             const std::string& display_name = "");
    template<typename T>
    void add(const std::string& class_name,
             const std::string& display_name = "");
    void addPlugin(const std::string& class_name,
                   const Creator& creator,
                   const std::string& display_name = "");
    template<typename T>
    void addPlugin(const std::string& class_name,
                   const std::string& display_name = "");

    template<typename T>
    static std::unique_ptr<Base> createDefault(nlohmann::json& child_json, Parent& parent);

    bool knows(const std::string& class_name) const override;
    bool canCreate(const std::string& class_name) const override;
    CreationSource creationSource(const std::string& class_name) const override;
    std::unique_ptr<Base> create(nlohmann::json& child_json, Parent& parent) const;
    Configurable* createChild(nlohmann::json& child_json, Configurable& parent) const override;

    std::vector<std::string> classNames() const override;
    std::string displayName(const std::string& class_name) const;

private:
    std::map<std::string, Entry> entries_; // class name -> entry
};

/**
*/
template<typename Base, typename Parent>
SubConfigurableFactory<Base, Parent>::SubConfigurableFactory(std::initializer_list<Entry> entries)
{
    for (const auto& e : entries)
    {
        traced_assert(!e.class_name.empty());
        traced_assert_msg(!knows(e.class_name), ("sub-configurable class '" + e.class_name + "' listed twice").c_str());

        entries_[ e.class_name ] = e;
    }
}

/**
 * Adds a type not listed on construction, with the given creator.
 * Notice: plugin types are listed and filled with addPlugin().
 */
template<typename Base, typename Parent>
void SubConfigurableFactory<Base, Parent>::add(const std::string& class_name,
                                               const Creator& creator,
                                               const std::string& display_name)
{
    traced_assert(!class_name.empty());
    traced_assert(creator);
    traced_assert_msg(!knows(class_name),
                      ("sub-configurable class '" + class_name + "' already listed or added").c_str());

    entries_[ class_name ] = Entry{ class_name, creator, display_name, CreationSource::Plugin };
}

/**
 * Adds a creator for type T with the common constructor T(nlohmann::json&, Parent*).
 */
template<typename Base, typename Parent>
template<typename T>
void SubConfigurableFactory<Base, Parent>::add(const std::string& class_name,
                                               const std::string& display_name)
{
    add(class_name, &SubConfigurableFactory<Base, Parent>::createDefault<T>, display_name);
}

/**
 * Sets the creator of a plugin type listed on construction, called by the plugin.
 * Notice: a non-empty display name replaces the listed one.
 */
template<typename Base, typename Parent>
void SubConfigurableFactory<Base, Parent>::addPlugin(const std::string& class_name,
                                                     const Creator& creator,
                                                     const std::string& display_name)
{
    traced_assert(creator);

    auto it = entries_.find(class_name);
    traced_assert_msg(it != entries_.end(),
                      ("plugin sub-configurable class '" + class_name + "' not listed by the host").c_str());
    traced_assert_msg(it->second.source == CreationSource::Plugin,
                      ("built-in sub-configurable class '" + class_name + "' added by a plugin").c_str());
    traced_assert_msg(!it->second.creator,
                      ("plugin sub-configurable class '" + class_name + "' added twice").c_str());

    it->second.creator = creator;

    if (!display_name.empty())
        it->second.display_name = display_name;
}

/**
 * Sets a creator for plugin type T with the common constructor T(nlohmann::json&, Parent*).
 */
template<typename Base, typename Parent>
template<typename T>
void SubConfigurableFactory<Base, Parent>::addPlugin(const std::string& class_name,
                                                     const std::string& display_name)
{
    addPlugin(class_name, &SubConfigurableFactory<Base, Parent>::createDefault<T>, display_name);
}

/**
 * Creator for types with the common constructor T(nlohmann::json&, Parent*).
 */
template<typename Base, typename Parent>
template<typename T>
std::unique_ptr<Base> SubConfigurableFactory<Base, Parent>::createDefault(nlohmann::json& child_json, Parent& parent)
{
    static_assert(std::is_base_of<Base, T>::value, "T must derive from Base");

    return std::unique_ptr<Base>(new T(child_json, &parent));
}

/**
*/
template<typename Base, typename Parent>
bool SubConfigurableFactory<Base, Parent>::knows(const std::string& class_name) const
{
    return entries_.count(class_name) > 0;
}

/**
*/
template<typename Base, typename Parent>
bool SubConfigurableFactory<Base, Parent>::canCreate(const std::string& class_name) const
{
    auto it = entries_.find(class_name);
    return it != entries_.end() && it->second.creator;
}

/**
*/
template<typename Base, typename Parent>
CreationSource SubConfigurableFactory<Base, Parent>::creationSource(const std::string& class_name) const
{
    auto it = entries_.find(class_name);
    traced_assert(it != entries_.end());

    return it->second.source;
}

/**
 * Creates the sub-configurable whose class name is stored in the given json.
 */
template<typename Base, typename Parent>
std::unique_ptr<Base> SubConfigurableFactory<Base, Parent>::create(nlohmann::json& child_json, Parent& parent) const
{
    const auto class_name = Configuration::getClassName(child_json);

    auto it = entries_.find(class_name);
    traced_assert_msg(it != entries_.end() && it->second.creator,
                      ("sub-configurable class '" + class_name + "' cannot be created").c_str());

    auto obj = it->second.creator(child_json, parent);
    traced_assert(obj);

    return obj;
}

/**
*/
template<typename Base, typename Parent>
Configurable* SubConfigurableFactory<Base, Parent>::createChild(nlohmann::json& child_json, Configurable& parent) const
{
    static_assert(std::is_base_of<Configurable, Base>::value, "Base must derive from Configurable");
    static_assert(std::is_base_of<Configurable, Parent>::value, "Parent must derive from Configurable");

    auto parent_typed = dynamic_cast<Parent*>(&parent);
    traced_assert(parent_typed);

    return create(child_json, *parent_typed).release();
}

/**
*/
template<typename Base, typename Parent>
std::vector<std::string> SubConfigurableFactory<Base, Parent>::classNames() const
{
    std::vector<std::string> names;
    names.reserve(entries_.size());

    for (const auto& [ class_name, entry ] : entries_)
    {
        if (!entry.creator)
            continue;

        names.push_back(class_name);
    }

    return names;
}

/**
 * Returns the display name of the given class, or the class name if none is set.
 */
template<typename Base, typename Parent>
std::string SubConfigurableFactory<Base, Parent>::displayName(const std::string& class_name) const
{
    auto it = entries_.find(class_name);
    traced_assert(it != entries_.end());

    return it->second.display_name.empty() ? class_name : it->second.display_name;
}
