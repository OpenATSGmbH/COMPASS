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

#include "singleton.h"
#include "rtcommand.h"
#include "traced_assert.h"

#include <map>
#include <vector>
#include <functional>
#include <iostream>
#include <mutex>

#include <QString>

namespace rtcommand
{

/**
 * Registry/Factory for runtime commands.
 * Keeps track of registered runtime commands and spawns command templates given a registered command name.
 * - the commands are registered at startup by the init_*_commands(registry, ...) functions of their modules,
 *   whose creators pass the needed managers to the command constructors
 * - cleared by the composition root on shutdown (the creators reference the managers)
 * - thread safe: registration runs in the main thread, while the command threads may already create commands
 */
class RTCommandRegistry : public Singleton
{
public:
    typedef std::function<RTCommand*()>             CreatorFunc;
    typedef std::map<QString, RTCommandDescription> AvailableCommands;

    virtual ~RTCommandRegistry() = default;

    static RTCommandRegistry& instance(); // out of line: one instance across all shared libraries

    /**
     * Registers a command.
     */
    bool registerCommand(const QString& name, const QString& description, CreatorFunc func)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        bool inserted = creators_.insert(std::make_pair(name, func)).second;

        if (inserted)
        {
            RTCommandDescription d;
            d.name        = name;
            d.description = description;

            commands_.insert(std::make_pair(name, d));
        }

        return inserted;
    }

    /**
     * Registers a command whose creator passes the given dependencies to its constructor.
     * Notice: the dependencies must outlive the registration (see clear()).
     */
    template<typename Cmd, typename... Deps>
    void registerCommand(Deps&... deps)
    {
        bool inserted = registerCommand(Cmd::staticName(),
                                        Cmd::staticDescription(),
                                        [ &deps... ] () { return new Cmd(deps...); });
        traced_assert(inserted); // registered twice => programming error
    }

    /**
     * Removes all registered commands.
     */
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        creators_.clear();
        commands_.clear();
    }

    /**
     * Returns names and descriptions for all available/registered commands.
     */
    AvailableCommands availableCommands() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_;
    }

    /**
     * Cheacks if a command of the given name is registered.
     */
    bool hasCommand(const QString& name)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_.find(name) != commands_.end();
    }

    /**
     * Creates an unconfigured command template given a registered command name.
     */
    std::unique_ptr<RTCommand> createCommandTemplate(const QString& name)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = creators_.find(name);
        if (it == creators_.end() || !it->second)
            return std::unique_ptr<RTCommand>();

        return std::unique_ptr<RTCommand>(it->second());
    }

protected:
    RTCommandRegistry() = default;

    mutable std::mutex             mutex_;
    std::map<QString, CreatorFunc> creators_;
    AvailableCommands              commands_;
};

//template <typename T>
//struct RTCommandRegistrator
//{
//    RTCommandRegistrator(const QString& name)
//    {
//        std::cout << "RTCommandRegistrator: '" << name.toStdString() << "'" << std::endl;
//        RTCommandRegistry::instance().registerCommand(name, [] () { return new T; });
//    };
//};

} // namespace rtcommand
