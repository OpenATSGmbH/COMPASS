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

#include <map>
#include <vector>
#include <functional>
#include <iostream>

#include <QString>

namespace rtcommand
{

/**
 * Registry/Factory for runtime commands.
 * Keeps track of registered runtime commands and spawns command templates given a registered command name.
 */
class RTCommandRegistry : public Singleton
{
public:
    typedef std::function<RTCommand*()>             CreatorFunc;
    typedef std::map<QString, RTCommandDescription> AvailableCommands;

    virtual ~RTCommandRegistry() = default;

    static RTCommandRegistry& instance()
    {
        static RTCommandRegistry instance;
        return instance;
    }

    /**
     * Registers a command.
     */
    bool registerCommand(const QString& name, const QString& description, CreatorFunc func)
    {
        //std::cout << "Registering command '" << name.toStdString() << "'" << std::endl;

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
     * Returns names and descriptions for all available/registered commands.
     */
    const AvailableCommands& availableCommands() const { return commands_; }

    /**
     * Cheacks if a command of the given name is registered.
     */
    bool hasCommand(const QString& name) { return commands_.find(name) != commands_.end(); }

    /**
     * Creates an unconfigured command template given a registered command name.
     */
    std::unique_ptr<RTCommand> createCommandTemplate(const QString& name)
    {
        auto it = creators_.find(name);
        if (it == creators_.end() || !it->second)
            return std::unique_ptr<RTCommand>();

        return std::unique_ptr<RTCommand>(it->second());
    }

protected:
    RTCommandRegistry() = default;

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
