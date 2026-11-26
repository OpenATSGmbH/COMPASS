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

#include "exit_hooks.h"

#include <iostream>
#include <mutex>
#include <utility>
#include <vector>

namespace exit_hooks
{

namespace
{
    struct Registry
    {
        std::mutex                                                  mutex;
        std::vector<std::pair<std::string, std::function<void()>>> hooks;
        bool                                                        done = false;
    };

    Registry& registry()
    {
        //never destroyed: the hooks may run during the static destruction
        static Registry* reg = new Registry;
        return *reg;
    }
}

/**
 */
void add(const std::string& name, const std::function<void()>& hook)
{
    auto& reg = registry();

    std::lock_guard<std::mutex> lock(reg.mutex);
    reg.hooks.emplace_back(name, hook);
}

/**
 */
void run()
{
    auto& reg = registry();

    std::vector<std::pair<std::string, std::function<void()>>> hooks;
    {
        std::lock_guard<std::mutex> lock(reg.mutex);

        if (reg.done)
            return;

        reg.done = true;
        hooks.swap(reg.hooks);
    }

    //std::cerr: also used on the abort paths, where the logger may not be usable
    for (auto it = hooks.rbegin(); it != hooks.rend(); ++it)
    {
        try
        {
            it->second();
        }
        catch (const std::exception& e)
        {
            std::cerr << "exit hook '" << it->first << "' failed: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "exit hook '" << it->first << "' failed" << std::endl;
        }
    }
}

}
