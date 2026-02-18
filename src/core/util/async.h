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

#include <functional>
#include <string>

namespace Utils
{
namespace Async
{

extern bool waitDialogAsync(const std::function<bool()>& task,
                            const std::function<int()>& done_cb,
                            int steps,
                            const std::string& task_name,
                            const std::string& wait_msg = "");
extern bool waitDialogAsync(const std::function<bool()>& task,
                            const std::string& task_name,
                            const std::string& wait_msg);
extern bool waitDialogAsyncArray(const std::function<bool(int)>& task,
                                 int steps,
                                 const std::string& task_name,
                                 const std::string& wait_msg = "");

extern void waitAndProcessEventsFor (unsigned int milliseconds);

}
}
