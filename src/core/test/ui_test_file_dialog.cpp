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

#include "ui_test_file_dialog.h"
#include "traced_assert.h"
#include "logger.h"

#include <deque>

namespace ui_test
{

namespace
{
std::deque<QStringList> file_dialog_results;
}

void addFileDialogResult(const QStringList& files)
{
    loginf << "queueing " << files.size() << " file(s)";

    file_dialog_results.push_back(files);
}

bool hasFileDialogResult()
{
    return !file_dialog_results.empty();
}

QStringList takeFileDialogResult()
{
    traced_assert(!file_dialog_results.empty());

    QStringList result = file_dialog_results.front();
    file_dialog_results.pop_front();

    return result;
}

void clearFileDialogResults()
{
    file_dialog_results.clear();
}

} // namespace ui_test
