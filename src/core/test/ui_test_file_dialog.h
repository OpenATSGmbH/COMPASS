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

#include <QStringList>

namespace ui_test
{

/**
 * Pending results for native file dialog interception, queued via the
 * 'uifiledialog' runtime command and consumed one per dialog invocation
 * by the wrappers in Dialogs (dialogs.h).
 */

// queue a result for the next file dialog invocation; an empty list simulates a canceled dialog
void addFileDialogResult(const QStringList& files);

bool hasFileDialogResult();

// removes and returns the oldest queued result
QStringList takeFileDialogResult();

void clearFileDialogResults();

} // namespace ui_test
