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

#include "dialogs.h"
#include "ui_test_file_dialog.h"
#include "files.h"
#include "logger.h"

#include <QFileDialog>

namespace
{

// consumes a queued ui test result if available, checking existence for open-type dialogs;
// returns true if the call is intercepted (result then holds the files, empty = canceled)
bool takeTestResult(bool existing_files, QStringList& result)
{
    if (!ui_test::hasFileDialogResult())
        return false;

    result = ui_test::takeFileDialogResult();

    if (existing_files)
    {
        for (const auto& file : result)
        {
            if (!Utils::Files::fileExists(file.toStdString()))
            {
                logerr << "queued file '" << file.toStdString()
                       << "' does not exist, simulating canceled dialog";
                result.clear();
                break;
            }
        }
    }

    return true;
}

}

QString Dialogs::getOpenFileName(QWidget* parent, const QString& caption,
                                 const QString& dir, const QString& filter)
{
    QStringList test_result;
    if (takeTestResult(true, test_result))
        return test_result.isEmpty() ? QString() : test_result.first();

    return QFileDialog::getOpenFileName(parent, caption, dir, filter);
}

QStringList Dialogs::getOpenFileNames(QWidget* parent, const QString& caption,
                                      const QString& dir, const QString& filter)
{
    QStringList test_result;
    if (takeTestResult(true, test_result))
        return test_result;

    return QFileDialog::getOpenFileNames(parent, caption, dir, filter);
}

QString Dialogs::getSaveFileName(QWidget* parent, const QString& caption,
                                 const QString& dir, const QString& filter)
{
    QStringList test_result;
    if (takeTestResult(false, test_result))
        return test_result.isEmpty() ? QString() : test_result.first();

    return QFileDialog::getSaveFileName(parent, caption, dir, filter);
}
