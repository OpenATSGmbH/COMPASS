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

#include <QString>
#include <QStringList>

class QWidget;

/**
 * Wrappers around the native file dialogs, interceptable by the UI test
 * framework: if a result was queued via the 'uifiledialog' runtime command,
 * it is consumed and returned without showing a dialog. Use these instead
 * of the QFileDialog static functions so the calling code stays testable.
 */
class Dialogs
{
public:
    static QString getOpenFileName(QWidget* parent, const QString& caption,
                                   const QString& dir, const QString& filter = QString());

    static QStringList getOpenFileNames(QWidget* parent, const QString& caption,
                                        const QString& dir, const QString& filter = QString());

    static QString getSaveFileName(QWidget* parent, const QString& caption,
                                   const QString& dir, const QString& filter = QString());

    static QWidget* statusDialogParent();
};
