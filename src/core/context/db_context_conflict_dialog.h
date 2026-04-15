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

#include <QDialog>

class QLabel;

namespace context
{

class DBContextDiff;

/**
 * Shown on DB open when the file context and DB context differ.
 * Offers three resolution options:
 *   - Use File: overwrite DB with file definition
 *   - Use Database: overwrite file with DB definition
 *   - Merge: open a merge dialog (not yet implemented)
 */
class DBContextConflictDialog : public QDialog
{
    Q_OBJECT

public:
    enum Resolution { UseFile, UseDatabase, Merge };

    explicit DBContextConflictDialog(const std::string& context_name,
                                     const DBContextDiff& diff,
                                     QWidget* parent = nullptr);

    Resolution resolution() const { return resolution_; }

private slots:
    void useFileSlot();
    void useDatabaseSlot();
    void mergeSlot();

private:
    Resolution resolution_{UseFile};
};

} // namespace context
