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

#include "db_context.h"

#include <QDialog>

namespace context
{

class DBContextDiff;

/**
 * Dialog for interactively merging two versions of a context.
 * Shows per-item diffs and lets the user pick which side wins for each.
 * The result overwrites both the file and DB definitions.
 *
 * TODO: implement UI and merge logic.
 */
class DBContextMergeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DBContextMergeDialog(const DBContext& file_context,
                                  const DBContext& db_context,
                                  const DBContextDiff& diff,
                                  QWidget* parent = nullptr);

    /// Returns the merged context after the dialog is accepted.
    const DBContext& mergedContext() const { return merged_; }

private:
    DBContext merged_;
};

} // namespace context
