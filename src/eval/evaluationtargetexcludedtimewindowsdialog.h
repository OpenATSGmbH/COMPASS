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

#include "timewindowcollectionwidget.h"

class QLineEdit;

class EvaluationTargetExcludedTimeWindowsDialog : public QDialog
{
    Q_OBJECT

public:
    EvaluationTargetExcludedTimeWindowsDialog(
        const std::string utn_str,
        Utils::TimeWindowCollection& collection, std::string comment="", QWidget* parent=nullptr);
    virtual ~EvaluationTargetExcludedTimeWindowsDialog() = default;

    std::string comment() const;

protected:
    Utils::TimeWindowCollection& collection_;

    TimeWindowCollectionWidget* tw_widget_{nullptr};
    QLineEdit* comment_edit_{nullptr};
};

