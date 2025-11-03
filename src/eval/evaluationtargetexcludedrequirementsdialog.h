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

#include <set>
#include <string>

class QScrollArea;
class QGridLayout;
class QCheckBox;
class QLineEdit;

class EvaluationTargetExcludedRequirementsDialog : public QDialog
{
public:
    EvaluationTargetExcludedRequirementsDialog(
                const std::string utn_str, std::set<std::string> selected_requirements,
        std::set<std::string> available_requirements, std::string comment="", QWidget* parent=nullptr);
    virtual ~EvaluationTargetExcludedRequirementsDialog() = default;

    std::set<std::string> selectedRequirements() const;
    std::string comment() const;

protected:
    std::set<std::string> selected_requirements_;
    std::set<std::string> available_requirements_;

private:
    QScrollArea* scroll_area_{nullptr};
    QWidget* scroll_widget_{nullptr};
    QGridLayout* grid_layout_{nullptr};

    std::vector<QCheckBox*> requirement_checkboxes_;
    QLineEdit* comment_edit_{nullptr};
};

