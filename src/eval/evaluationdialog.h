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

#include <memory>

class EvaluationCalculator;
class EvaluationMainTabWidget;
//class EvaluationFilterTabWidget;
class EvaluationStandardTabWidget;
class EvaluationReportTabWidget;

class QLabel;

class EvaluationDialog : public QDialog
{
public:
    EvaluationDialog(EvaluationCalculator& calculator);
    virtual ~EvaluationDialog();

    void updateDataSources();
    void updateSectors();

    void updateButtons();

    //void updateFilterWidget();

protected:
    EvaluationCalculator& calculator_;

    std::unique_ptr<EvaluationMainTabWidget> main_tab_widget_;
    //std::unique_ptr<EvaluationFilterTabWidget> filter_widget_;
    std::unique_ptr<EvaluationStandardTabWidget> std_tab_widget_;
    std::unique_ptr<EvaluationReportTabWidget> report_tab_widget_;

    QLabel* not_eval_comment_label_ {nullptr};

    QPushButton* cancel_button_{nullptr};
    QPushButton* run_button_{nullptr};
};

