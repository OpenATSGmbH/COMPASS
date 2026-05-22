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

#include "evaluationdatasourcewidget.h"
#include "evaluationstandardcombobox.h"
#include "evaluationsectorwidget.h"

#include <QWidget>

#include <memory>

class EvaluationCalculator;
class EvaluationDialog;

class QComboBox;
class QLineEdit;

/**
*/
class EvaluationMainTabWidget : public QWidget
{
    Q_OBJECT

private slots:
    void dbContentRefNameChangedSlot(const std::string& dbcontent_name);
    void lineRefChangedSlot(unsigned int line_id);
    void dbContentTstNameChangedSlot(const std::string& dbcontent_name);
    void lineTstChangedSlot(unsigned int line_id);

    void usedDataSourcesRefChangedSlot(); // to be called by data source widget
    void usedDataSourcesTstChangedSlot(); // to be called by data source widget

    void minHeightFilterChangedSlot(int idx);

    void reportNameChangedSlot();

    void changedStandardsSlot(); // eval man
    void changedCurrentStandardSlot(); // eval man

public:
    EvaluationMainTabWidget(EvaluationCalculator& calculator,
                            EvaluationDialog& dialog);
    ~EvaluationMainTabWidget() override;

    void updateDataSources();
    void updateSectors();

    void setReportName(const std::string& name);
    std::string reportName() const;

    bool isComplete() const;

protected:
    void updateMinHeightFilterCombo();
    void updateReportName();

    EvaluationCalculator& calculator_;
    EvaluationDialog& dialog_;

    std::unique_ptr<EvaluationDataSourceWidget> data_source_ref_widget_ {nullptr};
    std::unique_ptr<EvaluationDataSourceWidget> data_source_tst_widget_ {nullptr};

    std::unique_ptr<EvaluationStandardComboBox> standard_box_ {nullptr};
    std::unique_ptr<EvaluationSectorWidget> sector_widget_ {nullptr};

    QComboBox* min_height_filter_combo_ {nullptr};
    QLineEdit* report_name_edit_ {nullptr};
};

