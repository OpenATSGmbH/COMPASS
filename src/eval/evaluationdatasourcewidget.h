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

#include "compass_eval_export.h"

#include <string>

#include <QFrame>

class EvaluationCalculator;
class DBContentComboBox;

class QCheckBox;
class QGridLayout;

class COMPASS_EVAL_API EvaluationDataSourceWidget : public QFrame
{
    Q_OBJECT

signals:
    void dbContentNameChangedSignal(const std::string& dbcontent_name);
    void usedDataSourceChangedSignal();
    void lineChangedSignal(unsigned int line_id);

public slots:
    void dbContentNameChangedSlot();
    void toggleDataSourceSlot();
    void lineIDEditSlot(const QString& text);

    void updateDataSourcesSlot();

public:
    EvaluationDataSourceWidget(EvaluationCalculator& calculator,
                               const std::string& title, 
                               const std::string& dbcontent_name, 
                               unsigned int line_id,
                               QWidget* parent=nullptr, Qt::WindowFlags f=Qt::WindowFlags());

    virtual ~EvaluationDataSourceWidget(); 

protected:
    EvaluationCalculator& calculator_;

    std::string title_;
    std::string dbcontent_name_;
    unsigned int line_id_;

    DBContentComboBox* dbcont_combo_ {nullptr};

    QGridLayout* data_source_layout_ {nullptr};
    std::map<unsigned int, QCheckBox*> data_sources_checkboxes_;

    void updateCheckboxesChecked();
};
