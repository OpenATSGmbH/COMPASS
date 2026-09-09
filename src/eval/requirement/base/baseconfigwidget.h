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

#include <QWidget>

class QLineEdit;
class QCheckBox;
class QComboBox;

class QFormLayout;

namespace EvaluationRequirement
{

class BaseConfig;
class ComparisonTypeComboBox;

class BaseConfigWidget : public QWidget
{
    Q_OBJECT

signals:

public slots:
    void changedNameSlot();
    void changedShortNameSlot();
    void changedCommentSlot();
    void changedTargetSelectionSlot(int index);


public:
    BaseConfigWidget(BaseConfig& cfg);
    virtual ~BaseConfigWidget();

protected:
    BaseConfig& config_;

    QFormLayout* form_layout_ {nullptr};
    QComboBox* target_selection_box_ {nullptr};
};

}
