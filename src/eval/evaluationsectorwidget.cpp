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

#include "evaluationsectorwidget.h"
#include "evaluationcalculator.h"
#include "evaluationdialog.h"
#include "sectorlayer.h"
#include "evaluationstandard.h"
#include "eval/requirement/group.h"

#include <QLabel>
#include <QCheckBox>
#include <QGridLayout>

using namespace std;

/**
*/
EvaluationSectorWidget::EvaluationSectorWidget(EvaluationCalculator& calculator, 
                                               EvaluationDialog& dialog, 
                                               QWidget* parent)
:   QScrollArea(parent)
,   calculator_(calculator)
,   dialog_    (dialog)
{
    QWidget* widget = new QWidget(this);
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    grid_layout_ = new QGridLayout;
    widget->setLayout(grid_layout_);

    setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAsNeeded);
    setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAsNeeded);

    update();

    setWidgetResizable(true);
    setWidget(widget);
}

/**
*/
void EvaluationSectorWidget::update()
{
    logdbg;

    traced_assert(grid_layout_);

    QLayoutItem* child;
    while (!grid_layout_->isEmpty() && (child = grid_layout_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    unsigned int row = 0;
    unsigned int col = 0;

    QFont font_italic;
    font_italic.setItalic(true);

    if (calculator_.hasCurrentStandard() && calculator_.sectorsLoaded())
    {
        EvaluationStandard& standard = calculator_.currentStandard();

        for (const auto& sec_lay_it : calculator_.sectorLayers())
        {
            col = 0;
            const string& sector_layer_name = sec_lay_it->name();

            QLabel* sec_label = new QLabel(sector_layer_name.c_str());
            sec_label->setFont(font_italic);
            grid_layout_->addWidget(sec_label, row, col);
            ++col;

            for (const auto& req_grp_it : standard)
            {
                const string& requirement_group_name = req_grp_it->name();

                QCheckBox* check = new QCheckBox(requirement_group_name.c_str());
                check->setChecked(calculator_.useGroupInSectorLayer(sector_layer_name, requirement_group_name));
                check->setProperty("sector_layer_name", sector_layer_name.c_str());
                check->setProperty("requirement_group_name", requirement_group_name.c_str());

                connect(check, &QCheckBox::clicked, this, &EvaluationSectorWidget::toggleUseGroupSlot);

                grid_layout_->addWidget(check, row, col);

                ++col;
            }
            ++row;
        }

        QSpacerItem* spacer = new QSpacerItem(1, 1, QSizePolicy::Preferred, QSizePolicy::Expanding);
        grid_layout_->addItem(spacer, row, 0);
    }

    logdbg << "done";
}

/**
*/
void EvaluationSectorWidget::toggleUseGroupSlot()
{
    QCheckBox* check = static_cast<QCheckBox*> (QObject::sender());
    traced_assert(check);

    string sector_layer_name = check->property("sector_layer_name").toString().toStdString();
    string requirement_group_name = check->property("requirement_group_name").toString().toStdString();

    traced_assert(calculator_.hasCurrentStandard());

    calculator_.useGroupInSectorLayer(sector_layer_name, requirement_group_name, check->checkState() == Qt::Checked);

    dialog_.updateButtons();
}
