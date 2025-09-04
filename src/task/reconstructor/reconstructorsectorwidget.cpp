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

#include "reconstructorsectorwidget.h"
#include "reconstructortask.h"
#include "sectorlayer.h"
#include "logger.h"
#include "stringconv.h"

#include <QLabel>
#include <QCheckBox>
#include <QVariant>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>

using namespace std;
using namespace Utils;

/**
*/
ReconstructorSectorWidget::ReconstructorSectorWidget(ReconstructorTask& task, 
                                               QWidget* parent)
:   QScrollArea(parent)
,   task_    (task)
{
    QWidget* widget = new QWidget(this);
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QVBoxLayout* layout = new QVBoxLayout();

    use_sectors_check_ = new QCheckBox ("Use Sectors");
    connect(use_sectors_check_, &QCheckBox::clicked, this, &ReconstructorSectorWidget::toggleUseSectorsSlot);
    layout->addWidget(use_sectors_check_);

    QHBoxLayout* grid_cont_lay = new QHBoxLayout();
    grid_cont_lay->addSpacing(40);

    grid_layout_ = new QGridLayout;
    grid_cont_lay->addLayout(grid_layout_);
    
    layout->addLayout(grid_cont_lay);

    layout->addWidget(new QLabel("Surrounding Delta [WGS84 deg]"));

    sectors_delta_edit_ = new QLineEdit();
    connect(sectors_delta_edit_, &QLineEdit::textEdited, 
        this, &ReconstructorSectorWidget::sectorsDeltaChangedSlot);

    layout->addWidget(sectors_delta_edit_);

    layout->addStretch();

    widget->setLayout(layout);

    setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAsNeeded);
    setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAsNeeded);

    update();

    setWidgetResizable(true);
    setWidget(widget);
}

void ReconstructorSectorWidget::toggleUseSectorsSlot()
{
    traced_assert(use_sectors_check_);
    task_.useSectorsExtend(use_sectors_check_->checkState() == Qt::Checked);
}

void ReconstructorSectorWidget::toggleSectorSlot()
{
    QCheckBox* check = (QCheckBox*)sender();
    traced_assert(check);

    std::string name = check->property("name").toString().toStdString();

    task_.useSector(name, check->checkState() == Qt::Checked);
}

/**
*/
void ReconstructorSectorWidget::update()
{
    logdbg;

    traced_assert(use_sectors_check_);
    use_sectors_check_->setChecked(task_.useSectorsExtend());

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

    auto& used_sectors = task_.usedSectors();

    if (used_sectors.size())
    {
        for (const auto& sec_lay_it : used_sectors)
        {
            QCheckBox* check = new QCheckBox(sec_lay_it.first.c_str());
            check->setProperty("name", sec_lay_it.first.c_str());
            check->setChecked(sec_lay_it.second);
            connect(check, SIGNAL(clicked()), this, SLOT(toggleSectorSlot()));

            grid_layout_->addWidget(check, row, col);

            ++row;
        }

        QSpacerItem* spacer = new QSpacerItem(1, 1, QSizePolicy::Preferred, QSizePolicy::Expanding);
        grid_layout_->addItem(spacer, row, 0);
    }

    use_sectors_check_->setEnabled(used_sectors.size());

    sectors_delta_edit_->setText(QString::number(task_.sectorDeltaDeg()));

    logdbg << "done";
}

void ReconstructorSectorWidget::sectorsDeltaChangedSlot(const QString& value_str)
{
    traced_assert(sectors_delta_edit_);

    bool ok;
    float value = value_str.toDouble(&ok);

    if (ok)
    {
        loginf << "sectors_delta " << value;
        task_.sectorDeltaDeg(value);
    }
    else
        loginf << "unable to parse string '" << value_str.toStdString() << "'";
}

