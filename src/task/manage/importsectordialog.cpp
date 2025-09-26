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

#include "importsectordialog.h"
#include "logger.h"

#include <QLineEdit>
#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QColorDialog>
#include <QApplication>

#include "traced_assert.h"

ImportSectorDialog::ImportSectorDialog(const std::string& layer_name,
                                       QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f)
{
    setWindowFlags(Qt::Window | Qt::WindowTitleHint); //  | Qt::CustomizeWindowHint

    setWindowTitle("Import Sector");

    setModal(true);

    QVBoxLayout* main_layout = new QVBoxLayout();

    int row = 0;
    QGridLayout* grid = new QGridLayout();

    // sector layer
    grid->addWidget(new QLabel("Sector Layer"), row, 0);

    layer_name_edit_edit_ = new QLineEdit ();
    layer_name_edit_edit_->setText(layer_name.c_str());
    grid->addWidget(layer_name_edit_edit_, row, 1);

    ++row;

    // exclude
    grid->addWidget(new QLabel("Exclude"), row, 0);

    exclude_check_ = new QCheckBox();
    exclude_check_->setChecked(false);
    grid->addWidget(exclude_check_, row, 1);

    // color
    ++row;
    grid->addWidget(new QLabel("Color"), row, 0);

    color_ = QColor("#AAAAAA");

    color_button_ = new QPushButton();
    color_button_->setFlat(true);

    QPalette pal = color_button_->palette();
    pal.setColor(QPalette::Button, color_);
    color_button_->setAutoFillBackground(true);
    color_button_->setPalette(pal);

    connect (color_button_, &QPushButton::clicked, this, &ImportSectorDialog::colorSlot);
    grid->addWidget(color_button_, row, 1);

    main_layout->addLayout(grid);

    main_layout->addStretch();

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* quit_button_ = new QPushButton("Cancel");
    connect(quit_button_, &QPushButton::clicked, this, &ImportSectorDialog::cancelSlot);
    button_layout->addWidget(quit_button_);

    QPushButton* import_button_ = new QPushButton("Import");
    connect(import_button_, &QPushButton::clicked, this, &ImportSectorDialog::importSlot);
    button_layout->addWidget(import_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

}


std::string ImportSectorDialog::layerName()
{
    traced_assert(layer_name_edit_edit_);
    return layer_name_edit_edit_->text().toStdString();
}

bool ImportSectorDialog::exclude ()
{
    traced_assert(exclude_check_);
    return exclude_check_->checkState() == Qt::Checked;
}

QColor ImportSectorDialog::color ()
{
    return color_;
}

void ImportSectorDialog::colorSlot()
{
    QColor color =
            QColorDialog::getColor(color_, QApplication::activeWindow(), "Select Sector");

    if (color.isValid())
    {
        loginf << "color " << color.name().toStdString();
        color_ = color;

        traced_assert(color_button_);

        QPalette pal = color_button_->palette();
        pal.setColor(QPalette::Button, color_);
        color_button_->setPalette(pal);
    }
}

void ImportSectorDialog::cancelSlot()
{
    reject();
}

void ImportSectorDialog::importSlot()
{
    accept();
}
