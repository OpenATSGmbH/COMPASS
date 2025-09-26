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

#include "asterixoverridewidget.h"
#include "asteriximporttask.h"
#include "textfielddoublevalidator.h"
#include "traced_assert.h"

#include <QCheckBox>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QTimeEdit>
#include <QVBoxLayout>

using namespace std;
using namespace Utils;

ASTERIXOverrideWidget::ASTERIXOverrideWidget(ASTERIXImportTask& task, QWidget* parent)
    : QWidget(parent), task_(task)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    QGridLayout* grid = new QGridLayout();

    unsigned int row = 0;

    // tod override

    grid->addWidget(new QLabel("Override Time of Day"), row, 0);

    override_active_check_ = new QCheckBox();
    connect(override_active_check_, &QCheckBox::clicked, this, &ASTERIXOverrideWidget::overrideActiveCheckedSlot);
    grid->addWidget(override_active_check_, row, 1);

    ++row;

    grid->addWidget(new QLabel("Offset [s]"), row, 1);

    tod_offset_edit_ = new QLineEdit();
    tod_offset_edit_->setValidator(new TextFieldDoubleValidator(-24 * 3600, 24 * 3600, 5));
    connect(tod_offset_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::todOffsetEditedSlot);
    grid->addWidget(tod_offset_edit_, row, 2);

    // filters

    // time of day

    ++row;

    grid->addWidget(new QLabel("Filter Time of Day"), row, 0);

    filter_tod_active_check_ = new QCheckBox();
    connect(filter_tod_active_check_, &QCheckBox::clicked,
            this, &ASTERIXOverrideWidget::filterTimeOfDayActiveCheckedSlot);
    grid->addWidget(filter_tod_active_check_, row, 1);

    ++row;

    grid->addWidget(new QLabel("Time of Day Min [HH:MM:SS]"), row, 1);

    filter_tod_min_edit_ = new QTimeEdit();
    connect(filter_tod_min_edit_, &QTimeEdit::timeChanged,
            this, &ASTERIXOverrideWidget::minTimeChanged);
    grid->addWidget(filter_tod_min_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Time of Day Max [HH:MM:SS]"), row, 1);

    filter_tod_max_edit_ = new QTimeEdit();
    connect(filter_tod_max_edit_, &QTimeEdit::timeChanged,
            this, &ASTERIXOverrideWidget::maxTimeChanged);
    grid->addWidget(filter_tod_max_edit_, row, 2);

    // position

    ++row;

    grid->addWidget(new QLabel("Filter Position"), row, 0);

    filter_position_active_check_ = new QCheckBox();
    connect(filter_position_active_check_, &QCheckBox::clicked,
            this, &ASTERIXOverrideWidget::filterPositionActiveCheckedSlot);
    grid->addWidget(filter_position_active_check_, row, 1);

    ++row;

    grid->addWidget(new QLabel("Latitude Min [deg]"), row, 1);

    filter_latitude_min_edit_ = new QLineEdit();
    filter_latitude_min_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 10));
    connect(filter_latitude_min_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::latitudeMinEditedSlot);
    grid->addWidget(filter_latitude_min_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Latitude Max [deg]"), row, 1);

    filter_latitude_max_edit_ = new QLineEdit();
    filter_latitude_max_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 10));
    connect(filter_latitude_max_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::latitudeMaxEditedSlot);
    grid->addWidget(filter_latitude_max_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Longitude Min [deg]"), row, 1);

    filter_longitude_min_edit_ = new QLineEdit();
    filter_longitude_min_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 10));
    connect(filter_longitude_min_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::longitudeMinEditedSlot);
    grid->addWidget(filter_longitude_min_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Longitude Max [deg]"), row, 1);

    filter_longitude_max_edit_ = new QLineEdit();
    filter_longitude_max_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 10));
    connect(filter_longitude_max_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::longitudeMaxEditedSlot);
    grid->addWidget(filter_longitude_max_edit_, row, 2);

    // mode c

    ++row;

    grid->addWidget(new QLabel("Filter Mode C"), row, 0);

    filter_modec_active_check_ = new QCheckBox();
    connect(filter_modec_active_check_, &QCheckBox::clicked,
            this, &ASTERIXOverrideWidget::filterModeCActiveCheckedSlot);
    grid->addWidget(filter_modec_active_check_, row, 1);

    ++row;

    grid->addWidget(new QLabel("Mode C Min [ft]"), row, 1);

    filter_modec_min_edit_ = new QLineEdit();
    filter_modec_min_edit_->setValidator(new TextFieldDoubleValidator(-10000, 50000, 2));
    connect(filter_modec_min_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::modeCMinEditedSlot);
    grid->addWidget(filter_modec_min_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Mode C Max [ft]"), row, 1);

    filter_modec_max_edit_ = new QLineEdit();
    filter_modec_max_edit_->setValidator(new TextFieldDoubleValidator(-10000, 50000, 2));
    connect(filter_modec_max_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::modeCMaxEditedSlot);
    grid->addWidget(filter_modec_max_edit_, row, 2);

    // obfuscate


    ++row;

    grid->addWidget(new QLabel("Obfuscate Secondary Information"), row, 0);

    obfuscate_secondary_info_check_ = new QCheckBox();
    connect(obfuscate_secondary_info_check_, &QCheckBox::clicked,
            this, &ASTERIXOverrideWidget::obfuscateSecondaryInfoCheckedSlot);
    grid->addWidget(obfuscate_secondary_info_check_, row, 1);

    main_layout->addLayout(grid);

    main_layout->addStretch();

    updateSlot();

    setLayout(main_layout);
}

ASTERIXOverrideWidget::~ASTERIXOverrideWidget() {}

void ASTERIXOverrideWidget::updateSlot()
{
    // tod override
    traced_assert(override_active_check_);
    override_active_check_->setChecked(task_.settings().override_tod_active_);
    traced_assert(tod_offset_edit_);
    tod_offset_edit_->setText(String::doubleToStringPrecision(task_.settings().override_tod_offset_, 3).c_str());

    // tod filter
    traced_assert(filter_tod_active_check_);
    filter_tod_active_check_->setChecked(task_.settings().filter_tod_active_);
    traced_assert(filter_tod_min_edit_);
    filter_tod_min_edit_->setTime(
                QTime::fromString(String::timeStringFromDouble(task_.settings().filter_tod_min_).c_str()));
    traced_assert(filter_tod_max_edit_);
    filter_tod_max_edit_->setTime(
                QTime::fromString(String::timeStringFromDouble(task_.settings().filter_tod_max_).c_str()));

    // pos filter
    traced_assert(filter_position_active_check_);
    filter_position_active_check_->setChecked(task_.settings().filter_position_active_);
    traced_assert(filter_latitude_min_edit_);
    filter_latitude_min_edit_->setText(QString::number(task_.settings().filter_latitude_min_, 'g', 10));
    traced_assert(filter_latitude_max_edit_);
    filter_latitude_max_edit_->setText(QString::number(task_.settings().filter_latitude_max_, 'g', 10));
    traced_assert(filter_longitude_min_edit_);
    filter_longitude_min_edit_->setText(QString::number(task_.settings().filter_longitude_min_, 'g', 10));
    traced_assert(filter_longitude_max_edit_);
    filter_longitude_max_edit_->setText(QString::number(task_.settings().filter_longitude_max_, 'g', 10));

    // mode c filter
    traced_assert(filter_modec_active_check_);
    filter_modec_active_check_->setChecked(task_.settings().filter_modec_active_);
    traced_assert(filter_modec_min_edit_);
    filter_modec_min_edit_->setText(QString::number(task_.settings().filter_modec_min_));
    traced_assert(filter_modec_max_edit_);
    filter_modec_max_edit_->setText(QString::number(task_.settings().filter_modec_max_));

    traced_assert(obfuscate_secondary_info_check_);
    obfuscate_secondary_info_check_->setChecked(task_.settings().obfuscate_secondary_info_);
}

void ASTERIXOverrideWidget::overrideActiveCheckedSlot()
{
    loginf;
    traced_assert(override_active_check_);

    task_.settings().override_tod_active_ = override_active_check_->checkState() == Qt::Checked;
}

void ASTERIXOverrideWidget::todOffsetEditedSlot(const QString& value)
{
    loginf << "value '" << value.toStdString() << "'";
    TextFieldDoubleValidator::displayValidityAsColor(tod_offset_edit_);

    if (tod_offset_edit_->hasAcceptableInput())
        task_.settings().override_tod_offset_ = tod_offset_edit_->text().toDouble();
}

void ASTERIXOverrideWidget::filterTimeOfDayActiveCheckedSlot()
{
    loginf;
    traced_assert(filter_tod_active_check_);

    task_.settings().filter_tod_active_ = filter_tod_active_check_->checkState() == Qt::Checked;
}
void ASTERIXOverrideWidget::minTimeChanged(QTime time)
{
    float value = String::timeFromString(time.toString().toStdString());

    loginf << "value '" << time.toString().toStdString()
           << "' seconds " << value;

    task_.settings().filter_tod_min_ = value;
}
void ASTERIXOverrideWidget::maxTimeChanged(QTime time)
{
    float value = String::timeFromString(time.toString().toStdString());

    loginf << "value '" << time.toString().toStdString()
           << "' seconds " << value;

    task_.settings().filter_tod_max_ = value;
}

void ASTERIXOverrideWidget::filterPositionActiveCheckedSlot()
{
    loginf;
    traced_assert(filter_position_active_check_);

    task_.settings().filter_position_active_ = filter_position_active_check_->checkState() == Qt::Checked;
}
void ASTERIXOverrideWidget::latitudeMinEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_latitude_min_ = value;

}

void ASTERIXOverrideWidget::latitudeMaxEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_latitude_max_ = value;
}

void ASTERIXOverrideWidget::longitudeMinEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_longitude_min_ = value;
}

void ASTERIXOverrideWidget::longitudeMaxEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_longitude_max_ = value;
}

void ASTERIXOverrideWidget::filterModeCActiveCheckedSlot()
{
    loginf;
    traced_assert(filter_modec_active_check_);

    task_.settings().filter_modec_active_ = filter_modec_active_check_->checkState() == Qt::Checked;
}

void ASTERIXOverrideWidget::modeCMinEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_modec_min_ = value;
}

void ASTERIXOverrideWidget::modeCMaxEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_modec_max_ = value;
}

void ASTERIXOverrideWidget::obfuscateSecondaryInfoCheckedSlot()
{
    loginf;
    traced_assert(obfuscate_secondary_info_check_);

    task_.settings().obfuscate_secondary_info_ = obfuscate_secondary_info_check_->checkState() == Qt::Checked;
}
