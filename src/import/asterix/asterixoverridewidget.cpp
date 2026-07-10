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
#include "compass.h"
#include "textfielddoublevalidator.h"
#include "traced_assert.h"

#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
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

    QPushButton* resurf_btn = new QPushButton("From ReSURF Log...");
    resurf_btn->setToolTip(
        "Calculate TOD offset from ReSURF log output.\n"
        "Paste lines containing:\n"
        "  File input=...: first frame time set to HH:MM:SS.ss\n"
        "  File input=...: rendez-vous time set to HH:MM:SS.ss");
    connect(resurf_btn, &QPushButton::clicked, this, &ASTERIXOverrideWidget::fromResurfLogSlot);
    grid->addWidget(resurf_btn, row, 3);

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

    // position rectangle

    ++row;

    grid->addWidget(new QLabel("Filter Position Rectangle"), row, 0);

    filter_position_rec_active_check_ = new QCheckBox();
    connect(filter_position_rec_active_check_, &QCheckBox::clicked,
            this, &ASTERIXOverrideWidget::filterPositionRecActiveCheckedSlot);
    grid->addWidget(filter_position_rec_active_check_, row, 1);

    ++row;

    grid->addWidget(new QLabel("Latitude Min [deg]"), row, 1);

    filter_rec_latitude_min_edit_ = new QLineEdit();
    filter_rec_latitude_min_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 10));
    connect(filter_rec_latitude_min_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::recLatitudeMinEditedSlot);
    grid->addWidget(filter_rec_latitude_min_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Latitude Max [deg]"), row, 1);

    filter_rec_latitude_max_edit_ = new QLineEdit();
    filter_rec_latitude_max_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 10));
    connect(filter_rec_latitude_max_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::recLatitudeMaxEditedSlot);
    grid->addWidget(filter_rec_latitude_max_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Longitude Min [deg]"), row, 1);

    filter_rec_longitude_min_edit_ = new QLineEdit();
    filter_rec_longitude_min_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 10));
    connect(filter_rec_longitude_min_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::recLongitudeMinEditedSlot);
    grid->addWidget(filter_rec_longitude_min_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Longitude Max [deg]"), row, 1);

    filter_rec_longitude_max_edit_ = new QLineEdit();
    filter_rec_longitude_max_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 10));
    connect(filter_rec_longitude_max_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::recLongitudeMaxEditedSlot);
    grid->addWidget(filter_rec_longitude_max_edit_, row, 2);


    // position circle

    ++row;

    grid->addWidget(new QLabel("Filter Position Circle"), row, 0);

    filter_position_circ_active_check_ = new QCheckBox();
    connect(filter_position_circ_active_check_, &QCheckBox::clicked,
            this, &ASTERIXOverrideWidget::filterPositionCircActiveCheckedSlot);
    grid->addWidget(filter_position_circ_active_check_, row, 1);

    ++row;

    grid->addWidget(new QLabel("Center Latitude [deg]"), row, 1);

    filter_circ_latitude_edit_ = new QLineEdit();
    filter_circ_latitude_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 10));
    connect(filter_circ_latitude_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::circLatitudeEditedSlot);
    grid->addWidget(filter_circ_latitude_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Center Longitude [deg]"), row, 1);

    filter_circ_longitude_edit_ = new QLineEdit();
    filter_circ_longitude_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 10));
    connect(filter_circ_longitude_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::circLongitudeEditedSlot);
    grid->addWidget(filter_circ_longitude_edit_, row, 2);

    ++row;

    grid->addWidget(new QLabel("Range [nm]"), row, 1);

    filter_circ_range_edit_ = new QLineEdit();
    filter_circ_range_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 2));
    connect(filter_circ_range_edit_, &QLineEdit::textEdited, this, &ASTERIXOverrideWidget::circRangeEditedSlot);
    grid->addWidget(filter_circ_range_edit_, row, 2);

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

    // pos rec filter
    traced_assert(filter_position_rec_active_check_);
    filter_position_rec_active_check_->setChecked(task_.settings().filter_position_rec_active_);
    traced_assert(filter_rec_latitude_min_edit_);
    filter_rec_latitude_min_edit_->setText(QString::number(task_.settings().filter_rec_latitude_min_, 'g', 10));
    traced_assert(filter_rec_latitude_max_edit_);
    filter_rec_latitude_max_edit_->setText(QString::number(task_.settings().filter_rec_latitude_max_, 'g', 10));
    traced_assert(filter_rec_longitude_min_edit_);
    filter_rec_longitude_min_edit_->setText(QString::number(task_.settings().filter_rec_longitude_min_, 'g', 10));
    traced_assert(filter_rec_longitude_max_edit_);
    filter_rec_longitude_max_edit_->setText(QString::number(task_.settings().filter_rec_longitude_max_, 'g', 10));

    // pos circ filter
    traced_assert(filter_position_circ_active_check_);
    filter_position_circ_active_check_->setChecked(task_.settings().filter_position_circ_active_);
    traced_assert(filter_circ_latitude_edit_);
    filter_circ_latitude_edit_->setText(QString::number(task_.settings().filter_circ_latitude_, 'g', 10));
    traced_assert(filter_circ_longitude_edit_);
    filter_circ_longitude_edit_->setText(QString::number(task_.settings().filter_circ_longitude_, 'g', 10));
    traced_assert(filter_circ_range_edit_);
    filter_circ_range_edit_->setText(QString::number(task_.settings().filter_circ_range_, 'g', 2));

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
    TextFieldDoubleValidator::displayValidityAsColor(tod_offset_edit_, task_.compass().lineEditInvalidStyle());

    if (tod_offset_edit_->hasAcceptableInput())
        task_.settings().override_tod_offset_ = tod_offset_edit_->text().toDouble();
}

void ASTERIXOverrideWidget::fromResurfLogSlot()
{
    loginf;

    const QString example_text =
        "File input=LQ_ADSB_20260312/20260310.ff: first frame time set to 07:59:50.02\n"
        "File input=LQ_ADSB_20260312/20260310.ff: rendez-vous time set to 14:56:30.96";

    QDialog dlg(this);
    dlg.setWindowTitle("Calculate Offset from ReSURF Log");
    dlg.setMinimumWidth(600);

    QVBoxLayout* layout = new QVBoxLayout();

    layout->addWidget(new QLabel("Paste ReSURF log output containing 'first frame time set to' "
                                 "and 'rendez-vous time set to' lines:"));

    QPlainTextEdit* text_edit = new QPlainTextEdit();
    text_edit->setPlaceholderText(example_text);
    layout->addWidget(text_edit);

    QLabel* result_label = new QLabel();
    layout->addWidget(result_label);

    QDialogButtonBox* btn_box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    btn_box->button(QDialogButtonBox::Ok)->setEnabled(false);
    layout->addWidget(btn_box);

    double computed_offset = 0.0;

    connect(text_edit, &QPlainTextEdit::textChanged, [&]()
    {
        QString text = text_edit->toPlainText();

        QRegularExpression first_frame_re(R"(first frame time set to (\d{2}:\d{2}:\d{2}\.\d+))");
        QRegularExpression rendez_vous_re(R"(rendez-vous time set to (\d{2}:\d{2}:\d{2}\.\d+))");

        auto first_match = first_frame_re.match(text);
        auto rdv_match = rendez_vous_re.match(text);

        if (first_match.hasMatch() && rdv_match.hasMatch())
        {
            bool ok_first = false, ok_rdv = false;

            double first_secs = String::timeFromString(first_match.captured(1).toStdString(), &ok_first);
            double rdv_secs = String::timeFromString(rdv_match.captured(1).toStdString(), &ok_rdv);

            if (ok_first && ok_rdv)
            {
                computed_offset = first_secs - rdv_secs;
                result_label->setText(
                    QString("First frame: %1 (%2s)  |  Rendez-vous: %3 (%4s)  |  Offset: %5s")
                        .arg(first_match.captured(1))
                        .arg(first_secs, 0, 'f', 2)
                        .arg(rdv_match.captured(1))
                        .arg(rdv_secs, 0, 'f', 2)
                        .arg(computed_offset, 0, 'f', 2));
                btn_box->button(QDialogButtonBox::Ok)->setEnabled(true);
                return;
            }
        }

        result_label->setText("Could not parse both timestamps.");
        btn_box->button(QDialogButtonBox::Ok)->setEnabled(false);
    });

    connect(btn_box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btn_box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    dlg.setLayout(layout);

    if (dlg.exec() == QDialog::Accepted)
    {
        task_.settings().override_tod_active_ = true;
        task_.settings().override_tod_offset_ = computed_offset;
        updateSlot();

        loginf << "set TOD offset from ReSURF log: " << computed_offset;
    }
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

void ASTERIXOverrideWidget::filterPositionRecActiveCheckedSlot()
{
    loginf;
    traced_assert(filter_position_rec_active_check_);

    task_.settings().filter_position_rec_active_ = filter_position_rec_active_check_->checkState() == Qt::Checked;
}
void ASTERIXOverrideWidget::recLatitudeMinEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_rec_latitude_min_ = value;

}

void ASTERIXOverrideWidget::recLatitudeMaxEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_rec_latitude_max_ = value;
}

void ASTERIXOverrideWidget::recLongitudeMinEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_rec_longitude_min_ = value;
}

void ASTERIXOverrideWidget::recLongitudeMaxEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_rec_longitude_max_ = value;
}

void ASTERIXOverrideWidget::filterPositionCircActiveCheckedSlot()
{
    loginf;
    traced_assert(filter_position_circ_active_check_);

    task_.settings().filter_position_circ_active_ = filter_position_circ_active_check_->checkState() == Qt::Checked;
}
void ASTERIXOverrideWidget::circLatitudeEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_circ_latitude_ = value;
}
void ASTERIXOverrideWidget::circLongitudeEditedSlot(const QString& value_str)
{    
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_circ_longitude_ = value;
}
void ASTERIXOverrideWidget::circRangeEditedSlot(const QString& value_str)
{
    loginf << "value '" << value_str.toStdString() << "'";

    double value = value_str.toDouble();

    task_.settings().filter_circ_range_ = value;
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

