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

#include "adsbqualityfilterwidget.h"
#include "logger.h"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>

ADSBQualityFilterWidget::ADSBQualityFilterWidget(ADSBQualityFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    int row = 0;

    // v0
    use_v0_check_ = new QCheckBox ("Use v0");
    connect(use_v0_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseV0Slot);
    child_layout_->addWidget(use_v0_check_, row++, 0);

    // nucp
    use_min_nucp_check_ = new QCheckBox ("Use Min NUCp");
    connect(use_min_nucp_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMinNUCPSlot);
    child_layout_->addWidget(use_min_nucp_check_, row, 0);

    min_nucp_edit_ = new QLineEdit();
    connect(min_nucp_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::minNUCPEditedSlot);
    child_layout_->addWidget(min_nucp_edit_, row++, 1);

    use_max_nucp_check_ = new QCheckBox ("Use Max NUCp");
    connect(use_max_nucp_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMaxNUCPSlot);
    child_layout_->addWidget(use_max_nucp_check_, row, 0);

    max_nucp_edit_ = new QLineEdit();
    connect(max_nucp_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::maxNUCPEditedSlot);
    child_layout_->addWidget(max_nucp_edit_, row++, 1);

    // others
    use_v1_check_ = new QCheckBox ("Use v1");
    connect(use_v1_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseV1Slot);
    child_layout_->addWidget(use_v1_check_, row++, 0);

    use_v2_check_ = new QCheckBox ("Use v2");
    connect(use_v2_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseV2Slot);
    child_layout_->addWidget(use_v2_check_, row++, 0);

    // nic
    use_min_nic_check_ = new QCheckBox ("Use Min NIC");
    connect(use_min_nic_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMinNICSlot);
    child_layout_->addWidget(use_min_nic_check_, row, 0);

    min_nic_edit_ = new QLineEdit();
    connect(min_nic_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::minNICEditedSlot);
    child_layout_->addWidget(min_nic_edit_, row++, 1);

    use_max_nic_check_ = new QCheckBox ("Use Max NIC");
    connect(use_max_nic_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMaxNICSlot);
    child_layout_->addWidget(use_max_nic_check_, row, 0);

    max_nic_edit_ = new QLineEdit();
    connect(max_nic_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::maxNICEditedSlot);
    child_layout_->addWidget(max_nic_edit_, row++, 1);

    // nacp
    use_min_nacp_check_ = new QCheckBox ("Use Min NACp");
    connect(use_min_nacp_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMinNACpSlot);
    child_layout_->addWidget(use_min_nacp_check_, row, 0);

    min_nacp_edit_ = new QLineEdit();
    connect(min_nacp_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::minNACPEditedSlot);
    child_layout_->addWidget(min_nacp_edit_, row++, 1);

    use_max_nacp_check_ = new QCheckBox ("Use Max NACp");
    connect(use_max_nacp_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMaxNACpSlot);
    child_layout_->addWidget(use_max_nacp_check_, row, 0);

    max_nacp_edit_ = new QLineEdit();
    connect(max_nacp_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::maxNACPEditedSlot);
    child_layout_->addWidget(max_nacp_edit_, row++, 1);

    // sil
    use_min_sil_v1_check_ = new QCheckBox ("Use Min SIL v1");
    connect(use_min_sil_v1_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMinSILv1Slot);
    child_layout_->addWidget(use_min_sil_v1_check_, row, 0);

    min_sil_v1_edit_ = new QLineEdit();
    connect(min_sil_v1_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::minSILv1PEditedSlot);
    child_layout_->addWidget(min_sil_v1_edit_, row++, 1);

    use_max_sil_v1_check_ = new QCheckBox ("Use Max SIL v1");
    connect(use_max_sil_v1_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMaxSILv1Slot);
    child_layout_->addWidget(use_max_sil_v1_check_, row, 0);

    max_sil_v1_edit_ = new QLineEdit();
    connect(max_sil_v1_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::maxSILv1PEditedSlot);
    child_layout_->addWidget(max_sil_v1_edit_, row++, 1);

    use_min_sil_v2_check_ = new QCheckBox ("Use Min SIL v2");
    connect(use_min_sil_v2_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMinSILv2Slot);
    child_layout_->addWidget(use_min_sil_v2_check_, row, 0);

    min_sil_v2_edit_ = new QLineEdit();
    connect(min_sil_v2_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::minSILv2PEditedSlot);
    child_layout_->addWidget(min_sil_v2_edit_, row++, 1);

    use_max_sil_v2_check_ = new QCheckBox ("Use Max SIL v2");
    connect(use_max_sil_v2_check_, &QCheckBox::clicked, this, &ADSBQualityFilterWidget::toggleUseMaxSILv2Slot);
    child_layout_->addWidget(use_max_sil_v2_check_, row, 0);

    max_sil_v2_edit_ = new QLineEdit();
    connect(max_sil_v2_edit_, &QLineEdit::textEdited, this, &ADSBQualityFilterWidget::maxSILv2PEditedSlot);
    child_layout_->addWidget(max_sil_v2_edit_, row++, 1);

    update();
}

ADSBQualityFilterWidget::~ADSBQualityFilterWidget()
{

}

void ADSBQualityFilterWidget::toggleUseV0Slot()
{
    traced_assert(use_v0_check_);
    filter_.useV0(use_v0_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::toggleUseV1Slot()
{
    traced_assert(use_v1_check_);
    filter_.useV1(use_v1_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::toggleUseV2Slot()
{
    traced_assert(use_v2_check_);
    filter_.useV2(use_v2_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::toggleUseMinNUCPSlot()
{
    traced_assert(use_min_nucp_check_);
    filter_.useMinNUCP(use_min_nucp_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::minNUCPEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.minNUCP(val);
}

void ADSBQualityFilterWidget::toggleUseMinNICSlot()
{
    traced_assert(use_min_nic_check_);
    filter_.useMinNIC(use_min_nic_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::minNICEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.minNIC(val);
}

void ADSBQualityFilterWidget::toggleUseMinNACpSlot()
{
    traced_assert(use_min_nacp_check_);
    filter_.useMinNACp(use_min_nacp_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::minNACPEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.minNACp(val);
}

void ADSBQualityFilterWidget::toggleUseMinSILv1Slot()
{
    traced_assert(use_min_sil_v1_check_);
    filter_.useMinSILv1(use_min_sil_v1_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::minSILv1PEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.minSILv1(val);
}


void ADSBQualityFilterWidget::toggleUseMinSILv2Slot()
{
    traced_assert(use_min_sil_v2_check_);
    filter_.useMinSILv2(use_min_sil_v2_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::minSILv2PEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.minSILv2(val);
}

void ADSBQualityFilterWidget::toggleUseMaxNUCPSlot()
{
    traced_assert(use_max_nucp_check_);
    filter_.useMaxNUCP(use_max_nucp_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::maxNUCPEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.maxNUCP(val);
}

void ADSBQualityFilterWidget::toggleUseMaxNICSlot()
{
    traced_assert(use_max_nic_check_);
    filter_.useMaxNIC(use_max_nic_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::maxNICEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.maxNIC(val);
}

void ADSBQualityFilterWidget::toggleUseMaxNACpSlot()
{
    traced_assert(use_max_nacp_check_);
    filter_.useMaxNACp(use_max_nacp_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::maxNACPEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.maxNACp(val);
}

void ADSBQualityFilterWidget::toggleUseMaxSILv1Slot()
{
    traced_assert(use_max_sil_v1_check_);
    filter_.useMaxSILv1(use_max_sil_v1_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::maxSILv1PEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.maxSILv1(val);
}


void ADSBQualityFilterWidget::toggleUseMaxSILv2Slot()
{
    traced_assert(use_max_sil_v2_check_);
    filter_.useMaxSILv2(use_max_sil_v2_check_->checkState() == Qt::Checked);
}

void ADSBQualityFilterWidget::maxSILv2PEditedSlot (const QString& text)
{
    unsigned int val;
    bool ok;

    val = text.toUInt(&ok);

    if (!ok)
        logwrn << "unable to parse value '" << text.toStdString() << "'";
    else
        filter_.maxSILv2(val);
}



void ADSBQualityFilterWidget::update()
{
    DBFilterWidget::update();

    traced_assert(use_v0_check_);
    use_v0_check_->setChecked(filter_.useV0());

    traced_assert(use_min_nucp_check_);
    use_min_nucp_check_->setChecked(filter_.useMinNUCP());
    traced_assert(min_nucp_edit_);
    min_nucp_edit_->setText(QString::number(filter_.minNUCP()));

    traced_assert(use_v1_check_);
    use_v1_check_->setChecked(filter_.useV1());
    traced_assert(use_v2_check_);
    use_v2_check_->setChecked(filter_.useV2());

    traced_assert(use_min_nic_check_);
    use_min_nic_check_->setChecked(filter_.useMinNIC());
    traced_assert(min_nic_edit_);
    min_nic_edit_->setText(QString::number(filter_.minNIC()));

    traced_assert(use_min_nacp_check_);
    use_min_nacp_check_->setChecked(filter_.useMinNACp());
    traced_assert(min_nacp_edit_);
    min_nacp_edit_->setText(QString::number(filter_.minNACp()));

    traced_assert(use_min_sil_v1_check_);
    use_min_sil_v1_check_->setChecked(filter_.useMinSILv1());
    traced_assert(min_sil_v1_edit_);
    min_sil_v1_edit_->setText(QString::number(filter_.minSILv1()));

    traced_assert(use_min_sil_v2_check_);
    use_min_sil_v2_check_->setChecked(filter_.useMinSILv2());
    traced_assert(min_sil_v2_edit_);
    min_sil_v2_edit_->setText(QString::number(filter_.minSILv2()));

    //
    traced_assert(use_max_nucp_check_);
    use_max_nucp_check_->setChecked(filter_.useMaxNUCP());
    traced_assert(max_nucp_edit_);
    max_nucp_edit_->setText(QString::number(filter_.maxNUCP()));

    traced_assert(use_max_nic_check_);
    use_max_nic_check_->setChecked(filter_.useMaxNIC());
    traced_assert(max_nic_edit_);
    max_nic_edit_->setText(QString::number(filter_.maxNIC()));

    traced_assert(use_max_nacp_check_);
    use_max_nacp_check_->setChecked(filter_.useMaxNACp());
    traced_assert(max_nacp_edit_);
    max_nacp_edit_->setText(QString::number(filter_.maxNACp()));

    traced_assert(use_max_sil_v1_check_);
    use_max_sil_v1_check_->setChecked(filter_.useMaxSILv1());
    traced_assert(max_sil_v1_edit_);
    max_sil_v1_edit_->setText(QString::number(filter_.maxSILv1()));

    traced_assert(use_max_sil_v2_check_);
    use_max_sil_v2_check_->setChecked(filter_.useMaxSILv2());
    traced_assert(max_sil_v2_edit_);
    max_sil_v2_edit_->setText(QString::number(filter_.maxSILv2()));
}


