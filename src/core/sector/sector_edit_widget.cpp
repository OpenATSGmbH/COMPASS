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

#include "sector_edit_widget.h"
#include "sector.h"
#include "logger.h"
#include "textfielddoublevalidator.h"
#include "traced_assert.h"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace context
{

SectorEditWidget::SectorEditWidget(std::function<void()> on_changed,
                                   LayerNamesFunc layer_names_func,
                                   QWidget* parent)
    : QWidget(parent)
    , on_changed_(std::move(on_changed))
    , layer_names_func_(std::move(layer_names_func))
{
    auto* layout = new QVBoxLayout();

    auto* form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::editingFinished, this, &SectorEditWidget::nameEditedSlot);
    form->addRow(new QLabel("Name"), name_edit_);

    layer_combo_ = new QComboBox();
    layer_combo_->setEditable(true);
    // commit on finished edit / dropdown selection only - currentTextChanged fires
    // on every keystroke in an editable combo and would move the sector per character
    connect(layer_combo_->lineEdit(), &QLineEdit::editingFinished, this, &SectorEditWidget::layerChangedSlot);
    connect(layer_combo_, QOverload<int>::of(&QComboBox::activated), this, &SectorEditWidget::layerChangedSlot);
    form->addRow(new QLabel("Layer"), layer_combo_);

    exclude_check_ = new QCheckBox();
    connect(exclude_check_, &QCheckBox::clicked, this, &SectorEditWidget::excludeChangedSlot);
    form->addRow(new QLabel("Exclusion Sector"), exclude_check_);

    color_button_ = new QPushButton();
    color_button_->setFixedHeight(24);
    connect(color_button_, &QPushButton::clicked, this, &SectorEditWidget::colorClickedSlot);
    form->addRow(new QLabel("Color"), color_button_);

    alt_min_edit_ = new QLineEdit();
    alt_min_edit_->setValidator(new TextFieldDoubleValidator(-10000, 100000, 1));
    alt_min_edit_->setPlaceholderText("none");
    connect(alt_min_edit_, &QLineEdit::editingFinished, this, &SectorEditWidget::altMinEditedSlot);
    form->addRow(new QLabel("Altitude Min [ft]"), alt_min_edit_);

    alt_max_edit_ = new QLineEdit();
    alt_max_edit_->setValidator(new TextFieldDoubleValidator(-10000, 100000, 1));
    alt_max_edit_->setPlaceholderText("none");
    connect(alt_max_edit_, &QLineEdit::editingFinished, this, &SectorEditWidget::altMaxEditedSlot);
    form->addRow(new QLabel("Altitude Max [ft]"), alt_max_edit_);

    points_label_ = new QLabel();
    form->addRow(new QLabel("Points"), points_label_);

    layout->addLayout(form);
    layout->addStretch();

    setLayout(layout);
}

void SectorEditWidget::show(Sector& sector)
{
    loginf << "showing sector '" << sector.name() << "' id " << sector.id();

    current_sector_ = &sector;

    // block signals during population
    name_edit_->blockSignals(true);
    layer_combo_->blockSignals(true);
    exclude_check_->blockSignals(true);

    name_edit_->setText(QString::fromStdString(sector.name()));

    // populate layer combo with existing layers
    layer_combo_->clear();
    if (layer_names_func_)
    {
        for (const auto& name : layer_names_func_())
            layer_combo_->addItem(QString::fromStdString(name));
    }
    layer_combo_->setCurrentText(QString::fromStdString(sector.layerName()));

    exclude_check_->setChecked(sector.isExclusionSector());

    // color button
    QColor color(QString::fromStdString(sector.colorStr()));
    color_button_->setStyleSheet("background-color: " + color.name() + ";");
    color_button_->setText(color.name());

    // altitude
    if (sector.hasMinimumAltitude())
        alt_min_edit_->setText(QString::number(sector.minimumAltitude(), 'f', 1));
    else
        alt_min_edit_->setText("");

    if (sector.hasMaximumAltitude())
        alt_max_edit_->setText(QString::number(sector.maximumAltitude(), 'f', 1));
    else
        alt_max_edit_->setText("");

    points_label_->setText(QString::number(sector.size()) + " points");

    name_edit_->blockSignals(false);
    layer_combo_->blockSignals(false);
    exclude_check_->blockSignals(false);
}

unsigned int SectorEditWidget::currentSectorId() const
{
    traced_assert(current_sector_);
    return current_sector_->id();
}

void SectorEditWidget::clear()
{
    current_sector_ = nullptr;

    name_edit_->blockSignals(true);
    layer_combo_->blockSignals(true);

    name_edit_->clear();
    layer_combo_->clear();
    exclude_check_->setChecked(false);
    color_button_->setStyleSheet("");
    color_button_->setText("");
    alt_min_edit_->clear();
    alt_max_edit_->clear();
    points_label_->setText("");

    name_edit_->blockSignals(false);
    layer_combo_->blockSignals(false);
}

void SectorEditWidget::setReadOnly(bool read_only)
{
    read_only_ = read_only;

    name_edit_->setReadOnly(read_only);
    layer_combo_->setEnabled(!read_only);
    exclude_check_->setEnabled(!read_only);
    color_button_->setEnabled(!read_only);
    alt_min_edit_->setReadOnly(read_only);
    alt_max_edit_->setReadOnly(read_only);
}

void SectorEditWidget::nameEditedSlot()
{
    if (!current_sector_ || read_only_)
        return;

    std::string new_name = name_edit_->text().trimmed().toStdString();
    if (new_name.empty() || new_name == current_sector_->name())
        return;

    loginf << "renaming sector " << current_sector_->id() << " to '" << new_name << "'";

    current_sector_->name(new_name);

    if (on_changed_)
        on_changed_();
}

void SectorEditWidget::layerChangedSlot()
{
    if (!current_sector_ || read_only_)
        return;

    std::string new_layer = layer_combo_->currentText().trimmed().toStdString();
    if (new_layer.empty() || new_layer == current_sector_->layerName())
        return;

    loginf << "moving sector " << current_sector_->id() << " to layer '" << new_layer << "'";

    current_sector_->layerName(new_layer);

    if (on_changed_)
        on_changed_();
}

void SectorEditWidget::excludeChangedSlot()
{
    if (!current_sector_ || read_only_)
        return;

    loginf << "setting sector " << current_sector_->id() << " exclude=" << exclude_check_->isChecked();

    current_sector_->exclude(exclude_check_->isChecked());

    if (on_changed_)
        on_changed_();
}

void SectorEditWidget::colorClickedSlot()
{
    if (!current_sector_ || read_only_)
        return;

    QColor current(QString::fromStdString(current_sector_->colorStr()));
    QColor color = QColorDialog::getColor(current, this, "Sector Color");

    if (!color.isValid())
        return;

    loginf << "setting sector " << current_sector_->id() << " color=" << color.name().toStdString();

    current_sector_->colorStr(color.name().toStdString());

    color_button_->setStyleSheet("background-color: " + color.name() + ";");
    color_button_->setText(color.name());

    if (on_changed_)
        on_changed_();
}

void SectorEditWidget::altMinEditedSlot()
{
    if (!current_sector_ || read_only_)
        return;

    QString text = alt_min_edit_->text().trimmed();
    if (text.isEmpty())
    {
        if (current_sector_->hasMinimumAltitude())
        {
            current_sector_->removeMinimumAltitude();
            if (on_changed_)
                on_changed_();
        }
    }
    else
    {
        bool ok = false;
        double val = text.toDouble(&ok);
        if (ok)
        {
            current_sector_->setMinimumAltitude(val);
            if (on_changed_)
                on_changed_();
        }
    }
}

void SectorEditWidget::altMaxEditedSlot()
{
    if (!current_sector_ || read_only_)
        return;

    QString text = alt_max_edit_->text().trimmed();
    if (text.isEmpty())
    {
        if (current_sector_->hasMaximumAltitude())
        {
            current_sector_->removeMaximumAltitude();
            if (on_changed_)
                on_changed_();
        }
    }
    else
    {
        bool ok = false;
        double val = text.toDouble(&ok);
        if (ok)
        {
            current_sector_->setMaximumAltitude(val);
            if (on_changed_)
                on_changed_();
        }
    }
}

} // namespace context
