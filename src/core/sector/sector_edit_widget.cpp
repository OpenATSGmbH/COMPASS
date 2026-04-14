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
#include "db_context_manager.h"
#include "sector.h"
#include "sectorlayer.h"
#include "logger.h"
#include "traced_assert.h"
#include "textfielddoublevalidator.h"

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

SectorEditWidget::SectorEditWidget(DBContextManager& manager,
                                   std::function<void()> on_changed,
                                   QWidget* parent)
    : QWidget(parent)
    , manager_(manager)
    , on_changed_(std::move(on_changed))
{
    auto* layout = new QVBoxLayout();

    auto* form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::editingFinished, this, &SectorEditWidget::nameEditedSlot);
    form->addRow(new QLabel("Name"), name_edit_);

    layer_combo_ = new QComboBox();
    layer_combo_->setEditable(true);
    connect(layer_combo_, &QComboBox::currentTextChanged, this, &SectorEditWidget::layerChangedSlot);
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

void SectorEditWidget::showSector(unsigned int sector_id)
{
    loginf << "showing sector id " << sector_id;

    if (!manager_.hasSector(sector_id))
    {
        clear();
        return;
    }

    has_current_ = true;
    current_sector_id_ = sector_id;

    auto sector = manager_.sector(sector_id);

    // block signals during population
    name_edit_->blockSignals(true);
    layer_combo_->blockSignals(true);
    exclude_check_->blockSignals(true);

    name_edit_->setText(QString::fromStdString(sector->name()));

    // populate layer combo with existing layers
    layer_combo_->clear();
    for (const auto& layer : manager_.sectorLayers())
        layer_combo_->addItem(QString::fromStdString(layer->name()));
    layer_combo_->setCurrentText(QString::fromStdString(sector->layerName()));

    exclude_check_->setChecked(sector->isExclusionSector());

    // color button
    QColor color(QString::fromStdString(sector->colorStr()));
    color_button_->setStyleSheet("background-color: " + color.name() + ";");
    color_button_->setText(color.name());

    // altitude
    if (sector->hasMinimumAltitude())
        alt_min_edit_->setText(QString::number(sector->minimumAltitude(), 'f', 1));
    else
        alt_min_edit_->setText("");

    if (sector->hasMaximumAltitude())
        alt_max_edit_->setText(QString::number(sector->maximumAltitude(), 'f', 1));
    else
        alt_max_edit_->setText("");

    points_label_->setText(QString::number(sector->size()) + " points");

    name_edit_->blockSignals(false);
    layer_combo_->blockSignals(false);
    exclude_check_->blockSignals(false);
}

void SectorEditWidget::clear()
{
    has_current_ = false;
    current_sector_id_ = 0;

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

void SectorEditWidget::saveCurrent()
{
    if (!has_current_)
        return;

    manager_.saveSector(current_sector_id_);
}

void SectorEditWidget::nameEditedSlot()
{
    if (!has_current_)
        return;

    auto sector = manager_.sector(current_sector_id_);
    if (!sector)
        return;

    std::string new_name = name_edit_->text().trimmed().toStdString();
    if (new_name.empty() || new_name == sector->name())
        return;

    loginf << "renaming sector " << current_sector_id_ << " to '" << new_name << "'";

    sector->name(new_name);
    saveCurrent();

    if (on_changed_)
        on_changed_();
}

void SectorEditWidget::layerChangedSlot()
{
    if (!has_current_)
        return;

    auto sector = manager_.sector(current_sector_id_);
    if (!sector)
        return;

    std::string new_layer = layer_combo_->currentText().trimmed().toStdString();
    if (new_layer.empty() || new_layer == sector->layerName())
        return;

    loginf << "moving sector " << current_sector_id_ << " to layer '" << new_layer << "'";

    std::string old_layer = sector->layerName();
    manager_.moveSector(current_sector_id_, old_layer, new_layer);

    // re-show to update layer combo entries
    showSector(current_sector_id_);

    if (on_changed_)
        on_changed_();
}

void SectorEditWidget::excludeChangedSlot()
{
    if (!has_current_)
        return;

    auto sector = manager_.sector(current_sector_id_);
    if (!sector)
        return;

    loginf << "setting sector " << current_sector_id_ << " exclude=" << exclude_check_->isChecked();

    sector->exclude(exclude_check_->isChecked());
    saveCurrent();

    if (on_changed_)
        on_changed_();
}

void SectorEditWidget::colorClickedSlot()
{
    if (!has_current_)
        return;

    auto sector = manager_.sector(current_sector_id_);
    if (!sector)
        return;

    QColor current(QString::fromStdString(sector->colorStr()));
    QColor color = QColorDialog::getColor(current, this, "Sector Color");

    if (!color.isValid())
        return;

    loginf << "setting sector " << current_sector_id_ << " color=" << color.name().toStdString();

    sector->colorStr(color.name().toStdString());
    saveCurrent();

    color_button_->setStyleSheet("background-color: " + color.name() + ";");
    color_button_->setText(color.name());
}

void SectorEditWidget::altMinEditedSlot()
{
    if (!has_current_)
        return;

    auto sector = manager_.sector(current_sector_id_);
    if (!sector)
        return;

    QString text = alt_min_edit_->text().trimmed();
    if (text.isEmpty())
    {
        if (sector->hasMinimumAltitude())
        {
            sector->removeMinimumAltitude();
            saveCurrent();
        }
    }
    else
    {
        bool ok = false;
        double val = text.toDouble(&ok);
        if (ok)
        {
            sector->setMinimumAltitude(val);
            saveCurrent();
        }
    }
}

void SectorEditWidget::altMaxEditedSlot()
{
    if (!has_current_)
        return;

    auto sector = manager_.sector(current_sector_id_);
    if (!sector)
        return;

    QString text = alt_max_edit_->text().trimmed();
    if (text.isEmpty())
    {
        if (sector->hasMaximumAltitude())
        {
            sector->removeMaximumAltitude();
            saveCurrent();
        }
    }
    else
    {
        bool ok = false;
        double val = text.toDouble(&ok);
        if (ok)
        {
            sector->setMaximumAltitude(val);
            saveCurrent();
        }
    }
}

} // namespace context
