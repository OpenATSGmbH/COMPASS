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

#include "sectorsfilterwidget.h"
#include "textfielddoublevalidator.h"
#include "logger.h"

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include <algorithm>

/**
 */
SectorsFilterWidget::SectorsFilterWidget(SectorsFilter& filter)
    : DBFilterWidget(filter), sectors_filter_(filter)
{
    const double DeltaMin = 0.0;
    const double DeltaMax = 10.0;

    layers_widget_ = new QWidget();
    layers_widget_->setObjectName("filter_layers_widget");

    layers_layout_ = new QVBoxLayout(layers_widget_);
    layers_layout_->setContentsMargins(0, 0, 0, 0);
    layers_layout_->setSpacing(2);

    addNameValuePair("Sector Layers", layers_widget_);

    use_delta_check_ = new QCheckBox();
    use_delta_check_->setObjectName("filter_use_delta_check");
    connect(use_delta_check_, &QCheckBox::clicked, this, &SectorsFilterWidget::useDeltaChangedSlot);

    addNameValuePair("Surrounding Delta [WGS84]", use_delta_check_);

    delta_edit_ = new QLineEdit();
    delta_edit_->setObjectName("filter_delta_edit");
    delta_edit_->setValidator(new TextFieldDoubleValidator(DeltaMin, DeltaMax, Precision));
    connect(delta_edit_, &QLineEdit::textEdited, this, &SectorsFilterWidget::deltaEditedSlot);

    addNameValuePair("Delta [deg]", delta_edit_);

    update();
}

SectorsFilterWidget::~SectorsFilterWidget() = default;

/**
 */
void SectorsFilterWidget::update()
{
    DBFilterWidget::update();

    traced_assert(use_delta_check_);
    traced_assert(delta_edit_);

    rebuildLayerChecks();

    use_delta_check_->setChecked(sectors_filter_.useDelta());

    delta_edit_->setText(QString::number(sectors_filter_.deltaDeg(), 'f', Precision));
    delta_edit_->setEnabled(sectors_filter_.useDelta());
}

/**
 * Recreates the layer checkboxes from the layers of the active context. Layers of other contexts
 * are not shown, the filter drops them from its selection.
 */
void SectorsFilterWidget::rebuildLayerChecks()
{
    traced_assert(layers_layout_);

    while (QLayoutItem* item = layers_layout_->takeAt(0))
    {
        delete item->widget();
        delete item;
    }

    std::vector<std::string> names    = sectors_filter_.availableLayers();
    std::vector<std::string> selected = sectors_filter_.selectedLayers();

    if (names.empty())
    {
        QLabel* label = new QLabel("No sector layers in the active Data Context");
        label->setObjectName("filter_layers_empty_label");
        label->setEnabled(false);

        layers_layout_->addWidget(label);
        return;
    }

    for (const auto& name : names)
    {
        QCheckBox* check = new QCheckBox(QString::fromStdString(name));
        check->setObjectName(QString::fromStdString("filter_layer_check_" + name));
        check->setChecked(std::find(selected.begin(), selected.end(), name) != selected.end());

        connect(check, &QCheckBox::clicked, this, [this, name](bool checked)
        {
            sectors_filter_.selectLayer(name, checked);
        });

        layers_layout_->addWidget(check);
    }
}

/**
 */
void SectorsFilterWidget::useDeltaChangedSlot()
{
    traced_assert(use_delta_check_);
    traced_assert(delta_edit_);

    sectors_filter_.useDelta(use_delta_check_->isChecked());
    delta_edit_->setEnabled(sectors_filter_.useDelta());
}

/**
 */
void SectorsFilterWidget::deltaEditedSlot(const QString& value)
{
    if (!value.size())
    {
        loginf << "skipping empty string";
        return;
    }

    bool ok = false;
    double delta = value.toDouble(&ok);

    if (!ok)
    {
        logwrn << "invalid delta '" << value.toStdString() << "'";
        return;
    }

    sectors_filter_.deltaDeg(delta);
}
