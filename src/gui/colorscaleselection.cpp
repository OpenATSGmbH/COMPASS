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

#include "colorscaleselection.h"
#include "colormap.h"

#include "ui_test_common.h"

#include <QComboBox>
#include <QHBoxLayout>

/**
*/
ColorScaleSelection::ColorScaleSelection(QWidget* parent)
:   QWidget(parent)
{
    auto layout = new QHBoxLayout;
    setLayout(layout);

    layout->setMargin(0);
    layout->setContentsMargins(0, 0, 0, 0);

    combo_ = ColorMap::generateScaleSelection();
    layout->addWidget(combo_);

    UI_TEST_OBJ_NAME(combo_, "combo");

    connect(combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ColorScaleSelection::scaleChanged);
}

/**
*/
void ColorScaleSelection::setSelectedScale(colorscale::ColorScale scale)
{
    int idx = combo_->findData(QVariant((int)scale));
    if (idx < 0)
        return;

    combo_->setCurrentIndex(idx);
}

/**
*/
colorscale::ColorScale ColorScaleSelection::selectedScale() const
{
    return (colorscale::ColorScale)combo_->currentData().toInt();
}

