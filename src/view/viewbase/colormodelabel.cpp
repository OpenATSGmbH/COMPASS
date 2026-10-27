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

#include "colormodelabel.h"

#include "color_provider.h"
#include "compass.h"

/**
 */
ColorModeLabel::ColorModeLabel(COMPASS& compass, QWidget* parent)
:   QLabel(parent)
{
    showMode(compass.colorMode());

    connect(&compass, &COMPASS::colorModeChangedSignal, this, &ColorModeLabel::showMode);
}

/**
 */
void ColorModeLabel::showMode(unsigned int mode)
{
    const auto name = context::ColorProvider::modeName((context::ColorProvider::Mode)mode);

    setText("Color Mode: " + QString::fromStdString(name));
}
