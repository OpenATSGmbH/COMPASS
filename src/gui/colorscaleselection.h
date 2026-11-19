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

#pragma once

#include "compass_gui_export.h"

#include "colormap_defs.h"

#include <QWidget>

class QComboBox;

/**
 */
class COMPASS_GUI_API ColorScaleSelection : public QWidget
{
    Q_OBJECT
public:
    ColorScaleSelection(QWidget* parent = nullptr);
    virtual ~ColorScaleSelection() = default;

    void setSelectedScale(colorscale::ColorScale scale);
    colorscale::ColorScale selectedScale() const;

signals:
    int scaleChanged();

private:
    QComboBox* combo_ = nullptr;
};
