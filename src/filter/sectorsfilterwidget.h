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

#include "dbfilterwidget.h"
#include "sectorsfilter.h"

#include <vector>

class QCheckBox;
class QLineEdit;
class QVBoxLayout;
class QWidget;

/**
 * Widget of the Sectors filter: one checkbox per sector layer of the active Data Context, the
 * surrounding delta checkbox and its value in degrees.
 */
class SectorsFilterWidget : public DBFilterWidget
{
    Q_OBJECT

protected slots:
    void useDeltaChangedSlot();
    void deltaEditedSlot(const QString& value);

public:
    SectorsFilterWidget(SectorsFilter& filter);
    virtual ~SectorsFilterWidget();

    virtual void update() override;

protected:
    void rebuildLayerChecks();

    static const int Precision = 4;

    SectorsFilter& sectors_filter_;

    QWidget*     layers_widget_ {nullptr};
    QVBoxLayout* layers_layout_ {nullptr};
    QCheckBox*   use_delta_check_ {nullptr};
    QLineEdit*   delta_edit_ {nullptr};
};
