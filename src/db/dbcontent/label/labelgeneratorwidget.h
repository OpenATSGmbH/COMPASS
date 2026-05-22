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

#include <QWidget>

class QCheckBox;
class QLineEdit;

namespace dbContent
{

class LabelGenerator;
class LabelDSWidget;

// is not owned by LabelGenerator, can be deleted
// TODO multiple instances do not update each other
class LabelGeneratorWidget : public QWidget
{
    Q_OBJECT

public slots:
    void editSettingsSlot();
    void editDBContentSlot();

    void toggleUseUTNSlot();

    void labelAllDSSlot();
    void labelNoDSSlot();

    void autoLabelChangedSlot(bool checked);
    void lodChangedSlot(const QString& text);

    void declutterLabelsChangedSlot(bool checked);

    void filterMode3AActiveChangedSlot(bool checked);
    void filterMode3AChangedSlot(const QString& text);

    void filterModeCMinActiveChangedSlot(bool checked);
    void filterModeCMinChangedSlot(const QString& text);
    void filterModeCMaxActiveChangedSlot(bool checked);
    void filterModeCMaxChangedSlot(const QString& text);
    void filterModeCNullWantedChangedSlot(bool checked);

    void filterTIActiveChangedSlot(bool checked);
    void filterTIChangedSlot(const QString& text);

    void filterTAActiveChangedSlot(bool checked);
    void filterTAChangedSlot(const QString& text);

    void filterPSROnlyActiveChangedSlot(bool checked);

    void opacitySliderChangedSlot(int value);

public:
    LabelGeneratorWidget(LabelGenerator& label_generator);
    virtual ~LabelGeneratorWidget();

protected:
    LabelGenerator& label_generator_;

    LabelDSWidget* label_ds_widget_ {nullptr};
};

}
