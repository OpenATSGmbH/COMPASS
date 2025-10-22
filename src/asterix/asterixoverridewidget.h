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

class ASTERIXImportTask;

class QCheckBox;
class QLineEdit;
class QTimeEdit;

class ASTERIXOverrideWidget : public QWidget
{
    Q_OBJECT

  public slots:
    void updateSlot();

    void overrideActiveCheckedSlot();
    void todOffsetEditedSlot(const QString& value);

    void filterTimeOfDayActiveCheckedSlot();
    void minTimeChanged(QTime time);
    void maxTimeChanged(QTime time);

    void filterPositionRecActiveCheckedSlot();
    void recLatitudeMinEditedSlot(const QString& value_str);
    void recLatitudeMaxEditedSlot(const QString& value_str);
    void recLongitudeMinEditedSlot(const QString& value_str);
    void recLongitudeMaxEditedSlot(const QString& value_str);

    void filterPositionCircActiveCheckedSlot();
    void circLatitudeEditedSlot(const QString& value_str);
    void circLongitudeEditedSlot(const QString& value_str);
    void circRangeEditedSlot(const QString& value_str);

    void filterModeCActiveCheckedSlot();
    void modeCMinEditedSlot(const QString& value_str);
    void modeCMaxEditedSlot(const QString& value_str);

    void obfuscateSecondaryInfoCheckedSlot();

  public:
    ASTERIXOverrideWidget(ASTERIXImportTask& task, QWidget* parent = nullptr);
    virtual ~ASTERIXOverrideWidget();

  protected:
    ASTERIXImportTask& task_;

    QCheckBox* override_active_check_{nullptr};
    QLineEdit* tod_offset_edit_{nullptr};

    QCheckBox* filter_tod_active_check_{nullptr};
    QTimeEdit* filter_tod_min_edit_{nullptr};
    QTimeEdit* filter_tod_max_edit_{nullptr};

    QCheckBox* filter_position_rec_active_check_{nullptr};
    QLineEdit* filter_rec_latitude_min_edit_{nullptr};
    QLineEdit* filter_rec_latitude_max_edit_{nullptr};
    QLineEdit* filter_rec_longitude_min_edit_{nullptr};
    QLineEdit* filter_rec_longitude_max_edit_{nullptr};

    QCheckBox* filter_position_circ_active_check_{nullptr};
    QLineEdit* filter_circ_latitude_edit_{nullptr};
    QLineEdit* filter_circ_longitude_edit_{nullptr};
    QLineEdit* filter_circ_range_edit_{nullptr};

    QCheckBox* filter_modec_active_check_{nullptr};
    QLineEdit* filter_modec_min_edit_{nullptr};
    QLineEdit* filter_modec_max_edit_{nullptr};

    QCheckBox* obfuscate_secondary_info_check_{nullptr};
};
