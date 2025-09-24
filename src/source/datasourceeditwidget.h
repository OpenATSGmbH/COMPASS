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

#include <qcheckbox.h>
#include <QWidget>

class DataSourceManager;
class DataSourcesConfigurationDialog;
class DSTypeSelectionComboBox;

namespace dbContent
{
    class DataSourceBase;
}

class QLabel;
class QLineEdit;
class QPushButton;
class QGridLayout;
class QComboBox;
class QCheckBox;

class DataSourceEditWidget : public QWidget
{
    Q_OBJECT

public slots:
    void nameEditedSlot(const QString& value);
    void shortNameEditedSlot(const QString& value);
    void dsTypeEditedSlot(const QString& value);

    void updateIntervalEditedSlot(const QString& value_str);

    void detectionTypeChangedSlot(int index); // Slot to handle detection type change

    void groundOnlyCheckedSlot();

    void latitudeEditedSlot(const QString& value_str);
    void longitudeEditedSlot(const QString& value_str);
    void altitudeEditedSlot(const QString& value_str);

    void pdEditedSlot(const QString& value_str);
    void clutterRateEditedSlot(const QString& value_str);

    void addRadarRangesSlot();
    void radarRangeEditedSlot(const QString& value_str);

    void addRadarAccuraciesSlot();
    void radarAccuraciesEditedSlot(const QString& value_str);

    void addNetLinesSlot();
    void netLineEditedSlot(const QString& value_str);

    void deleteSlot();

public:
    DataSourceEditWidget(bool show_network_lines, DataSourceManager& ds_man, std::function<void(unsigned int)> update_ds_func,
        std::function<void(unsigned int)> delete_ds_func);

    void showID(unsigned int ds_id);
    void clear();

    void updateContent();

protected:
    bool show_network_lines_;

    DataSourceManager& ds_man_;
    std::function<void(unsigned int)> update_ds_func_;
    std::function<void(unsigned int)> delete_ds_func_;

    bool has_current_ds_ {false};
    unsigned int current_ds_id_ {0};
    bool current_ds_in_db_ {false};

    QLineEdit* name_edit_{nullptr};
    QLineEdit* short_name_edit_{nullptr};

    DSTypeSelectionComboBox* dstype_combo_{nullptr};

    QLabel* sac_sic_id_label_{nullptr};

    // update_interval
    QLineEdit* update_interval_edit_{nullptr};

    QComboBox* detection_type_combo_{nullptr};
    QCheckBox* ground_only_check_{nullptr}; 

    // position
    QWidget* position_widget_{nullptr};
    QLineEdit* latitude_edit_{nullptr};
    QLineEdit* longitude_edit_{nullptr};
    QLineEdit* altitude_edit_{nullptr};

    // psr settings
    QWidget* psr_jpda_widget_{nullptr};
    QLineEdit* psr_pd_edit_{nullptr};
    QLineEdit* psr_clutter_rate_edit_{nullptr};    

    // radar ranges
    QWidget* ranges_widget_{nullptr};
    QLineEdit* psr_min_edit_{nullptr};
    QLineEdit* psr_max_edit_{nullptr};
    QLineEdit* ssr_min_edit_{nullptr};
    QLineEdit* ssr_max_edit_{nullptr};
    QLineEdit* mode_s_min_edit_{nullptr};
    QLineEdit* mode_s_max_edit_{nullptr};

    QPushButton* add_ranges_button_{nullptr};

    // radar accuracies
    QWidget* accuracies_widget_{nullptr};
    QLineEdit* acc_psr_azm_edit_{nullptr};
    QLineEdit* acc_psr_rng_edit_{nullptr};
    QLineEdit* acc_ssr_azm_edit_{nullptr};
    QLineEdit* acc_ssr_rng_edit_{nullptr};
    QLineEdit* acc_mode_s_azm_edit_{nullptr};
    QLineEdit* acc_mode_s_rng_edit_{nullptr};

    QPushButton* add_accuracies_button_{nullptr};

    // net lines
    QPushButton* add_lines_button_{nullptr};

    QWidget* net_widget_{nullptr};
    std::map<std::string, std::vector<QLineEdit*>> net_edits_; // L1 -> edits (listen, mcastip, mcastport, sender)

    QPushButton* delete_button_{nullptr};

    void disableAll();
    void updateMain(dbContent::DataSourceBase* ds);
    void updatePosition(dbContent::DataSourceBase* ds);
    void updateRadar(dbContent::DataSourceBase* ds);
    void updateNetwork(dbContent::DataSourceBase* ds);
};

