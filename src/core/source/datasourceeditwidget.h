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

#include <functional>

namespace context { class DBContextManager; class DataSource; }
class DataSourcesConfigurationDialog;
class DSTypeSelectionComboBox;


class QTabWidget;
class QLabel;
class QLineEdit;
class QPushButton;
class QGridLayout;
class QComboBox;
class QCheckBox;
class QFormLayout;
class QVBoxLayout;
class QTreeWidget;

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
    void ignoreRadarAzmRangeCheckedSlot();

    void latitudeEditedSlot(const QString& value_str);
    void longitudeEditedSlot(const QString& value_str);
    void altitudeEditedSlot(const QString& value_str);

    void pdEditedSlot(const QString& value_str);
    void clutterRateEditedSlot(const QString& value_str);

    void addRadarRangesSlot();
    void radarRangeEditedSlot(const QString& value_str);

    void radarAccuraciesEditedSlot(const QString& value_str);

    void addRadarBiasSlot();
    void radarBiasEditedSlot(const QString& value_str);

    void addMLATRemoteUnitsSlot(); 
    void addMLATRemoteUnitSlot();
    void importMLATRemoteUnitsSlot();
    void clearMLATRemoteUnitsSlot();
    void showRemoteUnitContextMenuSlot(const QPoint& pos);
    void clearSelectedMLATRemoteUnitsSlot();
    void editMLATRemoteUnitSlot();

    void addNetLinesSlot();
    void netLineEditedSlot(const QString& value_str);

    void deleteSlot();

public:
    DataSourceEditWidget(bool show_network_lines,
                         std::function<void(unsigned int)> update_ds_func,
                         std::function<void(unsigned int)> delete_ds_func,
                         context::DBContextManager* ctx_man = nullptr);

    void show(context::DataSource& ds, const std::string& last_used_path = {});
    void clear();
    void setReadOnly(bool read_only);

    void updateContent();

    static const std::string TabMainName;
    static const std::string TabRadarRangesName;
    static const std::string TabRadarAccuraciesName;
    static const std::string TabMLATRemoteUnitsName;
    static const std::string TabNetworkLinesName;

protected:
    bool show_network_lines_;

    std::function<void(unsigned int)> update_ds_func_;
    std::function<void(unsigned int)> delete_ds_func_;

    context::DBContextManager* ctx_man_ {nullptr};

    context::DataSource* current_ds_ {nullptr};
    std::string last_used_path_;
    bool read_only_ {false};

    QTabWidget* tab_widget_ {nullptr};

    QLineEdit* name_edit_{nullptr};
    QLineEdit* short_name_edit_{nullptr};

    DSTypeSelectionComboBox* dstype_combo_{nullptr};

    QLabel* sac_sic_id_label_{nullptr};

    // update_interval
    QLineEdit* update_interval_edit_{nullptr};

    QComboBox* detection_type_combo_{nullptr};
    QCheckBox* ground_only_check_{nullptr}; 
    
    // radar-specific (label + field hidden/shown together)
    QLabel* radar_ignore_label_{nullptr};
    QCheckBox* radar_ignore_azmrng_check_{nullptr};

    // position (labels for show/hide)
    QLabel* latitude_label_{nullptr};
    QLineEdit* latitude_edit_{nullptr};
    QLabel* longitude_label_{nullptr};
    QLineEdit* longitude_edit_{nullptr};
    QLabel* altitude_label_{nullptr};
    QLineEdit* altitude_edit_{nullptr};

    // psr settings (labels for show/hide)
    QLabel* psr_pd_label_{nullptr};
    QLineEdit* psr_pd_edit_{nullptr};
    QLabel* psr_clutter_rate_label_{nullptr};
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

    // radar bias
    QWidget* bias_widget_{nullptr};
    QLineEdit* bias_range_edit_{nullptr};
    QLineEdit* bias_range_stddev_edit_{nullptr};
    QLineEdit* bias_range_gain_edit_{nullptr};
    QLineEdit* bias_range_gain_stddev_edit_{nullptr};
    QLineEdit* bias_azimuth_edit_{nullptr};
    QLineEdit* bias_azimuth_stddev_edit_{nullptr};

    QPushButton* add_bias_button_{nullptr};

    // net lines
    QWidget* net_widget_{nullptr};
    std::map<std::string, std::vector<QLineEdit*>> net_edits_; // L1 -> edits (listen, mcastip, mcastport, sender)

    QPushButton* add_lines_button_{nullptr};

    // mlat remote units
    QWidget* remote_units_widget_{nullptr};
    QTreeWidget* remote_units_list_{nullptr};
    QPushButton* ru_add_button_{nullptr};
    QPushButton* ru_import_button_{nullptr};
    QPushButton* ru_clear_button_{nullptr};

    QPushButton* add_remote_units_button_{nullptr};
    QWidget* add_remote_units_placeholder_{nullptr};

    QPushButton* delete_button_{nullptr};

    std::map<std::string, int> tab_map_;

    context::DataSource& currentDataSource();

    QVBoxLayout* createTab(const std::string& name, bool has_scroll_area);
    int tabIndex(const std::string& name) const;

    void createUI();
    void createMainTab();
    void createNetworkTab();
    void createRadarRangesTab();
    void createRadarAccuraciesTab();
    void createRemoteUnitsTab();

    void enableAll(bool enable);
    void enableCommon(bool enable);
    void enableRadar(bool enable);
    void enableMLAT(bool enable);
    void enableNetwork(bool enable);

    void updateMain(context::DataSource* ds);
    void updatePosition(context::DataSource* ds);
    void updateRadar(context::DataSource* ds);
    void updateMLAT(context::DataSource* ds);
    void updateNetwork(context::DataSource* ds);

    bool editRemoteUnit(int idx);

    void setTabVisibleCompat(int index, bool visible);
};
