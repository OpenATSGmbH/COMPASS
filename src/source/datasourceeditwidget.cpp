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

#include "datasourceeditwidget.h"
#include "datasourcemanager.h"
#include "configurationdatasource.h"
#include "dbdatasource.h"
#include "dstypeselectioncombobox.h"
#include "datasourcesconfigurationdialog.h"
#include "logger.h"
#include "textfielddoublevalidator.h"
#include "datasourcebase.h"
#include "number.h"
#include "files.h"
#include "datasourceremoteunit.h"

#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTreeWidget>
#include <QHeaderView>
#include <QMessageBox>
#include <QScrollArea>
#include <QFormLayout>
#include <QFileDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QMenu>

using namespace std;
using namespace dbContent;
using namespace Utils;

const std::string DataSourceEditWidget::TabMainName            = "Main";
const std::string DataSourceEditWidget::TabRadarRangesName     = "Ranges";
const std::string DataSourceEditWidget::TabRadarAccuraciesName = "Accuracies";
const std::string DataSourceEditWidget::TabMLATRemoteUnitsName = "Remote Units";
const std::string DataSourceEditWidget::TabNetworkLinesName    = "Network";

DataSourceEditWidget::DataSourceEditWidget(bool show_network_lines, DataSourceManager& ds_man, 
    std::function<void(unsigned int)> update_ds_func, std::function<void(unsigned int)> delete_ds_func)
    : show_network_lines_(show_network_lines), ds_man_(ds_man),
    update_ds_func_(update_ds_func), delete_ds_func_(delete_ds_func)
{
    //setMaximumWidth(400);

    createUI();
}

void DataSourceEditWidget::createUI()
{
    QVBoxLayout* layout = new QVBoxLayout;
    setLayout(layout);

    tab_widget_ = new QTabWidget(this);
    layout->addWidget(tab_widget_);

    createMainTab();
    createRadarRangesTab();
    createRadarAccuraciesTab();
    createRemoteUnitsTab();
    createNetworkTab();

    updateContent();
}

QVBoxLayout* DataSourceEditWidget::createTab(const std::string& name, bool has_scroll_area)
{
    traced_assert(tab_widget_);
    traced_assert(tab_map_.count(name) == 0);

    QWidget* main_widget = new QWidget();
    QVBoxLayout* main_layout = new QVBoxLayout();

    main_widget->setLayout(main_layout);

    int idx = -1;

    if (has_scroll_area)
    {
        QScrollArea* scroll_area = new QScrollArea();
        scroll_area->setWidgetResizable(true);
        scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

        scroll_area->setWidget(main_widget);

        idx = tab_widget_->addTab(scroll_area, QString::fromStdString(name));
    }
    else
    {
        idx = tab_widget_->addTab(main_widget, QString::fromStdString(name));
    }

    tab_map_[ name ] = idx;

    return main_layout;
}

int DataSourceEditWidget::tabIndex(const std::string& name) const
{
    traced_assert(tab_map_.count(name) > 0);
    return tab_map_.at(name);
}

void DataSourceEditWidget::createMainTab()
{
    auto main_layout = createTab(TabMainName, true);

    QGridLayout* properties_layout_ = new QGridLayout();

    unsigned int row = 0;

    // "Name", "Short Name", "DSType", "SAC", "SIC"

    //name_edit_
    properties_layout_->addWidget(new QLabel("Name"), row, 0);

    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::nameEditedSlot);
    properties_layout_->addWidget(name_edit_, row, 1);
    row++;

    //short_name_edit_

    properties_layout_->addWidget(new QLabel("Short Name"), row, 0);

    short_name_edit_ = new QLineEdit();
    connect(short_name_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::shortNameEditedSlot);
    properties_layout_->addWidget(short_name_edit_, row, 1);
    row++;

    // dstype_combo_

    properties_layout_->addWidget(new QLabel("DSType"), row, 0);

    dstype_combo_ = new DSTypeSelectionComboBox();
    connect(dstype_combo_, &DSTypeSelectionComboBox::changedTypeSignal, this, &DataSourceEditWidget::dsTypeEditedSlot);
    properties_layout_->addWidget(dstype_combo_, row, 1);
    row++;

    //QLabel* sac_label_{nullptr};

    properties_layout_->addWidget(new QLabel("SAC/SIC    (DS ID)"), row, 0);

    sac_sic_id_label_ = new QLabel();
    properties_layout_->addWidget(sac_sic_id_label_, row, 1);
    row++;

    // update interval

    properties_layout_->addWidget(new QLabel("Update Interval [s]"), row, 0);

    update_interval_edit_ = new QLineEdit();
    update_interval_edit_->setValidator(new TextFieldDoubleValidator(0, 90, 3));
    connect(update_interval_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::updateIntervalEditedSlot);
    properties_layout_->addWidget(update_interval_edit_, row, 1);

    main_layout->addLayout(properties_layout_);

    ++row;

    detection_type_combo_ = new QComboBox(this);
    detection_type_combo_->addItem("Undefined");
    detection_type_combo_->addItem("Primary Only");
    detection_type_combo_->addItem("Mode A/C");
    detection_type_combo_->addItem("Mode A/C Combined");
    detection_type_combo_->addItem("Mode S");
    detection_type_combo_->addItem("Mode S Combined");

    properties_layout_->addWidget(new QLabel("Detection Type"), row, 0);
    properties_layout_->addWidget(detection_type_combo_, row, 1);

    connect(detection_type_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DataSourceEditWidget::detectionTypeChangedSlot);

    // ground_only_check_

    ++row;
    properties_layout_->addWidget(new QLabel("Ground Only"), row, 0);
    
    ground_only_check_ = new QCheckBox();
    connect(ground_only_check_, &QCheckBox::clicked,
            this, &DataSourceEditWidget::groundOnlyCheckedSlot);

    properties_layout_->addWidget(ground_only_check_, row, 1);

    //  radar_widget_
    radar_widget_ = new QWidget();
    radar_widget_->setContentsMargins(0, 0, 0, 0);

    QGridLayout* radar_layout = new QGridLayout();
    radar_layout->setContentsMargins(0, 0, 0, 0);

    radar_layout->addWidget(new QLabel("Ignore Range/Azimuth Values"), 0, 0);

    radar_ignore_azmrng_check_ = new QCheckBox();
    connect(radar_ignore_azmrng_check_, &QCheckBox::clicked, this,
            &DataSourceEditWidget::ignoreRadarAzmRangeCheckedSlot);

    radar_layout->addWidget(radar_ignore_azmrng_check_, 0, 1);

    radar_widget_->setLayout(radar_layout);

    main_layout->addWidget(radar_widget_);

    // position_widget_

    position_widget_ = new QWidget();
    position_widget_->setContentsMargins(0, 0, 0, 0);

    QGridLayout* position_layout = new QGridLayout();
    position_layout->setContentsMargins(0, 0, 0, 0);

    position_layout->addWidget(new QLabel("Latitude"), 0, 0);

    latitude_edit_ = new QLineEdit();
    //latitude_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 12));
    connect(latitude_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::latitudeEditedSlot);
    position_layout->addWidget(latitude_edit_, 0, 1);

    position_layout->addWidget(new QLabel("Longitude"), 1, 0);

    longitude_edit_ = new QLineEdit();
    //longitude_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 12));
    connect(longitude_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::longitudeEditedSlot);
    position_layout->addWidget(longitude_edit_, 1, 1);

    position_layout->addWidget(new QLabel("Altitude"), 2, 0);

    altitude_edit_ = new QLineEdit();
    altitude_edit_->setValidator(new TextFieldDoubleValidator(-10000, 10000, 12));
    connect(altitude_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::altitudeEditedSlot);
    position_layout->addWidget(altitude_edit_, 2, 1);

    position_widget_->setLayout(position_layout);

    main_layout->addWidget(position_widget_);

    // psr settings
    {
        psr_jpda_widget_ = new QWidget();
        psr_jpda_widget_->setContentsMargins(0, 0, 0, 0);

        QGridLayout* jpda_layout = new QGridLayout();
        unsigned int row_cnt = 0;

        jpda_layout->addWidget(new QLabel("JPDA PD [1]"), row_cnt, 0);

        psr_pd_edit_ = new QLineEdit();
        psr_pd_edit_->setValidator(new TextFieldDoubleValidator(0.001, 1, 3));
        connect(psr_pd_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::pdEditedSlot);
        jpda_layout->addWidget(psr_pd_edit_, row_cnt, 1);

        ++row_cnt;

        jpda_layout->addWidget(new QLabel("Clutter Rate [1]"), row_cnt, 0);

        psr_clutter_rate_edit_ = new QLineEdit();
        psr_clutter_rate_edit_->setValidator(new TextFieldDoubleValidator(1, 10000, 0));
        connect(psr_clutter_rate_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::clutterRateEditedSlot);
        jpda_layout->addWidget(psr_clutter_rate_edit_, row_cnt, 1);

        psr_jpda_widget_->setLayout(jpda_layout);

        main_layout->addWidget(psr_jpda_widget_);
    }

    delete_button_ = new QPushButton("Delete");
    delete_button_->setToolTip("Deletes the data source in configuration");
    connect(delete_button_, &QPushButton::clicked, this, &DataSourceEditWidget::deleteSlot);
    main_layout->addWidget(delete_button_);

    //hide for now
    delete_button_->setHidden(true);

    main_layout->addStretch();
}

void DataSourceEditWidget::createRadarRangesTab()
{
    auto main_layout = createTab(TabRadarRangesName, true);

    ranges_widget_ = new QWidget();
    ranges_widget_->setContentsMargins(0, 0, 0, 0);

    QFont font_bold;
    font_bold.setBold(true);

    auto ranges_header_label = new QLabel("Radar Ranges [nm]");
    ranges_header_label->setFont(font_bold);

    QGridLayout* ranges_layout = new QGridLayout();
    unsigned int row_cnt = 0;
    ranges_layout->addWidget(ranges_header_label, row_cnt, 0, 1, 2);

    // psr
    ++row_cnt;
    ranges_layout->addWidget(new QLabel("PSR Minimum"), row_cnt, 0);

    psr_min_edit_ = new QLineEdit();
    psr_min_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 2));
    psr_min_edit_->setProperty("key", DataSourceBase::PSRIRMinKey.c_str());
    connect(psr_min_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarRangeEditedSlot);
    ranges_layout->addWidget(psr_min_edit_, row_cnt, 1);

    ++row_cnt;
    ranges_layout->addWidget(new QLabel("PSR Maximum"), row_cnt, 0);

    psr_max_edit_ = new QLineEdit();
    psr_max_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 2));
    psr_max_edit_->setProperty("key", DataSourceBase::PSRIRMaxKey.c_str());
    connect(psr_max_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarRangeEditedSlot);
    ranges_layout->addWidget(psr_max_edit_, row_cnt, 1);

    // ssr
    ++row_cnt;
    ranges_layout->addWidget(new QLabel("SSR Minimum"), row_cnt, 0);

    ssr_min_edit_ = new QLineEdit();
    ssr_min_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 2));
    ssr_min_edit_->setProperty("key", DataSourceBase::SSRIRMinKey.c_str());
    connect(ssr_min_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarRangeEditedSlot);
    ranges_layout->addWidget(ssr_min_edit_, row_cnt, 1);

    ++row_cnt;
    ranges_layout->addWidget(new QLabel("SSR Maximum"), row_cnt, 0);

    ssr_max_edit_ = new QLineEdit();
    ssr_max_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 2));
    ssr_max_edit_->setProperty("key", DataSourceBase::SSRIRMaxKey.c_str());
    connect(ssr_max_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarRangeEditedSlot);
    ranges_layout->addWidget(ssr_max_edit_, row_cnt, 1);

    // mode s
    ++row_cnt;
    ranges_layout->addWidget(new QLabel("Mode S Minimum"), row_cnt, 0);

    mode_s_min_edit_ = new QLineEdit();
    mode_s_min_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 2));
    mode_s_min_edit_->setProperty("key", DataSourceBase::ModeSIRMinKey.c_str());
    connect(mode_s_min_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarRangeEditedSlot);
    ranges_layout->addWidget(mode_s_min_edit_, row_cnt, 1);

    ++row_cnt;
    ranges_layout->addWidget(new QLabel("Mode S Maximum"), row_cnt, 0);

    mode_s_max_edit_ = new QLineEdit();
    mode_s_max_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 2));
    mode_s_max_edit_->setProperty("key", DataSourceBase::ModeSIRMaxKey.c_str());
    connect(mode_s_max_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarRangeEditedSlot);
    ranges_layout->addWidget(mode_s_max_edit_, row_cnt, 1);

    ranges_widget_->setLayout(ranges_layout);
    //ranges_widget_->setMinimumHeight(300);

    main_layout->addWidget(ranges_widget_);

    add_ranges_button_ = new QPushButton("Add Radar Ranges");
    add_ranges_button_->setToolTip("Adds Radar ranges information");
    connect(add_ranges_button_, &QPushButton::clicked, this, &DataSourceEditWidget::addRadarRangesSlot);
    main_layout->addWidget(add_ranges_button_);

    main_layout->addStretch();
}

void DataSourceEditWidget::createRadarAccuraciesTab()
{
    auto main_layout = createTab(TabRadarAccuraciesName, true);

    accuracies_widget_ = new QWidget();
    accuracies_widget_->setContentsMargins(0, 0, 0, 0);

    QFont font_bold;
    font_bold.setBold(true);

    auto acc_header_label = new QLabel("Radar Accuracies");
    acc_header_label->setFont(font_bold);

    QGridLayout* accuracies_layout = new QGridLayout();
    unsigned int row_cnt = 0;
    accuracies_layout->addWidget(acc_header_label, row_cnt, 0, 1, 2);

    // psr
    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("PSR Azimuth StdDev [deg]"), row_cnt, 0);

    acc_psr_azm_edit_ = new QLineEdit();
    acc_psr_azm_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 2));
    acc_psr_azm_edit_->setProperty("key", DataSourceBase::PSRAzmSDKey.c_str());
    connect(acc_psr_azm_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_psr_azm_edit_, row_cnt, 1);

    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("PSR Range StdDev [m]"), row_cnt, 0);

    acc_psr_rng_edit_ = new QLineEdit();
    acc_psr_rng_edit_->setValidator(new TextFieldDoubleValidator(-50000, 50000, 2));
    acc_psr_rng_edit_->setProperty("key", DataSourceBase::PSRRngSDKey.c_str());
    connect(acc_psr_rng_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_psr_rng_edit_, row_cnt, 1);

    // ssr
    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("SSR Azimuth StdDev [deg]"), row_cnt, 0);

    acc_ssr_azm_edit_ = new QLineEdit();
    acc_ssr_azm_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 2));
    acc_ssr_azm_edit_->setProperty("key", DataSourceBase::SSRAzmSDKey.c_str());
    connect(acc_ssr_azm_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_ssr_azm_edit_, row_cnt, 1);

    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("SSR Range StdDev [m]"), row_cnt, 0);

    acc_ssr_rng_edit_ = new QLineEdit();
    acc_ssr_rng_edit_->setValidator(new TextFieldDoubleValidator(-50000, 50000, 2));
    acc_ssr_rng_edit_->setProperty("key", DataSourceBase::SSRRngSDKey.c_str());
    connect(acc_ssr_rng_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_ssr_rng_edit_, row_cnt, 1);

    // mode s
    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("Mode S Azimuth StdDev [deg]"), row_cnt, 0);

    acc_mode_s_azm_edit_ = new QLineEdit();
    acc_mode_s_azm_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 2));
    acc_mode_s_azm_edit_->setProperty("key", DataSourceBase::ModeSAzmSDKey.c_str());
    connect(acc_mode_s_azm_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_mode_s_azm_edit_, row_cnt, 1);

    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("Mode S Range StdDev [m]"), row_cnt, 0);

    acc_mode_s_rng_edit_ = new QLineEdit();
    acc_mode_s_rng_edit_->setValidator(new TextFieldDoubleValidator(-50000, 50000, 2));
    acc_mode_s_rng_edit_->setProperty("key", DataSourceBase::ModeSRngSDKey.c_str());
    connect(acc_mode_s_rng_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_mode_s_rng_edit_, row_cnt, 1);

    accuracies_widget_->setLayout(accuracies_layout);
    //accuracies_widget_->setMinimumHeight(300);

    main_layout->addWidget(accuracies_widget_);

    add_accuracies_button_ = new QPushButton("Add Radar Accuracies");
    add_accuracies_button_->setToolTip("Adds Radar accuracy information");
    connect(add_accuracies_button_, &QPushButton::clicked, this, &DataSourceEditWidget::addRadarAccuraciesSlot);
    main_layout->addWidget(add_accuracies_button_);

    main_layout->addStretch();
}

void DataSourceEditWidget::createRemoteUnitsTab()
{
    auto main_layout = createTab(TabMLATRemoteUnitsName, false);

    remote_units_widget_ = new QWidget;

    auto ru_layout = new QVBoxLayout;
    remote_units_widget_->setLayout(ru_layout);

    QStringList headers;
    headers << "Index";
    headers << "Name";
    headers << "Comment";
    headers << "Latitude";
    headers << "Longitude";
    headers << "Altitude";

    remote_units_list_ = new QTreeWidget;
    remote_units_list_->setHeaderLabels(headers);
    remote_units_list_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    remote_units_list_->setSelectionMode(QTreeWidget::SelectionMode::ExtendedSelection);
    remote_units_list_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

    remote_units_list_->header()->setSectionResizeMode(QHeaderView::ResizeMode::ResizeToContents);
    remote_units_list_->header()->setSectionResizeMode(2, QHeaderView::ResizeMode::Stretch);

    ru_layout->addWidget(remote_units_list_);

    auto button_layout = new QHBoxLayout;
    
    ru_add_button_ = new QPushButton("Add");
    connect(ru_add_button_, &QPushButton::pressed, this, &DataSourceEditWidget::addMLATRemoteUnitSlot);

    ru_import_button_ = new QPushButton("Import");
    connect(ru_import_button_, &QPushButton::pressed, this, &DataSourceEditWidget::importMLATRemoteUnitsSlot);

    ru_clear_button_ = new QPushButton("Remove All");
    connect(ru_clear_button_, &QPushButton::pressed, this, &DataSourceEditWidget::clearMLATRemoteUnitsSlot);

    button_layout->addWidget(ru_add_button_);
    button_layout->addWidget(ru_import_button_);
    button_layout->addWidget(ru_clear_button_);
    button_layout->addStretch();

    ru_layout->addLayout(button_layout);

    main_layout->addWidget(remote_units_widget_);

    //add placeholder widget with button
    add_remote_units_placeholder_ = new QWidget;

    auto add_ru_layout = new QVBoxLayout;
    add_ru_layout->setContentsMargins(0, 0, 0, 0);

    add_remote_units_placeholder_->setLayout(add_ru_layout);

    add_remote_units_button_ = new QPushButton("Add Remote Units");
    add_remote_units_button_->setToolTip("Adds Remote Unit information");
    connect(add_remote_units_button_, &QPushButton::clicked, this, &DataSourceEditWidget::addMLATRemoteUnitsSlot);

    add_ru_layout->addWidget(add_remote_units_button_);
    add_ru_layout->addStretch();

    main_layout->addWidget(add_remote_units_placeholder_);

    connect(remote_units_list_, &QTreeWidget::customContextMenuRequested, this, &DataSourceEditWidget::showRemoteUnitContextMenuSlot);
}

void DataSourceEditWidget::createNetworkTab()
{
    auto main_layout = createTab(TabNetworkLinesName, true);

    QFont font_bold;
    font_bold.setBold(true);
    
    net_widget_ = new QWidget();
    net_widget_->setContentsMargins(0, 0, 0, 0);

    QVBoxLayout* net_wiget_layout = new QVBoxLayout();

    QLabel* net_lines_label = new QLabel("Network Lines");
    net_lines_label->setFont(font_bold);
    net_wiget_layout->addWidget(net_lines_label);

    QFormLayout* net_layout = new QFormLayout();

    string line_str;

    for (unsigned int cnt = 0; cnt < 4; ++cnt)
    {
        line_str = "L" + QString::number(cnt + 1).toStdString();

        QLabel* line_label = new QLabel(line_str.c_str());
        line_label->setAlignment(Qt::AlignLeft | Qt::AlignTop);

        QGridLayout* line_layout = new QGridLayout();

        // listen
        line_layout->addWidget(new QLabel("Listen IP"), 0, 0);

        QLineEdit* listen_edit = new QLineEdit();
        connect(listen_edit, &QLineEdit::textEdited, this,
                &DataSourceEditWidget::netLineEditedSlot);
        listen_edit->setProperty("line", line_str.c_str());
        listen_edit->setProperty("item", "Listen IP");
        line_layout->addWidget(listen_edit, 0, 1);
        net_edits_[line_str].push_back(listen_edit);

        // mcast
        line_layout->addWidget(new QLabel("MCast IP"), 1, 0);

        QLineEdit* sender_ip_edit = new QLineEdit();
        connect(sender_ip_edit, &QLineEdit::textEdited, this,
                &DataSourceEditWidget::netLineEditedSlot);
        sender_ip_edit->setProperty("line", line_str.c_str());
        sender_ip_edit->setProperty("item", "MCast IP");
        line_layout->addWidget(sender_ip_edit, 1, 1);
        net_edits_[line_str].push_back(sender_ip_edit);

        line_layout->addWidget(new QLabel("MCast Port"), 2, 0);

        QLineEdit* sender_port_edit = new QLineEdit();
        connect(sender_port_edit, &QLineEdit::textEdited, this,
                &DataSourceEditWidget::netLineEditedSlot);
        sender_port_edit->setProperty("line", line_str.c_str());
        sender_port_edit->setProperty("item", "MCast Port");
        line_layout->addWidget(sender_port_edit, 2, 1);
        net_edits_[line_str].push_back(sender_port_edit);

        // sender
        line_layout->addWidget(new QLabel("Sender IP"), 3, 0);

        QLineEdit* sender_edit = new QLineEdit();
        connect(sender_edit, &QLineEdit::textEdited, this,
                &DataSourceEditWidget::netLineEditedSlot);
        sender_edit->setProperty("line", line_str.c_str());
        sender_edit->setProperty("item", "Sender IP");
        line_layout->addWidget(sender_edit, 3, 1);
        net_edits_[line_str].push_back(sender_edit);

        net_layout->addRow(line_label, line_layout);
    }

    net_wiget_layout->addLayout(net_layout);

    net_widget_->setLayout(net_wiget_layout);

    //net_widget_->setMinimumHeight(300);

    main_layout->addWidget(net_widget_);

    add_lines_button_ = new QPushButton("Add Network Lines");
    add_lines_button_->setToolTip("Adds network lines to the data source");
    connect(add_lines_button_, &QPushButton::clicked, this, &DataSourceEditWidget::addNetLinesSlot);
    main_layout->addWidget(add_lines_button_);

    main_layout->addStretch();
}

void DataSourceEditWidget::showID(unsigned int ds_id)
{
    has_current_ds_ = true;
    current_ds_id_ = ds_id;
    current_ds_in_db_ = ds_man_.hasDBDataSource(current_ds_id_);

    loginf << "id " << ds_id << " in db " << current_ds_in_db_;

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));

    updateContent();
}

void DataSourceEditWidget::clear()
{
    loginf;

    has_current_ds_ = false;
    current_ds_id_ = 0;
    current_ds_in_db_ = false;

    updateContent();
}


void DataSourceEditWidget::nameEditedSlot(const QString& value)
{
    string text = value.toStdString();

    loginf << "'" << text << "'";

    if (!text.size())
    {
        QMessageBox m_warning(QMessageBox::Warning, "Invalid Name",
                              "Empty names are not permitted. Please set another name.",
                              QMessageBox::Ok);

        m_warning.exec();
        return;
    }

    traced_assert(has_current_ds_);

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).name(text);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).name(text);

    update_ds_func_(current_ds_id_);
}

void DataSourceEditWidget::shortNameEditedSlot(const QString& value)
{
    string text = value.toStdString();

    loginf << "'" << text << "'";

    traced_assert(has_current_ds_);

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).shortName(text);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).shortName(text);

    update_ds_func_(current_ds_id_);
}

void DataSourceEditWidget::dsTypeEditedSlot(const QString& value)
{
    string text = value.toStdString();

    loginf << "'" << text << "'";

    traced_assert(has_current_ds_);

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).dsType(text);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).dsType(text);

    update_ds_func_(current_ds_id_);

    updateContent();
}

void DataSourceEditWidget::updateIntervalEditedSlot(const QString& value_str)
{
    string text = value_str.toStdString();

    loginf << "'" << text << "'";

    if (!value_str.size()) // remove if empty
    {
        if (current_ds_in_db_)
        {
            traced_assert(ds_man_.hasDBDataSource(current_ds_id_));

            if (ds_man_.dbDataSource(current_ds_id_).hasUpdateInterval())
                ds_man_.dbDataSource(current_ds_id_).removeUpdateInterval();
        }

        traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));

        if (ds_man_.configDataSource(current_ds_id_).hasUpdateInterval())
            ds_man_.configDataSource(current_ds_id_).removeUpdateInterval();

        return;
    }

    float value = value_str.toFloat();

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).updateInterval(value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).updateInterval(value);
}

void DataSourceEditWidget::detectionTypeChangedSlot(int index)
{
    if (!has_current_ds_)
        return;

    using DetectionType = dbContent::DataSourceBase::DetectionType;

    DetectionType selected_type = static_cast<DetectionType>(index);

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).detectionType(selected_type);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).detectionType(selected_type);

    updateContent();
}

void DataSourceEditWidget::groundOnlyCheckedSlot()
{
    loginf;

    traced_assert(ground_only_check_);

    bool checked = ground_only_check_->checkState() == Qt::Checked;

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).groundOnly(checked);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).groundOnly(checked);
}

void DataSourceEditWidget::ignoreRadarAzmRangeCheckedSlot()
{
    loginf;

    traced_assert(radar_ignore_azmrng_check_);

    bool checked = radar_ignore_azmrng_check_->checkState() == Qt::Checked;

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).ignoreRadarAzmRange(checked);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).ignoreRadarAzmRange(checked);    
}

void DataSourceEditWidget::latitudeEditedSlot(const QString& value_str)
{
    bool ok;

    double value = value_str.toDouble(&ok);

    if (!ok)
    {
        value = Number::convertLatitude(value_str.toStdString(), ok);

        if (ok)
        {
            traced_assert(latitude_edit_);
            latitude_edit_->setText(QString::number(value, 'g', 12));
        }
    }

    loginf << "'" << value << "' ok " << ok;

    if (!ok)
        return;

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).latitude(value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).latitude(value);
}

void DataSourceEditWidget::longitudeEditedSlot(const QString& value_str)
{
    bool ok;

    double value = value_str.toDouble(&ok);

    if (!ok)
    {
        value = Number::convertLongitude(value_str.toStdString(), ok);

        if (ok)
        {
            traced_assert(longitude_edit_);
            longitude_edit_->setText(QString::number(value, 'g', 12));
        }
    }

    loginf << "'" << value << "'";

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).longitude(value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).longitude(value);
}

void DataSourceEditWidget::pdEditedSlot(const QString& value_str)
{
    bool ok;

    double value = value_str.toDouble(&ok);

    if (!ok)
    {
        logwrn << "impossible value '" << value_str.toStdString() << "'";
        return;
    }

    loginf << "'" << value << "'";

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).probabilityOfDetection(value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).probabilityOfDetection(value);
}

void DataSourceEditWidget::clutterRateEditedSlot(const QString& value_str)
{
    bool ok;

    double value = value_str.toDouble(&ok);

    if (!ok)
    {
        logwrn << "impossible value '" << value_str.toStdString() << "'";
        return;
    }

    loginf << "'" << value << "'";

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).clutterRate(value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).clutterRate(value);
}

void DataSourceEditWidget::altitudeEditedSlot(const QString& value_str)
{
    double value = value_str.toDouble();

    loginf << "'" << value << "'";

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).altitude(value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).altitude(value);
}

void DataSourceEditWidget::addRadarRangesSlot()
{
    loginf;

    traced_assert(has_current_ds_);

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).addRadarRanges();
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).addRadarRanges();

    updateContent();
}

void DataSourceEditWidget::radarRangeEditedSlot(const QString& value_str)
{
    QLineEdit* line_edit = dynamic_cast<QLineEdit*> (QObject::sender());
    traced_assert(line_edit);

    string key = line_edit->property("key").toString().toStdString();

    if (!value_str.size() || value_str.toDouble() == 0)
    {
        // remove key
        loginf << "removing key '" << key << "'";

        if (current_ds_in_db_)
        {
            traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
            ds_man_.dbDataSource(current_ds_id_).removeRadarRange(key);
        }

        traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
        ds_man_.configDataSource(current_ds_id_).removeRadarRange(key);

        return;
    }

    double value = value_str.toDouble();

    loginf << "key '" << key << "' value '" << value << "'";

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).radarRange(key, value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).radarRange(key, value);
}

void DataSourceEditWidget::addRadarAccuraciesSlot()
{
    loginf;

    traced_assert(has_current_ds_);

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).addRadarAccuracies();
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).addRadarAccuracies();

    updateContent();
}

void DataSourceEditWidget::radarAccuraciesEditedSlot(const QString& value_str)
{
    double value = value_str.toDouble();

    QLineEdit* line_edit = dynamic_cast<QLineEdit*> (QObject::sender());
    traced_assert(line_edit);

    string key = line_edit->property("key").toString().toStdString();

    loginf << "key '" << key << "' value '" << value << "'";

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).radarAccuracy(key, value);
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).radarAccuracy(key, value);
}

void DataSourceEditWidget::addMLATRemoteUnitsSlot()
{
    loginf;

    auto ds_db   = currentDBDataSource();
    auto ds_conf = currentConfigDataSource();

    traced_assert(ds_conf || ds_db);
    traced_assert(!ds_conf || ds_conf->dsType() == "MLAT");
    traced_assert(!ds_db || ds_db->dsType() == "MLAT");

    if (ds_db)
        ds_db->addRemoteUnits();
    if (ds_conf)
        ds_conf->addRemoteUnits();

    updateContent();
}

bool DataSourceEditWidget::editRemoteUnit(int idx)
{
    bool add = idx < 0;

    auto ds_db   = currentDBDataSource();
    auto ds_conf = currentConfigDataSource();
    auto ds      = currentDataSource();

    traced_assert(ds);
    traced_assert(!ds_conf || ds_conf->dsType() == "MLAT");
    traced_assert(!ds_db || ds_db->dsType() == "MLAT");

    RemoteUnitDefinition ru_def_in;
    if (!add)
    {
        traced_assert(ds->hasRemoteUnit(idx));
        auto ru = ds->remoteUnit(idx);
        traced_assert(ru);

        ru_def_in = ru->toDefinition();
    }

    auto ds_name = ds->hasShortName() ? ds->shortName() : ds->name();
    QString title = add ? QString::fromStdString("Add Remote Unit for Sensor '" + ds_name + "'") :
                          QString::fromStdString("Edit Remote Unit " + std::to_string(idx) + " for Sensor '" + ds_name + "'");
    QDialog dlg;
    dlg.setWindowTitle(title);

    auto main_layout   = new QVBoxLayout;
    auto data_layout   = new QFormLayout;
    auto button_layout = new QHBoxLayout;

    dlg.setLayout(main_layout);

    main_layout->addLayout(data_layout);
    main_layout->addStretch();
    main_layout->addLayout(button_layout);

    auto index_box = new QSpinBox;
    index_box->setMinimum(0);
    index_box->setMaximum(std::numeric_limits<int>::max());
    index_box->setValue(add ? 0 : ru_def_in.index);
    index_box->setEnabled(add);

    auto name_box = new QLineEdit;
    name_box->setText(add ? "" : QString::fromStdString(ru_def_in.name));

    auto comment_box = new QLineEdit;
    comment_box->setText(add ? "" : QString::fromStdString(ru_def_in.comment));

    auto lat_box = new QDoubleSpinBox;
    lat_box->setMinimum(-90);
    lat_box->setMaximum( 90);
    lat_box->setValue(add ? 0 : ru_def_in.latitude);

    auto lon_box = new QDoubleSpinBox;
    lon_box->setMinimum(-180);
    lon_box->setMaximum( 180);
    lon_box->setValue(add ? 0 : ru_def_in.longitude);

    auto alt_box = new QDoubleSpinBox;
    alt_box->setMinimum(std::numeric_limits<double>::lowest());
    alt_box->setMaximum(std::numeric_limits<double>::max());
    alt_box->setValue(add ? 0 : ru_def_in.altitude);

    data_layout->addRow("Index: "    , index_box);
    data_layout->addRow("Name: "     , name_box);
    data_layout->addRow("Comment: "  , comment_box);
    data_layout->addRow("Latitude: " , lat_box);
    data_layout->addRow("Longitude: ", lon_box);
    data_layout->addRow("Altitude: " , alt_box);

    auto ok_button = new QPushButton(add ? "Add" : "Apply");
    auto cancel_button = new QPushButton("Cancel");

    button_layout->addWidget(cancel_button);
    button_layout->addStretch();
    button_layout->addWidget(ok_button);

    connect(cancel_button, &QPushButton::pressed, &dlg, &QDialog::reject);

    auto cb = [ & ] ()
    {
        QString err;

        if ((add && ds_db && ds_db->hasRemoteUnit(index_box->value())) ||
            (add && ds_conf && ds_conf->hasRemoteUnit(index_box->value())))
        {
            err = "Please choose a unique index.";
        }
        else if (name_box->text().isEmpty())
        {
            err = "Please choose a non-empty name.";
        }

        if (!err.isEmpty())
        {
            QMessageBox::critical(&dlg, "Error", err);
            return;
        }

        dlg.accept();
    };

    connect(ok_button, &QPushButton::pressed, cb);

    if (dlg.exec() == QDialog::Rejected)
        return false;

    RemoteUnitDefinition ru_def;
    ru_def.index     = index_box->value();
    ru_def.name      = name_box->text().toStdString();
    ru_def.comment   = comment_box->text().toStdString();
    ru_def.latitude  = lat_box->value();
    ru_def.longitude = lon_box->value();
    ru_def.altitude  = alt_box->value();

    if (add)
    {
        //add using configuration
        if (ds_db)
            ds_db->createRemoteUnit(ru_def);
        if (ds_conf)
            ds_conf->createRemoteUnit(ru_def);
    }
    else
    {
        //apply configuration
        if (ds_db)
        {
            traced_assert(ds_db->hasRemoteUnit(idx));
            traced_assert(ds_db->remoteUnit(idx));
            ds_db->remoteUnit(idx)->configure(ru_def);
        }
        if (ds_conf)
        {
            traced_assert(ds_conf->hasRemoteUnit(idx));
            traced_assert(ds_conf->remoteUnit(idx));
            ds_conf->remoteUnit(idx)->configure(ru_def);
        }
    }

    updateMLAT(ds);

    return true;
}

void DataSourceEditWidget::addMLATRemoteUnitSlot()
{
    editRemoteUnit(-1);
}

void DataSourceEditWidget::importMLATRemoteUnitsSlot()
{
    auto ds_db   = currentDBDataSource();
    auto ds_conf = currentConfigDataSource();
    auto ds      = currentDataSource();

    traced_assert(ds);
    traced_assert(!ds_conf || ds_conf->dsType() == "MLAT");
    traced_assert(!ds_db || ds_db->dsType() == "MLAT");

    auto    ds_name = ds->hasShortName() ? ds->shortName() : ds->name();
    QString title   = QString::fromStdString("Select CSV File for Sensor '" + ds_name + "'");
    QString path    = QString::fromStdString(COMPASS::instance().lastUsedPath());

    QString fn = QFileDialog::getOpenFileName(this, title, path, "*.csv");
    if (fn.isEmpty())
        return;

    std::string error;
    std::map<int, RemoteUnitDefinition> ru_defs;
    auto ok = dbContent::DataSourceBase::importRemoteUnitsCSV(ru_defs, fn.toStdString(), &error);

    if (!ok)
    {
        QMessageBox::critical(this, "Error", "Error importing Remote Units: " + QString::fromStdString(error));
        return;
    }

    auto addRUs = [ & ] (dbContent::DataSourceBase* ds)
    {
        if (!ds)
            return;
        
        ds->removeRemoteUnits();
        ds->createRemoteUnits(ru_defs);
    };

    addRUs(ds_db);
    addRUs(ds_conf);
    
    updateMLAT(ds);
}

void DataSourceEditWidget::clearMLATRemoteUnitsSlot()
{
    auto ds_db   = currentDBDataSource();
    auto ds_conf = currentConfigDataSource();
    auto ds      = currentDataSource();

    traced_assert(ds);
    traced_assert(!ds_conf || ds_conf->dsType() == "MLAT");
    traced_assert(!ds_db || ds_db->dsType() == "MLAT");

    if (ds_db)
        ds_db->removeRemoteUnits();
    if (ds_conf)
        ds_conf->removeRemoteUnits();

    updateMLAT(ds);
}

void DataSourceEditWidget::showRemoteUnitContextMenuSlot(const QPoint& pos)
{
    if (remote_units_list_->selectedItems().empty())
        return;

    QMenu menu;

    auto edit_action = menu.addAction("Edit");
    auto remove_action = menu.addAction("Remove");

    connect(edit_action, &QAction::triggered, this, &DataSourceEditWidget::editMLATRemoteUnitSlot);
    connect(remove_action, &QAction::triggered, this, &DataSourceEditWidget::clearSelectedMLATRemoteUnitsSlot);

    menu.exec(QCursor::pos());
}

void DataSourceEditWidget::clearSelectedMLATRemoteUnitsSlot()
{
    if (remote_units_list_->selectedItems().empty())
        return;

    auto ds_db   = currentDBDataSource();
    auto ds_conf = currentConfigDataSource();
    auto ds      = currentDataSource();

    traced_assert(ds);
    traced_assert(!ds_conf || ds_conf->dsType() == "MLAT");
    traced_assert(!ds_db || ds_db->dsType() == "MLAT");

    for (auto item : remote_units_list_->selectedItems())
    {
        int index = item->data(0, Qt::DisplayRole).toInt();

        if (ds_db)
            ds_db->removeRemoteUnit(index);
        if (ds_conf)
            ds_conf->removeRemoteUnit(index);
    }

    updateMLAT(ds);
}

void DataSourceEditWidget::editMLATRemoteUnitSlot()
{
    if (remote_units_list_->selectedItems().empty())
        return;

    bool ok = false;
    int index = remote_units_list_->selectedItems().front()->data(0, Qt::DisplayRole).toInt();

    editRemoteUnit(index);
}

void DataSourceEditWidget::addNetLinesSlot()
{
    loginf;

    traced_assert(has_current_ds_);

    if (current_ds_in_db_)
    {
        traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
        ds_man_.dbDataSource(current_ds_id_).addNetworkLines();
    }

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    ds_man_.configDataSource(current_ds_id_).addNetworkLines();

    updateContent();
}

void DataSourceEditWidget::netLineEditedSlot(const QString& value_str)
{
    QLineEdit* edit = dynamic_cast<QLineEdit*> (sender());
    traced_assert(edit);

    string line_id = edit->property("line").toString().toStdString();
    string item = edit->property("item").toString().toStdString();

    traced_assert(line_id == "L1" || line_id == "L2" || line_id == "L3" || line_id == "L4");
    traced_assert(item == "Listen IP" || item == "MCast IP" || item == "MCast Port" || item == "Sender IP");

    traced_assert(has_current_ds_);

    if (item == "Listen IP" || item == "MCast IP" || item == "Sender IP")
    {
        string value = value_str.toStdString();

        loginf << "start" << line_id << " " << item << " ip '" << value << "'";

        if (current_ds_in_db_)
            traced_assert(ds_man_.hasDBDataSource(current_ds_id_));

        traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));

        if (item == "Listen IP")
        {
            if (current_ds_in_db_)
                ds_man_.dbDataSource(current_ds_id_).networkLine(line_id)->listenIP(value);

            ds_man_.configDataSource(current_ds_id_).networkLine(line_id)->listenIP(value);
        }
        else if (item == "MCast IP")
        {
            if (current_ds_in_db_)
                ds_man_.dbDataSource(current_ds_id_).networkLine(line_id)->mcastIP(value);

            ds_man_.configDataSource(current_ds_id_).networkLine(line_id)->mcastIP(value);
        }
        else // Sender IP
        {
            if (current_ds_in_db_)
                ds_man_.dbDataSource(current_ds_id_).networkLine(line_id)->listenIP(value);

            ds_man_.configDataSource(current_ds_id_).networkLine(line_id)->listenIP(value);
        }
    }
    else // MCast Port
    {
        unsigned int value = value_str.toUInt();

        loginf << "start" << line_id << " " << item << " port '" << value << "'";

        if (current_ds_in_db_)
            traced_assert(ds_man_.hasDBDataSource(current_ds_id_));

        traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));

        traced_assert(item == "MCast Port");

            if (current_ds_in_db_)
                ds_man_.dbDataSource(current_ds_id_).networkLine(line_id)->mcastPort(value);

            ds_man_.configDataSource(current_ds_id_).networkLine(line_id)->mcastPort(value);
    }
}

void DataSourceEditWidget::deleteSlot()
{
    loginf;

    traced_assert(has_current_ds_);
    traced_assert(!current_ds_in_db_);

    delete_ds_func_(current_ds_id_);

    clear();
}

dbContent::DataSourceBase* DataSourceEditWidget::currentDataSource()
{
    if (!has_current_ds_)
        return nullptr;
    
    dbContent::DataSourceBase* ds = nullptr;

    if (current_ds_in_db_) // db && cfg
        ds = currentDBDataSource();
    else
        ds = currentConfigDataSource();

    traced_assert(ds);

    return ds;
}

dbContent::DataSourceBase* DataSourceEditWidget::currentDBDataSource()
{
    if (!has_current_ds_ || !current_ds_in_db_)
        return nullptr;

    traced_assert(ds_man_.hasDBDataSource(current_ds_id_));
    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    return dynamic_cast<dbContent::DataSourceBase*>(&ds_man_.dbDataSource(current_ds_id_));
}

dbContent::DataSourceBase* DataSourceEditWidget::currentConfigDataSource()
{
    if (!has_current_ds_)
        return nullptr;

    traced_assert(ds_man_.hasConfigDataSource(current_ds_id_));
    return dynamic_cast<dbContent::DataSourceBase*>(&ds_man_.configDataSource(current_ds_id_));
}

void DataSourceEditWidget::updateContent()
{
    traced_assert(name_edit_);
    traced_assert(short_name_edit_);
    traced_assert(dstype_combo_);
    traced_assert(sac_sic_id_label_);
    traced_assert(detection_type_combo_);
    traced_assert(ground_only_check_);
    traced_assert(position_widget_);
    traced_assert(add_ranges_button_);
    traced_assert(ranges_widget_);
    traced_assert(accuracies_widget_);
    traced_assert(add_accuracies_button_);
    traced_assert(remote_units_widget_);
    traced_assert(add_remote_units_button_);
    traced_assert(add_remote_units_placeholder_);
    traced_assert(net_widget_);
    traced_assert(add_lines_button_);

    detection_type_combo_->blockSignals(true);

    auto ds = currentDataSource();

    if (!ds)
    {
        enableAll(false);
    }
    else
    {
        updateMain(ds);

        // position
        updatePosition(ds);

        // radars
        if (ds->dsType() == "Radar")
            updateRadar(ds);
        else
            enableRadar(false);

        // mlat
        if (ds->dsType() == "MLAT")
            updateMLAT(ds);
        else
            enableMLAT(false);

        // lines
        if (show_network_lines_)
            updateNetwork(ds);
        else
            enableNetwork(false);
    }

    detection_type_combo_->blockSignals(false);
}

void DataSourceEditWidget::enableAll(bool enable)
{
    enableCommon(enable);
    enableRadar(enable);
    enableMLAT(enable);
    enableNetwork(enable);
}

void DataSourceEditWidget::enableCommon(bool enable)
{
    name_edit_->setText("");
    name_edit_->setEnabled(enable);

    short_name_edit_->setText("");
    short_name_edit_->setEnabled(enable);

    dstype_combo_->setType("");
    dstype_combo_->setEnabled(enable);

    sac_sic_id_label_->setText("");

    update_interval_edit_->setText("");
    update_interval_edit_->setEnabled(enable);

    detection_type_combo_->setCurrentIndex(0);
    detection_type_combo_->setEnabled(enable);

    ground_only_check_->setVisible(enable);
    position_widget_->setVisible(enable);
}

void DataSourceEditWidget::enableRadar(bool enable)
{
    radar_widget_->setVisible(enable);
    psr_jpda_widget_->setVisible(enable);

    tab_widget_->setTabVisible(tabIndex(TabRadarAccuraciesName), enable);
    tab_widget_->setTabVisible(tabIndex(TabRadarRangesName), enable);
}

void DataSourceEditWidget::enableMLAT(bool enable)
{
    tab_widget_->setTabVisible(tabIndex(TabMLATRemoteUnitsName), enable);
}

void DataSourceEditWidget::enableNetwork(bool enable)
{
    tab_widget_->setTabVisible(tabIndex(TabNetworkLinesName), enable);
}

void DataSourceEditWidget::updateMain(dbContent::DataSourceBase* ds)
{
    name_edit_->setText(ds->name().c_str());
    name_edit_->setDisabled(false);

    if (ds->hasShortName())
        short_name_edit_->setText(ds->shortName().c_str());
    else
        short_name_edit_->setText("");

    short_name_edit_->setDisabled(false);

    dstype_combo_->setType(ds->dsType());
    dstype_combo_->setDisabled(false);

    sac_sic_id_label_->setText(QString::number(ds->sac()) + "/" + QString::number(ds->sic()) +
                               "    (" + QString::number(ds->id()) + ")");

    update_interval_edit_->setDisabled(false);
    if (ds->hasUpdateInterval())
        update_interval_edit_->setText(QString::number(ds->updateInterval()));
    else
        update_interval_edit_->setText("");

    detection_type_combo_->setDisabled(false);
    auto current_type = ds->detectionType();
    detection_type_combo_->setCurrentIndex((int)current_type);

    traced_assert (ground_only_check_);
    ground_only_check_->setHidden(false);
    ground_only_check_->setChecked(ds->groundOnly());

    loginf << "ds_type " << ds->dsType() << " has pos " << ds->hasPosition();
}

void DataSourceEditWidget::updatePosition(dbContent::DataSourceBase* ds)
{
    if (ds->hasPosition())
    {
        latitude_edit_->setText(QString::number(ds->latitude(), 'g', 12));
        longitude_edit_->setText(QString::number(ds->longitude(), 'g', 12));
        altitude_edit_->setText(QString::number(ds->altitude(), 'g', 12));
    }
    else
    {
        latitude_edit_->setText("0");
        longitude_edit_->setText("0");
        altitude_edit_->setText("0");
    }

    position_widget_->setHidden(false);
}

void DataSourceEditWidget::updateRadar(dbContent::DataSourceBase* ds)
{
    traced_assert (radar_widget_);
    radar_widget_->setHidden(false);

    radar_ignore_azmrng_check_->setChecked(ds->ignoreRadarAzmRange());

    if (ds->detectionType() == DataSourceBase::DetectionType::PrimaryOnly)
    {
        psr_jpda_widget_->setHidden(false);

        if (ds->hasProbabilityOfDetection())
            psr_pd_edit_->setText(QString::number(ds->probabilityOfDetection()));
        else
            psr_pd_edit_->setText("");

        if (ds->hasClutterRate())
            psr_clutter_rate_edit_->setText(QString::number(ds->clutterRate()));
        else
            psr_clutter_rate_edit_->setText("");            
    }
    else
    {
        psr_jpda_widget_->setHidden(true);
    }

    if (ds->hasRadarRanges())
    {
        ranges_widget_->setHidden(false);
        add_ranges_button_->setHidden(true);

        std::map<std::string, double> ranges = ds->radarRanges();

        // psr
        if (ranges.count(DataSourceBase::PSRIRMinKey))
            psr_min_edit_->setText(QString::number(ranges.at(DataSourceBase::PSRIRMinKey)));
        else
            psr_min_edit_->setText("");

        if (ranges.count(DataSourceBase::PSRIRMaxKey))
            psr_max_edit_->setText(QString::number(ranges.at(DataSourceBase::PSRIRMaxKey)));
        else
            psr_max_edit_->setText("");

        // ssr
        if (ranges.count(DataSourceBase::SSRIRMinKey))
            ssr_min_edit_->setText(QString::number(ranges.at(DataSourceBase::SSRIRMinKey)));
        else
            ssr_min_edit_->setText("");

        if (ranges.count(DataSourceBase::SSRIRMaxKey))
            ssr_max_edit_->setText(QString::number(ranges.at(DataSourceBase::SSRIRMaxKey)));
        else
            ssr_max_edit_->setText("");

        // mode s
        if (ranges.count(DataSourceBase::ModeSIRMinKey))
            mode_s_min_edit_->setText(QString::number(ranges.at(DataSourceBase::ModeSIRMinKey)));
        else
            mode_s_min_edit_->setText("");

        if (ranges.count(DataSourceBase::ModeSIRMaxKey))
            mode_s_max_edit_->setText(QString::number(ranges.at(DataSourceBase::ModeSIRMaxKey)));
        else
            mode_s_max_edit_->setText("");
    }
    else
    {
        ranges_widget_->setHidden(true);
        add_ranges_button_->setHidden(false);
    }

    if (ds->hasRadarAccuracies())
    {
        accuracies_widget_->setHidden(false);
        add_accuracies_button_->setHidden(true);

        std::map<std::string, double> ranges = ds->radarAccuracies();

        // psr
        if (ranges.count(DataSourceBase::PSRAzmSDKey))
            acc_psr_azm_edit_->setText(QString::number(ranges.at(DataSourceBase::PSRAzmSDKey)));
        else
            acc_psr_azm_edit_->setText("");

        if (ranges.count(DataSourceBase::PSRRngSDKey))
            acc_psr_rng_edit_->setText(QString::number(ranges.at(DataSourceBase::PSRRngSDKey)));
        else
            acc_psr_rng_edit_->setText("");

        // ssr
        if (ranges.count(DataSourceBase::SSRAzmSDKey))
            acc_ssr_azm_edit_->setText(QString::number(ranges.at(DataSourceBase::SSRAzmSDKey)));
        else
            acc_ssr_azm_edit_->setText("");

        if (ranges.count(DataSourceBase::SSRRngSDKey))
            acc_ssr_rng_edit_->setText(QString::number(ranges.at(DataSourceBase::SSRRngSDKey)));
        else
            acc_ssr_rng_edit_->setText("");

        // mode s
        if (ranges.count(DataSourceBase::ModeSAzmSDKey))
            acc_mode_s_azm_edit_->setText(
                QString::number(ranges.at(DataSourceBase::ModeSAzmSDKey)));
        else
            acc_mode_s_azm_edit_->setText("");

        if (ranges.count(DataSourceBase::ModeSRngSDKey))
            acc_mode_s_rng_edit_->setText(
                QString::number(ranges.at(DataSourceBase::ModeSRngSDKey)));
        else
            acc_mode_s_rng_edit_->setText("");
    }
    else
    {
        accuracies_widget_->setHidden(true);
        add_accuracies_button_->setHidden(false);
    }

    //reshow tab(s)
    tab_widget_->setTabVisible(tabIndex(TabRadarAccuraciesName), true);
    tab_widget_->setTabVisible(tabIndex(TabRadarRangesName), true);
}

void DataSourceEditWidget::updateMLAT(dbContent::DataSourceBase* ds)
{
    traced_assert(remote_units_widget_);
    traced_assert(remote_units_list_);

    if (ds->hasRemoteUnits())
    {
        add_remote_units_placeholder_->setVisible(false);
        remote_units_widget_->setVisible(true);

        remote_units_list_->blockSignals(true);
        remote_units_list_->clear();

        for (const auto& ru : ds->remoteUnits())
        {
            traced_assert(ru.second);

            auto item = new QTreeWidgetItem;
            item->setData(0, Qt::DisplayRole, ru.second->index());
            item->setData(1, Qt::DisplayRole, QString::fromStdString(ru.second->name()));
            item->setData(2, Qt::DisplayRole, QString::fromStdString(ru.second->comment()));
            item->setData(3, Qt::DisplayRole, ru.second->latitude());
            item->setData(4, Qt::DisplayRole, ru.second->longitude());
            item->setData(5, Qt::DisplayRole, ru.second->altitude());

            remote_units_list_->addTopLevelItem(item);
        }

        remote_units_list_->blockSignals(false);
    }
    else
    {
        add_remote_units_placeholder_->setVisible(true);
        remote_units_widget_->setVisible(false);
    }

    //reshow tab(s)
    tab_widget_->setTabVisible(tabIndex(TabMLATRemoteUnitsName), true);
}

void DataSourceEditWidget::updateNetwork(dbContent::DataSourceBase* ds)
{
    traced_assert(net_widget_);

    if (ds->hasNetworkLines())
    {
        add_lines_button_->setHidden(true);
        net_widget_->setHidden(false);

        std::map<std::string, std::shared_ptr<DataSourceLineInfo>> lines = ds->networkLines();

        for (auto& edit_it : net_edits_)  // line -> edits
        {
            traced_assert(edit_it.second.size() == 4);

            if (lines.count(edit_it.first))  // exists, set
            {
                std::shared_ptr<DataSourceLineInfo> line = lines.at(edit_it.first);

                if (line->hasListenIP())
                    edit_it.second.at(0)->setText(line->listenIP().c_str());
                else
                    edit_it.second.at(0)->setText("");

                edit_it.second.at(1)->setText(line->mcastIP().c_str());
                edit_it.second.at(2)->setText(QString::number(line->mcastPort()));

                if (line->hasSenderIP())
                    edit_it.second.at(3)->setText(line->senderIP().c_str());
                else
                    edit_it.second.at(3)->setText("");
            }
            else  // nope, clear
            {
                for (auto edit_ptr : edit_it.second)
                    edit_ptr->setText("");
            }
        }
    }
    else
    {
        add_lines_button_->setHidden(false);
        net_widget_->setHidden(true);
    }

    //reshow tab(s)
    tab_widget_->setTabVisible(tabIndex(TabNetworkLinesName), true);
}
