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
#include "data_source.h"
#include "dstypeselectioncombobox.h"
#include "logger.h"
#include "textfielddoublevalidator.h"
#include "datasourcebase.h"
#include "number.h"
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
const std::string DataSourceEditWidget::TabMLATRemoteUnitsName = "MLAT Remote Units";
const std::string DataSourceEditWidget::TabNetworkLinesName    = "Network";

DataSourceEditWidget::DataSourceEditWidget(bool show_network_lines,
    std::function<void(unsigned int)> update_ds_func, std::function<void(unsigned int)> delete_ds_func,
    context::DBContextManager* ctx_man)
    : show_network_lines_(show_network_lines),
    update_ds_func_(update_ds_func), delete_ds_func_(delete_ds_func),
    ctx_man_(ctx_man)
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

    auto* form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

    // common fields (always visible)
    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::nameEditedSlot);
    form->addRow(new QLabel("Name"), name_edit_);

    short_name_edit_ = new QLineEdit();
    connect(short_name_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::shortNameEditedSlot);
    form->addRow(new QLabel("Short Name"), short_name_edit_);

    dstype_combo_ = new DSTypeSelectionComboBox();
    connect(dstype_combo_, &DSTypeSelectionComboBox::changedTypeSignal, this, &DataSourceEditWidget::dsTypeEditedSlot);
    form->addRow(new QLabel("DSType"), dstype_combo_);

    sac_sic_id_label_ = new QLabel();
    form->addRow(new QLabel("SAC/SIC  (DS ID)"), sac_sic_id_label_);

    update_interval_edit_ = new QLineEdit();
    update_interval_edit_->setValidator(new TextFieldDoubleValidator(0, 90, 3));
    connect(update_interval_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::updateIntervalEditedSlot);
    form->addRow(new QLabel("Update Interval [s]"), update_interval_edit_);

    detection_type_combo_ = new QComboBox(this);
    detection_type_combo_->addItem("Undefined");
    detection_type_combo_->addItem("Primary Only");
    detection_type_combo_->addItem("Mode A/C");
    detection_type_combo_->addItem("Mode A/C Combined");
    detection_type_combo_->addItem("Mode S");
    detection_type_combo_->addItem("Mode S Combined");
    connect(detection_type_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DataSourceEditWidget::detectionTypeChangedSlot);
    form->addRow(new QLabel("Detection Type"), detection_type_combo_);

    ground_only_check_ = new QCheckBox();
    connect(ground_only_check_, &QCheckBox::clicked,
            this, &DataSourceEditWidget::groundOnlyCheckedSlot);
    form->addRow(new QLabel("Ground Only"), ground_only_check_);

    // radar-specific (hidden/shown via label + field visibility)
    radar_ignore_label_ = new QLabel("Ignore Range/Azimuth");
    radar_ignore_azmrng_check_ = new QCheckBox();
    connect(radar_ignore_azmrng_check_, &QCheckBox::clicked, this,
            &DataSourceEditWidget::ignoreRadarAzmRangeCheckedSlot);
    form->addRow(radar_ignore_label_, radar_ignore_azmrng_check_);

    // psr JPDA (hidden/shown via label + field visibility)
    psr_pd_label_ = new QLabel("JPDA PD [1]");
    psr_pd_edit_ = new QLineEdit();
    psr_pd_edit_->setValidator(new TextFieldDoubleValidator(0.001, 1, 3));
    connect(psr_pd_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::pdEditedSlot);
    form->addRow(psr_pd_label_, psr_pd_edit_);

    psr_clutter_rate_label_ = new QLabel("Clutter Rate [1]");
    psr_clutter_rate_edit_ = new QLineEdit();
    psr_clutter_rate_edit_->setValidator(new TextFieldDoubleValidator(1, 10000, 0));
    connect(psr_clutter_rate_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::clutterRateEditedSlot);
    form->addRow(psr_clutter_rate_label_, psr_clutter_rate_edit_);

    // position (hidden/shown via label + field visibility)
    latitude_label_ = new QLabel("Latitude");
    latitude_edit_ = new QLineEdit();
    connect(latitude_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::latitudeEditedSlot);
    form->addRow(latitude_label_, latitude_edit_);

    longitude_label_ = new QLabel("Longitude");
    longitude_edit_ = new QLineEdit();
    connect(longitude_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::longitudeEditedSlot);
    form->addRow(longitude_label_, longitude_edit_);

    altitude_label_ = new QLabel("Altitude");
    altitude_edit_ = new QLineEdit();
    altitude_edit_->setValidator(new TextFieldDoubleValidator(-10000, 10000, 12));
    connect(altitude_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::altitudeEditedSlot);
    form->addRow(altitude_label_, altitude_edit_);

    main_layout->addLayout(form);

    delete_button_ = new QPushButton("Delete");
    delete_button_->setIcon(QIcon());
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

    context::RadarAccuracyDefaults defs; // default values for placeholder text

    acc_psr_azm_edit_ = new QLineEdit();
    acc_psr_azm_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 3));
    acc_psr_azm_edit_->setProperty("key", DataSourceBase::PSRAzmSDKey.c_str());
    acc_psr_azm_edit_->setPlaceholderText(QString::number(defs.primary_azimuth_stddev));
    connect(acc_psr_azm_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_psr_azm_edit_, row_cnt, 1);

    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("PSR Range StdDev [m]"), row_cnt, 0);

    acc_psr_rng_edit_ = new QLineEdit();
    acc_psr_rng_edit_->setValidator(new TextFieldDoubleValidator(-50000, 50000, 1));
    acc_psr_rng_edit_->setProperty("key", DataSourceBase::PSRRngSDKey.c_str());
    acc_psr_rng_edit_->setPlaceholderText(QString::number(defs.primary_range_stddev));
    connect(acc_psr_rng_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_psr_rng_edit_, row_cnt, 1);

    // ssr
    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("SSR Azimuth StdDev [deg]"), row_cnt, 0);

    acc_ssr_azm_edit_ = new QLineEdit();
    acc_ssr_azm_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 3));
    acc_ssr_azm_edit_->setProperty("key", DataSourceBase::SSRAzmSDKey.c_str());
    acc_ssr_azm_edit_->setPlaceholderText(QString::number(defs.secondary_azimuth_stddev));
    connect(acc_ssr_azm_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_ssr_azm_edit_, row_cnt, 1);

    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("SSR Range StdDev [m]"), row_cnt, 0);

    acc_ssr_rng_edit_ = new QLineEdit();
    acc_ssr_rng_edit_->setValidator(new TextFieldDoubleValidator(-50000, 50000, 1));
    acc_ssr_rng_edit_->setProperty("key", DataSourceBase::SSRRngSDKey.c_str());
    acc_ssr_rng_edit_->setPlaceholderText(QString::number(defs.secondary_range_stddev));
    connect(acc_ssr_rng_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_ssr_rng_edit_, row_cnt, 1);

    // mode s
    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("Mode S Azimuth StdDev [deg]"), row_cnt, 0);

    acc_mode_s_azm_edit_ = new QLineEdit();
    acc_mode_s_azm_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 3));
    acc_mode_s_azm_edit_->setProperty("key", DataSourceBase::ModeSAzmSDKey.c_str());
    acc_mode_s_azm_edit_->setPlaceholderText(QString::number(defs.mode_s_azimuth_stddev));
    connect(acc_mode_s_azm_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_mode_s_azm_edit_, row_cnt, 1);

    ++row_cnt;
    accuracies_layout->addWidget(new QLabel("Mode S Range StdDev [m]"), row_cnt, 0);

    acc_mode_s_rng_edit_ = new QLineEdit();
    acc_mode_s_rng_edit_->setValidator(new TextFieldDoubleValidator(-50000, 50000, 1));
    acc_mode_s_rng_edit_->setProperty("key", DataSourceBase::ModeSRngSDKey.c_str());
    acc_mode_s_rng_edit_->setPlaceholderText(QString::number(defs.mode_s_range_stddev));
    connect(acc_mode_s_rng_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarAccuraciesEditedSlot);
    accuracies_layout->addWidget(acc_mode_s_rng_edit_, row_cnt, 1);

    accuracies_widget_->setLayout(accuracies_layout);

    main_layout->addWidget(accuracies_widget_);

    // radar bias
    auto bias_header_label = new QLabel("Radar Bias");
    bias_header_label->setFont(font_bold);

    bias_widget_ = new QWidget();
    bias_widget_->setContentsMargins(0, 0, 0, 0);

    QGridLayout* bias_layout = new QGridLayout();
    unsigned int bias_row = 0;
    bias_layout->addWidget(bias_header_label, bias_row, 0, 1, 2);

    ++bias_row;
    bias_layout->addWidget(new QLabel("Range Bias [m]"), bias_row, 0);

    bias_range_edit_ = new QLineEdit();
    bias_range_edit_->setValidator(new TextFieldDoubleValidator(-10000, 10000, 1));
    bias_range_edit_->setProperty("key", DataSourceBase::RangeBiasKey.c_str());
    connect(bias_range_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarBiasEditedSlot);
    bias_layout->addWidget(bias_range_edit_, bias_row, 1);

    ++bias_row;
    bias_layout->addWidget(new QLabel("Range Bias StdDev [m]"), bias_row, 0);

    bias_range_stddev_edit_ = new QLineEdit();
    bias_range_stddev_edit_->setValidator(new TextFieldDoubleValidator(0, 10000, 1));
    bias_range_stddev_edit_->setProperty("key", DataSourceBase::RangeBiasSDKey.c_str());
    connect(bias_range_stddev_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarBiasEditedSlot);
    bias_layout->addWidget(bias_range_stddev_edit_, bias_row, 1);

    ++bias_row;
    bias_layout->addWidget(new QLabel("Range Gain"), bias_row, 0);

    bias_range_gain_edit_ = new QLineEdit();
    bias_range_gain_edit_->setValidator(new TextFieldDoubleValidator(-1, 1, 6));
    bias_range_gain_edit_->setProperty("key", DataSourceBase::RangeGainKey.c_str());
    connect(bias_range_gain_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarBiasEditedSlot);
    bias_layout->addWidget(bias_range_gain_edit_, bias_row, 1);

    ++bias_row;
    bias_layout->addWidget(new QLabel("Range Gain StdDev"), bias_row, 0);

    bias_range_gain_stddev_edit_ = new QLineEdit();
    bias_range_gain_stddev_edit_->setValidator(new TextFieldDoubleValidator(0, 1, 6));
    bias_range_gain_stddev_edit_->setProperty("key", DataSourceBase::RangeGainSDKey.c_str());
    connect(bias_range_gain_stddev_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarBiasEditedSlot);
    bias_layout->addWidget(bias_range_gain_stddev_edit_, bias_row, 1);

    ++bias_row;
    bias_layout->addWidget(new QLabel("Azimuth Bias [deg]"), bias_row, 0);

    bias_azimuth_edit_ = new QLineEdit();
    bias_azimuth_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 4));
    bias_azimuth_edit_->setProperty("key", DataSourceBase::AzimuthBiasKey.c_str());
    connect(bias_azimuth_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarBiasEditedSlot);
    bias_layout->addWidget(bias_azimuth_edit_, bias_row, 1);

    ++bias_row;
    bias_layout->addWidget(new QLabel("Azimuth Bias StdDev [deg]"), bias_row, 0);

    bias_azimuth_stddev_edit_ = new QLineEdit();
    bias_azimuth_stddev_edit_->setValidator(new TextFieldDoubleValidator(0, 180, 4));
    bias_azimuth_stddev_edit_->setProperty("key", DataSourceBase::AzimuthBiasSDKey.c_str());
    connect(bias_azimuth_stddev_edit_, &QLineEdit::textEdited, this, &DataSourceEditWidget::radarBiasEditedSlot);
    bias_layout->addWidget(bias_azimuth_stddev_edit_, bias_row, 1);

    bias_widget_->setLayout(bias_layout);

    main_layout->addWidget(bias_widget_);

    add_bias_button_ = new QPushButton("Add Radar Biases");
    add_bias_button_->setToolTip("Adds Radar bias information");
    add_bias_button_->setIcon(QIcon());
    connect(add_bias_button_, &QPushButton::clicked, this, &DataSourceEditWidget::addRadarBiasSlot);
    main_layout->addWidget(add_bias_button_);

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

void DataSourceEditWidget::show(context::DataSource& ds, const std::string& last_used_path)
{
    current_ds_ = &ds;
    last_used_path_ = last_used_path;

    loginf << "id " << ds.id();

    updateContent();
}

void DataSourceEditWidget::clear()
{
    loginf;

    current_ds_ = nullptr;
    last_used_path_.clear();

    updateContent();
}

void DataSourceEditWidget::setReadOnly(bool read_only)
{
    read_only_ = read_only;
    updateContent();
}


void DataSourceEditWidget::nameEditedSlot(const QString& value)
{
    if (read_only_) return;

    string text = value.toStdString();

    loginf << "'" << text << "'";

    if (!text.size())
    {
        QMessageBox m_warning(QMessageBox::Warning, "Invalid Name",
                              "Empty names are not permitted. Please set another name.",
                              QMessageBox::Ok, this);

        m_warning.exec();
        return;
    }

    traced_assert(current_ds_);

traced_assert(current_ds_);
    currentDataSource().name(text);

    update_ds_func_(currentDataSource().id());
}

void DataSourceEditWidget::shortNameEditedSlot(const QString& value)
{
    if (read_only_) return;

    string text = value.toStdString();

    loginf << "'" << text << "'";

    traced_assert(current_ds_);

traced_assert(current_ds_);
    currentDataSource().shortName(text);

    update_ds_func_(currentDataSource().id());
}

void DataSourceEditWidget::dsTypeEditedSlot(const QString& value)
{
    if (read_only_) return;

    string text = value.toStdString();

    loginf << "'" << text << "'";

    traced_assert(current_ds_);

traced_assert(current_ds_);
    currentDataSource().dsType(text);

    update_ds_func_(currentDataSource().id());

    updateContent();
}

void DataSourceEditWidget::updateIntervalEditedSlot(const QString& value_str)
{
    if (read_only_) return;

    string text = value_str.toStdString();

    loginf << "'" << text << "'";

    if (!value_str.size()) // remove if empty
    {
traced_assert(current_ds_);

        if (currentDataSource().hasUpdateInterval())
            currentDataSource().removeUpdateInterval();

        return;
    }

    float value = value_str.toFloat();

traced_assert(current_ds_);
    currentDataSource().updateInterval(value);
}

void DataSourceEditWidget::detectionTypeChangedSlot(int index)
{
    if (read_only_ || !current_ds_)
        return;

    // Detection type as int: 0=Undefined, 1=PrimaryOnly, 2=ModeAC, 3=ModeACCombined, 4=ModeS, 5=ModeSCombined
    currentDataSource().detectionTypeInt(index);

    updateContent();
}

void DataSourceEditWidget::groundOnlyCheckedSlot()
{
    if (read_only_) return;
    loginf;

    traced_assert(ground_only_check_);

    bool checked = ground_only_check_->checkState() == Qt::Checked;

traced_assert(current_ds_);
    currentDataSource().groundOnly(checked);
}

void DataSourceEditWidget::ignoreRadarAzmRangeCheckedSlot()
{
    if (read_only_) return;
    loginf;

    traced_assert(radar_ignore_azmrng_check_);

    bool checked = radar_ignore_azmrng_check_->checkState() == Qt::Checked;

traced_assert(current_ds_);
    currentDataSource().ignoreRadarAzmRange(checked);    
}

void DataSourceEditWidget::latitudeEditedSlot(const QString& value_str)
{
    if (read_only_) return;
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

traced_assert(current_ds_);
    currentDataSource().latitude(value);
}

void DataSourceEditWidget::longitudeEditedSlot(const QString& value_str)
{
    if (read_only_) return;
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

traced_assert(current_ds_);
    currentDataSource().longitude(value);
}

void DataSourceEditWidget::pdEditedSlot(const QString& value_str)
{
    if (read_only_) return;
    bool ok;

    double value = value_str.toDouble(&ok);

    if (!ok)
    {
        logwrn << "impossible value '" << value_str.toStdString() << "'";
        return;
    }

    loginf << "'" << value << "'";

traced_assert(current_ds_);
    currentDataSource().probabilityOfDetection(value);
}

void DataSourceEditWidget::clutterRateEditedSlot(const QString& value_str)
{
    if (read_only_) return;
    bool ok;

    double value = value_str.toDouble(&ok);

    if (!ok)
    {
        logwrn << "impossible value '" << value_str.toStdString() << "'";
        return;
    }

    loginf << "'" << value << "'";

traced_assert(current_ds_);
    currentDataSource().clutterRate(value);
}

void DataSourceEditWidget::altitudeEditedSlot(const QString& value_str)
{
    if (read_only_) return;
    double value = value_str.toDouble();

    loginf << "'" << value << "'";

traced_assert(current_ds_);
    currentDataSource().altitude(value);
}

void DataSourceEditWidget::addRadarRangesSlot()
{
    loginf;

    traced_assert(current_ds_);

traced_assert(current_ds_);
    currentDataSource().addRadarRangesIfMissing();

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

        if (current_ds_)
        {
            traced_assert(current_ds_);
            currentDataSource().removeRadarRange(key);
        }

        currentDataSource().removeRadarRange(key);

        return;
    }

    double value = value_str.toDouble();

    loginf << "key '" << key << "' value '" << value << "'";

traced_assert(current_ds_);
    currentDataSource().radarRange(key, value);
}


void DataSourceEditWidget::radarAccuraciesEditedSlot(const QString& value_str)
{
    double value = value_str.toDouble();

    QLineEdit* line_edit = dynamic_cast<QLineEdit*> (QObject::sender());
    traced_assert(line_edit);

    string key = line_edit->property("key").toString().toStdString();

    loginf << "key '" << key << "' value '" << value << "'";

traced_assert(current_ds_);
    currentDataSource().radarAccuracy(key, value);
}

void DataSourceEditWidget::addRadarBiasSlot()
{
    loginf;

    traced_assert(current_ds_);
    currentDataSource().addRadarBiasIfMissing();

    updateContent();
}

void DataSourceEditWidget::radarBiasEditedSlot(const QString& value_str)
{
    double value = value_str.toDouble();

    QLineEdit* line_edit = dynamic_cast<QLineEdit*> (QObject::sender());
    traced_assert(line_edit);

    string key = line_edit->property("key").toString().toStdString();

    loginf << "key '" << key << "' value '" << value << "'";

    traced_assert(current_ds_);
    currentDataSource().radarBias(key, value);
}

void DataSourceEditWidget::addMLATRemoteUnitsSlot()
{
    loginf;

    auto& ds = currentDataSource();

    // assert already in currentDataSource()
    traced_assert(ds.dsType() == "MLAT");
    traced_assert(ds.dsType() == "MLAT");

    
        ds.addRemoteUnitsIfMissing();

    updateContent();
}

bool DataSourceEditWidget::editRemoteUnit(int idx)
{
    bool add = idx < 0;

    auto& ds = currentDataSource();

    // assert already in currentDataSource()
    traced_assert(ds.dsType() == "MLAT");
    traced_assert(ds.dsType() == "MLAT");

    RemoteUnitDefinition ru_def_in;
    if (!add)
    {
        traced_assert(ds.hasRemoteUnit(idx));
        // read remote unit from info JSON
        auto key = std::to_string(idx);
        const auto& ruj = ds.info().at("remote_units").at(key);
        ru_def_in.index = idx;
        ru_def_in.name = ruj.value("name", "");
        ru_def_in.comment = ruj.value("comment", "");
        ru_def_in.latitude = ruj.value("latitude", 0.0);
        ru_def_in.longitude = ruj.value("longitude", 0.0);
        ru_def_in.altitude = ruj.value("altitude", 0.0);
    }

    auto ds_name = ds.hasShortName() ? ds.shortName() : ds.name();
    QString title = add ? QString::fromStdString("Add Remote Unit for Sensor '" + ds_name + "'") :
                          QString::fromStdString("Edit Remote Unit " + std::to_string(idx) + " for Sensor '" + ds_name + "'");
    QDialog dlg(this);
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

        if (add && ds.hasRemoteUnit(index_box->value()))
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

    {
        auto key = std::to_string(ru_def.index);
        nlohmann::json ru_j;
        ru_j["name"] = ru_def.name;
        ru_j["comment"] = ru_def.comment;
        ru_j["latitude"] = ru_def.latitude;
        ru_j["longitude"] = ru_def.longitude;
        ru_j["altitude"] = ru_def.altitude;
        ds.info()["remote_units"][key] = ru_j;
    }

    updateMLAT(&ds);

    return true;
}

void DataSourceEditWidget::addMLATRemoteUnitSlot()
{
    editRemoteUnit(-1);
}

void DataSourceEditWidget::importMLATRemoteUnitsSlot()
{
    auto& ds = currentDataSource();

    // assert already in currentDataSource()
    traced_assert(ds.dsType() == "MLAT");
    traced_assert(ds.dsType() == "MLAT");

    auto    ds_name = ds.hasShortName() ? ds.shortName() : ds.name();
    QString title   = QString::fromStdString("Select CSV File for Sensor '" + ds_name + "'");
    QString path    = QString::fromStdString(last_used_path_);

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

    ds.removeRemoteUnits();
    ds.addRemoteUnitsIfMissing();
    // import remote units via info JSON
    for (const auto& ru_pair : ru_defs) {
        auto key = std::to_string(ru_pair.second.index);
        nlohmann::json ru_j;
        ru_j["name"] = ru_pair.second.name;
        ru_j["comment"] = ru_pair.second.comment;
        ru_j["latitude"] = ru_pair.second.latitude;
        ru_j["longitude"] = ru_pair.second.longitude;
        ru_j["altitude"] = ru_pair.second.altitude;
        ds.info()["remote_units"][key] = ru_j;
    }

    updateMLAT(&ds);
}

void DataSourceEditWidget::clearMLATRemoteUnitsSlot()
{
    auto& ds = currentDataSource();

    // assert already in currentDataSource()
    traced_assert(ds.dsType() == "MLAT");
    traced_assert(ds.dsType() == "MLAT");

    
        ds.removeRemoteUnits();

    updateMLAT(&ds);
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

    auto& ds = currentDataSource();

    // assert already in currentDataSource()
    traced_assert(ds.dsType() == "MLAT");
    traced_assert(ds.dsType() == "MLAT");

    for (auto item : remote_units_list_->selectedItems())
    {
        int index = item->data(0, Qt::DisplayRole).toInt();
        ds.removeRemoteUnit(index);
    }

    updateMLAT(&ds);
}

void DataSourceEditWidget::editMLATRemoteUnitSlot()
{
    if (remote_units_list_->selectedItems().empty())
        return;

    int index = remote_units_list_->selectedItems().front()->data(0, Qt::DisplayRole).toInt();

    editRemoteUnit(index);
}

void DataSourceEditWidget::addNetLinesSlot()
{
    loginf;

    traced_assert(current_ds_);

traced_assert(current_ds_);
    currentDataSource().addNetworkLinesIfMissing();

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

    traced_assert(current_ds_);

    std::string line_key = line_id; // already "L1", "L2", etc.
    auto& ds = currentDataSource();

    if (item == "Listen IP" || item == "MCast IP" || item == "Sender IP")
    {
        string value = value_str.toStdString();
        loginf << "start" << line_id << " " << item << " ip '" << value << "'";

        if (item == "Listen IP")
            ds.info()["network_lines"][line_key]["listen_ip"] = value;
        else if (item == "MCast IP")
            ds.info()["network_lines"][line_key]["mcast_ip"] = value;
        else // Sender IP
            ds.info()["network_lines"][line_key]["listen_ip"] = value;
    }
    else // MCast Port
    {
        unsigned int value = value_str.toUInt();
        loginf << "start" << line_id << " " << item << " port '" << value << "'";

        traced_assert(item == "MCast Port");
        ds.info()["network_lines"][line_key]["mcast_port"] = value;
    }
}

void DataSourceEditWidget::deleteSlot()
{
    loginf;

    traced_assert(current_ds_);
    

    delete_ds_func_(currentDataSource().id());

    clear();
}

context::DataSource& DataSourceEditWidget::currentDataSource()
{
    traced_assert(current_ds_);
    return *current_ds_;
}

void DataSourceEditWidget::updateContent()
{
    detection_type_combo_->blockSignals(true);

    auto* ds = current_ds_;

    if (!ds)
    {
        enableAll(false);
    }
    else
    {
        updateMain(ds);
        updatePosition(ds);

        if (ds->dsType() == "Radar")
            updateRadar(ds);
        else
            enableRadar(false);

        if (ds->dsType() == "MLAT")
            updateMLAT(ds);
        else
            enableMLAT(false);

        if (show_network_lines_)
            updateNetwork(ds);
        else
            enableNetwork(false);
    }

    if (read_only_ && ds)
    {
        // make inputs non-editable without graying out text
        name_edit_->setReadOnly(true);
        short_name_edit_->setReadOnly(true);
        update_interval_edit_->setReadOnly(true);
        psr_pd_edit_->setReadOnly(true);
        psr_clutter_rate_edit_->setReadOnly(true);
        latitude_edit_->setReadOnly(true);
        longitude_edit_->setReadOnly(true);
        altitude_edit_->setReadOnly(true);

        dstype_combo_->setEnabled(false);
        detection_type_combo_->setEnabled(false);
        ground_only_check_->setEnabled(false);
        radar_ignore_azmrng_check_->setEnabled(false);
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

    // hide all optional rows when disabled
    radar_ignore_label_->setVisible(false);
    radar_ignore_azmrng_check_->setVisible(false);
    psr_pd_label_->setVisible(false);
    psr_pd_edit_->setVisible(false);
    psr_clutter_rate_label_->setVisible(false);
    psr_clutter_rate_edit_->setVisible(false);
    latitude_label_->setVisible(false);
    latitude_edit_->setVisible(false);
    longitude_label_->setVisible(false);
    longitude_edit_->setVisible(false);
    altitude_label_->setVisible(false);
    altitude_edit_->setVisible(false);
}

void DataSourceEditWidget::enableRadar(bool enable)
{
    radar_ignore_label_->setVisible(enable);
    radar_ignore_azmrng_check_->setVisible(enable);
    psr_pd_label_->setVisible(false);
    psr_pd_edit_->setVisible(false);
    psr_clutter_rate_label_->setVisible(false);
    psr_clutter_rate_edit_->setVisible(false);

    setTabVisibleCompat(tabIndex(TabRadarAccuraciesName), enable);
    setTabVisibleCompat(tabIndex(TabRadarRangesName), enable);
}

void DataSourceEditWidget::enableMLAT(bool enable)
{
    //tab_widget_->setTabVisible(tabIndex(TabMLATRemoteUnitsName), enable);
    setTabVisibleCompat(tabIndex(TabMLATRemoteUnitsName), enable);
}

void DataSourceEditWidget::enableNetwork(bool enable)
{
    //tab_widget_->setTabVisible(tabIndex(TabNetworkLinesName), enable);
    setTabVisibleCompat(tabIndex(TabNetworkLinesName), enable);
}

void DataSourceEditWidget::updateMain(context::DataSource* ds)
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
    auto current_type = ds->detectionTypeInt();
    detection_type_combo_->setCurrentIndex(current_type);

    traced_assert (ground_only_check_);
    ground_only_check_->setHidden(false);
    ground_only_check_->setChecked(ds->groundOnly());

    loginf << "ds_type " << ds->dsType() << " has pos " << ds->hasPosition();
}

void DataSourceEditWidget::updatePosition(context::DataSource* ds)
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

    latitude_label_->setVisible(true);
    latitude_edit_->setVisible(true);
    longitude_label_->setVisible(true);
    longitude_edit_->setVisible(true);
    altitude_label_->setVisible(true);
    altitude_edit_->setVisible(true);
}

void DataSourceEditWidget::updateRadar(context::DataSource* ds)
{
    radar_ignore_label_->setVisible(true);
    radar_ignore_azmrng_check_->setVisible(true);
    radar_ignore_azmrng_check_->setChecked(ds->ignoreRadarAzmRange());

    bool show_jpda = (ds->detectionTypeInt() == 1); // PrimaryOnly
    psr_pd_label_->setVisible(show_jpda);
    psr_pd_edit_->setVisible(show_jpda);
    psr_clutter_rate_label_->setVisible(show_jpda);
    psr_clutter_rate_edit_->setVisible(show_jpda);

    if (show_jpda)
    {
        if (ds->info().contains("pd"))
            psr_pd_edit_->setText(QString::number(ds->probabilityOfDetection()));
        else
            psr_pd_edit_->setText("");

        if (ds->info().contains("clutter_rate"))
            psr_clutter_rate_edit_->setText(QString::number(ds->clutterRate()));
        else
            psr_clutter_rate_edit_->setText("");
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

    {
        // update PSR placeholder text based on ground-only flag
        context::RadarAccuracyDefaults defs;
        if (ds->groundOnly())
        {
            acc_psr_azm_edit_->setPlaceholderText(QString::number(defs.primary_azimuth_stddev_ground));
            acc_psr_rng_edit_->setPlaceholderText(QString::number(defs.primary_range_stddev_ground));
        }
        else
        {
            acc_psr_azm_edit_->setPlaceholderText(QString::number(defs.primary_azimuth_stddev));
            acc_psr_rng_edit_->setPlaceholderText(QString::number(defs.primary_range_stddev));
        }

        std::map<std::string, double> acc;
        if (ds->hasRadarAccuracies())
            acc = ds->radarAccuracies();

        // psr
        acc_psr_azm_edit_->setText(acc.count(DataSourceBase::PSRAzmSDKey)
            ? QString::number(acc.at(DataSourceBase::PSRAzmSDKey), 'f', 5) : "");

        acc_psr_rng_edit_->setText(acc.count(DataSourceBase::PSRRngSDKey)
            ? QString::number(acc.at(DataSourceBase::PSRRngSDKey), 'f', 2) : "");

        // ssr
        acc_ssr_azm_edit_->setText(acc.count(DataSourceBase::SSRAzmSDKey)
            ? QString::number(acc.at(DataSourceBase::SSRAzmSDKey), 'f', 5) : "");

        acc_ssr_rng_edit_->setText(acc.count(DataSourceBase::SSRRngSDKey)
            ? QString::number(acc.at(DataSourceBase::SSRRngSDKey), 'f', 2) : "");

        // mode s
        acc_mode_s_azm_edit_->setText(acc.count(DataSourceBase::ModeSAzmSDKey)
            ? QString::number(acc.at(DataSourceBase::ModeSAzmSDKey), 'f', 5) : "");

        acc_mode_s_rng_edit_->setText(acc.count(DataSourceBase::ModeSRngSDKey)
            ? QString::number(acc.at(DataSourceBase::ModeSRngSDKey), 'f', 2) : "");
    }

    // radar bias
    if (ds->hasRadarBias())
    {
        bias_widget_->setHidden(false);
        add_bias_button_->setHidden(true);

        std::map<std::string, double> bias = ds->radarBias();

        if (bias.count(DataSourceBase::RangeBiasKey))
            bias_range_edit_->setText(QString::number(bias.at(DataSourceBase::RangeBiasKey), 'f', 3));
        else
            bias_range_edit_->setText("");

        if (bias.count(DataSourceBase::RangeBiasSDKey))
            bias_range_stddev_edit_->setText(QString::number(bias.at(DataSourceBase::RangeBiasSDKey), 'f', 3));
        else
            bias_range_stddev_edit_->setText("");

        if (bias.count(DataSourceBase::RangeGainKey))
            bias_range_gain_edit_->setText(QString::number(bias.at(DataSourceBase::RangeGainKey), 'f', 7));
        else
            bias_range_gain_edit_->setText("");

        if (bias.count(DataSourceBase::RangeGainSDKey))
            bias_range_gain_stddev_edit_->setText(QString::number(bias.at(DataSourceBase::RangeGainSDKey), 'f', 7));
        else
            bias_range_gain_stddev_edit_->setText("");

        if (bias.count(DataSourceBase::AzimuthBiasKey))
            bias_azimuth_edit_->setText(QString::number(bias.at(DataSourceBase::AzimuthBiasKey), 'f', 5));
        else
            bias_azimuth_edit_->setText("");

        if (bias.count(DataSourceBase::AzimuthBiasSDKey))
            bias_azimuth_stddev_edit_->setText(QString::number(bias.at(DataSourceBase::AzimuthBiasSDKey), 'f', 5));
        else
            bias_azimuth_stddev_edit_->setText("");
    }
    else
    {
        bias_widget_->setHidden(true);
        add_bias_button_->setHidden(false);
    }

    //reshow tab(s)
    // tab_widget_->setTabVisible(tabIndex(TabRadarAccuraciesName), true);
    // tab_widget_->setTabVisible(tabIndex(TabRadarRangesName), true);
        
    setTabVisibleCompat(tabIndex(TabRadarAccuraciesName), true);
    setTabVisibleCompat(tabIndex(TabRadarRangesName), true);
}

void DataSourceEditWidget::updateMLAT(context::DataSource* ds)
{
    traced_assert(remote_units_widget_);
    traced_assert(remote_units_list_);

    if (ds->hasRemoteUnits())
    {
        add_remote_units_placeholder_->setVisible(false);
        remote_units_widget_->setVisible(true);

        remote_units_list_->blockSignals(true);
        remote_units_list_->clear();

        auto ru_obj = ds->info().value("remote_units", nlohmann::json::object());
        for (const auto& [key, ru_json] : ru_obj.items())
        {
            auto item = new QTreeWidgetItem;
            item->setData(0, Qt::DisplayRole, std::stoi(key));
            item->setData(1, Qt::DisplayRole, QString::fromStdString(ru_json.value("name", "")));
            item->setData(2, Qt::DisplayRole, QString::fromStdString(ru_json.value("comment", "")));
            item->setData(3, Qt::DisplayRole, ru_json.value("latitude", 0.0));
            item->setData(4, Qt::DisplayRole, ru_json.value("longitude", 0.0));
            item->setData(5, Qt::DisplayRole, ru_json.value("altitude", 0.0));

            remote_units_list_->addTopLevelItem(item);
        }

        remote_units_list_->sortItems(0, Qt::AscendingOrder);
        remote_units_list_->blockSignals(false);
    }
    else
    {
        add_remote_units_placeholder_->setVisible(true);
        remote_units_widget_->setVisible(false);
    }

    //reshow tab(s)
    //tab_widget_->setTabVisible(tabIndex(TabMLATRemoteUnitsName), true);
    setTabVisibleCompat(tabIndex(TabMLATRemoteUnitsName), true);
}

void DataSourceEditWidget::updateNetwork(context::DataSource* ds)
{
    traced_assert(net_widget_);

    if (ds->hasNetworkLines())
    {
        add_lines_button_->setHidden(true);
        net_widget_->setHidden(false);

        nlohmann::json lines_json = ds->info()["network_lines"];

        for (auto& edit_it : net_edits_)  // line -> edits
        {
            traced_assert(edit_it.second.size() == 4);

            if (lines_json.contains(edit_it.first))  // exists, set
            {
                const auto& line_json = lines_json.at(edit_it.first);

                edit_it.second.at(0)->setText(QString::fromStdString(line_json.value("listen_ip", "")));
                edit_it.second.at(1)->setText(QString::fromStdString(line_json.value("mcast_ip", "")));
                edit_it.second.at(2)->setText(QString::number(line_json.value("mcast_port", 0u)));
                edit_it.second.at(3)->setText(QString::fromStdString(line_json.value("sender_ip", "")));
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
    //tab_widget_->setTabVisible(tabIndex(TabNetworkLinesName), true);
    setTabVisibleCompat(tabIndex(TabNetworkLinesName), true);
}

void DataSourceEditWidget::setTabVisibleCompat(int index, bool visible)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    tab_widget_->setTabVisible(index, visible);
#else
    tab_widget_->setTabEnabled(index, visible);
    if (!visible && tab_widget_->currentIndex() == index)
        tab_widget_->setCurrentIndex(0);
#endif
}


