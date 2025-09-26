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


#include "calculatereferencestaskdialog.h"
#include "calculatereferencestask.h"
#include "selectdatasourceswidget.h"
#include "util/stringconv.h"
#include "reconstruction/reconstructor_defs.h"
#include "compass.h"
#include "global.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QTabWidget>
#include <QScrollArea>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>

using namespace std;
using namespace Utils;

CalculateReferencesTaskDialog::CalculateReferencesTaskDialog(CalculateReferencesTask& task)
    : QDialog(), task_(task)
{
    setWindowTitle("Calculate References");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
    setModal(true);
    setMinimumSize(QSize(800, 600));

    createUI();

    updateSourcesWidgets();
    updateButtons();
}

void CalculateReferencesTaskDialog::createUI()
{
    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    //tab widget
    QTabWidget* tab_widget = new QTabWidget;
    tab_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    main_layout->addWidget(tab_widget);

    //data sources settings tab
    QWidget* data_sources_settings_widget = new QWidget;
    createDataSourcesSettingsWidget(data_sources_settings_widget);
    tab_widget->addTab(data_sources_settings_widget, "Input Data Sources");

    QWidget* position_filter_widget = new QWidget;
    createPositionFilterSettingsWidget(position_filter_widget);
    tab_widget->addTab(position_filter_widget, "Position Data Filter");

    //kalman settings tab
    QWidget* kalman_settings_widget = new QWidget;
    createKalmanSettingsWidget(kalman_settings_widget);
    tab_widget->addTab(kalman_settings_widget, "Kalman Settings");

    //output settings tab
    QWidget* output_settings_widget = new QWidget;
    createOutputSettingsWidget(output_settings_widget);
    tab_widget->addTab(output_settings_widget, "Output");

    //read in values from task
    readOptions();

    //bottom buttons
    {
        QHBoxLayout* button_layout = new QHBoxLayout();

        cancel_button_ = new QPushButton("Close");
        connect(cancel_button_, &QPushButton::clicked, this, &CalculateReferencesTaskDialog::cancelClickedSlot);
        button_layout->addWidget(cancel_button_);

        button_layout->addStretch();

        run_button_ = new QPushButton("Run");
        connect(run_button_, &QPushButton::clicked, this, &CalculateReferencesTaskDialog::runClickedSlot);
        button_layout->addWidget(run_button_);

        main_layout->addLayout(button_layout);
    }
}

void CalculateReferencesTaskDialog::createDataSourcesSettingsWidget(QWidget* w)
{
    QWidget* content_widget = addScrollArea(w);

    QVBoxLayout* layout = new QVBoxLayout;
    content_widget->setLayout(layout);

    use_tracker_check_ = new QCheckBox ("Use Tracker Data");
    use_tracker_check_->setChecked(task_.useTrackerData());
    connect (use_tracker_check_, SIGNAL(toggled(bool)), this, SLOT(toggleTrackerSourcesSlot()));
    layout->addWidget(use_tracker_check_);

    tracker_sources_ = new SelectDataSourcesWidget("Tracker Sources", "Tracker");
    tracker_sources_->updateSelected(task_.trackerDataSources());
    connect(tracker_sources_, &SelectDataSourcesWidget::selectionChangedSignal,
            this, &CalculateReferencesTaskDialog::trackerSourcesChangedSlot);
    layout->addWidget(tracker_sources_);

    use_adsb_check_ = new QCheckBox ("Use ADS-B Data");
    use_adsb_check_->setChecked(task_.useADSBData());
    connect (use_adsb_check_, SIGNAL(toggled(bool)), this, SLOT(toggleADSBSourcesSlot()));
    layout->addWidget(use_adsb_check_);

    adsb_sources_ = new SelectDataSourcesWidget("ADS-B Sources", "ADSB");
    adsb_sources_->updateSelected(task_.adsbDataSources());
    connect(adsb_sources_, &SelectDataSourcesWidget::selectionChangedSignal,
            this, &CalculateReferencesTaskDialog::adsbSourcesChangedSlot);
    layout->addWidget(adsb_sources_);
}

void CalculateReferencesTaskDialog::createPositionFilterSettingsWidget(QWidget* w)
{
    QWidget* content_widget = addScrollArea(w);

    QGridLayout* layout = new QGridLayout;
    content_widget->setLayout(layout);

    auto boldify = [&] (QLabel* l)
    {
        auto f = l->font();
        f.setBold(true);
        l->setFont(f);
        return l;
    };

    int row = 0;

    auto addRow = [&] (const QString& name, QWidget* w)
    {
        QLabel* label = new QLabel(name + ":");
        label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        layout->addWidget(label, row, 0);
        if (w) layout->addWidget(w, row, 1);
        ++row;
    };

    auto addOptionalRow = [&] (QCheckBox* check_box, QWidget* w)
    {
        check_box->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        layout->addWidget(check_box, row, 0);
        if (w) layout->addWidget(w, row, 1);
        ++row;
    };

    auto addHeader = [&] (const QString& name)
    {
        QLabel* label = boldify(new QLabel(name));
        layout->addWidget(label, row++, 0);
    };

    filter_position_usage_check_ = new QCheckBox("Filter Position Data");
    addOptionalRow(filter_position_usage_check_, nullptr);

    addHeader("Tracker Position Data Usage");

    tracker_only_confirmed_positions_check_ = new QCheckBox("Only Use Confirmed");
    addOptionalRow(tracker_only_confirmed_positions_check_, nullptr);

    tracker_only_noncoasting_positions_check_ = new QCheckBox("Only Use Non-Coasting");
    addOptionalRow(tracker_only_noncoasting_positions_check_, nullptr);

    tracker_only_report_detection_positions_check_ = new QCheckBox("Only Use Detected Report");
    addOptionalRow(tracker_only_report_detection_positions_check_, nullptr);

    tracker_only_report_detection_nonpsronly_positions_check_ = new QCheckBox("Only Use Non-Single PSR-only Detections");
    addOptionalRow(tracker_only_report_detection_nonpsronly_positions_check_, nullptr);

    tracker_only_high_accuracy_postions_check_ = new QCheckBox("Only Use High Accuracy");
    addOptionalRow(tracker_only_high_accuracy_postions_check_, nullptr);

    tracker_minimum_accuracy_box_ = new QDoubleSpinBox;
    tracker_minimum_accuracy_box_->setMinimum(0.0);
    tracker_minimum_accuracy_box_->setMaximum(std::numeric_limits<double>::max());
    addRow("Minimum Position Stddev", tracker_minimum_accuracy_box_);

    addHeader("ADS-B Position Data Usage");

    adsb_only_v12_positions_check_ = new QCheckBox("Only Use MOPS V1 / V2");
    addOptionalRow(adsb_only_v12_positions_check_, nullptr);

    adsb_only_high_nucp_nic_positions_check_ = new QCheckBox("Only Use High NUCp / NIC");
    addOptionalRow(adsb_only_high_nucp_nic_positions_check_, nullptr);

    adsb_minimum_nucp_nic_box_ = new QSpinBox;
    adsb_minimum_nucp_nic_box_->setMinimum(0);
    adsb_minimum_nucp_nic_box_->setMaximum(20);
    addRow("Minimum Position NUCp / NIC", adsb_minimum_nucp_nic_box_);

    adsb_only_high_nacp_positions_check_ = new QCheckBox("Only Use High NACp");
    addOptionalRow(adsb_only_high_nacp_positions_check_, nullptr);

    adsb_minimum_nacp_box_ = new QSpinBox;
    adsb_minimum_nacp_box_->setMinimum(0);
    adsb_minimum_nacp_box_->setMaximum(20);
    addRow("Minimum Position NACp", adsb_minimum_nacp_box_);

    adsb_only_high_sil_positionss_check_ = new QCheckBox("Only Use High SIL");
    addOptionalRow(adsb_only_high_sil_positionss_check_, nullptr);

    adsb_minimum_sil_box_ = new QSpinBox;
    adsb_minimum_sil_box_->setMinimum(0);
    adsb_minimum_sil_box_->setMaximum(5);
    addRow("Minimum Position SIL", adsb_minimum_sil_box_);

    layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Fixed, QSizePolicy::Expanding), layout->rowCount(), 0);
    layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed), 0, layout->columnCount());
}

void CalculateReferencesTaskDialog::createKalmanSettingsWidget(QWidget* w)
{
    QWidget* content_widget = addScrollArea(w);

    QGridLayout* layout = new QGridLayout;
    content_widget->setLayout(layout);

    auto boldify = [&] (QLabel* l)
    {
        auto f = l->font();
        f.setBold(true);
        l->setFont(f);
        return l;
    };

    int row = 0;

    auto addLabel = [&] (const QString& name, int col, bool visible)
    {
        QLabel* label = new QLabel(name + ":");
        label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        label->setVisible(visible);
        layout->addWidget(label, row, col);
    };

    auto addCheckBox = [&] (const QString& name, int col, bool visible)
    {
        QCheckBox* check_box = new QCheckBox(name);
        check_box->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        check_box->setVisible(visible);
        layout->addWidget(check_box, row, col);

        return check_box;
    };

    auto addHeader = [&] (const QString& name)
    {
        QLabel* label = boldify(new QLabel(name));
        layout->addWidget(label, row++, 0);
    };

    auto addWidget = [&] (QWidget* w, int col)
    {
        layout->addWidget(w, row, col);
    };

    auto newRow = [&] ()
    {
        ++row;
    };

    auto addUncertainty = [ & ] (const QString& name, QDoubleSpinBox** sb_ptr, bool visible, const QString& unit)
    {
        auto sb = new QDoubleSpinBox;
        sb->setMinimum(0.0);
        sb->setMaximum(std::numeric_limits<double>::max());
        sb->setVisible(visible);
        sb->setSuffix(" " + unit);

        *sb_ptr = sb;

        addLabel(name, 0, visible);
        addWidget(sb, 2);
        newRow();
    };

    bool is_appimage = COMPASS::instance().isAppImage();

    rec_type_box_ = new QComboBox;
    rec_type_box_->addItem("UMKalman2D");
#if USE_EXPERIMENTAL_SOURCE
    rec_type_box_->addItem("AMKalman2D");
#endif
    addLabel("Reconstructor", 0, true);
    addWidget(rec_type_box_, 2);
    newRow();

    map_mode_box_ = new QComboBox;
    map_mode_box_->addItem("Static", QVariant((int)reconstruction::MapProjectionMode::MapProjectStatic));
    map_mode_box_->addItem("Dynamic", QVariant((int)reconstruction::MapProjectionMode::MapProjectDynamic));
    map_mode_box_->setVisible(!is_appimage);

    addLabel("Map Projection", 0, !is_appimage);
    addWidget(map_mode_box_, 2);
    newRow();

    //uncertainty section
    addHeader("Default Uncertainties");

    addUncertainty("Measurement Stddev", &R_std_box_, !is_appimage, "m");
    addUncertainty("Measurement Stddev (high)", &R_std_high_box_, !is_appimage, "m");
    addUncertainty("Process Stddev", &Q_std_box_, true, "m");
    addUncertainty("System Stddev", &P_std_box_, !is_appimage, "m");
    addUncertainty("System Stddev (high)", &P_std_high_box_, !is_appimage, "m");

    //addHeader("System Track Uncertainties");

    R_std_syst_use_box_ = addCheckBox("Use Tracker Specific Uncertainties", 0, !is_appimage);
    newRow();

    addUncertainty("Tracker Position Stddev", &R_std_syst_pos_box_, !is_appimage, "m");
    addUncertainty("Tracker Velocity Stddev", &R_std_syst_vel_box_, true, "m/s");
    addUncertainty("Tracker Acceleration Stddev", &R_std_syst_acc_box_, !is_appimage, "m/s²");

    //addHeader("ADS-B Uncertainties");

    R_std_adsb_use_box_ = addCheckBox("Use ADS-B Specific Uncertainties", 0, !is_appimage);
    newRow();

    addUncertainty("ADS-B Position Stddev", &R_std_adsb_pos_box_, !is_appimage, "m");
    addUncertainty("ADS-B Velocity Stddev", &R_std_adsb_vel_box_, true, "m/s");
    addUncertainty("ADS-B Acceleration Stddev", &R_std_adsb_acc_box_, !is_appimage, "m/s²");

    //chain section
    addHeader("Chain Generation");

    min_dt_box_ = new QDoubleSpinBox;
    min_dt_box_->setMinimum(0.0);
    min_dt_box_->setMaximum(std::numeric_limits<double>::max());
    min_dt_box_->setSuffix(" s");

    addLabel("Minimum Time Step", 0, true);
    addWidget(min_dt_box_, 2);

    newRow();

    max_dt_box_ = new QDoubleSpinBox;
    max_dt_box_->setMinimum(0.0);
    max_dt_box_->setMaximum(std::numeric_limits<double>::max());
    max_dt_box_->setSuffix(" s");

    addLabel("Maximum Time Step", 0, true);
    addWidget(max_dt_box_, 2);
    newRow();

    // min_chain_size_box_ = new QSpinBox;
    // min_chain_size_box_->setMinimum(1);
    // min_chain_size_box_->setMaximum(INT_MAX);

    // addLabel("Minimum Chain Size", 0, true);
    // addWidget(min_chain_size_box_, 2);
    // newRow();

    //additional option section
    addHeader("Additional Options");

    smooth_box_ = addCheckBox("Smooth Results", 0, true);
    newRow();

    //systemk track resampling
    resample_systracks_box_ = addCheckBox("Resample System Tracks", 0, true);

    resample_systracks_dt_box_ = new QDoubleSpinBox;
    resample_systracks_dt_box_->setMinimum(1.0);
    resample_systracks_dt_box_->setMaximum(std::numeric_limits<double>::max());
    resample_systracks_dt_box_->setSuffix(" s");

    addLabel("Resample Interval", 1, true);
    addWidget(resample_systracks_dt_box_, 2);
    newRow();

    resample_systracks_max_dt_box_ = new QDoubleSpinBox;
    resample_systracks_max_dt_box_->setMinimum(1.0);
    resample_systracks_max_dt_box_->setMaximum(std::numeric_limits<double>::max());
    resample_systracks_max_dt_box_->setSuffix(" s");

    addLabel("Maximum Time Step", 1, true);
    addWidget(resample_systracks_max_dt_box_, 2);
    newRow();

    resample_result_box_= addCheckBox("Resample Result", 0, true);

    resample_result_dt_box_ = new QDoubleSpinBox;
    resample_result_dt_box_->setMinimum(1.0);
    resample_result_dt_box_->setMaximum(std::numeric_limits<double>::max());
    resample_result_dt_box_->setSuffix(" s");

    resample_result_Q_std_box_ = new QDoubleSpinBox;
    resample_result_Q_std_box_->setMinimum(0.0);
    resample_result_Q_std_box_->setMaximum(std::numeric_limits<double>::max());
    resample_result_Q_std_box_->setSuffix(" m");

    addLabel("Resample Interval", 1, true);
    addWidget(resample_result_dt_box_, 2);
    newRow();

    addLabel("Resample Process Stddev", 1, true);
    addWidget(resample_result_Q_std_box_, 2);
    newRow();

    use_vel_mm_box_ = addCheckBox("Use Velocity Measurements", 0, !is_appimage);
    newRow();

    verbose_box_ = addCheckBox("Verbose", 0, !is_appimage);
    newRow();

    python_comp_box_ = addCheckBox("Python Compatibility Mode", 0, !is_appimage);
    newRow();

    layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Fixed, QSizePolicy::Expanding), layout->rowCount(), 0);
    layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed), 0, layout->columnCount());

    auto activityCB = [ & ]
    {
        resample_systracks_dt_box_->setEnabled(resample_systracks_box_->isChecked());
        resample_systracks_max_dt_box_->setEnabled(resample_systracks_box_->isChecked());

        resample_result_dt_box_->setEnabled(resample_result_box_->isChecked());
        resample_result_Q_std_box_->setEnabled(resample_result_box_->isChecked());

        R_std_syst_pos_box_->setEnabled(R_std_syst_use_box_->isChecked());
        R_std_syst_vel_box_->setEnabled(R_std_syst_use_box_->isChecked());
        R_std_syst_acc_box_->setEnabled(R_std_syst_use_box_->isChecked());

        R_std_adsb_pos_box_->setEnabled(R_std_adsb_use_box_->isChecked());
        R_std_adsb_vel_box_->setEnabled(R_std_adsb_use_box_->isChecked());
        R_std_adsb_acc_box_->setEnabled(R_std_adsb_use_box_->isChecked());
    };

    connect(resample_systracks_box_, &QCheckBox::toggled, activityCB);
    connect(resample_result_box_, &QCheckBox::toggled, activityCB);
    connect(R_std_syst_use_box_, &QCheckBox::toggled, activityCB);
    connect(R_std_adsb_use_box_, &QCheckBox::toggled, activityCB);
}

void CalculateReferencesTaskDialog::createOutputSettingsWidget(QWidget* w)
{
    QWidget* content_widget = addScrollArea(w);

    QVBoxLayout* out_layout = new QVBoxLayout();

    QGridLayout* layout = new QGridLayout;
    out_layout->addLayout(layout);
    out_layout->addStretch();

    content_widget->setLayout(out_layout);

    int row = 0;

    auto boldify = [&] (QLabel* l)
    {
        auto f = l->font();
        f.setBold(true);
        l->setFont(f);
        return l;
    };

    auto addRow = [&] (const QString& name, QWidget* w)
    {
        QLabel* label = new QLabel(name + ":");
        label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        layout->addWidget(label, row, 0);
        if (w) layout->addWidget(w, row, 1);
        ++row;
    };

    auto addHeader = [&] (const QString& name)
    {
        QLabel* label = boldify(new QLabel(name));
        layout->addWidget(label, row++, 0);
    };

    addHeader("Output Data Source");

    ds_name_edit_ = new QLineEdit();
    addRow("Name", ds_name_edit_);

    ds_sac_box_ = new QSpinBox;
    ds_sac_box_->setMinimum(0);
    ds_sac_box_->setMaximum(255);
    addRow("SAC", ds_sac_box_);

    ds_sic_box_ = new QSpinBox;
    ds_sic_box_->setMinimum(0);
    ds_sic_box_->setMaximum(255);
    addRow("SIC", ds_sic_box_);

    layout->addWidget(new QLabel(), row, 0);

    ds_line_box_ = new QComboBox();
    ds_line_box_->addItems({"1", "2", "3", "4"});

    addRow("Line ID", ds_line_box_);

    layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Fixed, QSizePolicy::Expanding), layout->rowCount(), 0);
    layout->addItem(new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed), 0, layout->columnCount());
}

void CalculateReferencesTaskDialog::readOptions()
{
    const auto& s = task_.settings();

    // position filter

    filter_position_usage_check_->setChecked(s.filter_position_usage);
    tracker_only_confirmed_positions_check_->setChecked(s.tracker_only_confirmed_positions);
    tracker_only_noncoasting_positions_check_->setChecked(s.tracker_only_noncoasting_positions);
    tracker_only_report_detection_positions_check_->setChecked(s.tracker_only_report_detection_positions);
    tracker_only_report_detection_nonpsronly_positions_check_->setChecked(
                s.tracker_only_report_detection_nonpsronly_positions);
    tracker_only_high_accuracy_postions_check_->setChecked(s.tracker_only_high_accuracy_postions);
    tracker_minimum_accuracy_box_->setValue(s.tracker_minimum_accuracy);

    adsb_only_v12_positions_check_->setChecked(s.adsb_only_v12_positions);
    adsb_only_high_nucp_nic_positions_check_->setChecked(s.adsb_only_high_nucp_nic_positions);
    adsb_minimum_nucp_nic_box_->setValue(s.adsb_minimum_nucp_nic);
    adsb_only_high_nacp_positions_check_->setChecked(s.adsb_only_high_nacp_positions);
    adsb_minimum_nacp_box_->setValue(s.adsb_minimum_nacp);
    adsb_only_high_sil_positionss_check_->setChecked(s.adsb_only_high_sil_positions);
    adsb_minimum_sil_box_->setValue(s.adsb_minimum_sil);

    // kalman

    rec_type_box_->setCurrentIndex((int)s.rec_type);
    map_mode_box_->setCurrentIndex(map_mode_box_->findData((int)s.map_proj_mode));

    R_std_box_->setValue(s.R_std);
    R_std_high_box_->setValue(s.R_std_high);
    Q_std_box_->setValue(s.Q_std);
    P_std_box_->setValue(s.P_std);
    P_std_high_box_->setValue(s.P_std_high);

    R_std_adsb_use_box_->setChecked(s.use_R_std_cat021);
    R_std_adsb_pos_box_->setValue(s.R_std_pos_cat021);
    R_std_adsb_vel_box_->setValue(s.R_std_vel_cat021);
    R_std_adsb_acc_box_->setValue(s.R_std_acc_cat021);

    R_std_syst_use_box_->setChecked(s.use_R_std_cat062);
    R_std_syst_pos_box_->setValue(s.R_std_pos_cat062);
    R_std_syst_vel_box_->setValue(s.R_std_vel_cat062);
    R_std_syst_acc_box_->setValue(s.R_std_acc_cat062);

    min_dt_box_->setValue(s.min_dt);
    max_dt_box_->setValue(s.max_dt);
    //min_chain_size_box_->setValue(s.min_chain_size);

    use_vel_mm_box_->setChecked(s.use_vel_mm);
    smooth_box_->setChecked(s.smooth_rts);

    resample_systracks_box_->setChecked(s.resample_systracks);
    resample_systracks_dt_box_->setValue(s.resample_systracks_dt);
    resample_systracks_max_dt_box_->setValue(s.resample_systracks_max_dt);

    resample_result_box_->setChecked(s.resample_result);
    resample_result_dt_box_->setValue(s.resample_result_dt);
    resample_result_Q_std_box_->setValue(s.resample_result_Q_std);

    verbose_box_->setChecked(s.verbose);
    python_comp_box_->setChecked(s.python_compatibility);

    // output
    ds_name_edit_->setText(s.ds_name.c_str());
    ds_sac_box_->setValue(s.ds_sac);
    ds_sic_box_->setValue(s.ds_sic);
    ds_line_box_->setCurrentText(String::lineStrFrom(s.ds_line).c_str());
}

void CalculateReferencesTaskDialog::writeOptions()
{
    CalculateReferencesTaskSettings& s = task_.settings();

    // position filter

    s.filter_position_usage = filter_position_usage_check_->isChecked();

    s.tracker_only_confirmed_positions = tracker_only_confirmed_positions_check_->isChecked();
    s.tracker_only_noncoasting_positions = tracker_only_noncoasting_positions_check_->isChecked();
    s.tracker_only_report_detection_positions = tracker_only_report_detection_positions_check_->isChecked();
    s.tracker_only_report_detection_nonpsronly_positions  =
            tracker_only_report_detection_nonpsronly_positions_check_->isChecked();
    s.tracker_only_high_accuracy_postions = tracker_only_high_accuracy_postions_check_->isChecked();
    s.tracker_minimum_accuracy  = tracker_minimum_accuracy_box_->value();

    s.adsb_only_v12_positions = adsb_only_v12_positions_check_->isChecked();
    s.adsb_only_high_nucp_nic_positions = adsb_only_high_nucp_nic_positions_check_->isChecked();
    s.adsb_minimum_nucp_nic = adsb_minimum_nucp_nic_box_->value();
    s.adsb_only_high_nacp_positions = adsb_only_high_nacp_positions_check_->isChecked();
    s.adsb_minimum_nacp = adsb_minimum_nacp_box_->value();
    s.adsb_only_high_sil_positions = adsb_only_high_sil_positionss_check_->isChecked();
    s.adsb_minimum_sil = adsb_minimum_sil_box_->value();

    // kalman

    s.rec_type              = (CalculateReferencesTaskSettings::ReconstructorType)rec_type_box_->currentIndex();
    s.map_proj_mode         = (CalculateReferencesTaskSettings::MapProjectionMode)map_mode_box_->currentData().toInt();

    s.R_std                 = R_std_box_->value();
    s.R_std_high            = R_std_high_box_->value();
    s.Q_std                 = Q_std_box_->value();
    s.P_std                 = P_std_box_->value();
    s.P_std_high            = P_std_high_box_->value();

    s.use_R_std_cat021 = R_std_adsb_use_box_->isChecked();
    s.R_std_pos_cat021 = R_std_adsb_pos_box_->value();
    s.R_std_vel_cat021 = R_std_adsb_vel_box_->value();
    s.R_std_acc_cat021 = R_std_adsb_acc_box_->value();

    s.use_R_std_cat062 = R_std_syst_use_box_->isChecked();
    s.R_std_pos_cat062 = R_std_syst_pos_box_->value();
    s.R_std_vel_cat062 = R_std_syst_vel_box_->value();
    s.R_std_acc_cat062 = R_std_syst_acc_box_->value();

    s.min_dt                = min_dt_box_->value();
    s.max_dt                = max_dt_box_->value();
    //s.min_chain_size        = min_chain_size_box_->value();

    s.use_vel_mm            = use_vel_mm_box_->isChecked();
    s.smooth_rts            = smooth_box_->isChecked();

    s.resample_systracks        = resample_systracks_box_->isChecked();
    s.resample_systracks_dt     = resample_systracks_dt_box_->value();
    s.resample_systracks_max_dt = resample_systracks_max_dt_box_->value();

    s.resample_result       = resample_result_box_->isChecked();
    s.resample_result_dt    = resample_result_dt_box_->value();
    s.resample_result_Q_std = resample_result_Q_std_box_->value();

    s.verbose               = verbose_box_->isChecked();
    s.python_compatibility  = python_comp_box_->isChecked();

    // output

    s.ds_name = ds_name_edit_->text().toStdString();
    s.ds_sac = ds_sac_box_->value();
    s.ds_sic = ds_sic_box_->value();

    bool ok;
    unsigned int line_id = ds_line_box_->currentText().toUInt(&ok);
    traced_assert(ok);
    traced_assert(line_id > 0 && line_id <= 4);

    task_.settings().ds_line = line_id-1;
}

QWidget* CalculateReferencesTaskDialog::addScrollArea(QWidget* w) const
{
    QVBoxLayout* layout_outer = new QVBoxLayout;
    layout_outer->setMargin(0);
    layout_outer->setContentsMargins(0, 0, 0, 0);

    w->setLayout(layout_outer);

    QScrollArea* scroll_area = new QScrollArea;
    scroll_area->setWidgetResizable(true);

    QWidget* scroll_widget = new QWidget;

    QVBoxLayout* layout_inner = new QVBoxLayout;
    layout_inner->setMargin(0);
    layout_inner->setContentsMargins(0, 0, 0, 0);

    QWidget* content_widget = new QWidget;
    layout_inner->addWidget(content_widget);

    scroll_widget->setLayout(layout_inner);
    scroll_area->setWidget(scroll_widget);

    layout_outer->addWidget(scroll_area);

    return content_widget;
}

void CalculateReferencesTaskDialog::updateSourcesWidgets()
{
    traced_assert(tracker_sources_);
    tracker_sources_->updateSelected(task_.trackerDataSources());
    tracker_sources_->setEnabled(task_.useTrackerData());

    traced_assert(adsb_sources_);
    adsb_sources_->updateSelected(task_.adsbDataSources());
    adsb_sources_->setEnabled(task_.useADSBData());

}

void CalculateReferencesTaskDialog::updateButtons()
{
    traced_assert(run_button_);

    run_button_->setDisabled(!task_.canRun());
}

void CalculateReferencesTaskDialog::runClickedSlot()
{
    //write selected values to task
    writeOptions();

    emit runSignal();
}

void CalculateReferencesTaskDialog::cancelClickedSlot()
{
    //write selected values to task
    writeOptions();

    emit cancelSignal();
}

void CalculateReferencesTaskDialog::toggleTrackerSourcesSlot()
{
    traced_assert(use_tracker_check_);
    task_.useTrackerData(use_tracker_check_->isChecked());

    updateSourcesWidgets();
    updateButtons();
}

void CalculateReferencesTaskDialog::trackerSourcesChangedSlot(std::map<std::string, bool> selection)
{
    task_.trackerDataSources(selection);

    updateSourcesWidgets();
    updateButtons();
}

void CalculateReferencesTaskDialog::toggleADSBSourcesSlot()
{
    traced_assert(use_adsb_check_);
    task_.useADSBData(use_adsb_check_->isChecked());

    updateSourcesWidgets();
    updateButtons();
}

void CalculateReferencesTaskDialog::adsbSourcesChangedSlot(std::map<std::string, bool> selection)
{
    task_.adsbDataSources(selection);

    updateSourcesWidgets();
    updateButtons();
}
