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

#include "reconstructortaskanalysiswidget.h"
#include "reconstructortask.h"
#include "stringconv.h"
#include "timeconv.h"
#include "logger.h"
#include "compass.h"
#include "datasourcemanager.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QCheckBox>

using namespace std;
using namespace Utils;


ReconstructorTaskAnalysisWidget::ReconstructorTaskAnalysisWidget(
    ReconstructorTask& task, bool probimm_reconst, QWidget *parent)
    : QWidget{parent}, task_(task), probimm_reconst_(probimm_reconst)
{
    bool add_debug_stuff = !COMPASS::isAppImage() || COMPASS::instance().expertMode();

    QFormLayout* combo_layout = new QFormLayout;
    //combo_layout->setMargin(0);
    combo_layout->setFormAlignment(Qt::AlignRight | Qt::AlignTop);

    //debug options

    debug_check_ = new QCheckBox ();
    connect(debug_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_ = ok; });

    if (add_debug_stuff)
        combo_layout->addRow("Debug Reconstruction", debug_check_);

    utns_edit_ = new QLineEdit();
    connect(utns_edit_, &QLineEdit::textEdited, this, &ReconstructorTaskAnalysisWidget::utnsChangedSlot);

    if (add_debug_stuff)
        combo_layout->addRow("Debug UTNs", utns_edit_);

    rec_nums_edit_ = new QLineEdit();
    connect(rec_nums_edit_, &QLineEdit::textEdited, this, &ReconstructorTaskAnalysisWidget::recNumsChangedSlot);

    if (add_debug_stuff)
        combo_layout->addRow("Debug Record Numbers", rec_nums_edit_);

    timestamp_min_edit_ = new QLineEdit();
    connect(timestamp_min_edit_, &QLineEdit::textEdited, this, &ReconstructorTaskAnalysisWidget::timestampsChanged);

    if (add_debug_stuff)
        combo_layout->addRow("Debug Timestamp Min.", timestamp_min_edit_);

    timestamp_max_edit_ = new QLineEdit();
    connect(timestamp_max_edit_, &QLineEdit::textEdited, this, &ReconstructorTaskAnalysisWidget::timestampsChanged);

    if (add_debug_stuff)
        combo_layout->addRow("Debug Timestamp Max.", timestamp_max_edit_);

    debug_association_check_= new QCheckBox();
    connect(debug_association_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_association_ = ok; });

    if (add_debug_stuff)
        combo_layout->addRow("Debug Association", debug_association_check_);

    debug_outliers_check_= new QCheckBox();
    connect(debug_outliers_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_outlier_detection_ = ok; });

    if (probimm_reconst_ && add_debug_stuff)
        combo_layout->addRow("Debug Outlier Detection", debug_outliers_check_);

    debug_reference_calculation_check_ = new QCheckBox();
    connect(debug_reference_calculation_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_reference_calculation_ = ok; });

    if (add_debug_stuff)
        combo_layout->addRow("Debug Reference Calculation", debug_reference_calculation_check_);

    debug_kalman_chains_check_ = new QCheckBox();
    connect(debug_kalman_chains_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_kalman_chains_= ok; });

    if (add_debug_stuff)
        combo_layout->addRow("Debug Kalman Chains", debug_kalman_chains_check_);

    debug_jpda_check_ = new QCheckBox();
    connect(debug_jpda_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_jpda_= ok; });

    if (add_debug_stuff)
        combo_layout->addRow("Debug JPDA", debug_jpda_check_);

    debug_tentative_targets_check_ = new QCheckBox();
    connect(debug_tentative_targets_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_tentative_targets_= ok; });

    if (add_debug_stuff)
        combo_layout->addRow("Debug Tentative Targets", debug_tentative_targets_check_);

    debug_write_reconstruction_viewpoints_check_ = new QCheckBox();
    connect(debug_write_reconstruction_viewpoints_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().debug_write_reconstruction_viewpoints_ = ok; });

    if (add_debug_stuff)
        combo_layout->addRow("Write Debug Reconstruction View Points", debug_write_reconstruction_viewpoints_check_);

    //analyse options

    analyze_check_= new QCheckBox();
    connect(analyze_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().analyze_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Analyse Reconstruction", analyze_check_);

    analyze_association_check_= new QCheckBox();
    connect(analyze_association_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().analyze_association_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Analyse Association", analyze_association_check_);

    analyze_outliers_check_ = new QCheckBox();
    connect(analyze_outliers_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().analyze_outlier_detection_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Analyse Outlier Detection", analyze_outliers_check_);

    // acc est

    analyze_accuracy_est_check_ = new QCheckBox();
    connect(analyze_accuracy_est_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().analyze_accuracy_estimation_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Analyse Accuracy Estimation", analyze_accuracy_est_check_);

    analyze_bias_correction_check_ = new QCheckBox();
    connect(analyze_bias_correction_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().analyze_bias_correction_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Analyse Bias Correction", analyze_bias_correction_check_);

    analyze_geo_altitude_correction_check_ = new QCheckBox();
    connect(analyze_geo_altitude_correction_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().analyze_geo_altitude_correction_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Analyse Geo.Altitude Correction", analyze_geo_altitude_correction_check_);

    // statistics options

    stats_check_ = new QCheckBox();
    connect(stats_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().stats_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Collect Statistics", stats_check_);

    stats_reference_calculation_check_ = new QCheckBox();
    connect(stats_reference_calculation_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().stats_reference_calculation_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Collect Reference Calculation Statistics", stats_reference_calculation_check_);

    stats_kalman_chains_check_ = new QCheckBox();
    connect(stats_kalman_chains_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().stats_kalman_chains_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Collect Kalman Chains Statistics", stats_kalman_chains_check_);

    stats_jpda_check_ = new QCheckBox();
    connect(stats_jpda_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().stats_jpda_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Collect JPDA Statistics", stats_jpda_check_);

    stats_tentative_targets_check_ = new QCheckBox();
    connect(stats_tentative_targets_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().stats_tentative_targets_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Collect Tentative Target Statistics", stats_tentative_targets_check_);

    stats_adsb_check_ = new QCheckBox();
    connect(stats_adsb_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().stats_adsb_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Collect ADSB Target Statistics", stats_adsb_check_);

    stats_association_check_ = new QCheckBox();
    connect(stats_association_check_, &QCheckBox::clicked,
            this, [ = ] (bool ok) { task_.debugSettings().stats_association_ = ok; });

    if (probimm_reconst_)
        combo_layout->addRow("Collect Association Statistics", stats_association_check_);

    setLayout(combo_layout);

    updateValues();
}

ReconstructorTaskAnalysisWidget::~ReconstructorTaskAnalysisWidget()
{
}

void ReconstructorTaskAnalysisWidget::updateValues()
{
    loginf;

    bool add_debug_stuff = !COMPASS::isAppImage() || COMPASS::instance().expertMode();

    if (!add_debug_stuff) // disable everything not shown in the settings, to be on the safe side
    {
        task_.debugSettings().debug_utns_.clear();
        task_.debugSettings().debug_rec_nums_.clear();

        // task_.debugSettings().debug_timestamp_min_ = {}; // should be ok not to reset
        // task_.debugSettings().debug_timestamp_max_ = {};

        task_.debugSettings().debug_association_ = false;

        task_.debugSettings().debug_reference_calculation_ = false;
        task_.debugSettings().debug_kalman_chains_ = false;
        task_.debugSettings().debug_write_reconstruction_viewpoints_ = false;
    }

    traced_assert(debug_check_);
    debug_check_->setChecked(task_.debugSettings().debug_);

    traced_assert(utns_edit_);
    utns_edit_->setText(String::compress(task_.debugSettings().debug_utns_, ',').c_str());

    traced_assert(rec_nums_edit_);
    rec_nums_edit_->setText(String::compress(task_.debugSettings().debug_rec_nums_, ',').c_str());

    traced_assert(timestamp_min_edit_);
    if (!task_.debugSettings().debug_timestamp_min_.is_not_a_date_time())
        timestamp_min_edit_->setText(QString::fromStdString(
            Utils::Time::toString(task_.debugSettings().debug_timestamp_min_)));
    else
        timestamp_min_edit_->setText("");

    traced_assert(timestamp_max_edit_);
    if (!task_.debugSettings().debug_timestamp_max_.is_not_a_date_time())
        timestamp_max_edit_->setText(QString::fromStdString(
            Utils::Time::toString(task_.debugSettings().debug_timestamp_max_)));
    else
        timestamp_max_edit_->setText("");

    traced_assert(debug_association_check_);
    debug_association_check_->setChecked(task_.debugSettings().debug_association_);

    traced_assert(debug_outliers_check_);
    debug_outliers_check_->setChecked(task_.debugSettings().debug_outlier_detection_);

    traced_assert(debug_reference_calculation_check_);
    debug_reference_calculation_check_->setChecked(task_.debugSettings().debug_reference_calculation_);

    traced_assert(debug_kalman_chains_check_);
    debug_kalman_chains_check_->setChecked(task_.debugSettings().debug_kalman_chains_);

    traced_assert(debug_jpda_check_);
    debug_jpda_check_->setChecked(task_.debugSettings().debug_jpda_);

    traced_assert(debug_tentative_targets_check_);
    debug_tentative_targets_check_->setChecked(task_.debugSettings().debug_tentative_targets_);

    traced_assert(debug_write_reconstruction_viewpoints_check_);
    debug_write_reconstruction_viewpoints_check_->setChecked(
        task_.debugSettings().debug_write_reconstruction_viewpoints_);

    traced_assert(analyze_check_);
    analyze_check_->setChecked(task_.debugSettings().analyze_);

    traced_assert(analyze_association_check_);
    analyze_association_check_->setChecked(task_.debugSettings().analyze_association_);

    traced_assert(analyze_outliers_check_);
    analyze_outliers_check_->setChecked(task_.debugSettings().analyze_outlier_detection_);
    analyze_outliers_check_->setDisabled(true);

    // acc est

    traced_assert(analyze_accuracy_est_check_);
    analyze_accuracy_est_check_->setChecked(task_.debugSettings().analyze_accuracy_estimation_);

    traced_assert(analyze_bias_correction_check_);
    analyze_bias_correction_check_->setChecked(task_.debugSettings().analyze_bias_correction_);

    traced_assert(analyze_geo_altitude_correction_check_);
    analyze_geo_altitude_correction_check_->setChecked(task_.debugSettings().analyze_geo_altitude_correction_);

    // stats

    traced_assert(stats_check_);
    stats_check_->setChecked(task_.debugSettings().stats_);

    traced_assert(stats_reference_calculation_check_);
    stats_reference_calculation_check_->setChecked(task_.debugSettings().stats_reference_calculation_);

    traced_assert(stats_kalman_chains_check_);
    stats_kalman_chains_check_->setChecked(task_.debugSettings().stats_kalman_chains_);

    traced_assert(stats_jpda_check_);
    stats_jpda_check_->setChecked(task_.debugSettings().stats_jpda_);

    traced_assert(stats_tentative_targets_check_);
    stats_tentative_targets_check_->setChecked(task_.debugSettings().stats_tentative_targets_);

    traced_assert(stats_adsb_check_);
    stats_adsb_check_->setChecked(task_.debugSettings().stats_adsb_);

    traced_assert(stats_association_check_);
    stats_association_check_->setChecked(task_.debugSettings().stats_association_);
}

void ReconstructorTaskAnalysisWidget::utnsChangedSlot(const QString& value)
{
    loginf << "value '" << value.toStdString() << "'";

    set<unsigned int> values_tmp;
    vector<string> split_str = String::split(value.toStdString(), ',');

    bool ok;

    for (auto& tmp_str : split_str)
    {
        unsigned int utn_tmp = QString(tmp_str.c_str()).toInt(&ok, 10);

        if (!ok)
        {
            logerr << "utn '" << tmp_str << "' not valid";
            break;
        }

        values_tmp.insert(utn_tmp);
    }

    task_.debugSettings().debug_utns_ = values_tmp;
}

void ReconstructorTaskAnalysisWidget::recNumsChangedSlot(const QString& value)
{
    loginf << "value '" << value.toStdString() << "'";

    set<unsigned long> values_tmp;
    vector<string> split_str = String::split(value.toStdString(), ',');

    bool ok;

    for (auto& tmp_str : split_str)
    {
        unsigned long utn_tmp = QString(tmp_str.c_str()).toInt(&ok, 10);

        if (!ok)
        {
            logerr << "utn '" << tmp_str << "' not valid";
            break;
        }

        values_tmp.insert(utn_tmp);
    }

    task_.debugSettings().debug_rec_nums_ = values_tmp;
}

void ReconstructorTaskAnalysisWidget::timestampsChanged()
{
    auto checkTimestamp = [ & ] (QLineEdit* line_edit)
    {
        auto txt = line_edit->text().toStdString();
        auto ts  = Utils::Time::fromString(txt);

        bool ts_ok = !ts.is_not_a_date_time();

        boost::optional<boost::posix_time::ptime> ret;

        if (ts_ok)
            ret = ts;

        line_edit->setStyleSheet(ts_ok ? "" : "color: red");

        return ret;
    };

    auto ts_min = checkTimestamp(timestamp_min_edit_);
    task_.debugSettings().debug_timestamp_min_ = ts_min.has_value() ? ts_min.value() : boost::posix_time::ptime();

    if (ts_min.has_value())
        loginf << "set ts min to "
               << Utils::Time::toString(ts_min.value());

    auto ts_max = checkTimestamp(timestamp_max_edit_);
    task_.debugSettings().debug_timestamp_max_ = ts_max.has_value() ? ts_max.value() : boost::posix_time::ptime();

    if (ts_max.has_value())
        loginf << "set ts max to "
               << Utils::Time::toString(ts_max.value());
}
