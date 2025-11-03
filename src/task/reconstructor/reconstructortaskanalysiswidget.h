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

class ReconstructorTask;

class QCheckBox;
class QLineEdit;

class ReconstructorTaskAnalysisWidget : public QWidget
{
    Q_OBJECT
signals:

public slots:
    void utnsChangedSlot(const QString& value);
    void recNumsChangedSlot(const QString& value);

public:
    explicit ReconstructorTaskAnalysisWidget(ReconstructorTask& task, bool probimm_reconst, QWidget *parent = nullptr);
    virtual ~ReconstructorTaskAnalysisWidget();

    void updateValues();

protected:
    void timestampsChanged();

    ReconstructorTask& task_;
    bool probimm_reconst_ {false};

    QCheckBox* debug_check_{nullptr};

    QLineEdit* utns_edit_{nullptr};
    QLineEdit* rec_nums_edit_{nullptr};

    QLineEdit* timestamp_min_edit_{nullptr};
    QLineEdit* timestamp_max_edit_{nullptr};

    QCheckBox* debug_association_check_{nullptr};
    QCheckBox* debug_outliers_check_{nullptr};
    QCheckBox* debug_reference_calculation_check_{nullptr};
    QCheckBox* debug_kalman_chains_check_{nullptr};
    QCheckBox* debug_jpda_check_ {nullptr};
    QCheckBox* debug_tentative_targets_check_ {nullptr};
    QCheckBox* debug_write_reconstruction_viewpoints_check_{nullptr};

    QCheckBox* analyze_check_{nullptr};
    QCheckBox* analyze_association_check_{nullptr};
    QCheckBox* analyze_outliers_check_{nullptr};
    QCheckBox* analyze_accuracy_est_check_{nullptr};
    QCheckBox* analyze_bias_correction_check_{nullptr};
    QCheckBox* analyze_geo_altitude_correction_check_{nullptr};

    QCheckBox* stats_check_ {nullptr};
    QCheckBox* stats_reference_calculation_check_ {nullptr};
    QCheckBox* stats_kalman_chains_check_ {nullptr};
    QCheckBox* stats_jpda_check_ {nullptr};
    QCheckBox* stats_tentative_targets_check_ {nullptr};
    QCheckBox* stats_adsb_check_ {nullptr};
    QCheckBox* stats_association_check_ {nullptr};
};
