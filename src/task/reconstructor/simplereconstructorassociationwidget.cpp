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

#include "simplereconstructorassociationwidget.h"
#include "simplereconstructorwidget.h"
#include "simplereconstructor.h"
#include "noscrollqspinbox.h"

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

using namespace std;

SimpleReconstructorAssociationWidget::SimpleReconstructorAssociationWidget(
    SimpleReconstructor& reconstructor, SimpleReconstructorWidget& parent)
    : reconstructor_(reconstructor), parent_(parent)
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();


    int row = 0;

    QGridLayout* layout = new QGridLayout();

    layout->addWidget(new QLabel("Maximum Comparison Time Difference [s]"), row, 0);

    max_time_diff_edit_ = new QLineEdit();
    connect(max_time_diff_edit_, &QLineEdit::textEdited,
            this, &SimpleReconstructorAssociationWidget::maxTimeDiffEditedSlot);
    layout->addWidget(max_time_diff_edit_, row, 1);

    ++row;

    layout->addWidget(new QLabel("Maximum Track Time Difference [s]"), row, 0);

    max_time_diff_tracker_edit_ = new NoScrollQSpinBox();
    max_time_diff_tracker_edit_->setRange(0, 1000);
    connect(max_time_diff_tracker_edit_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &SimpleReconstructorAssociationWidget::maxTimeDiffTrackerEditedSlot);

    layout->addWidget(max_time_diff_tracker_edit_, row, 1);

    {
        //    QLineEdit* max_distance_quit_tracker_edit_{nullptr};
        ++row;

        layout->addWidget(new QLabel("Maximum Erroneous Distance Air [m]"), row, 0);

        max_distance_notok_air_edit_ = new QLineEdit();
        connect(max_distance_notok_air_edit_, &QLineEdit::textEdited,
                this, &SimpleReconstructorAssociationWidget::maxDistanceNotOKAirEditedSlot);
        layout->addWidget(max_distance_notok_air_edit_, row, 1);

        //    QLineEdit* max_distance_dubious_tracker_edit_{nullptr};
        ++row;

        layout->addWidget(new QLabel("Maximum Dubious Distance Air [m]"), row, 0);

        max_distance_dubious_air_edit_ = new QLineEdit();
        connect(max_distance_dubious_air_edit_, &QLineEdit::textEdited,
                this, &SimpleReconstructorAssociationWidget::maxDistanceDubiousAirEditedSlot);
        layout->addWidget(max_distance_dubious_air_edit_, row, 1);

        ++row;

        layout->addWidget(new QLabel("Maximum Acceptable Distance Air [m]"), row, 0);

        max_distance_acceptable_air_edit_ = new QLineEdit();
        connect(max_distance_acceptable_air_edit_, &QLineEdit::textEdited,
                this, &SimpleReconstructorAssociationWidget::maxDistanceAcceptableAirEditedSlot);
        layout->addWidget(max_distance_acceptable_air_edit_, row, 1);
    }

    {
        //    QLineEdit* max_distance_quit_tracker_edit_{nullptr};
        ++row;

        layout->addWidget(new QLabel("Maximum Erroneous Distance Ground [m]"), row, 0);

        max_distance_notok_ground_edit_ = new QLineEdit();
        connect(max_distance_notok_ground_edit_, &QLineEdit::textEdited,
                this, &SimpleReconstructorAssociationWidget::maxDistanceNotOKGroundEditedSlot);
        layout->addWidget(max_distance_notok_ground_edit_, row, 1);

        //    QLineEdit* max_distance_dubious_tracker_edit_{nullptr};
        ++row;

        layout->addWidget(new QLabel("Maximum Dubious Distance Ground [m]"), row, 0);

        max_distance_dubious_ground_edit_ = new QLineEdit();
        connect(max_distance_dubious_ground_edit_, &QLineEdit::textEdited,
                this, &SimpleReconstructorAssociationWidget::maxDistanceDubiousGroundEditedSlot);
        layout->addWidget(max_distance_dubious_ground_edit_, row, 1);

        ++row;

        layout->addWidget(new QLabel("Maximum Acceptable Distance Ground [m]"), row, 0);

        max_distance_acceptable_ground_edit_ = new QLineEdit();
        connect(max_distance_acceptable_ground_edit_, &QLineEdit::textEdited,
                this, &SimpleReconstructorAssociationWidget::maxDistanceAcceptableGroundEditedSlot);
        layout->addWidget(max_distance_acceptable_ground_edit_, row, 1);
    }

    //    QLineEdit* max_altitude_diff_tracker_edit_{nullptr};
    ++row;

    layout->addWidget(new QLabel("Maximum Altitude Difference [ft]"), row, 0);

    max_altitude_diff_edit_ = new QLineEdit();
    connect(max_altitude_diff_edit_, &QLineEdit::textEdited,
            this, &SimpleReconstructorAssociationWidget::maxAltitudeDiffEditedSlot);
    layout->addWidget(max_altitude_diff_edit_, row, 1);


    // QCheckBox* do_track_number_disassociate_using_distance_box_ {nullptr};
    ++row;
    layout->addWidget(new QLabel("Do Track Number Disassoc. Using Distance"), row, 0);

    do_track_number_disassociate_using_distance_box_ = new QCheckBox();

    connect(do_track_number_disassociate_using_distance_box_, &QCheckBox::toggled,
            [ = ] (bool ok) { this->reconstructor_.settings().do_track_number_disassociate_using_distance_ = ok; });

    layout->addWidget(do_track_number_disassociate_using_distance_box_, row, 1);


    // QSpinBox* tn_disassoc_distance_factor_edit_{nullptr};
    ++row;
    layout->addWidget(new QLabel("Track Number Disassoc. Distance Factor [1]"), row, 0);

    tn_disassoc_distance_factor_edit_ = new NoScrollQSpinBox();
    tn_disassoc_distance_factor_edit_->setRange(1, 10);
    connect(tn_disassoc_distance_factor_edit_, QOverload<int>::of(&QSpinBox::valueChanged),
            [ & ] (int v) { this->reconstructor_.settings().tn_disassoc_distance_factor_ = v; });

    layout->addWidget(tn_disassoc_distance_factor_edit_, row, 1);

    // QCheckBox* do_identity_change_cut_box_ {nullptr};
    ++row;
    layout->addWidget(new QLabel("Do Identity Change Cut"), row, 0);

    do_identity_change_cut_box_ = new QCheckBox();

    connect(do_identity_change_cut_box_, &QCheckBox::toggled,
            [ = ] (bool ok) { this->reconstructor_.settings().do_identity_change_cut_ = ok; });

    layout->addWidget(do_identity_change_cut_box_, row, 1);

    //    QLineEdit* min_updates_tracker_edit_{nullptr};
    ++row;

    layout->addWidget(new QLabel("Minimum Updates [1]"), row, 0);

    min_updates_edit_ = new QLineEdit();
    connect(min_updates_edit_, &QLineEdit::textEdited,
            this, &SimpleReconstructorAssociationWidget::minUpdatesEditedSlot);
    layout->addWidget(min_updates_edit_, row, 1);

    //    QLineEdit* prob_min_time_overlap_tracker_edit_{nullptr};
    ++row;

    layout->addWidget(new QLabel("Minimum Time Overlap Probability [0-1]"), row, 0);

    prob_min_time_overlap_edit_ = new QLineEdit();
    connect(prob_min_time_overlap_edit_, &QLineEdit::textEdited,
            this, &SimpleReconstructorAssociationWidget::probMinTimeOverlapEditedSlot);
    layout->addWidget(prob_min_time_overlap_edit_, row, 1);

    //    QLineEdit* max_speed_tracker_kts_edit_{nullptr};
    ++row;

    main_layout->addLayout(layout);

    main_layout->addStretch();

    setLayout(main_layout);
}

SimpleReconstructorAssociationWidget::~SimpleReconstructorAssociationWidget()
{

}

void SimpleReconstructorAssociationWidget::updateValues()
{
    // tracker
    //    QLineEdit* max_time_diff_tracker_edit_{nullptr};
    traced_assert(max_time_diff_edit_);
    max_time_diff_edit_->setText(QString::number(reconstructor_.settings().max_time_diff_));

    traced_assert(max_time_diff_tracker_edit_);
    max_time_diff_tracker_edit_->setValue(reconstructor_.settings().track_max_time_diff_);

    {
        //    QLineEdit* max_distance_quit_tracker_edit_{nullptr};
        traced_assert(max_distance_notok_air_edit_);
        max_distance_notok_air_edit_->setText(QString::number(reconstructor_.settings().max_distance_notok_air_));

        //    QLineEdit* max_distance_dubious_tracker_edit_{nullptr};
        traced_assert(max_distance_dubious_air_edit_);
        max_distance_dubious_air_edit_->setText(QString::number(
            reconstructor_.settings().max_distance_dubious_air_));

        //    QLineEdit* max_distance_acceptable_tracker_edit_{nullptr};
        traced_assert(max_distance_acceptable_air_edit_);
        max_distance_acceptable_air_edit_->setText(
            QString::number(reconstructor_.settings().max_distance_acceptable_air_));
    }

    {
        //    QLineEdit* max_distance_quit_tracker_edit_{nullptr};
        traced_assert(max_distance_notok_ground_edit_);
        max_distance_notok_ground_edit_->setText(QString::number(reconstructor_.settings().max_distance_notok_ground_));

        //    QLineEdit* max_distance_dubious_tracker_edit_{nullptr};
        traced_assert(max_distance_dubious_ground_edit_);
        max_distance_dubious_ground_edit_->setText(QString::number(
            reconstructor_.settings().max_distance_dubious_ground_));

        //    QLineEdit* max_distance_acceptable_tracker_edit_{nullptr};
        traced_assert(max_distance_acceptable_ground_edit_);
        max_distance_acceptable_ground_edit_->setText(
            QString::number(reconstructor_.settings().max_distance_acceptable_ground_));
    }

    //    QLineEdit* max_altitude_diff_tracker_edit_{nullptr};
    traced_assert(max_altitude_diff_edit_);
    max_altitude_diff_edit_->setText(QString::number(reconstructor_.settings().max_altitude_diff_));

    // QCheckBox* do_track_number_disassociate_using_distance_box_ {nullptr};
    traced_assert(do_track_number_disassociate_using_distance_box_);
    do_track_number_disassociate_using_distance_box_->setChecked(
        reconstructor_.settings().do_track_number_disassociate_using_distance_);

    // QSpinBox* tn_disassoc_distance_factor_edit_{nullptr};
    traced_assert(tn_disassoc_distance_factor_edit_);
    tn_disassoc_distance_factor_edit_->setValue(reconstructor_.settings().tn_disassoc_distance_factor_);

    // QCheckBox* do_identity_change_cut_box_ {nullptr};
    traced_assert(do_identity_change_cut_box_);
    do_identity_change_cut_box_->setChecked(reconstructor_.settings().do_identity_change_cut_);

    //    QLineEdit* min_updates_tracker_edit_{nullptr};
    traced_assert(min_updates_edit_);
    min_updates_edit_->setText(QString::number(reconstructor_.settings().target_min_updates_));

    //    QLineEdit* prob_min_time_overlap_tracker_edit_{nullptr};
    traced_assert(prob_min_time_overlap_edit_);
    prob_min_time_overlap_edit_->setText(
        QString::number(reconstructor_.settings().target_prob_min_time_overlap_));

}

void SimpleReconstructorAssociationWidget::maxTimeDiffEditedSlot (const QString& text)
{
    string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_time_diff_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}

void SimpleReconstructorAssociationWidget::maxTimeDiffTrackerEditedSlot (int value)
{
    loginf << "value '" << value << "'";

    reconstructor_.settings().track_max_time_diff_ = value;
}

void SimpleReconstructorAssociationWidget::maxDistanceNotOKAirEditedSlot (const QString& text)
{
    string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_distance_notok_air_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}

void SimpleReconstructorAssociationWidget::maxDistanceDubiousAirEditedSlot (const QString& text)
{
    string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_distance_dubious_air_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}

void SimpleReconstructorAssociationWidget::maxDistanceAcceptableAirEditedSlot (const QString& text)
{
    string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_distance_acceptable_air_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}

void SimpleReconstructorAssociationWidget::maxDistanceNotOKGroundEditedSlot (const QString& text)
{
        string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_distance_notok_ground_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}
void SimpleReconstructorAssociationWidget::maxDistanceDubiousGroundEditedSlot (const QString& text)
{
        string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_distance_dubious_ground_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}
void SimpleReconstructorAssociationWidget::maxDistanceAcceptableGroundEditedSlot (const QString& text)
{
        string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_distance_acceptable_ground_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}

void SimpleReconstructorAssociationWidget::maxAltitudeDiffEditedSlot (const QString& text)
{
    string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().max_altitude_diff_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}

void SimpleReconstructorAssociationWidget::probMinTimeOverlapEditedSlot (const QString& text)
{
    string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    double value = text.toDouble(&ok);

    if (ok)
        reconstructor_.settings().target_prob_min_time_overlap_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}

void SimpleReconstructorAssociationWidget::minUpdatesEditedSlot (const QString& text)
{
    string value_str = text.toStdString();

    loginf << "value '" << value_str << "'";

    bool ok;

    unsigned int value = text.toUInt(&ok);

    if (ok)
        reconstructor_.settings().target_min_updates_ = value;
    else
        logwrn << "unable to parse value '"
               << value_str << "'";
}



