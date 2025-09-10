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

#include "simplereconstructor.h"
#include "simplereconstructorwidget.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableset.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/target/targetreportaccessor.h"

#include "kalman_defs.h"

#include "timeconv.h"

using namespace std;
using namespace Utils;

SimpleReconstructor::SimpleReconstructor(const std::string& class_id, 
                                         const std::string& instance_id,
                                         ReconstructorTask& task, 
                                         std::unique_ptr<AccuracyEstimatorBase>&& acc_estimator)
    : ReconstructorBase(class_id, instance_id, task, std::move(acc_estimator), settings_, 0)
    , associatior_   (*this)
    , ref_calculator_(*this)
{
    registerParameter("max_distance_notok", &settings_.max_distance_notok_, 5*NM2M); // kb 5nm
    registerParameter("max_distance_dubious", &settings_.max_distance_dubious_, 2*NM2M);
    registerParameter("max_distance_acceptable", &settings_.max_distance_acceptable_, 1*NM2M);

    registerParameter("numerical_min_std_dev", &settings_.numerical_min_std_dev_, settings_.numerical_min_std_dev_);

    registerParameter("unspecific_pos_acc_fallback", &settings_.unspecific_pos_acc_fallback_,
                      settings_.unspecific_pos_acc_fallback_);
    registerParameter("unspecifc_vel_acc_fallback", &settings_.unspecifc_vel_acc_fallback_,
                      settings_.unspecifc_vel_acc_fallback_);
    registerParameter("unspecifc_acc_acc_fallback", &settings_.unspecifc_acc_acc_fallback_,
                      settings_.unspecifc_acc_acc_fallback_);
    registerParameter("no_value_acc_fallback", &settings_.no_value_acc_fallback_, settings_.no_value_acc_fallback_);


    // target classification
    registerParameter("min_aircraft_modec", &base_settings_.min_aircraft_modec_, base_settings_.min_aircraft_modec_);

    registerParameter("vehicle_acids", &base_settings_.vehicle_acids_, {});
    base_settings_.setVehicleACIDs(base_settings_.vehicle_acids_);
    registerParameter("vehicle_acads", &base_settings_.vehicle_acads_, {});
    base_settings_.setVehicleACADs(base_settings_.vehicle_acads_);

    // reconstruction settings (check base for other settings)
    registerParameter("ref_rec_type", (int*)&referenceCalculatorSettings().kalman_type_assoc,
                      (int)kalman::KalmanType::UMKalman2D);
    registerParameter("ref_rec_type_final", (int*)&referenceCalculatorSettings().kalman_type_final,
                      (int)kalman::KalmanType::UMKalman2D);

    
}

SimpleReconstructor::~SimpleReconstructor() {}

SimpleAssociator& SimpleReconstructor::associator()
{
    return associatior_;
}

dbContent::VariableSet SimpleReconstructor::getReadSetFor(const std::string& dbcontent_name) const
{
    dbContent::VariableSet read_set;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

            // ds id
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));

            // line id
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_line_id_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_line_id_));

            // timestamp
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));

            // aircraft address
    if (dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_));

            // aircraft id
    if (dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acid_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acid_));

            // track num
    if (dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_num_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_num_));

            // track end
    if (dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_end_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_end_));

            // mode 3a
    if(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_));

            // mode c
    if(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_));

    if (dbcontent_name == "CAT062")
    {
        traced_assert(dbcont_man.canGetVariable(dbcontent_name, DBContent::var_cat062_fl_measured_));
        read_set.add(dbcont_man.getVariable(dbcontent_name, DBContent::var_cat062_fl_measured_));
    }

            // latitude
    if(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_latitude_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_latitude_));

            // longitude
    if(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_longitude_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_longitude_));

            // assoc
    if(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_utn_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_utn_));

            // rec num, must be last for update process
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_rec_num_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rec_num_));

            // adsb mops
    if (dbcontent_name == "CAT021")
    {
        traced_assert(dbcont_man.canGetVariable(dbcontent_name, DBContent::var_cat021_mops_version_));
        read_set.add(dbcont_man.getVariable(dbcontent_name, DBContent::var_cat021_mops_version_));
    }

    if(dbcont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_message_type_))
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_message_type_));

    if (dbcontent_name == "CAT065")
    {
        traced_assert(dbcont_man.canGetVariable(dbcontent_name, DBContent::var_cat065_batch_number_));
        read_set.add(dbcont_man.getVariable(dbcontent_name, DBContent::var_cat065_batch_number_));
    }

    read_set.add(dbContent::TargetReportAccessor::getReadSetFor(dbcontent_name));

    return read_set;
}

void SimpleReconstructor::reset()
{
    loginf;

//    target_reports_.clear(); // done in base
//    tr_timestamps_.clear();
//    tr_ds_.clear();

    associatior_.reset();
    ref_calculator_.reset();

    ReconstructorBase::reset();
}

SimpleReconstructorSettings& SimpleReconstructor::settings()
{
    return settings_;
}

SimpleReconstructorWidget* SimpleReconstructor::widget() // ownage by caller
{
    SimpleReconstructorWidget* widget = new SimpleReconstructorWidget(*this);

    connect (this, &SimpleReconstructor::updateWidgetsSignal,
            widget, &SimpleReconstructorWidget::updateSlot);

    return widget;
}

void SimpleReconstructor::updateWidgets()
{
    emit updateWidgetsSignal();
}

void SimpleReconstructor::processSlice_impl()
{
    loginf << "current_slice_begin "
           << Time::toString(currentSlice().slice_begin_)
           << " end " << Time::toString(currentSlice().slice_begin_ + settings().sliceDuration())
           << " is last " << currentSlice().is_last_slice_;

            // remove_before_time_, new data >= current_slice_begin_

    if (cancelled_)
        return;

    acc_estimator_->prepareForNewSlice(); // does nothing here

    if (cancelled_)
        return;

    clearOldTargetReports();

    if (cancelled_)
        return;

    ref_calculator_.settings() = referenceCalculatorSettings();
    ref_calculator_.prepareForNextSlice();

    if (cancelled_)
        return;

    createTargetReports();

    if (cancelled_)
        return;

    createTargetReportBatches();

    if (cancelled_)
        return;

    associatior_.associateNewData();

    if (cancelled_)
        return;

    std::map<unsigned int, std::map<unsigned long, unsigned int>> associations = createAssociations();
    // only for ts < write_before_time, also updates target counts
    currentSlice().assoc_data_ = createAssociationBuffers(associations);

    if (cancelled_)
        return;

    ref_calculator_.computeReferences();

    if (cancelled_)
        return;

    acc_estimator_->postProccessNewSlice(); // does nothing here

    if (cancelled_)
        return;

    currentSlice().reftraj_data_ = createReferenceBuffers(); // only for ts < write_before_time

    return;
}

void SimpleReconstructor::createAdditionalAnnotations()
{
    ref_calculator_.createAnnotations();
}
