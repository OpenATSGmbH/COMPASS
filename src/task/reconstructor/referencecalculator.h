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

#include "referencecalculatorsettings.h"
#include "referencecalculatordefs.h"
#include "referencecalculatorannotations.h"
#include "test_target.h"

#include <vector>

#include <QColor>

class ReconstructorBase;

namespace dbContent 
{
    class ReconstructorTarget;
}

class ViewPointGenAnnotation;

namespace reconstruction
{
    class KalmanEstimator;
}

/**
 * Data for target export.
*/
struct ReferenceCalculatorTargetExportData
{
    TestTarget test_target;
    boost::optional<boost::posix_time::ptime> t0;
    double lat_;
    double lon_;
};

/**
 * Target reference data.
*/
struct ReferenceCalculatorTargetReferences
{
    typedef ReferenceCalculatorInputInfo InputInfo;

    void reset();
    void resetCounts();

    unsigned int utn;

    std::vector<reconstruction::Measurement>    measurements;
    std::vector<reconstruction::MMContribution> measurement_contributions;
    std::vector<kalman::KalmanUpdate>           updates;
    std::vector<kalman::KalmanUpdate>           updates_smooth;
    std::vector<double>                         updates_smooth_Qvars;
    std::vector<reconstruction::Reference>      references;
    std::map<kalman::UniqueUpdateID, InputInfo> input_infos;

    boost::optional<kalman::KalmanUpdate> init_update;
    boost::optional<size_t>               start_index;

    size_t num_updates                 = 0;
    size_t num_updates_ccoeff_corr     = 0;
    size_t num_updates_valid           = 0;
    size_t num_updates_raf             = 0; //reinit after fail
    size_t num_updates_raf_numeric     = 0;
    size_t num_updates_raf_badstate    = 0;
    size_t num_updates_raf_other       = 0;
    size_t num_updates_failed          = 0;
    size_t num_updates_failed_numeric  = 0;
    size_t num_updates_failed_badstate = 0;
    size_t num_updates_failed_other    = 0;
    size_t num_updates_skipped         = 0;
    size_t num_smooth_steps_failed     = 0;
    size_t num_smoothing_failed        = 0;
    size_t num_interp_steps_failed     = 0;

    refcalc_annotations::ReferenceCalculatorAnnotations annotations;

    std::unique_ptr<ReferenceCalculatorTargetExportData> export_data;
};

/**
*/
class ReferenceCalculator
{
public:
    typedef ReferenceCalculatorSettings         Settings;
    typedef ReferenceCalculatorTargetReferences TargetReferences;

    typedef std::vector<reconstruction::Measurement>                       Measurements;
    typedef std::multimap<boost::posix_time::ptime, unsigned long>         TargetReports;
    typedef std::map<unsigned int, std::vector<reconstruction::Reference>> References;

    ReferenceCalculator(ReconstructorBase& reconstructor);
    virtual ~ReferenceCalculator();

    void prepareForNextSlice();
    void prepareForCurrentSlice();
    bool computeReferences();

    void reset();

    void createAnnotations();

    Settings& settings() { return settings_; }
    
    static std::vector<std::vector<reconstruction::Measurement>> splitMeasurements(const Measurements& measurements,
                                                                                   double max_dt);
    static const QColor ColorMeasurements;
    static const QColor ColorKalman;
    static const QColor ColorKalmanSmoothed;
    static const QColor ColorKalmanResampled;

    static const float PointSizeOSG;
    static const float PointSizeMeasurements;
    static const float PointSizeKalman;
    static const float PointSizeKalmanSmoothed;
    static const float PointSizeKalmanResampled;

    static const float LineWidthBase;

protected:
    virtual void reconstructMeasurements();
                                
private:
    enum class InitRecResult 
    {
        NoMeasurements = 0,
        NoStartIndex,
        Success
    };

    void resetDataStructs();
    void updateInterpOptions();

    void generateMeasurements();
    void generateTargetMeasurements(const dbContent::ReconstructorTarget& target);
    void generateLineMeasurements(const dbContent::ReconstructorTarget& target,
                                  unsigned int dbcontent_id,
                                  unsigned int sensor_id,
                                  unsigned int line_id,
                                  const TargetReports& target_reports);
    
    void addMeasurements(unsigned int utn,
                         unsigned int dbcontent_id, 
                         Measurements& measurements);
    void preprocessMeasurements(unsigned int dbcontent_id, 
                                Measurements& measurements);
    void interpolateMeasurements(Measurements& measurements, 
                                 const reconstruction::InterpOptions& options) const;
    
    InitRecResult initReconstruction(TargetReferences& refs);
    void reconstructMeasurements(TargetReferences& refs);
    void reconstructSmoothMeasurements(std::vector<kalman::KalmanUpdate>& updates,
                                       TargetReferences& refs,
                                       reconstruction::KalmanEstimator& estimator);
    void obtainRemainingUpdates(std::vector<kalman::KalmanUpdate>& updates,
                                TargetReferences& refs,
                                reconstruction::KalmanEstimator& estimator);

    void updateReferences();

    boost::posix_time::ptime getJoinThreshold() const;

    bool writeTargetData(TargetReferences& refs,
                         const std::string& fn);

    bool shallAddAnnotationData() const;

    ReconstructorBase& reconstructor_;

    Settings settings_;

    int  slice_idx_     = -1;
    bool is_last_slice_ = false;

    std::map<unsigned int, TargetReferences>              references_;
    std::map<unsigned int, reconstruction::InterpOptions> interp_options_;
};
