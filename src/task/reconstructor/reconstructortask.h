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

#include "configurable.h"
#include "reconstructorbase.h"
#include "task.h"
#include "global.h"
#include "json_fwd.hpp"

#include <QObject>

#include <memory>
#include <future>

class TaskManager;
class DBContent;
class Buffer;
class ReconstructorBase;
class SimpleReconstructor;

class ViewPointGenVP;
class ViewPointGenAnnotation;

#if USE_EXPERIMENTAL_SOURCE == true
class ProbIMMReconstructor;
#endif

namespace dbContent
{
class Variable;
class MetaVariable;
}

class QProgressDialog;

class ReconstructorTask : public Task, public Configurable
{
    Q_OBJECT

public:
    struct DebugSettings
    {
        std::set<unsigned int> debug_utns_;
        std::set<unsigned long> debug_rec_nums_;

        boost::posix_time::ptime debug_timestamp_min_;
        boost::posix_time::ptime debug_timestamp_max_;

        bool debug_ {false};
        bool debug_association_ {false};
        bool debug_outlier_detection_ {false};
        bool debug_geo_altitude_correction_ {false}; // not yet set using gui
        bool debug_reference_calculation_ {false};
        bool debug_kalman_chains_ {false};
        bool debug_jpda_ {false};
        bool debug_tentative_targets_ {false};
        bool debug_write_reconstruction_viewpoints_ {false};

        bool analyze_ {false};
        bool analyze_association_ {false};
        bool analyze_outlier_detection_ {false};
        bool analyze_accuracy_estimation_ {false};
        bool analyze_bias_correction_ {false};
        bool analyze_geo_altitude_correction_ {false};

        bool stats_ {false};
        bool stats_reference_calculation_ {false};
        bool stats_kalman_chains_ {false};
        bool stats_jpda_ {false};
        bool stats_tentative_targets_ {false};
        bool stats_adsb_ {false};
        bool stats_association_ {false};

        bool debugUTN(unsigned int utn) { return debug_utns_.count(utn); }
        bool debugRecNum(unsigned long rec_num) { return debug_rec_nums_.count(rec_num); }

        double grid_max_resolution_{0.0005};
        unsigned int max_num_grid_cells_ {500};
    };

signals:
    void configChanged();

public slots:
    void deleteCalculatedReferencesDoneSlot();
    void deleteTargetsDoneSlot();
    void deleteAssociationsDoneSlot();

    void sectorsChangedSlot();

    void loadedDataSlot(const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset);
    void loadingDoneSlot();

    void processingDoneSlot();
    void writeDoneSlot();

    void runCancelledSlot();

    void updateProgressSlot(const QString& msg, bool add_slice_progress);

public:
    ReconstructorTask(const std::string& class_id, const std::string& instance_id,
                      TaskManager& task_manager);
    virtual ~ReconstructorTask();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    virtual bool canRun() override;
    virtual void run() override;

    std::string currentReconstructorStr() const;
    void currentReconstructorStr(const std::string& value);

    static const std::string ScoringUMReconstructorName;

    ReconstructorBase* currentReconstructor() const;
    SimpleReconstructor* simpleReconstructor() const;

#if USE_EXPERIMENTAL_SOURCE == true
    static const std::string ProbImmReconstructorName;

    ProbIMMReconstructor* probIMMReconstructor() const;
#endif

    std::set<unsigned int> disabledDataSources() const;

    bool useDStype(const std::string& ds_type) const;
    void useDSType(const std::string& ds_type, bool value);
    bool useDataSource(unsigned int ds_id) const;
    void useDataSource(unsigned int ds_id, bool value);
    bool useDataSourceLine(unsigned int ds_id, unsigned int line_id) const;
    void useDataSourceLine(unsigned int ds_id, unsigned int line_id, bool value);

    std::set<unsigned int> unusedDSIDs() const;
    std::map<unsigned int, std::set<unsigned int>> unusedDSIDLines() const;

    bool useSectorsExtend() const;
    void useSectorsExtend(bool value);
    
    const std::map<std::string, bool>& usedSectors() const;
    void useSector(const std::string& sector_name, bool value);

    float sectorDeltaDeg() const { return sector_delta_deg_; }
    void sectorDeltaDeg(float value) { sector_delta_deg_ = value; }

    ReconstructorBase::DataSlice& processingSlice();
    const ReconstructorBase::DataSlice& processingSlice() const;

    std::unique_ptr<ViewPointGenVP> getDebugViewpoint(const std::string& name, const std::string& type, bool* created = nullptr) const;
    std::unique_ptr<ViewPointGenVP> getDebugViewpointNoData(const std::string& name, const std::string& type); // w/o sur data
    std::unique_ptr<ViewPointGenVP> getDebugViewpointForUTN(unsigned long utn, const std::string& name_prefix="") const;

    bool skipReferenceDataWriting() const;
    void skipReferenceDataWriting(bool newSkip_reference_data_writing);

    void showDialog();

    void checkReconstructor();

    virtual void updateFeatures() override final;
    virtual void initTask() override final;

    DebugSettings& debugSettings() { return debug_settings_; }

protected:
    virtual void checkSubConfigurables() override;
    void deleteCalculatedReferences();

    void loadDataSlice();
    void processDataSlice();
    void writeDataSlice();
    void endReconstruction();

    void finalizeSlice(std::unique_ptr<ReconstructorBase::DataSlice>& slice);

    std::string current_reconstructor_str_;

    nlohmann::json use_dstypes_; // dstype -> bool
    nlohmann::json use_data_sources_; // ds_id -> bool
    nlohmann::json use_data_sources_lines_; // ds_id -> line_id -> bool

    bool use_sectors_extend_{false};
    std::map<std::string, bool> used_sectors_;
    float sector_delta_deg_ {0.1};

    std::unique_ptr<SimpleReconstructor> simple_reconstructor_; // has to be reset after each calculation

#if USE_EXPERIMENTAL_SOURCE == true
    std::unique_ptr<ProbIMMReconstructor> probimm_reconstructor_; // has to be reset after each calculation
#endif

    std::unique_ptr<QProgressDialog> progress_dialog_;
    boost::posix_time::ptime run_start_time_;
    boost::posix_time::ptime run_start_time_after_del_;

    size_t current_slice_idx_ = 0;

    std::unique_ptr<ReconstructorBase::DataSlice> loading_slice_;
    bool loading_data_ {false};
    std::unique_ptr<ReconstructorBase::DataSlice> processing_slice_;
    std::unique_ptr<ReconstructorBase::DataSlice> writing_slice_;

    DebugSettings debug_settings_;

    std::future<void> delcalcref_future_;
    std::future<void> deltgts_future_;
    std::future<void> delassocs_future_;
    std::future<void> process_future_;
    bool processing_data_slice_ {false};
    bool cancelled_ {false};

    bool skip_reference_data_writing_ {false};

    //mutable std::map<std::pair<std::string,std::string>, std::unique_ptr<ViewPointGenVP>> debug_viewpoints_;
};
