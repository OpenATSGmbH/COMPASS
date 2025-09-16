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
#include "evaluationdata.h"
#include "evaluationresultsgenerator.h"
#include "evaluationcalculator.h"
#include "viewabledataconfig.h"
#include "evaluationmanagerwidget.h"
#include "eval/results/report/pdfgenerator.h"
#include "sector.h"
#include "result.h"
#include "util/timewindow.h"

#include <QObject>

#include "json_fwd.hpp"

class COMPASS;
class EvaluationStandard;
class DBContent;
class DBContentManager;
class SectorLayer;
class AirSpace;
struct EvaluationSettings;
class EvaluationTargetFilter;

namespace dbContent 
{
    class VariableSet;
}

class QWidget;

/**
 */
class EvaluationManager : public QObject, public Configurable
{
    Q_OBJECT

signals:
    void sectorsChangedSignal();         // sectors changed (due to manual editing, loading, etc.)
    //void sectorsEditedSignal();          // sector edit dialog closed
    void standardsChangedSignal();       // emitted if standard was added or deleted
    void currentStandardChangedSignal(); // emitted if current standard was changed
    void evaluationDoneSignal();         // evaluation ended
    void hasNewData();                   // new data for evaluation is ready to be fetched
    void resultsNeedUpdate(int type);    // evaluation results need the sent update

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();
    void dataSourcesChangedSlot();
    void associationStatusChangedSlot();

    void targetInfoChangedSlot();
    void partialResultsUpdateNeededSlot();
    void fullResultsUpdateNeededSlot();
    void lockResultsSlot();

public:
    EvaluationManager(const std::string& class_id, const std::string& instance_id, COMPASS* compass);
    virtual ~EvaluationManager();

    void init();
    void close();
    void clearData();

    Result canEvaluate() const;
    Result evaluate(bool show_dialog);

    // sectors
    bool sectorsLoaded() const;

    bool hasSectorLayer (const std::string& layer_name) const;
    //void renameSectorLayer (const std::string& name, const std::string& new_name);
    std::shared_ptr<SectorLayer> sectorLayer(const std::string& layer_name) const;

    void createNewSector (const std::string& name, const std::string& layer_name,
                          bool exclude, QColor color, std::vector<std::pair<double,double>> points);
    bool hasSector (const std::string& name, const std::string& layer_name);
    bool hasSector (unsigned int id);
    std::shared_ptr<Sector> sector (const std::string& name, const std::string& layer_name);
    std::shared_ptr<Sector> sector (unsigned int id);
    void moveSector(unsigned int id, const std::string& old_layer_name, const std::string& new_layer_name); // moves sector to new layer, saves
    void saveSector(unsigned int id); // write to db and add
    void saveSector(std::shared_ptr<Sector> sector); // write to db and add
    std::vector<std::shared_ptr<SectorLayer>>& sectorsLayers();
    void deleteSector(std::shared_ptr<Sector> sector);
    void deleteAllSectors();
    void importSectors (const std::string& filename);
    void exportSectors (const std::string& filename);
    unsigned int getMaxSectorId ();

    bool importAirSpace(const AirSpace& air_space, 
                        const boost::optional<std::set<std::string>>& sectors_to_import = {});

    //data loading
    bool needsAdditionalVariables() const;
    void addVariables(const std::string dbcontent_name, dbContent::VariableSet& read_set);

    //viewables
    void setViewableDataConfig (const nlohmann::json::object_t& data);
    void resetViewableDataConfig(bool reset_view_point);

    //timestamps
    boost::posix_time::ptime loadTimestampBegin() const;
    void loadTimestampBegin(boost::posix_time::ptime value);

    boost::posix_time::ptime loadTimestampEnd() const;
    void loadTimestampEnd(boost::posix_time::ptime value);

    Utils::TimeWindowCollection& excludedTimeWindows(); // needs to be saved externally
    void saveTimeConstraints();

    bool useTimestampFilter() const;
    void useTimestampFilter(bool value);
    std::string timestampFilterStr() const;

    //standards
    bool hasCurrentStandard() const;
    const EvaluationStandard& currentStandard() const;

    //other stuff
    EvaluationTargetFilter& targetFilter() const;
    const std::string& lastResultName() const { return last_result_name_; }

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;
protected:
    friend class EvaluationCalculator;

    virtual void checkSubConfigurables() override;
    virtual void onConfigurationChanged(const std::vector<std::string>& changed_params) override;

    void updateSectorLayers();

    void loadData(const EvaluationCalculator& calculator,
                  bool blocking);
    std::map<std::string, std::shared_ptr<Buffer>> fetchData();

private:
    void loadSectors();
    void clearSectors();
    void updateMaxSectorID();

    void configureLoadFilters(const EvaluationCalculator& calculator);
    void loadingDone();

    DBContentManager& dbcontent_man_;

    bool sectors_loaded_ {false};
    bool initialized_ {false};
    bool active_load_connection_ {false};

    bool needs_additional_variables_ {false}; // indicates if variables should be added during loading

    std::vector<std::shared_ptr<SectorLayer>> sector_layers_;
    unsigned int max_sector_id_ {0};

    std::unique_ptr<EvaluationTargetFilter> target_filter_;
    std::unique_ptr<EvaluationCalculator> calculator_; // sub-configurable
    //EvaluationResultsReport::PDFGenerator pdf_gen_;

    std::unique_ptr<ViewableDataConfig>            viewable_data_cfg_;
    std::map<std::string, std::shared_ptr<Buffer>> raw_data_;
    bool                                           raw_data_available_ = false;

    bool use_timestamp_filter_ {false};
    boost::posix_time::ptime load_timestamp_begin_;
    boost::posix_time::ptime load_timestamp_end_;
    Utils::TimeWindowCollection load_filtered_time_windows_;

    std::string last_result_name_;
};
