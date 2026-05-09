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

#include "db_context.h"
#include "db_context_diff.h"
#include "radar_accuracy_defs.h"
#include "idatasourceprovider.h"

#include <json.hpp>

#include <QColor>
#include <QObject>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

class COMPASS;
class DataSourcesToolWidget;
class DataSourcesStatusToolWidget;
class SectorLayer;
class Sector;
class AirSpace;

namespace context
{

/**
 * Manages named DBContexts stored at ~/.compass/data_contexts/.
 * Owned by COMPASS. NOT Configurable — uses its own file-based persistence.
 *
 * Only one context is active at a time. On DB open, the stored context is
 * compared with the file definition; diffs must be resolved.
 *
 * Also provides all data source and FFT query/lookup methods that were
 * previously in DataSourceManager and FFTManager.
 */
class DBContextManager : public QObject, public IDataSourceProvider
{
    Q_OBJECT

signals:
    void activeContextChangedSignal();
    void contextsChangedSignal();
    void dataSourcesChangedSignal();
    void fftsChangedSignal();
    void sectorsChangedSignal();
    void countsChangedSignal();
    void asterixInfoChangedSignal();

public:
    explicit DBContextManager(COMPASS& compass);
    virtual ~DBContextManager();

    // ================================================================
    // Context CRUD
    // ================================================================
    std::vector<std::string> contextNames() const;
    bool hasContext(const std::string& name) const;
    const DBContext& context(const std::string& name) const;
    DBContext& context(const std::string& name);

    void createContext(const std::string& name);
    void deleteContext(const std::string& name);
    void renameContext(const std::string& old_name, const std::string& new_name);
    void duplicateContext(const std::string& src, const std::string& dest);
    void saveContext(const std::string& name);

    // ================================================================
    // Active context
    // ================================================================
    bool hasActiveContext() const;
    std::string activeContextName() const;
    const DBContext& activeContext() const;
    DBContext& activeContext();
    void setActiveContext(const std::string& name);

    // ================================================================
    // Data source query/lookup (replaces DataSourceManager)
    // ================================================================
    bool hasDataSource(unsigned int ds_id) const;
    const DataSource* dataSource(unsigned int ds_id) const;
    DataSource* dataSource(unsigned int ds_id);
    bool hasDataSource(const std::string& name) const;
    unsigned int getDataSourceId(const std::string& name) const;

    /// O(1) remote-unit name lookup. Returns the configured RU name for the
    /// given (data source, RU index) pair. Falls back to the index as a string
    /// when the data source or RU is unknown. Backed by a lazy cache that is
    /// invalidated on context / data-source mutations.
    std::string remoteUnitName(unsigned int ds_id, int ru_idx) const;

    std::vector<unsigned int> allDataSourceIds() const;
    std::vector<IDataSourceProvider::DataSourceInfo> dataSourceInfos() const override;

    std::set<unsigned int> groundOnlyDataSources() const;
    std::map<unsigned int, std::string> dsTypes() const; // ds_id -> type string

    // data source CRUD in active context
    DataSource& createDataSource(unsigned int sac, unsigned int sic,
                                     const std::string& name = "",
                                     const std::string& ds_type = "Other");
    void deleteDataSource(unsigned int ds_id);

    /// if ds.baseColor() is invalid, pick a hue-distant base color (using the
    /// active context's preference) and derive line_colors_. No-op otherwise.
    void autoAssignColors(DataSource& ds) const;

    // ================================================================
    // Data source loading/filtering (replaces DataSourceManager)
    // ================================================================
    bool dsTypeLoadingWanted(const std::string& ds_type) const;
    void dsTypeLoadingWanted(const std::string& ds_type, bool wanted);
    bool dsTypeFiltered() const;
    std::set<std::string> wantedDSTypes() const;
    void setLoadDSTypes(bool loading_wanted);
    void setLoadOnlyDSTypes(std::set<std::string> ds_types);

    bool loadingWanted(unsigned int ds_id) const;
    void loadingWanted(unsigned int ds_id, bool wanted);
    bool loadingWanted(const std::string& dbcontent_name) const;
    bool hasDSFilter(const std::string& dbcontent_name) const;
    bool hasDataSourcesOfDBContent(const std::string& dbcontent_name) const;

    void setLoadDataSources(bool loading_wanted);
    void setLoadOnlyDataSources(std::map<unsigned int, std::set<unsigned int>> ds_ids);
    void setLoadAllDataSourceLines();
    std::map<unsigned int, std::set<unsigned int>> getLoadDataSources() const;
    bool loadDataSourcesFiltered() const;

    bool lineSpecificLoadingRequired(const std::string& dbcontent_name) const;
    bool lineLoadingWanted(unsigned int ds_id, unsigned int line_id) const;
    void lineLoadingWanted(unsigned int ds_id, unsigned int line_id, bool wanted);

    void selectAllDSTypes();
    void deselectAllDSTypes();
    void selectAllDataSources();
    void deselectAllDataSources();
    void deselectAllLines();
    void selectSpecificLine(unsigned int line_id);
    void selectDSTypeSpecificDataSources(const std::string& ds_type);
    void deselectDSTypeSpecificDataSources(const std::string& ds_type);

    std::vector<unsigned int> unfilteredDS(const std::string& dbcontent_name) const;

    // ================================================================
    // Runtime counts (replaces DBDataSource counts, to be persisted in db_info)
    // ================================================================
    void setLoadedCounts(std::map<unsigned int, std::map<std::string, std::map<unsigned int, unsigned int>>> loaded_counts);
    void clearInsertedCounts(const std::string& dbcontent_name);
    void clearInsertedCounts(unsigned int ds_id, const std::string& dbcontent_name,
                             const std::vector<unsigned int>& line_ids = {});
    void applyDeleteInfo(const nlohmann::json& delete_info);

    void addNumInserted(unsigned int ds_id, const std::string& dbcontent_name,
                        unsigned int line_id, unsigned int count);
    void saveCountsToDB();
    void maxTimestamp(unsigned int ds_id, unsigned int line_id, boost::posix_time::ptime value);
    boost::posix_time::ptime maxTimestamp(unsigned int ds_id, unsigned int line_id) const;

    const std::map<unsigned int, std::map<std::string, std::map<unsigned int, unsigned int>>>& insertedCounts() const
    { return inserted_counts_; }

    unsigned int numInserted(unsigned int ds_id, const std::string& dbcontent_name) const;
    bool hasNumInserted(unsigned int ds_id) const;
    bool hasInsertedData() const;
    unsigned int numLoaded(unsigned int ds_id, const std::string& dbcontent_name) const;
    std::map<unsigned int, unsigned int> numInsertedPerLine(unsigned int ds_id, const std::string& dbcontent_name) const;
    std::map<unsigned int, unsigned int> numInsertedLinesMap(unsigned int ds_id) const;

    // ================================================================
    // Cumulative ASTERIX import info (per DS, per CAT, per data item)
    // Persisted in db_info under key "asterix_info"; cleared on DB close.
    // Populated by ASTERIXImportTask after each import via mergeAsterixInfo.
    // ================================================================
    struct AsterixItemStats
    {
        std::size_t    count = 0;
        nlohmann::json min;     // null if not provided / unknown
        nlohmann::json max;     // null if not provided / unknown
    };

    struct AsterixCategoryStats
    {
        std::size_t                                total_count = 0;
        std::map<std::string, AsterixItemStats>    items;
    };

    // ds_id -> cat -> { total_count, items[item_name] -> stats }
    using AsterixInfoMap = std::map<unsigned int,
                                    std::map<unsigned int, AsterixCategoryStats>>;

    const AsterixInfoMap& asterixInfo() const { return asterix_info_; }
    bool hasAsterixInfo(unsigned int ds_id) const;
    bool hasAnyAsterixInfo() const;

    /// Sum counts/total_count and refine min/max from `delta` into the cumulative store.
    /// Skips ds_id == 0 (the aggregator's "unknown SAC/SIC" bucket).
    void mergeAsterixInfo(const AsterixInfoMap& delta);

    void saveAsterixInfoToDB();

    /// Pure data-manipulation helpers — exposed for unit testing.
    static void mergeAsterixInfoInto(AsterixInfoMap& dst, const AsterixInfoMap& delta);
    static nlohmann::json asterixInfoToJSON(const AsterixInfoMap& src);
    static AsterixInfoMap asterixInfoFromJSON(const nlohmann::json& j);

    // ================================================================
    // Network lines (replaces DataSourceManager network methods)
    // ================================================================
    std::map<unsigned int, std::map<std::string, nlohmann::json>> getNetworkLines() const;
    void createNetworkDBDataSources();

    // ================================================================
    // Sensor status config (replaces DataSourceManager::Config)
    // ================================================================
    struct SensorConfig
    {
        double primary_azimuth_stddev{0.05};
        double primary_range_stddev{120.0};
        double secondary_azimuth_stddev{0.025};
        double secondary_range_stddev{70.0};
        double mode_s_azimuth_stddev{0.02};
        double mode_s_range_stddev{50.0};

        RadarAccuracyDefaults radarAccuracyDefaults() const
        {
            return {primary_azimuth_stddev, primary_range_stddev,
                    0.05, 7.5, // SMR defaults (ground-only PSR)
                    secondary_azimuth_stddev, secondary_range_stddev,
                    mode_s_azimuth_stddev, mode_s_range_stddev};
        }

        unsigned int ds_font_size{10};

        nlohmann::json sensor_status_max_status_age_options = nlohmann::json::array({10u, 20u, 30u, 40u, 50u, 60u});
        unsigned int sensor_status_max_status_age_index{1};
        unsigned int sensor_status_max_event_buf_size{1000};
        bool sensor_status_show_last_updates{false};

        double sensorStatusMaxStatusAgeValue() const;
        double sensorStatusMaxStatusAgeMaxValue() const;
    };

    SensorConfig& sensorConfig() { return sensor_config_; }
    const SensorConfig& sensorConfig() const { return sensor_config_; }

    // ================================================================
    // FFT query/lookup (replaces FFTManager)
    // ================================================================
    bool hasFFT(const std::string& name) const;
    const FFT* fft(const std::string& name) const;
    FFT* fft(const std::string& name);
    std::vector<std::string> allFFTNames() const;

    FFT& createFFT(const std::string& name);
    void deleteFFT(const std::string& name);
    void deleteAllFFTs();

    /// Check if a position/codes match an FFT. Returns {is_fft, altitude_ft}.
    std::pair<bool, float> isFromFFT(double latitude_deg, double longitude_deg,
                                     boost::optional<unsigned int> mode_s_address,
                                     bool ignore_mode_s,
                                     boost::optional<unsigned int> mode_a_code,
                                     boost::optional<float> mode_c_code) const;

    // ================================================================
    // Sector layer access (cached view of context sectors)
    // ================================================================
    bool sectorsLoaded() const;
    std::vector<std::shared_ptr<SectorLayer>>& sectorLayers();
    const std::vector<std::shared_ptr<SectorLayer>>& sectorLayers() const;
    bool hasSectorLayer(const std::string& layer_name) const;
    std::shared_ptr<SectorLayer> sectorLayer(const std::string& layer_name) const;
    bool hasSector(const std::string& name, const std::string& layer_name) const;
    bool hasSector(unsigned int id) const;
    std::shared_ptr<Sector> sector(const std::string& name, const std::string& layer_name) const;
    std::shared_ptr<Sector> sector(unsigned int id) const;
    unsigned int maxSectorId() const;

    // sector CRUD
    std::shared_ptr<Sector> createSector(const std::string& name, const std::string& layer_name,
                                          bool exclude, QColor color,
                                          std::vector<std::pair<double,double>> points);
    void deleteSector(std::shared_ptr<Sector> sector);
    void deleteAllSectors();
    void saveSector(unsigned int id);
    void saveSector(std::shared_ptr<Sector> sector);
    void moveSector(unsigned int id, const std::string& old_layer, const std::string& new_layer);

    void importAirSpace(const AirSpace& air_space,
                        const std::map<std::string, bool>& sectors_to_import,
                        const std::string& target_layer_name = "");

    // ================================================================
    // ASTERIX decoding config access
    // ================================================================
    bool hasAsterixConfig(unsigned int category) const;
    ASTERIXDecodingConfig* asterixConfig(unsigned int category);
    const ASTERIXDecodingConfig* asterixConfig(unsigned int category) const;
    ASTERIXDecodingConfig& getOrCreateAsterixConfig(unsigned int category,
        const std::string& default_edition = "",
        const std::string& default_ref = "",
        const std::string& default_spf = "");

    void setAsterixEdition(unsigned int category, const std::string& edition,
                           const std::string& default_ref = "");
    void setAsterixRef(unsigned int category, const std::string& ref,
                       const std::string& default_edition = "");
    void setAsterixSpf(unsigned int category, const std::string& spf,
                       const std::string& default_edition = "");

    // ================================================================
    // DB sync
    // ================================================================
    void writeContextToDB();
    DBContext readContextFromDB() const;

    // ================================================================
    // Diff
    // ================================================================
    DBContextDiff diffWithDB() const;
    DBContextDiff diff(const DBContext& a, const DBContext& b) const;

    // ================================================================
    // Per-section import/export (from legacy JSON formats)
    // ================================================================
    void importSensors(const std::string& filepath);
    void importFFTs(const std::string& filepath);
    void importSectors(const std::string& filepath);
    void exportSensors(const std::string& filepath);
    void exportFFTs(const std::string& filepath);
    void exportSectors(const std::string& filepath);

    // import/export a full context
    void exportContext(const std::string& name, const std::string& filepath);
    void importContext(const std::string& filepath);

    // zip-based full context export/import
    void exportContextZip(const std::string& name, const std::string& zip_filepath);
    void importContextZip(const std::string& zip_filepath);

    // ================================================================
    // Widgets (lazy creation, owned by this manager)
    // ================================================================
    DataSourcesToolWidget* loadWidget();
    DataSourcesStatusToolWidget* statusWidget();

    // ================================================================
    // Utility
    // ================================================================
    void rebuildSectorLayers();
    static std::string basePath();
    COMPASS& compass() { return compass_; }

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();

private:
    void loadContextList();
    void saveActiveContextName();
    void loadActiveContextName();
    void loadCountsFromDB();
    void loadAsterixInfoFromDB();

    void ensureDataSourceCache() const;
    void invalidateDataSourceCache() const;

    COMPASS& compass_;
    std::string active_context_name_;
    std::map<std::string, DBContext> contexts_;

    // cached sector layers (rebuilt from context sectors)
    std::vector<std::shared_ptr<SectorLayer>> sector_layers_;
    unsigned int max_sector_id_{0};
    bool sectors_loaded_{false};

    // runtime loading state (not persisted in context files, per-session)
    std::map<std::string, bool> ds_type_loading_wanted_;           // ds_type -> wanted
    std::map<unsigned int, bool> ds_loading_wanted_;               // ds_id -> wanted
    std::map<unsigned int, std::map<unsigned int, bool>> line_loading_wanted_; // ds_id -> line_id -> wanted

    // runtime counts (per DB session, persisted in db_info on close)
    // ds_id -> dbcontent_name -> line_id -> count
    std::map<unsigned int, std::map<std::string, std::map<unsigned int, unsigned int>>> inserted_counts_;
    std::map<unsigned int, std::map<std::string, std::map<unsigned int, unsigned int>>> loaded_counts_;

    // cumulative ASTERIX probe stats for this DB (persisted in db_info on close)
    AsterixInfoMap asterix_info_;

    // runtime max timestamps: ds_id -> line_id -> max_timestamp
    std::map<unsigned int, std::map<unsigned int, boost::posix_time::ptime>> max_timestamps_;

    SensorConfig sensor_config_;

    // widgets (lazy, owned)
    std::unique_ptr<DataSourcesToolWidget> load_widget_;
    std::unique_ptr<DataSourcesStatusToolWidget> status_widget_;

    static constexpr double max_fft_plot_distance_m_ = 5000.0;

    // Lazy RU-name cache over active context's data sources. ds_id lookup
    // itself is served directly by activeContext().dataSources() (a std::map),
    // so no by-id cache is needed. Invalidated on context switch and on any
    // saveContext (which is called after every edit and structural change).
    // Build is O(R), where R is the total RU count of the active context.
    mutable std::unordered_map<unsigned int, std::unordered_map<int, std::string>> ru_name_cache_;
    mutable bool ds_cache_valid_ {false};
};

} // namespace context
