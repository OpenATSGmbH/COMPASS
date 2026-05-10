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

#include "dbcontent/target/targetreportchain.h"

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

#include <map>
#include <memory>
#include <set>
#include <string>

class COMPASS;

namespace dbContent
{
class DBContentAccessor;
}

/**
 * @brief Combined dataset folded across all selected data sources for an
 *        Analyse-Data-Source run, like the Evaluation builds for test sources.
 *
 * Configures load filters (selected DS / lines), blocks the load through
 * `DBContentManager::loadBlocking`, and packages the loaded buffers into
 * per-UTN reference / test `dbContent::TargetReport::Chain`s suitable for
 * the inspectors to walk.
 *
 * The reference chain is always built from `RefTraj`. Test chains are built
 * from the configured test dbcontents (e.g. `CAT020`, `CAT010`); a UTN may
 * have separate test chains in different dbcontents, kept apart so that
 * cat-specific accessors (e.g. I020/500 std-devs) remain meaningful.
 *
 * The dataset is general: the caller supplies the test dbcontents to load
 * and the set of selected data source IDs to filter by.
 */
class AnalysisDataset
{
public:
    explicit AnalysisDataset(COMPASS& compass);
    ~AnalysisDataset();

    /**
     * Configure load filters and block-load the requested dbcontents.
     * @param selected_ds_ids   test data source IDs to fold into the combined dataset
     * @param line_id_tst       line ID (0..3) to load on the test side
     * @param ref_ds_ids        reference (RefTraj) data source IDs to load
     * @param line_id_ref       line ID (0..3) to load on the reference side
     * @param test_dbcontents   dbcontent names to load on the test side
     *                          (the reference dbcontent "RefTraj" is always added)
     * @param error_out         filled with a human-readable error if the load fails
     * @return true on success.
     */
    bool load(const std::set<unsigned int>& selected_ds_ids,
              unsigned int line_id_tst,
              const std::set<unsigned int>& ref_ds_ids,
              unsigned int line_id_ref,
              const std::set<std::string>& test_dbcontents,
              std::string& error_out);

    /// True if at least one UTN has both reference and test data.
    bool hasUsableData() const;

    /// All UTNs that have any data in the dataset.
    const std::set<unsigned int>& utns() const { return utns_; }

    /// True if `utn` has reference (RefTraj) data.
    bool hasReferenceChain(unsigned int utn) const;

    /// True if `utn` has test data in `dbcontent_name`.
    bool hasTestChain(unsigned int utn, const std::string& dbcontent_name) const;

    /// Reference chain (RefTraj) for `utn`. Caller must check `hasReferenceChain`.
    dbContent::TargetReport::Chain& referenceChain(unsigned int utn);

    /// Test chain for (`utn`, `dbcontent_name`). Caller must check `hasTestChain`.
    dbContent::TargetReport::Chain& testChain(unsigned int utn,
                                              const std::string& dbcontent_name);

    /// Test dbcontent names actually present in the dataset (subset of those requested).
    const std::set<std::string>& testDbContentsPresent() const { return tst_dbcontents_present_; }

    /// Reference content size (number of RefTraj rows folded in, before per-UTN filtering).
    unsigned int numReferenceRecordsTotal() const { return num_ref_records_total_; }

    /// Total test record count across all dbcontents.
    unsigned int numTestRecordsTotal() const { return num_tst_records_total_; }

    /// Number of (utn, dbcontent) test chains.
    unsigned int numTestChains() const;

    /// Interpolate the reference position on `utn`'s RefTraj at `timestamp`.
    /// Returns boost::none if no RefTraj data exists for `utn` or the timestamp
    /// is outside the chain's bracket. `d_max` bounds the interpolation gap.
    boost::optional<dbContent::TargetPosition>
        mappedRefPos(unsigned int utn,
                     boost::posix_time::ptime timestamp,
                     boost::posix_time::time_duration d_max) const;

    /// Approximate centre latitude of the loaded reference data (used for
    /// degree/metre conversion in the 3D grid). 0.0 if unknown.
    double centreLatitudeDeg() const { return centre_lat_deg_; }

    /// Bounds of the loaded data (latitude / longitude). Both default-constructed
    /// when no positions were loaded.
    double minLatitudeDeg() const { return min_lat_; }
    double maxLatitudeDeg() const { return max_lat_; }
    double minLongitudeDeg() const { return min_lon_; }
    double maxLongitudeDeg() const { return max_lon_; }

    /// Altitude bounds of the loaded reference data (feet, barometric).
    /// `hasAltitudeExtent()` is false when no reference rows had a usable
    /// Mode-C value; in that case `min/max` are 0.
    bool   hasAltitudeExtent() const { return has_altitude_extent_; }
    double minAltitudeFt()    const { return min_alt_ft_; }
    double maxAltitudeFt()    const { return max_alt_ft_; }

private:
    void buildChains(const std::set<unsigned int>& selected_ds_ids,
                     const std::set<unsigned int>& ref_ds_ids,
                     const std::set<std::string>& test_dbcontents);
    void addToReferenceChain(unsigned int utn, boost::posix_time::ptime ts, unsigned int idx);
    void addToTestChain(unsigned int utn, const std::string& dbcontent,
                        boost::posix_time::ptime ts, unsigned int idx);

    COMPASS& compass_;
    std::shared_ptr<dbContent::DBContentAccessor> accessor_;

    static constexpr const char* kReferenceDBContent = "RefTraj";

    /// Map keyed by UTN -> reference chain (RefTraj).
    std::map<unsigned int, std::unique_ptr<dbContent::TargetReport::Chain>> ref_chains_;
    /// Map keyed by (UTN, dbcontent_name) -> test chain.
    std::map<std::pair<unsigned int, std::string>,
             std::unique_ptr<dbContent::TargetReport::Chain>> tst_chains_;

    std::set<unsigned int> utns_;
    std::set<std::string>  tst_dbcontents_present_;

    unsigned int num_ref_records_total_ = 0;
    unsigned int num_tst_records_total_ = 0;

    double centre_lat_deg_ = 0.0;
    double min_lat_ = 0.0, max_lat_ = 0.0;
    double min_lon_ = 0.0, max_lon_ = 0.0;
    bool   has_position_extent_ = false;

    double min_alt_ft_ = 0.0, max_alt_ft_ = 0.0;
    bool   has_altitude_extent_ = false;
};
