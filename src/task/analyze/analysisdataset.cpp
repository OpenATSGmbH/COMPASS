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

#include "analysisdataset.h"

#include "buffer.h"
#include "compass.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentaccessor.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentstatusinfo.h"
#include "dbcontent/target/targetbase.h"
#include "dbcontent/variable/variableset.h"
#include "targetposition.h"
#include "idbvariableresolver.h"
#include "loadrequest.h"
#include "logger.h"
#include "viewmanager.h"

#include <algorithm>

#include <boost/date_time/posix_time/posix_time.hpp>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;

namespace
{
constexpr unsigned int kInvalidIdx = std::numeric_limits<unsigned int>::max();
}

AnalysisDataset::AnalysisDataset(COMPASS& compass)
    : compass_(compass)
{
    accessor_ = std::make_shared<dbContent::DBContentAccessor>(compass.dbContentManager());
}

AnalysisDataset::~AnalysisDataset() = default;

void AnalysisDataset::loadStatusCycles()
{
    status_cycles_.clear();

    auto& dbcontent_man = compass_.dbContentManager();

    const std::string status_content = "CAT019";

    if (!dbcontent_man.existsDBContent(status_content))
        return;
    auto& dbcont = dbcontent_man.dbContent(status_content);
    if (!dbcont.loadable() || !dbcont.containsStatusContent())
        return;

    dbContent::DBContentStatusInfo status_info(dbcontent_man);
    dbContent::VariableSet status_rs = status_info.getReadSetFor(status_content);

    LoadRequest req = LoadRequest::forContent(status_content, status_rs);
    req.show_status_ = false;
    req.cancellable_ = false;

    auto& view_man = compass_.viewManager();

    dbcontent_man.clearData();
    view_man.disableDataDistribution(true);
    dbcontent_man.loadBlocking(req);
    view_man.disableDataDistribution(false);

    auto status_buffers = dbcontent_man.loadedData();
    if (!status_buffers.empty())
    {
        status_info.process(status_buffers);

        for (const auto& ds : status_info.getInfo())
            for (const auto& line : ds.second)
                status_cycles_.insert(status_cycles_.end(),
                                       line.second.begin(),
                                       line.second.end());

        std::sort(status_cycles_.begin(), status_cycles_.end());
        status_cycles_.erase(std::unique(status_cycles_.begin(),
                                          status_cycles_.end()),
                              status_cycles_.end());
    }

    dbcontent_man.clearData();

    loginf << "loaded " << status_cycles_.size()
           << " " << status_content << " start-of-cycle timestamps";
}

bool AnalysisDataset::load(const std::set<unsigned int>& selected_ds_ids,
                           unsigned int line_id_tst,
                           const std::set<unsigned int>& ref_ds_ids,
                           unsigned int line_id_ref,
                           const std::set<std::string>& test_dbcontents,
                           std::string& error_out)
{
    error_out.clear();

    if (selected_ds_ids.empty())
    {
        error_out = "No data sources selected.";
        return false;
    }

    auto& dbcontent_man = compass_.dbContentManager();

    if (!dbcontent_man.hasAssociations())
    {
        error_out = "No reconstruction associations are present; "
                    "run reconstruction before analyzing the data source.";
        return false;
    }

    auto& ctx_man = compass_.dbContextManager();

    if (ref_ds_ids.empty())
    {
        error_out = "No reference data source (RefTraj) selected.";
        return false;
    }

    // Loading filter: selected test data sources on the configured test line,
    // plus the configured reference data sources on the configured ref line.
    std::map<unsigned int, std::set<unsigned int>> ds_load_map;
    for (auto ds_id : selected_ds_ids)
        ds_load_map[ds_id] = { line_id_tst };

    for (auto ref_ds_id : ref_ds_ids)
    {
        if (!ctx_man.hasDataSource(ref_ds_id))
            continue;
        ds_load_map[ref_ds_id] = { line_id_ref };
    }

    ctx_man.setLoadDSTypes(true);
    ctx_man.setLoadOnlyDataSources(ds_load_map);

    // Mirror EvaluationManager::loadData / loadingDone:
    //   1. clear cached data (so views don't keep stale state)
    //   2. disable data distribution to views BEFORE the load
    //   3. blocking load
    //   4. re-enable data distribution
    //   5. fetch loaded buffers into our own accessor
    //   6. clearData() once the accessor holds the buffers, so the views never
    //      see this batch (the clearData signal also flushes any stale view
    //      state from before).
    LoadRequest req;
    req.dbcontents_.insert(kReferenceDBContent);
    req.dbcontents_.insert(test_dbcontents.begin(), test_dbcontents.end());
    req.apply_view_filters_ = false;
    req.show_status_        = false;
    // Explicit read set: load every variable the inspectors and the scope
    // filter consume (in particular the ground bit, which the default read set
    // omits). Mirrors EvaluationManager attaching its own read set.
    req.read_set_ = [this](const std::string& name) { return buildReadSet(name); };

    auto& view_man = compass_.viewManager();

    dbcontent_man.clearData();

    // !do not distribute this reload to views!
    view_man.disableDataDistribution(true);

    dbcontent_man.loadBlocking(req);

    // !reenable distribution to views!
    view_man.disableDataDistribution(false);

    auto buffers = dbcontent_man.loadedData();
    if (buffers.empty())
    {
        error_out = "No data loaded for the selected data sources.";
        dbcontent_man.clearData();
        return false;
    }

    accessor_->clear();
    accessor_->add(buffers);

    // Clear local data (mirrors EvaluationManager::loadingDone). This also
    // emits the cleared loadedDataSignal that flushes any view state.
    dbcontent_man.clearData();

    // Always load status-bearing dbcontents (CAT019 today) as a separate
    // single-content request, with the status-specific read set. No source
    // filter -- we want every status source the DB carries. Empty result is
    // OK; consumers (status-message PD) check `statusCycles().empty()`.
    loadStatusCycles();

    buildChains(selected_ds_ids, ref_ds_ids, test_dbcontents);

    if (!hasUsableData())
    {
        error_out = "Loaded buffers contain no associated UTNs with both "
                    "reference and test data.";
        return false;
    }

    return true;
}

dbContent::VariableSet AnalysisDataset::buildReadSet(const std::string& name) const
{
    using namespace dbcontent_vars;

    auto& m = compass_.dbContentManager();
    dbContent::VariableSet rs;

    auto addMeta = [&](const auto& mv) {
        if (m.metaCanGetVariable(name, mv))
            rs.add(m.metaGetVariable(name, mv));
    };
    auto addVar = [&](const auto& v) {
        if (m.canGetVariable(name, v))
            rs.add(m.getVariable(name, v));
    };

    // identity / time / position (mirrors EvaluationManager::addVariables; the
    // Chain finalize step requires Mode-3A / Mode-C, so the set must be complete)
    addMeta(meta_var_rec_num_);
    addMeta(meta_var_ds_id_);
    addMeta(meta_var_line_id_);
    addMeta(meta_var_utn_);
    addMeta(meta_var_timestamp_);
    addMeta(meta_var_latitude_);
    addMeta(meta_var_longitude_);
    addMeta(meta_var_max_stddev_xy_);
    addMeta(meta_var_acad_);
    addMeta(meta_var_acid_);        // callsign

    // flight level / Mode-C
    addMeta(meta_var_mc_);          // barometric altitude (flight level filter)
    addMeta(meta_var_mc_g_);
    addMeta(meta_var_mc_v_);
    addVar(var_cat062_baro_alt_);
    addVar(var_cat062_fl_measured_);

    // Mode-3A (Chain::updateModeACodes)
    addMeta(meta_var_m3a_);
    addMeta(meta_var_m3a_g_);
    addMeta(meta_var_m3a_v_);

    addMeta(meta_var_track_num_);

    // ground state: CAT0xx ground_bit, RefTraj surface_target
    addMeta(meta_var_ground_bit_);

    // velocity / acceleration / vertical rate / moms
    addMeta(meta_var_ground_speed_);
    addMeta(meta_var_track_angle_);
    addMeta(meta_var_ax_);
    addMeta(meta_var_ay_);
    addMeta(meta_var_rocd_);
    addMeta(meta_var_mom_long_acc_);
    addMeta(meta_var_mom_trans_acc_);
    addMeta(meta_var_mom_vert_rate_);
    addMeta(meta_var_track_coasting_);

    // reported position accuracy (XY std-dev path: CAT020 / CAT010 / RefTraj)
    addMeta(meta_var_x_stddev_);
    addMeta(meta_var_y_stddev_);
    addMeta(meta_var_xy_cov_);

    // ADS-B quality indicators (CAT021 accuracy path + qi_key grouping)
    addVar(var_cat021_mops_version_);
    addVar(var_cat021_nacp_);
    addVar(var_cat021_nucp_nic_);

    // contributing receivers (CAT020 RU inspectors)
    addVar(var_cat020_contrib_recv_);

    return rs;
}

bool AnalysisDataset::targetGroundOnly(unsigned int utn) const
{
    auto it = ground_only_cache_.find(utn);
    if (it != ground_only_cache_.end())
        return it->second;

    bool ground_only = false;
    auto& dbcont_man = compass_.dbContentManager();
    if (dbcont_man.existsTarget(utn))
    {
        auto cat = dbcont_man.emitterCategory(utn);
        ground_only = (cat != TargetBase::Category::Unknown) && TargetBase::isGroundOnly(cat);
    }
    ground_only_cache_[utn] = ground_only;
    return ground_only;
}

void AnalysisDataset::buildChains(const std::set<unsigned int>& selected_ds_ids,
                                  const std::set<unsigned int>& ref_ds_ids,
                                  const std::set<std::string>& test_dbcontents)
{
    using namespace dbcontent_vars;

    bool first_position = true;

    const ScopeFilter& sf = scope_filter_;

    // Per-report inside test (mirrors EvaluationTargetData::computeSectorInsideInfo):
    // a report is in scope when its ground state and barometric flight level pass
    // the configured constraints. `gb_vec` / `mc_vec` are the (optional) ground-bit
    // and Mode-C vectors for the dbcontent; pass nullptr when absent.
    auto inScope = [&sf](const NullableVector<bool>* gb_vec,
                         const NullableVector<float>* mc_vec,
                         unsigned int idx,
                         bool target_ground_only) -> bool
    {
        if (!sf.active())
            return true;

        bool on_ground = target_ground_only;
        if (!on_ground && gb_vec && !gb_vec->isNull(idx))
            on_ground = gb_vec->get(idx);

        if (sf.ground_only && !on_ground)
            return false;

        // Altitude band in flight levels (FL = feet / 100). When the altitude is
        // absent the FL check is skipped, as in Sector::isInside.
        if ((sf.use_min_fl || sf.use_max_fl) && mc_vec && !mc_vec->isNull(idx))
        {
            double fl = static_cast<double>(mc_vec->get(idx)) / 100.0;
            if (sf.use_min_fl && fl < sf.min_fl)
                return false;
            if (sf.use_max_fl && fl > sf.max_fl)
                return false;
        }
        return true;
    };

    // Reference (RefTraj): take rows from the configured ref DS that are
    // associated to a UTN.
    if (accessor_->has(kReferenceDBContent)
        && accessor_->hasMetaVar<ptime>(kReferenceDBContent, meta_var_timestamp_)
        && accessor_->hasMetaVar<unsigned int>(kReferenceDBContent, meta_var_utn_)
        && accessor_->hasMetaVar<unsigned int>(kReferenceDBContent, meta_var_ds_id_))
    {
        auto& ts_vec    = accessor_->getMetaVar<ptime>(kReferenceDBContent, meta_var_timestamp_);
        auto& utn_vec   = accessor_->getMetaVar<unsigned int>(kReferenceDBContent, meta_var_utn_);
        auto& ds_id_vec = accessor_->getMetaVar<unsigned int>(kReferenceDBContent, meta_var_ds_id_);

        const bool has_lat = accessor_->hasMetaVar<double>(kReferenceDBContent, meta_var_latitude_);
        const bool has_lon = accessor_->hasMetaVar<double>(kReferenceDBContent, meta_var_longitude_);
        const bool has_mc  = accessor_->hasMetaVar<float>(kReferenceDBContent, meta_var_mc_);

        NullableVector<float>* ref_mc_vec = has_mc
            ? &accessor_->getMetaVar<float>(kReferenceDBContent, meta_var_mc_) : nullptr;
        NullableVector<bool>* ref_gb_vec =
            accessor_->hasMetaVar<bool>(kReferenceDBContent, meta_var_ground_bit_)
            ? &accessor_->getMetaVar<bool>(kReferenceDBContent, meta_var_ground_bit_) : nullptr;

        unsigned int n = ts_vec.contentSize();
        for (unsigned int i = 0; i < n; ++i)
        {
            if (ts_vec.isNull(i) || utn_vec.isNull(i) || ds_id_vec.isNull(i))
                continue;

            if (!ref_ds_ids.count(ds_id_vec.get(i)))
                continue;

            unsigned int utn = utn_vec.get(i);
            ptime ts         = ts_vec.get(i);

            if (!inScope(ref_gb_vec, ref_mc_vec, i, targetGroundOnly(utn)))
                continue;

            addToReferenceChain(utn, ts, i);
            ++num_ref_records_total_;

            if (has_lat && has_lon)
            {
                auto& lat_vec = accessor_->getMetaVar<double>(kReferenceDBContent, meta_var_latitude_);
                auto& lon_vec = accessor_->getMetaVar<double>(kReferenceDBContent, meta_var_longitude_);
                if (!lat_vec.isNull(i) && !lon_vec.isNull(i))
                {
                    double lat = lat_vec.get(i);
                    double lon = lon_vec.get(i);
                    if (first_position)
                    {
                        min_lat_ = max_lat_ = lat;
                        min_lon_ = max_lon_ = lon;
                        first_position = false;
                        has_position_extent_ = true;
                    }
                    else
                    {
                        if (lat < min_lat_) min_lat_ = lat;
                        if (lat > max_lat_) max_lat_ = lat;
                        if (lon < min_lon_) min_lon_ = lon;
                        if (lon > max_lon_) max_lon_ = lon;
                    }
                }
            }

            if (has_mc)
            {
                auto& mc_vec = accessor_->getMetaVar<float>(kReferenceDBContent, meta_var_mc_);
                if (!mc_vec.isNull(i))
                {
                    double alt = static_cast<double>(mc_vec.get(i));
                    if (!has_altitude_extent_)
                    {
                        min_alt_ft_ = max_alt_ft_ = alt;
                        has_altitude_extent_ = true;
                    }
                    else
                    {
                        if (alt < min_alt_ft_) min_alt_ft_ = alt;
                        if (alt > max_alt_ft_) max_alt_ft_ = alt;
                    }
                }
            }
        }
    }
    else
    {
        logwrn << "RefTraj buffer or required meta variables not present";
    }

    // Test (CAT020 / CAT010 / ...): keep only rows from selected DS IDs and that
    // are associated to a UTN.
    for (const auto& dbcontent_name : test_dbcontents)
    {
        if (!accessor_->has(dbcontent_name))
            continue;

        if (!(accessor_->hasMetaVar<ptime>(dbcontent_name, meta_var_timestamp_)
              && accessor_->hasMetaVar<unsigned int>(dbcontent_name, meta_var_utn_)
              && accessor_->hasMetaVar<unsigned int>(dbcontent_name, meta_var_ds_id_)))
        {
            logwrn << "test dbcontent " << dbcontent_name
                   << " missing required meta variables; skipping";
            continue;
        }

        auto& ts_vec    = accessor_->getMetaVar<ptime>(dbcontent_name, meta_var_timestamp_);
        auto& utn_vec   = accessor_->getMetaVar<unsigned int>(dbcontent_name, meta_var_utn_);
        auto& ds_id_vec = accessor_->getMetaVar<unsigned int>(dbcontent_name, meta_var_ds_id_);

        NullableVector<float>* tst_mc_vec =
            accessor_->hasMetaVar<float>(dbcontent_name, meta_var_mc_)
            ? &accessor_->getMetaVar<float>(dbcontent_name, meta_var_mc_) : nullptr;
        NullableVector<bool>* tst_gb_vec =
            accessor_->hasMetaVar<bool>(dbcontent_name, meta_var_ground_bit_)
            ? &accessor_->getMetaVar<bool>(dbcontent_name, meta_var_ground_bit_) : nullptr;

        unsigned int n = ts_vec.contentSize();
        bool any_added = false;
        for (unsigned int i = 0; i < n; ++i)
        {
            if (ts_vec.isNull(i) || utn_vec.isNull(i) || ds_id_vec.isNull(i))
                continue;

            unsigned int ds_id = ds_id_vec.get(i);
            if (!selected_ds_ids.count(ds_id))
                continue;

            unsigned int utn = utn_vec.get(i);
            ptime ts         = ts_vec.get(i);

            if (!inScope(tst_gb_vec, tst_mc_vec, i, targetGroundOnly(utn)))
                continue;

            addToTestChain(utn, dbcontent_name, ts, i);
            ++num_tst_records_total_;
            any_added = true;
        }

        if (any_added)
            tst_dbcontents_present_.insert(dbcontent_name);
    }

    // Finalize chains and collect UTNs that have at least one of (ref, test).
    for (auto& kv : ref_chains_)
    {
        kv.second->finalize();
        utns_.insert(kv.first);
    }
    for (auto& kv : tst_chains_)
    {
        kv.second->finalize();
        utns_.insert(kv.first.first);
    }

    if (has_position_extent_)
        center_lat_deg_ = 0.5 * (min_lat_ + max_lat_);
}

void AnalysisDataset::addToReferenceChain(unsigned int utn, ptime ts, unsigned int idx)
{
    auto it = ref_chains_.find(utn);
    if (it == ref_chains_.end())
    {
        auto chain = std::make_unique<dbContent::TargetReport::Chain>(accessor_, kReferenceDBContent);
        chain->addIndex(ts, idx);
        ref_chains_.emplace(utn, std::move(chain));
    }
    else
    {
        it->second->addIndex(ts, idx);
    }
}

void AnalysisDataset::addToTestChain(unsigned int utn, const std::string& dbcontent,
                                     ptime ts, unsigned int idx)
{
    auto key = std::make_pair(utn, dbcontent);
    auto it = tst_chains_.find(key);
    if (it == tst_chains_.end())
    {
        auto chain = std::make_unique<dbContent::TargetReport::Chain>(accessor_, dbcontent);
        chain->addIndex(ts, idx);
        tst_chains_.emplace(std::move(key), std::move(chain));
    }
    else
    {
        it->second->addIndex(ts, idx);
    }
}

bool AnalysisDataset::hasUsableData() const
{
    for (const auto& kv : tst_chains_)
    {
        if (ref_chains_.count(kv.first.first))
            return true;
    }
    return false;
}

bool AnalysisDataset::hasReferenceChain(unsigned int utn) const
{
    return ref_chains_.count(utn) > 0;
}

bool AnalysisDataset::hasTestChain(unsigned int utn, const std::string& dbcontent_name) const
{
    return tst_chains_.count({utn, dbcontent_name}) > 0;
}

dbContent::TargetReport::Chain& AnalysisDataset::referenceChain(unsigned int utn)
{
    return *ref_chains_.at(utn);
}

dbContent::TargetReport::Chain& AnalysisDataset::testChain(unsigned int utn,
                                                            const std::string& dbcontent_name)
{
    return *tst_chains_.at({utn, dbcontent_name});
}

unsigned int AnalysisDataset::numTestChains() const
{
    return static_cast<unsigned int>(tst_chains_.size());
}

boost::optional<dbContent::TargetPosition>
AnalysisDataset::mappedRefPos(unsigned int utn, ptime timestamp, time_duration d_max) const
{
    auto it = ref_chains_.find(utn);
    if (it == ref_chains_.end())
        return boost::none;

    auto mapping = it->second->calculateDataMapping(timestamp);

    if (!mapping.has_ref_pos_)
        return boost::none;

    auto absDur = [](time_duration d) {
        return d.is_negative() ? -d : d;
    };

    if (mapping.has_ref1_ && mapping.has_ref2_)
    {
        auto lo = absDur(timestamp - mapping.timestamp_ref1_);
        auto hi = absDur(mapping.timestamp_ref2_ - timestamp);
        if (lo > d_max || hi > d_max)
            return boost::none;
    }
    else if (mapping.has_ref1_)
    {
        if (absDur(timestamp - mapping.timestamp_ref1_) > d_max)
            return boost::none;
    }
    else if (mapping.has_ref2_)
    {
        if (absDur(mapping.timestamp_ref2_ - timestamp) > d_max)
            return boost::none;
    }
    else
    {
        return boost::none;
    }

    return mapping.pos_ref_;
}
