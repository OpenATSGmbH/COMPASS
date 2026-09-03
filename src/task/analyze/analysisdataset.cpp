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
#include "dbcontent/dbcontentdataengine.h"
#include "dbcontent/loadoperation.h"
#include "dbcontent/dbcontentstatusinfo.h"
#include "dbcontent/target/targetbase.h"
#include "dbcontent/variable/variableset.h"
#include "sectorlayer.h"
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

// CAT010 I010/000 message types
constexpr unsigned char kCAT010MsgTargetReport       = 1;
constexpr unsigned char kCAT010MsgStartOfUpdateCycle = 2;

// CAT010 I010/020 TYP: 3 = PSR (SMR)
constexpr unsigned char kCAT010DetectionTypePSR = 3;

const std::vector<unsigned int>& emptyIndices()
{
    static const std::vector<unsigned int> empty;
    return empty;
}

const std::vector<ptime>& emptyCycles()
{
    static const std::vector<ptime> empty;
    return empty;
}
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

    // isolated batch load: fill our own operation and read its buffers - the
    // view dataset is never touched (no clearData / distribution toggling)
    auto op = std::make_shared<LoadOperation>(dbcontent_man, req);
    dbcontent_man.dataEngine().load(op);
    op->wait();

    // @TODO: op state unchecked - a Failed/Cancelled load analyses partial data
    auto status_buffers = op->buffers();
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

    // Isolated batch load: build the operation, fill it via the engine, and read
    // its buffers into our own accessor. The view dataset is never touched.
    LoadRequest req;
    req.dbcontents_.insert(kReferenceDBContent);
    req.dbcontents_.insert(test_dbcontents.begin(), test_dbcontents.end());
    req.apply_view_filters_ = false;
    // only the needed data sources - per-op, so the user's selection stays untouched
    req.datasrc_selection_ = ds_load_map;
    // Explicit read set: load every variable the inspectors and the scope
    // filter consume (in particular the ground bit, which the default read set
    // omits). Mirrors EvaluationManager attaching its own read set.
    req.read_set_ = [this](const std::string& name) { return buildReadSet(name); };

    auto op = std::make_shared<LoadOperation>(dbcontent_man, req);
    dbcontent_man.dataEngine().load(op);
    op->wait();

    // @TODO: op state unchecked - a Failed/Cancelled load analyses partial data
    auto buffers = op->buffers();
    if (buffers.empty())
    {
        error_out = "No data loaded for the selected data sources.";
        return false;
    }

    accessor_->clear();
    accessor_->add(buffers);

    // Always load status-bearing dbcontents (CAT019 today) as a separate
    // single-content request, with the status-specific read set. No source
    // filter -- we want every status source the DB carries. Empty result is
    // OK; consumers (status-message PD) check `statusCycles().empty()`.
    loadStatusCycles();

    buildChains(selected_ds_ids, ref_ds_ids, test_dbcontents);

    if (!hasUsableData())
    {
        error_out = require_reference_
            ? "Loaded buffers contain no associated UTNs with both "
              "reference and test data."
            : "Loaded buffers contain no test data for the selected data sources.";
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
    // Variable names are not unique over the dbcontents, and the data type can
    // differ: "Target Length" is FLOAT in CAT010 but UCHAR in CAT062 and
    // RefTraj. getVariable() asserts on a type mismatch, so the type is checked
    // before the variable is added.
    auto addVar = [&](const auto& v) {
        if (!m.canGetVariable(name, v))
            return;
        if (m.dbContent(name).variable(v.name()).dataType() != v.dataType())
            return;
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

    // CAT010 SMR: message type (scan cycles), polar position, descriptor
    // flags, target size, plot amplitude
    addMeta(meta_var_message_type_);
    addVar(var_radar_range_);
    addVar(var_radar_azimuth_);
    addVar(var_cat010_detection_type_);
    addVar(var_cat010_slant_range_corrected_);
    addVar(var_cat010_target_length_);
    addVar(var_cat010_target_width_);
    addVar(var_cat010_target_orientation_);
    addVar(var_cat010_psr_amplitude_);

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
    // a report is in scope when its ground state, barometric flight level and
    // sector membership pass the configured constraints. `lat/lon/mc/gb` are the
    // (optional) position, Mode-C and ground-bit vectors; pass nullptr when absent.
    // `forced_on_ground` marks reports known to be on ground without a ground
    // bit (ground-only reconstructed target, or ground-only data source).
    auto inScope = [&sf](const NullableVector<double>* lat_vec,
                         const NullableVector<double>* lon_vec,
                         const NullableVector<float>* mc_vec,
                         const NullableVector<bool>* gb_vec,
                         unsigned int idx,
                         bool forced_on_ground) -> bool
    {
        if (!sf.active())
            return true;

        boost::optional<bool> ground_bit;
        if (gb_vec && !gb_vec->isNull(idx))
            ground_bit = gb_vec->get(idx);

        bool on_ground = forced_on_ground || (ground_bit.has_value() && ground_bit.value());

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

        // Sector membership: keep only reports inside any selected sector layer,
        // using the evaluation's exact inside test
        // (SectorLayer::isInside(pos, has_gb, gb_set)).
        if (sf.limit_by_sectors && !sf.sectors.empty())
        {
            if (!lat_vec || !lon_vec || lat_vec->isNull(idx) || lon_vec->isNull(idx))
                return false;

            dbContent::TargetPosition pos;
            pos.latitude_  = lat_vec->get(idx);
            pos.longitude_ = lon_vec->get(idx);
            if (mc_vec && !mc_vec->isNull(idx))
            {
                pos.has_altitude_ = true;
                pos.altitude_     = mc_vec->get(idx);
            }

            // a forced on-ground report is presented like a set ground bit
            const bool has_gb = ground_bit.has_value() || forced_on_ground;
            const bool gb_set = on_ground;

            bool inside = false;
            for (const auto& sl : sf.sectors)
            {
                if (sl && sl->isInside(pos, has_gb, gb_set))
                {
                    inside = true;
                    break;
                }
            }
            if (!inside)
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

        NullableVector<double>* ref_lat_vec = has_lat
            ? &accessor_->getMetaVar<double>(kReferenceDBContent, meta_var_latitude_) : nullptr;
        NullableVector<double>* ref_lon_vec = has_lon
            ? &accessor_->getMetaVar<double>(kReferenceDBContent, meta_var_longitude_) : nullptr;
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

            if (!inScope(ref_lat_vec, ref_lon_vec, ref_mc_vec, ref_gb_vec, i, targetGroundOnly(utn)))
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

    // Test (CAT020 / CAT010 / ...): keep rows from selected DS IDs. Rows with a
    // UTN go into per-UTN chains, rows without a UTN into the unassociated
    // pool. CAT010 status messages are not target reports: type 002 (Start of
    // Update Cycle) rows become per-source scan cycles, the others are dropped.
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

        NullableVector<double>* tst_lat_vec =
            accessor_->hasMetaVar<double>(dbcontent_name, meta_var_latitude_)
            ? &accessor_->getMetaVar<double>(dbcontent_name, meta_var_latitude_) : nullptr;
        NullableVector<double>* tst_lon_vec =
            accessor_->hasMetaVar<double>(dbcontent_name, meta_var_longitude_)
            ? &accessor_->getMetaVar<double>(dbcontent_name, meta_var_longitude_) : nullptr;
        NullableVector<float>* tst_mc_vec =
            accessor_->hasMetaVar<float>(dbcontent_name, meta_var_mc_)
            ? &accessor_->getMetaVar<float>(dbcontent_name, meta_var_mc_) : nullptr;
        NullableVector<bool>* tst_gb_vec =
            accessor_->hasMetaVar<bool>(dbcontent_name, meta_var_ground_bit_)
            ? &accessor_->getMetaVar<bool>(dbcontent_name, meta_var_ground_bit_) : nullptr;

        const bool is_cat010 = (dbcontent_name == "CAT010");

        NullableVector<unsigned char>* msg_type_vec =
            is_cat010 && accessor_->hasMetaVar<unsigned char>(dbcontent_name, meta_var_message_type_)
            ? &accessor_->getMetaVar<unsigned char>(dbcontent_name, meta_var_message_type_) : nullptr;

        NullableVector<unsigned char>* det_type_vec =
            is_cat010 && sf.smr_only
            && accessor_->hasVar<unsigned char>(dbcontent_name, var_cat010_detection_type_)
            ? &accessor_->getVar<unsigned char>(dbcontent_name, var_cat010_detection_type_) : nullptr;

        if (is_cat010 && sf.smr_only && !det_type_vec)
            logwrn << "CAT010 detection type not loaded, SMR-only filter not applied";

        unsigned int n = ts_vec.contentSize();
        bool any_added = false;
        for (unsigned int i = 0; i < n; ++i)
        {
            if (ts_vec.isNull(i) || ds_id_vec.isNull(i))
                continue;

            unsigned int ds_id = ds_id_vec.get(i);
            if (!selected_ds_ids.count(ds_id))
                continue;

            ptime ts = ts_vec.get(i);

            if (msg_type_vec && !msg_type_vec->isNull(i))
            {
                const unsigned char msg_type = msg_type_vec->get(i);
                if (msg_type == kCAT010MsgStartOfUpdateCycle)
                {
                    status_cycles_by_ds_[ds_id].push_back(ts);
                    continue;
                }
                if (msg_type != kCAT010MsgTargetReport)
                    continue;
            }

            if (det_type_vec
                && (det_type_vec->isNull(i) || det_type_vec->get(i) != kCAT010DetectionTypePSR))
            {
                ++num_non_psr_skipped_;
                continue;
            }

            const bool ds_ground_only = sf.ground_only_ds_ids.count(ds_id) > 0;

            if (utn_vec.isNull(i))
            {
                if (!inScope(tst_lat_vec, tst_lon_vec, tst_mc_vec, tst_gb_vec, i, ds_ground_only))
                    continue;

                unassociated_idx_[dbcontent_name].push_back(i);
                ++num_unassoc_records_total_;
                continue;
            }

            unsigned int utn = utn_vec.get(i);

            if (!inScope(tst_lat_vec, tst_lon_vec, tst_mc_vec, tst_gb_vec, i,
                         ds_ground_only || targetGroundOnly(utn)))
                continue;

            addToTestChain(utn, dbcontent_name, ts, i);
            ++num_tst_records_total_;
            any_added = true;
        }

        if (any_added)
            tst_dbcontents_present_.insert(dbcontent_name);
    }

    for (auto& kv : status_cycles_by_ds_)
    {
        auto& cycles = kv.second;
        std::sort(cycles.begin(), cycles.end());
        cycles.erase(std::unique(cycles.begin(), cycles.end()), cycles.end());
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

    loginf << "built " << ref_chains_.size() << " reference chains, "
           << tst_chains_.size() << " test chains, "
           << num_unassoc_records_total_ << " unassociated test records, "
           << num_non_psr_skipped_ << " non-PSR CAT010 records skipped, "
           << status_cycles_by_ds_.size() << " sources with scan cycles";
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
    if (!require_reference_)
        return !tst_chains_.empty() || num_unassoc_records_total_ > 0;

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

const std::vector<unsigned int>&
AnalysisDataset::unassociatedIndices(const std::string& dbcontent_name) const
{
    auto it = unassociated_idx_.find(dbcontent_name);
    if (it == unassociated_idx_.end())
        return emptyIndices();
    return it->second;
}

bool AnalysisDataset::hasStatusCycles(unsigned int ds_id) const
{
    auto it = status_cycles_by_ds_.find(ds_id);
    return it != status_cycles_by_ds_.end() && !it->second.empty();
}

const std::vector<ptime>& AnalysisDataset::statusCycles(unsigned int ds_id) const
{
    auto it = status_cycles_by_ds_.find(ds_id);
    if (it == status_cycles_by_ds_.end())
        return emptyCycles();
    return it->second;
}

boost::optional<dbContent::TargetReport::DataMapping>
AnalysisDataset::refMappingWithin(unsigned int utn, ptime timestamp, time_duration d_max) const
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

    return mapping;
}

boost::optional<dbContent::TargetPosition>
AnalysisDataset::mappedRefPos(unsigned int utn, ptime timestamp, time_duration d_max,
                              boost::optional<dbContent::TargetPositionAccuracy>* ref_pos_acc_out) const
{
    if (ref_pos_acc_out)
        *ref_pos_acc_out = boost::none;

    auto mapping = refMappingWithin(utn, timestamp, d_max);
    if (!mapping)
        return boost::none;

    if (ref_pos_acc_out)
    {
        const auto& chain = *ref_chains_.at(utn);

        // worse (larger std-dev) of the bracketing reference updates, like
        // ReconstructorTarget::interpolatedRefPosForTime
        boost::optional<dbContent::TargetPositionAccuracy> acc1, acc2;

        if (mapping->has_ref1_)
            acc1 = chain.posAccuracy(mapping->dataid_ref1_);
        if (mapping->has_ref2_)
            acc2 = chain.posAccuracy(mapping->dataid_ref2_);

        if (acc1 && acc2)
            *ref_pos_acc_out = acc1->max() > acc2->max() ? acc1 : acc2;
        else if (acc1)
            *ref_pos_acc_out = acc1;
        else if (acc2)
            *ref_pos_acc_out = acc2;
    }

    return mapping->pos_ref_;
}

boost::optional<float>
AnalysisDataset::mappedRefTrackAngle(unsigned int utn, ptime timestamp, time_duration d_max) const
{
    auto mapping = refMappingWithin(utn, timestamp, d_max);
    if (!mapping)
        return boost::none;

    const auto& chain = *ref_chains_.at(utn);

    if (mapping->has_ref1_)
    {
        auto angle = chain.trackAngle(mapping->dataid_ref1_);
        if (angle)
            return angle;
    }
    if (mapping->has_ref2_)
    {
        auto angle = chain.trackAngle(mapping->dataid_ref2_);
        if (angle)
            return angle;
    }
    return boost::none;
}
