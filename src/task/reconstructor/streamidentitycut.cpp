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
 *
 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#include "streamidentitycut.h"
#include "streamidentitydetector.h"
#include "reconstructorbase.h"
#include "reconstructortarget.h"
#include "targetreportdefs.h"
#include "logger.h"
#include "stringconv.h"
#include "timeconv.h"

#include <set>
#include <tuple>
#include <vector>

using namespace std;
using namespace Utils;

namespace
{

/**
 * Finds the targets holding reports of the given tracker stream at or after ts.
 * Normally exactly one; a stream can be spread over several targets after earlier
 * association decisions.
 */
vector<unsigned int> targetsHoldingStream(const ReconstructorBase& reconstructor,
                                          unsigned int ds_id, unsigned int line_id,
                                          unsigned int track_number,
                                          const boost::posix_time::ptime& ts)
{
    vector<unsigned int> utns;

    for (const auto& target_it : reconstructor.targets_container_.targets_)
    {
        const dbContent::ReconstructorTarget& target = target_it.second;

        bool found {false};

        for (const auto& dbcont_it : target.tr_ds_timestamps_)
        {
            if (dbcont_it.first != 62 && dbcont_it.first != 255)
                continue;

            if (!dbcont_it.second.count(ds_id))
                continue;

            const auto& ds_map = dbcont_it.second.at(ds_id);

            if (!ds_map.count(line_id))
                continue;

            const auto& line_map = ds_map.at(line_id);

            for (auto ts_it = line_map.lower_bound(ts); ts_it != line_map.end(); ++ts_it)
            {
                if (!reconstructor.target_reports_.count(ts_it->second))
                    continue;

                const dbContent::targetReport::ReconstructorInfo& tr =
                    reconstructor.target_reports_.at(ts_it->second);

                if (tr.track_number_ && *tr.track_number_ == track_number)
                {
                    found = true;
                    break;
                }
            }

            if (found)
                break;
        }

        if (found)
            utns.push_back(target_it.first);
    }

    return utns;
}

/**
 * Mode S addresses carried by one side of a stream inside a target.
 */
set<unsigned int> streamACADs(const ReconstructorBase& reconstructor, unsigned int utn,
                              unsigned int ds_id, unsigned int line_id,
                              unsigned int track_number,
                              const boost::posix_time::ptime& ts, bool at_or_after)
{
    set<unsigned int> acads;

    const dbContent::ReconstructorTarget& target =
        reconstructor.targets_container_.targets_.at(utn);

    for (auto rec_num : target.target_reports_)
    {
        if (!reconstructor.target_reports_.count(rec_num))
            continue;

        const dbContent::targetReport::ReconstructorInfo& tr =
            reconstructor.target_reports_.at(rec_num);

        if (tr.ds_id_ != ds_id || tr.line_id_ != line_id
            || !tr.track_number_ || *tr.track_number_ != track_number)
            continue;

        if (at_or_after ? tr.timestamp_ < ts : tr.timestamp_ >= ts)
            continue;

        if (tr.acad_)
            acads.insert(*tr.acad_);
    }

    return acads;
}

} // namespace

unsigned int StreamIdentityCut::cutDetectedTransitions(ReconstructorBase& reconstructor)
{
    StreamIdentityDetector::Settings settings =
        StreamIdentityDetector::resolveSettings(reconstructor);

    vector<StreamIdentityDetector::Transition> transitions =
        StreamIdentityDetector::analyze(reconstructor, settings);

    unsigned int num_cuts {0};

    reconstructor.targets_container_.do_not_merge_pairs_.clear();

    // a stream is cut at most once per invocation: later transitions belong to the
    // separated remainder, cutting them again only over-segments it
    set<tuple<unsigned int, unsigned int, unsigned int>> streams_cut;

    for (const auto& transition : transitions)
    {
        if (!transition.accepted())
        {
            logdbg << "IdentityTransition: " << transition.asStr();
            continue;
        }

        loginf << "IdentityTransition: " << transition.asStr();

        auto stream_key = make_tuple(transition.ds_id_, transition.line_id_,
                                     transition.track_number_);

        if (streams_cut.count(stream_key))
        {
            loginf << "stream already cut in this slice, not cutting again";
            continue;
        }

        vector<unsigned int> utns = targetsHoldingStream(
            reconstructor, transition.ds_id_, transition.line_id_,
            transition.track_number_, transition.transition_time_);

        if (utns.empty())
        {
            loginf << "no target holds ds " << transition.ds_id_
                   << " line " << transition.line_id_
                   << " tn " << transition.track_number_ << " after the transition, not cutting";
            continue;
        }

        for (auto utn : utns)
        {
            traced_assert(reconstructor.targets_container_.targets_.count(utn));

            auto& container = reconstructor.targets_container_;

            // Which side of the transition is foreign to this target? Normally the
            // continuation after it, but if the target's own aircraft is the one that
            // captured the track number, the part BEFORE the transition is the intruder.
            set<unsigned int> acads_after = streamACADs(
                reconstructor, utn, transition.ds_id_, transition.line_id_,
                transition.track_number_, transition.transition_time_, true);

            bool cut_after {true};

            for (auto acad : acads_after)
            {
                if (container.targets_.at(utn).hasACAD(acad))
                    cut_after = false; // the later part is this target's own aircraft
            }

            vector<unsigned long> moved =
                container.targets_.at(utn).removeStreamReports(
                    transition.ds_id_, transition.line_id_, transition.track_number_,
                    transition.transition_time_, cut_after);

            if (!moved.size())
            {
                loginf << "utn " << utn << " holds no reports of the "
                       << (cut_after ? "later" : "earlier") << " part, nothing to cut";
                continue;
            }

            // Mode S addresses carried by the separated reports decide where they go:
            // a target must never hold an address another target already owns
            set<unsigned int> moved_acads;

            for (auto rec_num : moved)
            {
                const dbContent::targetReport::ReconstructorInfo& tr =
                    reconstructor.target_reports_.at(rec_num);

                if (tr.acad_)
                    moved_acads.insert(*tr.acad_);
            }

            bool revert = !container.targets_.at(utn).target_reports_.size()
                          || moved_acads.size() > 1;

            for (auto acad : moved_acads)
                if (container.targets_.at(utn).hasACAD(acad))
                    revert = true; // same aircraft on both sides, not a swap after all

            if (revert)
            {
                for (auto rec_num : moved)
                    container.targets_.at(utn).addTargetReport(rec_num);

                loginf << "cut of utn " << utn << " reverted (source emptied or"
                       << " inconsistent addresses in the separated reports)";
                continue;
            }

            // if another target already owns the address, the reports belong there
            int host_utn {-1};

            for (auto acad : moved_acads)
            {
                if (container.acad_2_utn_.count(acad) && container.acad_2_utn_.at(acad) != utn)
                    host_utn = (int) container.acad_2_utn_.at(acad);
            }

            ++num_cuts;
            streams_cut.insert(stream_key);

            if (host_utn >= 0)
            {
                traced_assert(container.targets_.count(host_utn));

                container.do_not_merge_pairs_.insert(
                    {min(utn, (unsigned int) host_utn), max(utn, (unsigned int) host_utn)});

                for (auto rec_num : moved)
                    container.targets_.at(host_utn).addTargetReport(rec_num);

                if (cut_after) // the stream's future belongs to the host now
                    container.tn2utn_[transition.ds_id_][transition.line_id_][transition.track_number_] =
                        std::pair<unsigned int, boost::posix_time::ptime>(
                            host_utn, reconstructor.target_reports_.at(moved.back()).timestamp_);

                loginf << "cut utn " << utn << (cut_after ? " (later part)" : " (earlier part)")
                       << " -> existing utn " << host_utn
                       << " (owns the address), moved " << moved.size()
                       << " report(s) of ds " << transition.ds_id_
                       << " line " << transition.line_id_ << " tn " << transition.track_number_
                       << " from " << Time::toString(transition.transition_time_);
            }
            else
            {
                unsigned int new_utn = container.createTargetFromReports(moved, utn);

                container.do_not_merge_pairs_.insert(
                    {min(utn, new_utn), max(utn, new_utn)});

                loginf << "cut utn " << utn << (cut_after ? " (later part)" : " (earlier part)")
                       << " -> new utn " << new_utn
                       << ", moved " << moved.size() << " report(s) of ds " << transition.ds_id_
                       << " line " << transition.line_id_ << " tn " << transition.track_number_
                       << " from " << Time::toString(transition.transition_time_);
            }
        }
    }

    return num_cuts;
}
