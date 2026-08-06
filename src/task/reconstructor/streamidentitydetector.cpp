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

#include "streamidentitydetector.h"
#include "reconstructorbase.h"
#include "targetreportdefs.h"
#include "logger.h"
#include "stringconv.h"
#include "timeconv.h"

#include <boost/optional.hpp>

#include <algorithm>
#include <map>
#include <sstream>
#include <tuple>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace
{

struct StreamKey
{
    unsigned int ds_id_ {0};
    unsigned int line_id_ {0};
    unsigned int track_number_ {0};

    bool operator< (const StreamKey& other) const
    {
        return tie(ds_id_, line_id_, track_number_)
               < tie(other.ds_id_, other.line_id_, other.track_number_);
    }
};

struct ReportInfo
{
    ptime timestamp_;
    ptime meas_time_; // mode 3/A measurement time, timestamp minus reported age
    boost::optional<unsigned int> code_;
    bool has_acad_ {false};
    string acid_;
    bool in_current_slice_ {false};
};

struct StreamData
{
    vector<ReportInfo> reports_; // sorted by timestamp after collection
};

/**
 * One sustained or candidate identity of a stream, counted in distinct measurements.
 */
struct CodeState
{
    boost::optional<unsigned int> code_;
    unsigned int count_ {0};
    ptime first_, last_;
    ptime last_meas_; // measurement dedup, aged repetitions do not add evidence
    ptime first_report_ts_;
    bool acad_ {false};
    set<string> acids_;
    bool in_current_slice_ {false};

    void start(unsigned int code, const ReportInfo& info)
    {
        code_ = code;
        count_ = 1;
        first_ = info.meas_time_;
        last_ = info.meas_time_;
        last_meas_ = info.meas_time_;
        first_report_ts_ = info.timestamp_;
        acad_ = false;
        acids_.clear();
        in_current_slice_ = info.in_current_slice_;
    }

    void observe(const ReportInfo& info)
    {
        if (info.meas_time_ != last_meas_) // new measurement, not an aged repetition
        {
            ++count_;
            last_meas_ = info.meas_time_;
        }

        last_ = info.meas_time_;
        in_current_slice_ |= info.in_current_slice_;
    }

    void reset()
    {
        code_.reset();
        count_ = 0;
        acad_ = false;
        acids_.clear();
        in_current_slice_ = false;
    }

    bool sustained(unsigned int min_updates, const time_duration& min_duration) const
    {
        return count_ >= min_updates && last_ - first_ >= min_duration;
    }
};

/**
 * True if the old code is freshly measured again on the stream shortly after the
 * transition - then the streams interleave instead of transitioning.
 */
bool oldIdentityReturns(const StreamData& data, unsigned int old_code,
                        const ptime& transition_time, const time_duration& window)
{
    for (const auto& info : data.reports_)
    {
        if (!info.code_ || *info.code_ != old_code)
            continue;

        if (info.meas_time_ <= transition_time)
            continue; // aged repetition of an earlier measurement

        if (info.meas_time_ - transition_time <= window)
            return true;
    }

    return false;
}

/**
 * True if another tracker SYSTEM made the same code change on one of its own tracks -
 * then the real aircraft recoded. Lines of the same system are not independent.
 * Old and new code must appear on the SAME other stream: a second aircraft in a wrongly
 * merged target carries the new code legitimately on a stream of its own.
 */
bool otherSystemMadeSameTransition(const map<StreamKey, StreamData>& streams,
                                   const StreamKey& key, unsigned int old_code,
                                   unsigned int new_code, const ptime& transition_time,
                                   const time_duration& window)
{
    for (const auto& other_it : streams)
    {
        if (other_it.first.ds_id_ == key.ds_id_)
            continue; // same tracker system, not an independent opinion

        bool has_old {false}, has_new {false};

        for (const auto& info : other_it.second.reports_)
        {
            if (!info.code_)
                continue;

            if (info.meas_time_ < transition_time - window
                || info.meas_time_ > transition_time + window)
                continue;

            if (*info.code_ == old_code)
                has_old = true;
            else if (*info.code_ == new_code)
                has_new = true;
        }

        if (has_old && has_new)
            return true;
    }

    return false;
}

/**
 * True if another tracker system observed the old code during the old identity's
 * sustained window (positive old-identity coverage). Without that, swap and recode
 * are indistinguishable - e.g. code changes right after fresh track acquisition,
 * where other systems only ever saw the new code.
 */
bool otherSystemObservedOldIdentity(const map<StreamKey, StreamData>& streams,
                                    const StreamKey& key, unsigned int old_code,
                                    const ptime& old_first, const ptime& transition_time,
                                    const time_duration& margin)
{
    for (const auto& other_it : streams)
    {
        if (other_it.first.ds_id_ == key.ds_id_)
            continue;

        for (const auto& info : other_it.second.reports_)
        {
            if (!info.code_ || *info.code_ != old_code)
                continue;

            if (info.meas_time_ >= old_first - margin
                && info.meas_time_ <= transition_time + margin)
                return true;
        }
    }

    return false;
}

} // namespace

string StreamIdentityDetector::Transition::asStr() const
{
    stringstream ss;

    ss << "utn " << utn_
       << " ds " << ds_id_ << " line " << line_id_ << " tn " << track_number_
       << " at " << Time::toString(transition_time_)
       << " code " << String::octStringFromInt(old_code_, 4, '0')
       << " -> " << String::octStringFromInt(new_code_, 4, '0')
       << " before " << old_count_ << " meas " << Time::partialSeconds(old_duration_)
       << " s acad " << acad_before_ << " acids '" << String::compress(acids_before_, ',') << "'"
       << " after " << new_count_ << " meas " << Time::partialSeconds(new_duration_)
       << " s acad " << acad_after_ << " acids '" << String::compress(acids_after_, ',') << "'";

    if (accepted())
        ss << " SWAP CANDIDATE";
    else
        ss << " rejected: " << reject_reason_;

    return ss.str();
}

StreamIdentityDetector::Settings StreamIdentityDetector::resolveSettings(
    const ReconstructorBase& reconstructor)
{
    const auto& reconstructor_settings = reconstructor.settings();

    Settings settings;

    settings.max_value_age_ = Time::partialSeconds(reconstructor_settings.identity_value_max_age_);
    settings.min_duration_ = Time::partialSeconds(
        reconstructor_settings.identity_transition_min_duration_);
    settings.min_updates_ = reconstructor_settings.identity_transition_min_updates_;

    for (const auto& code_str : String::split(
             reconstructor_settings.identity_conspicuity_codes_, ','))
    {
        string trimmed = String::trim(code_str);

        if (trimmed.empty())
            continue;

        try
        {
            settings.conspicuity_codes_.insert((unsigned int) stoul(trimmed, nullptr, 8));
        }
        catch (const exception& e)
        {
            logwrn << "unparsable conspicuity mode 3/A code '" << trimmed << "'";
        }
    }

    return settings;
}

std::vector<StreamIdentityDetector::Transition> StreamIdentityDetector::analyze(
    const ReconstructorBase& reconstructor, const Settings& settings)
{
    vector<Transition> transitions;

    // global stream registry: all in-memory tracker reports, associated or not, so
    // cross-system evidence is independent of association decisions

    map<StreamKey, StreamData> streams;

    for (const auto& tr_it : reconstructor.target_reports_)
    {
        const dbContent::targetReport::ReconstructorInfo& tr = tr_it.second;

        if (tr.dbcont_id_ != 62 && tr.dbcont_id_ != 255) // tracker track streams only
            continue;

        if (!tr.track_number_)
            continue;

        ReportInfo info;

        info.timestamp_ = tr.timestamp_;
        info.meas_time_ = tr.timestamp_;
        info.in_current_slice_ = tr.in_current_slice_;
        info.has_acad_ = tr.acad_.has_value();

        if (tr.acid_)
            info.acid_ = String::trim(*tr.acid_);

        if (tr.mode_a_code_ && tr.mode_a_code_->hasReliableValue())
        {
            bool age_usable {true};

            if (tr.mode_a_code_age_ && *tr.mode_a_code_age_ > 0)
            {
                time_duration age = Time::partialSeconds(*tr.mode_a_code_age_);

                info.meas_time_ = tr.timestamp_ - age;
                age_usable = age <= settings.max_value_age_;
            }

            if (age_usable)
                info.code_ = tr.mode_a_code_->code_;
        }

        streams[StreamKey{tr.ds_id_, tr.line_id_, *tr.track_number_}].reports_.push_back(info);
    }

    for (auto& stream_it : streams)
        sort(stream_it.second.reports_.begin(), stream_it.second.reports_.end(),
             [] (const ReportInfo& a, const ReportInfo& b) { return a.timestamp_ < b.timestamp_; });

    // detect sustained identity transitions per stream

    for (const auto& stream_it : streams)
    {
        const StreamKey& key = stream_it.first;
        const StreamData& data = stream_it.second;

        CodeState sustained, candidate;

        // advances the identity state with one coded report, appends a classified
        // transition once a new identity is sustained
        auto process_code = [ & ] (const ReportInfo& info)
        {
            unsigned int code = *info.code_;

            if (!sustained.code_)
            {
                sustained.start(code, info);
                return;
            }

            if (code == *sustained.code_)
            {
                sustained.observe(info);
                candidate.reset(); // old identity resumed, candidate was flapping
                return;
            }

            if (!candidate.code_ || code != *candidate.code_)
            {
                candidate.start(code, info);
                return;
            }

            candidate.observe(info);

            if (!candidate.sustained(settings.min_updates_, settings.min_duration_))
                return;

            // report only if confirmed by current-slice data, dedupes re-detection while
            // the transition stays inside the retained window
            if (sustained.sustained(settings.min_updates_, settings.min_duration_)
                && candidate.in_current_slice_)
            {
                Transition transition;

                transition.ds_id_ = key.ds_id_;
                transition.line_id_ = key.line_id_;
                transition.track_number_ = key.track_number_;
                transition.transition_time_ = candidate.first_report_ts_;
                transition.old_code_ = *sustained.code_;
                transition.new_code_ = *candidate.code_;
                transition.old_count_ = sustained.count_;
                transition.new_count_ = candidate.count_;
                transition.old_duration_ = sustained.last_ - sustained.first_;
                transition.new_duration_ = candidate.last_ - candidate.first_;
                transition.acad_before_ = sustained.acad_;
                transition.acad_after_ = candidate.acad_;
                transition.acids_before_ = sustained.acids_;
                transition.acids_after_ = candidate.acids_;

                // informative only: current track number association
                const auto& tn2utn = reconstructor.targets_container_.tn2utn_;

                if (tn2utn.count(key.ds_id_) && tn2utn.at(key.ds_id_).count(key.line_id_)
                    && tn2utn.at(key.ds_id_).at(key.line_id_).count(key.track_number_))
                    transition.utn_ = (int) tn2utn.at(key.ds_id_).at(key.line_id_)
                                          .at(key.track_number_).first;

                // classify: swap candidate or legitimate recode

                if (settings.conspicuity_codes_.count(transition.old_code_)
                    || settings.conspicuity_codes_.count(transition.new_code_))
                {
                    transition.reject_reason_ = "conspicuity code, not target specific";
                }
                else if (transition.acad_before_ && transition.acad_after_)
                {
                    transition.reject_reason_ = "acad present before and after, same aircraft";
                }
                else if (!transition.acids_before_.empty()
                         && transition.acids_before_ == transition.acids_after_)
                {
                    transition.reject_reason_ = "same acid before and after, same aircraft";
                }
                else if (oldIdentityReturns(data, transition.old_code_,
                                            transition.transition_time_,
                                            settings.max_value_age_))
                {
                    transition.reject_reason_ = "old code measured again, streams interleave";
                }
                else if (otherSystemMadeSameTransition(streams, key, transition.old_code_,
                                                       transition.new_code_,
                                                       transition.transition_time_,
                                                       settings.max_value_age_))
                {
                    transition.reject_reason_ =
                        "other tracker system made the same transition, recode";
                }
                else if (!otherSystemObservedOldIdentity(streams, key, transition.old_code_,
                                                         sustained.first_,
                                                         transition.transition_time_,
                                                         settings.max_value_age_))
                {
                    transition.reject_reason_ =
                        "old identity not observed by another tracker system, undecidable";
                }

                transitions.push_back(transition);
            }

            // the new identity is the sustained one from here on
            sustained = candidate;
            candidate.reset();
        };

        for (const auto& info : data.reports_)
        {
            if (info.code_)
                process_code(info);

            // attribute identity context to the window the report belongs to, after the
            // state update so the first report of a new identity counts towards it
            CodeState& context = candidate.code_ ? candidate : sustained;

            if (info.has_acad_)
                context.acad_ = true;

            if (!info.acid_.empty())
                context.acids_.insert(info.acid_);
        }
    }

    return transitions;
}
