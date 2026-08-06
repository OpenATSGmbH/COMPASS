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

#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>

#include <set>
#include <string>
#include <vector>

class ReconstructorBase;

/**
 * Detects sustained Mode 3/A identity transitions within tracker track streams
 * (ds_id, line_id, track_number) and classifies them as track swap candidates or as
 * legitimate recodes.
 *
 * Streams are collected GLOBALLY from all in-memory target reports (associated or not),
 * so cross-system evidence is visible regardless of which target a stream was associated
 * to - association splits must not hide corroboration. This also reports each physical
 * transition exactly once.
 *
 * A stream carries a sustained code A which is replaced by a sustained code B. Such a
 * transition is a swap candidate only if it survives all of:
 *
 * - neither code is a conspicuity code (7000, 1000, ... are shared by many aircraft and
 *   therefore carry no identity information)
 * - the aircraft identity is not demonstrably unchanged (ACAD on both sides, or the same
 *   ACID on both sides, means the same aircraft recoded)
 * - the old identity actually ceases (if the old code keeps being freshly measured, the
 *   two streams merely interleave)
 * - no other tracker SYSTEM made the same transition on one of its own tracks (that is a
 *   recode of the real aircraft, seen independently). Lines of the same system are NOT
 *   independent - a swap inside a tracker shows on all of its output lines.
 * - another tracker SYSTEM observed the OLD code during the old identity's sustained
 *   window (positive old-identity coverage). If no other system ever saw the old
 *   identity, swap and recode are indistinguishable - e.g. code changes right after
 *   fresh track acquisition.
 *
 * Aged Mode 3/A repetitions are collapsed by measurement time (timestamp minus the
 * reported age), so evidence is counted in distinct measurements, not in reports.
 *
 * Analysis only: the detected transitions are executed by StreamIdentityCut.
 */
class StreamIdentityDetector
{
  public:
    struct Settings
    {
        boost::posix_time::time_duration max_value_age_;
        boost::posix_time::time_duration min_duration_;
        unsigned int min_updates_ {0};
        std::set<unsigned int> conspicuity_codes_;
    };

    struct Transition
    {
        int utn_ {-1}; // current track number lookup association, -1 if none

        unsigned int ds_id_ {0};
        unsigned int line_id_ {0};
        unsigned int track_number_ {0};

        boost::posix_time::ptime transition_time_; // first report carrying the new identity

        unsigned int old_code_ {0}; // mode 3/A codes as stored, print via octStringFromInt
        unsigned int new_code_ {0};

        unsigned int old_count_ {0}; // distinct sustaining measurements
        unsigned int new_count_ {0};

        boost::posix_time::time_duration old_duration_;
        boost::posix_time::time_duration new_duration_;

        bool acad_before_ {false};
        bool acad_after_ {false};

        std::set<std::string> acids_before_;
        std::set<std::string> acids_after_;

        // empty if the transition is a swap candidate, else why it was rejected
        std::string reject_reason_;

        bool accepted() const { return reject_reason_.empty(); }

        std::string asStr() const;
    };

    static Settings resolveSettings(const ReconstructorBase& reconstructor);

    // analyzes all in-memory tracker streams, returns classified transitions confirmed
    // by current-slice data
    static std::vector<Transition> analyze(const ReconstructorBase& reconstructor,
                                           const Settings& settings);
};
