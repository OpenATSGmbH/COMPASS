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

class ReconstructorBase;

/**
 * Executes the retrospective identity cut: for each swap candidate found by the
 * StreamIdentityDetector, the reports of the affected tracker track stream from the
 * transition onwards are moved out of their target into a new one.
 *
 * The cut is per stream, not a time cut of the target: in a merged multi-source target
 * both aircraft coexist in time, only the wrongly continued stream must be separated.
 *
 * Runs after association and before self-association, so the cut remainder is a valid
 * merge candidate and can be re-homed to the target it belongs to. The reports moved
 * are re-associated to the new target when the slice's associations are written, so no
 * database rewrite is needed - reports already purged from memory keep their previous
 * association.
 *
 * Pass stability: the ProbIMM repeat runs restore the target container from a snapshot
 * taken before the slice, so each pass re-detects and re-applies the same cut.
 */
class StreamIdentityCut
{
  public:
    // returns the number of cuts performed
    static unsigned int cutDetectedTransitions(ReconstructorBase& reconstructor);
};
