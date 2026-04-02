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

#include "eval/results/base/probabilitybase.h"

#include "targetposition.h"

#include <memory>
#include <string>
#include <map>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class EvaluationDetailComments;

namespace EvaluationRequirement
{
class Base;
}

namespace EvaluationRequirementResult
{

/**
*/
class DubiousBase
{
public:
    DubiousBase();
    DubiousBase(unsigned int num_updates,
                unsigned int num_pos_outside,
                unsigned int num_pos_inside,
                unsigned int num_pos_inside_dubious);

    unsigned int numUpdates() const;
    unsigned int numPosOutside() const;
    unsigned int numPosInside() const;
    unsigned int numPosInsideDubious() const;

protected:
    unsigned int num_updates_            {0};
    unsigned int num_pos_outside_        {0};
    unsigned int num_pos_inside_         {0};
    unsigned int num_pos_inside_dubious_ {0};

    mutable boost::optional<double> p_dubious_update_;
};

/**
*/
class SingleDubiousBase : public DubiousBase, public SingleProbabilityBase
{
public:
    /**
     * Raw data for a detail.
     */
    struct DetailData
    {
        DetailData() = default;
        DetailData(unsigned int utn_or_track_number, boost::posix_time::ptime ts_begin);

        void assignTo(EvaluationDetail& detail) const;
        unsigned int numDubious() const;

        unsigned int                       utn_or_tracknum        = 0;
        bool                               first_inside           = true;
        boost::posix_time::ptime           tod_begin;
        boost::posix_time::ptime           tod_end;
        boost::posix_time::time_duration   duration;
        unsigned int                       num_pos_inside         = 0;
        unsigned int                       num_pos_inside_dubious = 0;
        bool                               has_mode_ac            = false;
        bool                               has_mode_s             = false;
        bool                               left_sector            = false;
        bool                               is_dubious             = false;
        dbContent::TargetPosition          pos_begin;
        dbContent::TargetPosition          pos_last;
        EvaluationDetails                  details;
        std::map<std::string, std::string> dubious_reasons;
    };

    SingleDubiousBase(const std::string& result_type,
                      const std::string& result_id,
                      std::shared_ptr<EvaluationRequirement::Base> requirement,
                      const SectorLayer& sector_layer,
                      unsigned int utn,
                      const EvaluationTargetData* target,
                      EvaluationCalculator& calculator,
                      const EvaluationDetails& details,
                      unsigned int num_updates,
                      unsigned int num_pos_outside,
                      unsigned int num_pos_inside,
                      unsigned int num_pos_inside_dubious);
    virtual ~SingleDubiousBase();

    static const std::string DetailCommentGroupDubious;

    enum DetailKey
    {
        CommentGroupDubious,
        UTNOrTrackNum,   //unsigned int
        FirstInside,     //bool
        TODBegin,        //ptime
        TODEnd,          //ptime
        Duration,        //time_duration
        NumPosInside,    //unsigned int
        NumPosInsideDub, //unsigned int
        HasModeAC,       //bool
        HasModeS,        //bool
        LeftSector,      //bool
        IsDubious       //bool
    };

    static void logComment(EvaluationDetail& d, const std::string& id, const std::string& comment);
    static void logComments(EvaluationDetail& d, const EvaluationDetailComments::CommentGroup& group);
    static EvaluationDetails generateDetails(const std::vector<DetailData>& detail_data);

protected:
    static std::string dubiousReasonsString(const EvaluationDetailComments& comments);
};

/**
*/
class JoinedDubiousBase : public DubiousBase, public JoinedProbabilityBase
{
public:
    JoinedDubiousBase(const std::string& result_type,
                      const std::string& result_id,
                      std::shared_ptr<EvaluationRequirement::Base> requirement,
                      const SectorLayer& sector_layer,
                      EvaluationCalculator& calculator);
protected:
    float duration_all_     {0};
    float duration_nondub_  {0};
    float duration_dubious_ {0};
};

}
