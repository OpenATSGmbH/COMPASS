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

#include "dbcontent/target/targetposition.h"

#include <QVariant>

#include "boost/date_time/posix_time/ptime.hpp"

namespace EvaluationRequirement
{
class PositionDetail
{
public:
    PositionDetail(
            boost::posix_time::ptime timestamp, dbContent::TargetPosition tst_pos,
            bool has_ref_pos, dbContent::TargetPosition ref_pos,
            QVariant pos_inside, QVariant value, bool check_passed,
            unsigned int num_pos, unsigned int num_no_ref,
            unsigned int num_inside, unsigned int num_outside,
            unsigned int num_check_failed, unsigned int num_check_passed,
            const std::string& comment)
        : timestamp_(timestamp), tst_pos_(tst_pos), has_ref_pos_(has_ref_pos), ref_pos_(ref_pos),
          value_(value), check_passed_(check_passed),
          pos_inside_(pos_inside), num_pos_(num_pos), num_no_ref_(num_no_ref),
          num_inside_(num_inside), num_outside_(num_outside),
          num_check_failed_(num_check_failed), num_check_passed_(num_check_passed),
          comment_(comment)
    {
    }

    boost::posix_time::ptime timestamp_;

    dbContent::TargetPosition tst_pos_;

    bool has_ref_pos_ {false};
    dbContent::TargetPosition ref_pos_;

    QVariant value_ {0};
    bool check_passed_ {false};

    QVariant pos_inside_ {false};

    unsigned int num_pos_ {0};
    unsigned int num_no_ref_ {0};
    unsigned int num_inside_ {0};
    unsigned int num_outside_ {0};

    unsigned int num_check_failed_ {0};
    unsigned int num_check_passed_ {0};

    std::string comment_;
};
}
