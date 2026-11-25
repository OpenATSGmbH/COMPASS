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

#include "targetreportmapping.h"

#include <iterator>

using namespace boost::posix_time;

namespace dbContent {

namespace TargetReport {

Bracket findBracket(const IndexMap& indexes, const ptime& timestamp)
{
    Bracket ret;

    auto lower = indexes.lower_bound(timestamp);
    auto upper = indexes.upper_bound(timestamp);

    if (lower != indexes.begin())
        ret.before = DataID(*std::prev(lower));

    if (lower != upper)
        ret.exact = DataID(*lower);

    if (upper != indexes.end())
        ret.after = DataID(*upper);

    return ret;
}

DataMapping findDataMapping(const IndexMap& indexes, const ptime& timestamp)
{
    DataMapping ret;
    ret.timestamp_ = timestamp;

    Bracket bracket = findBracket(indexes, timestamp);

    if (bracket.exact)
    {
        ret.has_ref1_       = true;
        ret.timestamp_ref1_ = timestamp;
        ret.dataid_ref1_    = *bracket.exact;
    }
    else if (bracket.before && bracket.after)
    {
        ret.has_ref1_       = true;
        ret.timestamp_ref1_ = bracket.before->timestamp();
        ret.dataid_ref1_    = *bracket.before;

        ret.has_ref2_       = true;
        ret.timestamp_ref2_ = bracket.after->timestamp();
        ret.dataid_ref2_    = *bracket.after;
    }

    return ret;
}

DataMappingTimes findDataMappingTimes(const IndexMap& indexes, const ptime& timestamp)
{
    DataMappingTimes ret;
    ret.timestamp_ = timestamp;

    Bracket bracket = findBracket(indexes, timestamp);

    const auto& other2 = bracket.exact ? bracket.exact : bracket.after;

    if (!other2 || (!bracket.exact && !bracket.before))
        return ret;

    ret.has_other2_       = true;
    ret.timestamp_other2_ = other2->timestamp();
    ret.dataid_other2_    = *other2;

    if (bracket.before)
    {
        ret.has_other1_       = true;
        ret.timestamp_other1_ = bracket.before->timestamp();
        ret.dataid_other1_    = *bracket.before;
    }

    return ret;
}

} // namespace TargetReport

} // namespace dbContent
