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

#include "evaluationdetail.h"

/*****************************************************************************
 * EvaluationDetailComments
 *****************************************************************************/

/**
 * Returns the number of stored comments for the given comment group.
 */
size_t EvaluationDetailComments::numComments(const std::string& group_id) const 
{
    if (!comments_.has_value())
        return 0;

    auto it = comments_.value().find(group_id);
    if (it == comments_.value().end())
        return 0;

    return it->second.size();
}

/**
 * Sets a general comment.
 */
EvaluationDetailComments& EvaluationDetailComments::generalComment(const std::string& c)
{
    comment_ = c;
    return *this;
}

/**
 * Returns the current general comment.
 */
const std::string& EvaluationDetailComments::generalComment() const
{
    return comment_;
}

/**
 * Stores the given comment under the given comment group and comment id.
 */
EvaluationDetailComments& EvaluationDetailComments::comment(const std::string& group_id,
                                                            const std::string& comment_id,
                                                            const std::string& c)
{
    comments()[ group_id ][ comment_id ] = c;
    return (*this);
}

/**
 * Returns the comment stored under the given comment group and comment id.
 */
std::string EvaluationDetailComments::comment(const std::string& group_id,
                                              const std::string& comment_id) const
{
    if (!comments_.has_value())
        return "";

    auto it = comments_.value().find(group_id);
    if (it == comments_.value().end())
        return "";

    auto itc = it->second.find(comment_id);
    if (itc == it->second.end())
        return "";

    return itc->second;
}

/**
 * Stores the given comment group under the given group id.
 */
EvaluationDetailComments& EvaluationDetailComments::group(const std::string& group_id,
                                                          const CommentGroup& cg)
{
    comments()[ group_id ] = cg;
    return (*this);
}

/**
 * Returns the comment group stored under the given group id.
 */
boost::optional<EvaluationDetailComments::CommentGroup> EvaluationDetailComments::group(const std::string& group_id) const
{
    if (!comments_.has_value())
        return {};

    auto it = comments_.value().find(group_id);
    if (it == comments_.value().end())
        return {};

    return it->second;
}

/**
 * Checks if comments are stored under the given group id.
 */
bool EvaluationDetailComments::hasComments(const std::string& group_id) const
{
    return numComments(group_id) > 0;
}

/**
 * Returns how many comments are stored under the given group id.
*/
std::map<std::string, EvaluationDetailComments::CommentGroup>& EvaluationDetailComments::comments() const
{
    if (!comments_.has_value())
        comments_ = std::map<std::string, EvaluationDetailComments::CommentGroup>();

    return comments_.value();
}

/*****************************************************************************
 * EvaluationDetail
 *****************************************************************************/

/**
*/
EvaluationDetail::EvaluationDetail(const Timestamp& ts, 
                                   const Position& pos)
:   timestamp_(ts)
{
    addPosition(pos);
}

/**
*/
EvaluationDetail::EvaluationDetail(const Timestamp& ts, 
                                   const Position& pos0,
                                   const Position& pos1)
:   timestamp_(ts)
{
    positions_.resize(2);
    positions_[ 0 ] = pos0;
    positions_[ 1 ] = pos1;
}

/**
*/
EvaluationDetail::EvaluationDetail(const Timestamp& ts, 
                                   const std::vector<Position>& positions)
:   timestamp_(ts)
,   positions_(positions)
{
}

/**
*/
EvaluationDetail& EvaluationDetail::setValue(const Key& key, const QVariant& value)
{
    if (key >= values_.size())
        values_.resize(key+1);

    values_[ key ] = value;

    return *this;
}

/**
*/
EvaluationDetail& EvaluationDetail::setValue(const Key& key, const boost::posix_time::ptime& value)
{
    QString v = QString::fromStdString(Utils::Time::toString(value));

    if (key >= values_.size())
        values_.resize(key+1);

    values_[ key ] = v;

    return *this;
}

/**
*/
EvaluationDetail& EvaluationDetail::setValue(const Key& key, const boost::posix_time::time_duration& value)
{
    double v = Utils::Time::partialSeconds(value);

    if (key >= values_.size())
        values_.resize(key+1);

    values_[ key ] = v;

    return *this;
}

/**
*/
QVariant EvaluationDetail::getValue(const Key& key) const
{
    if (key >= values_.size()) // never set
        return {};

    traced_assert(key < values_.size());
    return values_.at(key);
}

/**
*/
EvaluationDetail& EvaluationDetail::addPosition(const Position& p)
{
    positions_.push_back(p);

    return *this;
}

/**
*/
EvaluationDetail& EvaluationDetail::addPosition(const boost::optional<Position>& p)
{
    if (p.has_value())
        addPosition(p.value());
    
    return *this;
}

/**
*/
size_t EvaluationDetail::numPositions() const
{
    return positions_.size();
}

/**
*/
const std::vector<EvaluationDetail::Position>& EvaluationDetail::positions() const
{
    return positions_;
}

/**
*/
const EvaluationDetail::Position& EvaluationDetail::position(size_t idx) const
{
    return positions_.at(idx);
}

/**
*/
const EvaluationDetail::Position& EvaluationDetail::firstPos() const
{
    return *positions_.begin();
}

/**
*/
const EvaluationDetail::Position& EvaluationDetail::lastPos() const
{
    return *positions_.rbegin();
}

/**
*/
QRectF EvaluationDetail::bounds(double eps) const
{
    size_t n = numPositions();

    if (n == 0)
        return QRectF();

    double lat_min =  std::numeric_limits<double>::max();
    double lon_min =  std::numeric_limits<double>::max();
    double lat_max =  std::numeric_limits<double>::lowest();
    double lon_max =  std::numeric_limits<double>::lowest();

    for (size_t i = 0; i < n; ++i)
    {
        const auto& pos = positions_[ i ];

        if (pos.latitude_  < lat_min) lat_min = pos.latitude_;
        if (pos.longitude_ < lon_min) lon_min = pos.longitude_;
        if (pos.latitude_  > lat_max) lat_max = pos.latitude_;
        if (pos.longitude_ > lon_max) lon_max = pos.longitude_;
    }

    return QRectF(lat_min - eps, lon_min - eps, lat_max - lat_min + 2 * eps, lon_max - lon_min + 2 * eps);
}

/**
*/
EvaluationDetail::Details& EvaluationDetail::genDetails() const
{
    if (!details_.has_value())
        details_ = Details();

    return details_.value();
}

/**
*/
EvaluationDetail& EvaluationDetail::addDetail(const EvaluationDetail& detail)
{
    genDetails().push_back(detail);
    return *this;
}

/**
*/
EvaluationDetail& EvaluationDetail::addDetails(const Details& details)
{
    auto& det = genDetails();
    det.insert(det.begin(), details.begin(), details.end());
    return *this;
}

/**
*/
EvaluationDetail& EvaluationDetail::setDetails(const Details& details)
{
    details_ = details;
    return *this;
}

/**
*/
bool EvaluationDetail::hasDetails() const
{
    return details_.has_value();
}

/**
*/
size_t EvaluationDetail::numDetails() const
{
    if (!hasDetails())
        return 0;

    return details_.value().size();
}

/**
*/
const EvaluationDetail::Details& EvaluationDetail::details() const
{
    if (!hasDetails())
        throw std::runtime_error("EvaluationDetail::details(): No details attached");

    return details_.value();
}

/**
*/
EvaluationDetail& EvaluationDetail::generalComment(const std::string& c)
{
    comments().generalComment(c);

    return *this;
}
