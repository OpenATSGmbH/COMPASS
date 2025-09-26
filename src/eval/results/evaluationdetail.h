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
#include "timeconv.h"
#include "traced_assert.h"

#include <string>
#include <map>
#include <tuple>

#include <QVariant>
#include <QColor>
#include <QRectF>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>

/**
*/
class EvaluationDetailComments
{
public:
    typedef std::map<std::string, std::string> CommentGroup;

    EvaluationDetailComments() = default;
    virtual ~EvaluationDetailComments() = default;

    bool hasComments(const std::string& group_id) const;
    size_t numComments(const std::string& group_id) const;

    EvaluationDetailComments& generalComment(const std::string& c);
    const std::string& generalComment() const;

    EvaluationDetailComments& comment(const std::string& group_id,
                                      const std::string& comment_id,
                                      const std::string& c);
    std::string comment(const std::string& group_id,
                        const std::string& comment_id) const;

    EvaluationDetailComments& group(const std::string& group_id,
                                    const CommentGroup& cg);
    boost::optional<CommentGroup> group(const std::string& group_id) const;

private:
    std::map<std::string, CommentGroup>& comments() const;

    std::string comment_;
    mutable boost::optional<std::map<std::string, CommentGroup>> comments_;
};

/**
*/
class EvaluationDetail
{
public:
    typedef unsigned char                 Key;
    typedef boost::posix_time::ptime      Timestamp;
    typedef dbContent::TargetPosition     Position;
    typedef std::vector<EvaluationDetail> Details;

    EvaluationDetail() = default;
    EvaluationDetail(const Timestamp& ts, 
                     const Position& pos);
    EvaluationDetail(const Timestamp& ts, 
                     const Position& pos0,
                     const Position& pos1);
    EvaluationDetail(const Timestamp& ts, 
                     const std::vector<Position>& positions);
    virtual ~EvaluationDetail() = default;

    const Timestamp& timestamp() const { return timestamp_; }
    
    EvaluationDetail& setValue(const Key& key, const QVariant& value);
    EvaluationDetail& setValue(const Key& key, const boost::posix_time::ptime& value);
    EvaluationDetail& setValue(const Key& key, const boost::posix_time::time_duration& value);
    QVariant getValue(const Key& key) const;

    template<typename T>
    boost::optional<T> getValueAs(const Key& key) const
    {
        if (key >= values_.size()) // never set
            return {};

        traced_assert(key < values_.size());
        if (!values_.at(key).canConvert<T>())
            return {};

        return values_.at(key).value<T>();
    }

    template<typename T>
    T getValueAsOrAssert(const Key& key) const
    {
        auto v = getValueAs<T>(key);
        traced_assert(v.has_value());
        return v.value();
    }

    EvaluationDetail& addPosition(const Position& p);
    EvaluationDetail& addPosition(const boost::optional<Position>& p);
    size_t numPositions() const;
    const std::vector<Position>& positions() const;
    const Position& position(size_t idx) const;
    const Position& firstPos() const;
    const Position& lastPos() const;

    QRectF bounds(double eps = 0.0) const;

    EvaluationDetailComments& comments() { return comments_; }
    const EvaluationDetailComments& comments() const { return comments_; }
    EvaluationDetail& generalComment(const std::string& c);

    EvaluationDetail& addDetail(const EvaluationDetail& detail);
    EvaluationDetail& addDetails(const Details& details);
    EvaluationDetail& setDetails(const Details& details);
    bool hasDetails() const;
    size_t numDetails() const;
    const Details& details() const;
    
private:
    Details& genDetails() const;

    Timestamp                        timestamp_;
    std::vector<Position>            positions_;  // by design the first value shall refer to the position the detail is assigned to,
                                                  // the last position shall refer to the position the detail is measured against,
                                                  // e.g. a reference trajectory position, an interval begin, etc.
    std::vector<QVariant>            values_;
    EvaluationDetailComments         comments_;
    mutable boost::optional<Details> details_;
};

template<>
inline boost::optional<boost::posix_time::ptime> EvaluationDetail::getValueAs<boost::posix_time::ptime>(const Key& key) const
{
    auto v = getValueAs<QString>(key);
    if (!v.has_value())
        return {};
    return Utils::Time::fromString(v.value().toStdString());
}
template<>
inline boost::optional<boost::posix_time::time_duration> EvaluationDetail::getValueAs<boost::posix_time::time_duration>(const Key& key) const
{
    auto v = getValueAs<double>(key);
    if (!v.has_value())
        return {};
    return Utils::Time::partialSeconds(v.value());
}
