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

#include <vector>
#include <string>

#include <QPointF>
#include <QRectF>

#include <boost/optional.hpp>

namespace projection
{

/**
*/
class CoordConverter
{
public:
    static std::vector<boost::optional<QPointF>> convert(const std::vector<QPointF>& points,
                                                         const std::string& srs_src,
                                                         const std::string& srs_dst);
    static QRectF convert(const QRectF& roi,
                          const std::string& srs_src,
                          const std::string& srs_dst);
};

}
