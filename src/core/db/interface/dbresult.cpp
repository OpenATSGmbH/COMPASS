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

#include "dbresult.h"
#include "property_templates.h"
#include "timeconv.h"

#include "json.hpp"

/**
 */
std::string DBResult::printResult() const
{
    if (!containsData() || !buffer_ || buffer_->size() < 1)
        return "";

    std::stringstream ss;

    #define UpdateFunc(PDType, DType, Suffix)                                                                  \
        bool is_null = buffer_->isNull(p, r);                                                                  \
        ss << (is_null ? "null" : property_templates::toString<DType>(buffer_->get<DType>(pname).get(r), dec));

    #define NotFoundFunc                                                                        \
        logerr << "unknown property type " << Property::asString(dtype); \
        traced_assert(false);

    size_t n  = buffer_->size();
    size_t np = buffer_->properties().size();

    const auto& properties = buffer_->properties().properties();

    for (size_t c = 0; c < np; ++c)
        ss << properties[ c ].name() << (c < np - 1 ? "\t|\t" : "");
    
    ss << "\n";

    const int dec = 2;

    for (size_t r = 0; r < n; ++r)
    {
        for (size_t c = 0; c < np; ++c)
        {
            const auto& p     = properties[ c ];
            auto        dtype = p.dataType();
            const auto& pname = p.name();

            SwitchPropertyDataType(dtype, UpdateFunc, NotFoundFunc)

            ss << "\t\t";
        }

        if (r < n - 1)
            ss << "\n";
    }

    return ss.str();
}
