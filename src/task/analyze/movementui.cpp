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

#include "movementui.h"
#include "analysisdataset.h"
#include "dbcontent/target/targetreportchain.h"

#include <algorithm>
#include <utility>

namespace analysis
{

using boost::posix_time::ptime;
using dbContent::TargetReport::Chain;

SpeedSamples gatherTestSpeeds(unsigned int utn, AnalysisDataset& dataset)
{
    std::vector<std::pair<ptime, double>> rows;
    for (const auto& dbc : dataset.testDbContentsPresent())
    {
        if (!dataset.hasTestChain(utn, dbc))
            continue;
        auto& chain = dataset.testChain(utn, dbc);
        for (const auto& kv : chain.timestampIndexes())
        {
            Chain::DataID id(kv.first);
            auto gs = chain.groundSpeed(id);
            rows.emplace_back(kv.first, gs.has_value()
                              ? static_cast<double>(*gs)
                              : std::numeric_limits<double>::quiet_NaN());
        }
    }
    std::sort(rows.begin(), rows.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    SpeedSamples out;
    out.ts.reserve(rows.size());
    out.sp.reserve(rows.size());
    for (const auto& r : rows) { out.ts.push_back(r.first); out.sp.push_back(r.second); }
    return out;
}

SpeedSamples gatherRefSpeeds(Chain& ref_chain)
{
    SpeedSamples out;  // timestampIndexes() is time-sorted
    for (const auto& kv : ref_chain.timestampIndexes())
    {
        Chain::DataID id(kv.first);
        auto gs = ref_chain.groundSpeed(id);
        out.ts.push_back(kv.first);
        out.sp.push_back(gs.has_value() ? static_cast<double>(*gs)
                                        : std::numeric_limits<double>::quiet_NaN());
    }
    return out;
}

}  // namespace analysis
