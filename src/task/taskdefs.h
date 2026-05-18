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

namespace task
{
    enum TaskResultType
    {
        Generic = 0,
        Evaluation,
        DataSourceAnalysis
    };

    enum UpdateState
    {
        UpToDate = 0,         // no update needed
        ContentUpdateNeeded,  // specific contents need update (e.g. tables)
        PartialUpdateNeeded,  // partial update needed
        FullUpdateNeeded,     // full update needed
        Locked                // critical full update needed, result is locked
    };
}
