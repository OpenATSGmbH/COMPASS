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

#include "kalman_tools.h"

namespace kalman
{

/**
 */
void mixStates(Vector& x_mixed,
               Matrix& P_mixed,
               const std::vector<Vector>& xs,
               const std::vector<Matrix>& Ps,
               const std::vector<double>& weights)
{
    traced_assert(xs.size() == Ps.size());
    traced_assert(xs.size() == weights.size());

    size_t n = xs.size();

    x_mixed.setZero();
    P_mixed.setZero();

    if (n == 0)
        return;

    x_mixed = xs[ 0 ];
    x_mixed.setZero();

    P_mixed = Ps[ 0 ];
    P_mixed.setZero();

    for (size_t i = 0; i < n; ++i)
    {
        x_mixed += xs[ i ] * weights[ i ];
    }

    Vector y;
    for (size_t i = 0; i < n; ++i)
    {
        y  = xs[ i ] - x_mixed;
        P_mixed += (Ps[ i ] + y * y.transpose()) * weights[ i ];
    }
}

} // namespace kalman
