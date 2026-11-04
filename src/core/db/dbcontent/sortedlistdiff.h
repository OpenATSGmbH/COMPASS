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

#include <cstddef>
#include <vector>

namespace dbContent
{

/**
 * One contiguous change of a sorted list, see diffSortedLists().
 */
struct SortedListOp
{
    bool   insert       = false; // insert (else remove) count rows at row
    size_t row          = 0;     // row in the list as it is when the op is applied
    size_t count        = 0;
    size_t target_begin = 0;     // insert: first inserted value in the target list
};

/**
 * Turns the current list into the target list with contiguous removes and inserts, kept values stay in place.
 * - both lists must be sorted by the same strict order (less), i.e. without equal values
 * - the ops are applied in the returned order, each row refers to the list after the previous ops
 */
template<typename T, typename Less>
std::vector<SortedListOp> diffSortedLists(const std::vector<T>& current,
                                          const std::vector<T>& target,
                                          Less                  less)
{
    std::vector<SortedListOp> ops;

    size_t i   = 0; // current
    size_t j   = 0; // target
    size_t row = 0; // row of current[ i ] after the ops so far

    const size_t n_cur = current.size();
    const size_t n_tgt = target.size();

    while (i < n_cur || j < n_tgt)
    {
        // run of values only in current => remove
        if (j == n_tgt || (i < n_cur && less(current[ i ], target[ j ])))
        {
            SortedListOp op;
            op.row = row;

            while (i < n_cur && (j == n_tgt || less(current[ i ], target[ j ])))
            {
                ++i;
                ++op.count;
            }

            ops.push_back(op);
            continue;
        }

        // run of values only in target => insert
        if (i == n_cur || less(target[ j ], current[ i ]))
        {
            SortedListOp op;
            op.insert       = true;
            op.row          = row;
            op.target_begin = j;

            while (j < n_tgt && (i == n_cur || less(target[ j ], current[ i ])))
            {
                ++j;
                ++op.count;
            }

            row += op.count;

            ops.push_back(op);
            continue;
        }

        // in both => kept
        ++i;
        ++j;
        ++row;
    }

    return ops;
}

} // namespace dbContent
