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

#include <functional>
#include <set>
#include <vector>
#include <utility>
#include <type_traits>

#include <boost/optional.hpp>

namespace Utils
{
namespace Combinatorial
{

/**
 * Basic dusplicate item check using a set of items.
 */
template<typename T>
class DefaultDuplicateCheck
{
public:
    DefaultDuplicateCheck(const boost::optional<T>& sentinel_value = boost::optional<T>()) : sentinel_value_(sentinel_value) {}
    ~DefaultDuplicateCheck() = default;

    /**
     * Adds the item to the duplicate check if not yet added and returns true, 
     * or returns false if the item already has been added.
     */
    bool add(const T& v)
    { 
        if (sentinel_value_.has_value() && sentinel_value_.value() == v)
            return true;  // skip sentinel value
        if (unique_elems_.count(v))
            return false; // already added
        
        unique_elems_.insert(v);
        return true;
    }

    /**
     * Erases the item from the duplicate check.
     */
    void erase(const T& v) 
    { 
        if (sentinel_value_.has_value() && sentinel_value_.value() == v)
            return; // skip sentinel value
        unique_elems_.erase(v);
    }

private:
    std::set<T>        unique_elems_;
    boost::optional<T> sentinel_value_;
};

/**
 * Basic dusplicate item check using a set of items.
 */
template<typename Titem, typename Tindex>
class AdapterDuplicateCheck
{
public:
    AdapterDuplicateCheck(const std::function<Tindex(const Titem&)>& get_index,
                          const boost::optional<Tindex>& sentinel_value = boost::optional<Tindex>()) 
    :   get_index_func_(get_index     )
    ,   check_index_   (sentinel_value) {}
    ~AdapterDuplicateCheck() = default;

    /**
     */
    bool add(const Titem& v)
    { 
        auto index = get_index_func_(v);
        return check_index_.add(index);
    }

    /**
     */
    void erase(const Titem& v) 
    { 
        auto index = get_index_func_(v);
        check_index_.erase(index);
    }

private:
    std::function<Tindex(const Titem&)> get_index_func_;
    DefaultDuplicateCheck<Tindex>       check_index_;
};

/**
 */
template <class T, class Callback, class DuplicateCheck = DefaultDuplicateCheck<T>>
bool cartesianProduct(const std::vector<std::vector<T>>& input,
                      Callback&& callback,
                      bool distinct_values = false,
                      const DuplicateCheck& duplicate_check = DuplicateCheck())
{
    // Edge cases
    if (input.empty()) {                     // product of zero sets is {()}
        static const std::vector<T> empty;
        return callback(empty);
    }
    for (const auto& v : input) {            // any empty factor => empty product
        if (v.empty()) return true;
    }

    const std::size_t k = input.size();

    // --- FAST PATH: no distinctness needed, no sentinel behavior required ---------------------
    if (!distinct_values) {
        std::vector<std::size_t> idx(k, 0);
        std::vector<T> tuple(k);

        while (true) {
            for (std::size_t i = 0; i < k; ++i) 
                tuple[i] = input[i][idx[i]];
            
            if (!callback(tuple))
                return false;

            std::size_t pos = k;
            while (pos > 0) {
                --pos;
                if (++idx[pos] < input[pos].size()) {
                    for (std::size_t j = pos + 1; j < k; ++j) idx[j] = 0;
                    break;
                }
                idx[pos] = 0;
            }
            if (pos == 0 && idx[0] == 0) 
                return true;
        }
    }

    // --- DISTINCT PATH: no duplicates across positions (except sentinel) ----------------------
    std::vector<std::size_t> next_idx(k, 0); // next candidate to try at each depth
    std::vector<T> tuple(k);                 // reused buffer
    std::vector<bool> counted(k, false);     // did we insert tuple[depth] into 'used'?
    std::size_t depth = 0;

    auto is_duplicate = duplicate_check;

    while (true) {
        if (depth == k) {
            if (!callback(tuple))
                return false;
            --depth;
            if (counted[depth]) { is_duplicate.erase(tuple[depth]); counted[depth] = false; }
            continue;
        }

        const auto& list = input[depth];
        std::size_t& i = next_idx[depth];

        bool advanced = false;
        while (i < list.size()) {
            const T& candidate = list[i];
            if (!distinct_values || is_duplicate.add(candidate)) 
            {
                tuple[depth] = candidate;
                counted[depth] = true;
                ++i;
                ++depth;
                if (depth < k) next_idx[depth] = 0;
                advanced = true;
                break;
            }
            ++i;
        }

        if (advanced) continue;

        if (depth == 0) break;  // explored everything
        i = 0;
        --depth;
        if (counted[depth]) { is_duplicate.erase(tuple[depth]); counted[depth] = false; }
    }

    return true;
}

/**
 */
template <class T, class DuplicateCheck = DefaultDuplicateCheck<T>>
std::vector<std::vector<T>> cartesianProduct(const std::vector<std::vector<T>>& input,
                                             bool distinct = false,
                                             const DuplicateCheck& duplicate_check = DuplicateCheck())
{
    std::vector<std::vector<T>> result;

    std::function<bool(const std::vector<T>&)> cb = [ &result ] (const std::vector<T>& combo)
    {
        result.push_back(combo);
        return true;
    };

    cartesianProduct(input, cb, distinct, duplicate_check);

    return result;
}

}  // namespace Combinatorial
}  // namespace Utils
