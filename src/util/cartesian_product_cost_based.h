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
namespace CostBased
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
 * Cost-based early exit functionality for cartesian product calculation.
 * Allows skipping combinations whose costs fall below a threshold.
 */
template<typename T, typename CostType = double>
class CostBasedEarlyExit
{
public:
    // Function type to compute cost for an item at specific positions
    using CostFunction = std::function<CostType(size_t vector_idx, size_t element_idx, const T& element)>;
    
    // Function type to determine the current threshold
    using ThresholdFunction = std::function<CostType()>;
    
    CostBasedEarlyExit(
        const CostFunction& cost_func = nullptr, 
        const ThresholdFunction& threshold_func = nullptr,
        CostType initial_cost = CostType(1))
        : cost_func_(cost_func)
        , threshold_func_(threshold_func)
        , initial_cost_(initial_cost)
    {}
    ~CostBasedEarlyExit() = default;

    // Check if adding an element keeps the cost above threshold
    // Returns true if we should continue, false if we should prune
    bool checkAndUpdate(CostType& current_cost, size_t vector_idx, size_t element_idx, const T& element) const
    {
        if (!cost_func_ || !threshold_func_)
            return true;  // No cost checking if functions not provided
            
        CostType element_cost = cost_func_(vector_idx, element_idx, element);
        CostType new_cost = current_cost * element_cost;
        
        if (new_cost < threshold_func_())
            return false;  // Cost below threshold, prune this branch
            
        current_cost = new_cost;  // Update the running cost
        return true;
    }
    
    // Reset cost to initial value
    CostType initialize() const { return initial_cost_; }
    
    // Remove cost contribution when backtracking
    void backtrack(CostType& current_cost, size_t vector_idx, size_t element_idx, const T& element) const
    {
        if (!cost_func_)
            return;
            
        CostType element_cost = cost_func_(vector_idx, element_idx, element);
        if (element_cost != CostType(0))
            current_cost /= element_cost;
    }
    
    // Check if cost checking is enabled
    bool isEnabled() const { return cost_func_ != nullptr && threshold_func_ != nullptr; }
    
private:
    CostFunction cost_func_;
    ThresholdFunction threshold_func_;
    CostType initial_cost_;
};

/**
 */
template <class T, 
          class Callback, 
          class DuplicateCheck = DefaultDuplicateCheck<T>,
          class CostCheck = CostBasedEarlyExit<T>>
bool cartesianProduct(const std::vector<std::vector<T>>& input,
                      Callback&& callback,
                      bool distinct_values = false,
                      const DuplicateCheck& duplicate_check = DuplicateCheck(),
                      const CostCheck& cost_check = CostCheck())
{
    // Edge cases
    if (input.empty()) {                     // product of zero sets is {()}
        static const std::vector<T> empty;
        return callback(empty, 0.0);
    }
    for (const auto& v : input) {            // any empty factor => empty product
        if (v.empty()) return true;
    }

    const std::size_t k = input.size();
    using CostType = decltype(cost_check.initialize());

    // --- FAST PATH: no distinctness needed, no sentinel behavior required, no cost check required ---------------------
    if (!distinct_values && !cost_check.isEnabled()) {
        std::vector<std::size_t> idx(k, 0);
        std::vector<T> tuple(k);

        while (true) {
            for (std::size_t i = 0; i < k; ++i) 
                tuple[i] = input[i][idx[i]];
            
            if (!callback(tuple, 0.0))
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

    // --- COST PATH: no distinctness, but with cost checking ---------------------
    if (!distinct_values && cost_check.isEnabled()) {
        std::vector<std::size_t> idx(k, 0);
        std::vector<T> tuple(k);
        std::vector<CostType> costs_at_depth(k+1);
        costs_at_depth[0] = cost_check.initialize();

        size_t depth = 0;
        
        while (true) {
            // Try to extend the current partial solution
            while (depth < k) {
                const auto& list = input[depth];
                std::size_t& i = idx[depth];
                
                // Try candidates at this depth
                bool found_valid_candidate = false;
                while (i < list.size()) {
                    // Check if this element's cost keeps us above threshold
                    if (cost_check.checkAndUpdate(costs_at_depth[depth], depth, i, list[i])) {
                        tuple[depth] = list[i];
                        costs_at_depth[depth+1] = costs_at_depth[depth]; // Store cost for next level
                        ++i;
                        ++depth;
                        idx[depth] = 0; // Reset counter for next depth
                        found_valid_candidate = true;
                        break;
                    }
                    ++i;
                }
                
                if (!found_valid_candidate) {
                    // No valid candidate at this depth, backtrack
                    idx[depth] = 0;
                    if (depth == 0) return true; // Explored everything
                    --depth;
                }
            }
            
            // Process complete solution
            if (depth == k) {
                if (!callback(tuple, costs_at_depth[k]))
                    return false;
                --depth; // Backtrack to continue search
            }
            
            // Move to next candidate
            std::size_t pos = depth;
            while (pos > 0) {
                if (idx[pos] < input[pos].size()) break;
                idx[pos] = 0;
                --pos;
            }
            
            // If we've exhausted all possibilities, we're done
            if (pos == 0 && idx[0] >= input[0].size())
                return true;
        }
    }

    // --- GENERAL PATH: with distinctness and/or cost checking ----------------------
    //@TODO: !needs additional tests if this path works as expected!
    std::vector<std::size_t> next_idx(k, 0);   // next candidate to try at each depth
    std::vector<T> tuple(k);                   // reused buffer
    std::vector<bool> counted(k, false);       // did we insert tuple[depth] into 'used'?
    std::size_t depth = 0;
    CostType current_cost = cost_check.initialize();

    auto is_duplicate = duplicate_check;

    while (true) {
        if (depth == k) {
            if (!callback(tuple, current_cost))
                return false;
            --depth;
            if (counted[depth]) { 
                is_duplicate.erase(tuple[depth]); 
                counted[depth] = false; 
                if (cost_check.isEnabled())
                    cost_check.backtrack(current_cost, depth, next_idx[depth]-1, tuple[depth]);
            }
            continue;
        }

        const auto& list = input[depth];
        std::size_t& i = next_idx[depth];

        bool advanced = false;
        while (i < list.size()) {
            const T& candidate = list[i];
            bool duplicate_ok = !distinct_values || is_duplicate.add(candidate);
            
            // Check both duplicate and cost constraints
            bool cost_ok = duplicate_ok;
            if (duplicate_ok && cost_check.isEnabled()) {
                cost_ok = cost_check.checkAndUpdate(current_cost, depth, i, candidate);
                // Undo duplicate check if cost check fails
                if (!cost_ok && distinct_values) {
                    is_duplicate.erase(candidate);
                }
            }
            
            if (cost_ok) {
                tuple[depth] = candidate;
                counted[depth] = duplicate_ok;
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
        if (counted[depth]) { 
            is_duplicate.erase(tuple[depth]); 
            counted[depth] = false;
            if (cost_check.isEnabled())
                cost_check.backtrack(current_cost, depth, next_idx[depth]-1, tuple[depth]);
        }
    }

    return true;
}

/**
 */
template <class T, 
          class DuplicateCheck = DefaultDuplicateCheck<T>,
          class CostCheck = CostBasedEarlyExit<T>>
std::vector<std::vector<T>> cartesianProduct(const std::vector<std::vector<T>>& input,
                                             bool distinct = false,
                                             const DuplicateCheck& duplicate_check = DuplicateCheck(),
                                             const CostCheck& cost_check = CostCheck())
{
    std::vector<std::vector<T>> result;

    std::function<bool(const std::vector<T>&)> cb = [ &result ] (const std::vector<T>& combo)
    {
        result.push_back(combo);
        return true;
    };

    cartesianProduct(input, cb, distinct, duplicate_check, cost_check);

    return result;
}

}  // namespace CostBased
}  // namespace Combinatorial
}  // namespace Utils
