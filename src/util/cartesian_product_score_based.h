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
namespace ScoreBased
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
class ScoreCheckMultiply
{
public:
    ScoreCheckMultiply() = default;
    virtual ~ScoreCheckMultiply() = default;

    virtual void init(size_t n) {}
    virtual bool add(std::size_t vec_idx, std::size_t elem_idx) { return true; }
    virtual void erase() {}
    virtual double score() const { return 0.0; }
    virtual bool enabled() const { return false; }
};

/**
 * Score check for early exit in cartesian product.
 * Maintains a stack of scores for each depth.
 * Handles scores in [0, 1].
 */
template<typename T_score_func, typename T_thres_func>
class ScoreCheckMultiplyT : public ScoreCheckMultiply
{
public:
    ScoreCheckMultiplyT(const T_score_func& score_func,
                        const T_thres_func& threshold_func)
    :   score_func_    (score_func    )
    ,   threshold_func_(threshold_func)
    {
    }

    virtual ~ScoreCheckMultiplyT() = default;

    void init(size_t n) override
    {
        idx_ = 0;
        score_stack_.resize(n + 1);
        score_stack_[ 0 ] = 1.0;
    }

    // Called when adding an element at (vec_idx, elem_idx)
    bool add(std::size_t vec_idx, std::size_t elem_idx) override
    {
        if (!valid_) return true;
        double new_score = score_stack_[ idx_ ] * score_func_(vec_idx, elem_idx);
        ++idx_;
        score_stack_[ idx_ ] = new_score;
        return new_score > threshold_func_();
    }

    // Called when backtracking/removing an element
    void erase() override
    {
        if (idx_ > 0)
            --idx_;
    }

    double score() const override { return score_stack_[ idx_ ]; }
    bool enabled() const override { return true; }

private:
    T_score_func        score_func_;
    T_thres_func        threshold_func_;
    std::vector<double> score_stack_;
    size_t              idx_ = 0;
    bool                valid_ = false;
};

/**
 */
template <class T, 
          class Callback,
          class DuplicateCheck = DefaultDuplicateCheck<T>,
          class ScoreCheckT = ScoreCheckMultiply>
bool cartesianProduct(const std::vector<std::vector<T>>& input,
                      Callback&& callback,
                      bool distinct_values = false,
                      const DuplicateCheck& duplicate_check = DuplicateCheck(),
                      const ScoreCheckT& score_check = ScoreCheckT())
{
    // Edge cases
    if (input.empty()) {                     // product of zero sets is {()}
        static const std::vector<T> empty;
        return callback(empty, boost::optional<double>());
    }
    for (const auto& v : input) {            // any empty factor => empty product
        if (v.empty()) return true;
    }

    const std::size_t k = input.size();

    // --- FAST PATH: no distinctness, no score check ---
    if (!distinct_values && !score_check.enabled()) {
        std::vector<std::size_t> idx(k, 0);
        std::vector<T> tuple(k);

        while (true) {
            for (std::size_t i = 0; i < k; ++i) 
                tuple[i] = input[i][idx[i]];
            
            if (!callback(tuple, boost::optional<double>()))
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

    // --- SCORE CHECK PATH: only score check, no distinctness ---
    if (!distinct_values && score_check.enabled()) {
        std::vector<std::size_t> next_idx(k, 0);
        std::vector<T> tuple(k);
        std::size_t depth = 0;
        auto scores = score_check;
        scores.init(k);

        while (true) {
            if (depth == k) {
                if (!callback(tuple, scores.score()))
                    return false;
                --depth;
                scores.erase();
                continue;
            }

            const auto& list = input[depth];
            std::size_t& i = next_idx[depth];

            bool advanced = false;
            while (i < list.size()) {
                tuple[depth] = list[i];
                bool cost_ok = scores.add(depth, i);
                if (cost_ok) {
                    ++i;
                    ++depth;
                    if (depth < k) next_idx[depth] = 0;
                    advanced = true;
                    break;
                }
                scores.erase();
                ++i;
            }

            if (advanced) continue;

            if (depth == 0) break;  // explored everything
            i = 0;
            --depth;
            scores.erase();
        }

        return true;
    }

    // --- DISTINCT PATH: no duplicates across positions (except sentinel) ---
    //@TODO: !this path needs further tests!
    std::vector<std::size_t> next_idx(k, 0); // next candidate to try at each depth
    std::vector<T> tuple(k);                 // reused buffer
    std::vector<bool> counted(k, false);     // did we insert tuple[depth] into 'used'?
    std::size_t depth = 0;

    auto is_duplicate = duplicate_check;
    auto scores = score_check;
    scores.init(k);

    while (true) {
        if (depth == k) {
            if (!callback(tuple, scores.enabled() ? scores.score() : boost::optional<double>()))
                return false;
            --depth;
            if (counted[depth]) { is_duplicate.erase(tuple[depth]); counted[depth] = false; }
            if (scores.enabled()) scores.erase();
            continue;
        }

        const auto& list = input[depth];
        std::size_t& i = next_idx[depth];

        bool advanced = false;
        while (i < list.size()) {
            const T& candidate = list[i];
            bool can_use = (!distinct_values || is_duplicate.add(candidate));
            bool score_ok = true;
            if (scores.enabled()) {
                score_ok = scores.add(depth, i);
            }
            if (can_use && score_ok) 
            {
                tuple[depth] = candidate;
                counted[depth] = true;
                ++i;
                ++depth;
                if (depth < k) next_idx[depth] = 0;
                advanced = true;
                break;
            }
            if (scores.enabled()) scores.erase();
            if (can_use && distinct_values) is_duplicate.erase(candidate);
            ++i;
        }

        if (advanced) continue;

        if (depth == 0) break;  // explored everything
        i = 0;
        --depth;
        if (counted[depth]) { is_duplicate.erase(tuple[depth]); counted[depth] = false; }
        if (scores.enabled()) scores.erase();
    }

    return true;
}

/**
 */
template <class T, 
          class DuplicateCheck = DefaultDuplicateCheck<T>,
          class ScoreCheckT = ScoreCheckMultiply>
std::vector<std::pair<std::vector<T>,boost::optional<double>>> cartesianProduct(const std::vector<std::vector<T>>& input,
                                                                                bool distinct = false,
                                                                                const DuplicateCheck& duplicate_check = DuplicateCheck(),
                                                                                const ScoreCheckT& score_check = ScoreCheckT())
{
    std::vector<std::pair<std::vector<T>,boost::optional<double>>> result;

    std::function<bool(const std::vector<T>&, const boost::optional<double>&)> cb = [ &result ] (const std::vector<T>& combo, const boost::optional<double>& score)
    {
        result.emplace_back(combo, score);
        return true;
    };

    cartesianProduct(input, cb, distinct, duplicate_check, score_check);

    return result;
}

}  // namespace ScoreBased
}  // namespace Combinatorial
}  // namespace Utils
