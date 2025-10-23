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
#include <queue>

namespace Utils
{
namespace Probability
{

template <typename T>
struct HypothesisT
{
    std::vector<T> hyp;
    double         probability;
};

template <typename T>
struct CompareProbababilityBigger
{
    bool operator()(const HypothesisT<T>& a, const HypothesisT<T>& b) const 
    {
        return a.probability > b.probability; 
    }
};

template <typename T>
class HypothesisQueue
{
public:
    typedef std::priority_queue<HypothesisT<T>, std::vector<HypothesisT<T>>, CompareProbababilityBigger<T>> Queue;

    HypothesisQueue(size_t k) : k_(k) {}
    virtual ~HypothesisQueue() = default;

    size_t size() const { return queue_.size(); }
    bool empty() const { return queue_.empty(); }

    void clear()
    {
        queue_     = Queue();
        is_full_   = false;
        threshold_ = 0.0;
    }

    double threshold() const
    {
        if (!is_full_)
            return 0.0;
        
        return threshold_;
    }

    bool isFull() const
    {
        return is_full_;
    }

    bool canAdd(const HypothesisT<T>& hyp) const
    {
        if (!is_full_)
            return true;

        return hyp.probability > threshold_;
    }

    bool add(const HypothesisT<T>& hyp)
    {
        //queue not full => just add
        if (!is_full_)
        {
            size_t n_before = queue_.size();
            queue_.push(hyp);
            
            if (n_before + 1 >= k_)
                is_full_ = true;
            if (n_before == 0 || hyp.probability < threshold_)
                threshold_ = hyp.probability;
            
            return true;
        }

        //not in current top k? => skip
        if (hyp.probability <= threshold_)
            return false;

        //replace in queue
        queue_.pop();
        queue_.push(hyp);

        threshold_ = queue_.top().probability;

        return true;
    }

    bool add(const std::vector<T>& hyp, double probability)
    {
        HypothesisT<T> h;
        h.hyp         = hyp;
        h.probability = probability;

        return add(std::move(h));
    };

    std::vector<HypothesisT<T>> popHypotheses()
    {
        std::vector<HypothesisT<T>> hyps;
        while (!queue_.empty())
        {
            hyps.push_back(queue_.top());
            queue_.pop();
        }
        return hyps;
    }

private:
    Queue  queue_;
    size_t k_;
    bool   is_full_ = false;
    double threshold_ = 0.0;
};

}  // namespace Probability
}  // namespace Utils
