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
#include <cstddef> 

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
        queue_ = Queue();
    }

    double threshold() const
    {
        if (!isFull())
            return 0.0;
        
        return queue_.top().probability;
    }

    bool isFull() const
    {
        return queue_.size() >= k_;
    }

    bool canAdd(const HypothesisT<T>& hyp) const
    {
        if (!isFull())
            return true;

        return hyp.probability > queue_.top().probability;
    }

    bool add(const HypothesisT<T>& hyp)
    {
        //queue not full => just add
        if (!isFull())
        {
            queue_.push(hyp);
            return true;
        }

        //not in current top k? => skip
        if (hyp.probability <= queue_.top().probability)
            return false;

        //replace in queue
        queue_.pop();
        queue_.push(hyp);

        return true;
    }

    bool add(const std::vector<T>& hyp, double probability)
    {
        HypothesisT<T> h;
        h.hyp         = hyp;
        h.probability = probability;

        return add(h);
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
};

}  // namespace Probability
}  // namespace Utils
