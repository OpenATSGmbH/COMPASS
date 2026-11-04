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

#include "labelplacement_defs.h"

#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>
#include <unordered_set>
#include <cstdint>
#include <iterator>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <QRectF>
#include <QLineF>

namespace label_placement
{

//minimal displacement overlap removal with stable separation constraints (VPSC, solved exactly per axis):
//labels are placed as close as possible to their reference (home) positions subject to non-overlap
//constraints between labels, with anchors and with the region of interest. Each overlapping pair gets one
//separation constraint (left/right/above/below); its direction is taken over from the previous placement
//(the labels' initial positions) unless that is much more expensive than the cheapest direction. Optionally
//the distance from the previous placement is weighted in as well (cs_weight_prev).
//=> decluttered (hard constraints), local (minimal displacement, the previous placement only as a weighted
//term, no drift), smooth (the solution is continuous in the inputs as long as the constraint set does not
//change) and regular (pairs keep their sides between runs).
namespace constraint
{
    enum Dir
    {
        None = -1,
        Left,  //first left of second
        Right, //second left of first
        Up,    //first above second
        Down   //second above first
    };

    /**
     * Runtime limit of a placement run, checked between interruptible steps.
     */
    class Deadline
    {
    public:
        explicit Deadline(double max_ms)
        :   start_ (Clock::now())
        ,   max_ms_(max_ms)
        {
        }

        bool active() const { return max_ms_ > 0.0; }
        bool expired() const { return active() && elapsedMS() >= max_ms_; }

        //true if a step of the given duration would not end before the limit
        bool exceededBy(double step_ms) const { return active() && elapsedMS() + step_ms >= max_ms_; }

        double elapsedMS() const
        {
            return std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
        }

    private:
        typedef std::chrono::steady_clock Clock;

        Clock::time_point start_;
        double            max_ms_ = 0.0; // 0 = unlimited
    };

    /**
     * Separation constraint on one axis: v[ right ] - v[ left ] >= gap.
     */
    struct Separation
    {
        int    left  = -1;
        int    right = -1;
        double gap   = 0.0;
    };

    /**
     * Exact solver of one axis (VPSC, Dwyer, Marriott, Stuckey: "Fast node overlap removal", 2006): minimizes
     * sum weight * (v - desired)^2 subject to separation constraints without cycles.
     * - variables are merged into blocks connected by active (tight) constraints, a block sits at the weighted
     *   mean of its variables' desired positions (minus their offsets in the block)
     * - satisfy(): variables in topological order, each block is merged with its most violated incoming
     *   constraint until none is violated => feasible
     * - refine(): a block is split at an active constraint with a negative Lagrange multiplier (the two parts
     *   want to move apart), the parts are merged again where violated, until no such constraint is left => optimal
     */
    class AxisSolver
    {
    public:
        AxisSolver(const std::vector<double>& desired,
                   const std::vector<double>& weights,
                   const std::vector<Separation>& constraints)
        {
            const size_t n = desired.size();

            vars_.resize(n);
            for (size_t i = 0; i < n; ++i)
            {
                vars_[ i ].desired = desired[ i ];
                vars_[ i ].weight  = weights[ i ];
            }

            cons_.reserve(constraints.size());
            for (const auto& c : constraints)
            {
                vars_[ c.left  ].out.push_back((int)cons_.size());
                vars_[ c.right ].in.push_back ((int)cons_.size());

                Con con;
                con.left  = c.left;
                con.right = c.right;
                con.gap   = c.gap;

                cons_.push_back(con);
            }

            blocks_.reserve(2 * n);
            for (size_t i = 0; i < n; ++i)
                addToBlock(newBlock(), (int)i);

            mark_.assign(n, 0);
        }

        enum class Status
        {
            Optimal,    // optimal and feasible
            NotOptimal, // cyclic constraints, split limit (positions set, possibly infeasible)
            Feasible,   // stopped by the deadline while optimizing: feasible, not optimal (positions set)
            Expired     // stopped by the deadline before feasible (positions not set)
        };

        /**
         * Solves, checking the given deadline between the merge and split steps.
         */
        Status solve(std::vector<double>& v, const Deadline* deadline = nullptr)
        {
            deadline_ = deadline;
            expired_  = false;

            const bool satisfied = satisfy();
            if (expired_)
                return Status::Expired;

            const bool optimal = satisfied && refine();

            v.resize(vars_.size());
            for (size_t i = 0; i < vars_.size(); ++i)
                v[ i ] = position((int)i);

            bool feasible = true;
            for (size_t c = 0; c < cons_.size(); ++c)
                if (violation((int)c) > FeasibilityTolerance)
                    feasible = false;

            if (expired_)
                return feasible ? Status::Feasible : Status::NotOptimal;

            return optimal && feasible ? Status::Optimal : Status::NotOptimal;
        }

        static constexpr double FeasibilityTolerance = 1e-6;  // max violation of a satisfied constraint
        static constexpr double ViolationTolerance   = 1e-9;  // violations below are not merged
        static constexpr double LagrangeTolerance    = -1e-6; // multipliers below split a block

    private:
        struct Var
        {
            double           desired = 0.0;
            double           weight  = 1.0;
            double           offset  = 0.0; // position relative to the block position
            int              block   = -1;
            std::vector<int> in;            // constraints with this variable on the right
            std::vector<int> out;           // constraints with this variable on the left
        };

        struct Con
        {
            int    left   = -1;
            int    right  = -1;
            double gap    = 0.0;
            bool   active = false; // tight, connects two variables of a block
            double lm     = 0.0;   // Lagrange multiplier (active constraints only)
        };

        struct Block
        {
            std::vector<int> vars;
            double           wposn  = 0.0; // sum weight * (desired - offset)
            double           weight = 0.0;
            double           posn   = 0.0;
            bool             alive  = true;
        };

        double position(int v) const { return blocks_[ vars_[ v ].block ].posn + vars_[ v ].offset; }
        double violation(int c) const { return position(cons_[ c ].left) + cons_[ c ].gap - position(cons_[ c ].right); }
        double dfdv(int v) const { return 2.0 * vars_[ v ].weight * (position(v) - vars_[ v ].desired); }

        int newBlock()
        {
            blocks_.emplace_back();
            return (int)blocks_.size() - 1;
        }

        void addToBlock(int b, int v)
        {
            auto& block = blocks_[ b ];
            auto& var   = vars_[ v ];

            var.block = b;
            block.vars.push_back(v);

            block.weight += var.weight;
            block.wposn  += var.weight * (var.desired - var.offset);
            block.posn    = block.wposn / block.weight;
        }

        //block at its optimal position (weighted mean of the desired positions)
        void resetPosition(int b)
        {
            auto& block = blocks_[ b ];

            block.wposn = 0.0;
            for (int v : block.vars)
                block.wposn += vars_[ v ].weight * (vars_[ v ].desired - vars_[ v ].offset);

            block.posn = block.wposn / block.weight;
        }

        //merges the blocks of c (smaller into larger), c becomes tight and active
        int merge(int c)
        {
            auto& con = cons_[ c ];

            int bl = vars_[ con.left  ].block;
            int br = vars_[ con.right ].block;

            //offset of the right block's variables in the left block so that c is tight
            const double dist = vars_[ con.left ].offset + con.gap - vars_[ con.right ].offset;

            int target = bl;
            int source = br;
            double shift = dist;

            if (blocks_[ br ].vars.size() > blocks_[ bl ].vars.size())
            {
                target = br;
                source = bl;
                shift  = -dist;
            }

            const std::vector<int> moved = blocks_[ source ].vars;

            blocks_[ source ].alive = false;
            blocks_[ source ].vars.clear();

            for (int v : moved)
            {
                vars_[ v ].offset += shift;
                addToBlock(target, v);
            }

            con.active = true;

            return target;
        }

        //most violated constraint into (incoming = true) or out of the block, -1 if none is violated
        int mostViolated(int b, bool incoming) const
        {
            int    best     = -1;
            double best_vio = ViolationTolerance;

            for (int v : blocks_[ b ].vars)
            {
                for (int c : incoming ? vars_[ v ].in : vars_[ v ].out)
                {
                    const int other = incoming ? cons_[ c ].left : cons_[ c ].right;
                    if (vars_[ other ].block == b)
                        continue;

                    const double vio = violation(c);
                    if (vio > best_vio)
                    {
                        best     = c;
                        best_vio = vio;
                    }
                }
            }

            return best;
        }

        int mergeLeft(int b)
        {
            int c;
            while ((c = mostViolated(b, true)) >= 0)
                b = merge(c);

            return b;
        }

        int mergeRight(int b)
        {
            int c;
            while ((c = mostViolated(b, false)) >= 0)
                b = merge(c);

            return b;
        }

        bool satisfy()
        {
            //topological order of the variables (Kahn)
            const size_t n = vars_.size();

            std::vector<int> num_in(n, 0);
            for (const auto& c : cons_)
                ++num_in[ c.right ];

            std::vector<int> order;
            order.reserve(n);
            for (size_t i = 0; i < n; ++i)
                if (num_in[ i ] == 0)
                    order.push_back((int)i);

            for (size_t k = 0; k < order.size(); ++k)
                for (int c : vars_[ order[ k ] ].out)
                    if (--num_in[ cons_[ c ].right ] == 0)
                        order.push_back(cons_[ c ].right);

            if (order.size() != n)
                return false; // cycle

            for (int v : order)
            {
                if (deadline_ && deadline_->expired())
                {
                    expired_ = true;
                    return false;
                }

                mergeLeft(vars_[ v ].block);
            }

            return true;
        }

        //Lagrange multipliers of the active constraints of the subtree at v (entered via constraint from_c),
        //returns the derivative of the subtree, keeps the constraint with the smallest multiplier
        double computeLM(int v, int from_c, int& min_c)
        {
            double d = dfdv(v);

            for (int c : vars_[ v ].out)
            {
                auto& con = cons_[ c ];
                if (!con.active || c == from_c)
                    continue;

                con.lm = computeLM(con.right, c, min_c);
                d     += con.lm;

                if (min_c < 0 || con.lm < cons_[ min_c ].lm)
                    min_c = c;
            }
            for (int c : vars_[ v ].in)
            {
                auto& con = cons_[ c ];
                if (!con.active || c == from_c)
                    continue;

                con.lm = -computeLM(con.left, c, min_c);
                d     -= con.lm;

                if (min_c < 0 || con.lm < cons_[ min_c ].lm)
                    min_c = c;
            }

            return d;
        }

        //splits block b at its active constraint c into the part left of c and the part right of c
        void split(int b, int c)
        {
            auto& con = cons_[ c ];
            con.active = false;

            //left part: variables reachable from the left variable via active constraints (marked)
            ++mark_gen_;
            auto inLeft = [ & ] (int v) { return mark_[ v ] == mark_gen_; };

            std::vector<int> stack { con.left };

            mark_[ con.left ] = mark_gen_;

            while (!stack.empty())
            {
                const int v = stack.back();
                stack.pop_back();

                auto visit = [ & ] (int c2, int other)
                {
                    if (cons_[ c2 ].active && !inLeft(other))
                    {
                        mark_[ other ] = mark_gen_;
                        stack.push_back(other);
                    }
                };

                for (int c2 : vars_[ v ].out)
                    visit(c2, cons_[ c2 ].right);
                for (int c2 : vars_[ v ].in)
                    visit(c2, cons_[ c2 ].left);
            }

            const double           old_posn = blocks_[ b ].posn;
            const std::vector<int> old_vars = blocks_[ b ].vars;

            blocks_[ b ].alive = false;
            blocks_[ b ].vars.clear();

            const int bl = newBlock();
            const int br = newBlock();

            for (int v : old_vars)
                addToBlock(inLeft(v) ? bl : br, v);

            //the right part stays in place while the left part moves to its optimum and is merged leftwards
            blocks_[ br ].posn  = old_posn;
            blocks_[ br ].wposn = old_posn * blocks_[ br ].weight;

            mergeLeft(bl);

            //the right part (possibly merged meanwhile) moves to its optimum and is merged rightwards
            const int r = vars_[ con.right ].block;
            resetPosition(r);
            mergeRight(r);
        }

        bool refine()
        {
            const size_t max_splits = 10 * (vars_.size() + cons_.size()) + 100;

            size_t splits = 0;

            //blocks to check; a split only changes the blocks it ends up in (the multipliers of a block depend on
            //its own variables only), so only these are checked again
            std::vector<int> todo;
            todo.reserve(blocks_.size());
            for (size_t b = 0; b < blocks_.size(); ++b)
                todo.push_back((int)b);

            while (!todo.empty())
            {
                const int b = todo.back();
                todo.pop_back();

                if (!blocks_[ b ].alive || blocks_[ b ].vars.size() < 2)
                    continue;

                int min_c = -1;
                computeLM(blocks_[ b ].vars.front(), -1, min_c);

                if (min_c < 0 || cons_[ min_c ].lm >= LagrangeTolerance)
                    continue;

                if (++splits > max_splits)
                    return false;

                //stopped between splits => still feasible
                if (deadline_ && deadline_->expired())
                {
                    expired_ = true;
                    return false;
                }

                split(b, min_c);

                todo.push_back(vars_[ cons_[ min_c ].left  ].block);
                todo.push_back(vars_[ cons_[ min_c ].right ].block);
            }

            return true;
        }

        std::vector<Var>   vars_;
        std::vector<Con>   cons_;
        std::vector<Block> blocks_;

        std::vector<int> mark_;         // variable marks of split(), current mark = mark_gen_
        int              mark_gen_ = 0;

        const Deadline* deadline_ = nullptr; // runtime limit of the current solve (optional)
        bool            expired_  = false;   // the current solve was stopped by the deadline
    };

    /**
     * Variables and constraints of one axis. Hard constraints between labels, soft bounds (violable at the cost
     * weight * violation^2) for obstacles and the region of interest, so the problem stays feasible (ordering
     * constraints without cycles plus soft bounds). A soft bound is a dummy variable at the bound with the soft
     * weight, separated from the label by a zero gap (same cost as a slack variable).
     */
    struct Axis
    {
        Axis(const std::vector<double>& targets, double weight)
        :   desired(targets)
        ,   weights(targets.size(), weight)
        {
        }

        //v[ right ] - v[ left ] >= gap
        void addHard(int left, int right, double gap)
        {
            constraints.push_back({ left, right, gap });
        }

        //v[ a ] >= bound, soft
        void addSoftLower(int a, double bound, double weight)
        {
            constraints.push_back({ addDummy(bound, weight), a, 0.0 });
        }

        //v[ a ] <= bound, soft
        void addSoftUpper(int a, double bound, double weight)
        {
            constraints.push_back({ a, addDummy(bound, weight), 0.0 });
        }

        /**
         * Label positions in v, solved per connected component of the constraints (independent problems => same
         * optimum as one solve), smallest first. Marks the labels of the solved components in solved (optional).
         * When the deadline expires, the components solved so far keep their new positions, the others are left
         * untouched (Expired; Feasible if the last component was stopped while optimizing, its positions are set).
         */
        AxisSolver::Status solve(std::vector<double>& v,
                                 const Deadline* deadline = nullptr,
                                 std::vector<char>* solved = nullptr) const
        {
            typedef AxisSolver::Status Status;

            const size_t nv = desired.size();
            const size_t nl = v.size(); // labels, dummies follow

            //components (union-find over the constraints)
            std::vector<int> root(nv);
            std::iota(root.begin(), root.end(), 0);

            auto find = [ & ] (int a)
            {
                while (root[ a ] != a)
                    a = root[ a ] = root[ root[ a ] ];
                return a;
            };

            for (const auto& c : constraints)
                root[ find(c.left) ] = find(c.right);

            std::vector<int> comp_index(nv, -1);
            std::vector<std::vector<int>> comps;

            for (size_t i = 0; i < nv; ++i)
            {
                const int r = find((int)i);
                if (comp_index[ r ] < 0)
                {
                    comp_index[ r ] = (int)comps.size();
                    comps.emplace_back();
                }
                comps[ comp_index[ r ] ].push_back((int)i);
            }

            std::vector<std::vector<Separation>> comp_cons(comps.size());
            for (const auto& c : constraints)
                comp_cons[ comp_index[ find(c.left) ] ].push_back(c);

            //smallest first (stable => deterministic)
            std::vector<size_t> order(comps.size());
            std::iota(order.begin(), order.end(), 0);
            std::stable_sort(order.begin(), order.end(),
                             [ & ] (size_t a, size_t b) { return comps[ a ].size() < comps[ b ].size(); });

            Status status = Status::Optimal;

            std::vector<int>    local(nv, -1);
            std::vector<double> d, w, positions;
            std::vector<Separation> cons;

            for (size_t ci : order)
            {
                const auto& vars = comps[ ci ];

                //unconstrained variable => at its desired position
                if (vars.size() == 1)
                {
                    if ((size_t)vars[ 0 ] < nl)
                    {
                        v[ vars[ 0 ] ] = desired[ vars[ 0 ] ];
                        if (solved)
                            (*solved)[ vars[ 0 ] ] = 1;
                    }
                    continue;
                }

                d.clear();
                w.clear();
                for (size_t k = 0; k < vars.size(); ++k)
                {
                    local[ vars[ k ] ] = (int)k;
                    d.push_back(desired[ vars[ k ] ]);
                    w.push_back(weights[ vars[ k ] ]);
                }

                cons.clear();
                for (const auto& c : comp_cons[ ci ])
                    cons.push_back({ local[ c.left ], local[ c.right ], c.gap });

                AxisSolver solver(d, w, cons);
                const Status s = solver.solve(positions, deadline);

                if (s == Status::Expired)
                    return Status::Expired;

                for (size_t k = 0; k < vars.size(); ++k)
                {
                    if ((size_t)vars[ k ] >= nl)
                        continue;

                    v[ vars[ k ] ] = positions[ k ];
                    if (solved)
                        (*solved)[ vars[ k ] ] = 1;
                }

                if (s == Status::Feasible)
                    return Status::Feasible;
                if (s == Status::NotOptimal)
                    status = Status::NotOptimal;
            }

            return status;
        }

        bool empty() const { return constraints.empty(); }

        std::vector<double>     desired;
        std::vector<double>     weights;
        std::vector<Separation> constraints;

    private:
        int addDummy(double position, double weight)
        {
            desired.push_back(position);
            weights.push_back(weight);

            return (int)desired.size() - 1;
        }
    };

    /**
     * Directed graph of the ordering constraints of one axis, rejects edges closing a cycle (infeasible) and detects
     * implied ones: a path from a to b implies 'a before b' with the needed separation (its first edge already
     * separates by the size of a plus the gap, all further edges by positive amounts) => no constraint needed, the
     * solution is unchanged (same feasible set).
     */
    class OrderGraph
    {
    public:
        explicit OrderGraph(size_t n) : edges_(n), visited_(n, 0) {}

        bool reaches(int from, int to) const
        {
            ++visit_gen_;

            stack_.clear();
            stack_.push_back(from);

            while (!stack_.empty())
            {
                int v = stack_.back();
                stack_.pop_back();

                if (v == to)
                    return true;
                if (visited_[ v ] == visit_gen_)
                    continue;

                visited_[ v ] = visit_gen_;

                for (int w : edges_[ v ])
                    stack_.push_back(w);
            }

            return false;
        }

        enum class AddResult
        {
            Added,   // new ordering
            Implied, // 'from' already reaches 'to' (not added)
            Cycle    // 'to' already reaches 'from' (infeasible, not added)
        };

        //adds 'from' before 'to'
        AddResult add(int from, int to)
        {
            if (reaches(to, from))
                return AddResult::Cycle;
            if (reaches(from, to))
                return AddResult::Implied;

            edges_[ from ].push_back(to);
            return AddResult::Added;
        }

    private:
        std::vector<std::vector<int>> edges_;

        mutable std::vector<int> visited_;       // visit marks of reaches(), current mark = visit_gen_
        mutable std::vector<int> stack_;
        mutable int              visit_gen_ = 0;
    };

    /**
     * Spatial index of boxes for neighbourhood queries (bulk loaded r-tree), replaces tests of all pairs.
     * - a query returns the boxes intersecting the query box including touching ones, a superset of the boxes
     *   overlapping it (exact together with the caller's overlap test)
     * - results in ascending order => same order as a loop over all boxes (the placement depends on the order)
     */
    class BoxIndex
    {
    public:
        /**
         * use_tree: r-tree, otherwise a test of all boxes per query (same results, for comparisons).
         */
        explicit BoxIndex(bool use_tree = true) : use_tree_(use_tree) {}

        /**
         * Builds the index from the boxes, boxes flagged in skip (optional) are left out.
         */
        void build(const std::vector<QRectF>& boxes,
                   const std::vector<char>* skip = nullptr)
        {
            std::vector<Value> values;
            values.reserve(boxes.size());

            for (size_t i = 0; i < boxes.size(); ++i)
            {
                if (skip && (*skip)[ i ])
                    continue;

                values.emplace_back(toBox(boxes[ i ]), (int)i);
            }

            if (use_tree_)
                tree_ = Tree(values.begin(), values.end()); // packing algorithm
            else
                values_ = std::move(values);
        }

        /**
         * Indices of the boxes intersecting or touching r, ascending if sorted (otherwise in tree order; always
         * ascending without the tree).
         */
        void query(const QRectF& r,
                   std::vector<int>& result,
                   bool sorted = true) const
        {
            result.clear();

            if (!use_tree_)
            {
                const Box q = toBox(r);
                for (const auto& v : values_)
                    if (boost::geometry::intersects(v.first, q))
                        result.push_back(v.second);
                return;
            }

            hits_.clear();
            tree_.query(boost::geometry::index::intersects(toBox(r)), std::back_inserter(hits_));

            result.reserve(hits_.size());
            for (const auto& hit : hits_)
                result.push_back(hit.second);

            if (sorted)
                std::sort(result.begin(), result.end());
        }

    private:
        typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> Point;
        typedef boost::geometry::model::box<Point>                                         Box;
        typedef std::pair<Box, int>                                                        Value;
        typedef boost::geometry::index::rtree<Value, boost::geometry::index::quadratic<16>> Tree;

        static Box toBox(const QRectF& r)
        {
            return Box(Point(r.left(), r.top()), Point(r.right(), r.bottom()));
        }

        bool use_tree_ = true;

        Tree                       tree_;
        std::vector<Value>         values_; // boxes without the tree
        mutable std::vector<Value> hits_;   // query buffer
    };

    /**
     * Needed separation per direction for two boxes (x, y = top left), gaps included.
     */
    inline void separationCosts(double xi, double yi, double wi, double hi,
                                double xj, double yj, double wj, double hj,
                                double gap_x,
                                double gap_y,
                                double* cost)
    {
        cost[ Left  ] = xi + wi + gap_x - xj;
        cost[ Right ] = xj + wj + gap_x - xi;
        cost[ Up    ] = yi + hi + gap_y - yj;
        cost[ Down  ] = yj + hj + gap_y - yi;
    }

    /**
     * Candidate directions in order of preference: the previous one first if separated before and not too
     * expensive now, then by needed separation (ties in a fixed order => deterministic). The needed separation is
     * weighted by the size of the pair along the axis (cs_balance), so wide labels are not always stacked vertically.
     */
    inline std::vector<int> directionCandidates(const double* cost,
                                                const double* prev_cost,
                                                double min_w,
                                                double min_h,
                                                double size_x,
                                                double size_y,
                                                const Settings& settings)
    {
        const double norm_x = std::pow(std::max(size_x, 1.0), settings.cs_balance);
        const double norm_y = std::pow(std::max(size_y, 1.0), settings.cs_balance);

        auto norm  = [ & ] (int d) { return d == Left || d == Right ? norm_x : norm_y; };
        auto score = [ & ] (int d) { return cost[ d ] / norm(d); };

        std::vector<int> dirs { Left, Right, Up, Down };

        std::stable_sort(dirs.begin(), dirs.end(), [ & ] (int a, int b) { return score(a) < score(b); });

        //previous separation: the satisfied direction with the largest slack, within a tolerance relative to the
        //label size (minimal displacement solutions are tight, labels move relative to each other between runs)
        int    prev_dir   = None;
        double prev_slack = 0.0;

        for (int d = Left; d <= Down; ++d)
        {
            const double tolerance = settings.cs_prev_tolerance * (d == Left || d == Right ? min_w : min_h);

            double slack = -prev_cost[ d ];
            if (slack >= -tolerance && (prev_dir == None || slack > prev_slack))
            {
                prev_dir   = d;
                prev_slack = slack;
            }
        }

        if (prev_dir != None &&
            score(prev_dir) <= settings.cs_keep_factor * score(dirs.front()) + settings.cs_keep_abs / norm(prev_dir))
        {
            dirs.erase(std::find(dirs.begin(), dirs.end(), prev_dir));
            dirs.insert(dirs.begin(), prev_dir);
        }

        return dirs;
    }

    /**
     * Overlap area of two boxes.
     */
    inline double overlapArea(const QRectF& r0, const QRectF& r1)
    {
        const double ox = std::min(r0.right(),  r1.right())  - std::max(r0.left(), r1.left());
        const double oy = std::min(r0.bottom(), r1.bottom()) - std::max(r0.top(),  r1.top());

        return ox > 0 && oy > 0 ? ox * oy : 0.0;
    }

    /**
     * Distance between a point and a box (0 if inside).
     */
    inline double pointBoxDistance(double x, double y, const QRectF& r)
    {
        const double dx = std::max({ r.left() - x, 0.0, x - r.right()  });
        const double dy = std::max({ r.top()  - y, 0.0, y - r.bottom() });

        return std::hypot(dx, dy);
    }

    /**
     * Candidate box of a label in direction (sx, sy) from the anchor (each -1, 0, 1; screen coordinates, y down): the
     * box is placed so that its nearest point to the anchor lies at the given distance in that direction.
     */
    inline QRectF candidateBox(const Label& l, int sx, int sy, double dist)
    {
        const double norm = std::hypot((double)sx, (double)sy);
        const double ox   = dist * sx / norm;
        const double oy   = dist * sy / norm;

        const double x = l.x_anchor + ox + (sx > 0 ? 0.0 : (sx < 0 ? -l.w : -l.w / 2));
        const double y = l.y_anchor + oy + (sy > 0 ? 0.0 : (sy < 0 ? -l.h : -l.h / 2));

        return QRectF(x, y, l.w, l.h);
    }

    /**
     * Chooses a candidate position per label among positions around its anchor (4: diagonal, 8: diagonal and
     * axis-aligned), at the anchor distance of its reference position, independent of the reference direction.
     * Minimizes the overlap with the other labels, the obstacles and the area outside the region of interest;
     * leaving the candidate of the previous placement costs cs_cand_switch, leaving the reference position
     * cs_cand_penalty (fractions of the label area). Labels without a previous placement choose freely (ties =>
     * reference position). Deterministic sweeps over the labels until no label changes (or the deadline expired,
     * the current choice is kept). Replaces hx/hy by the chosen positions, flags labels which keep the candidate of
     * their previous placement. Excluded labels (optional) are left out (they keep their reference position).
     */
    inline void chooseCandidates(const std::vector<Label>& labels,
                                 const std::vector<QRectF>& obstacles,
                                 const Settings& settings,
                                 std::vector<double>& hx,
                                 std::vector<double>& hy,
                                 std::vector<char>& keeps_prev,
                                 int* side_changes = nullptr,
                                 const Deadline* deadline = nullptr,
                                 const std::vector<char>* excluded = nullptr)
    {
        auto isExcluded = [ & ] (size_t i) { return excluded && (*excluded)[ i ]; };

        const size_t n = labels.size();

        keeps_prev.assign(n, 0);
        for (size_t i = 0; i < n; ++i)
            keeps_prev[ i ] = labels[ i ].has_last ? 1 : 0;

        if (side_changes)
            *side_changes = 0;

        if (settings.cs_candidates < 4)
            return;

        //directions: diagonals first, then axis-aligned
        static const int Dirs[ 8 ][ 2 ] = { {  1, -1 }, { -1, -1 }, {  1,  1 }, { -1,  1 },
                                            {  1,  0 }, { -1,  0 }, {  0, -1 }, {  0,  1 } };

        const int nc = settings.cs_candidates >= 8 ? 8 : 4;

        std::vector<QRectF> cands(n * nc);
        std::vector<int>    ref(n, 0), prev(n, -1), cur(n, 0);
        std::vector<QRectF> reach(n); // union of all candidate boxes

        auto nearest = [ & ] (size_t i, const QRectF& box)
        {
            int    best   = 0;
            double best_d = std::numeric_limits<double>::max();

            for (int k = 0; k < nc; ++k)
            {
                const double d = QLineF(cands[ i * nc + k ].center(), box.center()).length();
                if (d < best_d)
                {
                    best   = k;
                    best_d = d;
                }
            }
            return best;
        };

        for (size_t i = 0; i < n; ++i)
        {
            const auto& l = labels[ i ];

            const QRectF ref_box(hx[ i ], hy[ i ], l.w, l.h);
            const double dist = pointBoxDistance(l.x_anchor, l.y_anchor, ref_box);

            for (int k = 0; k < nc; ++k)
                cands[ i * nc + k ] = candidateBox(l, Dirs[ k ][ 0 ], Dirs[ k ][ 1 ], dist);

            ref [ i ] = nearest(i, ref_box);
            prev[ i ] = l.has_last ? nearest(i, QRectF(l.x_last, l.y_last, l.w, l.h)) : -1;
            cur [ i ] = prev[ i ] >= 0 ? prev[ i ] : ref[ i ];

            reach[ i ] = cands[ i * nc ];
            for (int k = 1; k < nc; ++k)
                reach[ i ] |= cands[ i * nc + k ];
        }

        //neighbours which can interact (ascending)
        BoxIndex reach_index(settings.cs_spatial_index), obstacle_index(settings.cs_spatial_index);
        reach_index.build(reach, excluded);
        obstacle_index.build(obstacles);

        std::vector<std::vector<int>> neighbours(n), near_obstacles(n);
        std::vector<int>              near;

        for (size_t i = 0; i < n; ++i)
        {
            if (isExcluded(i))
                continue;

            reach_index.query(reach[ i ], near);
            for (int j : near)
                if ((size_t)j != i && reach[ i ].intersects(reach[ j ]))
                    neighbours[ i ].push_back(j);

            obstacle_index.query(reach[ i ], near);
            for (int o : near)
                if (reach[ i ].intersects(obstacles[ o ]))
                    near_obstacles[ i ].push_back(o);
        }

        const bool   use_roi = settings.cs_keep_in_roi && !settings.roi.isEmpty();
        const double eps     = 1e-9;

        auto cost = [ & ] (size_t i, int k)
        {
            const QRectF& box  = cands[ i * nc + k ];
            const double  area = box.width() * box.height();

            double c = 0.0;
            if (k != ref[ i ])
                c += area * settings.cs_cand_penalty;
            if (prev[ i ] >= 0 && k != prev[ i ])
                c += area * settings.cs_cand_switch;

            for (int j : neighbours[ i ])
                c += overlapArea(box, cands[ j * nc + cur[ j ] ]);
            for (int o : near_obstacles[ i ])
                c += overlapArea(box, obstacles[ o ]);
            if (use_roi)
                c += area - overlapArea(box, settings.roi);

            return c;
        };

        bool expired = false;

        for (int sweep = 0; sweep < settings.cs_cand_sweeps && !expired; ++sweep)
        {
            bool changed = false;

            for (size_t i = 0; i < n; ++i)
            {
                if (deadline && deadline->expired())
                {
                    expired = true;
                    break;
                }

                if (isExcluded(i))
                    continue;

                int    best      = cur[ i ];
                double best_cost = cost(i, cur[ i ]);

                for (int k = 0; k < nc; ++k)
                {
                    if (k == cur[ i ])
                        continue;

                    const double c = cost(i, k);
                    if (c < best_cost - eps)
                    {
                        best      = k;
                        best_cost = c;
                    }
                }

                if (best != cur[ i ])
                {
                    cur[ i ] = best;
                    changed  = true;
                }
            }

            if (!changed)
                break;
        }

        for (size_t i = 0; i < n; ++i)
        {
            if (isExcluded(i))
                continue;

            const QRectF& box = cands[ i * nc + cur[ i ] ];

            hx[ i ] = box.left();
            hy[ i ] = box.top();

            keeps_prev[ i ] = prev[ i ] >= 0 && cur[ i ] == prev[ i ] ? 1 : 0;

            if (side_changes && prev[ i ] >= 0 && cur[ i ] != prev[ i ])
                ++(*side_changes);
        }
    }

    /**
     * Density per label of its group: groups are labels with overlapping home boxes (gaps included, transitively),
     * the density is their label area (gaps included) relative to the area the group can use: the union of the
     * regions of their candidate positions (box around each anchor at the anchor distance of its home in all
     * directions), clipped to clip if given (e.g. the region of interest). The union is measured on a raster (cells
     * counted by their centers, cell size half the mean label height). A label without overlaps forms a group of its
     * own. spatial_index: r-tree for the overlaps, otherwise all pairs (same result).
     */
    inline std::vector<double> groupDensities(const std::vector<Label>& labels,
                                              const std::vector<double>& hx,
                                              const std::vector<double>& hy,
                                              double gap_x,
                                              double gap_y,
                                              const QRectF& clip = QRectF(),
                                              bool spatial_index = true)
    {
        const size_t MaxRasterCells = 1 << 20;

        const size_t n = labels.size();

        std::vector<int> root(n);
        std::iota(root.begin(), root.end(), 0);

        auto find = [ & ] (int a)
        {
            while (root[ a ] != a)
                a = root[ a ] = root[ root[ a ] ];
            return a;
        };

        std::vector<QRectF> homes(n);
        for (size_t i = 0; i < n; ++i)
            homes[ i ] = QRectF(hx[ i ], hy[ i ], labels[ i ].w, labels[ i ].h);

        BoxIndex index(spatial_index);
        index.build(homes);

        std::vector<int> near;

        for (size_t i = 0; i < n; ++i)
        {
            const auto& li = labels[ i ];

            index.query(homes[ i ].adjusted(-gap_x, -gap_y, gap_x, gap_y), near, false); // groups independent of the order

            for (int j : near)
            {
                const auto& lj = labels[ j ];

                if ((size_t)j > i &&
                    hx[ i ] < hx[ j ] + lj.w + gap_x && hx[ j ] < hx[ i ] + li.w + gap_x &&
                    hy[ i ] < hy[ j ] + lj.h + gap_y && hy[ j ] < hy[ i ] + li.h + gap_y)
                    root[ find((int)i) ] = find(j);
            }
        }

        //label area per group, regions of the candidate positions
        std::vector<int>    group(n);
        std::vector<double> area(n, 0.0);
        std::vector<QRectF> reach(n);

        QRectF bounds;
        double sum_h = 0.0;

        for (size_t i = 0; i < n; ++i)
        {
            const auto&  l    = labels[ i ];
            const double dist = pointBoxDistance(l.x_anchor, l.y_anchor, QRectF(hx[ i ], hy[ i ], l.w, l.h));

            reach[ i ] = QRectF(l.x_anchor - dist - l.w, l.y_anchor - dist - l.h, 2 * (dist + l.w), 2 * (dist + l.h));
            if (!clip.isEmpty())
                reach[ i ] = reach[ i ].intersected(clip);

            if (!reach[ i ].isEmpty())
                bounds = bounds.isNull() ? reach[ i ] : bounds.united(reach[ i ]);

            group[ i ] = find((int)i);

            area[ group[ i ] ] += (l.w + gap_x) * (l.h + gap_y);
            sum_h              += l.h;
        }

        if (bounds.isEmpty())
            return std::vector<double>(n, 0.0);

        //raster over all regions, enlarged cells if too many
        double cell = std::max(0.5 * sum_h / n, 1.0);

        const double num_cells = (bounds.width() / cell + 1) * (bounds.height() / cell + 1);
        if (num_cells > MaxRasterCells)
            cell *= std::sqrt(num_cells / MaxRasterCells);

        const int nx = (int)std::ceil(bounds.width()  / cell);
        const int ny = (int)std::ceil(bounds.height() / cell);

        //covered cells per group: groups one after another, a cell is counted once per group (stamped)
        std::vector<int> order(n);
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [ & ] (int a, int b) { return group[ a ] < group[ b ]; });

        std::vector<int>    stamp((size_t)nx * ny, -1);
        std::vector<size_t> covered(n, 0);

        for (int i : order)
        {
            const QRectF& r = reach[ i ];
            if (r.isEmpty())
                continue;

            //cells with their center inside the region
            const int ix0 = std::max((int)std::ceil ((r.left()   - bounds.left()) / cell - 0.5), 0);
            const int ix1 = std::min((int)std::floor((r.right()  - bounds.left()) / cell - 0.5), nx - 1);
            const int iy0 = std::max((int)std::ceil ((r.top()    - bounds.top())  / cell - 0.5), 0);
            const int iy1 = std::min((int)std::floor((r.bottom() - bounds.top())  / cell - 0.5), ny - 1);

            for (int iy = iy0; iy <= iy1; ++iy)
            {
                for (int ix = ix0; ix <= ix1; ++ix)
                {
                    auto& s = stamp[ (size_t)iy * nx + ix ];
                    if (s == group[ i ])
                        continue;

                    s = group[ i ];
                    ++covered[ group[ i ] ];
                }
            }
        }

        std::vector<double> density(n, 0.0);
        for (size_t i = 0; i < n; ++i)
        {
            const double a = covered[ group[ i ] ] * cell * cell;

            density[ i ] = a > 0.0 ? area[ group[ i ] ] / a : 0.0;
        }

        return density;
    }

    /**
     * Summed area table of the area of boxes on a raster: area within any rectangle in O(1).
     * - cell values = overlap areas of the boxes with the cell (exact), spread evenly within the cell
     * - the integral of such a raster is bilinear within each cell => interpolated exactly between the cell corners,
     *   continuous in the rectangle (no jumps at cell borders)
     * - the raster covers the given bounds, nothing outside counts; enlarged cells if more than MaxCells
     */
    class AreaTable
    {
    public:
        static constexpr size_t MaxCells = 1 << 20;

        void build(const std::vector<QRectF>& boxes,
                   const QRectF& bounds,
                   double cell)
        {
            x0_   = bounds.left();
            y0_   = bounds.top();
            cell_ = std::max(cell, 1.0);

            const double num_cells = (bounds.width() / cell_ + 1) * (bounds.height() / cell_ + 1);
            if (num_cells > MaxCells)
                cell_ *= std::sqrt(num_cells / MaxCells);

            nx_ = std::max((int)std::ceil(bounds.width()  / cell_), 1);
            ny_ = std::max((int)std::ceil(bounds.height() / cell_), 1);

            //cell values
            std::vector<double> cells((size_t)nx_ * ny_, 0.0);

            for (const auto& b : boxes)
            {
                const int ix0 = std::max((int)std::floor((b.left()   - x0_) / cell_), 0);
                const int ix1 = std::min((int)std::floor((b.right()  - x0_) / cell_), nx_ - 1);
                const int iy0 = std::max((int)std::floor((b.top()    - y0_) / cell_), 0);
                const int iy1 = std::min((int)std::floor((b.bottom() - y0_) / cell_), ny_ - 1);

                for (int iy = iy0; iy <= iy1; ++iy)
                {
                    for (int ix = ix0; ix <= ix1; ++ix)
                    {
                        const QRectF c(x0_ + ix * cell_, y0_ + iy * cell_, cell_, cell_);
                        cells[ (size_t)iy * nx_ + ix ] += overlapArea(b, c);
                    }
                }
            }

            //sums of the cells above and left of each cell corner
            sums_.assign((size_t)(nx_ + 1) * (ny_ + 1), 0.0);

            for (int iy = 0; iy < ny_; ++iy)
                for (int ix = 0; ix < nx_; ++ix)
                    sums_[ corner(ix + 1, iy + 1) ] = cells[ (size_t)iy * nx_ + ix ] + sums_[ corner(ix, iy + 1) ]
                                                    + sums_[ corner(ix + 1, iy) ] - sums_[ corner(ix, iy) ];
        }

        /**
         * Area of the boxes within r.
         */
        double area(const QRectF& r) const
        {
            if (sums_.empty())
                return 0.0;

            return integral(r.right(), r.bottom()) - integral(r.left(), r.bottom())
                 - integral(r.right(), r.top())    + integral(r.left(), r.top());
        }

    private:
        size_t corner(int ix, int iy) const { return (size_t)iy * (nx_ + 1) + ix; }

        //area above and left of (x, y), bilinear between the cell corners
        double integral(double x, double y) const
        {
            const double u = std::min(std::max((x - x0_) / cell_, 0.0), (double)nx_);
            const double v = std::min(std::max((y - y0_) / cell_, 0.0), (double)ny_);

            const int    ix = std::min((int)u, nx_ - 1);
            const int    iy = std::min((int)v, ny_ - 1);
            const double fu = u - ix;
            const double fv = v - iy;

            return (1 - fu) * (1 - fv) * sums_[ corner(ix,     iy)     ] + fu * (1 - fv) * sums_[ corner(ix + 1, iy)     ]
                 + (1 - fu) * fv       * sums_[ corner(ix,     iy + 1) ] + fu * fv       * sums_[ corner(ix + 1, iy + 1) ];
        }

        double x0_   = 0.0;
        double y0_   = 0.0;
        double cell_ = 1.0;
        int    nx_   = 0;
        int    ny_   = 0;

        std::vector<double> sums_; // (nx + 1) x (ny + 1) cell corners
    };

    /**
     * Crowding per label: whether the labels around it can be decluttered close to their anchors. Per scale (windows
     * around the anchor: the region of the label's candidate positions, i.e. anchor +- (anchor distance of its home +
     * label size), doubled scales - 1 times, clipped to clip if given), the label area within the window (home boxes
     * incl. gaps) relative to the window area, divided by the room gained if labels may be pushed out of the window
     * by up to reach label sizes (sqrt(w * h)): (1 + reach px / r)^2, r = half the side of a square of the window
     * area. Crowding = max over the scales, in units of a density at large scales: a small dense cluster with room
     * around it is not crowded, a large area of the same density is.
     */
    inline std::vector<double> localCrowding(const std::vector<Label>& labels,
                                             const std::vector<double>& hx,
                                             const std::vector<double>& hy,
                                             double gap_x,
                                             double gap_y,
                                             int scales,
                                             double reach,
                                             const QRectF& clip = QRectF())
    {
        const size_t n = labels.size();
        if (n == 0)
            return {};

        //home boxes incl. half the gap on each side => area (w + gap) * (h + gap)
        std::vector<QRectF> homes(n);

        QRectF bounds;
        double sum_h = 0.0;

        for (size_t i = 0; i < n; ++i)
        {
            homes[ i ] = QRectF(hx[ i ] - gap_x / 2, hy[ i ] - gap_y / 2, labels[ i ].w + gap_x, labels[ i ].h + gap_y);
            bounds     = i == 0 ? homes[ i ] : bounds.united(homes[ i ]);
            sum_h     += labels[ i ].h;
        }

        if (!clip.isEmpty())
            bounds = clip;

        AreaTable table;
        table.build(homes, bounds, 0.5 * sum_h / n);

        std::vector<double> crowding(n, 0.0);

        for (size_t i = 0; i < n; ++i)
        {
            const auto&  l        = labels[ i ];
            const double dist     = pointBoxDistance(l.x_anchor, l.y_anchor, QRectF(hx[ i ], hy[ i ], l.w, l.h));
            const double reach_px = reach * std::sqrt(l.w * l.h);

            double rx = dist + l.w;
            double ry = dist + l.h;

            for (int s = 0; s < scales; ++s, rx *= 2, ry *= 2)
            {
                QRectF win(l.x_anchor - rx, l.y_anchor - ry, 2 * rx, 2 * ry);
                if (!clip.isEmpty())
                    win = win.intersected(clip);

                const double win_area = win.width() * win.height();
                if (win_area <= 0.0)
                    continue;

                const double density = table.area(win) / win_area;
                const double room    = 1.0 + reach_px / (0.5 * std::sqrt(win_area));

                crowding[ i ] = std::max(crowding[ i ], density / (room * room));
            }
        }

        return crowding;
    }

    /**
     * Diagnostics of a placement run.
     */
    struct Stats
    {
        int    rounds            = 0;     // rounds of constraint generation
        bool   rounds_exhausted  = false; // new overlaps were still found in the last round
        bool   solver_converged  = true;  // all axis solves optimal and feasible
        int    unresolvable      = 0;     // overlapping pairs without an acyclic separation direction
        int    overlaps          = 0;     // overlapping label pairs left (without gap), at least one of them decluttered
        int    side_changes      = 0;     // labels placed on another side of their anchor than before (candidates)
        int    implied           = 0;     // separations implied by others (no constraint needed)
        int    separations       = 0;     // hard separation constraints between labels (both axes)
        int    obstacle_bounds   = 0;     // soft bounds of obstacles (anchors, additional objects)
        bool   time_exhausted    = false; // stopped by the runtime limit (cs_max_time_ms), last complete state returned
        double max_group_density = 0.0;   // highest group density (only computed if cs_max_group_density is set)
        double max_local_density = 0.0;   // highest local crowding (only computed if cs_adaptive is set)
        int    overcrowded       = 0;     // labels of overcrowded groups or crowded areas, not decluttered (at home)
    };

    /**
     * Places the labels, see namespace description. Reference position = target (falls back to the initial
     * position), initial position = previous placement (only used for the separation directions).
     * Runtime limit (cs_max_time_ms): checked in the candidate choice, between the rounds (a round is not started if
     * the last one would not fit anymore) and in the solver. The axes are solved per component of the constraints,
     * smallest first. When reached, the last complete state is returned: per label and axis the position of the last
     * completed solve of its component (feasible for its constraints, also if stopped while optimizing), or the
     * previous placement if its component was never solved.
     * Not decluttered (Label::excluded, at home): overcrowded groups (cs_max_group_density) or, adaptive
     * (cs_adaptive), labels in crowded areas (cs_max_local_density, hysteresis by Label::excluded_last), which are
     * fixed obstacles for the others.
     * Notice: Returns true unless an error occurred, also if overlaps remain (no feasible direction, too dense
     * for the region of interest, solver limits, runtime limit), see stats.
     */
    inline bool placeLabels(std::vector<Label>& labels,
                            const Settings& settings,
                            Stats* stats = nullptr)
    {
        Stats tmp_stats;
        Stats& st = stats ? *stats : tmp_stats;
        st = Stats();

        const Deadline deadline(settings.cs_max_time_ms);

        const size_t n = labels.size();
        if (n == 0)
            return true;

        const double gap_x = settings.cs_gap_x;
        const double gap_y = settings.cs_gap_y;

        std::vector<double> hx(n), hy(n), px(n), py(n), w(n), h(n);

        for (size_t i = 0; i < n; ++i)
        {
            const auto& l = labels[ i ];

            hx[ i ] = l.x_ref.has_value() ? l.x_ref.value() : l.x_last;
            hy[ i ] = l.y_ref.has_value() ? l.y_ref.value() : l.y_last;
            px[ i ] = l.x_last;
            py[ i ] = l.y_last;
            w [ i ] = l.w;
            h [ i ] = l.h;
        }

        //overcrowded groups (much more label area than space around their anchors, e.g. local pile-ups or a crowded
        //screen) are not decluttered: left out of the candidate choice and the constraints, the labels stay at home
        //(their anchors are still obstacles for the others)
        //adaptive: per label instead, labels in crowded areas stay at home and are fixed for the others, the rest is
        //decluttered (hysteresis: a label stays excluded until its area is clearly less crowded)
        const QRectF clip = settings.cs_keep_in_roi ? settings.roi : QRectF();

        std::vector<char> excluded(n, 0);
        if (settings.cs_adaptive)
        {
            const auto density = localCrowding(labels,
                                               hx,
                                               hy,
                                               gap_x,
                                               gap_y,
                                               settings.cs_local_scales,
                                               settings.cs_local_reach,
                                               clip);
            for (size_t i = 0; i < n; ++i)
            {
                st.max_local_density = std::max(st.max_local_density, density[ i ]);

                const double threshold = labels[ i ].excluded_last ? settings.cs_local_hysteresis * settings.cs_max_local_density
                                                                   : settings.cs_max_local_density;
                if (density[ i ] > threshold)
                {
                    excluded[ i ] = 1;
                    ++st.overcrowded;
                }
            }
        }
        else if (settings.cs_max_group_density > 0.0)
        {
            const auto density = groupDensities(labels, hx, hy, gap_x, gap_y, clip, settings.cs_spatial_index);

            for (size_t i = 0; i < n; ++i)
            {
                st.max_group_density = std::max(st.max_group_density, density[ i ]);

                if (density[ i ] > settings.cs_max_group_density)
                {
                    excluded[ i ] = 1;
                    ++st.overcrowded;
                }
            }
        }

        //fixed obstacles: anchors, then the additional objects, then the labels left at home (adaptive)
        std::vector<QRectF> obstacles;
        if (settings.cs_avoid_anchors && settings.cs_anchor_radius > 0)
        {
            const double r = settings.cs_anchor_radius;

            obstacles.reserve(n);
            for (const auto& l : labels)
                obstacles.emplace_back(l.x_anchor - r, l.y_anchor - r, 2 * r, 2 * r);
        }

        const size_t no_anchors = obstacles.size();

        if (settings.avoid_objects)
            obstacles.insert(obstacles.end(), settings.additional_objects.begin(), settings.additional_objects.end());

        if (settings.cs_adaptive)
        {
            for (size_t i = 0; i < n; ++i)
                if (excluded[ i ])
                    obstacles.emplace_back(hx[ i ], hy[ i ], w[ i ], h[ i ]);
        }

        const size_t no = obstacles.size();

        const double soft = settings.cs_soft_weight;

        //candidate positions: home on the side of the anchor with the least conflicts
        std::vector<char> keeps_prev;
        chooseCandidates(labels, obstacles, settings, hx, hy, keeps_prev, &st.side_changes, &deadline, &excluded);

        //labels without a valid previous placement (no history, changed sides): no previous separation sides, no
        //smoothness
        for (size_t i = 0; i < n; ++i)
        {
            if (keeps_prev[ i ])
                continue;

            px[ i ] = hx[ i ];
            py[ i ] = hy[ i ];
        }

        //targets: home, optionally blended with the previous placement (smoothness between runs, a label without
        //conflict returns home gradually):
        //  w_home * (v - home)^2 + w_prev * (v - prev)^2 = (w_home + w_prev) * (v - target)^2 + const
        const double w_prev = std::max(0.0, settings.cs_weight_prev);

        std::vector<double> tx(n), ty(n);
        for (size_t i = 0; i < n; ++i)
        {
            const double wp = keeps_prev[ i ] ? w_prev : 0.0;

            tx[ i ] = (hx[ i ] + wp * px[ i ]) / (1.0 + wp);
            ty[ i ] = (hy[ i ] + wp * py[ i ]) / (1.0 + wp);
        }

        Axis ax(tx, 1.0 + w_prev), ay(ty, 1.0 + w_prev);

        //region of interest as (soft) bounds
        if (settings.cs_keep_in_roi && !settings.roi.isEmpty())
        {
            const auto& roi = settings.roi;

            for (size_t i = 0; i < n; ++i)
            {
                if (excluded[ i ])
                    continue;

                ax.addSoftLower((int)i, roi.left(),            soft);
                ax.addSoftUpper((int)i, roi.right()  - w[ i ], soft);
                ay.addSoftLower((int)i, roi.top(),             soft);
                ay.addSoftUpper((int)i, roi.bottom() - h[ i ], soft);
            }
        }

        //label pairs and label/obstacle pairs with a separation (keys i * n + j with i < j, i * no + o)
        std::unordered_set<std::uint64_t> separated_pairs, bounded_obstacles;

        auto pairKey     = [ & ] (size_t i, size_t j) { return (std::uint64_t)i * n  + j; };
        auto obstacleKey = [ & ] (size_t i, size_t o) { return (std::uint64_t)i * no + o; };

        //candidates for overlaps: labels at their current positions, obstacles (fixed)
        BoxIndex label_index(settings.cs_spatial_index), obstacle_index(settings.cs_spatial_index);
        obstacle_index.build(obstacles);

        std::vector<QRectF> boxes(n);
        std::vector<int>    near;

        OrderGraph graph_x(n), graph_y(n);

        std::vector<double> x = tx, y = ty;

        auto overlaps = [ & ] (double x0, double y0, double w0, double h0,
                               double x1, double y1, double w1, double h1)
        {
            return x0 < x1 + w1 + gap_x && x1 < x0 + w0 + gap_x &&
                   y0 < y1 + h1 + gap_y && y1 < y0 + h0 + gap_y;
        };

        typedef AxisSolver::Status Status;

        bool   solved        = false; // solved at least once
        double last_solve_ms = 0.0;   // duration of the last solve (deadline)

        std::vector<char> solved_x(n, 0), solved_y(n, 0); // labels of solved components per axis (deadline)

        for (int round = 0; round < settings.cs_max_rounds; ++round)
        {
            bool added = false;

            st.rounds = round + 1;

            //label pairs (positions are fixed within a round)
            for (size_t i = 0; i < n; ++i)
                boxes[ i ] = QRectF(x[ i ], y[ i ], w[ i ], h[ i ]);

            label_index.build(boxes, &excluded);

            for (size_t i = 0; i < n && !st.time_exhausted; ++i)
            {
                if (deadline.expired())
                {
                    st.time_exhausted = true;
                    break;
                }

                if (excluded[ i ])
                    continue;

                label_index.query(boxes[ i ].adjusted(-gap_x, -gap_y, gap_x, gap_y), near);

                for (int jj : near)
                {
                    const size_t j = (size_t)jj;
                    if (j <= i)
                        continue;

                    if (!overlaps(x[ i ], y[ i ], w[ i ], h[ i ], x[ j ], y[ j ], w[ j ], h[ j ]))
                        continue;

                    if (separated_pairs.count(pairKey(i, j)))
                        continue;

                    double cost[ 4 ], prev_cost[ 4 ];
                    separationCosts(x [ i ], y [ i ], w[ i ], h[ i ], x [ j ], y [ j ], w[ j ], h[ j ], gap_x, gap_y, cost);
                    separationCosts(px[ i ], py[ i ], w[ i ], h[ i ], px[ j ], py[ j ], w[ j ], h[ j ], gap_x, gap_y, prev_cost);

                    for (int d : directionCandidates(cost, prev_cost,
                                                     std::min(w[ i ], w[ j ]), std::min(h[ i ], h[ j ]),
                                                     (w[ i ] + w[ j ]) / 2, (h[ i ] + h[ j ]) / 2,
                                                     settings))
                    {
                        const int a = (int)i;
                        const int b = (int)j;

                        OrderGraph::AddResult res;

                        if (d == Left)
                            res = graph_x.add(a, b);
                        else if (d == Right)
                            res = graph_x.add(b, a);
                        else if (d == Up)
                            res = graph_y.add(a, b);
                        else
                            res = graph_y.add(b, a);

                        if (res == OrderGraph::AddResult::Cycle)
                            continue;

                        //second - first >= size of the first + gap (hard), not needed if implied by others
                        if (res == OrderGraph::AddResult::Implied)
                        {
                            ++st.implied;
                        }
                        else
                        {
                            if (d == Left)
                                ax.addHard(a, b, w[ i ] + gap_x);
                            else if (d == Right)
                                ax.addHard(b, a, w[ j ] + gap_x);
                            else if (d == Up)
                                ay.addHard(a, b, h[ i ] + gap_y);
                            else
                                ay.addHard(b, a, h[ j ] + gap_y);

                            ++st.separations;
                        }

                        separated_pairs.insert(pairKey(i, j));

                        added = true;
                        break;
                    }
                }
            }

            //labels vs obstacles
            for (size_t i = 0; i < n && !st.time_exhausted; ++i)
            {
                if (deadline.expired())
                {
                    st.time_exhausted = true;
                    break;
                }

                if (excluded[ i ])
                    continue;

                obstacle_index.query(boxes[ i ].adjusted(-gap_x, -gap_y, gap_x, gap_y), near);

                for (int oo : near)
                {
                    const size_t o  = (size_t)oo;
                    const auto&  ob = obstacles[ o ];

                    if (!overlaps(x[ i ], y[ i ], w[ i ], h[ i ], ob.left(), ob.top(), ob.width(), ob.height()))
                        continue;

                    if (!bounded_obstacles.insert(obstacleKey(i, o)).second)
                        continue;

                    double cost[ 4 ], prev_cost[ 4 ];
                    separationCosts(x [ i ], y [ i ], w[ i ], h[ i ], ob.left(), ob.top(), ob.width(), ob.height(), gap_x, gap_y, cost);
                    separationCosts(px[ i ], py[ i ], w[ i ], h[ i ], ob.left(), ob.top(), ob.width(), ob.height(), gap_x, gap_y, prev_cost);

                    const int d = directionCandidates(cost, prev_cost,
                                                      std::min(w[ i ], ob.width()), std::min(h[ i ], ob.height()),
                                                      (w[ i ] + ob.width()) / 2, (h[ i ] + ob.height()) / 2,
                                                      settings).front();
                    const int    a  = (int)i;
                    const double wo = o < no_anchors ? soft : settings.cs_object_weight;

                    //soft: a label squeezed between obstacles may cover one rather than another label
                    if (d == Left)
                        ax.addSoftUpper(a, ob.left() - w[ i ] - gap_x,      wo);
                    else if (d == Right)
                        ax.addSoftLower(a, ob.left() + ob.width() + gap_x,  wo);
                    else if (d == Up)
                        ay.addSoftUpper(a, ob.top() - h[ i ] - gap_y,       wo);
                    else
                        ay.addSoftLower(a, ob.top() + ob.height() + gap_y,  wo);

                    ++st.obstacle_bounds;

                    added = true;
                }
            }

            if (st.time_exhausted)
                break;

            //no new overlaps => done (but solve at least once if there are constraints, e.g. the region of interest)
            if (!added && (solved || (ax.empty() && ay.empty())))
                break;

            //another solve would not end before the deadline (estimated by the last one)
            if (solved && deadline.exceededBy(last_solve_ms))
            {
                st.time_exhausted = true;
                break;
            }

            st.rounds_exhausted = added && round + 1 == settings.cs_max_rounds;

            //minimal displacement from the reference positions per axis (per component, smallest first); components
            //not solved before the deadline keep the positions of their last completed solve
            const double solve_start = deadline.elapsedMS();

            const Status sx = ax.solve(x, &deadline, &solved_x);
            const Status sy = sx == Status::Expired ? Status::Expired : ay.solve(y, &deadline, &solved_y);

            if (sx == Status::NotOptimal || sy == Status::NotOptimal)
                st.solver_converged = false;

            solved = true;

            last_solve_ms = deadline.elapsedMS() - solve_start;

            if (sx == Status::Feasible || sx == Status::Expired ||
                sy == Status::Feasible || sy == Status::Expired)
            {
                st.time_exhausted = true;
                break;
            }
        }

        //runtime limit before a label's component was solved => previous placement (home for labels without one)
        if (st.time_exhausted)
        {
            for (size_t i = 0; i < n; ++i)
            {
                if (!solved_x[ i ])
                    x[ i ] = px[ i ];
                if (!solved_y[ i ])
                    y[ i ] = py[ i ];
            }
        }

        //overcrowded => home
        for (size_t i = 0; i < n; ++i)
        {
            if (!excluded[ i ])
                continue;

            x[ i ] = hx[ i ];
            y[ i ] = hy[ i ];
        }

        for (size_t i = 0; i < n; ++i)
        {
            labels[ i ].x        = x[ i ];
            labels[ i ].y        = y[ i ];
            labels[ i ].excluded = excluded[ i ] != 0;
        }

        //diagnostics: pairs left overlapping (without gap), pairs without a feasible direction; pairs of two labels
        //not decluttered are not counted (overlapping on purpose)
        for (size_t i = 0; i < n; ++i)
            boxes[ i ] = QRectF(x[ i ], y[ i ], w[ i ], h[ i ]);

        label_index.build(boxes);

        for (size_t i = 0; i < n; ++i)
        {
            if (excluded[ i ])
                continue;

            label_index.query(boxes[ i ], near, false); // counts only

            for (int jj : near)
            {
                const size_t j = (size_t)jj;
                if (j == i || (j < i && !excluded[ j ])) // pairs of decluttered labels counted once
                    continue;

                if (x[ i ] < x[ j ] + w[ j ] && x[ j ] < x[ i ] + w[ i ] &&
                    y[ i ] < y[ j ] + h[ j ] && y[ j ] < y[ i ] + h[ i ])
                {
                    ++st.overlaps;

                    //only pairs of decluttered labels are separated (here i < j)
                    if (!excluded[ j ] && !separated_pairs.count(pairKey(i, j)))
                        ++st.unresolvable;
                }
            }
        }

        return true;
    }

} //namespace constraint
} //namespace label_placement
