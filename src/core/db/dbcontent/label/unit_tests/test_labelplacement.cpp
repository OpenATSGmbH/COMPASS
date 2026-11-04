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

#include "catch.hpp"

#include "labelplacement.h"
#include "labelplacement_helpers.h"
#include "labelplacement_spring.h"
#include "labelplacement_constraint.h"

#include <numeric>
#include <random>

#include <QLineF>

namespace
{

/**
 * Constraint based method without the optional mechanisms (the defaults are the settings of the label overlay):
 * reference position only, anchors and region of interest not avoided, no smoothing, no runtime limit, no density
 * checks. Tests of single mechanisms switch on what they need.
 */
void minimalConstraintSettings(label_placement::Settings& settings)
{
    settings.method               = label_placement::Method::ConstraintBased;
    settings.cs_balance           = 0.0;
    settings.cs_candidates        = 1;
    settings.cs_anchor_radius     = 0.0;
    settings.cs_keep_in_roi       = false;
    settings.cs_weight_prev       = 0.0;
    settings.cs_max_time_ms       = 0.0;
    settings.cs_max_group_density = 0.0;
    settings.cs_adaptive          = false;
}

}

TEST_CASE("LabelPlacementEngine coincident labels", "[labelplacement]")
{
    LabelPlacementEngine engine;

    SECTION("simple force method")
    {
        engine.settings().method = label_placement::Method::ForceBasedSimple;
    }
    SECTION("extended force method")
    {
        engine.settings().method = label_placement::Method::ForceBasedExtended;
    }
    SECTION("relaxed force method")
    {
        engine.settings().method = label_placement::Method::ForceBasedRelaxed;
    }

    //two labels at the same anchor and the same position (e.g. the same target from two sources)
    double x = 110.0;
    double y = 80.0;

    engine.addLabel("a", 100.0, 100.0, 40.0, 10.0, &x, &y);
    engine.addLabel("b", 100.0, 100.0, 40.0, 10.0, &x, &y);

    REQUIRE(engine.placeLabels());

    const auto* a = engine.getLabel("a");
    const auto* b = engine.getLabel("b");
    REQUIRE(a);
    REQUIRE(b);

    REQUIRE(!a->boundingBox().intersects(b->boundingBox()));

    //separated vertically, in label order
    REQUIRE(a->x == Approx(x));
    REQUIRE(b->x == Approx(x));
    REQUIRE(a->y < b->y);
}

TEST_CASE("LabelPlacementEngine fixed objects", "[labelplacement]")
{
    const auto method = GENERATE(label_placement::Method::ForceBasedSimple,
                                 label_placement::Method::ForceBasedExtended,
                                 label_placement::Method::ForceBasedExact,
                                 label_placement::Method::ForceBasedRelaxed,
                                 label_placement::Method::ConstraintBased);
    CAPTURE((int)method);

    LabelPlacementEngine engine;

    auto& settings = engine.settings();
    minimalConstraintSettings(settings);

    settings.method = method;
    settings.roi    = QRectF(0, 0, 400, 300);

    //fixed object (e.g. a manually placed label) covering the home position of label a
    const QRectF object(100.0, 80.0, 60.0, 20.0);

    settings.avoid_objects      = true;
    settings.additional_objects = { object };

    double ax = 95.0, ay = 85.0;
    double bx = 300.0, by = 200.0;

    engine.addLabel("a",  90.0, 100.0, 40.0, 10.0, &ax, &ay, &ax, &ay);
    engine.addLabel("b", 290.0, 215.0, 40.0, 10.0, &bx, &by, &bx, &by);

    REQUIRE(engine.placeLabels());

    const auto* a = engine.getLabel("a");
    const auto* b = engine.getLabel("b");
    REQUIRE(a);
    REQUIRE(b);

    //the label avoids the object, the object does not move anything else
    REQUIRE(!a->boundingBox().intersects(object));
    REQUIRE(!b->boundingBox().intersects(object));
    REQUIRE(QLineF(QPointF(b->x, b->y), QPointF(bx, by)).length() <= 1.0);
}

TEST_CASE("LabelPlacementEngine regularisation", "[labelplacement]")
{
    LabelPlacementEngine engine;

    auto& settings = engine.settings();
    settings.method          = label_placement::Method::ForceBasedExtended;
    settings.fb_weight_ref   = 0.05;
    settings.fb_weight_init  = 0.2;
    settings.fb_max_ref_dist = 60.0;

    SECTION("a label without conflict moves towards its reference position, repeated runs reach it")
    {
        double x_ref = 100.0;
        double y_ref = 100.0;
        double x     = 130.0;
        double y     = 100.0;

        engine.addLabel("a", 90.0, 120.0, 40.0, 10.0, &x, &y, &x_ref, &y_ref);
        REQUIRE(engine.placeLabels());

        //equilibrium of both pulls: (w_ref * ref + w_init * init) / (w_ref + w_init)
        REQUIRE(engine.getLabel("a")->x == Approx((0.05 * 100.0 + 0.2 * 130.0) / 0.25).margin(0.5));
        REQUIRE(engine.getLabel("a")->y == Approx(100.0).margin(0.5));

        //next runs start from the last placement
        for (int run = 0; run < 40; ++run)
        {
            x = engine.getLabel("a")->x;
            y = engine.getLabel("a")->y;

            engine.addLabel("a", 90.0, 120.0, 40.0, 10.0, &x, &y, &x_ref, &y_ref);
            REQUIRE(engine.placeLabels());
        }

        REQUIRE(engine.getLabel("a")->x == Approx(100.0).margin(1.0));
    }

    SECTION("labels never leave the max distance from their reference position")
    {
        settings.fb_max_ref_dist = 3.0;

        //coincident labels, separating them would need more than the max distance
        double x = 110.0;
        double y = 80.0;

        engine.addLabel("a", 100.0, 100.0, 40.0, 10.0, &x, &y, &x, &y);
        engine.addLabel("b", 100.0, 100.0, 40.0, 10.0, &x, &y, &x, &y);
        REQUIRE(engine.placeLabels());

        for (const char* id : { "a", "b" })
        {
            const auto* l = engine.getLabel(id);
            REQUIRE(QLineF(l->x, l->y, x, y).length() <= 3.0 + 1e-6);
        }
    }

    SECTION("coincident labels are still separated")
    {
        SECTION("simple force method")
        {
            settings.method = label_placement::Method::ForceBasedSimple;
        }
        SECTION("extended force method")
        {
            settings.method = label_placement::Method::ForceBasedExtended;
        }

        double x = 110.0;
        double y = 80.0;

        engine.addLabel("a", 100.0, 100.0, 40.0, 10.0, &x, &y, &x, &y);
        engine.addLabel("b", 100.0, 100.0, 40.0, 10.0, &x, &y, &x, &y);
        REQUIRE(engine.placeLabels());

        const auto* a = engine.getLabel("a");
        const auto* b = engine.getLabel("b");

        REQUIRE(!a->boundingBox().intersects(b->boundingBox()));
        REQUIRE(QLineF(a->x, a->y, x, y).length() <= 60.0 + 1e-6);
        REQUIRE(QLineF(b->x, b->y, x, y).length() <= 60.0 + 1e-6);
    }
}

TEST_CASE("LabelPlacementEngine relaxed force method", "[labelplacement]")
{
    LabelPlacementEngine engine;

    auto& settings = engine.settings();
    settings.method         = label_placement::Method::ForceBasedRelaxed;
    settings.fb_start_relax = 0.2;

    double x_ref = 100.0;
    double y_ref = 100.0;
    double x     = 130.0;
    double y     = 100.0;

    SECTION("a label without conflict starts the given fraction closer to its reference position")
    {
        engine.addLabel("a", 90.0, 120.0, 40.0, 10.0, &x, &y, &x_ref, &y_ref);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x == Approx(124.0));
        REQUIRE(engine.getLabel("a")->y == Approx(100.0));

        //next runs start from the last placement => reference position is approached
        for (int run = 0; run < 40; ++run)
        {
            x = engine.getLabel("a")->x;
            y = engine.getLabel("a")->y;

            engine.addLabel("a", 90.0, 120.0, 40.0, 10.0, &x, &y, &x_ref, &y_ref);
            REQUIRE(engine.placeLabels());
        }

        REQUIRE(engine.getLabel("a")->x == Approx(100.0).margin(0.01));
    }

    SECTION("no relaxation => the label stays where it is")
    {
        settings.fb_start_relax = 0.0;

        engine.addLabel("a", 90.0, 120.0, 40.0, 10.0, &x, &y, &x_ref, &y_ref);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x == Approx(130.0));
    }

    SECTION("a label without reference position stays where it is")
    {
        engine.addLabel("a", 90.0, 120.0, 40.0, 10.0, &x, &y);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x == Approx(130.0));
    }
}

TEST_CASE("LabelPlacementEngine constraint based method", "[labelplacement]")
{
    LabelPlacementEngine engine;
    minimalConstraintSettings(engine.settings());

    //label with home (reference) and previous placement (initial position), anchor below the home position
    auto add = [ & ] (const std::string& id, double hx, double hy, double px, double py, double w = 40.0, double h = 10.0)
    {
        engine.addLabel(id, hx, hy + 30.0, w, h, &px, &py, &hx, &hy);
    };

    auto rect = [ & ] (const char* id)
    {
        const auto* l = engine.getLabel(id);
        return QRectF(l->x, l->y, l->w, l->h);
    };

    SECTION("a label without conflict is placed at home, also if placed elsewhere before")
    {
        add("a", 100.0, 100.0, 130.0, 80.0);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x == Approx(100.0));
        REQUIRE(engine.getLabel("a")->y == Approx(100.0));
    }

    SECTION("with a weight on the previous placement a label without conflict returns home gradually")
    {
        engine.settings().cs_weight_prev = 1.0;

        add("a", 100.0, 100.0, 130.0, 80.0);
        REQUIRE(engine.placeLabels());

        //equal weights => halfway
        REQUIRE(engine.getLabel("a")->x == Approx(115.0));
        REQUIRE(engine.getLabel("a")->y == Approx(90.0));
    }

    SECTION("the separation direction can be weighted by the label size")
    {
        //wide labels, mostly side by side: vertical needs 14 px, horizontal 22 px (previously coincident => no
        //previous side)
        add("a", 100.0, 100.0, 100.0, 100.0, 60.0, 16.0);
        add("b", 140.0, 104.0, 100.0, 100.0, 60.0, 16.0);

        SECTION("absolute => vertical")
        {
            engine.settings().cs_balance = 0.0;
            REQUIRE(engine.placeLabels());

            REQUIRE(engine.getLabel("a")->x == Approx(100.0));
            REQUIRE(engine.getLabel("b")->x == Approx(140.0));
        }
        SECTION("relative to the size => horizontal (22 / 60 < 14 / 16)")
        {
            engine.settings().cs_balance = 1.0;
            REQUIRE(engine.placeLabels());

            REQUIRE(engine.getLabel("a")->y == Approx(100.0));
            REQUIRE(engine.getLabel("b")->y == Approx(104.0));
        }

        REQUIRE(!rect("a").intersects(rect("b")));
    }

    SECTION("candidate positions: a conflicting label moves to the other side of its anchor")
    {
        engine.settings().cs_candidates   = 4;
        engine.settings().cs_cand_penalty = 0.0;
        engine.settings().cs_cand_switch  = 0.5;

        //anchors 2 px apart vertically (same x), homes right up of them (anchor line 20 px) => overlapping by 8 px,
        //more than the cost of changing sides (0.5 * label area)
        double ahx = 114.0, ahy = 106.0;
        double bhx = 114.0, bhy = 108.0;
        engine.addLabel("a", 100.0, 130.0, 40.0, 10.0, &ahx, &ahy, &ahx, &ahy);
        engine.addLabel("b", 100.0, 132.0, 40.0, 10.0, &bhx, &bhy, &bhx, &bhy);
        REQUIRE(engine.placeLabels());

        REQUIRE(!rect("a").intersects(rect("b")));

        //a is mirrored to the left of its anchor (2 * 100 - 114 - 40 = 46), b stays home, no displacement needed
        REQUIRE(engine.getLabel("a")->x == Approx(46.0));
        REQUIRE(engine.getLabel("a")->y == Approx(106.0));
        REQUIRE(engine.getLabel("b")->x == Approx(114.0));
        REQUIRE(engine.getLabel("b")->y == Approx(108.0));
    }

    SECTION("candidate positions: the previous side is kept")
    {
        engine.settings().cs_candidates = 8;

        //no conflict, home right up of the anchor (anchor line 20 px), previously left up
        double hx = 114.0, hy = 106.0;
        double px = 46.0,  py = 106.0;
        engine.addLabel("a", 100.0, 130.0, 40.0, 10.0, &px, &py, &hx, &hy);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x == Approx(46.0));
        REQUIRE(engine.getLabel("a")->y == Approx(106.0));
    }

    SECTION("candidate positions: a label without history chooses its side freely")
    {
        engine.settings().cs_candidates   = 8;
        engine.settings().cs_cand_penalty = 0.0;
        engine.settings().cs_cand_switch  = 0.5;

        //anchors 6 px apart vertically => homes overlap by 4 px, less than the cost of changing sides
        double ahx = 114.0, ahy = 106.0;
        double bhx = 114.0, bhy = 112.0;
        engine.addLabel("b", 100.0, 136.0, 40.0, 10.0, &bhx, &bhy, &bhx, &bhy);

        SECTION("with history => both stay on their side, separated by displacement")
        {
            engine.addLabel("a", 100.0, 130.0, 40.0, 10.0, &ahx, &ahy, &ahx, &ahy);
            REQUIRE(engine.placeLabels());

            REQUIRE(engine.getLabel("a")->x == Approx(114.0));
            REQUIRE(engine.getLabel("b")->x == Approx(114.0));
        }
        SECTION("without history => a moves to a free side, no displacement needed")
        {
            engine.addLabel("a", 100.0, 130.0, 40.0, 10.0, &ahx, &ahy, &ahx, &ahy, false);
            REQUIRE(engine.placeLabels());

            const auto* a = engine.getLabel("a");
            REQUIRE(a->x != Approx(114.0));
            REQUIRE(engine.getLabel("b")->x == Approx(114.0));
            REQUIRE(engine.getLabel("b")->y == Approx(112.0));
        }

        REQUIRE(!rect("a").intersects(rect("b")));
    }

    SECTION("without candidate positions labels stay on their side")
    {
        add("a", 114.0, 100.0, 114.0, 100.0);
        add("b", 114.0, 106.0, 114.0, 106.0);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x + 20.0 > engine.getLabel("a")->x_anchor);
        REQUIRE(engine.getLabel("b")->x + 20.0 > engine.getLabel("b")->x_anchor);
    }

    SECTION("gaps per axis")
    {
        engine.settings().cs_gap_y = 4.0;

        add("a", 100.0, 100.0, 100.0, 100.0);
        add("b", 100.0, 100.0, 100.0, 100.0);
        REQUIRE(engine.placeLabels());

        //height + vertical gap = 14, split evenly
        REQUIRE(engine.getLabel("a")->y == Approx(93.0).margin(0.01));
        REQUIRE(engine.getLabel("b")->y == Approx(107.0).margin(0.01));
    }

    SECTION("coincident labels are separated with minimal displacement")
    {
        add("a", 100.0, 100.0, 100.0, 100.0);
        add("b", 100.0, 100.0, 100.0, 100.0);
        REQUIRE(engine.placeLabels());

        //vertical is cheaper (height + gap = 12), split evenly, first label above
        REQUIRE(engine.getLabel("a")->x == Approx(100.0));
        REQUIRE(engine.getLabel("b")->x == Approx(100.0));
        REQUIRE(engine.getLabel("a")->y == Approx(94.0).margin(0.01));
        REQUIRE(engine.getLabel("b")->y == Approx(106.0).margin(0.01));
        REQUIRE(!rect("a").intersects(rect("b")));
    }

    SECTION("the previous side of a pair is kept")
    {
        //b was above a before
        add("a", 100.0, 100.0, 100.0, 100.0);
        add("b", 100.0, 100.0, 100.0, 88.0);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("b")->y == Approx(94.0).margin(0.01));
        REQUIRE(engine.getLabel("a")->y == Approx(106.0).margin(0.01));
    }

    SECTION("the previous axis is kept if not much more expensive")
    {
        //side by side before, narrow labels: horizontal needs 22, vertical 12 => kept (22 <= 2 * 12 + 10)
        add("a", 100.0, 100.0, 100.0, 100.0, 20.0, 10.0);
        add("b", 100.0, 100.0, 125.0, 100.0, 20.0, 10.0);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x == Approx(89.0).margin(0.01));
        REQUIRE(engine.getLabel("b")->x == Approx(111.0).margin(0.01));
        REQUIRE(engine.getLabel("a")->y == Approx(100.0));
        REQUIRE(engine.getLabel("b")->y == Approx(100.0));
    }

    SECTION("a much more expensive previous axis is not kept")
    {
        //side by side before, wide labels: horizontal needs 42, vertical 12 => switched (42 > 2 * 12 + 10)
        add("a", 100.0, 100.0, 100.0, 100.0);
        add("b", 100.0, 100.0, 145.0, 100.0);
        REQUIRE(engine.placeLabels());

        REQUIRE(engine.getLabel("a")->x == Approx(100.0));
        REQUIRE(engine.getLabel("b")->x == Approx(100.0));
        REQUIRE(!rect("a").intersects(rect("b")));
    }

    SECTION("a label passing another one moves smoothly, without overlap")
    {
        //b's home moves past a in steps of 1 px, each run starts from the last placement
        double ax = 100.0, ay = 100.0, bx = 60.0, by = 100.0;

        int large_changes = 0;

        for (int step = 0; step <= 80; ++step)
        {
            const double bhx = 60.0 + step;

            engine.clearLabels();
            add("a", 100.0, 100.0, ax, ay);
            add("b", bhx, 100.0, bx, by);
            REQUIRE(engine.placeLabels());

            REQUIRE(!rect("a").intersects(rect("b")));

            const auto* a = engine.getLabel("a");
            const auto* b = engine.getLabel("b");

            //change relative to the home positions (b's home moved by 1)
            const double change = std::max(std::hypot(a->x - ax, a->y - ay),
                                           std::hypot((b->x - bhx) - (bx - (bhx - 1.0)), b->y - by));
            if (step > 0 && change > 2.0)
                ++large_changes;

            ax = a->x; ay = a->y; bx = b->x; by = b->y;
        }

        //continuous except where the pair has to change its axis (needed separation gets too large)
        REQUIRE(large_changes <= 2);
    }

    SECTION("anchors are avoided")
    {
        engine.settings().cs_anchor_radius = 10.0;

        double hx = 100.0, hy = 100.0;

        //b's anchor lies within a's home position
        engine.addLabel("a", 100.0, 160.0, 40.0, 10.0, &hx, &hy, &hx, &hy);

        double bhx = 300.0, bhy = 300.0;
        engine.addLabel("b", 120.0, 105.0, 40.0, 10.0, &bhx, &bhy, &bhx, &bhy);

        REQUIRE(engine.placeLabels());

        REQUIRE(!rect("a").intersects(QRectF(110.0, 95.0, 20.0, 20.0)));
    }

    SECTION("a dense stack is decluttered")
    {
        for (int i = 0; i < 20; ++i)
            add("l" + std::to_string(i), 500.0, 500.0, 500.0, 500.0);

        REQUIRE(engine.placeLabels());

        for (int i = 0; i < 20; ++i)
            for (int j = i + 1; j < 20; ++j)
                REQUIRE(!rect(("l" + std::to_string(i)).c_str()).intersects(rect(("l" + std::to_string(j)).c_str())));
    }

    SECTION("labels are kept within the region of interest")
    {
        engine.settings().cs_keep_in_roi = true;
        engine.settings().roi            = QRectF(0.0, 0.0, 400.0, 300.0);

        add("a", 380.0, -5.0, 380.0, -5.0);
        REQUIRE(engine.placeLabels());

        //soft bounds: violation / overshoot = 1 / cs_soft_weight (100) => (380 + 100 * 360) / 101
        REQUIRE(engine.getLabel("a")->x == Approx((380.0 + 100.0 * 360.0) / 101.0).margin(0.01));
        REQUIRE(engine.getLabel("a")->y == Approx((-5.0 + 100.0 * 0.0) / 101.0).margin(0.01));
    }

    SECTION("same input, same placement")
    {
        auto run = [ & ] ()
        {
            engine.clearLabels();
            for (int i = 0; i < 10; ++i)
                add("l" + std::to_string(i), 100.0 + i * 7.0, 100.0 + (i % 3) * 4.0, 100.0 + i * 7.0, 100.0);

            REQUIRE(engine.placeLabels());

            std::vector<QPointF> pos;
            for (int i = 0; i < 10; ++i)
                pos.emplace_back(engine.getLabel(i).x, engine.getLabel(i).y);
            return pos;
        };

        REQUIRE(run() == run());
    }
}

namespace
{
    /**
     * Reference solver for the axis problem (Hildreth's method, plain projected dual coordinate ascent).
     */
    std::vector<double> referenceSolve(const std::vector<double>& desired,
                                       const std::vector<double>& weights,
                                       const std::vector<label_placement::constraint::Separation>& constraints)
    {
        std::vector<double> v = desired;
        std::vector<double> lambda(constraints.size(), 0.0);

        for (int sweep = 0; sweep < 1000000; ++sweep)
        {
            double max_change = 0.0;

            for (size_t k = 0; k < constraints.size(); ++k)
            {
                const auto& c = constraints[ k ];

                const double norm       = 1.0 / weights[ c.left ] + 1.0 / weights[ c.right ];
                const double lambda_new = std::max(0.0, lambda[ k ] + (c.gap - (v[ c.right ] - v[ c.left ])) / norm);
                const double dl         = lambda_new - lambda[ k ];

                v[ c.right ] += dl / weights[ c.right ];
                v[ c.left  ] -= dl / weights[ c.left  ];

                lambda[ k ] = lambda_new;
                max_change  = std::max(max_change, std::fabs(dl));
            }

            if (max_change < 1e-14)
                break;
        }

        return v;
    }
}

TEST_CASE("Constraint based placement axis solver", "[labelplacement]")
{
    using label_placement::constraint::AxisSolver;
    using label_placement::constraint::Separation;

    const auto Optimal = AxisSolver::Status::Optimal;

    std::vector<double> v;

    SECTION("a violated constraint is split by the weights")
    {
        //min (a - 0)^2 + 3 (b - 0)^2 with b - a >= 10 => a = -7.5, b = 2.5
        AxisSolver solver({ 0.0, 0.0 }, { 1.0, 3.0 }, { { 0, 1, 10.0 } });
        REQUIRE(solver.solve(v) == Optimal);

        REQUIRE(v[ 0 ] == Approx(-7.5));
        REQUIRE(v[ 1 ] == Approx(2.5));
    }

    SECTION("a satisfied constraint does not move anything")
    {
        AxisSolver solver({ 0.0, 20.0 }, { 1.0, 1.0 }, { { 0, 1, 10.0 } });
        REQUIRE(solver.solve(v) == Optimal);

        REQUIRE(v[ 0 ] == Approx(0.0));
        REQUIRE(v[ 1 ] == Approx(20.0));
    }

    SECTION("a block merged too eagerly is split again")
    {
        //a and b are merged first, then the heavy c pulls the block far left => b's multiplier is negative, b is
        //split off and stays at its desired position; a and c: min a^2 + 10 (c + 100)^2 with a <= c => -1000 / 11
        AxisSolver solver({ 0.0, 0.0, -100.0 }, { 1.0, 1.0, 10.0 }, { { 0, 1, 10.0 }, { 0, 2, 0.0 } });
        REQUIRE(solver.solve(v) == Optimal);

        REQUIRE(v[ 0 ] == Approx(-1000.0 / 11.0));
        REQUIRE(v[ 1 ] == Approx(0.0).margin(1e-9));
        REQUIRE(v[ 2 ] == Approx(-1000.0 / 11.0));
    }

    SECTION("cyclic constraints are reported")
    {
        AxisSolver solver({ 0.0, 0.0 }, { 1.0, 1.0 }, { { 0, 1, 10.0 }, { 1, 0, 10.0 } });
        REQUIRE(solver.solve(v) == AxisSolver::Status::NotOptimal);
    }

    SECTION("random problems: same optimum as the reference solver")
    {
        std::mt19937 rng(42);

        std::uniform_real_distribution<double> u_pos(0.0, 100.0);
        std::uniform_real_distribution<double> u_gap(0.0, 20.0);
        std::uniform_int_distribution<int>     u_n  (2, 12);
        std::bernoulli_distribution            heavy(0.2);

        for (int problem = 0; problem < 100; ++problem)
        {
            const int n = u_n(rng);

            std::vector<double> desired(n), weights(n);
            for (int i = 0; i < n; ++i)
            {
                desired[ i ] = u_pos(rng);
                weights[ i ] = heavy(rng) ? 100.0 : 1.0;
            }

            //random order => acyclic constraints
            std::vector<int> order(n);
            std::iota(order.begin(), order.end(), 0);
            std::shuffle(order.begin(), order.end(), rng);

            std::vector<Separation> constraints;
            std::uniform_int_distribution<int> u_idx(0, n - 1);

            for (int k = 0; k < 2 * n; ++k)
            {
                int i = u_idx(rng);
                int j = u_idx(rng);
                if (i == j)
                    continue;
                if (i > j)
                    std::swap(i, j);

                constraints.push_back({ order[ i ], order[ j ], u_gap(rng) });
            }

            CAPTURE(problem);

            AxisSolver solver(desired, weights, constraints);
            REQUIRE(solver.solve(v) == Optimal);

            const auto ref = referenceSolve(desired, weights, constraints);

            for (const auto& c : constraints)
                REQUIRE(v[ c.right ] - v[ c.left ] >= c.gap - 1e-6);

            for (int i = 0; i < n; ++i)
                REQUIRE(v[ i ] == Approx(ref[ i ]).margin(1e-4));
        }
    }
}

TEST_CASE("Constraint based placement of an overcrowded cluster", "[labelplacement]")
{
    //50 labels with homes within a small area (more space needed than available locally)
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> ux(650.0, 750.0), uy(400.0, 460.0);

    std::vector<label_placement::Label> labels(50);
    for (size_t i = 0; i < labels.size(); ++i)
    {
        auto& l = labels[ i ];

        l.id       = std::to_string(i);
        l.w        = 60.0;
        l.h        = 16.0;
        l.x_anchor = ux(rng);
        l.y_anchor = uy(rng);
        l.x        = l.x_anchor + 14.0;
        l.y        = l.y_anchor - 30.0;
        l.x_last   = l.x;
        l.y_last   = l.y;
        l.x_ref    = l.x;
        l.y_ref    = l.y;
    }

    const double weight_prev = GENERATE(0.0, 1.0);
    CAPTURE(weight_prev);

    label_placement::Settings settings;
    minimalConstraintSettings(settings);

    settings.cs_weight_prev   = weight_prev;
    settings.roi              = QRectF(0.0, 0.0, 1599.0, 899.0);
    settings.cs_avoid_anchors = true;
    settings.cs_anchor_radius = 10.0;
    settings.cs_keep_in_roi   = true;

    label_placement::constraint::Stats stats;
    REQUIRE(label_placement::constraint::placeLabels(labels, settings, &stats));

    REQUIRE(stats.solver_converged);
    REQUIRE(!stats.rounds_exhausted);
    REQUIRE(!stats.time_exhausted); // no limit
    REQUIRE(stats.overlaps == 0);
}

TEST_CASE("Constraint based placement implied separations and runtime limit", "[labelplacement]")
{
    label_placement::Settings settings;
    minimalConstraintSettings(settings);

    SECTION("a separation implied by others adds no constraint")
    {
        //tall labels in a row (horizontal separation cheapest): 1 left of 0 (pair 0-1), 0 left of 2 (pair 0-2)
        //=> 1 left of 2 (pair 1-2, checked last) is implied
        std::vector<label_placement::Label> labels(3);

        const double xs[ 3 ] = { 4.0, 0.0, 8.0 };
        for (size_t i = 0; i < labels.size(); ++i)
        {
            auto& l = labels[ i ];

            l.id       = std::to_string(i);
            l.w        = 16.0;
            l.h        = 60.0;
            l.x_anchor = xs[ i ];
            l.y_anchor = 200.0;
            l.x        = l.x_last = xs[ i ];
            l.y        = l.y_last = 100.0;
            l.x_ref    = l.x;
            l.y_ref    = l.y;
        }

        label_placement::constraint::Stats stats;
        REQUIRE(label_placement::constraint::placeLabels(labels, settings, &stats));

        REQUIRE(stats.implied     == 1);
        REQUIRE(stats.separations == 2);
        REQUIRE(stats.overlaps    == 0);

        //order and gaps hold (the implied one too)
        REQUIRE(labels[ 0 ].x - labels[ 1 ].x >= 16.0 + settings.cs_gap_x - 1e-6);
        REQUIRE(labels[ 2 ].x - labels[ 0 ].x >= 16.0 + settings.cs_gap_x - 1e-6);
        REQUIRE(labels[ 2 ].x - labels[ 1 ].x >= 16.0 + settings.cs_gap_x - 1e-6);
    }

    SECTION("a runtime limit stops the placement, the previous placement is kept")
    {
        //overlapping cluster, previous placement = home
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> ux(650.0, 750.0), uy(400.0, 460.0);

        std::vector<label_placement::Label> labels(50);
        for (size_t i = 0; i < labels.size(); ++i)
        {
            auto& l = labels[ i ];

            l.id       = std::to_string(i);
            l.w        = 60.0;
            l.h        = 16.0;
            l.x_anchor = ux(rng);
            l.y_anchor = uy(rng);
            l.x        = l.x_last = l.x_anchor + 14.0;
            l.y        = l.y_last = l.y_anchor - 30.0;
            l.x_ref    = l.x;
            l.y_ref    = l.y;
        }

        //expired before the first solve
        settings.cs_max_time_ms = 1e-6;

        label_placement::constraint::Stats stats;
        REQUIRE(label_placement::constraint::placeLabels(labels, settings, &stats));

        REQUIRE(stats.time_exhausted);

        for (const auto& l : labels)
        {
            REQUIRE(l.x == l.x_last);
            REQUIRE(l.y == l.y_last);
        }
    }
}

TEST_CASE("Constraint based placement box index", "[labelplacement]")
{
    //random boxes of different sizes, some touching
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> ux(0.0, 800.0), uy(0.0, 600.0), uw(5.0, 120.0), uh(5.0, 40.0);

    std::vector<QRectF> boxes;
    for (int i = 0; i < 300; ++i)
        boxes.emplace_back(ux(rng), uy(rng), uw(rng), uh(rng));

    boxes.emplace_back(boxes[ 0 ].right(), boxes[ 0 ].top(), 10.0, 10.0); // touches box 0

    std::vector<char> skip(boxes.size(), 0);
    for (size_t i = 0; i < boxes.size(); i += 7)
        skip[ i ] = 1;

    //r-tree and test of all boxes
    const bool use_tree = GENERATE(true, false);

    SECTION("a query returns all intersecting boxes in ascending order")
    {
        label_placement::constraint::BoxIndex index(use_tree);
        index.build(boxes);

        std::vector<int> near;
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            index.query(boxes[ i ], near);

            REQUIRE(std::is_sorted(near.begin(), near.end()));

            std::vector<int> expected, found;
            for (size_t j = 0; j < boxes.size(); ++j)
                if (boxes[ i ].intersects(boxes[ j ]))
                    expected.push_back((int)j);
            for (int j : near)
                if (boxes[ i ].intersects(boxes[ j ]))
                    found.push_back(j);

            REQUIRE(found == expected);
        }

        //touching boxes are returned as well (superset)
        index.query(boxes[ 0 ], near);
        REQUIRE(std::find(near.begin(), near.end(), (int)boxes.size() - 1) != near.end());
    }

    SECTION("skipped boxes are left out")
    {
        label_placement::constraint::BoxIndex index(use_tree);
        index.build(boxes, &skip);

        std::vector<int> near;
        for (size_t i = 0; i < boxes.size(); ++i)
        {
            index.query(boxes[ i ], near);

            for (int j : near)
                REQUIRE(!skip[ j ]);
        }
    }
}

TEST_CASE("Constraint based placement without the spatial index", "[labelplacement]")
{
    //spread labels and a dense cluster
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> ux(20.0, 1500.0), uy(40.0, 860.0), cx(700.0, 800.0), cy(400.0, 460.0);

    std::vector<label_placement::Label> labels(300);
    for (size_t i = 0; i < labels.size(); ++i)
    {
        auto& l = labels[ i ];

        const bool cluster = i % 5 == 0;

        l.id       = std::to_string(i);
        l.w        = 60.0;
        l.h        = 16.0;
        l.x_anchor = cluster ? cx(rng) : ux(rng);
        l.y_anchor = cluster ? cy(rng) : uy(rng);
        l.x        = l.x_last = l.x_anchor + 14.0;
        l.y        = l.y_last = l.y_anchor - 30.0;
        l.x_ref    = l.x;
        l.y_ref    = l.y;
    }

    //overlay settings (defaults), no runtime limit (deterministic)
    label_placement::Settings settings;
    settings.roi            = QRectF(0.0, 0.0, 1599.0, 899.0);
    settings.cs_max_time_ms = 0.0;
    settings.cs_adaptive    = GENERATE(false, true);

    auto run = [ & ] (bool spatial_index, label_placement::constraint::Stats& stats)
    {
        auto ls = labels;

        settings.cs_spatial_index = spatial_index;
        REQUIRE(label_placement::constraint::placeLabels(ls, settings, &stats));

        return ls;
    };

    label_placement::constraint::Stats stats_tree, stats_pairs;

    const auto with_tree    = run(true,  stats_tree);
    const auto without_tree = run(false, stats_pairs);

    //bit-identical
    for (size_t i = 0; i < labels.size(); ++i)
    {
        REQUIRE(with_tree[ i ].x        == without_tree[ i ].x);
        REQUIRE(with_tree[ i ].y        == without_tree[ i ].y);
        REQUIRE(with_tree[ i ].excluded == without_tree[ i ].excluded);
    }

    REQUIRE(stats_tree.separations == stats_pairs.separations);
    REQUIRE(stats_tree.overlaps    == stats_pairs.overlaps);
    REQUIRE(stats_tree.overcrowded == stats_pairs.overcrowded);
    REQUIRE(stats_tree.overcrowded >  0);
}

TEST_CASE("Constraint based placement of overcrowded groups", "[labelplacement]")
{
    //20 labels at the same anchor (pile-up) and a label far away
    std::vector<label_placement::Label> labels(21);
    for (size_t i = 0; i < labels.size(); ++i)
    {
        auto& l = labels[ i ];

        const double ax = i < 20 ? 400.0 : 1200.0;

        l.id       = std::to_string(i);
        l.w        = 60.0;
        l.h        = 16.0;
        l.x_anchor = ax;
        l.y_anchor = 400.0;
        l.x        = l.x_last = ax + 14.0;
        l.y        = l.y_last = 400.0 - 30.0;
        l.x_ref    = l.x;
        l.y_ref    = l.y;
    }

    SECTION("group densities: the pile-up needs much more space than available, the single label not")
    {
        std::vector<double> hx, hy;
        for (const auto& l : labels)
        {
            hx.push_back(l.x_last);
            hy.push_back(l.y_last);
        }

        const auto density = label_placement::constraint::groupDensities(labels, hx, hy, 2.0, 2.0);

        REQUIRE(density[ 0 ]  > 1.0);
        REQUIRE(density[ 19 ] == Approx(density[ 0 ]));
        REQUIRE(density[ 20 ] < 0.5);
    }

    SECTION("group densities: the area of a group is the union of its candidate regions, clipped if given")
    {
        //diagonal chain of 10 labels (home boxes overlap with the next one only)
        const double dx = 50.0, dy = 14.0;

        std::vector<label_placement::Label> chain(10);
        std::vector<double> hx, hy;

        for (size_t i = 0; i < chain.size(); ++i)
        {
            auto& l = chain[ i ];

            l.id       = std::to_string(i);
            l.w        = 60.0;
            l.h        = 16.0;
            l.x_anchor = 100.0 + i * dx;
            l.y_anchor = 100.0 + i * dy;

            hx.push_back(l.x_anchor + 14.0);
            hy.push_back(l.y_anchor - 30.0);
        }

        //candidate regions: anchor +- (anchor distance of the home + label size), the union of the shifted
        //regions is a staircase
        const double dist = std::hypot(14.0, 14.0);
        const double rw   = 2 * (dist + 60.0);
        const double rh   = 2 * (dist + 16.0);
        const double n    = (double)chain.size();

        const double union_area = rw * rh + (n - 1) * (rw * rh - (rw - dx) * (rh - dy));
        const double bbox_area  = (rw + (n - 1) * dx) * (rh + (n - 1) * dy);
        const double label_area = n * 62.0 * 18.0;

        const auto density = label_placement::constraint::groupDensities(chain, hx, hy, 2.0, 2.0);

        //one group, measured on the raster (cells of 8 px)
        REQUIRE(density.front() == Approx(label_area / union_area).epsilon(0.03));
        REQUIRE(density.back()  == Approx(density.front()));
        REQUIRE(density.front() > 1.8 * label_area / bbox_area);

        //clipped to the upper left part => less area available
        const QRectF clip(0.0, 0.0, 100.0 + 5 * dx, 1000.0);

        const auto density_clipped = label_placement::constraint::groupDensities(chain, hx, hy, 2.0, 2.0, clip);

        REQUIRE(density_clipped.front() > 1.5 * density.front());
    }

    SECTION("an overcrowded group is not decluttered, the others are")
    {
        label_placement::Settings settings;
        minimalConstraintSettings(settings);

        settings.cs_max_group_density = 1.0;

        label_placement::constraint::Stats stats;
        REQUIRE(label_placement::constraint::placeLabels(labels, settings, &stats));

        REQUIRE(stats.overcrowded == 20);
        REQUIRE(stats.max_group_density > 1.0);
        REQUIRE(stats.separations == 0);

        //at home
        for (const auto& l : labels)
        {
            REQUIRE(l.x == Approx(l.x_ref.value()));
            REQUIRE(l.y == Approx(l.y_ref.value()));
        }
    }

    SECTION("without the threshold the pile-up is decluttered")
    {
        label_placement::Settings settings;
        minimalConstraintSettings(settings);

        label_placement::constraint::Stats stats;
        REQUIRE(label_placement::constraint::placeLabels(labels, settings, &stats));

        REQUIRE(stats.overcrowded == 0);
        REQUIRE(stats.separations > 0);
        REQUIRE(stats.overlaps == 0);
    }
}

TEST_CASE("Constraint based placement area table and crowding", "[labelplacement]")
{
    SECTION("area table: exact for cell aligned rectangles, continuous in between")
    {
        std::mt19937 rng(5);
        std::uniform_real_distribution<double> ux(-20.0, 780.0), uy(-20.0, 580.0), uw(5.0, 90.0), uh(5.0, 40.0);

        std::vector<QRectF> boxes;
        for (int i = 0; i < 200; ++i)
            boxes.emplace_back(ux(rng), uy(rng), uw(rng), uh(rng));

        const QRectF bounds(0.0, 0.0, 800.0, 600.0);

        label_placement::constraint::AreaTable table;
        table.build(boxes, bounds, 8.0);

        auto bruteForce = [ & ] (const QRectF& r)
        {
            double a = 0.0;
            for (const auto& b : boxes)
                a += label_placement::constraint::overlapArea(b, r.intersected(bounds));
            return a;
        };

        REQUIRE(table.area(bounds) == Approx(bruteForce(bounds)).epsilon(1e-9));

        const QRectF aligned(16.0, 24.0, 400.0, 320.0);
        REQUIRE(table.area(aligned) == Approx(bruteForce(aligned)).epsilon(1e-9));

        //not aligned: close to the exact area, no jumps when moved by a fraction of a cell
        const QRectF r(13.7, 21.3, 317.9, 211.4);
        REQUIRE(table.area(r) == Approx(bruteForce(r)).epsilon(0.02));
        REQUIRE(table.area(r.translated(0.1, 0.1)) == Approx(table.area(r)).epsilon(0.002));
    }

    SECTION("crowding: a small dense cluster with room around it is less crowded than a large area of the same density")
    {
        //labels 60x16 on a grid of 70x24 px: a 3x3 cluster and a field over the whole region
        auto grid = [ & ] (int nx, int ny, double x0, double y0)
        {
            std::vector<label_placement::Label> labels;
            for (int iy = 0; iy < ny; ++iy)
            {
                for (int ix = 0; ix < nx; ++ix)
                {
                    label_placement::Label l;
                    l.w        = 60.0;
                    l.h        = 16.0;
                    l.x_anchor = x0 + ix * 70.0;
                    l.y_anchor = y0 + iy * 24.0;
                    labels.push_back(l);
                }
            }
            return labels;
        };

        auto centerCrowding = [ & ] (const std::vector<label_placement::Label>& labels, size_t center)
        {
            std::vector<double> hx, hy;
            for (const auto& l : labels)
            {
                hx.push_back(l.x_anchor + 14.0);
                hy.push_back(l.y_anchor - 30.0);
            }

            const auto crowding = label_placement::constraint::localCrowding(labels, hx, hy, 2.0, 2.0, 4, 2.0,
                                                                             QRectF(0.0, 0.0, 1600.0, 900.0));
            return crowding[ center ];
        };

        const auto cluster = grid(3, 3, 800.0, 450.0);
        const auto field   = grid(22, 37, 30.0, 40.0);

        const double c_cluster = centerCrowding(cluster, 4);
        const double c_field   = centerCrowding(field, 18 * 22 + 11);

        REQUIRE(c_cluster < 0.5);
        REQUIRE(c_field   > 0.5);
        REQUIRE(c_field   > 2.0 * c_cluster);
    }
}

TEST_CASE("Constraint based placement adaptive", "[labelplacement]")
{
    //core of 30 labels at one anchor, chain of 5 labels leading away from it (home boxes overlapping the next one)
    std::vector<label_placement::Label> labels(35);
    for (size_t i = 0; i < labels.size(); ++i)
    {
        auto& l = labels[ i ];

        const double ax = i < 30 ? 400.0 : 455.0 + (i - 30) * 50.0;

        l.id       = std::to_string(i);
        l.w        = 60.0;
        l.h        = 16.0;
        l.x_anchor = ax;
        l.y_anchor = 400.0;
        l.x        = l.x_last = ax + 14.0;
        l.y        = l.y_last = 400.0 - 30.0;
        l.x_ref    = l.x;
        l.y_ref    = l.y;
    }

    label_placement::Settings settings;
    minimalConstraintSettings(settings);

    settings.cs_max_group_density = 1.0;

    //adaptive parameters fixed (independent of the tuned defaults): the core of 30 labels is crowded
    settings.cs_max_local_density = 0.5;
    settings.cs_local_scales      = 4;
    settings.cs_local_reach       = 2.0;

    SECTION("group density: one group, nothing decluttered")
    {
        label_placement::constraint::Stats stats;
        REQUIRE(label_placement::constraint::placeLabels(labels, settings, &stats));

        REQUIRE(stats.overcrowded == 35);

        for (const auto& l : labels)
            REQUIRE(l.excluded);
    }

    SECTION("adaptive: the core and the chain labels next to it stay at home, the rest is decluttered")
    {
        settings.cs_adaptive = true;

        label_placement::constraint::Stats stats;
        REQUIRE(label_placement::constraint::placeLabels(labels, settings, &stats));

        REQUIRE(stats.overcrowded == 32);
        REQUIRE(stats.max_local_density > settings.cs_max_local_density);
        REQUIRE(stats.max_group_density == 0.0); // replaced

        for (size_t i = 0; i < labels.size(); ++i)
        {
            const auto& l = labels[ i ];

            REQUIRE(l.excluded == (i < 32));

            if (l.excluded)
            {
                REQUIRE(l.x == Approx(l.x_ref.value()));
                REQUIRE(l.y == Approx(l.y_ref.value()));
            }
        }

        //labels left at home are fixed obstacles: the first decluttered label leaves the home of its neighbour
        const auto& fixed = labels[ 31 ];
        const auto& moved = labels[ 32 ];

        REQUIRE(QRectF(fixed.x_ref.value(), fixed.y_ref.value(), fixed.w, fixed.h).intersects(
                QRectF(moved.x_ref.value(), moved.y_ref.value(), moved.w, moved.h)));
        REQUIRE(!QRectF(fixed.x, fixed.y, fixed.w, fixed.h).intersects(QRectF(moved.x, moved.y, moved.w, moved.h)));

        REQUIRE(stats.unresolvable == 0);
    }

    SECTION("adaptive: hysteresis, a label not decluttered before rejoins only below the reduced threshold")
    {
        std::vector<double> hx, hy;
        for (const auto& l : labels)
        {
            hx.push_back(l.x_ref.value());
            hy.push_back(l.y_ref.value());
        }

        const auto crowding = label_placement::constraint::localCrowding(labels,
                                                                         hx,
                                                                         hy,
                                                                         2.0,
                                                                         2.0,
                                                                         settings.cs_local_scales,
                                                                         settings.cs_local_reach);

        //last chain label between the reduced and the full threshold
        const double d = crowding.back();
        REQUIRE(d > 0.0);

        settings.cs_adaptive          = true;
        settings.cs_max_local_density = d / 0.9;
        settings.cs_local_hysteresis  = 0.8;

        auto run = [ & ] (bool excluded_last)
        {
            auto ls = labels;
            ls.back().excluded_last = excluded_last;

            REQUIRE(label_placement::constraint::placeLabels(ls, settings));
            return ls.back().excluded;
        };

        REQUIRE(!run(false));
        REQUIRE(run(true));
    }
}

TEST_CASE("LabelPlacementEngine label lookup after removal", "[labelplacement]")
{
    LabelPlacementEngine engine;

    engine.addLabel("a", 0.0, 0.0, 10.0, 10.0);
    engine.addLabel("b", 1.0, 0.0, 10.0, 10.0);
    engine.addLabel("c", 2.0, 0.0, 10.0, 10.0);

    engine.removeLabel("a");

    REQUIRE(!engine.getLabel("a"));
    REQUIRE(engine.getLabel("b"));
    REQUIRE(engine.getLabel("b")->id == "b");
    REQUIRE(engine.getLabel("c"));
    REQUIRE(engine.getLabel("c")->id == "c");

    engine.removeLabel(size_t(0)); // b

    REQUIRE(!engine.getLabel("b"));
    REQUIRE(engine.getLabel("c"));
    REQUIRE(engine.getLabel("c")->id == "c");
    REQUIRE(engine.getLabel(0).id == "c");
}

TEST_CASE("Label placement helpers", "[labelplacement]")
{
    SECTION("data frame of negative coordinates")
    {
        label_placement::Label a;
        a.x        = -50.0;
        a.y        = -40.0;
        a.x_anchor = -60.0;
        a.y_anchor = -50.0;

        label_placement::Label b;
        b.x        = -20.0;
        b.y        = -10.0;
        b.x_anchor = -30.0;
        b.y_anchor = -20.0;

        QRectF frame = label_placement::dataFrame({ a, b });

        REQUIRE(frame.left()   == Approx(-60.0));
        REQUIRE(frame.top()    == Approx(-50.0));
        REQUIRE(frame.right()  == Approx(-20.0));
        REQUIRE(frame.bottom() == Approx(-10.0));
    }

    SECTION("boxes are put back within the roi at all edges")
    {
        const QRectF roi(0.0, 0.0, 100.0, 100.0);

        QRectF right = label_placement::force_exact::putWithinBounds(QRectF(90.0, 20.0, 20.0, 10.0), roi);
        REQUIRE(right.left()  == Approx(80.0));
        REQUIRE(right.width() == Approx(20.0));
        REQUIRE(right.top()   == Approx(20.0));

        QRectF left = label_placement::force_exact::putWithinBounds(QRectF(-5.0, 20.0, 20.0, 10.0), roi);
        REQUIRE(left.left() == Approx(0.0));

        QRectF bottom = label_placement::force_exact::putWithinBounds(QRectF(20.0, 95.0, 20.0, 10.0), roi);
        REQUIRE(bottom.top() == Approx(90.0));

        QRectF top = label_placement::force_exact::putWithinBounds(QRectF(20.0, -5.0, 20.0, 10.0), roi);
        REQUIRE(top.top() == Approx(0.0));
    }
}
