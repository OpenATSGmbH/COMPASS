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

#include <string>
#include <vector>

#include <QRectF>
#include <QPointF>

#include <boost/optional.hpp>


namespace label_placement
{
    struct Label
    {
        /**
         * Revert current to initial position.
         */
        void revert()
        {
            x = x_last;
            y = y_last;
        }

        /**
         * Check if the label is properly initialized.
         */
        bool isInit() const 
        {
            return (w > 0 && h > 0 && !id.empty());
        }

        /**
         * Compute (scaled) bounding box.
         */
        QRectF boundingBox(double tx = 1.0, double ty = 1.0) const
        {
            if (w < 0 || h < 0)
                return QRectF();

            QRectF bbox(x, y, w, h);
            if (tx != 1 || ty != 1)
            {
                double wnew = w * tx;
                double hnew = h * ty;
                double dx = wnew - w;
                double dy = hnew - h;
                dx /= 2;
                dy /= 2;
                bbox = QRectF(x - dx, y - dy, wnew, hnew);
            }

            return bbox;
        }

        /**
         * Compute the center of the label's bounding box.
         */
        QPointF midPoint() const
        {
            return QPointF(x + w / 2, y + h / 2);
        }

        std::string id;

        double      x_anchor = 0; //x pos of the position the label is attached to
        double      y_anchor = 0; //y pos of the position the label is attached to
        double      x        = 0; //current x pos of the label (top left)
        double      y        = 0; //current y pos of the label (top left)
        double      x_last   = 0; //initial x pos of the label (top left)
        double      y_last   = 0; //initial y pos of the label (top left)
        double      w        = 0; //label width
        double      h        = 0; //Label height

        boost::optional<double> x_ref; //a reference x position, which is deemed 'optimal' and MIGHT get enforced by a label algorithm
        boost::optional<double> y_ref; //a reference y position, which is deemed 'optimal' and MIGHT get enforced by a label algorithm

        bool has_last = true; //initial position is a previous placement (false: no history, e.g. a new label)

        bool excluded_last = false; //not decluttered in the previous run (constraint based method, adaptive: hysteresis)
        bool excluded      = false; //result: not decluttered, stays at home (constraint based method: overcrowded)
    };

    enum class Method
    {
        ForceBasedSimple = 0, //simple force based approach, just repels into ANY direction to avoid label conflicts
        ForceBasedExtended,   //extended force based approach, tries to estimate the correct repel force directions
        ForceBasedExact,      //exact approach, computes the correct repel force directions using intersections
        ConstraintBased,      //minimal displacement from the reference positions subject to non-overlap constraints
        ForceBasedRelaxed     //extended force based approach, starting from the initial positions moved partly towards
                              //the reference positions (see 'fb_start_relax')
    };

    enum ForceDirection
    {
        X = 0, //compute forces into x direction only
        Y,     //compute forces into y direction only
        XY     //compute forces into x and y directions
    };

    enum StickyPosition
    {
        None = 0, //do not enforce
        InitPos,  //enforce sticking to the initial position of the label
        RefPos,   //enforce sticking to the desired reference position of the label (if available)
        Anchor    //enforce sticking to the anchor position of the label
    };

    struct Settings
    {
        typedef std::vector<QRectF> Objects;

        Method method = Method::ConstraintBased;  //used label placement approach
        QRectF roi;                               //region of interest, MIGHT be enforced by a specific labeling algorithm

        //settings for methods 'ForceBasedSimple', 'ForceBasedExtended' and 'ForceBasedRelaxed'
        int       fb_max_iter        = 500;   //maximum number of iterations
        double    fb_precision       = 0.001;  //precision = convergence limit
        double    fb_expand_x        = 1.2;   //bounding box expansion in x direction (TODO: per avoidance type)
        double    fb_expand_y        = 1.1;   //bounding box expansion in y direction (TODO: per avoidance type)
        bool      fb_avoid_labels    = true;  //avoid other labels (should be on)
        bool      fb_avoid_anchors   = false; //avoid anchor locations (= positions the labels are attached to)
        bool      fb_avoid_roi       = false; //avoid region of interest specified in 'roi'
        double    fb_weight_labels   = 0.2;   //force weight for label avoidance (TODO: separate x and y part)
        double    fb_weight_anchors  = 0.4;   //force weight for anchor avoidance (TODO: separate x and y part)
        double    fb_weight_objects  = 0.2;   //force weight for object avoidance (TODO: separate x and y part)
        double    fb_weight_roi      = 0.2;   //force weight for roi avoidance (TODO: separate x and y part)
        double    fb_anchor_radius   = 0.0;   //repel radius around anchors if 'fb_avoid_anchors' is on
        double    fb_weight_ref      = 0.0;   //pull towards the reference position (x_ref, y_ref) while a label is not in conflict (0 = off)
        double    fb_weight_init     = 0.0;   //pull towards the initial position (x_last, y_last) while a label is not in conflict (0 = off)
        double    fb_max_ref_dist    = 0.0;   //max distance of a label from its reference position (0 = unlimited)
    double    fb_start_relax     = 0.2;   //method 'ForceBasedRelaxed': fraction the initial position is moved towards the
                                          //  reference position before the run

        //settings for method 'ForceBasedExact'
        int            fbe_max_iter      = 10000;
        double         fbe_force_push    = 1e-06; //defined for normalized data in [0,1]!
        double         fbe_force_pull    = 1e-06; //defined for normalized data in [0,1]!
        int            fbe_max_overlaps  = 10000; 
        double         fbe_anchor_radius = 0.0;
        ForceDirection fbe_force_dir     = ForceDirection::XY;
        StickyPosition fbe_sticky_pos    = StickyPosition::InitPos;

        //settings for method 'ConstraintBased' (target: reference position, previous placement: initial position;
        //defaults = settings of the geographic view's label overlay)
        double cs_gap_x         = 2.0;   //min horizontal gap between labels and between labels and obstacles
        double cs_gap_y         = 2.0;   //min vertical gap between labels and between labels and obstacles
        double cs_balance       = 0.5;   //separation direction by the needed displacement divided by size^cs_balance of the
                                         //  pair along the axis (0 = absolute => wide labels stack vertically, 1 = relative)
        int    cs_candidates    = 8;     //candidate positions around the anchor (at the anchor distance of the reference
                                         //  position): 1 = reference position only, 4 = diagonal, 8 = also axis-aligned
        double cs_cand_penalty  = 0.0;   //cost of leaving the reference position, fraction of the label area
                                         //  (0 = all directions equal)
        double cs_cand_switch   = 0.5;   //cost of leaving the candidate of the previous placement, fraction of the label area
        int    cs_cand_sweeps   = 10;    //max sweeps of the candidate choice
        bool   cs_avoid_anchors = true;  //anchors are obstacles (squares of 2 * cs_anchor_radius)
        double cs_anchor_radius = 10.0;  //obstacle radius around anchors (0 = anchors not avoided)
        bool   cs_keep_in_roi   = true;  //keep labels within the region of interest 'roi' (if set)
        double cs_keep_factor   = 2.0;   //the previous separation direction of a pair is kept unless it needs more than
        double cs_keep_abs      = 10.0;  //  cs_keep_factor * cheapest + cs_keep_abs separation
        double cs_prev_tolerance= 0.5;   //previous placements count as separated up to this overlap, as fraction of the
                                         //  smaller label size along the axis (tight solutions, relative movement)
        double cs_soft_weight   = 100.0; //cost weight of violating an obstacle or the region of interest (labels: hard)
        double cs_object_weight = 1e4;   //cost weight of violating an additional object (fixed labels => practically hard)
        int    cs_max_rounds    = 20;    //max rounds of constraint generation (new overlaps after solving)
        double cs_weight_prev   = 1.0;   //weight of the distance from the previous placement relative to the distance
                                         //  from home (0 = off; smoothness between runs, costs locality)
        double cs_max_time_ms   = 100.0; //runtime limit in ms (0 = unlimited): when reached, the last complete state is
                                         //  returned (see constraint::placeLabels())
        double cs_max_group_density = 1.0; //groups of labels with overlapping homes above this density (label area relative
                                           //  to the area around their anchors) are not decluttered (0 = off)
        bool   cs_adaptive          = true;  //declutter what is possible: labels in crowded areas (local crowding above
                                             //  cs_max_local_density) stay at home and are fixed obstacles for the
                                             //  others, replaces cs_max_group_density
        double cs_max_local_density = 0.5;   //adaptive: max crowding = label area (gaps included) relative to the area
                                             //  around the label's anchor, incl. the room within cs_local_reach
        double cs_local_hysteresis  = 0.8;   //adaptive: a label not decluttered in the previous run rejoins below this
                                             //  fraction of cs_max_local_density
        int    cs_local_scales      = 4;     //adaptive: windows around the anchor (region of the label's candidate
                                             //  positions, doubled cs_local_scales - 1 times)
        double cs_local_reach       = 2.5;   //adaptive: distance labels may be pushed out of a crowded window, in label
                                             //  sizes (sqrt(w * h))
        bool   cs_spatial_index     = true;  //neighbours and overlaps via an r-tree, otherwise by testing all pairs
                                             //  (same result)

        bool verbose = false;

        bool    avoid_objects = false; //avoid the 'additional_objects' (all methods)
        Objects additional_objects;    //additional fixed objects to avoid (e.g. manually placed labels)
    };
}
