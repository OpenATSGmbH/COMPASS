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
    };

    enum class Method
    {
        ForceBasedSimple = 0, //simple force based approach, just repels into ANY direction to avoid label conflicts
        ForceBasedExtended,   //extended force based approach, tries to estimate the correct repel force directions
        ForceBasedExact       //exact approach, computes the correct repel force directions using intersections
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

        Method method = Method::ForceBasedSimple; //used label placement approach
        QRectF roi;                               //region of interest, MIGHT be enforced by a specific labeling algorithm

        //settings for methods'ForceBasedSimple' and 'ForceBasedExtended'
        int       fb_max_iter        = 500;   //maximum number of iterations
        double    fb_precision       = 0.001;  //precision = convergence limit
        double    fb_expand_x        = 1.2;   //bounding box expansion in x direction (TODO: per avoidance type)
        double    fb_expand_y        = 1.1;   //bounding box expansion in y direction (TODO: per avoidance type)
        bool      fb_avoid_labels    = true;  //avoid other labels (should be on)
        bool      fb_avoid_anchors   = false; //avoid anchor locations (= positions the labels are attached to)
        bool      fb_avoid_objects   = false; //avoid objects added as 'fb_additional_objects'
        bool      fb_avoid_roi       = false; //avoid region of interest specified in 'roi'
        double    fb_weight_labels   = 0.2;   //force weight for label avoidance (TODO: separate x and y part)
        double    fb_weight_anchors  = 0.4;   //force weight for anchor avoidance (TODO: separate x and y part)
        double    fb_weight_objects  = 0.2;   //force weight for object avoidance (TODO: separate x and y part)
        double    fb_weight_roi      = 0.2;   //force weight for roi avoidance (TODO: separate x and y part)
        double    fb_anchor_radius   = 0.0;   //repel radius around anchors if 'fb_avoid_anchors' is on

        //settings for method 'ForceBasedExact'
        int            fbe_max_iter      = 10000;
        double         fbe_force_push    = 1e-06; //defined for normalized data in [0,1]!
        double         fbe_force_pull    = 1e-06; //defined for normalized data in [0,1]!
        int            fbe_max_overlaps  = 10000; 
        double         fbe_anchor_radius = 0.0;
        ForceDirection fbe_force_dir     = ForceDirection::XY;
        StickyPosition fbe_sticky_pos    = StickyPosition::InitPos;

        bool verbose = false;

        Objects additional_objects; //additional objects to avoid, must be enabled by 'fb_avoid_objects'
    };
}
