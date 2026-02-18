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
#include "labelplacement_helpers.h"
#include "logger.h"

#include <vector>
#include <cmath>

#include <boost/optional.hpp>

#include <Eigen/Core>

namespace label_placement
{

//helpers for the exact force-based approach.
//see: https://github.com/slowkow/ggrepel
namespace force_exact
{
    struct Circle
    {
        Circle() {}
        Circle(const QPointF& p, double r) : pos(p), radius(r) {}
        Circle(double x, double y, double r) : pos(x, y), radius(r) {}

        QPointF pos;
        double  radius;
    };

    /**
     */
    bool intersectCircleRect(const Circle& circle, const QRectF& rect) 
    {
        const double c_x  = circle.pos.x();
        const double r_x  = (rect.left() + rect.right()) / 2;
        const double r_wh = std::fabs(rect.left() - r_x);
        const double cx   = std::fabs(c_x - r_x);
        const double xd   = r_wh + circle.radius;
        if (cx > xd)
            return false;
        
        const double c_y  = circle.pos.y();
        const double r_y  = (rect.top() + rect.bottom()) / 2;
        const double r_hh = std::fabs(rect.top() - r_y);
        const double cy   = std::fabs(c_y - r_y);
        const double yd   = r_hh + circle.radius;
        if (cy > yd)
            return false;
        
        if (cx <= r_wh || cy <= r_hh)
            return true;

        const double x_corner_dist     = cx - r_wh;
        const double y_corner_dist     = cy - r_hh;
        const double x_corner_dist_sqr = x_corner_dist * x_corner_dist;
        const double y_corner_dist_sqr = y_corner_dist * y_corner_dist;
        const double max_d_sqr         = circle.radius * circle.radius;

        return (x_corner_dist_sqr + y_corner_dist_sqr <= max_d_sqr);
    }

    /**
     */
    QPointF intersectLineCircle(const QPointF& line_pos, const Circle& circle) 
    {
        // 0 = x, 1 = y
        double theta = std::atan2(line_pos.y() - circle.pos.y(), line_pos.x() - circle.pos.x());
        const double x = circle.pos.x() + circle.radius * std::cos(theta);
        const double y = circle.pos.y() + circle.radius * std::sin(theta);

        return QPointF(x, y);
    }

    /**
     */
    QPointF intersectLineRectangle(const QPointF& p1, const QPointF& p2, const QRectF& rect)
    {
        const double dy = p2.y() - p1.y();
        const double dx = p2.x() - p1.x();
        const double k  = dy / dx;
        const double d  = p2.y() - p2.x() * k;

        Eigen::Vector2d p1e(p1.x(), p1.y());
        Eigen::Vector2d isec_min(0, 0);
        double dmin = std::numeric_limits<double>::max();
        double x, y;

        auto checkIsec = [&] (double x, double y) 
        {
            const double d = (p1e - Eigen::Vector2d(x, y)).squaredNorm();

            if (d < dmin)
            {
                dmin = d;
                isec_min[ 0 ] = x;
                isec_min[ 1 ] = y;
            }
        };

        //   +----------+ < b[3]
        //   |          |
        //   |          | < y
        //   |          |
        //   +----------+ < b[1]
        //   ^    ^     ^
        //  b[0]  x    b[2]

        if (dx != 0) 
        {
            // Left boundary
            x = rect.left();
            y = dy == 0 ? p1.y() : k * x + d;
            if (rect.top() <= y && y <= rect.bottom()) 
                checkIsec(x, y);

            // Right boundary
            x = rect.right();
            y = dy == 0 ? p1.y() : k * x + d;
            if (rect.top() <= y && y <= rect.bottom()) 
                checkIsec(x, y);
        }

        if (dy != 0) 
        {
            // Bottom boundary
            y = rect.top();
            x = dx == 0 ? p1.x() : (y - d) / k;
            if (rect.left() <= x && x <= rect.right()) 
                checkIsec(x, y);

            // Top boundary
            y = rect.bottom();
            x = dx == 0 ? p1.x() : (y - d) / k;
            if (rect.left() <= x && x <= rect.right()) 
                checkIsec(x, y);
        }

        return QPointF(isec_min.x(), isec_min.y());
    }

    /**
     */
    QPointF selectLineConnection(const QPointF& p, const QRectF& rect)
    {
        Eigen::Vector2d out;

        // Find shortest path
        //   +----------+ < b[3]
        //   |          |
        //   |          |
        //   |          |
        //   +----------+ < b[1]
        //   ^          ^
        //  b[0]      b[2]

        bool top    = false;
        bool left   = false;
        bool right  = false;
        bool bottom = false;

        if ((p.x() >= rect.left()) & (p.x() <= rect.right())) 
        {
            out[ 0 ] = p.x();
        } 
        else if (p.x() > rect.right()) 
        {
            out[ 0 ] = rect.right();
            right    = true;
        } 
        else
        {
            out[ 0 ] = rect.left();
            left     = true;
        }

        if ((p.y() >= rect.top()) & (p.y() <= rect.bottom())) 
        {
            out[ 1 ] = p.y();
        } 
        else if (p.y() > rect.bottom()) 
        {
            out[ 1 ] = rect.bottom();
            bottom = true;
        } 
        else
        {
            out[ 1 ] = rect.top();
            top      = true;
        }

        Eigen::Vector2d pe(p.x(), p.y());

        // Nudge to center
        const double midx = (rect.left() + rect.right() ) * 0.5;
        const double midy = (rect.top()  + rect.bottom()) * 0.5;
        const double d    = (out - pe).norm();

        if ((top || bottom) && !(left || right)) 
        {
            // top or bottom
            const double altd = (pe - Eigen::Vector2d(midx, out[ 1 ])).norm();
            out[ 0 ] = out[ 0 ] + (midx - out[ 0 ]) * d / altd;
        } 
        else if ((left || right) && !(top || bottom)) 
        {
            // left or right
            const double altd = (pe - Eigen::Vector2d(out[ 0 ], midy)).norm();
            out[ 1 ] = out[ 1 ] + (midy - out[ 1 ]) * d / altd;
        } 
        else if ((left || right) && (top || bottom)) 
        {
            const double altd1 = (pe - Eigen::Vector2d(midx, out[ 1 ])).norm();
            const double altd2 = (pe - Eigen::Vector2d(out[ 0 ], midy)).norm();
            
            if (altd1 < altd2)
                out[ 0 ] = out[ 0 ] + (midx - out[ 0 ]) * d / altd1;
            else
                out[ 1 ] = out[ 1 ] + (midy - out[ 1 ]) * d / altd2;
        }

        return QPointF(out[ 0 ], out[ 1 ]);
    }

    /**
     */
    bool approxEqual(double x1, double x2) 
    {
        return std::fabs(x2 - x1) < (std::numeric_limits<double>::epsilon() * 100);
    }

    /**
     */
    bool intersectLineLine(const QPointF& p1, const QPointF& p2, const QPointF& q1, const QPointF& q2) 
    {
        // Special exception, where q1 and q2 are equal (do intersect)
        if (q1.x() == q2.x() && q1.y() == q2.y())
            return false;
        // If line is point
        if (p1.x() == q1.x() && p1.y() == q1.y())
            return false;
        if (p2.x() == q2.x() && p2.y() == q2.y())
            return false;

        const double dy1 = q1.y() - p1.y();
        const double dx1 = q1.x() - p1.x();

        const double slope1     = dy1 / dx1;
        const double intercept1 = q1.y() - q1.x() * slope1;

        const double dy2 = q2.y() - p2.y();
        const double dx2 = q2.x() - p2.x();

        const double slope2     = dy2 / dx2;
        const double intercept2 = q2.y() - q2.x() * slope2;

        double x,y;

        // check if lines vertical
        if (approxEqual(dx1,0.0)) 
        {
            if (approxEqual(dx2,0.0)) 
            {
                return false;
            } 
            else 
            {
                x = p1.x();
                y = slope2 * x + intercept2;
            }
        } 
        else if (approxEqual(dx2,0.0)) 
        {
            x = p2.x();
            y = slope1 * x + intercept1;
        } 
        else 
        {
            if (approxEqual(slope1,slope2)) 
            {
                return false;
            }
            x = (intercept2 - intercept1) / (slope1 - slope2);
            y = slope1 * x + intercept1;
        }

        if (x < p1.x() && x < q1.x()) 
        {
            return false;
        } 
        else if (x > p1.x() && x > q1.x()) 
        {
            return false;
        } 
        else if (y < p1.y() && y < q1.y()) 
        {
            return false;
        } 
        else if (y > p1.y() && y > q1.y()) 
        {
            return false;
        } 
        else if (x < p2.x() && x < q2.x()) 
        {
            return false;
        } 
        else if (x > p2.x() && x > q2.x()) 
        {
            return false;
        } 
        else if (y < p2.y() && y < q2.y()) 
        {
            return false;
        } 
        else if (y > p2.y() && y > q2.y()) 
        {
            return false;
        } 

        return true;
    }

    /**
     */
    QRectF putWithinBounds(const QRectF& rect, const QRectF& roi, double force = 1e-5) 
    {
        double x0 = rect.left();
        double x1 = rect.right();
        double y0 = rect.top();
        double y1 = rect.bottom();

        const double width  = rect.width();
        const double height = rect.height();

        if (x0 < roi.left()) 
        {
            x0 = roi.left();
            x1 = x0 + width;
        } 
        else if (x1 > rect.right()) 
        {
            x1 = rect.right();
            x0 = x1 - width;
        }
        if (y0 < roi.top()) 
        {
            y0 = roi.top();
            y1 = y0 + height;
        } 
        else if (y1 > roi.bottom()) 
        {
            y1 = roi.bottom();
            y0 = y1 - height;
        }
        return QRectF(x0, y0, x1 - x0, y1 - y0);
    }

    /**
     */
    Eigen::Vector2d repelForceXY(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double force = 0.000001) 
    {
        const double dx = std::fabs(a[ 0 ] - b[ 0 ]);
        const double dy = std::fabs(a[ 1 ] - b[ 1 ]);

        // Constrain the minimum distance, so it is never 0.
        const double d2 = std::max(dx * dx + dy * dy, 0.0004);

        // Compute a unit vector in the direction of the force.
        Eigen::Vector2d v = (a - b) / std::sqrt(d2);
        // Divide the force by the squared distance.
        Eigen::Vector2d f = force * v / d2;
        if (dx > dy) 
        {
            // f.y = f.y * dx / dy;
            f[ 1 ] *= 2;
        } 
        else 
        {
            // f.x = f.x * dy / dx;
            f[ 0 ] *= 2;
        }
        return f;
    }


    /**
     */
    Eigen::Vector2d repelForceY(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double force = 0.000001) 
    {
        const double dx = std::fabs(a[ 0 ] - b[ 0 ]);
        const double dy = std::fabs(a[ 1 ] - b[ 1 ]);

        // Constrain the minimum distance, so it is never 0.
        const double d2 = std::max(dx * dx + dy * dy, 0.0004);

        // Compute a unit vector in the direction of the force.
        Eigen::Vector2d v = {0,1};
        if (a[ 1 ] < b[ 1 ]) 
        {
            v[ 1 ] = -1;
        }
        // Divide the force by the distance.
        Eigen::Vector2d f = force * v / d2 * 2;
        return f;
    }

    /**
     */
    Eigen::Vector2d repelForceX(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double force = 0.000001) 
    {
        const double dx = std::fabs(a[ 0 ] - b[ 0 ]);
        const double dy = std::fabs(a[ 1 ] - b[ 1 ]);

        // Constrain the minimum distance, so it is never 0.
        const double d2 = std::max(dx * dx + dy * dy, 0.0004);

        // Compute a unit vector in the direction of the force.
        Eigen::Vector2d v = {1,0};
        if (a[ 0 ] < b[ 0 ]) 
        {
            v[ 0 ] = -1;
        }
        // Divide the force by the squared distance.
        Eigen::Vector2d f = force * v / d2 * 2;
        return f;
    }

    /**
     */
    QPointF repelForce(const QPointF& a, const QPointF& b, double force = 0.000001, ForceDirection direction = ForceDirection::XY) 
    {
        Eigen::Vector2d out;
        if (direction == ForceDirection::X) 
        {
            out = repelForceX(Eigen::Vector2d(a.x(), a.y()), Eigen::Vector2d(b.x(), b.y()), force);
        } 
        else if (direction == ForceDirection::Y) 
        {
            out = repelForceY(Eigen::Vector2d(a.x(), a.y()), Eigen::Vector2d(b.x(), b.y()), force);
        } 
        else //ForceDirection::XY
        {
            out = repelForceXY(Eigen::Vector2d(a.x(), a.y()), Eigen::Vector2d(b.x(), b.y()), force);
        }

        if (out[ 0 ] != out[ 0 ] || out[ 1 ] != out[ 1 ])
        {
            std::stringstream ss;
            ss << "repelForce: force is nan between (" << a.x() << "," << a.y() << ") and (" << b.x() << "," << b.y() << ") @force " << force << std::endl;
            throw std::runtime_error(ss.str().c_str());
        }
        
        return QPointF(out[ 0 ], out[ 1 ]);
    }

    /**
     */
    Eigen::Vector2d springForceXY(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double force = 0.000001) 
    {
        Eigen::Vector2d f = {0, 0};
        Eigen::Vector2d v = (a - b) ;
        f = force * v;
        return f;
    }

    /**
     */
    Eigen::Vector2d springForceY(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double force = 0.000001) 
    {
        Eigen::Vector2d f = {0, 0};
        Eigen::Vector2d v = {0, (a.y() - b.y())};
        f = force * v;
        return f;
    }
   
    /**
     */
    Eigen::Vector2d springForceX(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double force = 0.000001) 
    {
        Eigen::Vector2d f = {0, 0};
        Eigen::Vector2d v = {(a.x() - b.x()), 0};
        f = force * v ;
        return f;
    }

    /**
     */
    QPointF springForce(const QPointF& a, const QPointF& b, double force = 0.000001, ForceDirection direction = ForceDirection::XY) 
    {
        Eigen::Vector2d out;
        if (direction == ForceDirection::X) 
        {
            out = springForceX(Eigen::Vector2d(a.x(), a.y()), Eigen::Vector2d(b.x(), b.y()), force);
        } 
        else if (direction == ForceDirection::Y) 
        {
            out = springForceY(Eigen::Vector2d(a.x(), a.y()), Eigen::Vector2d(b.x(), b.y()), force);
        } 
        else //ForceDirection::XY
        {
            out = springForceXY(Eigen::Vector2d(a.x(), a.y()), Eigen::Vector2d(b.x(), b.y()), force);
        }

        if (out[ 0 ] != out[ 0 ] || out[ 1 ] != out[ 1 ])
        {
            std::stringstream ss;
            ss << "springForce: force is nan between (" << a.x() << "," << a.y() << ") and (" << b.x() << "," << b.y() << ") @force " << force << std::endl;
            throw std::runtime_error(ss.str().c_str());
        }

        return QPointF(out[ 0 ], out[ 1 ]);
    }

    /**
     */
    void rescale(std::vector<double>& v) 
    {
        const double min_value = *std::min_element(v.begin(), v.end());
        const double max_value = *std::max_element(v.begin(), v.end());

        if (max_value - min_value < 1e-07)
        {
            for(double& val : v)
                val /= max_value; 
        }
        else
        {
            for(double& val : v)
                val = (val - min_value) / (max_value - min_value);
        }       
    }

    /**
     */
    bool placeLabels(std::vector<Label>& labels,
                     const Settings& settings)
    {
        size_t nl = labels.size();

        if (nl < 1)
            return true;

        //get data frame for normalization.
        //Note: for this method to work, all data is internally normalized.
        //Not doing the normalization would result in range-dependent effects!
        auto data_frame = dataFrame(labels);

        const double anchor_radius = settings.fbe_anchor_radius;
   
        //some magic numbers
        const double force_point_size    = 100; // Larger data points push text away with greater force.
        const double velocity_decay      = 0.7;
        const double force_push_decay    = 0.99999;
        const double force_pull_decay    = 0.9999;
        //const double line_clash_bump     = 1.25;
        //const int    line_clash_interval = 5;

        //init anchor points
        std::vector<QPointF> anchor_points(nl);
        std::vector<Circle>  anchor_regions(nl);
        for (size_t i = 0; i < nl; ++i) 
        {
            const auto& l = labels[ i ];

            anchor_regions[ i ].pos    = QPointF(l.x_anchor, l.y_anchor);
            anchor_regions[ i ].radius = anchor_radius;
            anchor_points [ i ]        = anchor_regions[ i ].pos;

            //normalize anchor data
            normalizeData(anchor_regions[ i ].pos, data_frame);
            normalizeData(anchor_regions[ i ].radius, data_frame);
            normalizeData(anchor_points [ i ], data_frame);   
        }

        //init text boxes
        std::vector<QRectF>  text_boxes(nl);
        std::vector<double>  text_box_widths(nl, 0.0);
        std::vector<boost::optional<QPointF>> sticky_positions(nl);
        for (size_t i = 0; i < nl; ++i) 
        {
            const auto& l = labels[ i ];

            text_boxes     [ i ] = l.boundingBox();
            text_box_widths[ i ] = l.w;

            boost::optional<QPointF> ref_pos;
            if (l.x_ref.has_value() && l.y_ref.has_value())
                ref_pos = QPointF(l.x_ref.value(), l.y_ref.value());

            //normalize text box data
            normalizeData(text_boxes[ i ], data_frame);
            normalizeData(text_box_widths[ i ], data_frame);
            if (ref_pos.has_value())
                normalizeData(ref_pos.value(), data_frame);

            // add a tiny bit of jitter to each text box at the start.
            // don't add jitter if the user wants to repel in just one direction.
            const double jitter = Eigen::Vector2d().setRandom().x() * settings.fbe_force_push;
            if (settings.fbe_force_dir != ForceDirection::Y) 
                text_boxes[ i ].translate(jitter, 0);
            if (settings.fbe_force_dir != ForceDirection::X) 
                text_boxes[ i ].translate(0, jitter);

            //choose appropraite 'sticky position'
            if (settings.fbe_sticky_pos == StickyPosition::InitPos)
            {
                sticky_positions[ i ] = text_boxes[ i ].center();
            }          
            else if (settings.fbe_sticky_pos == StickyPosition::Anchor)
            {
                sticky_positions[ i ] = anchor_points[ i ];
            }        
            else if (settings.fbe_sticky_pos == StickyPosition::RefPos && ref_pos.has_value())
            {
                sticky_positions[ i ] = ref_pos.value();
            }
        }

        //rescale box widths to be in the range [0,1]
        rescale(text_box_widths);

        //normalize anchor radius
        double anchor_radius_frame = anchor_radius;
        normalizeData(anchor_radius_frame, data_frame);

        //normalize roi
        QRectF roi = settings.roi;
        if (!roi.isEmpty())
            normalizeData(roi, data_frame);

        QPointF f, ci, cj;

        std::vector<QPointF> velocities(nl, QPointF(0, 0));
        std::vector<double>  total_overlaps(nl, 0);
        std::vector<bool>    too_many_overlaps(nl, false);

        int  iter       = 0;
        int  n_overlaps = 1;
        bool i_overlaps = true;

        double force_push = settings.fbe_force_push;
        double force_pull = settings.fbe_force_pull;

        int too_many_overlaps_happened = 0;
        int overlaps_detected          = 0;

        while (n_overlaps && iter < settings.fbe_max_iter) 
        {
            iter      += 1;
            n_overlaps = 0;

            // The forces get weaker over time.
            force_push *= force_push_decay;
            force_pull *= force_pull_decay;

            for (size_t i = 0; i < nl; ++i) 
            {
                try
                {
                    if (iter == 2 && total_overlaps[ i ] > settings.fbe_max_overlaps) 
                    {
                        too_many_overlaps[ i ] = true;
                        ++too_many_overlaps_happened;
                    }
        
                    if (too_many_overlaps[ i ])
                        continue;
            
                    // Reset overlaps for the next iteration
                    total_overlaps[ i ] = 0;

                    i_overlaps = false;
                    f = QPointF(0, 0);

                    ci = text_boxes[ i ].center();

                    for (size_t j = 0; j < nl; ++j) 
                    {
                        if (i == j) 
                        {
                            // Skip the data points if the size and padding is 0.
                            if (anchor_radius == 0)
                                continue;
                            
                            // Repel the box from its data point.
                            if (intersectCircleRect(anchor_regions[ i ], text_boxes[ i ])) 
                            {
                                overlaps_detected   += 1;
                                n_overlaps          += 1;
                                i_overlaps           = true;
                                total_overlaps[ i ] += 1;

                                f = f + repelForce(ci, anchor_points[ i ], anchor_radius_frame * force_point_size * force_push, settings.fbe_force_dir);
                            }
                        } 
                        else if (too_many_overlaps[j]) 
                        {
                            // Skip the data points if the size and padding is 0.
                            if (anchor_radius == 0) 
                                continue;
                            
                            // Repel the box from other data points.
                            if (intersectCircleRect(anchor_regions[ j ], text_boxes[ i ])) 
                            {
                                overlaps_detected   += 1;
                                n_overlaps          += 1;
                                i_overlaps           = true;
                                total_overlaps[ i ] += 1;

                                f = f + repelForce(ci, anchor_points[ j ], anchor_radius_frame * force_point_size * force_push, settings.fbe_force_dir);
                            }
                        } 
                        else 
                        {
                            cj = text_boxes[ j ].center();

                            // Repel the box from overlapping boxes.
                            if (text_boxes[ i ].intersects(text_boxes[ j ])) 
                            {
                                overlaps_detected   += 1;
                                n_overlaps          += 1;
                                i_overlaps           = true;  
                                total_overlaps[ i ] += 1;

                                f = f + repelForce(ci, cj, force_push, settings.fbe_force_dir);
                            }
                            
                            // Skip the data points if the size and padding is 0.
                            if (anchor_radius == 0)
                                continue;
                            
                            // Repel the box from other data points.
                            if (intersectCircleRect(anchor_regions[ j ], text_boxes[ i ])) 
                            {
                                overlaps_detected   += 1;
                                n_overlaps          += 1;
                                i_overlaps           = true;
                                total_overlaps[ i ] += 1;

                                f = f + repelForce(ci, anchor_points[ j ], anchor_radius_frame * force_point_size * force_push, settings.fbe_force_dir);
                            }
                        }
                    }

                    //std::cout << "force" << i << ": (" << f.x() << "," << f.y() << ")" << std::endl;

                    // pull the box toward its sticky position.
                    if (!i_overlaps && sticky_positions[ i ].has_value()) 
                    {
                        auto df = springForce(sticky_positions[ i ].value(), ci, force_pull, settings.fbe_force_dir);
                        f = f + df;             
                    }
                    
                    double overlap_multiplier = 1.0;
                    if (total_overlaps[ i ] > 10)
                        overlap_multiplier += 0.5;
                    else
                        overlap_multiplier += 0.05 * total_overlaps[ i ];

                    if (f.x() != f.x() || f.y() != f.y())
                    {
                        std::stringstream ss;
                        ss << "force is nan for label" << i << std::endl;
                        throw std::runtime_error(ss.str().c_str());
                    }

                    auto v_old = velocities[ i ];
                    velocities[ i ] = overlap_multiplier * velocities[ i ] * (text_box_widths[ i ] + 1e-6) * velocity_decay + f;



                    if (velocities[ i ].x() != velocities[ i ].x() || velocities[ i ].y() != velocities[ i ].y())
                    {
                        std::stringstream ss;
                        ss << "velocity is nan for label" << i << ": mult=" << overlap_multiplier << ", boxw=" << text_box_widths[ i ] << ", decay=" << velocity_decay << ", v_old=" << v_old.x() << "," << v_old.y() << std::endl;
                        throw std::runtime_error(ss.str().c_str());
                    }

                    text_boxes[ i ].translate(velocities[ i ]);

                    // put text boxes back within roi if specified
                    if (!roi.isEmpty())
                        text_boxes[ i ] = putWithinBounds(text_boxes[ i ], roi);

                    // look for line clashes
                    // if (n_overlaps == 0 || iter % line_clash_interval == 0) 
                    // {
                    //     for (size_t j = 0; j < nl; ++j) 
                    //     {             
                    //         ci = text_boxes[ i ].center();
                    //         cj = text_boxes[ j ].center();

                    //         // switch label positions if lines overlap
                    //         if (i != j && intersectLineLine(ci, anchor_points[ i ], cj, anchor_points[ j ])) 
                    //         {
                    //             n_overlaps += 1;

                    //             QPointF fi = springForce(cj, ci, 1.0, settings.sb_force_dir);
                    //             QPointF fj = springForce(ci, cj, 1.0, settings.sb_force_dir);

                    //             text_boxes[ i ].translate(fi);
                    //             text_boxes[ j ].translate(fj);

                    //             // Check if resolved
                    //             ci = text_boxes[ i ].center();
                    //             cj = text_boxes[ j ].center();

                    //             if (intersectLineLine(ci, anchor_points[ i ], cj, anchor_points[ j ])) 
                    //             {
                    //                 QPointF fi = springForce(cj, ci, line_clash_bump, settings.sb_force_dir);
                    //                 QPointF fj = springForce(ci, cj, line_clash_bump, settings.sb_force_dir);

                    //                 text_boxes[ i ].translate(fi);
                    //                 text_boxes[ j ].translate(fj);
                    //             }
                    //         }
                    //     }
                    // }
                }
                catch (const std::runtime_error& err)
                {
                    logerr << "encountered error in iteration " << iter << " @label" << i << ": " << err.what();
                    return false;
                }
            } // loop through all text labels
        } // while any overlaps exist and we haven't reached max iterations

        if (settings.verbose)
        {
            bool converged = (iter < settings.fbe_max_iter);
            loginf << (converged ? "CONVERGED" : "MAX ITER") << " remaining overlaps: " << n_overlaps << ", too many overlaps: " << too_many_overlaps_happened << ", overlaps detected: " << overlaps_detected;
        }

        //write optimized positions to labels
        for (size_t i = 0; i < nl; ++i) 
        {
            QPointF opt = text_boxes[ i ].topLeft();
            denormalizeData(opt, data_frame);

            labels[ i ].x = opt.x();
            labels[ i ].y = opt.y();

            //std::cout << "label" << i << " moved to " << labels[ i ].x << "," << labels[ i ].y << std::endl;
        }

        return true;
    }

} //namespace force_exact
} //namespace label_placement
