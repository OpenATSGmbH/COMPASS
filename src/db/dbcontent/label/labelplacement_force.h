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

#include "tbbhack.h"

#include <vector>
#include <iostream>

#include <Eigen/Core>

namespace label_placement
{

//helpers for the force-based approach.
//see: https://github.com/Phlya/adjustText
namespace force
{
    /**
     */
    int sign(double v)
    {
        if (v < 0)
            return -1;
        if (v > 0)
            return 1;
        return 0;
    }

    /**
     */
    QPointF repelFromPosition(const QRectF& bbox, 
                              double x, 
                              double y, 
                              bool simple)
    {
        if (!bbox.contains(x, y))
            return QPointF(0, 0);

        if (simple)
        {
            QPointF c = bbox.center();

            int dir_x = sign(c.x() - x);
            int dir_y = sign(c.y() - y);

            double dx = dir_x == 0 ? 0 : (dir_x == -1 ? x - bbox.right()  : x - bbox.left());
            double dy = dir_y == 0 ? 0 : (dir_y == -1 ? y - bbox.bottom() : y - bbox.top() );

            return {dx, dy};
        }

        QPointF c = bbox.center();

        double dx = c.x() - x;
        double dy = c.y() - y;
        double d_sqr = std::max(dx * dx + dy * dy, 1e-06);
        double d = std::sqrt(d_sqr);
        dx /= d;
        dy /= d;

        const double f = std::max(bbox.width(), bbox.height());

        return {dx * f, dy * f};
    }

    /**
     */
    QPointF repelFromBox(const QRectF& bbox, 
                         const QRectF& bbox2, 
                         bool simple)
    {
        if (!bbox.intersects(bbox2))
        {
            return {0, 0};
        }

        if (simple)
        {
            auto r_isec = bbox & bbox2;
            if (r_isec.isEmpty())
                return {0, 0};

            const double dx = r_isec.width();
            const double dy = r_isec.height();
            const int    sx = sign(bbox.left() - bbox2.left());
            const int    sy = sign(bbox.top()  - bbox2.top() );

            return {dx * sx, dy * sy};
        }

        QPointF c0 = bbox.center();
        QPointF c1 = bbox2.center();

        double dx = c0.x() - c1.x();
        double dy = c0.y() - c1.y();
        double d_sqr = std::max(dx * dx + dy * dy, 1e-06);
        double d = std::sqrt(d_sqr);
        dx /= d;
        dy /= d;

        const double f = std::max(bbox.width(), bbox.height());

        return {dx * f, dy * f};     
    }

    /**
     */
    void repelFromLabels(std::vector<Eigen::Vector2d>& movements,
                         std::vector<Eigen::Vector2d>& totals,
                         const std::vector<Label>& labels, 
                         double tx, 
                         double ty,
                         bool simple)
    {
        /*"""
        Repel texts from each other while expanding their bounding boxes by expand
        (x, y), e.g. (1.2, 1.2) would multiply width and height by 1.2.
        Requires a renderer to get the actual sizes of the text, and to that end
        either one needs to be directly provided, or the axes have to be specified,
        and the renderer is then got from the axes object.
        """
        ax = ax or plt.gca()
        r = renderer or get_renderer(ax.get_figure())
        bboxes = get_bboxes(texts, r, expand, ax=ax)
        xmins = [bbox.xmin for bbox in bboxes]
        xmaxs = [bbox.xmax for bbox in bboxes]
        ymaxs = [bbox.ymax for bbox in bboxes]
        ymins = [bbox.ymin for bbox in bboxes]

        overlaps_x = np.zeros((len(bboxes), len(bboxes)))
        overlaps_y = np.zeros_like(overlaps_x)
        overlap_directions_x = np.zeros_like(overlaps_x)
        overlap_directions_y = np.zeros_like(overlaps_y)
        for i, bbox1 in enumerate(bboxes):
            overlaps = get_points_inside_bbox(
                xmins * 2 + xmaxs * 2, (ymins + ymaxs) * 2, bbox1
            ) % len(bboxes)
            overlaps = np.unique(overlaps)
            for j in overlaps:
                bbox2 = bboxes[j]
                x, y = bbox1.intersection(bbox1, bbox2).size
                overlaps_x[i, j] = x
                overlaps_y[i, j] = y
                direction = np.sign(bbox1.extents - bbox2.extents)[:2]
                overlap_directions_x[i, j] = direction[0]
                overlap_directions_y[i, j] = direction[1]

        move_x = overlaps_x * overlap_directions_x
        move_y = overlaps_y * overlap_directions_y

        delta_x = move_x.sum(axis=1)
        delta_y = move_y.sum(axis=1)

        q = np.sum(overlaps_x), np.sum(overlaps_y)
        if move:
            move_texts(texts, delta_x, delta_y, bboxes, ax=ax)
        return delta_x, delta_y, q*/

        size_t n = labels.size();

        movements.assign(n, Eigen::Vector2d(0, 0));
        totals.assign(n, Eigen::Vector2d(0, 0));

        if (n <= 1)
            return;

        std::vector<QRectF> bboxes = collectBoundingBoxes(labels, tx, ty);

//        for (size_t i = 0; i < n; ++i)
//        {
        tbb::parallel_for(size_t(0), n, [&](size_t i) {
            //const auto& l = labels[ i ];

            const auto& bbox = bboxes[ i ];

            for (size_t j = 0; j < n; ++j)
            {
                if (i == j)
                    continue;

                //const auto& l2 = labels[ j ];

                const auto& bbox2 = bboxes[ j ];

                auto offset = repelFromBox(bbox, bbox2, simple);

                movements[ i ] += Eigen::Vector2d(offset.x(), offset.y());
                totals[ i ]    += Eigen::Vector2d(std::fabs(offset.x()), std::fabs(offset.y()));
            }
          });
//        }
    }

    /**
     */
    void repelFromObjects(std::vector<Eigen::Vector2d>& movements,
                          std::vector<Eigen::Vector2d>& totals,
                          const std::vector<QRectF>& objects,
                          const std::vector<Label>& labels, 
                          double tx, 
                          double ty,
                          bool simple)
    {
        /*"""
        Repel texts from other objects' bboxes while expanding their (texts')
        bounding boxes by expand (x, y), e.g. (1.2, 1.2) would multiply width and
        height by 1.2.
        Requires a renderer to get the actual sizes of the text, and to that end
        either one needs to be directly provided, or the axes have to be specified,
        and the renderer is then got from the axes object.
        """
        ax = ax or plt.gca()
        r = renderer or get_renderer(ax.get_figure())

        bboxes = get_bboxes(texts, r, expand, ax=ax)

        overlaps_x = np.zeros((len(bboxes), len(add_bboxes)))
        overlaps_y = np.zeros_like(overlaps_x)
        overlap_directions_x = np.zeros_like(overlaps_x)
        overlap_directions_y = np.zeros_like(overlaps_y)

        for i, bbox1 in enumerate(bboxes):
            for j, bbox2 in enumerate(add_bboxes):
                try:
                    x, y = bbox1.intersection(bbox1, bbox2).size
                    direction = np.sign(bbox1.extents - bbox2.extents)[:2]
                    overlaps_x[i, j] = x
                    overlaps_y[i, j] = y
                    overlap_directions_x[i, j] = direction[0]
                    overlap_directions_y[i, j] = direction[1]
                except AttributeError:
                    pass

        move_x = overlaps_x * overlap_directions_x
        move_y = overlaps_y * overlap_directions_y

        delta_x = move_x.sum(axis=1)
        delta_y = move_y.sum(axis=1)

        q = np.sum(overlaps_x), np.sum(overlaps_y)
        if move:
            move_texts(texts, delta_x, delta_y, bboxes, ax=ax)
        return delta_x, delta_y, q*/

        size_t n = labels.size();

        movements.assign(n, Eigen::Vector2d(0, 0));
        totals.assign(n, Eigen::Vector2d(0, 0));

        size_t no = objects.size();
        if (no < 1)
            return;

        std::vector<QRectF> bboxes = collectBoundingBoxes(labels, tx, ty);

        // auto coutBBox = [ & ] (const QRectF& bbox)
        // {
        //     loginf << "("
        //            << bbox.x() << "," << bbox.y() << "," << bbox.width() << "x" << bbox.height() << ")";
        // };

        //for (size_t i = 0; i < n; ++i)
        //{
        tbb::parallel_for(size_t(0), n, [&](size_t i) {
            //const auto& l = labels[ i ];

            const auto& bbox = bboxes[ i ];

            for (size_t j = 0; j < no; ++j)
            {
                const auto& bbox2 = objects[ j ];
                if (bbox2.isEmpty())
                    continue;

                // std::cout << "      ";
                // coutBBox(bbox);
                // std::cout << " ";
                // coutBBox(bbox2);
                // std::cout << " => " << bbox.intersects(bbox2) << std::endl;

                auto offset = repelFromBox(bbox, bbox2, simple);

                movements[ i ] += Eigen::Vector2d(offset.x(), offset.y());
                totals[ i ]    += Eigen::Vector2d(std::fabs(offset.x()), std::fabs(offset.y()));
            }
        //}
        });
    }

    /**
     */
    void repelFromPoints(std::vector<Eigen::Vector2d>& movements,
                         std::vector<Eigen::Vector2d>& totals,
                         const std::vector<QPointF>& points,
                         const std::vector<Label>& labels, 
                         double tx, 
                         double ty,
                         bool simple)
    {
        /*""
        Repel texts from all points specified by x and y while expanding their
        (texts'!) bounding boxes by expandby  (x, y), e.g. (1.2, 1.2)
        would multiply both width and height by 1.2.
        Requires a renderer to get the actual sizes of the text, and to that end
        either one needs to be directly provided, or the axes have to be specified,
        and the renderer is then got from the axes object.
        """
        assert len(x) == len(y)
        ax = ax or plt.gca()
        r = renderer or get_renderer(ax.get_figure())
        bboxes = get_bboxes(texts, r, expand, ax=ax)

        # move_x[i,j] is the x displacement of the i'th text caused by the j'th point
        move_x = np.zeros((len(bboxes), len(x)))
        move_y = np.zeros((len(bboxes), len(x)))
        for i, bbox in enumerate(bboxes):
            xy_in = get_points_inside_bbox(x, y, bbox)
            for j in xy_in:
                xp, yp = x[j], y[j]
                dx, dy = overlap_bbox_and_point(bbox, xp, yp)

                move_x[i, j] = dx
                move_y[i, j] = dy

        delta_x = move_x.sum(axis=1)
        delta_y = move_y.sum(axis=1)
        q = np.sum(np.abs(move_x)), np.sum(np.abs(move_y))
        if move:
            move_texts(texts, delta_x, delta_y, bboxes, ax=ax)
        return delta_x, delta_y, q*/

        size_t n = labels.size();

        movements.assign(n, Eigen::Vector2d(0, 0));
        totals.assign(n, Eigen::Vector2d(0, 0));

        size_t np = points.size();
        if (np < 1)
            return;

        std::vector<QRectF> bboxes = collectBoundingBoxes(labels, tx, ty);

//        for (size_t i = 0; i < n; ++i)
//        {
         tbb::parallel_for(size_t(0), n, [&](size_t i) {
            //const auto& l = labels[ i ];

            const auto& bbox = bboxes[ i ];

            for (size_t j = 0; j < np; ++j)
            {
                const auto& point = points[ j ];
                
                auto offset = repelFromPosition(bbox, point.x(), point.y(), simple);

                movements[ i ] += Eigen::Vector2d(offset.x(), offset.y());
                totals[ i ]    += Eigen::Vector2d(std::fabs(offset.x()), std::fabs(offset.y()));
            }
        //}
        });
    }

    /**
     */
    void repelFromROI(std::vector<Eigen::Vector2d>& movements,
                      std::vector<Eigen::Vector2d>& totals,
                      const QRectF& roi,
                      const std::vector<Label>& labels, 
                      double tx, 
                      double ty)
    {
        size_t n = labels.size();

        movements.assign(n, Eigen::Vector2d(0, 0));
        totals.assign(n, Eigen::Vector2d(0, 0));

        if (roi.isEmpty())
            return;
        

        std::vector<QRectF> bboxes = collectBoundingBoxes(labels, tx, ty);

//        for (size_t i = 0; i < n; ++i)
//        {
          tbb::parallel_for(size_t(0), n, [&](size_t i) {
            //const auto& l = labels[ i ];

            const auto& bbox = bboxes[ i ];

            double dx = 0;
            double dy = 0;
            if (bbox.left() < roi.left())
                dx = roi.left() - bbox.left();
            if (bbox.right() > roi.right())
                dx = roi.right() - bbox.right();
            if (bbox.top() < roi.top())
                dy = roi.top() - bbox.top();
            if (bbox.bottom() > roi.bottom())
                dy = roi.bottom() - bbox.bottom();

            movements[ i ] += Eigen::Vector2d(dx, dy);
            totals[ i ]    += Eigen::Vector2d(std::fabs(dx), std::fabs(dy));
        //}
        });
    }

    /**
     */
    bool placeLabels(std::vector<Label>& labels,
                     const Settings& settings)
    {
        /*
        """Iteratively adjusts the locations of texts.
        Call adjust_text the very last, after all plotting (especially
        anything that can change the axes limits) has been done. This is
        because to move texts the function needs to use the dimensions of
        the axes, and without knowing the final size of the plots the
        results will be completely nonsensical, or suboptimal.
        First moves all texts that are outside the axes limits
        inside. Then in each iteration moves all texts away from each
        other and from points. In the end hides texts and substitutes them
        with annotations to link them to the respective points.
        Parameters
        ----------
        texts : list
            A list of :obj:`matplotlib.text.Text` objects to adjust.
        Other Parameters
        ----------------
        x : array_like
            x-coordinates of points to repel from; if not provided only uses text
            coordinates.
        y : array_like
            y-coordinates of points to repel from; if not provided only uses text
            coordinates
        add_objects : list or PathCollection
            a list of additional matplotlib objects to avoid; they must have a
            `.get_window_extent()` method; alternatively, a PathCollection or a
            list of Bbox objects.
        ax : matplotlib axe, default is current axe (plt.gca())
            axe object with the plot
        expand_text : array_like, default (1.05, 1.2)
            a tuple/list/... with 2 multipliers (x, y) by which to expand the
            bounding box of texts when repelling them from each other.
        expand_points : array_like, default (1.05, 1.2)
            a tuple/list/... with 2 multipliers (x, y) by which to expand the
            bounding box of texts when repelling them from points.
        expand_objects : array_like, default (1.05, 1.2)
            a tuple/list/... with 2 multipliers (x, y) by which to expand the
            bounding box of texts when repelling them from other objects.
        expand_align : array_like, default (1.05, 1.2)
            a tuple/list/... with 2 multipliers (x, y) by which to expand the
            bounding box of texts when autoaligning texts.
        autoalign: str or boolean {'xy', 'x', 'y', True, False}, default 'xy'
            Direction in wich the best alignement will be determined
            - 'xy' or True, best alignment of all texts determined in all
            directions automatically before running the iterative adjustment
            (overriding va and ha),
            - 'x', will only align horizontally,
            - 'y', will only align vertically,
            - False, do nothing (i.e. preserve va and ha)
        va : str, default 'center'
            vertical alignment of texts
        ha : str, default 'center'
            horizontal alignment of texts,
        force_text : tuple, default (0.1, 0.25)
            the repel force from texts is multiplied by this value
        force_points : tuple, default (0.2, 0.5)
            the repel force from points is multiplied by this value
        force_objects : float, default (0.1, 0.25)
            same as other forces, but for repelling additional objects
        lim : int, default 500
            limit of number of iterations
        precision : float, default 0.01
            iterate until the sum of all overlaps along both x and y are less than
            this amount, as a fraction of the total widths and heights,
            respectively. May need to increase for complicated situations.
        only_move : dict, default {'points':'xy', 'text':'xy', 'objects':'xy'}
            a dict to restrict movement of texts to only certain axes for certain
            types of overlaps.
            Valid keys are 'points', 'text', and 'objects'.
            Valid values are '', 'x', 'y', and 'xy'.
            For example, only_move={'points':'y', 'text':'xy', 'objects':'xy'}
            forbids moving texts along the x axis due to overlaps with points.
        avoid_text : bool, default True
            whether to repel texts from each other.
        avoid_points : bool, default True
            whether to repel texts from points. Can be helpful to switch off in
            extremely crowded plots.
        avoid_self : bool, default True
            whether to repel texts from its original positions.
        save_steps : bool, default False
            whether to save intermediate steps as images.
        save_prefix : str, default ''
            if `save_steps` is True, a path and/or prefix to the saved steps.
        save_format : str, default 'png'
            if `save_steps` is True, a format to save the steps into.
        add_step_numbers : bool, default True
            if `save_steps` is True, whether to add step numbers as titles to the
            images of saving steps.
        args and kwargs :
            any arguments will be fed into obj:`ax.annotate` after all the
            optimization is done just for plotting the connecting arrows if
            required.
        Return
        ------
        int
            Number of iteration
        """
        plt.draw()
        ax = ax or plt.gca()
        r = get_renderer(ax.get_figure())
        transform = texts[0].get_transform()
        if (x is not None) & (y is not None):
            for ix, tupxy in enumerate(zip(x, y)):
                t_x, t_y = transform.transform(tupxy)
                x[ix] = t_x
                y[ix] = t_y
        orig_xy = [get_text_position(text, ax) for text in texts]
        orig_x = [xy[0] for xy in orig_xy]
        orig_y = [xy[1] for xy in orig_xy]
        force_objects = float_to_tuple(force_objects)
        force_text = float_to_tuple(force_text)
        force_points = float_to_tuple(force_points)

        #    xdiff = np.diff(ax.get_xlim())[0]
        #    ydiff = np.diff(ax.get_ylim())[0]

        bboxes = get_bboxes(texts, r, (1.0, 1.0), ax)
        sum_width = np.sum(list(map(lambda bbox: bbox.width, bboxes)))
        sum_height = np.sum(list(map(lambda bbox: bbox.height, bboxes)))
        if not any(list(map(lambda val: "x" in val, only_move.values()))):
            precision_x = np.inf
        else:
            precision_x = precision * sum_width
        #
        if not any(list(map(lambda val: "y" in val, only_move.values()))):
            precision_y = np.inf
        else:
            precision_y = precision * sum_height

        if x is None:
            if y is None:
                if avoid_self:
                    x, y = orig_x, orig_y
                else:
                    x, y = [], []
            else:
                raise ValueError("Please specify both x and y, or neither")
        if y is None:
            raise ValueError("Please specify both x and y, or neither")
        if add_objects is None:
            text_from_objects = False
            add_bboxes = []
        else:
            try:
                add_bboxes = get_bboxes(add_objects, r, (1, 1), ax)
            except:
                raise ValueError(
                    "Can't get bounding boxes from add_objects - is'\
                                it a flat list of matplotlib objects?"
                )
                return
            text_from_objects = True
        for text in texts:
            text.set_va(va)
            text.set_ha(ha)
        if save_steps:
            if add_step_numbers:
                plt.title("Before")
            plt.savefig(
                "%s%s.%s" % (save_prefix, "000a", save_format), format=save_format, dpi=150
            )

        if autoalign:
            if autoalign is True:
                autoalign = "xy"
            for i in range(2):
                texts = optimally_align_text(
                    x,
                    y,
                    texts,
                    expand=expand_align,
                    add_bboxes=add_bboxes,
                    direction=autoalign,
                    renderer=r,
                    ax=ax,
                )

        if save_steps:
            if add_step_numbers:
                plt.title("Autoaligned")
            plt.savefig(
                "%s%s.%s" % (save_prefix, "000b", save_format), format=save_format, dpi=150
            )

        texts = repel_text_from_axes(texts, ax, renderer=r, expand=expand_points)
        history = [(np.inf, np.inf)] * 10
        for i in range(lim):
            #        q1, q2 = [np.inf, np.inf], [np.inf, np.inf]

            if avoid_text:
                d_x_text, d_y_text, q1 = repel_text(
                    texts, renderer=r, ax=ax, expand=expand_text
                )
            else:
                d_x_text, d_y_text, q1 = [0] * len(texts), [0] * len(texts), (0, 0)

            if avoid_points:
                d_x_points, d_y_points, q2 = repel_text_from_points(
                    x, y, texts, ax=ax, renderer=r, expand=expand_points
                )
            else:
                d_x_points, d_y_points, q2 = [0] * len(texts), [0] * len(texts), (0, 0)

            if text_from_objects:
                d_x_objects, d_y_objects, q3 = repel_text_from_bboxes(
                    add_bboxes, texts, ax=ax, renderer=r, expand=expand_objects
                )
            else:
                d_x_objects, d_y_objects, q3 = [0] * len(texts), [0] * len(texts), (0, 0)

            if only_move:
                if "text" in only_move:
                    if "x" not in only_move["text"]:
                        d_x_text = np.zeros_like(d_x_text)
                    if "y" not in only_move["text"]:
                        d_y_text = np.zeros_like(d_y_text)
                if "points" in only_move:
                    if "x" not in only_move["points"]:
                        d_x_points = np.zeros_like(d_x_points)
                    if "y" not in only_move["points"]:
                        d_y_points = np.zeros_like(d_y_points)
                if "objects" in only_move:
                    if "x" not in only_move["objects"]:
                        d_x_objects = np.zeros_like(d_x_objects)
                    if "y" not in only_move["objects"]:
                        d_y_objects = np.zeros_like(d_y_objects)

            dx = (
                np.array(d_x_text) * force_text[0]
                + np.array(d_x_points) * force_points[0]
                + np.array(d_x_objects) * force_objects[0]
            )
            dy = (
                np.array(d_y_text) * force_text[1]
                + np.array(d_y_points) * force_points[1]
                + np.array(d_y_objects) * force_objects[1]
            )
            qx = np.sum([q[0] for q in [q1, q2, q3]])
            qy = np.sum([q[1] for q in [q1, q2, q3]])
            histm = np.max(np.array(history), axis=0)
            history.pop(0)
            history.append((qx, qy))
            move_texts(texts, dx, dy, bboxes=get_bboxes(texts, r, (1, 1), ax), ax=ax)
            if save_steps:
                if add_step_numbers:
                    plt.title(i + 1)
                plt.savefig(
                    "%s%s.%s" % (save_prefix, "{0:03}".format(i + 1), save_format),
                    format=save_format,
                    dpi=150,
                )
            # Stop if we've reached the precision threshold, or if the x and y displacement
            # are both greater than the max over the last 10 iterations (suggesting a
            # failure to converge)
            if (qx < precision_x and qy < precision_y) or np.all([qx, qy] >= histm):
                break
            # Now adding arrows from texts to their original locations if required
        if "arrowprops" in kwargs:
            bboxes = get_bboxes(texts, r, (1, 1), ax)
            kwap = kwargs.pop("arrowprops")
            for j, (bbox, text) in enumerate(zip(bboxes, texts)):
                ap = {"patchA": text}  # Ensure arrow is clipped by the text
                ap.update(kwap)  # Add arrowprops from kwargs
                ax.annotate(
                    "",  # Add an arrow from the text to the point
                    xy=get_orig_coords(transform, orig_x[j], orig_y[j]),
                    xytext=transform.inverted().transform(get_midpoint(bbox)),
                    arrowprops=ap,
                    xycoords=transform,
                    textcoords=transform,
                    *args,
                    **kwargs
                )

        if save_steps:
            if add_step_numbers:
                plt.title(i + 1)
                plt.savefig(
                    "%s%s.%s" % (save_prefix, "{0:03}".format(i + 1), save_format),
                    format=save_format,
                    dpi=150,
                )

        return i + 1
        */

        size_t n = labels.size();

        //compute convergence limits
        auto bboxes = collectBoundingBoxes(labels, 1, 1);
        double sum_widths  = 0.0;
        double sum_heights = 0.0;
        for (const auto& bbox : bboxes)
        {
            if (bbox.isEmpty())
                continue;
            sum_widths  += bbox.width();
            sum_heights += bbox.height(); 
        }
        double tol_x = sum_widths  < std::numeric_limits<double>::epsilon() ? std::numeric_limits<double>::max() : sum_widths  * settings.fb_precision;
        double tol_y = sum_heights < std::numeric_limits<double>::epsilon() ? std::numeric_limits<double>::max() : sum_heights * settings.fb_precision;

        //collect anchor points
        std::vector<QPointF> anchors;
        std::vector<QRectF>  anchor_regions;
        anchors.reserve(n);
        for (const auto& l : labels)
        {
            anchors.emplace_back(l.x_anchor, l.y_anchor);

            if (settings.fb_anchor_radius > 0)
            {
                const double r = settings.fb_anchor_radius;
                anchor_regions.emplace_back(l.x_anchor - r, l.y_anchor - r, 2 * r, 2 * r);
            }
        }

        std::vector<Eigen::Vector2d> displacements(n);

        std::vector<Eigen::Vector2d> displacements_labels(n);
        std::vector<Eigen::Vector2d> displacements_anchors(n);
        std::vector<Eigen::Vector2d> displacements_objects(n);
        std::vector<Eigen::Vector2d> displacements_roi(n);
        std::vector<Eigen::Vector2d> totals(n);

        bool   converged = false;
        double last_dx   = 0.0;
        double last_dy   = 0.0;
        //int    used_iter = -1;
        
        bool simple_mode = settings.method == Method::ForceBasedSimple;

        auto sumUpTotals = [ & ] (Eigen::Vector2d& total, const std::vector<Eigen::Vector2d>& totals)
        {
            for (const auto& t : totals)
                total += t;
        };

        //iterate up to max iterations
        for (int i = 0; i < settings.fb_max_iter; ++i)
        {
            displacements.assign(n, {0, 0});

            displacements_labels.assign(n, {0, 0});
            displacements_anchors.assign(n, {0, 0});
            displacements_objects.assign(n, {0, 0});
            displacements_roi.assign(n, {0, 0});
            totals.assign(n, {0, 0});
            
            Eigen::Vector2d total(0, 0);
            Eigen::Vector2d total_labels(0, 0);
            Eigen::Vector2d total_anchors(0, 0);
            Eigen::Vector2d total_objects(0, 0);
            Eigen::Vector2d total_roi(0, 0);

            //compute various displacements by repelling from certain types of obstacles
            if (settings.fb_avoid_labels)
            {
                label_placement::force::repelFromLabels(displacements_labels, 
                                                        totals, 
                                                        labels, 
                                                        settings.fb_expand_x, 
                                                        settings.fb_expand_y, 
                                                        simple_mode);
                sumUpTotals(total_labels, totals);
            }        
            if (settings.fb_avoid_anchors)
            {
                if (settings.fb_anchor_radius > 0)
                {
                    label_placement::force::repelFromObjects(displacements_anchors, 
                                                             totals, 
                                                             anchor_regions, 
                                                             labels, 
                                                             settings.fb_expand_x, 
                                                             settings.fb_expand_y, 
                                                             simple_mode);
                    sumUpTotals(total_anchors, totals);
                }
                else
                {
                    label_placement::force::repelFromPoints(displacements_anchors, 
                                                            totals, 
                                                            anchors, 
                                                            labels, 
                                                            settings.fb_expand_x, 
                                                            settings.fb_expand_y, 
                                                            simple_mode);
                    sumUpTotals(total_anchors, totals);
                }
                
            }
            if (settings.fb_avoid_objects && !settings.additional_objects.empty())
            {
                label_placement::force::repelFromObjects(displacements_objects, 
                                                         totals, 
                                                         settings.additional_objects, 
                                                         labels, 
                                                         settings.fb_expand_x, 
                                                         settings.fb_expand_y, 
                                                         simple_mode);
                sumUpTotals(total_objects, totals);
            }
            if (settings.fb_avoid_roi && !settings.roi.isEmpty())
            {
                label_placement::force::repelFromROI(displacements_roi,
                                                     totals,
                                                     settings.roi,
                                                     labels,
                                                     settings.fb_expand_x, 
                                                     settings.fb_expand_y);
                sumUpTotals(total_roi, totals);
            }

            //sum up individual displacements using weights
            for (size_t j = 0; j < n; ++j)
            {
                displacements[ j ] += displacements_labels [ j ] * settings.fb_weight_labels;
                displacements[ j ] += displacements_anchors[ j ] * settings.fb_weight_anchors;
                displacements[ j ] += displacements_objects[ j ] * settings.fb_weight_objects;
                displacements[ j ] += displacements_roi    [ j ] * settings.fb_weight_roi;
            }

            //sum up total absolute movement
            total += total_labels;
            total += total_anchors;
            total += total_objects;
            total += total_roi;

            //move labels
            for (size_t j = 0; j < n; ++j)
            {
                labels[ j ].x += displacements[ j ].x();
                labels[ j ].y += displacements[ j ].y();
            }

            last_dx = total.x();
            last_dy = total.y();
            
            //convergence if total movement is below precomputed threshold
            if (total.x() <= tol_x && total.y() < tol_y)
            {
                converged = true;
                //used_iter = i + 1;

                break;
            }
        }

        if (settings.verbose)
            loginf << (converged ? "CONVERGED" : "MAX ITER") << " dx: " << last_dx << "/" << tol_x << " dy: " << last_dy << "/" << tol_y;

        //convergence or max iter reached
        return true;
    }

} //namespace force
} //namespace label_placement
