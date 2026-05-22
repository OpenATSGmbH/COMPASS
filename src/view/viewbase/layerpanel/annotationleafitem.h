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

#include "layertreeitem.h"

class VariableView;

/**
 * Leaf row representing one annotation under an AnnotationGroupItem.
 *
 * Radio-style single-select: the active annotation is the one with
 * `hidden_ == false`. All other AnnotationLeafItems in the same
 * AnnotationsRootItem subtree are forced to `hidden_ == true`.
 *
 * Override of setHidden enforces the radio rule:
 *  - hidden=true on the active leaf -> no-op (cannot uncheck the active one;
 *    one annotation is always shown when annotations are present).
 *  - hidden=false (activate) -> walk the AnnotationsRootItem subtree, force
 *    every other AnnotationLeafItem hidden silently, then activate self and
 *    forward to VariableView::setCurrentAnnotation + showAnnotation.
 *
 * persistenceId() is intentionally empty: the active annotation is captured
 * by VariableView::current_annotation_group_idx_ / current_annotation_idx_,
 * which are already persisted by the view.
 */
class AnnotationLeafItem : public LayerTreeItem
{
public:
    AnnotationLeafItem(const std::string& name,
                       int                group_idx,
                       int                annotation_idx,
                       VariableView*      view);
    ~AnnotationLeafItem() override = default;

    void setHidden(bool value, bool emit_signal = true) override;

    int groupIdx() const { return group_idx_; }
    int annotationIdx() const { return annotation_idx_; }

private:
    void activateAsRadio(bool emit_signal);

    int           group_idx_      {-1};
    int           annotation_idx_ {-1};
    VariableView* view_           {nullptr};
};
