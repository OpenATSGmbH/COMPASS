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
 * persistenceId() is intentionally empty and the checked state is not part of
 * any config: the active annotation is captured by
 * VariableView::current_annotation_group_idx_ / current_annotation_idx_, which
 * are already persisted by the view.
 *
 * Kept across updates by its update key (title, "#n" for repeated titles below
 * the same parent), see AnnotationsRootItem::updateFrom().
 */
class AnnotationLeafItem : public LayerTreeItem
{
public:
    AnnotationLeafItem(const std::string& name,
                       const std::string& update_key,
                       int                group_idx,
                       int                annotation_idx,
                       VariableView*      view);
    ~AnnotationLeafItem() override = default;

    void setHidden(bool value, bool emit_signal = true) override;
    std::string updateKey() const override { return update_key_; }

    int groupIdx() const { return group_idx_; }
    int annotationIdx() const { return annotation_idx_; }
    /// position of the annotation in the view's groups, changes with the annotations
    void setIndices(int group_idx, int annotation_idx);

    /// radio state from the view's current annotation, silent (no activation, no hidden signal)
    void setActive(bool active);

private:
    void activateAsRadio(bool emit_signal);

    std::string   update_key_;
    int           group_idx_      {-1};
    int           annotation_idx_ {-1};
    VariableView* view_           {nullptr};
};
