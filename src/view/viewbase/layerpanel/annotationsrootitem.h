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
#include "variableview.h"

#include <QIcon>

#include <vector>

/**
 * "Annotations" root item placed as a sibling to the DBContent root in a
 * LayerPanelWidget. Mirrors the Geographic View's Annotations item in name
 * and icon (compass.png).
 *
 * Children are AnnotationGroupItems (one per PlotMetadata::plot_group_) each
 * holding a flat list of AnnotationLeafItems. The variable-view annotation
 * model is consumer-side flat: VariableView::scanViewPointForAnnotations
 * collapses any nested feature hierarchy into 2 levels keyed by plot_group_.
 *
 * Lifecycle: data widgets call update() in response to
 * VariableView::annotationsChangedSignal. The update routes through
 * LayerTreeModel::refreshSubtree so expansion / selection elsewhere in the
 * panel survive.
 */
class AnnotationsRootItem : public LayerTreeItem
{
public:
    AnnotationsRootItem();
    ~AnnotationsRootItem() override = default;

    QVariant icon() const override;

    /// Rebuild the annotation subtree from the supplied groups. The leaf at
    /// (current_group_idx, current_anno_idx) starts with hidden_ == false;
    /// every other leaf starts hidden. Empty groups -> empty subtree.
    /// `view` is the VariableView passed through to each leaf so radio-style
    /// activation can call setCurrentAnnotation / showAnnotation.
    void update(const std::vector<VariableView::AnnotationGroup>& groups,
                int           current_group_idx,
                int           current_anno_idx,
                VariableView* view);

private:
    QIcon icon_;
};
