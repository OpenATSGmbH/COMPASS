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

#include "annotationleafitem.h"
#include "annotationsrootitem.h"
#include "layertreemodel.h"
#include "variableview.h"

#include <functional>

AnnotationLeafItem::AnnotationLeafItem(const std::string& name,
                                       int                group_idx,
                                       int                annotation_idx,
                                       VariableView*      view)
    : LayerTreeItem(name)
    , group_idx_(group_idx)
    , annotation_idx_(annotation_idx)
    , view_(view)
{
}

void AnnotationLeafItem::setHidden(bool value, bool emit_signal)
{
    if (value)
    {
        // Radio rule: cannot uncheck the active leaf via UI.
        // Programmatic hide of an already-hidden leaf is a no-op for our
        // purposes; nothing else relies on the cascade hook for these leaves.
        if (hidden_)
            LayerTreeItem::setHidden(value, emit_signal);
        return;
    }
    activateAsRadio(emit_signal);
}

void AnnotationLeafItem::activateAsRadio(bool emit_signal)
{
    // Walk up to the AnnotationsRootItem so we can scope the deactivation to
    // this annotations subtree only (other roots in the panel - e.g. the
    // DBContent root - must not be touched).
    LayerTreeItem* root = parent_item_;
    while (root && dynamic_cast<AnnotationsRootItem*>(root) == nullptr)
        root = root->parentItem();

    if (root)
    {
        std::function<void(LayerTreeItem*)> walk = [this, &walk](LayerTreeItem* node)
        {
            for (int i = 0; i < node->childCount(); ++i)
            {
                LayerTreeItem* c = node->child(i);
                if (auto* leaf = dynamic_cast<AnnotationLeafItem*>(c); leaf && leaf != this)
                    leaf->LayerTreeItem::setHidden(true, /*emit_signal=*/false);
                walk(c);
            }
        };
        walk(root);
    }

    LayerTreeItem::setHidden(false, /*emit_signal=*/false);

    if (view_)
    {
        view_->setCurrentAnnotation(group_idx_, annotation_idx_);
        view_->showAnnotation();
    }

    if (emit_signal && model_)
        emit model_->hiddenChangedSignal();
}
