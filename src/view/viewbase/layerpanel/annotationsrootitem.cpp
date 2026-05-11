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

#include "annotationsrootitem.h"
#include "annotationgroupitem.h"
#include "annotationleafitem.h"
#include "layertreemodel.h"

#include "files.h"

#include <memory>
#include <vector>

AnnotationsRootItem::AnnotationsRootItem()
    : LayerTreeItem("Annotations")
    , icon_(Utils::Files::IconProvider::getIcon("compass.png"))
{
}

QVariant AnnotationsRootItem::icon() const
{
    return QVariant(icon_);
}

void AnnotationsRootItem::update(const std::vector<VariableView::AnnotationGroup>& groups,
                                 int           current_group_idx,
                                 int           current_anno_idx,
                                 VariableView* view)
{
    auto build = [&]() -> std::vector<std::unique_ptr<LayerTreeItem>>
    {
        std::vector<std::unique_ptr<LayerTreeItem>> out;
        out.reserve(groups.size());

        for (int g = 0; g < (int)groups.size(); ++g)
        {
            const auto& group = groups[g];

            // Build the leaves for this group up front; they go either under
            // a named group item, or - when the group name is empty - directly
            // under the AnnotationsRootItem so the user does not see a blank
            // intermediate row for the "no plot_group" case.
            std::vector<std::unique_ptr<LayerTreeItem>> leaves;
            leaves.reserve(group.annotations.size());

            for (int a = 0; a < (int)group.annotations.size(); ++a)
            {
                const auto& anno = group.annotations[a];

                auto leaf = std::make_unique<AnnotationLeafItem>(
                    anno.metadata.fullTitle(), g, a, view);

                const bool is_active = (g == current_group_idx && a == current_anno_idx);
                leaf->LayerTreeItem::setHidden(!is_active, /*emit_signal=*/false);

                leaves.push_back(std::move(leaf));
            }

            if (group.name.empty())
            {
                for (auto& leaf : leaves)
                    out.push_back(std::move(leaf));
            }
            else
            {
                auto group_item = std::make_unique<AnnotationGroupItem>(group.name);
                for (auto& leaf : leaves)
                    group_item->appendChild(std::move(leaf));
                out.push_back(std::move(group_item));
            }
        }

        return out;
    };

    if (model_)
        model_->refreshSubtree(this, build);
    else
    {
        // No model attached yet: just swap children directly. Used during
        // initial wiring before the panel is on screen.
        clearChildren();
        for (auto& c : build())
            appendChild(std::move(c));
    }
}
