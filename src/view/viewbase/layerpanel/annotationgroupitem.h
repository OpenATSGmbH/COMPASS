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

/**
 * Plot-group container under AnnotationsRootItem.
 *
 * One row per `PlotMetadata::plot_group_` value collected from the loaded
 * view point's annotation features. Holds the `AnnotationLeafItem`s for
 * that group as children.
 *
 * Non-checkable: selection lives on the leaves (radio-style). Mirrors the
 * "Plot Group" combo's role in the previous variableviewannotationwidget
 * UI, which was display-only.
 */
class AnnotationGroupItem : public LayerTreeItem
{
public:
    explicit AnnotationGroupItem(const std::string& name);
    ~AnnotationGroupItem() override = default;

    bool canHide() const override { return false; }
};
