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

#include <QIcon>

/**
 * "Annotations" root item placed as a sibling to the DBContent root in a
 * LayerPanelWidget. Mirrors the Geographic View's Annotations item in name
 * and icon (compass.png). Currently a structural placeholder - no children,
 * visibility toggling is display-only.
 */
class AnnotationsRootItem : public LayerTreeItem
{
public:
    AnnotationsRootItem();
    ~AnnotationsRootItem() override = default;

    QVariant icon() const override;

private:
    QIcon icon_;
};
