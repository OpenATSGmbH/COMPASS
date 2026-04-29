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

#include "layerpanelwidget.h"
#include "layertreedefs.h"

class DBContentRootItem;
class AnnotationsRootItem;

/**
 */
class ViewLayerPanelWidget : public LayerPanelWidget
{
    Q_OBJECT

public:
    explicit ViewLayerPanelWidget(const std::vector<LayerColumnSpec>& custom_columns = std::vector<LayerColumnSpec>(),
                                  bool show_annotations = false,
                                  QWidget* parent = nullptr,
                                  LayerTreeItemDelegate* delegate = nullptr);
    virtual ~ViewLayerPanelWidget() override = default;

    DBContentRootItem* dbContentRootItem() { return dbcontent_root_item_; }
    const DBContentRootItem* dbContentRootItem() const { return dbcontent_root_item_; }

    AnnotationsRootItem* annotationsRootItem() { return annotations_root_item_; }
    const AnnotationsRootItem* annotationsRootItem() const { return annotations_root_item_; }

    bool showsAnnotations() const { return annotations_root_item_ != nullptr; }

private:
    DBContentRootItem*   dbcontent_root_item_   = nullptr;
    AnnotationsRootItem* annotations_root_item_ = nullptr;
};
