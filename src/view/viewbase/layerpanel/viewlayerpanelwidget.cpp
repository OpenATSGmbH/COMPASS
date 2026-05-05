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

#include "viewlayerpanelwidget.h"

#include "dbcontentlayer.h"
#include "annotationsrootitem.h"

#include <QTreeView>

/**
 */
ViewLayerPanelWidget::ViewLayerPanelWidget(const std::vector<LayerColumnSpec>& custom_columns,
                                           bool show_annotations,
                                           QWidget* parent, 
                                           LayerTreeItemDelegate* delegate,
                                           ViewLayerTreeModel* external_model)
    : LayerPanelWidget(parent, 
                       delegate, 
                       external_model ? external_model : new ViewLayerTreeModel(custom_columns, show_annotations),
                       external_model != nullptr)
{
    model()->applyHeaderSettings(treeView()->header());
}
