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

#include "viewlayertreemodel.h"
#include "color_provider.h"

class ViewLayerTreeModel;
class COMPASS;

/**
 */
class ViewLayerPanelWidget : public LayerPanelWidget
{
    Q_OBJECT

public:
    explicit ViewLayerPanelWidget(const std::vector<LayerColumnSpec>& custom_columns,
                                  bool show_annotations,
                                  QWidget* parent = nullptr,
                                  LayerTreeItemDelegate* delegate = nullptr,
                                  ViewLayerTreeModel* external_model = nullptr);
    virtual ~ViewLayerPanelWidget() override = default;

    virtual ViewLayerTreeModel* model() const override
    {
        return dynamic_cast<ViewLayerTreeModel*>(LayerPanelWidget::model());
    }

    /// default expansion of the items outside the Data subtree (default: annotations fully)
    void setDefaultExpansion(const DefaultExpansionFunc& func) override;
    /// default expansion of the Data subtree: the root plus depth levels below it (-1 = collapsed),
    /// applied to all Data rows on change
    void setDataExpansion(int depth);
    /// Data expansion follows the COMPASS color mode
    void followColorMode(COMPASS& compass);

    /// levels below the Data root expanded by default, so the rows distinguishing the mode's colors are visible
    static int dataExpansionDepth(context::ColorProvider::Mode mode);

    /// custom column of the null counts (DBContentLeafPayload::customColumn() 0), summed over groups
    static LayerColumnSpec nullCountColumn();

private:
    bool isExpandedByDefault(const LayerTreeItem& item, int depth) const;
    bool isExpandedByDefault(const LayerTreeItem&        item,
                             int                         depth,
                             const DefaultExpansionFunc& default_expansion,
                             int                         data_depth) const;

    DefaultExpansionFunc default_expansion_;         // items outside the Data subtree, see setDefaultExpansion()
    int                  data_expansion_depth_ = -1; // see setDataExpansion()
};
