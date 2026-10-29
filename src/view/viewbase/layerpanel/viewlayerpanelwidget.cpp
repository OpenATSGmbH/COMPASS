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
#include "viewlayertreemodel.h"
#include "layertreeitem.h"

#include "dbcontentlayer.h"
#include "annotationsrootitem.h"

#include "compass.h"

#include <QTreeView>

#include <functional>

namespace
{
    /// sum of the valid integer values, invalid if there is none
    QVariant sumULongLong(const std::vector<QVariant>& vals)
    {
        unsigned long long sum = 0;
        bool               any = false;

        for (const auto& v : vals)
        {
            bool ok = false;
            unsigned long long n = v.toULongLong(&ok);
            if (!ok)
                continue;

            sum += n;
            any  = true;
        }

        return any ? QVariant((qulonglong)sum) : QVariant();
    }
}

/**
 */
LayerColumnSpec ViewLayerPanelWidget::nullCountColumn()
{
    LayerColumnSpec spec;
    spec.header           = "#NULL";
    spec.default_width    = 70;
    spec.resize_mode      = QHeaderView::Interactive;
    spec.alignment        = Qt::AlignRight | Qt::AlignVCenter;
    spec.group_aggregator = &sumULongLong;

    return spec;
}

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

    // other items: annotations fully expanded
    default_expansion_ = [ this ] (const LayerTreeItem& item, int)
    {
        const auto* anno_root = model()->annotationsRootItem();

        for (const LayerTreeItem* it = &item; it; it = it->parentItem())
            if (anno_root && it == anno_root)
                return true;

        return false;
    };

    LayerPanelWidget::setDefaultExpansion([ this ] (const LayerTreeItem& item, int depth)
    {
        return isExpandedByDefault(item, depth);
    });
}

/**
 * Policy for the items outside the Data subtree, deviations from the previous policy are kept.
 */
void ViewLayerPanelWidget::setDefaultExpansion(const DefaultExpansionFunc& func)
{
    const DefaultExpansionFunc old_func  = default_expansion_;
    const int                  old_depth = data_expansion_depth_;

    default_expansion_ = func;

    defaultExpansionChanged([ this, old_func, old_depth ] (const LayerTreeItem& item, int depth)
    {
        return isExpandedByDefault(item, depth, old_func, old_depth);
    });
}

/**
 * Notice: the Data items all follow the new default on change (user changes included), the other
 * items keep their expansion.
 */
void ViewLayerPanelWidget::setDataExpansion(int depth)
{
    if (depth == data_expansion_depth_)
        return;

    data_expansion_depth_ = depth;

    if (const auto* db_root = model()->dbContentRootItem())
        applyDefaultExpansion(model()->index(db_root->row(), 0));
}

/**
 */
void ViewLayerPanelWidget::followColorMode(COMPASS& compass)
{
    setDataExpansion(dataExpansionDepth((context::ColorProvider::Mode)compass.colorMode()));

    connect(&compass, &COMPASS::colorModeChangedSignal, this, [ this ] (unsigned int mode)
    {
        setDataExpansion(dataExpansionDepth((context::ColorProvider::Mode)mode));
    });
}

/**
 * The Data root itself is always expanded.
 */
int ViewLayerPanelWidget::dataExpansionDepth(context::ColorProvider::Mode mode)
{
    switch (mode)
    {
        case context::ColorProvider::Mode::DSType:         return 0; // collapsed below the root
        case context::ColorProvider::Mode::DataSource:     return 1; // data source rows
        case context::ColorProvider::Mode::DataSourceLine: return 2; // line rows
        case context::ColorProvider::Mode::DBContent:      return 3; // everything
    }
    return 3;
}

/**
 */
bool ViewLayerPanelWidget::isExpandedByDefault(const LayerTreeItem& item, int depth) const
{
    return isExpandedByDefault(item, depth, default_expansion_, data_expansion_depth_);
}

/**
 * Data root plus data_depth levels below it, else the policy for the other items.
 */
bool ViewLayerPanelWidget::isExpandedByDefault(const LayerTreeItem&        item,
                                               int                         depth,
                                               const DefaultExpansionFunc& default_expansion,
                                               int                         data_depth) const
{
    const auto* db_root = model()->dbContentRootItem();

    int rel_depth = 0;
    for (const LayerTreeItem* it = &item; it; it = it->parentItem(), ++rel_depth)
        if (db_root && it == db_root)
            return data_depth >= 0 && rel_depth <= data_depth;

    return default_expansion && default_expansion(item, depth);
}
