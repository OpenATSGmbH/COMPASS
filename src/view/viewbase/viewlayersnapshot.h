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

#include "dbcontentlayer.h"
#include "viewlayerscan.h"

#include <string>

/**
 * Layer of a view's data as shown in the layer panel's data subtree: a snapshot of the
 * values the view computed for it, no reference into the view's data.
 *
 * - owned by ViewDataWidget (see ViewDataWidget::updateLayerTree()), its data tree leaf
 *   points at it until the next update
 * - visibility toggles are stored here, the view reads them (visible()) when drawing
 * - persistenceId() is the layer key ("<ds_type>:<ds_name>:L<n>:<dbcontent>")
 * - top leaves sit directly below the data root instead of the DSType groups
 */
class ViewLayerSnapshot : public DBContentLeafPayload
{
public:
    ViewLayerSnapshot(const std::string& key,
                      const view_layer_scan::LayerId& id,
                      unsigned int count,
                      unsigned int null_count = 0,
                      const QColor& color = QColor(),
                      bool visible = true)
    :   key_       (key)
    ,   id_        (id)
    ,   count_     (count)
    ,   null_count_(null_count)
    ,   color_     (color)
    ,   visible_   (visible)
    {}

    std::string name() const override { return id_.dbcontent; }
    unsigned int count() const override { return count_; }
    unsigned int nullCount() const override { return null_count_; }
    QColor color() const override { return color_; }

    void setVisible(bool v) override { visible_ = v; }
    bool visible() const { return visible_; }

    std::string persistenceId() const override { return key_; }

    /// custom column 0 = "#NULL" (views registering ViewLayerPanelWidget::nullCountColumn())
    QVariant customColumn(int view_col_idx) const override
    {
        if (view_col_idx == 0)
            return null_count_ ? QVariant((qulonglong)null_count_) : QVariant();
        return {};
    }

    const std::string& key() const { return key_; }
    const view_layer_scan::LayerId& id() const { return id_; }

    bool isTopLeaf() const { return top_leaf_; }
    void setTopLeaf(bool top_leaf) { top_leaf_ = top_leaf; }

private:
    std::string              key_;
    view_layer_scan::LayerId id_;
    unsigned int             count_      = 0;
    unsigned int             null_count_ = 0;
    QColor                   color_;
    bool                     visible_    = true;
    bool                     top_leaf_   = false; // below the data root, name = id().dbcontent
};
