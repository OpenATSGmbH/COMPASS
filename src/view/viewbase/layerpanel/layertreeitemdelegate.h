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

#include <QStyledItemDelegate>

/**
 * Delegate for LayerTreeModel.
 *
 * Column 0: paints [checkbox] [color icon] [name] inline.
 *   - Checkbox visible only when item->canHide() is true.
 *   - Color icon only painted when the item has a valid color.
 *   - Clicking the checkbox toggles the item's hidden state.
 *   - Color icon is NOT clickable - colors cannot be edited from the tree.
 *
 * Views that do not use per-layer colors can turn off the icon column with
 * setReserveIconColumn(false) so names sit directly after the checkbox.
 *
 * Other columns: delegate to the default painting path, with alignment taken
 * from the model's TextAlignmentRole.
 *
 * Stale items (LayerTreeItem::isStale()): greyed text and / or a rotating wait
 * icon in the count column, see setStaleIndication(). The icon is advanced by
 * the panel (advanceSpinner()) while the model has stale items.
 */
class LayerTreeItemDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    /// display of stale items, combined in the stale indication flags
    enum StaleIndication
    {
        StaleGreyed  = 1 << 0, // greyed text
        StaleSpinner = 1 << 1  // rotating wait icon in the count column
    };

    static const int SpinnerSteps = 12; // wait icon positions per turn

    explicit LayerTreeItemDelegate(QObject* parent = nullptr);

    void setReserveIconColumn(bool on) { reserve_icon_column_ = on; }
    bool reserveIconColumn() const { return reserve_icon_column_; }

    void setStaleIndication(int flags) { stale_indication_ = flags; }
    int staleIndication() const { return stale_indication_; }

    /// next position of the wait icon
    void advanceSpinner() { spinner_step_ = (spinner_step_ + 1) % SpinnerSteps; }

    void paint(QPainter* painter, const QStyleOptionViewItem& option,
               const QModelIndex& index) const override;

    bool editorEvent(QEvent* event, QAbstractItemModel* model,
                     const QStyleOptionViewItem& option,
                     const QModelIndex& index) override;

private:
    void paintSpinner(QPainter* painter, const QRect& rect) const;

    bool reserve_icon_column_ {true};
    int  stale_indication_    = StaleGreyed | StaleSpinner;
    int  spinner_step_        = 0;
};
