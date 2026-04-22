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

#include "scatterseries.h"
#include "scatterseriestreeitem.h"


#include <set>

#include <QAbstractItemModel>
#include <QColor>
#include <QModelIndex>
#include <QVariant>

class COMPASS;

/**
 * Tree model backing the Layers panel of the ScatterPlot view.
 *
 * Tree structure
 * --------------
 * A fixed 4-level tree grouping every scatter series by its key
 * "DSType:DS Name:L<n>:DBContent":
 *
 *     Root (invisible)
 *      |-- DSType          (level 1 — e.g. "Radar", "ADSB")
 *      |    |-- DataSource (level 2 — e.g. a named sensor)
 *      |    |    |-- Line  (level 3 — "L1".."L4")
 *      |    |    |    |-- DBContent leaf (level 4 — carries a DataSeries)
 *
 * Malformed keys drop a leaf directly under root as a DBContent-level item
 * so nothing is silently discarded.
 *
 * Columns
 * -------
 * Two columns: "Name" and "Count". The count is the number of scatter
 * points — leaves return their own point count, groups return the sum of
 * their descendants' point counts (see ScatterSeriesTreeItem::totalCount()).
 *
 * Coloring and color propagation
 * ------------------------------
 * Leaves are the only authoritative source of color. Their color is the
 * chart color computed upstream by resolveSeriesColor() based on the
 * active ColorProvider::Mode (DSType / DataSource / DataSourceLine /
 * DBContent) and the data context palettes.
 *
 * Group colors are derived bottom-up after the tree is built (and after
 * every leaf-color edit) by recomputeEffectiveColorRecursive():
 *   - if every direct child has the same valid color, the group takes
 *     that color and shows its icon;
 *   - if children disagree (or any child has no color), the group's
 *     color is invalid and no icon is painted.
 *
 * Because leaves are colored by the active color mode, this rule means:
 *   - DSType mode       -> all leaves under a DSType share one color, so
 *                          the color naturally propagates up to every
 *                          ancestor level.
 *   - DataSource mode   -> leaves under a DS share a color; the DS group
 *                          shows it, but the DSType group above typically
 *                          doesn't because sibling DSs differ.
 *   - DataSourceLine    -> propagation stops at the Line level.
 *   - DBContent mode    -> leaves generally differ, so parent groups show
 *                          no icon.
 *
 * Icons are purely informative: color picking from the layers tree is
 * intentionally disabled (see ScatterSeriesTreeItemDelegate::editorEvent
 * in the .cpp) so that colors always reflect the data context.
 *
 * Default expansion
 * -----------------
 * The config widget (ScatterPlotViewConfigWidget::updateToVisibilitySlot)
 * expands the tree by default to the level whose color mode is active:
 * DSType mode collapses everything; DataSource/DataSourceLine/DBContent
 * progressively expand deeper. The user can still expand/collapse
 * manually; the default is reapplied on every model reset.
 */
class ScatterSeriesModel : public QAbstractItemModel
{
    Q_OBJECT

signals:
    void visibilityChangedSignal();
    void colorChangedSignal(const std::string& series_name, const QColor& color);

public:
    ScatterSeriesModel();
    virtual ~ScatterSeriesModel();

    QVariant data(const QModelIndex& index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column,
                      const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;

    /// Rebuild the 4-level tree from `collection`. Leaves take their color
    /// directly from `DataSeries::color` (set by resolveSeriesColor before
    /// calling here); groups are then colored bottom-up by
    /// recomputeEffectiveColorRecursive(). `compass` is kept in the signature
    /// for API stability.
    void updateFrom(ScatterSeriesCollection& collection, COMPASS& compass);

    enum DataRole
    {
        IconRole = Qt::UserRole + 100
    };

    void deselectAll();

    std::set<std::string> hiddenSeriesNames() const;
    void applyHiddenSeriesNames(const std::set<std::string>& names);

    /// Emit dataChanged for `item`, every descendant, and every ancestor so
    /// the tree view re-queries icons after a color edit that cascades down
    /// and propagates up.
    void notifyIconChangedSubtreeAndAncestors(ScatterSeriesTreeItem* item);

private:
    std::unique_ptr<ScatterSeriesTreeItem> root_item_;
};

