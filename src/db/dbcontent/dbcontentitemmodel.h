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

#include "dbcontentitemprovider.h"

#include <vector>

#include "json.hpp"

#include <QAbstractListModel>

class QMenu;

/**
 * Flat Qt list model exposing the items held by a DBContentItemProvider.
 *
 * Each row represents one unique item ID (key in item_locations_).
 * Column 0 provides:
 *   - Qt::DisplayRole   — item name built from groupingToString() + item ID string
 *   - Qt::CheckStateRole — per-item visibility toggle
 *
 * Subclass and override the virtual hooks to react to visibility changes and
 * to populate custom context-menu entries.
 *
 * Intended for use with QTreeView (flat, no hierarchy):
 *
 *   view->setRootIsDecorated(false);
 *   view->setContextMenuPolicy(Qt::CustomContextMenu);
 *   connect(view, &QTreeView::customContextMenuRequested, [=](const QPoint& pos) {
 *       model->showContextMenu(view->indexAt(pos),
 *                              view->viewport()->mapToGlobal(pos), view);
 *   });
 */
class DBContentItemModel : public QAbstractListModel
{
    Q_OBJECT

public:
    explicit DBContentItemModel(DBContentItemProvider& provider, QObject* parent = nullptr);
    virtual ~DBContentItemModel() = default;

    // --- QAbstractListModel interface ---

    int      rowCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool     setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;

    // --- Public API ---

    DBContentItemProvider&       provider()       { return provider_; }
    const DBContentItemProvider& provider() const { return provider_; }

    /** Returns the display name for an item: "<grouping> <id_string>". */
    QString itemName(const nlohmann::json& item_id) const;

    /** Returns true if the item is currently marked visible (queried from provider). */
    bool isVisible(const nlohmann::json& item_id) const;

    /** Toggles visibility for a single item via the provider. */
    void setVisible(const nlohmann::json& item_id, bool visible);

    /** Toggles visibility for all but a single item via the provider. */
    void setSiblingsVisible(const nlohmann::json& item_id, bool visible);

    /** Toggles visibility for all current items via the provider. */
    void setAllVisible(bool visible);

    /**
     * Shows a context menu at global_pos.
     * Call this from the QTreeView's customContextMenuRequested handler.
     * The menu always contains "Show All" and "Hide All".
     * If index is valid, fillItemContextMenu() is called first to allow
     * subclass-specific entries to appear above those fixed actions.
     */
    void showContextMenu(const QModelIndex& index, const QPoint& global_pos,
                         QWidget* parent = nullptr);

protected:
    // --- Virtual hooks ---

    /// Subclass hook invoked from showContextMenu() between the built-in
    /// "Show <item>"/"Hide <item>" entries and the "Show All"/"Hide All" entries.
    virtual void fillItemContextMenu(QMenu& menu, const nlohmann::json& item_id) {}
    virtual void onItemDoubleClicked(const nlohmann::json& item_id) {}

public slots:
    void itemDoubleClickedSlot(const QModelIndex& index);

private slots:
    void dataResetSlot();
    void dataRefreshedSlot();
    void itemVisibilityChangedSlot();

private:
    void rebuild();

    DBContentItemProvider&          provider_;
    std::vector<nlohmann::json>     item_ids_;   // ordered list of unique item IDs
};
