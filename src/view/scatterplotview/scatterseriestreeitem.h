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

#include <QList>
#include <QVariant>
#include <QIcon>

#include <QItemDelegate>
#include <QStyledItemDelegate>

class ScatterSeriesModel;

class QMenu;
class QWidget;


class ScatterSeriesTreeItemDelegate : public QStyledItemDelegate  // public QItemDelegate
{
    Q_OBJECT

public:
    ScatterSeriesTreeItemDelegate(QObject* parent = 0);
    void paint(QPainter* painter, const QStyleOptionViewItem& option,
               const QModelIndex& index) const;
    bool editorEvent(QEvent* event, QAbstractItemModel* model, const QStyleOptionViewItem& option,
                     const QModelIndex& index);
    // QSize sizeHint(const QStyleOptionViewItem &  option , const QModelIndex & index) const;
};

class ScatterSeriesTreeItem : public QObject
{
    Q_OBJECT

public:
    // leaf (DBContent level) — carries a DataSeries; its color is editable
    ScatterSeriesTreeItem(const std::string& name,
                          const QColor& color,
                          ScatterSeriesModel& model,
                          ScatterSeriesCollection::DataSeries* data_series,
                          ScatterSeriesTreeItem* parent_item);

    // group (DSType / DS / Line) — no data series, color used only for icon
    ScatterSeriesTreeItem(const std::string& name,
                          const QColor& color,
                          ScatterSeriesModel& model,
                          ScatterSeriesTreeItem* parent_item = nullptr);

    virtual ~ScatterSeriesTreeItem();

    virtual ScatterSeriesTreeItem* child(int row);
    int childCount() const;
    int columnCount() const;
    virtual QVariant data(int column) const;
    virtual QVariant icon() const;
    int row() const;

    bool hasParentItem() const { return parent_item_ != nullptr; }
    ScatterSeriesTreeItem* parentItem();

    void appendChild(ScatterSeriesTreeItem* child); // takes ownership
    void clear();

    virtual bool hasMenu() const { return false; }
    virtual void execMenu(const QPoint& pos) {};

    virtual bool canHide() const { return true; }
    virtual bool hidden() const { return hidden_; }
    virtual void hide(bool value);

    void hideAll(bool emit_signal=true);

    void updateHidden();

    bool itemHidden() const;

    const std::string& name() const { return name_; }
    bool hasDataSeries() const { return data_series_ != nullptr; }
    bool canEditColor() const { return data_series_ != nullptr; }
    QColor color() const;
    void setColor(const QColor& color);
    void emitColorChanged();

    /// build a 14x14 rounded-square icon (mirrors DataSourcesWidget::makeColorIcon)
    static QIcon makeColorIcon(const QColor& color);

    unsigned int getIndexOf(ScatterSeriesTreeItem* child);

protected:
    bool hidden_{false};

    std::string name_;
    ScatterSeriesModel& model_;

    std::map<std::string, std::unique_ptr<ScatterSeriesTreeItem>> child_items_;


    ScatterSeriesCollection::DataSeries* data_series_{nullptr};
    ScatterSeriesTreeItem* parent_item_{nullptr};

    QColor color_;
    QIcon  color_icon_;

    void rebuildColorIcon();
};

