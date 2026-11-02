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

#include "dbcontentitemview.h"
#include "dbcontentitemmodel.h"
#include "dbcontentitemprovider.h"

#include "waitspinner.h"
#include "files.h"
#include "traced_assert.h"
#include "logger.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QKeyEvent>
#include <QSortFilterProxyModel>
#include <QStandardItemModel>
#include <QToolButton>
#include <QTreeView>
#include <QVBoxLayout>
#include <QApplication>

/**
 */
DBContentItemView::DBContentItemView(DBContentItemModel& model, QWidget* parent)
    : QWidget(parent)
    , model_(model)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(2);

    // Grouping combo - populated before connecting to avoid spurious setGrouping() calls
    QHBoxLayout* grouping_layout = new QHBoxLayout();

    QLabel* grouping_label = new QLabel("Item Mode", this);
    grouping_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    grouping_layout->addWidget(grouping_label);

    grouping_box_ = new QComboBox(this);

    for (auto g : DBContentItemProvider::getGroupings())
    {
        grouping_box_->addItem(
            QString::fromStdString(DBContentItemProvider::groupingToString(g)));
    }

    grouping_box_->setCurrentText(
        QString::fromStdString(
            DBContentItemProvider::groupingToString(model_.provider().grouping())));

    connect(grouping_box_, &QComboBox::currentTextChanged,
            this, &DBContentItemView::groupingChangedSlot);

    grouping_layout->addWidget(grouping_box_);

    // opens / closes the filter field (Ctrl+F opens it, Escape closes it, see eventFilter())
    filter_button_ = new QToolButton(this);
    filter_button_->setIcon(Utils::Files::IconProvider::getIcon("filter.png"));
    filter_button_->setCheckable(true);
    filter_button_->setAutoRaise(true);
    filter_button_->setToolTip("Filter items (Ctrl+F)");

    connect(filter_button_, &QToolButton::toggled, this, &DBContentItemView::setFilterVisible);

    grouping_layout->addWidget(filter_button_);

    // wait icon while the list shows the items of the previous data (reload)
    stale_spinner_ = new WaitSpinnerWidget(this);
    grouping_layout->addWidget(stale_spinner_);

    connect(&model_, &DBContentItemModel::staleChangedSignal, this, &DBContentItemView::updateStaleIndication);
    updateStaleIndication();

    layout->addLayout(grouping_layout);

    // filter: lists only the items whose name (or identification) matches
    filter_edit_ = new QLineEdit(this);
    filter_edit_->setPlaceholderText("Filter items (* and ? as wildcards, Escape closes)");
    filter_edit_->setClearButtonEnabled(true);
    filter_edit_->setVisible(false); // opened by the filter button

    connect(filter_edit_, &QLineEdit::textChanged, this, &DBContentItemView::filterChangedSlot);

    layout->addWidget(filter_edit_);

    // between model and view: passes the in-place row updates on => selection and scroll position stay
    proxy_model_ = new QSortFilterProxyModel(this);
    proxy_model_->setSourceModel(&model_);
    proxy_model_->setFilterCaseSensitivity(Qt::CaseInsensitive);
    proxy_model_->setFilterKeyColumn(-1); // all columns, e.g. the identification of the UTN mode

    // Item tree view
    tree_view_ = new QTreeView(this);
    tree_view_->setModel(proxy_model_);
    tree_view_->setRootIsDecorated(false);
    //flat list of same-height rows: skips the per-row size hint computation
    //(which queries the model for each item) on every model reset
    tree_view_->setUniformRowHeights(true);
    tree_view_->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(tree_view_, &QTreeView::customContextMenuRequested, this, &DBContentItemView::contextMenuSlot);

    connect(tree_view_, &QTreeView::doubleClicked, this, [ this ] (const QModelIndex& index)
    {
        model_.itemDoubleClickedSlot(proxy_model_->mapToSource(index));
    });
    
    connect(&model_.provider(), &DBContentItemProvider::groupingChangedSignal,
            this, &DBContentItemView::updateGrouping);

    layout->addWidget(tree_view_);

    // Ctrl+F / Escape on whichever child has the focus
    for (QWidget* w : std::initializer_list<QWidget*>{ grouping_box_, filter_button_, filter_edit_, tree_view_ })
        w->installEventFilter(this);
}

/**
 * Ctrl+F (open the filter) and Escape in the filter field (close it), whenever the focus is inside the widget.
 * Notice: Taken via the ShortcutOverride => wins over window wide shortcuts with the same keys (e.g. a menu
 * Ctrl+F added later), which then still work everywhere else.
 */
bool DBContentItemView::eventFilter(QObject* obj, QEvent* event)
{
    if (event->type() != QEvent::ShortcutOverride && event->type() != QEvent::KeyPress)
        return QWidget::eventFilter(obj, event);

    auto* key_event = static_cast<QKeyEvent*>(event);

    const bool open  = key_event->matches(QKeySequence::Find);
    const bool close = obj == filter_edit_ &&
                       key_event->key() == Qt::Key_Escape &&
                       key_event->modifiers() == Qt::NoModifier;

    if (!open && !close)
        return QWidget::eventFilter(obj, event);

    // claim the key => it arrives as a key press instead of triggering a shortcut
    if (event->type() == QEvent::ShortcutOverride)
    {
        event->accept();
        return true;
    }

    if (open)
        openFilter();
    else
        filter_button_->setChecked(false);

    return true;
}

/**
 * Opens the filter field if closed, focuses it and selects its text => it can be overwritten right away.
 */
void DBContentItemView::openFilter()
{
    filter_button_->setChecked(true);

    filter_edit_->setFocus();
    filter_edit_->selectAll();
}

/**
 */
void DBContentItemView::groupingChangedSlot(const QString& text)
{
    QApplication::setOverrideCursor(Qt::WaitCursor);

    model_.provider().setGrouping(
        DBContentItemProvider::groupingFromString(text.toStdString()), true);

    QApplication::restoreOverrideCursor();
}

/**
 * Case-insensitive, matches anywhere in the name, * and ? as wildcards; empty lists all items.
 */
void DBContentItemView::filterChangedSlot(const QString& text)
{
    proxy_model_->setFilterWildcard(text);
}

/**
 * Closes and empties the filter field => all items are listed again (e.g. on database close).
 */
void DBContentItemView::clearFilter()
{
    filter_button_->setChecked(false);
    filter_edit_->clear();
}

/**
 * Opens the filter field with the focus, or closes it: a closed field filters nothing (cleared), the focus
 * goes to the list so Ctrl+F still reaches the widget.
 */
void DBContentItemView::setFilterVisible(bool visible)
{
    filter_edit_->setVisible(visible);

    if (visible)
    {
        filter_edit_->setFocus();
        filter_edit_->selectAll();
        return;
    }

    filter_edit_->clear();
    tree_view_->setFocus();
}

/**
 * Notice: "All" and "Others" act on all items, also the ones the filter does not list.
 */
void DBContentItemView::contextMenuSlot(const QPoint& pos)
{
    const QModelIndex index = proxy_model_->mapToSource(tree_view_->indexAt(pos));

    model_.showContextMenu(index, tree_view_->viewport()->mapToGlobal(pos), tree_view_);
}

/**
 */
void DBContentItemView::updateGrouping()
{
    auto new_text = QString::fromStdString(model_.provider().groupingAsString());

    traced_assert(isGroupingActive(model_.provider().grouping()));
    traced_assert(grouping_box_->findText(new_text) >= 0);

    if (grouping_box_->currentText() == new_text)
        return;

    grouping_box_->blockSignals(true);
    grouping_box_->setCurrentText(new_text);
    grouping_box_->blockSignals(false);
}

/**
 */
void DBContentItemView::updateStaleIndication()
{
    const bool stale = model_.isStale();

    stale_spinner_->setSpinning(stale);
    stale_spinner_->setToolTip(stale ? "Data is being reloaded" : QString());
}

/**
 */
dbContent::Grouping DBContentItemView::currentGrouping() const
{
    return model_.provider().grouping();
}

/**
 */
void DBContentItemView::setActiveGroupings(unsigned int flags)
{
    active_groupings_ = flags;

    auto* model = qobject_cast<QStandardItemModel*>(grouping_box_->model());
    traced_assert(model);

    const auto all_groupings = DBContentItemProvider::getGroupings();
    traced_assert(static_cast<int>(all_groupings.size()) == model->rowCount());

    for (int i = 0; i < model->rowCount(); ++i)
    {
        const unsigned int g_flag = 1u << static_cast<unsigned int>(all_groupings[i]);
        if (auto* item = model->item(i))
            item->setEnabled((flags & g_flag) != 0);
    }

    //!default grouping shall always be active!
    traced_assert(isGroupingActive(DBContentItemProvider::DefaultGrouping));
}

/**
 */
bool DBContentItemView::isGroupingActive(dbContent::Grouping grouping) const
{
    const unsigned int g_flag = 1u << static_cast<unsigned int>(grouping);
    return (active_groupings_ & g_flag) != 0;
}
