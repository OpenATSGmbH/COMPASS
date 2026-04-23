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

#include "scatterplotviewconfigwidget.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "scatterplotviewwidget.h"
#include "scatterplotview.h"
#include "logger.h"
#include "variable.h"
#include "metavariable.h"
#include "ui_test_common.h"
#include "scatterseriestreeitem.h"
#include "scatterplotviewdatawidget.h"

#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTabWidget>
#include <QTreeView>
#include <QHeaderView>

using namespace Utils;
using namespace dbContent;

/**
*/
ScatterPlotViewConfigWidget::ScatterPlotViewConfigWidget(ScatterPlotViewWidget* view_widget, 
                                                         QWidget* parent)
:   VariableViewConfigWidget(view_widget, view_widget->getView(), parent)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    auto layout = configLayout();

    {
        color_mode_label_ = new QLabel(this);
        color_mode_label_->setText("Color Mode: " + colorModeText(view_->compass().colorMode()));
        layout->addWidget(color_mode_label_);

        connect(&view_->compass(), &COMPASS::colorModeChangedSignal,
                this, &ScatterPlotViewConfigWidget::colorModeChangedSlot);
    }

    {
        loginf << "creating lay view";

        layer_view_ = new QTreeView(this);
        ScatterSeriesTreeItemDelegate* delegate = new ScatterSeriesTreeItemDelegate(this);
        layer_view_->setItemDelegate(delegate);

        layer_view_->setModel(&view_widget->getViewDataWidget()->dataModel());

        // Name stretches to fill; Count and # NULL stay at fixed widths.
        layer_view_->header()->setStretchLastSection(false);
        layer_view_->header()->setSectionResizeMode(0 /*Name  */, QHeaderView::Stretch);
        layer_view_->header()->setSectionResizeMode(1 /*Count */, QHeaderView::Interactive);
        layer_view_->header()->setSectionResizeMode(2 /*# NULL*/, QHeaderView::Interactive);
        layer_view_->header()->resizeSection(1, 70);
        layer_view_->header()->resizeSection(2, 70);

        layer_view_->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(layer_view_, &QTreeView::customContextMenuRequested,
                this, &ScatterPlotViewConfigWidget::showLayerContextMenuSlot);

        connect(&view_widget->getViewDataWidget()->dataModel(), &ScatterSeriesModel::visibilityChangedSignal,
                this, &ScatterPlotViewConfigWidget::updateToVisibilitySlot);

        // tree collapses on beginResetModel/endResetModel — re-expand after each refresh
        connect(&view_widget->getViewDataWidget()->dataModel(), &QAbstractItemModel::modelReset,
                this, &ScatterPlotViewConfigWidget::updateToVisibilitySlot);

        //getTabWidget()->addTab(layer_view_, "Layers");
        layout->addWidget(layer_view_);
    }


    use_connection_lines_ = new QCheckBox("Use Connection Lines");
    use_connection_lines_->setChecked(view_->useConnectionLines());
    UI_TEST_OBJ_NAME(use_connection_lines_, use_connection_lines_->text())

    connect(use_connection_lines_, &QCheckBox::clicked,
            this, &ScatterPlotViewConfigWidget::useConnectionLinesSlot);
    
    layout->addWidget(use_connection_lines_);

    //showSwitch(0, true);
}

/**
*/
ScatterPlotViewConfigWidget::~ScatterPlotViewConfigWidget() = default;

/**
*/
void ScatterPlotViewConfigWidget::useConnectionLinesSlot()
{
    loginf;

    traced_assert(use_connection_lines_);
    view_->useConnectionLines(use_connection_lines_->checkState() == Qt::Checked);
}

/**
*/
void ScatterPlotViewConfigWidget::updateToVisibilitySlot()
{
    traced_assert(layer_view_);

    // Expand default depth by the current color mode so that the level whose
    // color is currently being distinguished is visible:
    //   DSType mode        -> show only DSType rows (collapsed)
    //   DataSource mode    -> expand DSType  (depth 0)
    //   DataSourceLine mode-> expand DataSource (depth 1)
    //   DBContent mode     -> expand Line (depth 2) = fully expanded
    const unsigned int mode = view_->compass().colorMode();
    switch (mode)
    {
        case 0: /* DSType         */ layer_view_->collapseAll(); break;
        case 2: /* DataSource     */ layer_view_->expandToDepth(0); break;
        case 3: /* DataSourceLine */ layer_view_->expandToDepth(1); break;
        case 1: /* DBContent      */ layer_view_->expandToDepth(2); break;
        default:                     layer_view_->expandToDepth(2); break;
    }

    // Name column is in Stretch mode (set in constructor); only the fixed
    // numeric columns need explicit widths here.
    layer_view_->header()->resizeSection(1 /*Count */, 70);
    layer_view_->header()->resizeSection(2 /*# NULL*/, 70);
}

/**
*/
void ScatterPlotViewConfigWidget::showLayerContextMenuSlot(const QPoint& pos)
{
    traced_assert(layer_view_);

    auto& model = view_->getDataWidget()->dataModel();

    // No layers → nothing to act on; suppress the context menu entirely.
    if (model.rowCount(QModelIndex()) == 0)
        return;

    using Level = ScatterSeriesTreeItem::Level;

    const QModelIndex idx = layer_view_->indexAt(pos);
    ScatterSeriesTreeItem* item = idx.isValid()
            ? static_cast<ScatterSeriesTreeItem*>(idx.internalPointer())
            : nullptr;

    // Walk the model top-down; invoke `fn` for every item at `lvl` and skip
    // descent into items once the target level is reached. The tree has a
    // fixed depth of 4, so unbounded recursion is safe here.
    auto forEachAtLevel = [&model](Level lvl, std::function<void(ScatterSeriesTreeItem*)> fn)
    {
        std::function<void(const QModelIndex&)> walk = [&](const QModelIndex& parent)
        {
            const int n = model.rowCount(parent);
            for (int i = 0; i < n; ++i)
            {
                QModelIndex child = model.index(i, 0, parent);
                auto* c = static_cast<ScatterSeriesTreeItem*>(child.internalPointer());
                if (c->level() == lvl) fn(c);
                else                   walk(child);
            }
        };
        walk(QModelIndex());
    };

    // Batch a group of hide/show calls so the chart only rebuilds once:
    // block model signals, run the ops, unblock, emit once at the end.
    auto batchVisibility = [&model](std::function<void()> ops)
    {
        model.blockSignals(true);
        ops();
        model.blockSignals(false);
        emit model.visibilityChangedSignal();
    };

    // Recursively expand/collapse the subtree rooted at `index`.
    std::function<void(const QModelIndex&, bool)> expandSubtree =
        [&](const QModelIndex& index, bool expand)
    {
        if (!index.isValid())
            return;
        if (expand) layer_view_->expand(index);
        else        layer_view_->collapse(index);
        const int n = model.rowCount(index);
        for (int i = 0; i < n; ++i)
            expandSubtree(model.index(i, 0, index), expand);
    };

    QMenu menu(this);
    menu.setToolTipsVisible(true);

    auto add = [&](QMenu* m, const QString& text, const QString& tip,
                   std::function<void()> fn)
    {
        QAction* a = m->addAction(text);
        a->setToolTip(tip);
        connect(a, &QAction::triggered, this, [fn]{ fn(); });
    };

    // --- Per-item section ---
    if (item && item->level() != Level::Root)
    {
        const QString name = QString::fromStdString(item->name());

        const bool is_group = item->level() == Level::DSType    ||
                              item->level() == Level::DataSource ||
                              item->level() == Level::Line;

        switch (item->level())
        {
            case Level::DSType:
            {
                menu.addSection(QString("DSType \"%1\"").arg(name));
                add(&menu, "Deselect Other DSTypes",
                    "Keep this DSType selected; deselect all other DSTypes",
                    [item, forEachAtLevel, batchVisibility]
                    {
                        batchVisibility([&]
                        {
                            forEachAtLevel(Level::DSType, [&](ScatterSeriesTreeItem* t)
                            {
                                if (t != item) t->hideAll(false);
                            });
                            item->showAll(false);
                        });
                    });
                add(&menu, "Select All Children",
                    "Make every descendant of this DSType visible",
                    [item]{ item->showAll(); });
                add(&menu, "Deselect All Children",
                    "Hide every descendant of this DSType",
                    [item]{ item->hideAll(); });
                break;
            }
            case Level::DataSource:
            {
                menu.addSection(QString("Data Source \"%1\"").arg(name));
                add(&menu, "Deselect Other Data Sources",
                    "Keep this Data Source selected; deselect all other Data Sources",
                    [item, forEachAtLevel, batchVisibility]
                    {
                        batchVisibility([&]
                        {
                            forEachAtLevel(Level::DataSource, [&](ScatterSeriesTreeItem* t)
                            {
                                if (t != item) t->hideAll(false);
                            });
                            item->showAll(false);
                            // un-hide ancestors so cascading lets this DS show
                            for (auto* a = item->parentItem(); a; a = a->parentItem())
                                a->hide(false);
                        });
                    });
                // "Select All <DSType> Data Sources" — show every sibling DS
                // (i.e. every DataSource-level item under the same parent
                // DSType). Matches the data sources widget's per-type action.
                if (item->parentItem())
                {
                    auto* parent = item->parentItem();
                    const QString ds_type = QString::fromStdString(parent->name());
                    add(&menu,
                        QString("Select All %1 Data Sources").arg(ds_type),
                        QString("Show every Data Source of type \"%1\"").arg(ds_type),
                        [parent]{ parent->showAll(); });
                }
                add(&menu, "Select All Children",
                    "Make every descendant of this Data Source visible",
                    [item]{ item->showAll(); });
                add(&menu, "Deselect All Children",
                    "Hide every descendant of this Data Source",
                    [item]{ item->hideAll(); });
                break;
            }
            case Level::Line:
            {
                menu.addSection(QString("Line \"%1\"").arg(name));
                add(&menu, "Deselect Other Lines",
                    "Keep this Line selected; deselect all other Lines of this Data Source",
                    [item, batchVisibility]
                    {
                        batchVisibility([&]
                        {
                            auto* parent = item->parentItem();
                            if (!parent) return;
                            for (int i = 0; i < parent->childCount(); ++i)
                            {
                                auto* sib = parent->child(i);
                                if (sib == item) sib->showAll(false);
                                else             sib->hideAll(false);
                            }
                            for (auto* a = parent; a; a = a->parentItem())
                                a->hide(false);
                        });
                    });
                add(&menu, "Select All Children",
                    "Make every DBContent under this Line visible",
                    [item]{ item->showAll(); });
                add(&menu, "Deselect All Children",
                    "Hide every DBContent under this Line",
                    [item]{ item->hideAll(); });
                break;
            }
            case Level::DBContent:
            {
                menu.addSection(QString("DBContent \"%1\"").arg(name));
                add(&menu, "Deselect Other DBContents",
                    "Keep this DBContent selected; deselect all other DBContents of this Line",
                    [item, batchVisibility]
                    {
                        batchVisibility([&]
                        {
                            auto* parent = item->parentItem();
                            if (!parent) return;
                            for (int i = 0; i < parent->childCount(); ++i)
                            {
                                auto* sib = parent->child(i);
                                if (sib == item) sib->hide(false);
                                else             sib->hide(true);
                            }
                            for (auto* a = parent; a; a = a->parentItem())
                                a->hide(false);
                        });
                    });
                break;
            }
            default: break;
        }

        if (is_group)
        {
            menu.addSeparator();
            const QModelIndex captured_idx = idx;
            add(&menu, "Expand",
                "Expand this entry's sub-tree",
                [expandSubtree, captured_idx]{ expandSubtree(captured_idx, true); });
            add(&menu, "Collapse",
                "Collapse this entry's sub-tree",
                [expandSubtree, captured_idx]{ expandSubtree(captured_idx, false); });
        }
    }

    // --- Global section (always) ---
    menu.addSection("All");
    add(&menu, "Select All",
        "Make every entry visible",
        [&model]{ model.selectAll(); });
    add(&menu, "Deselect All",
        "Hide every entry",
        [&model]{ model.deselectAll(); });
    menu.addSeparator();
    add(&menu, "Select All DSTypes",
        "Show every DSType and its descendants",
        [forEachAtLevel, batchVisibility]
        {
            batchVisibility([&]
            {
                forEachAtLevel(Level::DSType, [](ScatterSeriesTreeItem* t){ t->showAll(false); });
            });
        });
    add(&menu, "Deselect All DSTypes",
        "Hide every DSType and its descendants",
        [forEachAtLevel, batchVisibility]
        {
            batchVisibility([&]
            {
                forEachAtLevel(Level::DSType, [](ScatterSeriesTreeItem* t){ t->hideAll(false); });
            });
        });
    menu.addSeparator();
    add(&menu, "Expand All",
        "Expand every row",
        [this]{ layer_view_->expandAll(); });
    add(&menu, "Collapse All",
        "Collapse every row",
        [this]{ layer_view_->collapseAll(); });

    menu.exec(layer_view_->viewport()->mapToGlobal(pos));
}

/**
*/
void ScatterPlotViewConfigWidget::colorModeChangedSlot(unsigned int mode)
{
    traced_assert(color_mode_label_);
    color_mode_label_->setText("Color Mode: " + colorModeText(mode));
}

/**
*/
QString ScatterPlotViewConfigWidget::colorModeText(unsigned int mode)
{
    switch (mode)
    {
        case 0: return "DSType";
        case 1: return "DBContent";
        case 2: return "Data Source";
        case 3: return "Data Source + Line";
        default: return "Unknown";
    }
}

/**
*/
void ScatterPlotViewConfigWidget::onDisplayChange_impl()
{
    traced_assert(use_connection_lines_);
    use_connection_lines_->setChecked(view_->useConnectionLines());
}

/**
*/
void ScatterPlotViewConfigWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableViewConfigWidget::viewInfoJSON_impl(info);

    info[ "use_connection_lines" ] = use_connection_lines_->isChecked();
}

/**
*/
void ScatterPlotViewConfigWidget::configChanged_impl()
{
    use_connection_lines_->blockSignals(true);
    use_connection_lines_->setChecked(view_->useConnectionLines());
    use_connection_lines_->blockSignals(false);
}

//void ScatterPlotViewConfigWidget::exportSlot()
//{
//    logdbg;
//    //assert(overwrite_check_);
//    traced_assert(export_button_);

//    export_button_->setDisabled(true);
//    //emit exportSignal(overwrite_check_->checkState() == Qt::Checked);
//}

//void ScatterPlotViewConfigWidget::exportDoneSlot(bool cancelled)
//{
//    traced_assert(export_button_);

//    export_button_->setDisabled(false);

//    if (!cancelled)
//    {
//        QMessageBox msgBox;
//        msgBox.setText("Export complete.");
//        msgBox.exec();
//    }
//}

