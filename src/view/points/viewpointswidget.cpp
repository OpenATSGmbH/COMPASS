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

#include "viewpointswidget.h"
#include "viewmanager.h"
#include "viewpoint.h"
#include "logger.h"
#include "viewpointstablemodel.h"
#include "compass.h"
#include "mainwindow.h"
#include "dbcontent/dbcontentmanager.h"
#include "viewpointsreportgenerator.h"
#include "viewpointsreportgeneratordialog.h"
#include "files.h"
//#include "dbinterface.h"

#include <QTableView>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSortFilterProxyModel>
#include <QFileDialog>
#include <QMessageBox>
#include <QShortcut>
#include <QToolBar>
#include <QMenu>

ViewPointsWidget::ViewPointsWidget(ViewManager& view_manager)
    : ToolBoxWidget(), view_manager_(view_manager)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    table_model_ = new ViewPointsTableModel(view_manager_);
    connect (table_model_, &ViewPointsTableModel::typesChangedSignal,
             this, &ViewPointsWidget::typesChangedSlot);
    connect (table_model_, &ViewPointsTableModel::statusesChangedSignal,
             this, &ViewPointsWidget::statusesChangedSlot);

    proxy_model_ = new QSortFilterProxyModel();
    proxy_model_->setSourceModel(table_model_);

    typesChangedSlot(table_model_->types()); // needs first update
    statusesChangedSlot(table_model_->statuses()); // needs first update

    table_view_ = new QTableView();
    table_view_->setModel(proxy_model_);
    table_view_->setSortingEnabled(true);
    table_view_->sortByColumn(0, Qt::AscendingOrder);
    table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_view_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
    table_view_->setIconSize(QSize(24, 24));
    table_view_->setWordWrap(true);
    table_view_->reset();
    // update done later

    connect(table_view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
            this, &ViewPointsWidget::currentRowChanged);

    // HACKY adjust row size but works
    connect(table_view_->horizontalHeader(), SIGNAL(sectionResized(int, int, int)),
            table_view_, SLOT(resizeRowsToContents()));

    //connect(table_view_, &QTableView::clicked, this, &ViewPointsWidget::onTableClickedSlot);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
    main_layout->addWidget(table_view_);

    {
        QHBoxLayout* button_layout = new QHBoxLayout();

        import_button_ = new QPushButton("Import");
        connect (import_button_, &QPushButton::clicked, this, &ViewPointsWidget::importSlot);
        button_layout->addWidget(import_button_);

        delete_all_button_ = new QPushButton("Delete All");
        connect (delete_all_button_, &QPushButton::clicked, this, &ViewPointsWidget::deleteAllSlot);
        button_layout->addWidget(delete_all_button_);

        export_button_ = new QPushButton("Export");
        connect (export_button_, &QPushButton::clicked, this, &ViewPointsWidget::exportSlot);
        button_layout->addWidget(export_button_);

        export_pdf_button_ = new QPushButton("Export PDF");
        connect (export_pdf_button_, &QPushButton::clicked, this, &ViewPointsWidget::exportPDFSlot);
        button_layout->addWidget(export_pdf_button_);

        main_layout->addLayout(button_layout);
    }

    setLayout(main_layout);

    //DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    // shortcuts
    {
        QShortcut* n_shortcut = new QShortcut(QKeySequence(tr("Down", "Next")), this);
        connect (n_shortcut, &QShortcut::activated, this, &ViewPointsWidget::selectNextSlot);

        QShortcut* o_shortcut = new QShortcut(QKeySequence(tr("O", "Open")), this);
        connect (o_shortcut, &QShortcut::activated, this, &ViewPointsWidget::setSelectedOpenSlot);

        QShortcut* c_shortcut = new QShortcut(QKeySequence(tr("C", "Closed")), this);
        connect (c_shortcut, &QShortcut::activated, this, &ViewPointsWidget::setSelectedClosedSlot);

        QShortcut* t_shortcut = new QShortcut(QKeySequence(tr("T", "Todo")), this);
        connect (t_shortcut, &QShortcut::activated, this, &ViewPointsWidget::setSelectedTodoSlot);

        QShortcut* e_shortcut = new QShortcut(QKeySequence(tr("E", "Edit")), this);
        connect (e_shortcut, &QShortcut::activated, this, &ViewPointsWidget::editCommentSlot);

        QShortcut* p_shortcut = new QShortcut(QKeySequence(tr("Up", "Previous")), this);
        connect (p_shortcut, &QShortcut::activated, this, &ViewPointsWidget::selectPreviousSlot);
    }

    QObject::connect(&COMPASS::instance(), &COMPASS::databaseOpenedSignal,
                     this, &ViewPointsWidget::databaseOpenedSlot);
    QObject::connect(&COMPASS::instance(), &COMPASS::databaseClosedSignal,
                     this, &ViewPointsWidget::databaseClosedSlot);
}

ViewPointsWidget::~ViewPointsWidget() = default;

/**
 */
QIcon ViewPointsWidget::toolIcon() const
{
    return QIcon(Utils::Files::getIconFilepath("view_points.png").c_str());
}

/**
 */
std::string ViewPointsWidget::toolName() const 
{
    return "View Points";
}

/**
 */
std::string ViewPointsWidget::toolInfo() const 
{
    return "View Points";
}

/**
 */
std::vector<std::string> ViewPointsWidget::toolLabels() const 
{
    return { "View", "Points" };
}

/**
 */
toolbox::ScreenRatio ViewPointsWidget::defaultScreenRatio() const 
{
    return ToolBoxWidget::defaultScreenRatio();
}

/**
 */
void ViewPointsWidget::addToConfigMenu(QMenu* menu) 
{
    //edit columns
    {
        auto submenu = menu->addMenu("Edit Columns");

        QStringList vp_columns = columns();
        QStringList filtered_columns = filteredColumns();

        for (auto& col : vp_columns)
        {
            QAction* action = submenu->addAction(col);
            action->setCheckable(true);
            action->setChecked(!filtered_columns.contains(col));

            connect (action, &QAction::triggered, [ = ] () { this->filterColumn(col); });
        }

        submenu->addSeparator();

        QAction* main_action = submenu->addAction("Show Only Main");
        connect (main_action, &QAction::triggered, this, &ViewPointsWidget::showOnlyMainColumns);

        QAction* all_action = submenu->addAction("Show All");
        connect (all_action, &QAction::triggered, this, &ViewPointsWidget::showAllColumns);

        QAction* none_action = submenu->addAction("Show None");
        connect (none_action, &QAction::triggered, this, &ViewPointsWidget::showNoColumns);
    }

    //filter by type
    {
        auto submenu = menu->addMenu("Filter By Type");

        QStringList vp_types       = types();
        QStringList filtered_types = filteredTypes();

        for (auto& type : vp_types)
        {
            QAction* action = submenu->addAction(type);
            action->setCheckable(true);
            action->setChecked(!filtered_types.contains(type));
            connect (action, &QAction::triggered, [ = ] () { this->filterType(type); });
        }

        submenu->addSeparator();

        auto all_action = submenu->addAction("Show All");
        connect (all_action, &QAction::triggered, this, &ViewPointsWidget::showAllTypes);

        auto none_action = submenu->addAction("Show None");
        connect (none_action, &QAction::triggered, this, &ViewPointsWidget::showNoTypes);
    }

    //filter by status
    {
        auto submenu = menu->addMenu("Filter By Status");

        QStringList vp_statuses = statuses();
        QStringList filtered_statuses = filteredStatuses();

        for (auto& status : vp_statuses)
        {
            QAction* action = submenu->addAction(status);
            action->setCheckable(true);
            action->setChecked(!filtered_statuses.contains(status));
            connect (action, &QAction::triggered, [ = ] () { this->filterStatus(status); });

        }

        submenu->addSeparator();

        QAction* all_action = submenu->addAction("Show All");
        connect (all_action, &QAction::triggered, this, &ViewPointsWidget::showAllStatuses);

        QAction* none_action = submenu->addAction("Show None");
        connect (none_action, &QAction::triggered, this, &ViewPointsWidget::showNoStatuses);
    }
}

/**
 */
void ViewPointsWidget::addToToolBar(QToolBar* tool_bar)
{
    auto action_prev = tool_bar->addAction("Select Previous [Up]");
    action_prev->setIcon(Utils::Files::IconProvider::getIcon("up.png"));
    connect(action_prev, &QAction::triggered, this, &ViewPointsWidget::selectPreviousSlot);

    auto action_open = tool_bar->addAction("Set Selected Status Open [O]");
    action_open->setIcon(Utils::Files::IconProvider::getIcon("not_recommended.png"));
    connect(action_open, &QAction::triggered, this, &ViewPointsWidget::setSelectedOpenSlot);

    auto action_closed = tool_bar->addAction("Set Selected Status Closed [C]");
    action_closed->setIcon(Utils::Files::IconProvider::getIcon("not_todo.png"));
    connect(action_closed, &QAction::triggered, this, &ViewPointsWidget::setSelectedClosedSlot);

    auto action_todo = tool_bar->addAction("Set Selected Status ToDo [T]");
    action_todo->setIcon(Utils::Files::IconProvider::getIcon("todo.png"));
    connect(action_todo, &QAction::triggered, this, &ViewPointsWidget::setSelectedTodoSlot);

    auto action_edit = tool_bar->addAction("Edit Comment [E]");
    action_edit->setIcon(Utils::Files::IconProvider::getIcon("comment.png"));
    connect(action_edit, &QAction::triggered, this, &ViewPointsWidget::editCommentSlot);

    auto action_next = tool_bar->addAction("Select Next [Down]");
    action_next->setIcon(Utils::Files::IconProvider::getIcon("down.png"));
    connect(action_next, &QAction::triggered, this, &ViewPointsWidget::selectNextSlot);
}

void ViewPointsWidget::loadingStarted()
{
    load_in_progress_ = true;

    traced_assert(table_view_);
    table_view_->setDisabled(true);
}

void ViewPointsWidget::loadingDone()
{
    load_in_progress_ = false;

    traced_assert(table_view_);
    table_view_->setDisabled(false);

    if (restore_focus_)
    {
        setFocus();
        restore_focus_ = false;
    }
}

void ViewPointsWidget::loadViewPoints()
{
    loginf;

    traced_assert(table_model_);
    table_model_->clearViewPoints();
    table_model_->loadViewPoints();

    traced_assert(table_view_);
    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

void ViewPointsWidget::clearViewPoints()
{
    traced_assert(table_model_);
    table_model_->clearViewPoints();
}

void ViewPointsWidget::addViewPoints(const std::vector <nlohmann::json>& viewpoints)
{
    traced_assert(table_model_);
    table_model_->addViewPoints(viewpoints);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
}

void ViewPointsWidget::resizeColumnsToContents()
{
    loginf;
    //table_model_->update();
    table_view_->resizeColumnsToContents();
}

ViewPointsTableModel* ViewPointsWidget::tableModel() const
{
    return table_model_;
}

QStringList ViewPointsWidget::types() const
{
    return types_;
}

QStringList ViewPointsWidget::filteredTypes() const
{
    return filtered_types_;
}

void ViewPointsWidget::filterType (QString type)
{
    traced_assert(types_.contains(type));

    if (filtered_types_.contains(type))
        filtered_types_.removeAll(type);
    else
        filtered_types_.append(type);

    updateFilteredTypes();
}

void ViewPointsWidget::showAllTypes ()
{
    filtered_types_.clear();

    updateFilteredTypes();
}

void ViewPointsWidget::showNoTypes ()
{
    filtered_types_ = types_;

    updateFilteredTypes();
}

void ViewPointsWidget::selectPreviousSlot()
{
    logdbg << "start";

    if (load_in_progress_)
        return;

    QModelIndexList list = table_view_->selectionModel()->selectedRows();
    traced_assert(list.size() <= 1);

    if (!list.size()) // none selected yet, start with first
    {
        table_view_->selectRow(table_view_->model()->rowCount()-1);
        return;
    }

    QModelIndex current_index = list.at(0);

    if (current_index.isValid())
    {
        QModelIndex next_index = table_view_->model()->index(current_index.row()-1, 0);

        if (next_index.isValid())
        {
            logdbg << "setting index row " << next_index.row();
            table_view_->selectRow(next_index.row());
        }
        else
        {
            logdbg << "invalid next index, step to last";
            table_view_->selectRow(table_view_->model()->rowCount()-1);
        }
    }
    else
        logwrn << "invalid current index";
}

void ViewPointsWidget::selectNextSlot()
{
    logdbg << "start";

    if (load_in_progress_)
        return;

    QModelIndexList list = table_view_->selectionModel()->selectedRows();
    traced_assert(list.size() <= 1);

    if (!list.size()) // none selected yet, start with first
    {
        table_view_->selectRow(0);
        return;
    }

    QModelIndex current_index = list.at(0);

    if (current_index.isValid())
    {
        QModelIndex next_index = table_view_->model()->index(current_index.row()+1, 0);

        if (next_index.isValid())
        {
            logdbg << "setting index row " << next_index.row();
            table_view_->selectRow(next_index.row());
        }
        else
        {
            logdbg << "invalid next index, step to first";
            table_view_->selectRow(0);
        }
    }
    else
        logwrn << "invalid current index";
}

void ViewPointsWidget::setSelectedOpenSlot()
{
    QModelIndexList list = table_view_->selectionModel()->selectedRows();
    traced_assert(list.size() <= 1);

    if (!list.size()) // none selected yet, start with first
    {
        logwrn << "no row selected";
        return;
    }

    QModelIndex current_index = list.at(0);

    if (current_index.isValid())
    {
        auto const source_index = proxy_model_->mapToSource(current_index);
        traced_assert(source_index.isValid());
        table_model_->setStatus(source_index, "open");
    }
    else
        logwrn << "invalid current index";
}

void ViewPointsWidget::setSelectedClosedSlot()
{
    QModelIndexList list = table_view_->selectionModel()->selectedRows();
    traced_assert(list.size() <= 1);

    if (!list.size()) // none selected yet, start with first
    {
        logwrn << "no row selected";
        return;
    }

    QModelIndex current_index = list.at(0);

    if (current_index.isValid())
    {
        auto const source_index = proxy_model_->mapToSource(current_index);
        traced_assert(source_index.isValid());
        table_model_->setStatus(source_index, "closed");
    }
    else
        logwrn << "invalid current index";
}

void ViewPointsWidget::setSelectedTodoSlot()
{
    QModelIndexList list = table_view_->selectionModel()->selectedRows();
    traced_assert(list.size() <= 1);

    if (!list.size()) // none selected yet, start with first
    {
        logwrn << "no row selected";
        return;
    }

    QModelIndex current_index = list.at(0);

    if (current_index.isValid())
    {
        auto const source_index = proxy_model_->mapToSource(current_index);
        traced_assert(source_index.isValid());
        table_model_->setStatus(source_index, "todo");
    }
    else
        logwrn << "invalid current index";
}

void ViewPointsWidget::editCommentSlot()
{
    QModelIndexList list = table_view_->selectionModel()->selectedRows();
    traced_assert(list.size() <= 1);

    if (!list.size()) // none selected yet, start with first
    {
        logwrn << "no row selected";
        return;
    }

    QModelIndex current_index = list.at(0);

    if (current_index.isValid())
    {
        auto const source_index = proxy_model_->mapToSource(current_index);
        traced_assert(source_index.isValid());

        QModelIndex src_comment_index = table_model_->index(source_index.row(), table_model_->commentColumn(),
                                                        QModelIndex());
        traced_assert(src_comment_index.isValid());
        QModelIndex comment_index = proxy_model_->mapFromSource(src_comment_index);
        traced_assert(comment_index.isValid());

        table_view_->edit(comment_index);
    }
    else
        logwrn << "invalid current index";
}


//void ViewPointsWidget::openCurrentSelectNext()
//{
//    loginf;
//}

//void ViewPointsWidget::closeCurrentSelectNext()
//{
//    loginf;
//}

void ViewPointsWidget::databaseOpenedSlot()
{
    loginf;

    loadViewPoints();
}

void ViewPointsWidget::databaseClosedSlot()
{
    loginf;

    traced_assert(table_model_);
    table_model_->clearViewPoints();
}


void ViewPointsWidget::exportSlot()
{
    loginf;

    QFileDialog dialog(nullptr);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDirectory(COMPASS::instance().lastUsedPath().c_str());
    dialog.setNameFilter("JSON Files (*.json)");
    dialog.setDefaultSuffix("json");
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);

//    if (!overwrite)
//        dialog.setOption(QFileDialog::DontConfirmOverwrite);

    if (dialog.exec())
    {
        QStringList file_names = dialog.selectedFiles();
        traced_assert(file_names.size() == 1);
        std::string filename = file_names.at(0).toStdString();

        loginf << "filename '" << filename << "'";
        traced_assert(table_model_);
        table_model_->exportViewPoints(filename);
    }
}

void ViewPointsWidget::exportPDFSlot()
{
    loginf;

    ViewPointsReportGeneratorDialog& dialog = view_manager_.viewPointsGenerator().dialog();
    dialog.exec();
}

void ViewPointsWidget::deleteAllSlot()
{
    loginf;

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr, "Delete All", "Delete All Viewpoints?",
    QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {
        traced_assert(table_model_);
        table_model_->deleteAllViewPoints();
        resizeColumnsToContents();
    }
}

void ViewPointsWidget::importSlot()
{
    loginf;

    COMPASS::instance().mainWindow().importViewPointsSlot();

    resizeColumnsToContents();
}

void ViewPointsWidget::currentRowChanged(const QModelIndex& current, const QModelIndex& previous)
{
    if (!current.isValid())
    {
        loginf << "invalid index";
        return;
    }

    auto const source_index = proxy_model_->mapToSource(current);
    traced_assert(source_index.isValid());

    unsigned int id = table_model_->getIdOf(source_index);

    loginf << "current id " << id;
    restore_focus_ = true;

    view_manager_.setCurrentViewPoint(&table_model_->viewPoint(id));
}

void ViewPointsWidget::typesChangedSlot(QStringList types)
{
    loginf;

    types_ = types;
}

void ViewPointsWidget::statusesChangedSlot(QStringList statuses)
{
    loginf;

    statuses_ = statuses;
}

void ViewPointsWidget::updateFilteredTypes ()
{
    QStringList regex_list;

    for (auto& type : types_)
        if (!filtered_types_.contains(type))
            regex_list.append(type);

    //"^(Correlation|banana)$"
    QString regex = "^(" + regex_list.join("|") + ")$";
    loginf << "'" << regex.toStdString() << "'";

    traced_assert(proxy_model_);
    proxy_model_->setFilterRegExp(QRegExp(regex, Qt::CaseSensitive, QRegExp::RegExp));
    proxy_model_->setFilterKeyColumn(table_model_->typeColumn()); // type
}

void ViewPointsWidget::updateFilteredStatuses ()
{
    QStringList regex_list;

    for (auto& status : statuses_)
        if (!filtered_statuses_.contains(status))
            regex_list.append(status);

    //"^(Correlation|banana)$"
    QString regex = "^(" + regex_list.join("|") + ")$";
    loginf << "'" << regex.toStdString() << "'";

    traced_assert(proxy_model_);
    proxy_model_->setFilterRegExp(QRegExp(regex, Qt::CaseSensitive, QRegExp::RegExp));
    proxy_model_->setFilterKeyColumn(table_model_->statusColumn()); // status
}

QStringList ViewPointsWidget::columns() const
{
    traced_assert(table_model_);

    return table_model_->tableColumns();
}

QStringList ViewPointsWidget::filteredColumns() const
{
    return filtered_columns_;
}


void ViewPointsWidget::filterColumn(QString name)
{
    traced_assert(columns().contains(name));

    if (filtered_columns_.contains(name))
        filtered_columns_.removeAll(name);
    else
        filtered_columns_.append(name);

    updateFilteredColumns();
}

void ViewPointsWidget::showOnlyMainColumns ()
{
    filtered_columns_.clear();

    QStringList default_cols = table_model_->defaultTableColumns();

    for (auto& col : columns())
        if (!default_cols.contains(col))
            filtered_columns_.append(col);

    updateFilteredColumns();
}

void ViewPointsWidget::showAllColumns()
{
    filtered_columns_.clear();

    updateFilteredColumns();
}

void ViewPointsWidget::showNoColumns()
{
    filtered_columns_ = columns();

    updateFilteredColumns();
}

void ViewPointsWidget::updateFilteredColumns()
{
    traced_assert(table_view_);

    unsigned int cnt=0;

    for (auto& col : columns())
    {
        table_view_->setColumnHidden(cnt, filtered_columns_.contains(col));
        ++cnt;
    }
}

QStringList ViewPointsWidget::statuses() const
{
    return statuses_;
}

QStringList ViewPointsWidget::filteredStatuses() const
{
    return filtered_statuses_;
}

void ViewPointsWidget::filterStatus (QString status)
{
    traced_assert(statuses_.contains(status));

    if (filtered_statuses_.contains(status))
        filtered_statuses_.removeAll(status);
    else
        filtered_statuses_.append(status);

    updateFilteredStatuses();
}

void ViewPointsWidget::showAllStatuses ()
{
    filtered_statuses_.clear();

    updateFilteredStatuses();
}

void ViewPointsWidget::showNoStatuses ()
{
    filtered_statuses_ = statuses_;

    updateFilteredStatuses();
}

std::vector<unsigned int> ViewPointsWidget::viewPoints()
{
    std::vector<unsigned int> data;

    for (auto& vp_it : table_model_->viewPoints())
        data.push_back(vp_it.id());

    return data;
}

std::vector<unsigned int> ViewPointsWidget::viewedViewPoints()
{
    std::vector<unsigned int> data;

    QModelIndex index = table_view_->model()->index(0, 0);

    while (index.isValid())
    {
        loginf << "row " << index.row();

        auto const source_index = proxy_model_->mapToSource(index);
        traced_assert(source_index.isValid());

        unsigned int id = table_model_->getIdOf(source_index);

        loginf << "id " << id;

        data.push_back(id);

        index = table_view_->model()->index(index.row()+1, 0);
    }

    return data;
}

