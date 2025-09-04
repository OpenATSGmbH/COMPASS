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

#include "eval/results/report/sectioncontenttable.h"
#include "eval/results/base/base.h"
#include "eval/results/base/single.h"
#include "evaluationmanager.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "latexvisitor.h"
#include "logger.h"
#include "stringconv.h"
#include "stringmat.h"
#include "asynctask.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTableView>
#include <QHeaderView>
#include <QSortFilterProxyModel>
#include <QMenu>
#include <QMessageBox>
#include <QClipboard>
#include <QApplication>
#include <QInputDialog>

#include "traced_assert.h"
#include <type_traits>
#include <iostream>

using namespace std;
using namespace Utils;

namespace EvaluationResultsReport
{

const int SectionContentTable::DoubleClickCheckIntervalMSecs = 300;

SectionContentTable::SectionContentTable(const string& name, unsigned int num_columns,
                                         vector<string> headings, Section* parent_section,
                                         EvaluationManager& eval_man, bool sortable,
                                         unsigned int sort_column, Qt::SortOrder order)
    : SectionContent(name, parent_section, eval_man), num_columns_(num_columns), headings_(headings),
      sortable_(sortable), sort_column_(sort_column), order_(order)
{
    click_action_timer_.setSingleShot(true);
    click_action_timer_.setInterval(DoubleClickCheckIntervalMSecs);
    connect(&click_action_timer_, &QTimer::timeout, this, &SectionContentTable::performClickAction);
}

void SectionContentTable::addRow (vector<QVariant> row, EvaluationRequirementResult::Base* result_ptr,
                                  QVariant annotation)
{
    traced_assert(row.size() == num_columns_);

    rows_.push_back(row);
    result_ptrs_.push_back(result_ptr);
    annotations_.push_back(annotation);

    traced_assert(annotations_.size() == rows_.size());
    traced_assert(result_ptrs_.size() == rows_.size());
}

void SectionContentTable::addToLayout (QVBoxLayout* layout)
{
    traced_assert(layout);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QHBoxLayout* upper_layout = new QHBoxLayout();

    upper_layout->addWidget(new QLabel(("Table: "+name_).c_str()));
    upper_layout->addStretch();

    options_button_ = new QPushButton("Options");
    connect (options_button_, &QPushButton::clicked, this, &SectionContentTable::showMenuSlot);
    upper_layout->addWidget(options_button_);

    main_layout->addLayout(upper_layout);

    if (!proxy_model_)
    {
        proxy_model_ = new TableQSortFilterProxyModel();
        proxy_model_->showUnused(show_unused_);
        proxy_model_->setSourceModel(this);
    }

    if (!table_view_)
    {
        table_view_ = new QTableView();
        table_view_->setModel(proxy_model_);

        if (sortable_)
        {
            table_view_->setSortingEnabled(true);
            table_view_->sortByColumn(sort_column_, order_);
        }

        table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
        table_view_->setSelectionMode(QAbstractItemView::SingleSelection);
        table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
        table_view_->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
        table_view_->setContextMenuPolicy(Qt::CustomContextMenu);
        table_view_->setWordWrap(true);
        table_view_->reset();

        connect(table_view_, &QTableView::customContextMenuRequested,
                this, &SectionContentTable::customContextMenuSlot);

        connect(table_view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
                this, &SectionContentTable::clickedSlot);
        connect(table_view_, &QTableView::pressed,
                this, &SectionContentTable::clickedSlot);
        connect(table_view_, &QTableView::doubleClicked,
                this, &SectionContentTable::doubleClickedSlot);
    }

//    if (num_columns_ > 5)
//        table_view_->horizontalHeader()->setMaximumSectionSize(150);

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();

    main_layout->addWidget(table_view_);

    layout->addLayout(main_layout);

}

void SectionContentTable::accept(LatexVisitor& v)
{
    loginf;

    createOnDemandIfNeeded();

    v.visit(this);
}

int SectionContentTable::rowCount(const QModelIndex& parent) const
{
    return rows_.size();
}

int SectionContentTable::columnCount(const QModelIndex& parent) const
{
    return num_columns_;
}

QVariant SectionContentTable::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    switch (role)
    {
    case Qt::DisplayRole:
    {
        logdbg << "display role: row " << index.row() << " col " << index.column();

        traced_assert(index.row() >= 0);
        traced_assert(index.row() < rows_.size());
        traced_assert(index.column() < num_columns_);

        return rows_.at(index.row()).at(index.column());
    }
    case Qt::BackgroundRole:
    {
        traced_assert(index.row() >= 0);
        traced_assert(index.row() < rows_.size());

        unsigned int row_index = index.row();

        if (result_ptrs_.at(row_index) && !result_ptrs_.at(row_index)->use())
            return QBrush(Qt::lightGray);
        else
            return QVariant();

    }
    case Qt::ForegroundRole:
    {
        traced_assert(index.row() >= 0);
        traced_assert(index.row() < rows_.size());
        traced_assert(index.column() < num_columns_);

        QVariant data = rows_.at(index.row()).at(index.column());

        if (data.userType() == QMetaType::QString)
        {
            if (data == "Passed")
                return QVariant(QColor(Qt::darkGreen));
            else if (data == "Failed")
                return QVariant(QColor(Qt::red));
        }
        if (data.userType() == QMetaType::Bool)
        {
            if (data == true)
                return QVariant(QColor(Qt::darkGreen));
            else if (data == false)
                return QVariant(QColor(Qt::red));
        }
        return QVariant(QColor(Qt::black));
    }
    default:
    {
        return QVariant();
    }
    }
}

QVariant SectionContentTable::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < num_columns_);
        return headings_.at(section).c_str();
    }

    return QVariant();
}

QModelIndex SectionContentTable::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

QModelIndex SectionContentTable::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

Qt::ItemFlags SectionContentTable::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    traced_assert(index.column() < headings_.size());

    return QAbstractItemModel::flags(index);
}

vector<string> SectionContentTable::headings() const
{
    return headings_;
}

unsigned int SectionContentTable::filteredRowCount () const
{
    if (!proxy_model_)
    {
        proxy_model_ = new TableQSortFilterProxyModel();
        proxy_model_->showUnused(show_unused_);

        SectionContentTable* tmp = const_cast<SectionContentTable*>(this); // hacky
        traced_assert(tmp);
        proxy_model_->setSourceModel(tmp);
    }

    return proxy_model_->rowCount();
}

std::vector<std::string> SectionContentTable::sortedRowStrings(unsigned int row, bool latex) const
{
    if (!proxy_model_)
    {
        proxy_model_ = new TableQSortFilterProxyModel();
        proxy_model_->showUnused(show_unused_);

        SectionContentTable* tmp = const_cast<SectionContentTable*>(this); // hacky
        traced_assert(tmp);
        proxy_model_->setSourceModel(tmp);
    }

    if (!table_view_)
    {
        table_view_ = new QTableView();
        table_view_->setModel(proxy_model_);

        if (sortable_)
        {
            table_view_->setSortingEnabled(true);
            table_view_->sortByColumn(sort_column_, order_);
        }

        table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
        table_view_->setSelectionMode(QAbstractItemView::SingleSelection);
        table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
        table_view_->setContextMenuPolicy(Qt::CustomContextMenu);
        table_view_->setWordWrap(true);
        table_view_->reset();

        connect(table_view_, &QTableView::customContextMenuRequested,
                this, &SectionContentTable::customContextMenuSlot);

        connect(table_view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
                this, &SectionContentTable::currentRowChangedSlot);
        connect(table_view_, &QTableView::doubleClicked,
                this, &SectionContentTable::doubleClickedSlot);
    }

    logdbg << "row " << row << " rows " << proxy_model_->rowCount()
           << " data rows " << rows_.size();
    traced_assert(row < proxy_model_->rowCount());
    traced_assert(row < rows_.size());

    vector<string> result;

    for (unsigned int col=0; col < num_columns_; ++col)
    {
        QModelIndex index = proxy_model_->index(row, col);
        traced_assert(index.isValid());
        // get string can convert to latex
        if (latex)
            result.push_back(String::latexString(proxy_model_->data(index).toString().toStdString()));
        else
            result.push_back(proxy_model_->data(index).toString().toStdString());
    }

    return result;
}

bool SectionContentTable::hasReference (unsigned int row) const
{
    // if (!proxy_model_)
    // {
    //     proxy_model_ = new TableQSortFilterProxyModel();
    //     proxy_model_->showUnused(show_unused_);

    //     SectionContentTable* tmp = const_cast<SectionContentTable*>(this); // hacky
    //     traced_assert(tmp);
    //     proxy_model_->setSourceModel(tmp);
    // }

    // traced_assert(row < proxy_model_->rowCount());
    // traced_assert(row < rows_.size());

    // QModelIndex index = proxy_model_->index(row, 0);
    // traced_assert(index.isValid());

    // auto const source_index = proxy_model_->mapToSource(index);
    // traced_assert(source_index.isValid());

    // unsigned int row_index = source_index.row();

    // return result_ptrs_.at(row_index)
    //         && result_ptrs_.at(row_index)->hasReference(*this, annotations_.at(row_index));

    return false;
}

std::string SectionContentTable::reference (unsigned int row) const
{
    // if (!proxy_model_)
    // {
    //     proxy_model_ = new TableQSortFilterProxyModel();
    //     proxy_model_->showUnused(show_unused_);

    //     SectionContentTable* tmp = const_cast<SectionContentTable*>(this); // hacky
    //     traced_assert(tmp);
    //     proxy_model_->setSourceModel(tmp);
    // }

    // traced_assert(row < proxy_model_->rowCount());
    // traced_assert(row < rows_.size());

    // QModelIndex index = proxy_model_->index(row, 0);
    // traced_assert(index.isValid());

    // auto const source_index = proxy_model_->mapToSource(index);
    // traced_assert(source_index.isValid());

    // unsigned int row_index = source_index.row();

    // traced_assert(result_ptrs_.at(row_index)
    //         && result_ptrs_.at(row_index)->hasReference(*this, annotations_.at(row_index)));

    // string tmp = result_ptrs_.at(row_index)->reference(*this, annotations_.at(row_index));
    // //e.g. "Report:Results:"+getRequirementSectionID();
    // traced_assert(tmp.size() >= 14);

    // if (tmp == "Report:Results")
    //     return "";

    // traced_assert(tmp.rfind("Report:Results:", 0) == 0);
    // tmp.erase(0,15);

    // return tmp;

    return "";
}

bool SectionContentTable::showUnused() const
{
    return show_unused_;
}

void SectionContentTable::showUnused(bool value)
{
    loginf << "value " << value;

    traced_assert(proxy_model_);

    beginResetModel();

    show_unused_ = value;
    proxy_model_->showUnused(show_unused_);

    endResetModel();
}

void SectionContentTable::registerCallBack (const std::string& name, std::function<void()> func)
{
    traced_assert(!callback_map_.count(name));
    callback_map_.emplace(name, func);
}
void SectionContentTable::executeCallBack (const std::string& name)
{
    traced_assert(callback_map_.count(name));
    callback_map_.at(name)();
}

void SectionContentTable::setCreateOnDemand(
        std::function<void(void)> create_on_demand_fnc)
{
    create_on_demand_ = true;

    create_on_demand_fnc_ = create_on_demand_fnc;

    already_created_by_demand_ = false;
}

bool SectionContentTable::canFetchMore(const QModelIndex& parent) const
{
    return create_on_demand_ && !already_created_by_demand_;
}

void SectionContentTable::fetchMore(const QModelIndex& parent)
{
    createOnDemandIfNeeded();
}

void SectionContentTable::createOnDemandIfNeeded()
{
    if (create_on_demand_ && !already_created_by_demand_)
    {
        loginf << "creating";

        QApplication::setOverrideCursor(Qt::WaitCursor);

        beginResetModel();

        create_on_demand_fnc_();
        already_created_by_demand_ = true;

        endResetModel();

        if (table_view_)
            table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);

        QApplication::restoreOverrideCursor();
    }
}

void SectionContentTable::currentRowChangedSlot(const QModelIndex& current, const QModelIndex& previous)
{
    clickedSlot(current);
}

void SectionContentTable::clickedSlot(const QModelIndex& index)
{
    if (!index.isValid())
    {
        loginf << "invalid index";
        return;
    }

    if (QApplication::mouseButtons() & Qt::RightButton)
    {
        loginf << "RMB click ignored";
        return;
    }

    auto const source_index = proxy_model_->mapToSource(index);
    traced_assert(source_index.isValid());

    traced_assert(source_index.row() >= 0);
    traced_assert(source_index.row() < rows_.size());

    last_clicked_row_index_ = source_index.row();

    //fire timer to perform delayed click action
    click_action_timer_.start();
}

void SectionContentTable::performClickAction()
{
    // //double click did not interrupt click action => perform
    // if (!last_clicked_row_index_.has_value())
    //     return;

    // unsigned int row_index = last_clicked_row_index_.value();
    // last_clicked_row_index_.reset();

    // if (result_ptrs_.at(row_index) && result_ptrs_.at(row_index)->hasViewableData(*this, annotations_.at(row_index)))
    // {
    //     loginf << "index has associated viewable";

    //     std::shared_ptr<nlohmann::json::object_t> viewable;

    //     if (result_ptrs_.at(row_index)->viewableDataReady())
    //     {
    //         //view data ready, just get it
    //         viewable = result_ptrs_.at(row_index)->viewableData(*this, annotations_.at(row_index)); 
    //     }
    //     else
    //     {
    //         //recompute async and show wait dialog, this may take a while...
    //         auto func = [ & ] (const AsyncTaskState& state, AsyncTaskProgressWrapper& progress)
    //         {
    //             viewable = result_ptrs_.at(row_index)->viewableData(*this, annotations_.at(row_index)); 
    //             return Result::succeeded();
    //         };
            
    //         AsyncFuncTask task(func, "Updating Contents", "Updating contents...", false);
    //         task.runAsyncDialog();
    //     }

    //     traced_assert(viewable);

    //     eval_man_.setViewableDataConfig(*viewable.get());
    // }
}

void SectionContentTable::doubleClickedSlot(const QModelIndex& index)
{
    // //double click detected => interrupt any previously triggered click action
    // click_action_timer_.stop();

    // if (!index.isValid())
    // {
    //     loginf << "invalid index";
    //     return;
    // }

    // auto const source_index = proxy_model_->mapToSource(index);
    // traced_assert(source_index.isValid());

    // traced_assert(source_index.row() >= 0);
    // traced_assert(source_index.row() < rows_.size());

    // loginf << "row " << source_index.row();

    // unsigned int row_index = source_index.row();

    // if (result_ptrs_.at(row_index) && result_ptrs_.at(row_index)->hasReference(*this, annotations_.at(row_index)))
    // {
    //     string reference = result_ptrs_.at(row_index)->reference(*this, annotations_.at(row_index));
    //     traced_assert(reference.size());

    //     loginf << "index has associated reference '"
    //            << reference << "'";

    //     eval_man_.showResultId(reference);
    // }
    // else
    //     loginf << "index has no associated reference";
}

void SectionContentTable::customContextMenuSlot(const QPoint& p)
{
    logdbg << "start";

    traced_assert(table_view_);

    QModelIndex index = table_view_->indexAt(p);
    if (!index.isValid())
        return;

    auto const source_index = proxy_model_->mapToSource(index);
    traced_assert(source_index.isValid());

    loginf << "row " << index.row() << " src " << source_index.row();

    traced_assert(source_index.row() >= 0);
    traced_assert(source_index.row() < rows_.size());

    unsigned int row_index = source_index.row();

    if (result_ptrs_.at(row_index) && result_ptrs_.at(row_index)->isSingle())
    {
        EvaluationRequirementResult::Single* single_result =
                static_cast<EvaluationRequirementResult::Single*>(result_ptrs_.at(row_index));
        traced_assert(single_result);

        QMenu menu;

        unsigned int utn = single_result->utn();
        loginf << "utn " << utn;

        traced_assert(eval_man_.calculator().data().hasTargetData(utn));

        const EvaluationTargetData& target_data = eval_man_.calculator().data().targetData(utn);

        if (target_data.use())
        {
            QAction* action = new QAction("Remove", this);
            connect (action, &QAction::triggered, this, &SectionContentTable::removeUTNSlot);
            action->setData(utn);

            menu.addAction(action);
        }
        else
        {
            QAction* action = new QAction("Add", this);
            connect (action, &QAction::triggered, this, &SectionContentTable::addUTNSlot);
            action->setData(utn);

            menu.addAction(action);
        }

        QAction* action = new QAction("Show Full UTN", this);
        connect (action, &QAction::triggered, this, &SectionContentTable::showFullUTNSlot);
        action->setData(utn);
        menu.addAction(action);

        QAction* action2 = new QAction("Show Surrounding Data", this);
        connect (action2, &QAction::triggered, this, &SectionContentTable::showSurroundingDataSlot);
        action2->setData(utn);
        menu.addAction(action2);

        menu.exec(table_view_->viewport()->mapToGlobal(p));
    }
    else
        loginf << "no associated utn";
}

void SectionContentTable::addUTNSlot ()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    unsigned int utn = action->data().toUInt();

    loginf << "utn " << utn;

    COMPASS::instance().dbContentManager().utnUseEval(utn, true);
    //eval_man_.useUTN(utn, true, true);

}

void SectionContentTable::removeUTNSlot ()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    unsigned int utn = action->data().toUInt();

    bool ok;
    QString text =
        QInputDialog::getText(nullptr, "Remove UTN "+QString::number(utn),
                              "Please provide a comment as reason:", QLineEdit::Normal, "", &ok);
    if (ok && !text.isEmpty())
    {

        loginf << "utn " << utn;

        COMPASS::instance().dbContentManager().utnUseEval(utn, false);
        COMPASS::instance().dbContentManager().utnComment(utn, text.toStdString());
    }
}

void SectionContentTable::showFullUTNSlot ()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    unsigned int utn = action->data().toUInt();

    loginf << "utn " << utn;

    eval_man_.calculator().showFullUTN(utn);
}

void SectionContentTable::showSurroundingDataSlot ()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    unsigned int utn = action->data().toUInt();

    loginf << "utn " << utn;

    //eval_man_.calculator().showSurroundingData(utn);
}

void SectionContentTable::showMenuSlot()
{
    QMenu menu;

    //        toogle_show_unused_button_ = new QPushButton("Toggle Show Unused");
    //        connect (toogle_show_unused_button_, &QPushButton::clicked, this, &SectionContentTable::toggleShowUnusedSlot);
    //        upper_layout->addWidget(toogle_show_unused_button_);

    //        copy_button_ = new QPushButton("Copy Content");
    //        connect (copy_button_, &QPushButton::clicked, this, &SectionContentTable::copyContentSlot);
    //        upper_layout->addWidget(copy_button_);

    QAction* unused_action = new QAction("Toggle Show Unused", this);
    connect (unused_action, &QAction::triggered, this, &SectionContentTable::toggleShowUnusedSlot);
    menu.addAction(unused_action);

    QAction* copy_action = new QAction("Copy Content", this);
    connect (copy_action, &QAction::triggered, this, &SectionContentTable::copyContentSlot);
    menu.addAction(copy_action);

    for (auto& cb_it : callback_map_)
    {
        QAction* copy_action = new QAction(cb_it.first.c_str(), this);
        connect (copy_action, &QAction::triggered, this, &SectionContentTable::executeCallBackSlot);
        copy_action->setData(cb_it.first.c_str());
        menu.addAction(copy_action);
    }

    menu.exec(QCursor::pos());
}

void SectionContentTable::toggleShowUnusedSlot()
{
    showUnused(!show_unused_);
}

void SectionContentTable::copyContentSlot()
{
    loginf;

    stringstream ss;

    unsigned int num_cols = headings_.size();

    // headings
    for (unsigned int cnt=0; cnt < num_cols; ++cnt)
    {
        if (cnt == 0)
            ss << headings_.at(cnt);
        else
            ss <<  ";" << headings_.at(cnt);
    }
    ss << "\n";

    unsigned int num_rows = proxy_model_->rowCount();

    vector<string> row_strings;

    for (unsigned int row=0; row < num_rows; ++row)
    {
        row_strings = sortedRowStrings(row, false);
        traced_assert(row_strings.size() == num_cols);

        for (unsigned int cnt=0; cnt < num_cols; ++cnt)
        {
            if (cnt == 0)
                ss << row_strings.at(cnt);
            else
                ss <<  ";" << row_strings.at(cnt);
        }
        ss << "\n";
    }

    QApplication::clipboard()->setText(ss.str().c_str());
}

void SectionContentTable::executeCallBackSlot()
{
    QAction* action = dynamic_cast<QAction*> (QObject::sender());
    traced_assert(action);

    string name = action->data().toString().toStdString();

    loginf << "name " << name;

    traced_assert(callback_map_.count(name));
    executeCallBack(name);
}

Utils::StringTable SectionContentTable::toStringTable() const
{
    return Utils::StringTable(this);
}

nlohmann::json SectionContentTable::toJSON(bool rowwise,
                                           const std::vector<int>& cols) const
{
    return toStringTable().toJSON(rowwise, cols);
}

}
