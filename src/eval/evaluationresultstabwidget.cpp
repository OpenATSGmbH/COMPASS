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

#include "evaluationresultstabwidget.h"

#include "evaluationmanagerwidget.h"
#include "evaluationmanager.h"
#include "evaluationresultsgenerator.h"
#include "eval/results/report/treemodel.h"
#include "eval/results/report/treeitem.h"
#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttable.h"
#include "eval/results/report/sectioncontentfigure.h"
#include "files.h"
#include "logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QPushButton>
#include <QSettings>
#include <QMenu>
#include <QWidgetAction>

using namespace EvaluationResultsReport;
using namespace Utils;

EvaluationResultsTabWidget::EvaluationResultsTabWidget(EvaluationManager& eval_man, EvaluationManagerWidget& man_widget)
    : QWidget(nullptr), eval_man_(eval_man), man_widget_(man_widget)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    { // button layout
        QHBoxLayout* button_layout = new QHBoxLayout();

        QIcon left_icon(Files::IconProvider::getIcon("arrow_to_left.png"));

        back_button_ = new QPushButton ();
        back_button_->setIcon(left_icon);
        connect (back_button_, &QPushButton::clicked, this, &EvaluationResultsTabWidget::stepBackSlot);
        button_layout->addWidget(back_button_);

        button_layout->addStretch();

        sections_button_ = new QPushButton("Sections");
        button_layout->addWidget(sections_button_);

        main_layout->addLayout(button_layout);

    }
    //QHBoxLayout* res_layout = new QHBoxLayout();

    tree_view_ = new QTreeView;
    //tree_view_->setModel(&eval_man_.calculator().resultsGenerator().resultsModel());
    tree_view_->setRootIsDecorated(false);
    tree_view_->expandToDepth(3);
    tree_view_->setFixedSize(500, 1000);

    connect (tree_view_, &QTreeView::clicked, this, &EvaluationResultsTabWidget::itemClickedSlot);

    sections_menu_.reset(new QMenu);
    
    auto w_action = new QWidgetAction(sections_menu_.get());
    w_action->setDefaultWidget(tree_view_);

    sections_menu_->addAction(w_action);
    sections_button_->setMenu(sections_menu_.get());

    // results stack

    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);

    results_widget_ = new QStackedWidget();

    scroll_area->setWidget(results_widget_);

    main_layout->addWidget(scroll_area);

    setContentsMargins(0, 0, 0, 0);
    setLayout(main_layout);

    updateBackButton();
}

EvaluationResultsTabWidget::~EvaluationResultsTabWidget() = default;

void EvaluationResultsTabWidget::expand()
{
    loginf;

    tree_view_->expandToDepth(3);
}

namespace 
{
    void iterateTreeModel(const EvaluationResultsReport::TreeModel& model, const QModelIndex& index, const std::string& spacing)
    {
        std::cout << spacing << model.data(index, Qt::UserRole).toString().toStdString() << std::endl;

        if (model.hasChildren(index))
        {
            int rc = model.rowCount(index);
            int cc = model.columnCount(index);
            for (int r = 0; r < rc; ++r)
                for (int c = 0; c < cc; ++c)
                    iterateTreeModel(model, model.index(r, c, index), spacing + "   ");
        }
    }
}

void EvaluationResultsTabWidget::selectId (const std::string& id, 
                                           bool show_figure)
{
    loginf << "id '" << id << "'";

    //const auto& model = eval_man_.resultsGenerator().resultsModel();
    //iterateTreeModel(model, model.index(0, 0), "");

    // QModelIndex index = eval_man_.calculator().resultsGenerator().resultsModel().findItem(id);

    // if (!index.isValid())
    // {
    //     logerr << "id '" << id << "' not found";
    //     return;
    // }

    // traced_assert(tree_view_);
    // tree_view_->selectionModel()->clear();

    // expandAllParents(index);

    // tree_view_->selectionModel()->select(index, QItemSelectionModel::Select | QItemSelectionModel::Rows);
    // tree_view_->scrollTo(index);

    // itemClickedSlot(index);

    // if (show_figure)
    //     showFigure(index);
}

void EvaluationResultsTabWidget::reshowLastId ()
{
    if (id_history_.size() >= 1)
    {
        selectId(*id_history_.rbegin()); // select last one

        id_history_.pop_back(); // remove re-added id. slightly hacky
    }
}

void EvaluationResultsTabWidget::itemClickedSlot(const QModelIndex& index)
{
    TreeItem* item = static_cast<TreeItem*>(index.internalPointer());
    traced_assert(item);

    id_history_.push_back(item->id());

    loginf << "name " << item->name() << " id " << item->id();

    if (dynamic_cast<EvaluationResultsReport::RootItem*>(item))
    {
        loginf << "root";
        showResultWidget(nullptr);
    }
    else if (dynamic_cast<EvaluationResultsReport::Section*>(item))
    {
        EvaluationResultsReport::Section* section = dynamic_cast<EvaluationResultsReport::Section*>(item);
        traced_assert(section);

        loginf << "section " << section->name();
        showResultWidget(section->getContentWidget());
    }

    updateBackButton();
}

void EvaluationResultsTabWidget::showFigure(const QModelIndex& index)
{
    TreeItem* item = static_cast<TreeItem*>(index.internalPointer());
    traced_assert(item);

    loginf << "name " << item->name() << " id " << item->id();

    if (dynamic_cast<EvaluationResultsReport::RootItem*>(item))
    {
        return;
    }
    else if (dynamic_cast<EvaluationResultsReport::Section*>(item))
    {
        EvaluationResultsReport::Section* section = dynamic_cast<EvaluationResultsReport::Section*>(item);
        traced_assert(section);

        loginf << "section " << section->name();
        
        auto figures = section->getFigures();
        if (!figures.empty())
            figures[ 0 ]->viewSlot();
    }
}

void EvaluationResultsTabWidget::stepBackSlot()
{
    loginf;

    traced_assert(id_history_.size() > 1);

    id_history_.pop_back(); // remove last entry
    reshowLastId(); // show last id

    updateBackButton();
}

void EvaluationResultsTabWidget::showResultWidget(QWidget* widget)
{
    traced_assert(results_widget_);

    if (!widget)
    {
        while (results_widget_->count() > 0)  // remove all widgets
            results_widget_->removeWidget(results_widget_->widget(0));
        return;
    }

    if (results_widget_->indexOf(widget) < 0)
        results_widget_->addWidget(widget);

    results_widget_->setCurrentWidget(widget);
}

void EvaluationResultsTabWidget::expandAllParents (QModelIndex index)
{
    if (!index.isValid()) {
        return;
    }

    QModelIndex parent_index;

    for (parent_index = index.parent(); parent_index.isValid(); parent_index = parent_index.parent())
    {
        if (!tree_view_->isExpanded(parent_index))
            tree_view_->expand(parent_index);
    }
}

void EvaluationResultsTabWidget::updateBackButton ()
{
    traced_assert(back_button_);

    back_button_->setEnabled(id_history_.size() > 1);
}
