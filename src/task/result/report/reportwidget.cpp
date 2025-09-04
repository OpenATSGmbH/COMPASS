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

#include "reportwidget.h"
#include "report/treeitem.h"
#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontentfigure.h"
#include "report/sectioncontenttable.h"
#include "files.h"
#include "logger.h"
#include "popupmenu.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QPushButton>
#include <QSettings>
#include <QMenu>
#include <QWidgetAction>
#include <QApplication>
#include <QClipboard>
#include <QScrollBar>

#include <osg/Group>

using namespace Utils;

namespace ResultReport
{

const std::string ReportWidget::FieldConfigScrollPosV = "container_scroll_pos_v";
const std::string ReportWidget::FieldConfigScrollPosH = "container_scroll_pos_h";

ReportWidget::ReportWidget(TaskResultsWidget& task_result_widget)
    : QWidget(nullptr), task_result_widget_(task_result_widget)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    { // button layout
        QHBoxLayout* button_layout = new QHBoxLayout();

        QIcon left_icon(Files::IconProvider::getIcon("arrow_to_left.png"));

        back_button_ = new QPushButton ();
        back_button_->setIcon(left_icon);
        connect (back_button_, &QPushButton::clicked, this, &ReportWidget::stepBackSlot);
        button_layout->addWidget(back_button_);

        current_section_label_ = new QLabel;
        button_layout->addWidget(current_section_label_);

        button_layout->addStretch();

        sections_button_ = new QPushButton("Sections");
        button_layout->addWidget(sections_button_);

        main_layout->addLayout(button_layout);
    }

    tree_view_ = new QTreeView;
    tree_view_->setModel(&tree_model_);
    tree_view_->setRootIsDecorated(false);
    tree_view_->expandToDepth(3);
    tree_view_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    tree_view_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

    connect (tree_view_, &QTreeView::clicked, this, &ReportWidget::itemClickedSlot);
    connect (tree_view_, &QTreeView::doubleClicked, this, &ReportWidget::itemDblClickedSlot);
    connect (tree_view_, &QTreeView::customContextMenuRequested, this, &ReportWidget::contextMenuSlot);

    sections_menu_.reset(new PopupMenu(sections_button_, tree_view_));

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

ReportWidget::~ReportWidget()
{
}

void ReportWidget::setReport(const std::shared_ptr<Report>& report)
{
    tree_model_.setReport(report);

    tree_view_->setRootIsDecorated(false);
    tree_view_->expandToDepth(3);
    tree_view_->setFixedSize(500, 1000);

    id_history_.clear();

    current_section_ = nullptr;
    current_section_label_->setText("");

    updateBackButton();
    updateCurrentSectionLabel();

    showResultWidget(nullptr, false);
}

void ReportWidget::clear()
{
    tree_model_.clear();
    id_history_.clear();

    current_section_ = nullptr;
    current_section_label_->setText("");

    updateBackButton();
    updateCurrentSectionLabel();
}

void ReportWidget::expand()
{
    loginf;

    tree_view_->expandToDepth(3);
}

void ReportWidget::selectId (const std::string& id,
                             bool show_figure,
                             const nlohmann::json& config)
{
    loginf << "id '" << id << "'";

    QModelIndex index = tree_model_.findItem(id);

    if (!index.isValid())
    {
        logerr << "id '" << id << "' not found";
        return;
    }

    traced_assert(tree_view_);
    tree_view_->selectionModel()->clear();

    expandAllParents(index);

    tree_view_->selectionModel()->select(index, QItemSelectionModel::Select | QItemSelectionModel::Rows);
    tree_view_->scrollTo(index);

    //note: we should preload on-demand contents such as on-demand tables before showing a figure. 
    //otherwise loading the figure on-demand and loading the table on-demand could interfere with each other, 
    //as there is a timegap before a table is loaded on-demand.
    //(either signal-slot mechanics or multithreading related)
    bool preload_ondemand_contents = show_figure;

    triggerItem(index, preload_ondemand_contents);

    if (show_figure)
        showFigure(index);

    if (current_section_ && config.is_object() && !config.empty())
    {
        bool ok = configureSection(config);
        if (ok)
            loginf << "restored config of section '" << id << "'";
    }
}

void ReportWidget::reshowLastId ()
{
    if (id_history_.size() >= 1)
    {
        selectId(*id_history_.rbegin(), false, {}); // select last one

        id_history_.pop_back(); // remove re-added id. slightly hacky
    }
}

void ReportWidget::triggerItem (const QModelIndex& index,
                                bool preload_ondemand_contents)
{
    TreeItem* item = static_cast<TreeItem*>(index.internalPointer());
    traced_assert(item);

    id_history_.push_back(item->id());

    loginf << "name " << item->name() << " id " << item->id();

    if (dynamic_cast<ResultReport::Report*>(item))
    {
        loginf << "root report";
        showResultWidget(nullptr, preload_ondemand_contents);
    }
    else if (dynamic_cast<ResultReport::Section*>(item))
    {
        ResultReport::Section* section = dynamic_cast<ResultReport::Section*>(item);
        traced_assert(section);

        loginf << "section " << section->name();
        showResultWidget(section, preload_ondemand_contents);
    }

    updateBackButton();
    updateCurrentSectionLabel();
}

void ReportWidget::itemClickedSlot(const QModelIndex& index)
{
    triggerItem(index, false);
}

void ReportWidget::itemDblClickedSlot(const QModelIndex& index)
{
    triggerItem(index, false);

    sections_menu_->close();
}

void ReportWidget::contextMenuSlot(const QPoint& pos)
{
    auto index = tree_view_->indexAt(pos);
    if (!index.isValid())
        return;

    Section* section = static_cast<Section*>(index.internalPointer());
    if (!section)
        return;

    QMenu menu;

    auto action_copy_id = menu.addAction("Copy Section ID");
    connect(action_copy_id, &QAction::triggered, 
        [ section ] () { qApp->clipboard()->setText(QString::fromStdString(section->id())); });

    menu.exec(tree_view_->mapToGlobal(pos));
}

void ReportWidget::showFigure(const QModelIndex& index)
{
    TreeItem* item = static_cast<TreeItem*>(index.internalPointer());
    traced_assert(item);

    loginf << "name " << item->name() << " id " << item->id();

    if (dynamic_cast<ResultReport::Report*>(item))
    {
        return;
    }
    else if (dynamic_cast<ResultReport::Section*>(item))
    {
        ResultReport::Section* section = dynamic_cast<ResultReport::Section*>(item);
        traced_assert(section);

        loginf << "section " << section->name();

        auto figures = section->getFigures();
        if (!figures.empty())
        {
            figures[ 0 ]->view();
        }
    }
}

void ReportWidget::stepBackSlot()
{
    loginf;

    traced_assert(id_history_.size() > 1);

    id_history_.pop_back(); // remove last entry
    reshowLastId(); // show last id

    updateBackButton();
    updateCurrentSectionLabel();
}

void ReportWidget::showResultWidget(Section* section, 
                                    bool preload_ondemand_contents)
{
    traced_assert(results_widget_);

    current_section_ = section;

    if (!section)
    {
        while (results_widget_->count() > 0)  // remove all widgets
            results_widget_->removeWidget(results_widget_->widget(0));

        return;
    }

    auto widget = section->getContentWidget(preload_ondemand_contents);

    if (results_widget_->indexOf(widget) < 0)
        results_widget_->addWidget(widget);

    results_widget_->setCurrentWidget(widget);
}

void ReportWidget::expandAllParents (QModelIndex index)
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

void ReportWidget::updateBackButton ()
{
    traced_assert(back_button_);

    back_button_->setEnabled(id_history_.size() > 1);
}

void ReportWidget::updateCurrentSectionLabel()
{
    current_section_label_->setText(current_section_ ? QString::fromStdString(current_section_->id()) : "");
}

std::string ReportWidget::currentSectionID() const
{
    return current_section_ ? current_section_->id() : "";
}

nlohmann::json ReportWidget::currentSectionConfig() const
{
    if (!current_section_)
        return {};

    auto config = current_section_->jsonConfig();
    if (!config.is_object() || config.empty())
        return {};

    config[ FieldConfigScrollPosV ] = tree_view_->verticalScrollBar()->isVisible() ? tree_view_->verticalScrollBar()->value() : -1;
    config[ FieldConfigScrollPosH ] = tree_view_->horizontalScrollBar()->isVisible() ? tree_view_->horizontalScrollBar()->value() : -1;

    return config;
}

bool ReportWidget::configureSection(const nlohmann::json& config)
{
    if (!current_section_)
        return false;

    if (!config.is_object() || config.empty())
        return false;

    if (!config.contains(FieldConfigScrollPosV) || 
        !config.contains(FieldConfigScrollPosH))
        return false;

    int scroll_pos_v = config[FieldConfigScrollPosV].get<int>();
    int scroll_pos_h = config[FieldConfigScrollPosH].get<int>();

    //restore widget scroll positions
    if (scroll_pos_v >= 0 && tree_view_->verticalScrollBar()->isVisible())
        tree_view_->verticalScrollBar()->setValue(scroll_pos_v);

    if (scroll_pos_h >= 0 && tree_view_->horizontalScrollBar()->isVisible())
        tree_view_->horizontalScrollBar()->setValue(scroll_pos_h);

    //configure section
    return current_section_->configure(config);
}

}
