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
#include "report/reportdocumentview.h"
#include "files.h"
#include "logger.h"
#include "popupmenu.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSplitter>
#include <QPushButton>
#include <QMenu>
#include <QWidgetAction>
#include <QApplication>
#include <QClipboard>
#include <QScrollBar>
#include <QSignalBlocker>

#include <osg/Group>

using namespace Utils;

namespace ResultReport
{

const std::string ReportWidget::FieldConfigScrollPosV = "container_scroll_pos_v";
const std::string ReportWidget::FieldConfigScrollPosH = "container_scroll_pos_h";

namespace
{
    const int SidebarMinWidth     = 180;
    const int SidebarDefaultWidth = 280;
}

const int ReportWidget::SidebarHideWidth       = SidebarDefaultWidth + 500;
const int ReportWidget::SidebarShowHysteresis  = 50;

/**
 * The sidebar hides below SidebarHideWidth and shows again only above it plus the hysteresis, so it does
 * not show and hide repeatedly while the width changes.
 */
bool ReportWidget::sidebarShownForWidth(bool shown, int width)
{
    if (shown)
        return width >= SidebarHideWidth;

    return width >= SidebarHideWidth + SidebarShowHysteresis;
}

ReportWidget::ReportWidget(TaskResultsWidget& task_result_widget)
    : QWidget(nullptr), task_result_widget_(task_result_widget)
{
    setObjectName("report_widget");

    QVBoxLayout* main_layout = new QVBoxLayout();

    { // button layout
        QHBoxLayout* button_layout = new QHBoxLayout();

        QIcon left_icon(Files::IconProvider::getIcon("arrow_to_left.png"));

        back_button_ = new QPushButton ();
        back_button_->setObjectName("report_back_button");
        back_button_->setIcon(left_icon);
        connect (back_button_, &QPushButton::clicked, this, &ReportWidget::stepBackSlot);
        button_layout->addWidget(back_button_);

        current_section_label_ = new QLabel;
        current_section_label_->setObjectName("report_section_label");
        button_layout->addWidget(current_section_label_);

        button_layout->addStretch();

        sections_button_ = new QPushButton("Sections");
        sections_button_->setObjectName("report_sections_button");
        sections_button_->setToolTip("Show the section tree (disabled while the section sidebar is shown)");
        button_layout->addWidget(sections_button_);

        main_layout->addLayout(button_layout);
    }

    tree_view_ = new QTreeView;
    tree_view_->setObjectName("report_section_tree");
    tree_view_->setModel(&tree_model_);
    tree_view_->setRootIsDecorated(false);
    tree_view_->expandToDepth(3);
    tree_view_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    tree_view_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

    connect (tree_view_, &QTreeView::clicked, this, &ReportWidget::itemClickedSlot);
    connect (tree_view_, &QTreeView::doubleClicked, this, &ReportWidget::itemDblClickedSlot);
    connect (tree_view_, &QTreeView::customContextMenuRequested, this, &ReportWidget::contextMenuSlot);

    sections_menu_.reset(new PopupMenu(sections_button_, tree_view_));

    sidebar_view_ = new QTreeView;
    sidebar_view_->setObjectName("report_section_sidebar");
    sidebar_view_->setModel(&tree_model_);
    sidebar_view_->setHeaderHidden(true);
    sidebar_view_->setRootIsDecorated(false);
    sidebar_view_->setMinimumWidth(SidebarMinWidth);
    sidebar_view_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

    connect (sidebar_view_, &QTreeView::clicked, this, &ReportWidget::itemClickedSlot);
    connect (sidebar_view_, &QTreeView::customContextMenuRequested, this, &ReportWidget::contextMenuSlot);
    connect (sidebar_view_, &QTreeView::expanded, this, &ReportWidget::sidebarExpansionChangedSlot);
    connect (sidebar_view_, &QTreeView::collapsed, this, &ReportWidget::sidebarExpansionChangedSlot);

    document_view_ = new ReportDocumentView;

    connect (document_view_, &ReportDocumentView::currentSectionChangedSignal,
             this, &ReportWidget::currentSectionChangedSlot);

    splitter_ = new QSplitter(Qt::Horizontal);
    splitter_->setChildrenCollapsible(false);
    splitter_->addWidget(sidebar_view_);
    splitter_->addWidget(document_view_);
    splitter_->setStretchFactor(0, 0);
    splitter_->setStretchFactor(1, 1);
    splitter_->setSizes({ SidebarDefaultWidth, 4 * SidebarDefaultWidth });

    main_layout->addWidget(splitter_);

    setContentsMargins(0, 0, 0, 0);
    setLayout(main_layout);

    applySidebarShown(true);

    updateBackButton();
}

/**
 */
void ReportWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    applySidebarShown(sidebarShownForWidth(sidebar_shown_, width()));
}

/**
 */
void ReportWidget::applySidebarShown(bool shown)
{
    sidebar_shown_ = shown;

    sidebar_view_->setVisible(shown);
    sections_button_->setEnabled(!shown);
}

ReportWidget::~ReportWidget()
{
    //the document holds widgets of the sections of the report, release them while the report exists
    document_view_->clear();

    QObject::disconnect(report_clear_connection_);
}

void ReportWidget::setReport(const std::shared_ptr<Report>& report)
{
    document_view_->clear();

    QObject::disconnect(report_clear_connection_);

    tree_model_.setReport(report);
    report_ = report;

    report_clear_connection_ = connect(report.get(), &Report::aboutToClearSignal,
                                       this, &ReportWidget::reportAboutToClearSlot);

    tree_view_->setRootIsDecorated(false);
    tree_view_->expandToDepth(3);
    tree_view_->setFixedSize(500, 1000);

    expandSidebarInitially();

    id_history_.clear();

    current_section_ = nullptr;
    current_section_label_->setText("");

    updateBackButton();
    updateCurrentSectionLabel();

    document_view_->setRootSection(report->rootSection().get(), collapsedSections());
    document_outdated_ = false;
}

void ReportWidget::clear()
{
    document_view_->clear();

    QObject::disconnect(report_clear_connection_);

    tree_model_.clear();
    report_.reset();
    document_outdated_ = false;

    id_history_.clear();

    current_section_ = nullptr;
    current_section_label_->setText("");

    updateBackButton();
    updateCurrentSectionLabel();
}

/**
 * Stops all reads of the document from the database (built sections, preload), e.g. before the
 * database closes. The document is built again with the next jump to a section.
 */
void ReportWidget::releaseDocument()
{
    document_view_->clear();
    document_outdated_ = true;
}

/**
 * The report is about to delete all its sections. Drops all widgets and tree indexes which refer to
 * them. The document is built again with the next jump to a section.
 */
void ReportWidget::reportAboutToClearSlot()
{
    document_view_->clear();
    document_outdated_ = true;

    tree_model_.beginReset();
    tree_model_.endReset();

    id_history_.clear();

    current_section_ = nullptr;

    updateBackButton();
    updateCurrentSectionLabel();
}

void ReportWidget::currentSectionChangedSlot(Section* section)
{
    current_section_ = section;

    updateCurrentSectionLabel();

    if (section)
        selectInTrees(modelIndex(section));
}

/**
 * The document holds the sections whose parents are all expanded in the sidebar.
 */
void ReportWidget::sidebarExpansionChangedSlot()
{
    if (document_outdated_)
        return;

    document_view_->setCollapsedSections(collapsedSections());
}

void ReportWidget::expand()
{
    loginf;

    tree_view_->expandToDepth(3);
}

void ReportWidget::selectID(const std::string& id,
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

    selectInTrees(index);

    bool show_id_figure = always_show_section_figure_ || show_figure;

    //note: we should preload on-demand contents such as on-demand tables before showing a figure. 
    //otherwise loading the figure on-demand and loading the table on-demand could interfere with each other, 
    //as there is a timegap before a table is loaded on-demand.
    //(either signal-slot mechanics or multithreading related)
    bool preload_ondemand_contents = show_id_figure;

    triggerItem(index, preload_ondemand_contents);

    if (show_id_figure)
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
        selectID(*id_history_.rbegin(), false, {}); // select last one

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

    if (document_outdated_ && report_)
    {
        expandSidebarInitially();

        document_view_->setRootSection(report_->rootSection().get(), collapsedSections());
        document_outdated_ = false;
    }

    Section* section = dynamic_cast<ResultReport::Section*>(item);

    //the report item itself shows the document from the top
    if (!section && report_)
        section = report_->rootSection().get();

    //puts the section into the document
    expandAllParents(sidebar_view_, index);

    if (section)
        document_view_->scrollToSection(section, 0, preload_ondemand_contents);

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
    QTreeView* view = qobject_cast<QTreeView*>(sender());
    if (!view)
        view = tree_view_;

    auto index = view->indexAt(pos);
    if (!index.isValid())
        return;

    Section* section = static_cast<Section*>(index.internalPointer());
    if (!section)
        return;

    QMenu menu;

    auto action_copy_id = menu.addAction("Copy Section ID");
    connect(action_copy_id, &QAction::triggered, 
        [ section ] () { qApp->clipboard()->setText(QString::fromStdString(section->id())); });

    menu.exec(view->viewport()->mapToGlobal(pos));
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

void ReportWidget::expandAllParents (QTreeView* view, QModelIndex index)
{
    if (!index.isValid()) {
        return;
    }

    QModelIndex parent_index;

    for (parent_index = index.parent(); parent_index.isValid(); parent_index = parent_index.parent())
    {
        if (!view->isExpanded(parent_index))
            view->expand(parent_index);
    }
}

/**
 * Expands all sections in the sidebar except the per-target sections, the caller builds the document.
 */
void ReportWidget::expandSidebarInitially()
{
    QSignalBlocker blocker(sidebar_view_);

    std::vector<QModelIndex> stack = { tree_model_.index(0, 0) };

    while (!stack.empty())
    {
        QModelIndex index = stack.back();
        stack.pop_back();

        const int num_rows = tree_model_.rowCount(index);
        if (!index.isValid() || num_rows == 0)
            continue;

        auto section = dynamic_cast<Section*>(static_cast<TreeItem*>(index.internalPointer()));
        if (!section || section->perTargetSection())
            continue;

        sidebar_view_->expand(index);

        for (int row = 0; row < num_rows; ++row)
            stack.push_back(tree_model_.index(row, 0, index));
    }
}

/**
 * The sections with sub-sections which are collapsed in the sidebar and whose parents are all expanded.
 */
ReportDocumentLayout::SectionSet ReportWidget::collapsedSections() const
{
    ReportDocumentLayout::SectionSet collapsed;

    std::vector<QModelIndex> stack = { tree_model_.index(0, 0) };

    while (!stack.empty())
    {
        QModelIndex index = stack.back();
        stack.pop_back();

        const int num_rows = tree_model_.rowCount(index);
        if (!index.isValid() || num_rows == 0)
            continue;

        if (!sidebar_view_->isExpanded(index))
        {
            collapsed.insert(dynamic_cast<const Section*>(static_cast<TreeItem*>(index.internalPointer())));
            continue;
        }

        for (int row = 0; row < num_rows; ++row)
            stack.push_back(tree_model_.index(row, 0, index));
    }

    return collapsed;
}

/**
 * Selects the item in the popup tree and in the sidebar, without a jump in the document.
 */
void ReportWidget::selectInTrees(const QModelIndex& index)
{
    if (!index.isValid())
        return;

    for (QTreeView* view : { tree_view_, sidebar_view_ })
    {
        expandAllParents(view, index);

        view->selectionModel()->setCurrentIndex(index, QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
        view->scrollTo(index);
    }
}

/**
 */
QModelIndex ReportWidget::modelIndex(Section* section) const
{
    if (!section)
        return {};

    //the root section is the only child of the report
    Section* parent = section->parentSection();
    if (!parent)
        return tree_model_.index(0, 0);

    return tree_model_.index(section->row(), 0, modelIndex(parent));
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

    config[ FieldConfigScrollPosV ] = document_view_->offsetInCurrentSection();
    config[ FieldConfigScrollPosH ] = document_view_->horizontalScrollBar()->value();

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

    Section* section = current_section_;

    //configure the contents first, the scroll position belongs to the configured layout
    bool ok = section->configure(config);

    if (scroll_pos_v >= 0)
        document_view_->scrollToSection(section, scroll_pos_v, false);

    if (scroll_pos_h >= 0)
        document_view_->horizontalScrollBar()->setValue(scroll_pos_h);

    return ok;
}

}

