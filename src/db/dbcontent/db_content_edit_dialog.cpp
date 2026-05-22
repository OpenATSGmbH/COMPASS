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

#include "dbcontent/db_content_edit_dialog.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/variable/metavariabledetailwidget.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variabledetailwidget.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <algorithm>

using namespace std;

namespace dbContent
{

namespace
{
    constexpr int kKindRole = Qt::UserRole + 1;
    constexpr int kNameRole = Qt::UserRole + 2;
    constexpr int kDBContentRole = Qt::UserRole + 3;

    enum ItemKind
    {
        Kind_Group = 1,    // top-level "Meta" / "CAT001" / ... container
        Kind_MetaVar = 2,
        Kind_DBContentVar = 3,
    };

    constexpr const char* kMetaGroupName = "Meta";
}

DBContentEditDialog::DBContentEditDialog(DBContentManager& dbcont_man, QWidget* parent)
    : QDialog(parent), dbcont_man_(dbcont_man)
{
    if (dbcont_man_.compass().expertMode())
        setWindowTitle("DBContent");
    else
        setWindowTitle("Show DBContent");

    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);
    setMinimumSize(QSize(900, 600));

    QSettings settings("COMPASS", "DBContentEditDialog");

    auto* main_layout = new QVBoxLayout();

    splitter_ = new QSplitter(Qt::Horizontal);

    // tree
    tree_widget_ = new QTreeWidget();
    tree_widget_->setHeaderHidden(true);
    tree_widget_->setColumnCount(1);
    tree_widget_->setUniformRowHeights(true);
    connect(tree_widget_, &QTreeWidget::currentItemChanged,
            this, &DBContentEditDialog::itemSelectedSlot);
    connect(tree_widget_, &QTreeWidget::itemExpanded,
            this, &DBContentEditDialog::itemExpandedSlot);
    splitter_->addWidget(tree_widget_);

    // detail stack on the right
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);

    detail_stack_ = new QStackedWidget();

    placeholder_ = new QLabel("Select a Meta or DBContent variable from the tree.");
    static_cast<QLabel*>(placeholder_)->setAlignment(Qt::AlignCenter);
    detail_stack_->addWidget(placeholder_);

    meta_detail_widget_ = new MetaVariableDetailWidget(dbcont_man_);
    detail_stack_->addWidget(meta_detail_widget_);

    var_detail_widget_ = new VariableDetailWidget(dbcont_man_);
    detail_stack_->addWidget(var_detail_widget_);

    scroll->setWidget(detail_stack_);
    splitter_->addWidget(scroll);

    splitter_->setStretchFactor(0, 0);
    splitter_->setStretchFactor(1, 1);
    splitter_->restoreState(settings.value("mainSplitterSizes").toByteArray());
    if (splitter_->sizes().value(0, 0) <= 0)
        splitter_->setSizes({320, 580});

    main_layout->addWidget(splitter_, 1);

    // close button
    auto* button_layout = new QHBoxLayout();
    button_layout->addStretch();

    auto* ok_button = new QPushButton("Close");
    ok_button->setIcon(QIcon());
    ok_button->setToolTip("Close the dialog");
    connect(ok_button, &QPushButton::clicked, this, &DBContentEditDialog::okClickedSlot);
    button_layout->addWidget(ok_button);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    rebuildTree();
    showPlaceholder();
}

DBContentEditDialog::~DBContentEditDialog()
{
    QSettings settings("COMPASS", "DBContentEditDialog");
    settings.setValue("mainSplitterSizes", splitter_->saveState());
}

void DBContentEditDialog::rebuildTree()
{
    traced_assert(tree_widget_);

    tree_widget_->blockSignals(true);
    tree_widget_->clear();

    // Meta group
    {
        auto* meta_item = new QTreeWidgetItem(tree_widget_);
        meta_item->setText(0, kMetaGroupName);
        meta_item->setData(0, kKindRole, Kind_Group);
        meta_item->setData(0, kNameRole, kMetaGroupName);

        std::vector<std::string> meta_names;
        meta_names.reserve(dbcont_man_.metaVariables().size());
        for (const auto& mv_it : dbcont_man_.metaVariables())
            meta_names.push_back(mv_it.first);
        std::sort(meta_names.begin(), meta_names.end());

        for (const auto& name : meta_names)
        {
            auto* leaf = new QTreeWidgetItem(meta_item);
            leaf->setText(0, name.c_str());
            leaf->setData(0, kKindRole, Kind_MetaVar);
            leaf->setData(0, kNameRole, name.c_str());
            leaf->setToolTip(0, dbcont_man_.metaVariable(name).info().c_str());
        }
    }

    // One top-level item per DBContent, alphabetically by DBContent name.
    std::vector<std::string> dbc_names;
    dbc_names.reserve(static_cast<size_t>(std::distance(dbcont_man_.begin(), dbcont_man_.end())));
    for (auto it = dbcont_man_.begin(); it != dbcont_man_.end(); ++it)
        dbc_names.push_back(it->first);
    std::sort(dbc_names.begin(), dbc_names.end());

    for (const auto& dbc_name : dbc_names)
    {
        auto* dbc_item = new QTreeWidgetItem(tree_widget_);
        dbc_item->setText(0, dbc_name.c_str());
        dbc_item->setData(0, kKindRole, Kind_Group);
        dbc_item->setData(0, kNameRole, dbc_name.c_str());

        DBContent& dbc = dbcont_man_.dbContent(dbc_name);

        std::vector<std::string> var_names;
        for (const auto& v_it : dbc.variables())
            var_names.push_back(v_it.first);
        std::sort(var_names.begin(), var_names.end());

        for (const auto& var_name : var_names)
        {
            auto* leaf = new QTreeWidgetItem(dbc_item);
            leaf->setText(0, var_name.c_str());
            leaf->setData(0, kKindRole, Kind_DBContentVar);
            leaf->setData(0, kNameRole, var_name.c_str());
            leaf->setData(0, kDBContentRole, dbc_name.c_str());
            leaf->setToolTip(0, dbc.variable(var_name).info().c_str());
        }
    }

    tree_widget_->collapseAll();
    tree_widget_->blockSignals(false);
}

void DBContentEditDialog::itemExpandedSlot(QTreeWidgetItem* item)
{
    if (!item)
        return;

    // Only top-level groups participate in the one-expanded-at-a-time rule.
    if (item->parent() != nullptr)
        return;

    tree_widget_->blockSignals(true);
    for (int i = 0; i < tree_widget_->topLevelItemCount(); ++i)
    {
        QTreeWidgetItem* sibling = tree_widget_->topLevelItem(i);
        if (sibling != item && sibling->isExpanded())
            tree_widget_->collapseItem(sibling);
    }
    tree_widget_->blockSignals(false);
}

void DBContentEditDialog::itemSelectedSlot(QTreeWidgetItem* current, QTreeWidgetItem* /*previous*/)
{
    if (!current)
    {
        showPlaceholder();
        return;
    }

    int kind = current->data(0, kKindRole).toInt();

    if (kind == Kind_MetaVar)
    {
        std::string name = current->data(0, kNameRole).toString().toStdString();
        if (!dbcont_man_.existsMetaVariable(name))
        {
            showPlaceholder();
            return;
        }
        meta_detail_widget_->show(dbcont_man_.metaVariable(name));
        detail_stack_->setCurrentWidget(meta_detail_widget_);
    }
    else if (kind == Kind_DBContentVar)
    {
        std::string dbc_name = current->data(0, kDBContentRole).toString().toStdString();
        std::string var_name = current->data(0, kNameRole).toString().toStdString();

        if (!dbcont_man_.existsDBContent(dbc_name))
        {
            showPlaceholder();
            return;
        }

        DBContent& dbc = dbcont_man_.dbContent(dbc_name);
        if (!dbc.hasVariable(var_name))
        {
            showPlaceholder();
            return;
        }

        var_detail_widget_->show(dbc.variable(var_name));
        detail_stack_->setCurrentWidget(var_detail_widget_);
    }
    else
    {
        showPlaceholder();
    }
}

void DBContentEditDialog::showPlaceholder()
{
    if (meta_detail_widget_)
        meta_detail_widget_->clear();
    if (var_detail_widget_)
        var_detail_widget_->clear();
    detail_stack_->setCurrentWidget(placeholder_);
}

void DBContentEditDialog::okClickedSlot()
{
    emit okSignal();
    accept();
}

}
