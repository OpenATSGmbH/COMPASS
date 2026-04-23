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

#include "deletedatadialog.h"
#include "db_context_manager.h"
#include "data_source.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "stringconv.h"
#include "json.hpp"
#include "logger.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QMenu>
#include <QLabel>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

using namespace std;

DeleteDataDialog::DeleteDataDialog(context::DBContextManager& ctx_man, QWidget* parent)
    : QDialog(parent), ctx_man_(ctx_man)
{
    setWindowTitle("Delete Data");
    setMinimumSize(QSize(1000, 800));

    createUI();
}

void DeleteDataDialog::createUI()
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    // dbcontent checkboxes in grid (2 rows)
    QGroupBox* dbc_group = new QGroupBox("DBContent");
    QGridLayout* dbc_layout = new QGridLayout;

    DBContentManager& dbcont_man = ctx_man_.compass().dbContentManager();

    vector<string> dbc_names;
    for (auto it = dbcont_man.begin(); it != dbcont_man.end(); ++it)
    {
        if (it->second->hasData())
            dbc_names.push_back(it->first);
    }

    unsigned int cols = (dbc_names.size() + 1) / 2;
    for (unsigned int i = 0; i < dbc_names.size(); ++i)
    {
        QCheckBox* cb = new QCheckBox(dbc_names[i].c_str());
        cb->setChecked(false);
        dbc_layout->addWidget(cb, i / cols, i % cols);
        dbcontent_checks_[dbc_names[i]] = cb;
    }

    dbc_group->setLayout(dbc_layout);
    main_layout->addWidget(dbc_group);

    // select all / select none buttons
    QHBoxLayout* dbc_button_layout = new QHBoxLayout;

    QPushButton* select_all_btn = new QPushButton("Select All");
    connect(select_all_btn, &QPushButton::clicked, [this]() {
        for (auto& cb_it : dbcontent_checks_)
            cb_it.second->setChecked(true);
    });
    dbc_button_layout->addWidget(select_all_btn);

    QPushButton* select_none_btn = new QPushButton("Select None");
    connect(select_none_btn, &QPushButton::clicked, [this]() {
        for (auto& cb_it : dbcontent_checks_)
            cb_it.second->setChecked(false);
    });
    dbc_button_layout->addWidget(select_none_btn);

    dbc_button_layout->addStretch();

    main_layout->addLayout(dbc_button_layout);
    main_layout->addSpacing(10);

    // data sources tree with checkboxes
    QGroupBox* ds_group = new QGroupBox("Data Sources");
    QVBoxLayout* ds_group_layout = new QVBoxLayout;

    ds_tree_ = new QTreeWidget;
    ds_tree_->setHeaderLabels({"Name", "Count"});
    ds_tree_->header()->setStretchLastSection(false);
    ds_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    ds_tree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);

    populateDataSourcesTree();

    connect(ds_tree_, &QTreeWidget::itemChanged, this, &DeleteDataDialog::itemChangedSlot);

    ds_tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ds_tree_, &QTreeWidget::customContextMenuRequested, this, &DeleteDataDialog::showTreeContextMenu);

    ds_group_layout->addWidget(ds_tree_);
    ds_group->setLayout(ds_group_layout);
    main_layout->addWidget(ds_group, 1);

    // buttons
    QDialogButtonBox* button_box = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    button_box->button(QDialogButtonBox::Ok)->setText("Delete");
    button_box->button(QDialogButtonBox::Ok)->setIcon(QIcon());
    button_box->button(QDialogButtonBox::Cancel)->setIcon(QIcon());

    connect(button_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);

    main_layout->addWidget(button_box);

    setLayout(main_layout);
}

void DeleteDataDialog::populateDataSourcesTree()
{
    // hierarchy: DSType → DataSource → Line (all with checkboxes)
    map<string, QTreeWidgetItem*> dstype_items;

    if (ctx_man_.hasActiveContext())
    {
        for (const auto& ds : ctx_man_.activeContext().dataSources())
        {
            unsigned int ds_id = ds.id();

            if (!ctx_man_.hasNumInserted(ds_id))
                continue;

            const string ds_type = ds.dsType();

            // create DSType item if needed
            if (!dstype_items.count(ds_type))
            {
                auto* dstype_item = new QTreeWidgetItem(ds_tree_);
                dstype_item->setText(0, ds_type.c_str());
                dstype_item->setFlags(dstype_item->flags() | Qt::ItemIsUserCheckable);
                dstype_item->setCheckState(0, Qt::Unchecked);

                QFont font = dstype_item->font(0);
                font.setBold(true);
                dstype_item->setFont(0, font);

                dstype_items[ds_type] = dstype_item;
            }

            // data source item - compute total count from numInsertedLinesMap
            unsigned int total_count = 0;
            auto lines_map = ctx_man_.numInsertedLinesMap(ds_id);
            for (const auto& line_it : lines_map)
                total_count += line_it.second;

            auto* ds_item = new QTreeWidgetItem(dstype_items[ds_type]);
            ds_item->setText(0, ds.name().c_str());
            ds_item->setText(1, QString::number(total_count));
            ds_item->setData(0, Qt::UserRole, ds_id);
            ds_item->setFlags(ds_item->flags() | Qt::ItemIsUserCheckable);
            ds_item->setCheckState(0, Qt::Unchecked);

            // line items
            for (const auto& line_it : lines_map)
            {
                auto* line_item = new QTreeWidgetItem(ds_item);
                line_item->setText(0, Utils::String::lineStrFrom(line_it.first).c_str());
                line_item->setText(1, QString::number(line_it.second));
                line_item->setData(0, Qt::UserRole, ds_id);
                line_item->setData(0, Qt::UserRole + 1, line_it.first);
                line_item->setFlags(line_item->flags() | Qt::ItemIsUserCheckable);
                line_item->setCheckState(0, Qt::Unchecked);
            }
        }
    }

    ds_tree_->expandAll();
}

void DeleteDataDialog::collectSelections(set<string>& dbcontents,
                                         vector<DeleteDataDialog::SelectedDS>& data_sources) const
{
    // collect selected dbcontents (restricted set, if any)
    for (const auto& cb_it : dbcontent_checks_)
    {
        if (!cb_it.second->isChecked())
            continue;
        if (!restricted_dbcontents_.empty() && !restricted_dbcontents_.count(cb_it.first))
            continue;
        dbcontents.insert(cb_it.first);
    }

    // collect selected data sources + lines from tree
    for (int t = 0; t < ds_tree_->topLevelItemCount(); ++t)
    {
        QTreeWidgetItem* dstype_item = ds_tree_->topLevelItem(t);

        // DSType must be checked for any DS under it to count
        if (dstype_item->checkState(0) != Qt::Checked)
            continue;

        for (int d = 0; d < dstype_item->childCount(); ++d)
        {
            QTreeWidgetItem* ds_item = dstype_item->child(d);

            // DS must be checked for any lines under it to count
            if (ds_item->checkState(0) != Qt::Checked)
                continue;

            DeleteDataDialog::SelectedDS sel;
            sel.ds_id = ds_item->data(0, Qt::UserRole).toUInt();
            sel.name = ds_item->text(0).toStdString();

            // collect checked lines
            set<unsigned int> checked_lines;
            for (int l = 0; l < ds_item->childCount(); ++l)
            {
                QTreeWidgetItem* line_item = ds_item->child(l);
                if (line_item->checkState(0) == Qt::Checked)
                    checked_lines.insert(line_item->data(0, Qt::UserRole + 1).toUInt());
            }

            if (checked_lines.empty())
            {
                // DS checked but no lines checked → all lines
                sel.all_lines = true;
            }
            else
            {
                sel.all_lines = false;
                sel.line_ids = checked_lines;
            }

            data_sources.push_back(sel);
        }
    }
}

nlohmann::json DeleteDataDialog::selectedDeleteInfo() const
{
    nlohmann::json result = nlohmann::json::array();

    set<string> sel_dbcontents;
    vector<DeleteDataDialog::SelectedDS> sel_ds;
    collectSelections(sel_dbcontents, sel_ds);

    if (sel_dbcontents.empty() && sel_ds.empty())
        return result;

    // 1) selected DBContents: delete ALL data for each (from all data sources)
    for (const auto& dbcontent_name : sel_dbcontents)
    {
        nlohmann::json entry;
        entry["dbcontent"] = dbcontent_name;
        // no "data_sources" → delete all
        result.push_back(entry);
    }

    // 2) selected data sources: delete ALL dbcontent data from those sources/lines
    //    (skip dbcontents already fully deleted above)
    if (!sel_ds.empty())
    {
        DBContentManager& dbcont_man = ctx_man_.compass().dbContentManager();

        for (auto it = dbcont_man.begin(); it != dbcont_man.end(); ++it)
        {
            const string& dbcontent_name = it->first;

            // already fully deleted by dbcontent selection
            if (sel_dbcontents.count(dbcontent_name))
                continue;

            // honour the caller's restriction set (scoped delete)
            if (!restricted_dbcontents_.empty() &&
                !restricted_dbcontents_.count(dbcontent_name))
                continue;

            if (!it->second->hasData())
                continue;

            nlohmann::json ds_array = nlohmann::json::array();

            for (const auto& sel : sel_ds)
            {
                if (ctx_man_.numInserted(sel.ds_id, dbcontent_name) == 0)
                    continue;

                nlohmann::json ds_entry;
                ds_entry["ds_id"] = sel.ds_id;

                if (!sel.all_lines)
                {
                    nlohmann::json line_ids = nlohmann::json::array();
                    auto per_line = ctx_man_.numInsertedPerLine(sel.ds_id, dbcontent_name);
                    for (unsigned int line_id : sel.line_ids)
                    {
                        if (per_line.count(line_id) && per_line.at(line_id) > 0)
                            line_ids.push_back(line_id);
                    }
                    if (line_ids.empty())
                        continue;
                    ds_entry["line_ids"] = line_ids;
                }

                ds_array.push_back(ds_entry);
            }

            if (!ds_array.empty())
            {
                nlohmann::json entry;
                entry["dbcontent"] = dbcontent_name;
                entry["data_sources"] = ds_array;
                result.push_back(entry);
            }
        }
    }

    return result;
}

void DeleteDataDialog::itemChangedSlot(QTreeWidgetItem* /*item*/, int /*column*/)
{
    // no propagation — each item is toggled independently, like in DataSourcesUseWidget
}

void DeleteDataDialog::setCheckRecursive(QTreeWidgetItem* item, Qt::CheckState state)
{
    item->setCheckState(0, state);
    for (int i = 0; i < item->childCount(); ++i)
        setCheckRecursive(item->child(i), state);
}

void DeleteDataDialog::showTreeContextMenu(const QPoint& pos)
{
    QTreeWidgetItem* item = ds_tree_->itemAt(pos);

    QMenu menu;

    // Select All — entire tree
    menu.addAction("Select All", [this]() {
        ds_tree_->blockSignals(true);
        for (int i = 0; i < ds_tree_->topLevelItemCount(); ++i)
            setCheckRecursive(ds_tree_->topLevelItem(i), Qt::Checked);
        ds_tree_->blockSignals(false);
    });

    // Select None — entire tree
    menu.addAction("Select None", [this]() {
        ds_tree_->blockSignals(true);
        for (int i = 0; i < ds_tree_->topLevelItemCount(); ++i)
            setCheckRecursive(ds_tree_->topLevelItem(i), Qt::Unchecked);
        ds_tree_->blockSignals(false);
    });

    if (item)
    {
        menu.addSeparator();

        // Select All Siblings — same parent, same level, propagate down
        menu.addAction("Select All Siblings", [this, item]() {
            ds_tree_->blockSignals(true);
            QTreeWidgetItem* parent = item->parent();
            if (parent)
            {
                for (int i = 0; i < parent->childCount(); ++i)
                    setCheckRecursive(parent->child(i), Qt::Checked);
            }
            else
            {
                for (int i = 0; i < ds_tree_->topLevelItemCount(); ++i)
                    setCheckRecursive(ds_tree_->topLevelItem(i), Qt::Checked);
            }
            ds_tree_->blockSignals(false);
        });

        // Select None Siblings
        menu.addAction("Select None Siblings", [this, item]() {
            ds_tree_->blockSignals(true);
            QTreeWidgetItem* parent = item->parent();
            if (parent)
            {
                for (int i = 0; i < parent->childCount(); ++i)
                    setCheckRecursive(parent->child(i), Qt::Unchecked);
            }
            else
            {
                for (int i = 0; i < ds_tree_->topLevelItemCount(); ++i)
                    setCheckRecursive(ds_tree_->topLevelItem(i), Qt::Unchecked);
            }
            ds_tree_->blockSignals(false);
        });

        // Select All Children — only if item has children
        if (item->childCount() > 0)
        {
            menu.addSeparator();

            menu.addAction("Select All Children", [this, item]() {
                ds_tree_->blockSignals(true);
                for (int i = 0; i < item->childCount(); ++i)
                    setCheckRecursive(item->child(i), Qt::Checked);
                ds_tree_->blockSignals(false);
            });

            menu.addAction("Select None Children", [this, item]() {
                ds_tree_->blockSignals(true);
                for (int i = 0; i < item->childCount(); ++i)
                    setCheckRecursive(item->child(i), Qt::Unchecked);
                ds_tree_->blockSignals(false);
            });
        }
    }

    menu.exec(ds_tree_->viewport()->mapToGlobal(pos));
}

QString DeleteDataDialog::deleteDescription() const
{
    set<string> sel_dbcontents;
    vector<DeleteDataDialog::SelectedDS> sel_ds;
    collectSelections(sel_dbcontents, sel_ds);

    if (sel_dbcontents.empty() && sel_ds.empty())
        return {};

    QStringList lines;

    if (!sel_dbcontents.empty())
    {
        QStringList names;
        for (const auto& name : sel_dbcontents)
            names.append(QString::fromStdString(name));

        lines.append("All data for DBContent: " + names.join(", "));
    }

    if (!sel_ds.empty())
    {
        lines.append("All data from data sources:");
        for (const auto& sel : sel_ds)
        {
            QString ds_desc = "  - " + QString::fromStdString(sel.name);
            if (!sel.all_lines)
            {
                QStringList line_strs;
                for (unsigned int lid : sel.line_ids)
                    line_strs.append(QString::fromStdString(
                        Utils::String::lineStrFrom(lid)));
                ds_desc += " (" + line_strs.join(", ") + " only)";
            }
            lines.append(ds_desc);
        }
    }

    return lines.join("\n");
}

void DeleteDataDialog::preselectDSTypes(const std::set<std::string>& names)
{
    if (!ds_tree_) return;
    for (int t = 0; t < ds_tree_->topLevelItemCount(); ++t)
    {
        QTreeWidgetItem* dstype_item = ds_tree_->topLevelItem(t);
        if (names.count(dstype_item->text(0).toStdString()))
            setCheckRecursive(dstype_item, Qt::Checked);
    }
}

void DeleteDataDialog::preselectDataSources(const std::set<unsigned int>& ds_ids)
{
    if (!ds_tree_) return;
    for (int t = 0; t < ds_tree_->topLevelItemCount(); ++t)
    {
        QTreeWidgetItem* dstype_item = ds_tree_->topLevelItem(t);
        bool any_ds_checked = false;
        for (int d = 0; d < dstype_item->childCount(); ++d)
        {
            QTreeWidgetItem* ds_item = dstype_item->child(d);
            unsigned int ds_id = ds_item->data(0, Qt::UserRole).toUInt();
            if (ds_ids.count(ds_id))
            {
                setCheckRecursive(ds_item, Qt::Checked);
                any_ds_checked = true;
            }
        }
        if (any_ds_checked)
            dstype_item->setCheckState(0, Qt::Checked);
    }
}

void DeleteDataDialog::preselectDBContents(const std::set<std::string>& names)
{
    for (const auto& name : names)
    {
        auto it = dbcontent_checks_.find(name);
        if (it != dbcontent_checks_.end())
            it->second->setChecked(true);
    }
}

void DeleteDataDialog::setRestrictedDBContents(const std::set<std::string>& names)
{
    restricted_dbcontents_ = names;
}

