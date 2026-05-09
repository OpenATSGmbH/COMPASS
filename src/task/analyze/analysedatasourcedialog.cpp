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

#include "analysedatasourcedialog.h"
#include "analysedatasourcetask.h"
#include "datasourceinspectorbase.h"

#include "compass.h"
#include "db_context_manager.h"
#include "data_source.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QStackedWidget>
#include <QPushButton>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QHeaderView>

AnalyseDataSourceDialog::AnalyseDataSourceDialog(AnalyseDataSourceTask& task, QWidget* parent)
    : QDialog(parent), task_(task)
{
    setWindowTitle(QString::fromStdString("Analyse " + task_.dsType() + " Data Source"));
    setMinimumSize(900, 500);

    auto* main_layout = new QVBoxLayout(this);

    auto* splitter = new QSplitter(Qt::Horizontal, this);

    tree_ = new QTreeWidget(splitter);
    tree_->setHeaderLabels({"Inspector"});
    tree_->setColumnCount(1);
    tree_->setRootIsDecorated(true);
    tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    splitter->addWidget(tree_);

    stack_ = new QStackedWidget(splitter);
    splitter->addWidget(stack_);

    splitter->setStretchFactor(0, 2);
    splitter->setStretchFactor(1, 3);
    splitter->setSizes({360, 540});

    main_layout->addWidget(splitter, 1);

    auto* button_row = new QHBoxLayout();
    button_row->addStretch(1);

    auto* cancel = new QPushButton("Cancel", this);
    cancel->setIcon(QIcon());
    cancel->setToolTip("Close the dialog without running the analysis");
    connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
    button_row->addWidget(cancel);

    run_button_ = new QPushButton("Run", this);
    run_button_->setIcon(QIcon());
    run_button_->setToolTip("Run the analysis with the current selection");
    connect(run_button_, &QPushButton::clicked, this, &AnalyseDataSourceDialog::runSlot);
    button_row->addWidget(run_button_);

    main_layout->addLayout(button_row);

    buildTree();

    connect(tree_, &QTreeWidget::itemSelectionChanged,
            this, &AnalyseDataSourceDialog::treeSelectionChangedSlot);
    connect(tree_, &QTreeWidget::itemChanged,
            this, &AnalyseDataSourceDialog::treeItemChangedSlot);

    if (ds_item_)
        ds_item_->setSelected(true);

    updateRunEnabled();
}

void AnalyseDataSourceDialog::buildTree()
{
    updating_ui_ = true;

    ds_item_ = new QTreeWidgetItem(tree_);
    ds_item_->setText(0, "Data Source");
    ds_item_->setExpanded(true);
    stack_->insertWidget(0, buildDataSourceWidget());

    int stack_idx = 1;
    for (const auto& ins_uptr : task_.inspectors())
    {
        DataSourceInspectorBase* ins = ins_uptr.get();

        auto* item = new QTreeWidgetItem(ds_item_);
        std::string label = ins->name();
        if (ins->requiresProfessionalLicense())
            label += "  [pro]";
        item->setText(0, QString::fromStdString(label));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, task_.inspectorEnabled(ins->className())
                                   ? Qt::Checked : Qt::Unchecked);

        std::string reason;
        bool ok = ins->prerequisitesMet(reason);
        if (!ok)
        {
            item->setDisabled(true);
            item->setToolTip(0, QString::fromStdString(reason));
            item->setCheckState(0, Qt::Unchecked);
        }
        else if (!ins->description().empty())
        {
            item->setToolTip(0, QString::fromStdString(ins->description()));
        }

        stack_->insertWidget(stack_idx++, buildInspectorWidget(ins));
        inspector_rows_.push_back({item, ins});
    }

    tree_->setCurrentItem(ds_item_);

    updating_ui_ = false;
}

QWidget* AnalyseDataSourceDialog::buildDataSourceWidget()
{
    auto* w = new QWidget();
    auto* layout = new QVBoxLayout(w);

    auto* info = new QLabel(
        QString("Select the %1 data source(s) to fold into the combined dataset.")
            .arg(QString::fromStdString(task_.dsType())));
    info->setWordWrap(true);
    layout->addWidget(info);

    auto* list = new QListWidget();
    auto& ctx = task_.compass().dbContextManager();
    auto types = ctx.dsTypes();

    bool found_any = false;
    for (auto ds_id : ctx.allDataSourceIds())
    {
        auto it = types.find(ds_id);
        if (it == types.end() || it->second != task_.dsType())
            continue;

        const auto* ds = ctx.dataSource(ds_id);
        if (!ds)
            continue;

        std::string label = ds->name() + " (" + std::to_string(ds->sac()) + "/"
                            + std::to_string(ds->sic()) + ")";
        auto* item = new QListWidgetItem(QString::fromStdString(label), list);
        item->setData(Qt::UserRole, ds_id);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(task_.useDataSource(ds_id) ? Qt::Checked : Qt::Unchecked);
        found_any = true;
    }

    if (!found_any)
    {
        auto* placeholder = new QListWidgetItem(
            QString("No data sources of type ") + QString::fromStdString(task_.dsType())
            + " present.", list);
        placeholder->setFlags(Qt::NoItemFlags);
    }

    connect(list, &QListWidget::itemChanged, this, [this](QListWidgetItem* it) {
        if (updating_ui_)
            return;
        bool ok = false;
        unsigned int ds_id = it->data(Qt::UserRole).toUInt(&ok);
        if (!ok)
            return;
        task_.useDataSource(ds_id, it->checkState() == Qt::Checked);
        refreshInspectorRows();
        updateRunEnabled();
    });

    layout->addWidget(list, 1);

    return w;
}

QWidget* AnalyseDataSourceDialog::buildInspectorWidget(DataSourceInspectorBase* inspector)
{
    auto* w = new QWidget();
    auto* layout = new QVBoxLayout(w);

    auto* title = new QLabel(QString::fromStdString(inspector->name()));
    QFont f = title->font();
    f.setBold(true);
    f.setPointSize(f.pointSize() + 2);
    title->setFont(f);
    layout->addWidget(title);

    if (!inspector->description().empty())
    {
        auto* desc = new QLabel(QString::fromStdString(inspector->description()));
        desc->setWordWrap(true);
        layout->addWidget(desc);
    }

    auto* hint = new QLabel(
        "Settings for this inspector are configured via the JSON configuration file. "
        "A graphical settings widget will be added in a later iteration.");
    hint->setWordWrap(true);
    hint->setStyleSheet("color: gray;");
    layout->addWidget(hint);

    layout->addStretch(1);
    return w;
}

void AnalyseDataSourceDialog::refreshInspectorRows()
{
    updating_ui_ = true;
    for (auto& row : inspector_rows_)
    {
        std::string reason;
        bool ok = row.inspector->prerequisitesMet(reason);
        row.item->setDisabled(!ok);
        row.item->setToolTip(0, QString::fromStdString(ok ? row.inspector->description()
                                                          : reason));
        if (!ok)
            row.item->setCheckState(0, Qt::Unchecked);
    }
    updating_ui_ = false;
}

void AnalyseDataSourceDialog::treeSelectionChangedSlot()
{
    auto items = tree_->selectedItems();
    if (items.isEmpty())
        return;
    auto* sel = items.first();

    if (sel == ds_item_)
    {
        stack_->setCurrentIndex(0);
        return;
    }

    for (size_t i = 0; i < inspector_rows_.size(); ++i)
    {
        if (inspector_rows_[i].item == sel)
        {
            stack_->setCurrentIndex(static_cast<int>(i + 1));
            return;
        }
    }
}

void AnalyseDataSourceDialog::treeItemChangedSlot(QTreeWidgetItem* item, int column)
{
    if (updating_ui_ || column != 0)
        return;

    for (auto& row : inspector_rows_)
    {
        if (row.item != item)
            continue;
        bool checked = item->checkState(0) == Qt::Checked;
        task_.inspectorEnabled(row.inspector->className(), checked);
        updateRunEnabled();
        return;
    }
}

void AnalyseDataSourceDialog::updateRunEnabled()
{
    if (!run_button_)
        return;

    bool can_run = !task_.selectedDataSourceIDs().empty();
    if (can_run)
    {
        bool any_enabled = false;
        for (const auto& row : inspector_rows_)
        {
            if (row.item->isDisabled())
                continue;
            if (row.item->checkState(0) == Qt::Checked
                && task_.inspectorEnabled(row.inspector->className()))
            {
                any_enabled = true;
                break;
            }
        }
        can_run = any_enabled;
    }
    run_button_->setEnabled(can_run);
}

void AnalyseDataSourceDialog::runSlot()
{
    accept();
}
