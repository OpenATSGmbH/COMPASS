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

#include "asteriximportdatasourceswidget.h"

#include "asteriximporttask.h"
#include "asteriximportsource.h"
#include "compass.h"
#include "data_source.h"
#include "datasourceeditwidget.h"
#include "db_context.h"
#include "db_context_manager.h"
#include "files.h"
#include "logger.h"
#include "stringconv.h"

#include <jasterix/category.h>
#include <jasterix/iteminfo.h>
#include <jasterix/jasterix.h>

#include <QFontMetrics>
#include <QHBoxLayout>
#include <QTextDocument>
#include <QHeaderView>
#include <QLabel>
#include <QScrollArea>
#include <QSplitter>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace
{

// Tree item user-role keys
constexpr int kRoleKind     = Qt::UserRole + 1;
constexpr int kRoleDsId     = Qt::UserRole + 2;
constexpr int kRoleCategory = Qt::UserRole + 3;

enum class ItemKind : int
{
    DSTypeGroup = 0,
    DataSource,
    Category,
    SpecialGroup,            // "Probe only", "Context only", "Unknown SAC/SIC"
};

QString sacSicString(unsigned int sac, unsigned int sic)
{
    return QString("(%1/%2)").arg(sac).arg(sic);
}

constexpr int kSortRole = Qt::UserRole + 100;

/// Table item that sorts by a numeric value stored in kSortRole, while showing
/// any free-form text via setText() (e.g. "1234 (99.7 %)").
class NumericSortItem : public QTableWidgetItem
{
public:
    using QTableWidgetItem::QTableWidgetItem;

    bool operator<(const QTableWidgetItem& other) const override
    {
        const QVariant a = data(kSortRole);
        const QVariant b = other.data(kSortRole);
        if (a.isValid() && b.isValid())
            return a.toDouble() < b.toDouble();
        if (a.isValid()) return true;   // unsorted (no data) comes last
        if (b.isValid()) return false;
        return QTableWidgetItem::operator<(other);
    }
};

/// Build a warning tooltip for a context data source. Returns an empty string
/// when no warning applies.
QString contextDsWarning(const context::DataSource& ds)
{
    QStringList msgs;
    if (ds.dsType() == "Radar" && !ds.hasPosition())
        msgs << QObject::tr("Radar position is not configured.");
    if (ds.dsType() == "Other")
        msgs << QObject::tr("Data source has DS type \"Other\" — assign a real type "
                            "(Radar, MLAT, ADSB, Tracker, RefTraj).");
    return msgs.join('\n');
}

} // anonymous namespace


ASTERIXImportDataSourcesWidget::ASTERIXImportDataSourcesWidget(ASTERIXImportTask& task,
                                                               QWidget* parent)
    : QWidget(parent)
    , task_(task)
{
    buildUI();

    connect(&task_, &ASTERIXImportTask::decodingStateChanged,
            this, &ASTERIXImportDataSourcesWidget::rebuildAll);
    connect(&task_, &ASTERIXImportTask::sourceUsageChanged,
            this, &ASTERIXImportDataSourcesWidget::rebuildAll);
    connect(&task_.compass().dbContextManager(),
            &context::DBContextManager::activeContextChangedSignal,
            this, &ASTERIXImportDataSourcesWidget::rebuildAll);

    rebuildAll();
}

ASTERIXImportDataSourcesWidget::~ASTERIXImportDataSourcesWidget() = default;

void ASTERIXImportDataSourcesWidget::buildUI()
{
    auto* layout = new QHBoxLayout();
    layout->setContentsMargins(0, 0, 0, 0);

    splitter_ = new QSplitter(Qt::Horizontal);

    // left: tree
    tree_widget_ = new QTreeWidget();
    tree_widget_->setHeaderLabels({"Source", "SAC/SIC", "Records"});
    tree_widget_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_widget_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    tree_widget_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    connect(tree_widget_, &QTreeWidget::itemSelectionChanged,
            this, &ASTERIXImportDataSourcesWidget::onTreeSelectionChanged);
    splitter_->addWidget(tree_widget_);

    // right: stacked detail inside scroll area
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);

    detail_stack_ = new QStackedWidget();
    scroll->setWidget(detail_stack_);

    // page 0: placeholder
    placeholder_ = new QLabel("Select a data source or category to see details.");
    placeholder_->setAlignment(Qt::AlignCenter);
    placeholder_->setWordWrap(true);
    detail_stack_->addWidget(placeholder_);

    // page 1: data source edit widget (reused from the context edit dialog)
    ds_edit_widget_ = new DataSourceEditWidget(
        /*show_network_lines=*/false,
        [this](unsigned int /*ds_id*/) {
            // user edited the data source — persist and refresh icons in tree
            auto& ctx_man = task_.compass().dbContextManager();
            if (ctx_man.hasActiveContext())
                ctx_man.saveContext(ctx_man.activeContextName());
            rebuildTree();
        },
        [](unsigned int /*ds_id*/) {
            // deletion intentionally not allowed from the import dialog
        },
        &task_.compass().dbContextManager());
    detail_stack_->addWidget(ds_edit_widget_);

    // page 2: items table
    items_page_ = new QWidget();
    auto* items_layout = new QVBoxLayout(items_page_);
    items_layout->setContentsMargins(4, 4, 4, 4);

    items_header_ = new QLabel();
    items_header_->setWordWrap(true);
    items_layout->addWidget(items_header_);

    items_table_ = new QTableWidget();
    items_table_->setColumnCount(4);
    items_table_->setHorizontalHeaderLabels({"Item", "Count", "Min", "Max"});

    // Item column auto-expands; the three numeric columns are smaller and
    // remain manually adjustable.
    auto* h_header = items_table_->horizontalHeader();
    h_header->setSectionResizeMode(0, QHeaderView::Stretch);
    h_header->setSectionResizeMode(1, QHeaderView::Interactive);
    h_header->setSectionResizeMode(2, QHeaderView::Interactive);
    h_header->setSectionResizeMode(3, QHeaderView::Interactive);
    h_header->setStretchLastSection(false);
    items_table_->setColumnWidth(1, 130);   // Count (with %)
    items_table_->setColumnWidth(2, 90);    // Min
    items_table_->setColumnWidth(3, 90);    // Max

    items_table_->verticalHeader()->setVisible(false);
    items_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    items_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    items_table_->setSortingEnabled(true);
    items_table_->setTextElideMode(Qt::ElideRight);

    items_layout->addWidget(items_table_);

    detail_stack_->addWidget(items_page_);

    detail_stack_->setCurrentWidget(placeholder_);

    splitter_->addWidget(scroll);
    splitter_->setSizes({350, 650});

    layout->addWidget(splitter_);
    setLayout(layout);
}

void ASTERIXImportDataSourcesWidget::rebuildAll()
{
    auto& ctx_man = task_.compass().dbContextManager();
    last_result_ = ASTERIXImportProbeAggregator::aggregate(task_.source(), ctx_man);
    rebuildTree();
}

void ASTERIXImportDataSourcesWidget::rebuildTree()
{
    auto& ctx_man = task_.compass().dbContextManager();

    tree_widget_->clear();

    auto report_warnings = [this](bool any) {
        if (any != has_warnings_)
        {
            has_warnings_ = any;
            emit warningsChanged(any);
        }
    };

    // empty/error states
    if (!ctx_man.hasActiveContext())
    {
        placeholder_->setText("No active context — open or create a context to see data sources.");
        showDetailWidget(placeholder_);
        report_warnings(false);
        return;
    }

    if (task_.source().isNetworkType())
    {
        placeholder_->setText("Data Sources tab applies to file imports only.");
        showDetailWidget(placeholder_);
        report_warnings(false);
        return;
    }

    if (!last_result_.probe_available)
    {
        placeholder_->setText("Run the decoding check to populate data source information.");
        showDetailWidget(placeholder_);
        report_warnings(false);
        return;
    }

    placeholder_->setText("Select a data source or category to see details.");

    const QIcon hint_icon = Utils::Files::IconProvider::getIcon("hint.png");
    bool any_warning = false;

    // group context+probe data sources by ds_type for the tree.
    // probe-only sources are placed in a special "Detected (not in context)" group.

    // ds_type -> ordered list of (ds_id, ds_pointer-or-null)
    std::map<std::string, std::vector<unsigned int>> by_type;
    for (auto id : last_result_.both_ds_ids)
    {
        auto* ds = ctx_man.dataSource(id);
        if (!ds)
            continue;
        by_type[ds->dsType()].push_back(id);
    }
    for (auto id : last_result_.context_only_ds_ids)
    {
        auto* ds = ctx_man.dataSource(id);
        if (!ds)
            continue;
        by_type[ds->dsType()].push_back(id);
    }

    // build standard DSType groups in canonical order
    for (const auto& ds_type : context::DataSource::dsTypeStrings())
    {
        if (!by_type.count(ds_type))
            continue;

        auto* type_item = new QTreeWidgetItem();
        type_item->setText(0, QString::fromStdString(ds_type));
        type_item->setData(0, kRoleKind, static_cast<int>(ItemKind::DSTypeGroup));
        type_item->setFirstColumnSpanned(false);
        tree_widget_->addTopLevelItem(type_item);

        bool any_warning_in_type = false;

        for (auto id : by_type.at(ds_type))
        {
            auto* ds = ctx_man.dataSource(id);
            if (!ds)
                continue;

            auto* ds_item = new QTreeWidgetItem(type_item);

            QString name = ds->name().empty()
                               ? QString("DS %1").arg(id)
                               : QString::fromStdString(ds->name());
            ds_item->setText(0, name);
            ds_item->setText(1, sacSicString(ds->sac(), ds->sic()));
            ds_item->setData(0, kRoleKind,  static_cast<int>(ItemKind::DataSource));
            ds_item->setData(0, kRoleDsId,  id);

            QString warning = contextDsWarning(*ds);
            if (!warning.isEmpty())
            {
                ds_item->setIcon(0, hint_icon);
                ds_item->setToolTip(0, warning);
                any_warning_in_type = true;
                any_warning = true;
            }

            std::size_t total = 0;
            const auto probe_it = last_result_.probe_by_dsid.find(id);
            if (probe_it != last_result_.probe_by_dsid.end())
            {
                for (const auto& cat_kv : probe_it->second.categories)
                    total += cat_kv.second.total_count;
            }
            ds_item->setText(2, QString::number(total));

            // category children — only when there is probe data
            if (probe_it != last_result_.probe_by_dsid.end())
            {
                for (const auto& cat_kv : probe_it->second.categories)
                {
                    auto* cat_item = new QTreeWidgetItem(ds_item);
                    cat_item->setText(0, QString("CAT%1").arg(
                        QString::fromStdString(Utils::String::categoryString(cat_kv.first))));
                    cat_item->setText(2, QString::number(cat_kv.second.total_count));
                    cat_item->setData(0, kRoleKind,     static_cast<int>(ItemKind::Category));
                    cat_item->setData(0, kRoleDsId,     id);
                    cat_item->setData(0, kRoleCategory, cat_kv.first);
                }
            }
        }

        if (any_warning_in_type)
            type_item->setIcon(0, hint_icon);
    }

    // probe-only data sources
    if (!last_result_.probe_only_ds_ids.empty())
    {
        any_warning = true;

        auto* group_item = new QTreeWidgetItem();
        group_item->setText(0, "Detected (not in context)");
        group_item->setData(0, kRoleKind, static_cast<int>(ItemKind::SpecialGroup));
        group_item->setIcon(0, hint_icon);
        group_item->setToolTip(0, tr("These sensors were detected in the probed data "
                                     "but are not yet defined in the active context."));
        tree_widget_->addTopLevelItem(group_item);

        for (auto id : last_result_.probe_only_ds_ids)
        {
            const auto& probe = last_result_.probe_by_dsid.at(id);

            auto* ds_item = new QTreeWidgetItem(group_item);
            ds_item->setText(0, QString("Unknown sensor"));
            ds_item->setText(1, sacSicString(probe.sac, probe.sic));
            ds_item->setData(0, kRoleKind, static_cast<int>(ItemKind::DataSource));
            ds_item->setData(0, kRoleDsId, id);
            ds_item->setIcon(0, hint_icon);
            ds_item->setToolTip(0, tr("Detected in data but not present in the active context."));

            std::size_t total = 0;
            for (const auto& cat_kv : probe.categories)
                total += cat_kv.second.total_count;
            ds_item->setText(2, QString::number(total));

            for (const auto& cat_kv : probe.categories)
            {
                auto* cat_item = new QTreeWidgetItem(ds_item);
                cat_item->setText(0, QString("CAT%1").arg(
                    QString::fromStdString(Utils::String::categoryString(cat_kv.first))));
                cat_item->setText(2, QString::number(cat_kv.second.total_count));
                cat_item->setData(0, kRoleKind,     static_cast<int>(ItemKind::Category));
                cat_item->setData(0, kRoleDsId,     id);
                cat_item->setData(0, kRoleCategory, cat_kv.first);
            }
        }
    }

    // unknown SAC/SIC bucket (CAT001/CAT002 with missing 010)
    if (last_result_.unknown.unknown_sac_sic
        && !last_result_.unknown.categories.empty())
    {
        auto* group_item = new QTreeWidgetItem();
        group_item->setText(0, "Unknown SAC/SIC");
        group_item->setData(0, kRoleKind, static_cast<int>(ItemKind::SpecialGroup));
        tree_widget_->addTopLevelItem(group_item);

        std::size_t total = 0;
        for (const auto& cat_kv : last_result_.unknown.categories)
            total += cat_kv.second.total_count;
        group_item->setText(2, QString::number(total));

        for (const auto& cat_kv : last_result_.unknown.categories)
        {
            auto* cat_item = new QTreeWidgetItem(group_item);
            cat_item->setText(0, QString("CAT%1").arg(
                QString::fromStdString(Utils::String::categoryString(cat_kv.first))));
            cat_item->setText(2, QString::number(cat_kv.second.total_count));
            cat_item->setData(0, kRoleKind,     static_cast<int>(ItemKind::Category));
            cat_item->setData(0, kRoleDsId,     0u);
            cat_item->setData(0, kRoleCategory, cat_kv.first);
        }
    }

    tree_widget_->expandAll();

    report_warnings(any_warning);

    // restore selection if possible
    if (selected_ds_id_)
    {
        // depth-first walk to find a matching item
        std::function<QTreeWidgetItem*(QTreeWidgetItem*)> find_match;
        find_match = [&](QTreeWidgetItem* parent) -> QTreeWidgetItem* {
            int n = parent ? parent->childCount() : tree_widget_->topLevelItemCount();
            for (int i = 0; i < n; ++i)
            {
                auto* it = parent ? parent->child(i) : tree_widget_->topLevelItem(i);
                auto kind = static_cast<ItemKind>(it->data(0, kRoleKind).toInt());
                if (selected_category_)
                {
                    if (kind == ItemKind::Category
                        && it->data(0, kRoleDsId).toUInt() == *selected_ds_id_
                        && it->data(0, kRoleCategory).toUInt() == *selected_category_)
                        return it;
                }
                else
                {
                    if (kind == ItemKind::DataSource
                        && it->data(0, kRoleDsId).toUInt() == *selected_ds_id_)
                        return it;
                }

                if (auto* m = find_match(it))
                    return m;
            }
            return nullptr;
        };

        if (auto* match = find_match(nullptr))
        {
            tree_widget_->setCurrentItem(match);
            return;
        }
    }

    showDetailWidget(placeholder_);
}

void ASTERIXImportDataSourcesWidget::onTreeSelectionChanged()
{
    auto items = tree_widget_->selectedItems();
    if (items.isEmpty())
    {
        selected_ds_id_.reset();
        selected_category_.reset();
        showDetailWidget(placeholder_);
        return;
    }

    auto* it = items.first();
    auto kind = static_cast<ItemKind>(it->data(0, kRoleKind).toInt());

    if (kind == ItemKind::DataSource)
    {
        unsigned int ds_id = it->data(0, kRoleDsId).toUInt();
        selected_ds_id_    = ds_id;
        selected_category_.reset();

        auto& ctx_man = task_.compass().dbContextManager();
        if (auto* ds = ctx_man.dataSource(ds_id))
        {
            ds_edit_widget_->show(*ds, task_.compass().lastUsedPath());
            showDetailWidget(ds_edit_widget_);
        }
        else
        {
            // probe-only — no context entry yet, no edit widget for it (TODO: "Add to context")
            placeholder_->setText("This sensor was detected in the data but is not yet in the active context.");
            showDetailWidget(placeholder_);
        }
        return;
    }

    if (kind == ItemKind::Category)
    {
        unsigned int ds_id = it->data(0, kRoleDsId).toUInt();
        unsigned int cat   = it->data(0, kRoleCategory).toUInt();
        selected_ds_id_    = ds_id;
        selected_category_ = cat;

        populateItemsTable(ds_id, cat);
        showDetailWidget(items_page_);
        return;
    }

    // group / unrecognised
    selected_ds_id_.reset();
    selected_category_.reset();
    showDetailWidget(placeholder_);
}

void ASTERIXImportDataSourcesWidget::populateItemsTable(unsigned int ds_id,
                                                        unsigned int category)
{
    items_table_->setSortingEnabled(false);
    items_table_->setRowCount(0);

    // pick the right probe bucket — ds_id == 0 means the "unknown SAC/SIC" group
    const ASTERIXImportProbeAggregator::CategoryProbe* probe = nullptr;
    if (ds_id == 0)
    {
        auto it = last_result_.unknown.categories.find(category);
        if (it != last_result_.unknown.categories.end())
            probe = &it->second;
    }
    else
    {
        auto ds_it = last_result_.probe_by_dsid.find(ds_id);
        if (ds_it != last_result_.probe_by_dsid.end())
        {
            auto cat_it = ds_it->second.categories.find(category);
            if (cat_it != ds_it->second.categories.end())
                probe = &cat_it->second;
        }
    }

    // pull all items defined in the category from jasterix
    jASTERIX::CategoryItemInfo edition_items;
    auto jasterix = task_.jASTERIX();
    if (jasterix)
    {
        auto cat_def = jasterix->category(category);
        if (cat_def)
            edition_items = cat_def->itemInfo();
    }

    // header label
    {
        QString header = QString("CAT%1 — ").arg(
            QString::fromStdString(Utils::String::categoryString(category)));
        if (probe)
            header += QString("%1 records").arg(probe->total_count);
        else
            header += "no records";
        items_header_->setText(header);
    }

    // union of probed item keys + edition-defined item keys
    // (probed items already pre-filtered by the aggregator; filter edition
    // items the same way so jASTERIX bookkeeping / FX bits stay out of the table)
    std::set<std::string> all_keys;
    if (probe)
        for (const auto& kv : probe->items)
            all_keys.insert(kv.first);
    for (const auto& kv : edition_items)
        if (ASTERIXImportProbeAggregator::isDisplayableDataItem(kv.first))
            all_keys.insert(kv.first);

    items_table_->setRowCount(static_cast<int>(all_keys.size()));

    const std::size_t cat_total = probe ? probe->total_count : 0;

    int row = 0;
    for (const auto& key : all_keys)
    {
        QString description;
        auto info_it = edition_items.find(key);
        if (info_it != edition_items.end())
            description = QString::fromStdString(info_it->second.description_);

        const ASTERIXImportProbeAggregator::ItemStats* stats = nullptr;
        if (probe)
        {
            auto it = probe->items.find(key);
            if (it != probe->items.end())
                stats = &it->second;
        }

        auto* name_item = new QTableWidgetItem(QString::fromStdString(key));
        items_table_->setItem(row, 0, name_item);

        // Count and presence percentage share one cell — sort numerically by count
        auto* count_item = new NumericSortItem();
        if (stats)
        {
            QString text = QString::number(static_cast<qulonglong>(stats->count));
            if (cat_total > 0)
            {
                const double pct = 100.0 * static_cast<double>(stats->count)
                                  / static_cast<double>(cat_total);
                text += " (" + QString::number(pct, 'f', 1) + " %)";
            }
            count_item->setText(text);
            count_item->setData(kSortRole, static_cast<qulonglong>(stats->count));
        }
        else
        {
            count_item->setText("-");
            count_item->setData(kSortRole, 0);
        }
        count_item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        items_table_->setItem(row, 1, count_item);

        auto json_to_text = [](const nlohmann::json& v) -> QString {
            if (v.is_null())
                return "-";
            if (v.is_string())
                return QString::fromStdString(v.get<std::string>());
            if (v.is_number_integer())
                return QString::number(v.get<long long>());
            if (v.is_number_unsigned())
                return QString::number(v.get<unsigned long long>());
            if (v.is_number_float())
                return QString::number(v.get<double>(), 'g', 10);
            if (v.is_boolean())
                return v.get<bool>() ? "true" : "false";
            return QString::fromStdString(v.dump());
        };

        auto* min_item = new QTableWidgetItem(stats ? json_to_text(stats->min) : QString("-"));
        min_item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        items_table_->setItem(row, 2, min_item);

        auto* max_item = new QTableWidgetItem(stats ? json_to_text(stats->max) : QString("-"));
        max_item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        items_table_->setItem(row, 3, max_item);

        // description as tooltip on every cell of the row (force rich text so \n survives)
        if (!description.isEmpty())
        {
            const QString tt = Qt::convertFromPlainText(description);
            for (int c = 0; c < 4; ++c)
                items_table_->item(row, c)->setToolTip(tt);
        }

        // grey out items that exist in the edition but were not probed
        if (!stats)
        {
            const QColor faded = palette().color(QPalette::Disabled, QPalette::Text);
            for (int c = 0; c < 4; ++c)
                items_table_->item(row, c)->setForeground(faded);
        }

        ++row;
    }

    items_table_->setSortingEnabled(true);
    items_table_->sortItems(0, Qt::AscendingOrder);
}

void ASTERIXImportDataSourcesWidget::showDetailWidget(QWidget* widget)
{
    if (!widget)
        return;
    if (detail_stack_->indexOf(widget) < 0)
        detail_stack_->addWidget(widget);
    detail_stack_->setCurrentWidget(widget);
}
