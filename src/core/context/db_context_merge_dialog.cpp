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

#include "db_context_merge_dialog.h"
#include "datasourceeditwidget.h"
#include "fft_edit_widget.h"
#include "sector_edit_widget.h"
#include "data_source.h"
#include "fft.h"
#include "asterix_decoding_config.h"
#include "sector.h"
#include "files.h"
#include "logger.h"

#include <json.hpp>

#include <algorithm>
#include <map>
#include <set>

#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace context
{

DBContextMergeDialog::DBContextMergeDialog(const DBContext& config_context,
                                           const DBContext& db_context,
                                           const DBContextDiff& diff,
                                           QWidget* parent)
    : QDialog(parent)
    , config_ctx_(config_context)
    , db_ctx_(db_context)
{
    setWindowTitle("Merge Context");
    setWindowFlags(windowFlags() & ~Qt::WindowCloseButtonHint);
    setMinimumSize(1100, 700);
    setModal(true);

    auto* main_layout = new QVBoxLayout(this);

    // description
    auto* desc_label = new QLabel(
        "Choose which version to use for each conflicting item.\n"
        "Click an item to inspect its details on the right.");
    desc_label->setWordWrap(true);
    main_layout->addWidget(desc_label);

    // splitter: tree (left) + detail (right)
    auto* splitter = new QSplitter(Qt::Horizontal);

    // --- left: tree widget ---
    tree_widget_ = new QTreeWidget();
    tree_widget_->setColumnCount(3);
    tree_widget_->setHeaderLabels({"Item", "Status", "Use"});
    tree_widget_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_widget_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    tree_widget_->header()->setSectionResizeMode(2, QHeaderView::Fixed);
    tree_widget_->header()->resizeSection(2, 120);
    tree_widget_->setRootIsDecorated(true);

    connect(tree_widget_, &QTreeWidget::itemClicked,
            this, &DBContextMergeDialog::itemClickedSlot);

    splitter->addWidget(tree_widget_);

    // --- right: detail panel ---
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);

    detail_stack_ = new QStackedWidget();
    scroll->setWidget(detail_stack_);

    // create read-only edit widgets
    ds_widget_ = new DataSourceEditWidget(false, {}, {});
    ds_widget_->setReadOnly(true);
    detail_stack_->addWidget(ds_widget_);

    fft_widget_ = new FFTEditWidget({});
    fft_widget_->setReadOnly(true);
    detail_stack_->addWidget(fft_widget_);

    sector_widget_ = new SectorEditWidget({}, {});
    sector_widget_->setReadOnly(true);
    detail_stack_->addWidget(sector_widget_);

    // placeholder for items without a detail widget
    auto* placeholder = new QLabel("Select an item to view its details.");
    placeholder->setAlignment(Qt::AlignCenter);
    detail_stack_->addWidget(placeholder);
    detail_stack_->setCurrentWidget(placeholder);

    splitter->addWidget(scroll);
    splitter->setSizes({550, 550});

    main_layout->addWidget(splitter, 1);

    // populate sections
    addSection("Sensors", "sensors", diff.sensor_diffs);
    addSection("FFTs", "ffts", diff.fft_diffs);
    addSection("ASTERIX Decoding", "asterix", diff.asterix_diffs);
    addSection("Sectors", "sectors", diff.sector_diffs);

    tree_widget_->expandAll();

    // buttons
    auto* button_layout = new QHBoxLayout();
    button_layout->addStretch();

    ok_button_ = new QPushButton("OK");
    ok_button_->setIcon(QIcon());
    ok_button_->setToolTip("Accept the merge and apply");
    ok_button_->setEnabled(allDecided());
    connect(ok_button_, &QPushButton::clicked, this, &DBContextMergeDialog::acceptSlot);
    button_layout->addWidget(ok_button_);

    main_layout->addLayout(button_layout);
}

void DBContextMergeDialog::addSection(const std::string& section_name,
                                      const std::string& section_key,
                                      const std::vector<ItemDiff>& diffs)
{
    if (diffs.empty())
        return;

    auto* section_item = new QTreeWidgetItem(tree_widget_);

    QFont font_bold = section_item->font(0);
    font_bold.setBold(true);

    section_item->setText(0, QString::fromStdString(section_name));
    section_item->setFont(0, font_bold);
    section_item->setFlags(section_item->flags() & ~Qt::ItemIsSelectable);

    QIcon hint_icon(Utils::Files::getIconFilepath("hint.png").c_str());

    for (const auto& diff : diffs)
    {
        MergeItem mi;
        mi.diff = &diff;
        mi.section = section_key;

        auto* item = new QTreeWidgetItem(section_item);
        mi.tree_item = item;

        // column 0: item name
        const std::string& name = diff.display_key.empty() ? diff.key : diff.display_key;
        item->setText(0, QString::fromStdString(name));

        // column 1: status
        std::string status;
        switch (diff.type)
        {
        case ItemDiff::Added:
            status = "Only in Database";
            break;
        case ItemDiff::Removed:
            status = "Only in Configuration";
            break;
        case ItemDiff::Modified:
            status = "Modified";
            item->setIcon(0, hint_icon);
            break;
        }
        item->setText(1, QString::fromStdString(status));

        // column 2: combo box for choice
        auto* combo = new QComboBox();
        combo->addItem(""); // index 0 = undecided
        combo->addItem("Configuration"); // index 1
        combo->addItem("Database");      // index 2

        // set defaults: Added -> Database, Removed -> Configuration, Modified -> undecided
        switch (diff.type)
        {
        case ItemDiff::Added:
            combo->setCurrentIndex(2); // Database
            mi.choice = UseDatabase;
            break;
        case ItemDiff::Removed:
            combo->setCurrentIndex(1); // Configuration
            mi.choice = UseConfiguration;
            break;
        case ItemDiff::Modified:
            combo->setCurrentIndex(0); // undecided
            mi.choice = Undecided;
            break;
        }

        size_t idx = merge_items_.size();
        merge_items_.push_back(mi);

        connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this, idx](int combo_idx)
        {
            if (combo_idx == 1)
                merge_items_[idx].choice = UseConfiguration;
            else if (combo_idx == 2)
                merge_items_[idx].choice = UseDatabase;
            else
                merge_items_[idx].choice = Undecided;

            // remove hint icon once decided
            if (merge_items_[idx].choice != Undecided)
                merge_items_[idx].tree_item->setIcon(0, QIcon());
            else
            {
                QIcon hint(Utils::Files::getIconFilepath("hint.png").c_str());
                merge_items_[idx].tree_item->setIcon(0, hint);
            }

            choiceChangedSlot();
        });

        tree_widget_->setItemWidget(item, 2, combo);
    }
}

// ============================================================
// Detail panel
// ============================================================

void DBContextMergeDialog::itemClickedSlot(QTreeWidgetItem* item, int /*column*/)
{
    // find the MergeItem for this tree item
    for (const auto& mi : merge_items_)
    {
        if (mi.tree_item == item)
        {
            showDetail(mi);
            return;
        }
    }
}

void DBContextMergeDialog::showDetail(const MergeItem& mi)
{
    // pick which JSON to display:
    // Added = item_b (Database), Removed = item_a (Configuration)
    // Modified = stub for now
    const nlohmann::json* item_json = nullptr;
    QString source_label;

    switch (mi.diff->type)
    {
    case ItemDiff::Added:
        item_json = &mi.diff->item_b;
        source_label = "Database";
        break;
    case ItemDiff::Removed:
        item_json = &mi.diff->item_a;
        source_label = "Configuration";
        break;
    case ItemDiff::Modified:
        // TODO: show two widgets side-by-side
        showDetailWidget(createPlaceholderLabel(
            "Modified item — side-by-side comparison not yet implemented.\n\n"
            "Configuration and Database versions differ."));
        return;
    }

    if (!item_json || item_json->is_null())
    {
        showDetailWidget(createPlaceholderLabel("No data available."));
        return;
    }

    // show the appropriate widget
    if (mi.section == "sensors")
    {
        temp_ds_ = std::make_unique<DataSource>(DataSource::fromJSON(*item_json));
        ds_widget_->show(*temp_ds_);
        showDetailWidget(ds_widget_);
    }
    else if (mi.section == "ffts")
    {
        temp_fft_ = std::make_unique<FFT>(FFT::fromJSON(*item_json));
        fft_widget_->show(*temp_fft_);
        showDetailWidget(fft_widget_);
    }
    else if (mi.section == "sectors")
    {
        unsigned int sec_id = static_cast<unsigned int>(std::stoul(mi.diff->key));
        temp_sector_ = std::make_shared<Sector>(
            sec_id, item_json->at("name"), item_json->at("layer_name"), false);
        temp_sector_->readJSON(*item_json);
        sector_widget_->show(*temp_sector_);
        showDetailWidget(sector_widget_);
    }
    else if (mi.section == "asterix")
    {
        // TODO: dedicated ASTERIX detail
        auto cfg = ASTERIXDecodingConfig::fromJSON(*item_json);
        QString text = source_label + "\n\n"
            "Category: " + QString::number(cfg.category()) + "\n"
            "Edition: " + QString::fromStdString(cfg.edition()) + "\n"
            "REF: " + QString::fromStdString(cfg.ref()) + "\n"
            "SPF: " + QString::fromStdString(cfg.spf());
        showDetailWidget(createPlaceholderLabel(text));
    }
    else
    {
        showDetailWidget(createPlaceholderLabel("Unknown section."));
    }
}

void DBContextMergeDialog::showDetailWidget(QWidget* widget)
{
    if (!widget)
        return;

    if (detail_stack_->indexOf(widget) < 0)
        detail_stack_->addWidget(widget);

    detail_stack_->setCurrentWidget(widget);
}

QWidget* DBContextMergeDialog::createPlaceholderLabel(const QString& text)
{
    auto* label = new QLabel(text);
    label->setAlignment(Qt::AlignCenter);
    label->setWordWrap(true);
    return label;
}

// ============================================================
// State
// ============================================================

bool DBContextMergeDialog::allDecided() const
{
    for (const auto& mi : merge_items_)
        if (mi.choice == Undecided)
            return false;
    return true;
}

void DBContextMergeDialog::reject()
{
    // block close until all items are resolved
}

void DBContextMergeDialog::choiceChangedSlot()
{
    ok_button_->setEnabled(allDecided());
}

void DBContextMergeDialog::acceptSlot()
{
    buildMergedContext();
    accept();
}

// ============================================================
// Merge logic
// ============================================================

void DBContextMergeDialog::buildMergedContext()
{
    // start from configuration as base
    merged_ = config_ctx_;

    // --- Sensors ---
    for (const auto& mi : merge_items_)
    {
        if (mi.section != "sensors" || mi.choice != UseDatabase)
            continue;

        if (mi.diff->type == ItemDiff::Added || mi.diff->type == ItemDiff::Modified)
        {
            auto ds = DataSource::fromJSON(mi.diff->item_b);
            merged_.addOrReplaceDataSource(std::move(ds));
        }
        else if (mi.diff->type == ItemDiff::Removed)
        {
            // only in config, user wants DB (doesn't have it) — remove
            unsigned int ds_id = 0;
            for (const auto& ds : merged_.dataSources())
            {
                std::string key = std::to_string(ds.sac()) + "/" + std::to_string(ds.sic());
                if (key == mi.diff->key)
                {
                    ds_id = ds.id();
                    break;
                }
            }
            if (ds_id > 0)
            {
                auto& vec = merged_.dataSources();
                vec.erase(std::remove_if(vec.begin(), vec.end(),
                    [ds_id](const DataSource& ds) { return ds.id() == ds_id; }), vec.end());
            }
        }
    }

    // --- FFTs ---
    for (const auto& mi : merge_items_)
    {
        if (mi.section != "ffts" || mi.choice != UseDatabase)
            continue;

        if (mi.diff->type == ItemDiff::Added || mi.diff->type == ItemDiff::Modified)
        {
            auto fft = FFT::fromJSON(mi.diff->item_b);
            std::string name = fft.name();

            auto& vec = merged_.ffts();
            auto it = std::find_if(vec.begin(), vec.end(),
                [&name](const FFT& f) { return f.name() == name; });

            if (it != vec.end())
                *it = std::move(fft);
            else
                vec.push_back(std::move(fft));
        }
        else if (mi.diff->type == ItemDiff::Removed)
        {
            auto& vec = merged_.ffts();
            vec.erase(std::remove_if(vec.begin(), vec.end(),
                [&mi](const FFT& f) { return f.name() == mi.diff->key; }), vec.end());
        }
    }

    // --- ASTERIX Decoding ---
    for (const auto& mi : merge_items_)
    {
        if (mi.section != "asterix" || mi.choice != UseDatabase)
            continue;

        if (mi.diff->type == ItemDiff::Added || mi.diff->type == ItemDiff::Modified)
        {
            auto cfg = ASTERIXDecodingConfig::fromJSON(mi.diff->item_b);
            unsigned int cat = cfg.category();

            auto& vec = merged_.asterixDecoding();
            auto it = std::find_if(vec.begin(), vec.end(),
                [cat](const ASTERIXDecodingConfig& c) { return c.category() == cat; });

            if (it != vec.end())
                *it = std::move(cfg);
            else
                vec.push_back(std::move(cfg));
        }
        else if (mi.diff->type == ItemDiff::Removed)
        {
            unsigned int cat = static_cast<unsigned int>(std::stoul(mi.diff->key));
            auto& vec = merged_.asterixDecoding();
            vec.erase(std::remove_if(vec.begin(), vec.end(),
                [cat](const ASTERIXDecodingConfig& c) { return c.category() == cat; }), vec.end());
        }
    }

    // --- Sectors ---
    for (const auto& mi : merge_items_)
    {
        if (mi.section != "sectors" || mi.choice != UseDatabase)
            continue;

        if (mi.diff->type == ItemDiff::Added || mi.diff->type == ItemDiff::Modified)
        {
            unsigned int sec_id = static_cast<unsigned int>(std::stoul(mi.diff->key));
            const auto& sec_j = mi.diff->item_b;

            auto sector = std::make_shared<Sector>(
                sec_id, sec_j.at("name"), sec_j.at("layer_name"), false);
            sector->readJSON(sec_j);

            auto& vec = merged_.sectors();
            auto it = std::find_if(vec.begin(), vec.end(),
                [sec_id](const std::shared_ptr<Sector>& s) { return s->id() == sec_id; });

            if (it != vec.end())
                *it = sector;
            else
                vec.push_back(sector);
        }
        else if (mi.diff->type == ItemDiff::Removed)
        {
            unsigned int sec_id = static_cast<unsigned int>(std::stoul(mi.diff->key));
            auto& vec = merged_.sectors();
            vec.erase(std::remove_if(vec.begin(), vec.end(),
                [sec_id](const std::shared_ptr<Sector>& s) { return s->id() == sec_id; }), vec.end());
        }
    }

    loginf << "merged context built: "
           << merged_.dataSources().size() << " sensors, "
           << merged_.ffts().size() << " FFTs, "
           << merged_.asterixDecoding().size() << " ASTERIX configs, "
           << merged_.sectors().size() << " sectors";
}

} // namespace context
