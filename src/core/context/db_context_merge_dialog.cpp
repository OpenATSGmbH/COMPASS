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
#include "db_context_field_merge_widget.h"
#include "datasourceeditwidget.h"
#include "fft_edit_widget.h"
#include "sector_edit_widget.h"
#include "data_source.h"
#include "fft.h"
#include "asterix_decoding_config.h"
#include "sector.h"
#include "files.h"
#include "logger.h"
#include "number.h"

#include <json.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <sstream>

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
                                           const std::set<unsigned int>& ds_ids_with_data,
                                           QWidget* parent)
    : DBContextMergeDialog(config_context, db_context, diff, ds_ids_with_data,
                           /*cancellable*/ false, QString(),
                           QString("Configuration"), QString("Database"),
                           /*default_to_db*/ false, parent)
{
}

DBContextMergeDialog::DBContextMergeDialog(const DBContext& config_context,
                                           const DBContext& db_context,
                                           const DBContextDiff& diff,
                                           const std::set<unsigned int>& ds_ids_with_data,
                                           bool cancellable,
                                           const QString& cancel_tooltip,
                                           const QString& config_label,
                                           const QString& db_label,
                                           bool default_to_db,
                                           QWidget* parent)
    : QDialog(parent)
    , config_ctx_(config_context)
    , db_ctx_(db_context)
    , ds_ids_with_data_(ds_ids_with_data)
    , cancellable_(cancellable)
    , config_label_(config_label)
    , db_label_(db_label)
    , default_to_db_(default_to_db)
{
    build(diff, cancellable, cancel_tooltip);
}

void DBContextMergeDialog::build(const DBContextDiff& diff,
                                 bool cancellable,
                                 const QString& cancel_tooltip)
{
    setWindowTitle("Merge Context");
    if (!cancellable)
        setWindowFlags(windowFlags() & ~Qt::WindowCloseButtonHint);
    setMinimumSize(1300, 700);
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
    tree_widget_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    tree_widget_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    tree_widget_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    tree_widget_->header()->setStretchLastSection(false);
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
    ds_widget_ = new DataSourceEditWidget({}, {});
    ds_widget_->setReadOnly(true);
    detail_stack_->addWidget(ds_widget_);

    fft_widget_ = new FFTEditWidget({});
    fft_widget_->setReadOnly(true);
    detail_stack_->addWidget(fft_widget_);

    sector_widget_ = new SectorEditWidget({}, {});
    sector_widget_->setReadOnly(true);
    detail_stack_->addWidget(sector_widget_);

    // field merge widget for Modified items
    field_merge_widget_ = new FieldMergeWidget();
    detail_stack_->addWidget(field_merge_widget_);

    auto setComboForCurrent = [this](int combo_idx)
    {
        if (current_field_merge_idx_ < 0)
            return;
        auto& mi = merge_items_[current_field_merge_idx_];
        auto* combo = dynamic_cast<QComboBox*>(
            tree_widget_->itemWidget(mi.tree_item, 2));
        if (combo)
        {
            combo->blockSignals(true);
            combo->setCurrentIndex(combo_idx);
            combo->blockSignals(false);
        }
        if (combo_idx == 1)
            mi.choice = UseConfiguration;
        else if (combo_idx == 2)
            mi.choice = UseDatabase;
        else if (combo_idx == 3)
            mi.choice = ManuallyEdited;

        mi.tree_item->setIcon(0, QIcon());
        choiceChangedSlot();
    };

    connect(field_merge_widget_, &FieldMergeWidget::configChosenSignal,
            this, [setComboForCurrent]() { setComboForCurrent(1); });
    connect(field_merge_widget_, &FieldMergeWidget::dbChosenSignal,
            this, [setComboForCurrent]() { setComboForCurrent(2); });
    connect(field_merge_widget_, &FieldMergeWidget::valueEditedSignal,
            this, [setComboForCurrent]() { setComboForCurrent(3); });

    // placeholder for items without a detail widget
    auto* placeholder = new QLabel("Select an item to view its details.");
    placeholder->setAlignment(Qt::AlignCenter);
    detail_stack_->addWidget(placeholder);
    detail_stack_->setCurrentWidget(placeholder);

    splitter->addWidget(scroll);
    splitter->setStretchFactor(0, 0); // left side doesn't stretch
    splitter->setStretchFactor(1, 1); // right side gets extra space
    splitter->setSizes({450, 850});

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

    if (cancellable)
    {
        auto* cancel_button = new QPushButton("Cancel");
        cancel_button->setIcon(QIcon());
        if (!cancel_tooltip.isEmpty())
            cancel_button->setToolTip(cancel_tooltip);
        connect(cancel_button, &QPushButton::clicked, this, &QDialog::reject);
        button_layout->addWidget(cancel_button);
    }

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
        QString status;
        switch (diff.type)
        {
        case ItemDiff::Added:
            status = "Only in " + db_label_;
            break;
        case ItemDiff::Removed:
            status = "Only in " + config_label_;
            break;
        case ItemDiff::Modified:
            status = "Modified";
            item->setIcon(0, hint_icon);
            break;
        }
        item->setText(1, status);

        // column 2: combo box for choice
        auto* combo = new QComboBox();
        combo->addItem("");           // index 0 = undecided
        combo->addItem(config_label_); // index 1
        combo->addItem(db_label_);     // index 2
        combo->addItem("Edit");       // index 3 = manually edited

        // set defaults: Added -> Database, Removed -> Configuration, Modified -> undecided
        switch (diff.type)
        {
        case ItemDiff::Added:
        {
            combo->setCurrentIndex(2); // Database
            mi.choice = UseDatabase;

            // sensors with data in the DB cannot be removed
            if (section_key == "sensors" && !diff.item_b.is_null()
                && diff.item_b.contains("sac") && diff.item_b.contains("sic"))
            {
                unsigned int ds_id = Utils::Number::dsIdFrom(
                    diff.item_b.at("sac").get<unsigned int>(),
                    diff.item_b.at("sic").get<unsigned int>());

                if (ds_ids_with_data_.count(ds_id))
                {
                    combo->setEnabled(false);
                    combo->setToolTip("This sensor has data in the database and cannot be removed");
                }
            }
            break;
        }
        case ItemDiff::Removed:
            combo->setCurrentIndex(1); // Configuration
            mi.choice = UseConfiguration;
            break;
        case ItemDiff::Modified:
            if (default_to_db_)
            {
                combo->setCurrentIndex(2); // Database (the "new" side)
                mi.choice = UseDatabase;
                item->setIcon(0, QIcon()); // no hint - pre-decided
            }
            else
            {
                combo->setCurrentIndex(0); // undecided
                mi.choice = Undecided;
            }
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
            else if (combo_idx == 3)
                merge_items_[idx].choice = ManuallyEdited;
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
    loginf << "item clicked";

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
    const std::string& name = mi.diff->display_key.empty() ? mi.diff->key : mi.diff->display_key;
    loginf << "section '" << mi.section << "' item '" << name
           << "' type " << mi.diff->type;

    // save any in-progress field edits before switching
    if (current_field_merge_idx_ >= 0)
    {
        loginf << "saving field overrides for idx " << current_field_merge_idx_;
        field_overrides_[current_field_merge_idx_] = field_merge_widget_->mergedJSON();
        current_field_merge_idx_ = -1;
    }

    // find this item's index
    int mi_idx = -1;
    for (size_t i = 0; i < merge_items_.size(); ++i)
    {
        if (merge_items_[i].tree_item == mi.tree_item)
        {
            mi_idx = static_cast<int>(i);
            break;
        }
    }

    // pick which JSON to display:
    // Added = item_b (Database), Removed = item_a (Configuration)
    // Modified = field merge widget
    const nlohmann::json* item_json = nullptr;
    QString source_label;

    switch (mi.diff->type)
    {
    case ItemDiff::Added:
        item_json = &mi.diff->item_b;
        source_label = db_label_;
        break;
    case ItemDiff::Removed:
        item_json = &mi.diff->item_a;
        source_label = config_label_;
        break;
    case ItemDiff::Modified:
    {
        // determine which side is newer (or forced by caller)
        bool prefer_db = default_to_db_
                         || db_ctx_.modified() > config_ctx_.modified();

        const std::string& display_name = mi.diff->display_key.empty() ? mi.diff->key : mi.diff->display_key;
        field_merge_widget_->show(display_name, mi.diff->item_a, mi.diff->item_b,
                                  prefer_db, config_label_, db_label_);

        current_field_merge_idx_ = mi_idx;

        // auto-resolve as decided once the user inspects the fields
        if (mi_idx >= 0 && merge_items_[mi_idx].choice == Undecided)
        {
            merge_items_[mi_idx].choice = prefer_db ? UseDatabase : UseConfiguration;
            merge_items_[mi_idx].tree_item->setIcon(0, QIcon()); // remove hint

            // update combo to reflect
            auto* combo = dynamic_cast<QComboBox*>(
                tree_widget_->itemWidget(merge_items_[mi_idx].tree_item, 2));
            if (combo)
            {
                combo->blockSignals(true);
                combo->setCurrentIndex(prefer_db ? 2 : 1);
                combo->blockSignals(false);
            }

            choiceChangedSlot();
        }

        showDetailWidget(field_merge_widget_);
        return;
    }
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
    // block close when not cancellable (e.g. DB-open conflict flow)
    if (!cancellable_)
        return;

    QDialog::reject();
}

void DBContextMergeDialog::choiceChangedSlot()
{
    ok_button_->setEnabled(allDecided());
}

void DBContextMergeDialog::acceptSlot()
{
    // save any in-progress field edits
    if (current_field_merge_idx_ >= 0)
    {
        field_overrides_[current_field_merge_idx_] = field_merge_widget_->mergedJSON();
        current_field_merge_idx_ = -1;
    }

    buildMergedContext();
    accept();
}

// ============================================================
// Merge logic
// ============================================================

nlohmann::json DBContextMergeDialog::buildMergedItemJSON(const MergeItem& mi) const
{
    // find this item's index
    int mi_idx = -1;
    for (size_t i = 0; i < merge_items_.size(); ++i)
    {
        if (&merge_items_[i] == &mi)
        {
            mi_idx = static_cast<int>(i);
            break;
        }
    }

    // start from whichever side is newer as base (or forced by caller)
    bool prefer_db = default_to_db_
                     || db_ctx_.modified() > config_ctx_.modified();
    nlohmann::json result = prefer_db ? mi.diff->item_b : mi.diff->item_a;

    // apply field overrides if user edited this item
    if (mi_idx >= 0 && field_overrides_.count(mi_idx))
    {
        const auto& overrides = field_overrides_.at(mi_idx);

        for (auto it = overrides.begin(); it != overrides.end(); ++it)
        {
            const std::string& path = it.key();
            const auto& val = it.value();

            // walk the dotted path to find the target location
            // e.g. "info.network_lines.L2.mcast_ip"
            std::vector<std::string> parts;
            std::istringstream ss(path);
            std::string part;
            while (std::getline(ss, part, '.'))
                parts.push_back(part);

            if (parts.empty())
                continue;

            if (parts.size() == 1)
            {
                if (val.is_null())
                    result.erase(parts[0]);
                else
                    result[parts[0]] = val;
            }
            else
            {
                // navigate to parent, creating objects as needed
                nlohmann::json* node = &result;
                for (size_t i = 0; i < parts.size() - 1; ++i)
                {
                    if (!node->contains(parts[i]) || !(*node)[parts[i]].is_object())
                        (*node)[parts[i]] = nlohmann::json::object();
                    node = &(*node)[parts[i]];
                }

                const std::string& leaf = parts.back();
                if (val.is_null())
                    node->erase(leaf);
                else
                    (*node)[leaf] = val;
            }
        }
    }

    return result;
}

void DBContextMergeDialog::buildMergedContext()
{
    // start from configuration as base
    merged_ = config_ctx_;

    for (const auto& mi : merge_items_)
    {
        // for Modified items, always build from field overrides
        if (mi.diff->type == ItemDiff::Modified)
        {
            nlohmann::json merged_j = buildMergedItemJSON(mi);

            if (mi.section == "sensors")
            {
                merged_.addOrReplaceDataSource(DataSource::fromJSON(merged_j));
            }
            else if (mi.section == "ffts")
            {
                auto fft = FFT::fromJSON(merged_j);
                std::string name = fft.name();
                auto& vec = merged_.ffts();
                auto it = std::find_if(vec.begin(), vec.end(),
                    [&name](const FFT& f) { return f.name() == name; });
                if (it != vec.end())
                    *it = std::move(fft);
                else
                    vec.push_back(std::move(fft));
            }
            else if (mi.section == "asterix")
            {
                auto cfg = ASTERIXDecodingConfig::fromJSON(merged_j);
                unsigned int cat = cfg.category();
                auto& vec = merged_.asterixDecoding();
                auto it = std::find_if(vec.begin(), vec.end(),
                    [cat](const ASTERIXDecodingConfig& c) { return c.category() == cat; });
                if (it != vec.end())
                    *it = std::move(cfg);
                else
                    vec.push_back(std::move(cfg));
            }
            else if (mi.section == "sectors")
            {
                unsigned int sec_id = static_cast<unsigned int>(std::stoul(mi.diff->key));
                auto sector = std::make_shared<Sector>(
                    sec_id, merged_j.at("name"), merged_j.at("layer_name"), false);
                sector->readJSON(merged_j);
                auto& vec = merged_.sectors();
                auto it = std::find_if(vec.begin(), vec.end(),
                    [sec_id](const std::shared_ptr<Sector>& s) { return s->id() == sec_id; });
                if (it != vec.end())
                    *it = sector;
                else
                    vec.push_back(sector);
            }
            continue;
        }

        // for Added/Removed: only act if user chose UseDatabase
        if (mi.choice != UseDatabase)
            continue;

        if (mi.diff->type == ItemDiff::Added)
        {
            // exists only in DB - add it
            if (mi.section == "sensors")
                merged_.addOrReplaceDataSource(DataSource::fromJSON(mi.diff->item_b));
            else if (mi.section == "ffts")
                merged_.ffts().push_back(FFT::fromJSON(mi.diff->item_b));
            else if (mi.section == "asterix")
                merged_.asterixDecoding().push_back(ASTERIXDecodingConfig::fromJSON(mi.diff->item_b));
            else if (mi.section == "sectors")
            {
                unsigned int sec_id = static_cast<unsigned int>(std::stoul(mi.diff->key));
                auto sector = std::make_shared<Sector>(
                    sec_id, mi.diff->item_b.at("name"), mi.diff->item_b.at("layer_name"), false);
                sector->readJSON(mi.diff->item_b);
                merged_.sectors().push_back(sector);
            }
        }
        else if (mi.diff->type == ItemDiff::Removed)
        {
            // only in config, user wants DB (doesn't have it) - remove
            if (mi.section == "sensors")
            {
                unsigned int target_ds_id = 0;
                for (const auto& [ds_id, ds] : merged_.dataSources())
                {
                    std::string key = std::to_string(ds.sac()) + "/" + std::to_string(ds.sic());
                    if (key == mi.diff->key)
                    { target_ds_id = ds_id; break; }
                }
                if (target_ds_id > 0)
                    merged_.dataSources().erase(target_ds_id);
            }
            else if (mi.section == "ffts")
            {
                auto& vec = merged_.ffts();
                vec.erase(std::remove_if(vec.begin(), vec.end(),
                    [&mi](const FFT& f) { return f.name() == mi.diff->key; }), vec.end());
            }
            else if (mi.section == "asterix")
            {
                unsigned int cat = static_cast<unsigned int>(std::stoul(mi.diff->key));
                auto& vec = merged_.asterixDecoding();
                vec.erase(std::remove_if(vec.begin(), vec.end(),
                    [cat](const ASTERIXDecodingConfig& c) { return c.category() == cat; }), vec.end());
            }
            else if (mi.section == "sectors")
            {
                unsigned int sec_id = static_cast<unsigned int>(std::stoul(mi.diff->key));
                auto& vec = merged_.sectors();
                vec.erase(std::remove_if(vec.begin(), vec.end(),
                    [sec_id](const std::shared_ptr<Sector>& s) { return s->id() == sec_id; }), vec.end());
            }
        }
    }

    loginf << "merged context built: "
           << merged_.dataSources().size() << " sensors, "
           << merged_.ffts().size() << " FFTs, "
           << merged_.asterixDecoding().size() << " ASTERIX configs, "
           << merged_.sectors().size() << " sectors";
}

} // namespace context
