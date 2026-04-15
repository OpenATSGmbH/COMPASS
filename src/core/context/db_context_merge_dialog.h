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

#pragma once

#include "db_context.h"
#include "db_context_diff.h"

#include <QDialog>

#include <memory>
#include <set>
#include <vector>

class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QStackedWidget;
class Sector;

class DataSourceEditWidget;

namespace context
{

class FFTEditWidget;
class SectorEditWidget;
class FieldMergeWidget;

/**
 * Dialog for interactively merging two versions of a context.
 *
 * Shows a tree with sections (Sensors, FFTs, ASTERIX Decoding, Sectors),
 * each containing the diff items. For each item the user picks
 * "Configuration" or "Database" via a combo box.
 *
 * Clicking an item shows its details in a read-only panel on the right.
 * Added/Removed items show one widget; Modified items show two side-by-side
 * (stub for now).
 *
 * OK is disabled until all modified items have a selection.
 */
class DBContextMergeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DBContextMergeDialog(const DBContext& config_context,
                                  const DBContext& db_context,
                                  const DBContextDiff& diff,
                                  const std::set<unsigned int>& ds_ids_with_data,
                                  QWidget* parent = nullptr);

    const DBContext& mergedContext() const { return merged_; }

protected:
    void reject() override;

private slots:
    void choiceChangedSlot();
    void acceptSlot();
    void itemClickedSlot(QTreeWidgetItem* item, int column);

private:
    enum Choice { Undecided, UseConfiguration, UseDatabase, ManuallyEdited };

    struct MergeItem
    {
        const ItemDiff* diff;
        std::string section;      // "sensors", "ffts", "asterix", "sectors"
        QTreeWidgetItem* tree_item{nullptr};
        Choice choice{Undecided};
    };

    void addSection(const std::string& section_name,
                    const std::string& section_key,
                    const std::vector<ItemDiff>& diffs);
    void buildMergedContext();
    nlohmann::json buildMergedItemJSON(const MergeItem& mi) const;
    bool allDecided() const;

    void showDetail(const MergeItem& mi);
    void showDetailWidget(QWidget* widget);
    QWidget* createPlaceholderLabel(const QString& text);

    const DBContext& config_ctx_;
    const DBContext& db_ctx_;
    std::set<unsigned int> ds_ids_with_data_;

    QTreeWidget* tree_widget_{nullptr};
    QStackedWidget* detail_stack_{nullptr};
    QPushButton* ok_button_{nullptr};

    // edit widgets (read-only, for detail panel)
    DataSourceEditWidget* ds_widget_{nullptr};
    FFTEditWidget* fft_widget_{nullptr};
    SectorEditWidget* sector_widget_{nullptr};

    // field merge widget for Modified items
    FieldMergeWidget* field_merge_widget_{nullptr};

    // temporary objects for display (kept alive while widget shows them)
    std::unique_ptr<DataSource> temp_ds_;
    std::unique_ptr<FFT> temp_fft_;
    std::shared_ptr<Sector> temp_sector_;

    std::vector<MergeItem> merge_items_;
    int current_field_merge_idx_{-1};  // index into merge_items_ currently shown in field merge widget
    std::map<int, nlohmann::json> field_overrides_;  // merge item idx -> edited field values

    DBContext merged_;
};

} // namespace context
