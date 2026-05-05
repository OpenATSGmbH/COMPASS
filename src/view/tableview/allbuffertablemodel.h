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

#include "basetablemodel.h"

#include <QColor>
#include <QIcon>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

class AllBufferCSVExportJob;
class AllBufferTableWidget;

class AllBufferTableModel : public BaseBufferTableModel
{
    Q_OBJECT

  public slots:
    void setChangedSlot() override;

  public:
    AllBufferTableModel(TableView& view, AllBufferTableWidget* table_widget,
                        TableViewDataSource& data_source);
    virtual ~AllBufferTableModel();

    using BaseBufferTableModel::setData;

    void clearData() override;
    void setData(std::map<std::string, std::shared_ptr<Buffer>> buffers);

    void saveAsCSV(const std::string& file_name) override;
    void rebuild() override;

    std::pair<int,int> getSelectedRows(); // min, max selected row

    /// Installs a layer-level filter keyed by "<ds_type>:<ds_name>:L<n>:<dbcont>".
    /// nullopt = no filter (all rows shown). An empty set filters everything out.
    /// The filter is consulted inside buildRowIndexes(); callers must trigger a
    /// rebuild() (or setData()) after changing this to refresh the view.
    void setAllowedLayerIds(std::optional<std::set<std::string>> ids);

    /// Per-layer color map used to render the icon prefix column. Invalid
    /// QColor entries render as empty space (for non-target-report layers).
    /// Keys not present in the map render empty too.
    void setLayerColors(std::map<std::string, QColor> layer_colors);

  protected:
    unsigned int dataRowCount() const override;
    RowData resolveRow(int row) const override;
    unsigned int prefixColumnCount() const override;
    unsigned int dataColumnCount() const override;
    QVariant prefixColumnData(unsigned int col, const RowData& row_data) const override;
    QVariant prefixColumnDecoration(unsigned int col, const RowData& row_data) const override;
    QVariant prefixColumnHeader(unsigned int col) const override;
    bool resolveVariable(unsigned int data_col, const std::string& dbcontent_name,
                         dbContent::Variable*& out_var) const override;
    QVariant dataColumnHeader(unsigned int data_col) const override;
    void applyRowPermutation(const std::vector<unsigned int>& perm) override;
    void sortRowIndexes() override;

  private:
    std::map<std::string, std::shared_ptr<Buffer>> buffers_;

    std::shared_ptr<AllBufferCSVExportJob> export_job_;

    std::map<unsigned int, std::string> number_to_dbcont_;
    std::map<std::string, unsigned int> dbcont_to_number_;

    std::vector<std::pair<unsigned int, unsigned int>> row_indexes_;  // row index -> [dbcont num, buffer index]

    /// Per-row layer-id index, parallel to row_indexes_. Each entry indexes
    /// into layer_id_pool_. Computed once in buildRowIndexes() and reordered
    /// via applyRowPermutation() — avoids per-cell-paint DBContent / data
    /// source lookups in prefixColumnDecoration().
    std::vector<unsigned int> row_layer_index_;
    std::vector<std::string>  layer_id_pool_;   // 0 = "" (unknown)

    std::optional<std::set<std::string>> allowed_layer_ids_;
    std::map<std::string, QColor>        layer_colors_;

    /// Cache: (layer_id, is_selected) -> QIcon. Cleared whenever layer_colors_
    /// changes. "" key is used for rows without a known layer color.
    mutable std::map<std::pair<std::string, bool>, QIcon> icon_cache_;

    /// Cache: (data_col, dbcontent_name) -> Variable*. nullptr means "tried
    /// and not applicable" (a meta variable not present for this dbcontent,
    /// or the column belongs to a different dbcontent). Cleared in
    /// setChangedSlot() when the data source variable set changes.
    mutable std::map<std::pair<unsigned int, std::string>, dbContent::Variable*> variable_cache_;

    QIcon iconFor(const std::string& layer_id, bool selected) const;

    void buildRowIndexes();
};
