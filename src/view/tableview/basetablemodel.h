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

#include <QAbstractTableModel>

#include <memory>
#include <string>
#include <vector>

class COMPASS;
class TableView;
class Buffer;
class TableViewDataSource;
class BaseBufferTableWidget;

namespace dbContent {
class Variable;
}

class BaseBufferTableModel : public QAbstractTableModel
{
    Q_OBJECT

  signals:
    void exportDoneSignal(bool canceled);

  public slots:
    virtual void setChangedSlot();
    void exportJobObsoleteSlot();
    void exportJobDoneSlot();

  public:
    BaseBufferTableModel(TableView& view, BaseBufferTableWidget* table_widget,
                         TableViewDataSource& data_source);
    virtual ~BaseBufferTableModel();

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
    void sort(int column, Qt::SortOrder order = Qt::AscendingOrder) override;

    virtual void clearData() = 0;
    virtual void saveAsCSV(const std::string& file_name) = 0;

    void reset();
    virtual void rebuild() = 0;

    static bool getSpecialRepresentation(std::string& repr,
                                         dbContent::Variable& var,
                                         Buffer& buffer,
                                         unsigned int buffer_idx,
                                         COMPASS& compass);

  protected:
    struct RowData
    {
        Buffer* buffer;
        unsigned int buffer_index;
        std::string dbcontent_name;
        unsigned int row{0};       // display-row index (== caller's index.row())
    };

    /// Returns the number of data rows (after filtering)
    virtual unsigned int dataRowCount() const = 0;

    /// Resolves a display row index to a buffer pointer, buffer index, and dbcontent name
    virtual RowData resolveRow(int row) const = 0;

    /// Number of prefix columns before data columns (1 for single-buffer, 2 for all-buffer)
    virtual unsigned int prefixColumnCount() const = 0;

    /// Number of data (variable) columns
    virtual unsigned int dataColumnCount() const = 0;

    /// Returns display data for a prefix column (DisplayRole only)
    virtual QVariant prefixColumnData(unsigned int col, const RowData& row_data) const = 0;

    /// Optional DecorationRole source for a prefix column. Default: no icon.
    /// Called by data() when role == Qt::DecorationRole and col is inside
    /// prefixColumnCount().
    virtual QVariant prefixColumnDecoration(unsigned int /*col*/,
                                            const RowData& /*row_data*/) const { return {}; }

    /// Returns header text for a prefix column
    virtual QVariant prefixColumnHeader(unsigned int col) const = 0;

    /// Resolves a data column index + dbcontent name to a Variable. Returns false if
    /// the variable is not applicable for this dbcontent (e.g. meta variable not in this type).
    virtual bool resolveVariable(unsigned int data_col, const std::string& dbcontent_name,
                                 dbContent::Variable*& out_var) const = 0;

    /// Returns header text for a data column
    virtual QVariant dataColumnHeader(unsigned int data_col) const = 0;

    /// Reorders the subclass's row_indexes_ according to a permutation vector.
    /// perm[i] = j means new row i takes the value from old row j.
    virtual void applyRowPermutation(const std::vector<unsigned int>& perm) = 0;

    /// Sorts row_indexes_ in-place by the stored sort column/order.
    /// Call after building row_indexes_ (in rebuild/setData) to re-apply the active sort.
    /// No-op if no sort is active (sort_column_ < 0).
    virtual void sortRowIndexes();

    /// Wrappers around beginResetModel/endResetModel that track reset state.
    /// Use these instead of calling beginResetModel/endResetModel directly,
    /// so that sort() can detect when it is called as a side effect of endResetModel().
    void beginCustomResetModel();
    void endCustomResetModel();

    TableView& view_;
    BaseBufferTableWidget* table_widget_{nullptr};
    TableViewDataSource& data_source_;

    int sort_column_{-1};
    Qt::SortOrder sort_order_{Qt::AscendingOrder};
    bool resetting_model_{false};
};
