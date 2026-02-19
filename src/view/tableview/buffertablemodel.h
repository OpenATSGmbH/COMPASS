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
#include "dbcontent/variable/variableset.h"

#include <memory>

class DBContent;
class BufferCSVExportJob;
class BufferTableWidget;

class BufferTableModel : public BaseBufferTableModel
{
    Q_OBJECT

  public:
    BufferTableModel(BufferTableWidget* table_widget, DBContent& object,
                     TableView& view, TableViewDataSource& data_source);
    virtual ~BufferTableModel();

    using BaseBufferTableModel::setData;

    void clearData() override;
    void setData(std::shared_ptr<Buffer> buffer);
    bool hasData() const;

    void saveAsCSV(const std::string& file_name) override;
    void rebuild() override;

  public slots:
    void setChangedSlot() override;

  protected:
    unsigned int dataRowCount() const override;
    RowData resolveRow(int row) const override;
    unsigned int prefixColumnCount() const override;
    unsigned int dataColumnCount() const override;
    QVariant prefixColumnData(unsigned int col, const RowData& row_data) const override;
    QVariant prefixColumnHeader(unsigned int col) const override;
    bool resolveVariable(unsigned int data_col, const std::string& dbcontent_name,
                         dbContent::Variable*& out_var) const override;
    QVariant dataColumnHeader(unsigned int data_col) const override;

  private:
    DBContent& object_;
    std::shared_ptr<Buffer> buffer_;
    dbContent::VariableSet read_set_;
    std::shared_ptr<BufferCSVExportJob> export_job_;
    std::vector<unsigned int> row_indexes_;

    void updateRows();
};
