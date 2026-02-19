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

#include <map>
#include <memory>

class AllBufferCSVExportJob;
class AllBufferTableWidget;

class AllBufferTableModel : public BaseBufferTableModel
{
    Q_OBJECT

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
    void applyRowPermutation(const std::vector<unsigned int>& perm) override;

  private:
    std::map<std::string, std::shared_ptr<Buffer>> buffers_;

    std::shared_ptr<AllBufferCSVExportJob> export_job_;

    std::map<unsigned int, std::string> number_to_dbcont_;
    std::map<std::string, unsigned int> dbcont_to_number_;

    std::vector<std::pair<unsigned int, unsigned int>> row_indexes_;  // row index -> [dbcont num, buffer index]

    void buildRowIndexes();
};
