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

#include "basetablewidget.h"
#include "allbuffertablemodel.h"

#include <memory>
#include <map>
#include <vector>
#include <string>

class Buffer;

class AllBufferTableWidget : public BaseBufferTableWidget
{
    Q_OBJECT

  public:
    AllBufferTableWidget(TableView& view, TableViewDataSource& data_source, QWidget* parent = nullptr,
                         Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~AllBufferTableWidget();

    void show(std::map<std::string, std::shared_ptr<Buffer>> buffers);
    // commits row data prepared on a worker thread (see AllBufferTableModel::prepareData)
    void showPrepared(AllBufferTableModel::PreparedData&& prepared);

    void selectSelectedRows();
    // as above, with the selected row ranges already computed (from prepared data)
    void selectSelectedRows(const std::vector<std::pair<int,int>>& ranges);

    int rowCount() const;

    std::vector<std::vector<std::string>> getSelectedText(); // first is header
    std::vector<std::vector<std::string>> getText(unsigned int max_rows=30); // first is header

    AllBufferTableModel* allBufferTableModel() const { return all_buffer_model_; }

  private:
    AllBufferTableModel* all_buffer_model_{nullptr};
};
