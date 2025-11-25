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

#include "dbcontent/variable/variableset.h"

#include <QAbstractTableModel>

#include <memory>

class TableView;
class Buffer;
class DBContent;
class BufferCSVExportJob;
class TableViewDataSource;
class BufferTableWidget;

class BufferTableModel : public QAbstractTableModel
{
    Q_OBJECT

  signals:
    void exportDoneSignal(bool cancelled);

  public slots:
    void setChangedSlot();
    void exportJobObsoleteSlot();
    void exportJobDoneSlot();

  public:
    BufferTableModel(BufferTableWidget* table_widget, DBContent& object,
                     TableView& view, TableViewDataSource& data_source);
    virtual ~BufferTableModel();

    int rowCount(const QModelIndex& parent = QModelIndex()) const;
    int columnCount(const QModelIndex& parent = QModelIndex()) const;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const;
    virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role);
    virtual QVariant headerData(int section, Qt::Orientation orientation,
                                int role = Qt::DisplayRole) const;

    void clearData();
    void setData(std::shared_ptr<Buffer> buffer);
    bool hasData() const;

    void saveAsCSV(const std::string& file_name);

    void reset();
    void rebuild();

    static bool getSpecialRepresentation(std::string& repr,
                                         dbContent::Variable& var,
                                         Buffer& buffer,
                                         unsigned int buffer_idx);
  protected:
    BufferTableWidget* table_widget_{nullptr};
    DBContent& object_;
    TableView& view_;
    TableViewDataSource& data_source_;

    std::shared_ptr<Buffer> buffer_;
    dbContent::VariableSet read_set_;

    std::shared_ptr<BufferCSVExportJob> export_job_;

    std::vector<unsigned int> row_indexes_;

    void updateRows();
};

