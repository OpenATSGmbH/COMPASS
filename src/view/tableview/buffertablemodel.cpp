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

#include "buffertablemodel.h"
#include "buffertablewidget.h"
#include "buffercsvexportjob.h"
#include "buffer.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "global.h"
#include "jobmanager.h"
#include "tableview.h"
#include "tableviewdatasource.h"
#include "tableviewdatawidget.h"

namespace
{

/// Dispatches NullableVector<T>::sortIndices for the correct T based on PropertyDataType.
template <typename T>
void sortBufferIndices(Buffer& buffer, const std::string& var_name,
                       std::vector<unsigned int>& indices, bool ascending)
{
    if (buffer.has<T>(var_name))
        buffer.get<T>(var_name).sortIndices(indices, ascending);
}

void sortByVariable(Buffer& buffer, dbContent::Variable& var,
                    std::vector<unsigned int>& indices, bool ascending)
{
    const std::string& name = var.name();

    switch (var.dataType())
    {
    case PropertyDataType::BOOL:      sortBufferIndices<bool>(buffer, name, indices, ascending); break;
    case PropertyDataType::CHAR:      sortBufferIndices<char>(buffer, name, indices, ascending); break;
    case PropertyDataType::UCHAR:     sortBufferIndices<unsigned char>(buffer, name, indices, ascending); break;
    case PropertyDataType::INT:       sortBufferIndices<int>(buffer, name, indices, ascending); break;
    case PropertyDataType::UINT:      sortBufferIndices<unsigned int>(buffer, name, indices, ascending); break;
    case PropertyDataType::LONGINT:   sortBufferIndices<long int>(buffer, name, indices, ascending); break;
    case PropertyDataType::ULONGINT:  sortBufferIndices<unsigned long int>(buffer, name, indices, ascending); break;
    case PropertyDataType::FLOAT:     sortBufferIndices<float>(buffer, name, indices, ascending); break;
    case PropertyDataType::DOUBLE:    sortBufferIndices<double>(buffer, name, indices, ascending); break;
    case PropertyDataType::STRING:    sortBufferIndices<std::string>(buffer, name, indices, ascending); break;
    case PropertyDataType::JSON:      sortBufferIndices<nlohmann::json>(buffer, name, indices, ascending); break;
    case PropertyDataType::TIMESTAMP: sortBufferIndices<boost::posix_time::ptime>(buffer, name, indices, ascending); break;
    default: break;
    }
}

} // anonymous namespace

BufferTableModel::BufferTableModel(BufferTableWidget* table_widget,
                                   DBContent& object,
                                   TableView& view, TableViewDataSource& data_source)
    : BaseBufferTableModel(view, table_widget, data_source),
      object_(object)
{
    read_set_ = data_source_.getSet()->getFor(object_.name());
}

BufferTableModel::~BufferTableModel()
{
    buffer_ = nullptr;
}

void BufferTableModel::setChangedSlot()
{
    logdbg;

    beginCustomResetModel();
    read_set_ = data_source_.getSet()->getFor(object_.name());

    logdbg << "read set size " << read_set_.getSize();

    endCustomResetModel();
    traced_assert(table_widget_);
    table_widget_->resizeColumns();
}

unsigned int BufferTableModel::dataRowCount() const
{
    return row_indexes_.size();
}

BaseBufferTableModel::RowData BufferTableModel::resolveRow(int row) const
{
    traced_assert(row >= 0);
    traced_assert((unsigned int)row < row_indexes_.size());

    RowData rd;
    rd.buffer = buffer_.get();
    rd.buffer_index = row_indexes_.at(row);
    rd.dbcontent_name = object_.name();
    return rd;
}

unsigned int BufferTableModel::prefixColumnCount() const
{
    return 1;  // checkbox column only
}

unsigned int BufferTableModel::dataColumnCount() const
{
    return read_set_.getSize();
}

QVariant BufferTableModel::prefixColumnData(unsigned int col, const RowData& /*row_data*/) const
{
    if (col == 0)  // checkbox column returns empty for DisplayRole
        return QVariant();

    return QVariant();
}

QVariant BufferTableModel::prefixColumnHeader(unsigned int col) const
{
    if (col == 0)
        return QString();

    return QVariant();
}

bool BufferTableModel::resolveVariable(unsigned int data_col,
                                       const std::string& /*dbcontent_name*/,
                                       dbContent::Variable*& out_var) const
{
    traced_assert(data_col < read_set_.getSize());
    out_var = &read_set_.getVariable(data_col);
    return true;
}

QVariant BufferTableModel::dataColumnHeader(unsigned int data_col) const
{
    traced_assert(data_col < read_set_.getSize());
    dbContent::Variable& variable = read_set_.getVariable(data_col);
    logdbg << "col " << data_col << " variable " << variable.name();
    return QString(variable.name().c_str());
}

void BufferTableModel::clearData()
{
    beginCustomResetModel();

    buffer_ = nullptr;
    updateRows();

    endCustomResetModel();
}

void BufferTableModel::setData(std::shared_ptr<Buffer> buffer)
{
    logdbg;
    traced_assert(buffer);
    beginCustomResetModel();

    buffer_ = buffer;
    updateRows();
    sortRowIndexes();

    endCustomResetModel();
}

bool BufferTableModel::hasData() const
{
    return row_indexes_.size() != 0;
}

void BufferTableModel::updateRows()
{
    row_indexes_.clear();

    if (!buffer_)
        return;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (view_.settings().ignore_non_target_reports_
        && !dbcont_man.metaCanGetVariable(buffer_->dbContentName(), DBContent::meta_var_latitude_))
    {
        return;
    }

    if (table_widget_)
    {
        BufferTableWidget* btw = dynamic_cast<BufferTableWidget*>(table_widget_);
        if (btw)
            view_.getDataWidget()->showTab(btw, true);
    }

    unsigned int buffer_index{0};
    unsigned int buffer_size = buffer_->size();

    traced_assert(buffer_->has<bool>(DBContent::selected_var.name()));
    NullableVector<bool>& selected_vec = buffer_->get<bool>(DBContent::selected_var.name());

    while (buffer_index < buffer_size)
    {
        if (view_.settings().show_only_selected_)
        {
            if (selected_vec.isNull(buffer_index))
            {
                ++buffer_index;
                continue;
            }

            if (selected_vec.get(buffer_index))
                row_indexes_.push_back(buffer_index);
        }
        else
            row_indexes_.push_back(buffer_index);

        ++buffer_index;
    }
}

void BufferTableModel::rebuild()
{
    beginCustomResetModel();

    row_indexes_.clear();
    updateRows();
    sortRowIndexes();

    endCustomResetModel();
}

void BufferTableModel::applyRowPermutation(const std::vector<unsigned int>& perm)
{
    std::vector<unsigned int> new_indexes(perm.size());
    for (unsigned int i = 0; i < perm.size(); ++i)
        new_indexes[i] = row_indexes_[perm[i]];
    row_indexes_ = std::move(new_indexes);
}

void BufferTableModel::sortRowIndexes()
{
    if (sort_column_ < 0 || row_indexes_.empty() || !buffer_)
        return;

    unsigned int col = static_cast<unsigned int>(sort_column_);
    bool ascending = (sort_order_ == Qt::AscendingOrder);

    if (col == 0)  // checkbox column
    {
        if (buffer_->has<bool>(DBContent::selected_var.name()))
            buffer_->get<bool>(DBContent::selected_var.name()).sortIndices(row_indexes_, ascending);
        return;
    }

    // col < prefixColumnCount() has no other prefix columns for BufferTableModel (count == 1)

    unsigned int data_col = col - prefixColumnCount();
    dbContent::Variable* var = nullptr;

    if (resolveVariable(data_col, object_.name(), var) && var &&
        buffer_->properties().hasProperty(var->name()))
    {
        sortByVariable(*buffer_, *var, row_indexes_, ascending);
    }
}

void BufferTableModel::saveAsCSV(const std::string& file_name)
{
    loginf << "into filename " << file_name;

    traced_assert(buffer_);
    BufferCSVExportJob* export_job = new BufferCSVExportJob(
        buffer_, read_set_, file_name, true,
        view_.settings().show_only_selected_, view_.settings().use_presentation_);

    export_job_ = std::shared_ptr<BufferCSVExportJob>(export_job);
    connect(export_job, &BufferCSVExportJob::obsoleteSignal, this,
            &BufferTableModel::exportJobObsoleteSlot, Qt::QueuedConnection);
    connect(export_job, &BufferCSVExportJob::doneSignal, this,
            &BufferTableModel::exportJobDoneSlot, Qt::QueuedConnection);

    JobManager::instance().addBlockingJob(export_job_);
}
