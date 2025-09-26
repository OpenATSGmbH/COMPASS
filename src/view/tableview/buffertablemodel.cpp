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
#include "compass.h"
#include "buffer.h"
#include "buffercsvexportjob.h"
#include "buffertablewidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableset.h"
#include "global.h"
#include "jobmanager.h"
#include "tableview.h"
#include "tableviewdatasource.h"
#include "tableviewdatawidget.h"

#include <QApplication>

BufferTableModel::BufferTableModel(BufferTableWidget* table_widget,
                                   DBContent& object,
                                   TableView& view, TableViewDataSource& data_source)
    : QAbstractTableModel(table_widget),
      table_widget_(table_widget),
      object_(object), view_(view),
      data_source_(data_source)
{
    read_set_ = data_source_.getSet()->getFor(object_.name());

    connect(&data_source_, &TableViewDataSource::setChangedSignal, this, &BufferTableModel::setChangedSlot);
}

BufferTableModel::~BufferTableModel() 
{ 
    buffer_ = nullptr; 
}

void BufferTableModel::setChangedSlot()
{
    logdbg;

    beginResetModel();
    read_set_ = data_source_.getSet()->getFor(object_.name());

    logdbg << "read set size " << read_set_.getSize();

    // read_set_.print();

    endResetModel();
    traced_assert(table_widget_);
    table_widget_->resizeColumns();
}

int BufferTableModel::rowCount(const QModelIndex& /*parent*/) const
{
    logdbg << "start" << row_indexes_.size();
    return row_indexes_.size();
}

int BufferTableModel::columnCount(const QModelIndex& /*parent*/) const
{
    logdbg << "start" << read_set_.getSize();

    // selected
    return read_set_.getSize() + 1;
}

QVariant BufferTableModel::headerData(int section, 
                                      Qt::Orientation orientation, 
                                      int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal)
    {
        logdbg << "section " << section;
        unsigned int col = section;

        if (col == 0)
            return QString();

        col -= 1;  // for the actual properties

        traced_assert(col < read_set_.getSize());
        dbContent::Variable& variable = read_set_.getVariable(col);
        logdbg << "col " << col << " variable " << variable.name();
        return QString(variable.name().c_str());
    }
    else if (orientation == Qt::Vertical)
        return section;

    return QVariant();
}

Qt::ItemFlags BufferTableModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags flags;

    if (index.column() == 0)
    {
        flags |= Qt::ItemIsEnabled;
        flags |= Qt::ItemIsUserCheckable;
        flags |= Qt::ItemIsEditable;
        // flags |= Qt::ItemIsSelectable;
    }
    else
        return Qt::ItemIsEnabled | Qt::ItemIsSelectable;

    return flags;
}

QVariant BufferTableModel::data(const QModelIndex& index, int role) const
{
    logdbg << "row " << index.row() - 1 << " col " << index.column() - 1;

    bool null = false;

    traced_assert(index.row() >= 0);
    traced_assert((unsigned int)index.row() < row_indexes_.size());
    unsigned int buffer_index = row_indexes_.at(index.row());
    unsigned int col = index.column();

    if (role == Qt::CheckStateRole)
    {
        if (col == 0)  // selected special case
        {
            traced_assert(buffer_->has<bool>(DBContent::selected_var.name()));

            if (buffer_->get<bool>(DBContent::selected_var.name()).isNull(buffer_index))
                return Qt::Unchecked;

            if (buffer_->get<bool>(DBContent::selected_var.name()).get(buffer_index))
                return Qt::Checked;
            else
                return Qt::Unchecked;
        }
    }
    else if (role == Qt::DisplayRole)
    {
        traced_assert(buffer_);

        std::string value_str;

        const PropertyList& properties = buffer_->properties();

        traced_assert(buffer_index < buffer_->size());

        if (col == 0)  // selected special case
            return QVariant();

        col -= 1;  // for the actual properties

        traced_assert(col < read_set_.getSize());

        dbContent::Variable& variable = read_set_.getVariable(col);
        PropertyDataType data_type = variable.dataType();

        value_str = NULL_STRING;

        if (!properties.hasProperty(variable.name()))
        {
            logdbg << "variable " << variable.name()
                   << " not present in buffer";
        }
        else
        {
            std::string property_name = variable.name();

            if (data_type == PropertyDataType::BOOL)
            {
                traced_assert(buffer_->has<bool>(property_name));
                null = buffer_->get<bool>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<bool>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer_->get<bool>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::CHAR)
            {
                traced_assert(buffer_->has<char>(property_name));
                null = buffer_->get<char>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<char>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer_->get<char>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::UCHAR)
            {
                traced_assert(buffer_->has<unsigned char>(property_name));
                null = buffer_->get<unsigned char>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<unsigned char>(property_name).getAsString(buffer_index));
                    else
                        value_str =
                            buffer_->get<unsigned char>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::INT)
            {
                traced_assert(buffer_->has<int>(property_name));
                null = buffer_->get<int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<int>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer_->get<int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::UINT)
            {
                traced_assert(buffer_->has<unsigned int>(property_name));
                null = buffer_->get<unsigned int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<unsigned int>(property_name).getAsString(buffer_index));
                    else
                        value_str =
                            buffer_->get<unsigned int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::LONGINT)
            {
                traced_assert(buffer_->has<long int>(property_name));
                null = buffer_->get<long int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<long int>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer_->get<long int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::ULONGINT)
            {
                traced_assert(buffer_->has<unsigned long int>(property_name));
                null = buffer_->get<unsigned long int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<unsigned long int>(property_name)
                                .getAsString(buffer_index));
                    else
                        value_str = buffer_->get<unsigned long int>(property_name)
                                        .getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::FLOAT)
            {
                traced_assert(buffer_->has<float>(property_name));
                null = buffer_->get<float>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<float>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer_->get<float>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::DOUBLE)
            {
                traced_assert(buffer_->has<double>(property_name));
                null = buffer_->get<double>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer_->get<double>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer_->get<double>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::STRING)
            {
                traced_assert(buffer_->has<std::string>(property_name));
                null = buffer_->get<std::string>(property_name).isNull(buffer_index);
                if (!null)
                {
                    value_str = buffer_->get<std::string>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::JSON)
            {
                traced_assert(buffer_->has<nlohmann::json>(property_name));
                null = buffer_->get<nlohmann::json>(property_name).isNull(buffer_index);
                if (!null)
                {
                    value_str = buffer_->get<nlohmann::json>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::TIMESTAMP)
            {
                traced_assert(buffer_->has<boost::posix_time::ptime>(property_name));
                null = buffer_->get<boost::posix_time::ptime>(property_name).isNull(buffer_index);

                if (!null)
                {
                    value_str = buffer_->get<boost::posix_time::ptime>(property_name).getAsString(buffer_index);
                }
            }
            else
                throw std::domain_error("BufferTableWidget: show: unknown property data type");

            if (null)
                return QVariant();
            else
                return QString(value_str.c_str());
        }
    }
    return QVariant();
}

bool BufferTableModel::setData(const QModelIndex& index, 
                               const QVariant& value, 
                               int role)
{
    logdbg << "checked row " << index.row() << " col " << index.column();

    if (role == Qt::CheckStateRole && index.column() == 0)
    {
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        traced_assert(index.row() >= 0);
        traced_assert((unsigned int)index.row() < row_indexes_.size());
        unsigned int buffer_index = row_indexes_.at(index.row());

        traced_assert(buffer_);
        traced_assert(buffer_->has<bool>(DBContent::selected_var.name()));

        if (value == Qt::Checked)
        {
            loginf << "checked row index" << buffer_index;
            buffer_->get<bool>(DBContent::selected_var.name()).set(buffer_index, true);
        }
        else
        {
            loginf << "unchecked row index " << buffer_index;
            buffer_->get<bool>(DBContent::selected_var.name()).set(buffer_index, false);
        }
        traced_assert(table_widget_);
        table_widget_->view().emitSelectionChange();

        if (view_.settings().show_only_selected_)
        {
            beginResetModel();
            row_indexes_.clear();
            updateRows();
            endResetModel();
        }

        QApplication::restoreOverrideCursor();
    }
    return true;
}

void BufferTableModel::clearData()
{
    beginResetModel();

    buffer_ = nullptr;
    updateRows();

    endResetModel();
}

void BufferTableModel::setData(std::shared_ptr<Buffer> buffer)
{
    logdbg;
    traced_assert(buffer);
    beginResetModel();

    buffer_ = buffer;
    updateRows();

    endResetModel();
}

bool BufferTableModel::hasData() const
{
    return row_indexes_.size() != 0;
}

void BufferTableModel::updateRows()
{
    row_indexes_.clear();

    if (!buffer_)
    {
        // if (table_widget_)
        //     table_widget_->setHidden(true);

        return;
    }

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (view_.settings().ignore_non_target_reports_
        && !dbcont_man.metaCanGetVariable(buffer_->dbContentName(), DBContent::meta_var_latitude_))
    {
        return;
    }

    if (table_widget_)
        table_widget_->view().getDataWidget()->showTab(table_widget_, true);

    unsigned int buffer_index{0};  // index in buffer
    unsigned int buffer_size = buffer_->size();

    traced_assert(buffer_->has<bool>(DBContent::selected_var.name()));
    NullableVector<bool>& selected_vec = buffer_->get<bool>(DBContent::selected_var.name());

    while (buffer_index < buffer_size)
    {
        if (view_.settings().show_only_selected_)
        {
            if (selected_vec.isNull(buffer_index))  // check if null, skip if so
            {
                ++buffer_index;
                continue;
            }

            if (selected_vec.get(buffer_index))  // add if set
                row_indexes_.push_back(buffer_index);
        }
        else  // add
            row_indexes_.push_back(buffer_index);

        ++buffer_index;
    }
}

void BufferTableModel::reset()
{
    beginResetModel();
    endResetModel();
}

void BufferTableModel::saveAsCSV(const std::string& file_name)
{
    loginf << "into filename " << file_name;

    traced_assert(buffer_);
    BufferCSVExportJob* export_job = new BufferCSVExportJob(buffer_, read_set_, file_name, true, view_.settings().show_only_selected_, view_.settings().use_presentation_);

    export_job_ = std::shared_ptr<BufferCSVExportJob>(export_job);
    connect(export_job, &BufferCSVExportJob::obsoleteSignal, this,
            &BufferTableModel::exportJobObsoleteSlot, Qt::QueuedConnection);
    connect(export_job, &BufferCSVExportJob::doneSignal, this, &BufferTableModel::exportJobDoneSlot,
            Qt::QueuedConnection);

    JobManager::instance().addBlockingJob(export_job_);
}

void BufferTableModel::exportJobObsoleteSlot()
{
    logdbg;

    emit exportDoneSignal(true);
}

void BufferTableModel::exportJobDoneSlot()
{
    logdbg;

    emit exportDoneSignal(false);
}

void BufferTableModel::rebuild()
{
    beginResetModel();

    row_indexes_.clear();
    updateRows();

    endResetModel();
}
