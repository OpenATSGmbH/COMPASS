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

#include "buffertablewidget.h"
#include "buffertablemodel.h"
#include "buffer.h"
#include "dbcontent/dbcontent.h"
#include "logger.h"

#include <QTableView>

BufferTableWidget::BufferTableWidget(DBContent& object, TableView& view,
                                     TableViewDataSource& data_source, QWidget* parent,
                                     Qt::WindowFlags f)
    : BaseBufferTableWidget(view, data_source, parent, f), object_(object)
{
    buffer_model_ = new BufferTableModel(this, object_, view_, data_source_);
    initModel(buffer_model_);
}

BufferTableWidget::~BufferTableWidget() = default;

void BufferTableWidget::show(std::shared_ptr<Buffer> buffer)
{
    traced_assert(buffer);

    logdbg << "object " << object_.name() << " buffer size "
           << buffer->size() << " properties " << buffer->properties().size();
    traced_assert(table_);
    traced_assert(buffer_model_);

    buffer_model_->setData(buffer);
    table_->resizeColumnsToContents();

    logdbg << "end";
}

bool BufferTableWidget::hasData() const
{
    traced_assert(buffer_model_);
    return buffer_model_->hasData();
}

