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

#include <memory>

class BufferTableModel;
class Buffer;
class DBContent;

class BufferTableWidget : public BaseBufferTableWidget
{
    Q_OBJECT

  public:
    BufferTableWidget(DBContent& object, TableView& view, TableViewDataSource& data_source,
                      QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~BufferTableWidget();

    void show(std::shared_ptr<Buffer> buffer);

    bool hasData() const;

  private:
    DBContent& object_;
    BufferTableModel* buffer_model_{nullptr};
};
