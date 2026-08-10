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

#include "allbuffertablewidget.h"
#include "allbuffertablemodel.h"
#include "buffer.h"
#include "logger.h"

#include <QMessageBox>
#include <QTableView>
#include <QTimer>
#include <QItemSelection>

AllBufferTableWidget::AllBufferTableWidget(TableView& view, TableViewDataSource& data_source,
                                           QWidget* parent, Qt::WindowFlags f)
    : BaseBufferTableWidget(view, data_source, parent, f)
{
    all_buffer_model_ = new AllBufferTableModel(view_, this, data_source_);
    initModel(all_buffer_model_);
}

AllBufferTableWidget::~AllBufferTableWidget() {}

int AllBufferTableWidget::rowCount() const
{
    return table_->model()->rowCount();
}

void AllBufferTableWidget::show(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    traced_assert(table_);
    traced_assert(all_buffer_model_);

    all_buffer_model_->setData(buffers);
    table_->resizeColumnsToContents();
}

void AllBufferTableWidget::selectSelectedRows()
{
    loginf;

    traced_assert(table_);
    traced_assert(all_buffer_model_);

    std::vector<std::pair<int,int>> ranges = all_buffer_model_->getSelectedRows();

    if (ranges.empty())
    {
        table_->selectionModel()->clearSelection();
        return;
    }

    // Highlight the whole span from the first to the last selected row (one contiguous
    // range - the individual selected rows are indicated by the selected_ checkbox
    // column). ranges is sorted, so front().first / back().second bound the span.
    const int first_row = ranges.front().first;
    const int last_row  = ranges.back().second;

    QModelIndex first = all_buffer_model_->index(first_row, 0, QModelIndex());
    QModelIndex last  = all_buffer_model_->index(last_row,  0, QModelIndex());

    {
        // Block the selection model's change signal during the apply. For a selection
        // spanning most of a multi-million-row table, QTableView::selectionChanged calls
        // QItemSelection::indexes() to update header-section highlighting, which
        // materializes every selected cell (rows x selectable columns) and calls flags()
        // on each - O(selected cells), ~70 s on an 8M-row table. The selection is stored
        // regardless, and cell painting queries it per visible cell, so the highlight
        // still renders; we repaint the viewport so it shows immediately. The signal only
        // fires on selection change, so no such sweep happens when the tab is shown.
        QSignalBlocker blocker(table_->selectionModel());
        table_->selectionModel()->select(QItemSelection(first, last),
                                         QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Rows);
    }

    table_->viewport()->update();

    // needed, maybe because model is reset
    QTimer::singleShot(10, [this,first]{table_->scrollTo(first, QAbstractItemView::PositionAtCenter);});
}

std::vector<std::vector<std::string>> AllBufferTableWidget::getSelectedText()
{
    std::vector<std::vector<std::string>> data;

    QAbstractItemModel* model = table_->model();
    QItemSelectionModel* selection = table_->selectionModel();
    QModelIndexList indexes = selection->selectedIndexes();

    if (!indexes.size())
        return data;

    QModelIndex previous = indexes.first();

    if (!previous.isValid()) // empty
        return data;

    unsigned int row_count = 0;

    std::vector<std::string> header_data;
    std::vector<std::string> current_row_data;

    header_data.push_back(model->headerData(previous.column(), Qt::Horizontal).toString().toStdString());
    current_row_data.push_back(model->data(previous).toString().toStdString());
    indexes.removeFirst();

    foreach (const QModelIndex& current, indexes)
    {
        if (current.row() != previous.row())
        {
            if (!row_count)  // first row
                data.push_back(header_data);

            data.push_back(current_row_data);
            current_row_data.clear();

            ++row_count;

            if (row_count == 999)
            {
                QMessageBox m_warning(
                            QMessageBox::Warning, "Too Many Rows Selected",
                            "If more than 1000 lines are selected, only the first 1000 are copied.",
                            QMessageBox::Ok);
                m_warning.exec();
                break;
            }
        }

        current_row_data.push_back(model->data(current).toString().toStdString());

        if (!row_count)  // first row
            header_data.push_back(model->headerData(current.column(), Qt::Horizontal).toString().toStdString());

        previous = current;
    }

    return data;
}

std::vector<std::vector<std::string>> AllBufferTableWidget::getText(unsigned int max_rows)
{
    std::vector<std::vector<std::string>> data;

    QAbstractItemModel* model = table_->model();

    QModelIndex previous_index = model->index(0, 0);

    if (!previous_index.isValid()) // empty
        return data;

    unsigned int row_count = 0;

    std::vector<std::string> header_data;
    std::vector<std::string> current_row_data;

    header_data.push_back(model->headerData(previous_index.column(), Qt::Horizontal).toString().toStdString());
    current_row_data.push_back(model->data(previous_index).toString().toStdString());

    QModelIndex current_index;

    int rows = model->rowCount();
    int cols = model->columnCount();

    traced_assert(rows >= 0);
    traced_assert(cols >= 0);

    bool max_rows_hit = false;

    for (unsigned int row_cnt = 0; row_cnt < (unsigned int) rows; ++row_cnt)
    {
        for (unsigned int col_cnt = 0; col_cnt < (unsigned int) cols; ++col_cnt)
        {
            if (row_cnt == 0 && col_cnt == 0) // first, skip for previous
                continue;

            if (row_cnt >= max_rows)
            {
                max_rows_hit = true;
                break;
            }

            current_index = model->index(row_cnt, col_cnt);

            if (!current_index.isValid())
                break;

            if (current_index.row() != previous_index.row())
            {
                if (!row_count)  // first row
                    data.push_back(header_data);

                data.push_back(current_row_data);
                current_row_data.clear();

                ++row_count;
            }

            current_row_data.push_back(model->data(current_index).toString().toStdString());

            if (!row_count)  // first row
                header_data.push_back(model->headerData(current_index.column(), Qt::Horizontal).toString().toStdString());

            previous_index = current_index;
        }
    }

    if (max_rows_hit)
    {
        current_row_data.clear();

        for (int col_cnt = 0; col_cnt < cols; ++col_cnt)
            current_row_data.push_back("...");

        data.push_back(current_row_data);
        current_row_data.clear();
    }

    return data;
}

