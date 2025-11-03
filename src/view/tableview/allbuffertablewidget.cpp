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
#include "tableviewdatasource.h"
#include "logger.h"
#include "compass.h"

#include <QApplication>
#include <QClipboard>
#include <QFileDialog>
#include <QKeyEvent>
#include <QMessageBox>
#include <QTableView>
#include <QVBoxLayout>
#include <QTimer>

#include "boost/date_time/posix_time/posix_time.hpp"


AllBufferTableWidget::AllBufferTableWidget(TableView& view, TableViewDataSource& data_source,
                                           QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f), view_(view), data_source_(data_source)
{
    //setAutoFillBackground(true);

    QVBoxLayout* layout = new QVBoxLayout();

    table_ = new QTableView(this);
    table_->setSelectionBehavior(QAbstractItemView::SelectItems);
    table_->setSelectionMode(QAbstractItemView::ContiguousSelection);
    model_ = new AllBufferTableModel(view_, this, data_source_);
    table_->setModel(model_);

    connect(model_, &AllBufferTableModel::exportDoneSignal,
            this, &AllBufferTableWidget::exportDoneSlot);

    layout->addWidget(table_);
    table_->show();

    setLayout(layout);
}

AllBufferTableWidget::~AllBufferTableWidget() {}

int AllBufferTableWidget::rowCount() const
{
    return table_->model()->rowCount();
}

void AllBufferTableWidget::clear()
{
    traced_assert(model_);

    model_->clearData();
}

void AllBufferTableWidget::show(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    traced_assert(table_);
    traced_assert(model_);

    model_->setData(buffers);
    table_->resizeColumnsToContents();
}

void AllBufferTableWidget::exportSlot()
{
    loginf;

    QFileDialog dialog(nullptr);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDirectory(COMPASS::instance().lastUsedPath().c_str());
    dialog.setNameFilter("CSV Files (*.csv)");
    dialog.setDefaultSuffix("csv");
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);

    QStringList file_names;
    if (dialog.exec())
        file_names = dialog.selectedFiles();

    QString filename;

    if (file_names.size() == 1)
        filename = file_names.at(0);

    if (filename.size())
    {
        if (!filename.endsWith(".csv"))  // in case of qt bug
            filename += ".csv";

        loginf << "export filename " << filename.toStdString();
        traced_assert(model_);
        model_->saveAsCSV(filename.toStdString());
    }
    else
    {
        emit exportDoneSignal(true);
    }
}

void AllBufferTableWidget::exportDoneSlot(bool cancelled) { emit exportDoneSignal(cancelled); }

void AllBufferTableWidget::updateToSettingsChange()
{
    logdbg;

    traced_assert(model_);
    model_->rebuild();
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

void AllBufferTableWidget::resetModel()
{
    traced_assert(model_);
    model_->reset();
}

void AllBufferTableWidget::updateToSelection()
{
    traced_assert(model_);
    model_->rebuild();
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

void AllBufferTableWidget::resizeColumns()
{
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

void AllBufferTableWidget::selectSelectedRows()
{
    loginf;

    traced_assert(table_);
    traced_assert(model_);
    std::pair<int,int> rows = model_->getSelectedRows();

    if (rows.first >= 0 && rows.second >= 0)
    {
        loginf << "rows " << rows.first << " to " << rows.second;

        traced_assert(rows.first <= rows.second);

        QModelIndex first = model_->index(rows.first, 0, QModelIndex());
        traced_assert(first.isValid());

        QModelIndex last = model_->index(rows.second, 0, QModelIndex());
        traced_assert(last.isValid());

        table_->selectionModel()->select(QItemSelection(first, last),
                                         QItemSelectionModel::Select | QItemSelectionModel::Rows);

        //table_->scrollTo(first, QAbstractItemView::PositionAtCenter);
        // needed, maybe because model is reset
        QTimer::singleShot(10, [this,first]{table_->scrollTo(first, QAbstractItemView::PositionAtCenter);});
    }
    else
        loginf << "nothing selected";
}

TableView& AllBufferTableWidget::view() const { return view_; }

void AllBufferTableWidget::keyPressEvent(QKeyEvent* event)
{
    loginf << "got keypressed";

    traced_assert(table_);

    if (event->modifiers() & Qt::ControlModifier)
    {
        if (event->key() == Qt::Key_C)
        {
            loginf << "copying";

            QAbstractItemModel* model = table_->model();
            QItemSelectionModel* selection = table_->selectionModel();
            QModelIndexList indexes = selection->selectedIndexes();

            QString selected_text;
            QString selected_headers;
            // You need a pair of indexes to find the row changes
            QModelIndex previous = indexes.first();
            unsigned int row_count = 0;

            selected_headers = model->headerData(previous.column(), Qt::Horizontal).toString();
            selected_text = model->data(previous).toString();
            indexes.removeFirst();

            foreach (const QModelIndex& current, indexes)
            {
                // If you are at the start of the row the row number of the previous index
                // isn't the same.  Text is followed by a row separator, which is a newline.
                if (current.row() != previous.row())
                {
                    selected_text.append('\n');

                    if (!row_count)  // first row
                        selected_headers.append('\n');

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
                // Otherwise it's the same row, so append a column separator, which is a tab.
                else
                {
                    if (!row_count)  // first row
                        selected_headers.append(';');

                    selected_text.append(';');
                }

                QVariant data = model->data(current);
                QString text = data.toString();
                // At this point `text` contains the text in one cell
                selected_text.append(text);

                //                loginf << "UGA row " << current.row() << " col " <<
                //                current.column() << " text '"
                //                       << text.toStdString() << "'";

                if (!row_count)  // first row
                    selected_headers.append(
                                model->headerData(current.column(), Qt::Horizontal).toString());

                previous = current;
            }

            QApplication::clipboard()->setText(selected_headers + selected_text);
        }
    }

    loginf << "done";
}

std::vector<std::vector<std::string>> AllBufferTableWidget::getSelectedText ()
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
        // If you are at the start of the row the row number of the previous index
        // isn't the same.  Text is followed by a row separator, which is a newline.
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

        // Otherwise it's the same row, so append a column separator, which is a tab.
        current_row_data.push_back(model->data(current).toString().toStdString());

        if (!row_count)  // first row
            header_data.push_back(model->headerData(current.column(), Qt::Horizontal).toString().toStdString());

        previous = current;
    }

    return data;
}

std::vector<std::vector<std::string>> AllBufferTableWidget::getText (unsigned int max_rows)
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

            // If you are at the start of the row the row number of the previous index
            // isn't the same.  Text is followed by a row separator, which is a newline.
            if (current_index.row() != previous_index.row())
            {
                if (!row_count)  // first row
                    data.push_back(header_data);

                data.push_back(current_row_data);
                current_row_data.clear();

                ++row_count;
            }

            // Otherwise it's the same row, so append a column separator, which is a tab.
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
