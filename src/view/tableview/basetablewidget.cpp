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

#include "basetablewidget.h"
#include "basetablemodel.h"
#include "tableviewdatasource.h"
#include "logger.h"
#include "compass.h"

#include <QApplication>
#include <QClipboard>
#include <QFileDialog>
#include <QHeaderView>
#include <QKeyEvent>
#include <QMessageBox>
#include <QTableView>
#include <QVBoxLayout>

BaseBufferTableWidget::BaseBufferTableWidget(TableView& view, TableViewDataSource& data_source,
                                             QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f), view_(view), data_source_(data_source)
{
    QVBoxLayout* layout = new QVBoxLayout();

    table_ = new QTableView(this);
    table_->setSelectionBehavior(QAbstractItemView::SelectItems);
    table_->setSelectionMode(QAbstractItemView::ContiguousSelection);
    table_->setCornerButtonEnabled(false);

    layout->addWidget(table_);
    table_->show();

    setLayout(layout);
}

BaseBufferTableWidget::~BaseBufferTableWidget() {}

void BaseBufferTableWidget::initModel(BaseBufferTableModel* model)
{
    traced_assert(model);
    model_ = model;

    table_->setModel(model_);
    table_->horizontalHeader()->setResizeContentsPrecision(100); // sample first 100 rows only
    table_->setSortingEnabled(true);

    // default sort by Timestamp column
    int ts_col = -1;
    for (int col = 0; col < model_->columnCount(); ++col)
    {
        if (model_->headerData(col, Qt::Horizontal).toString() == "Timestamp")
        {
            ts_col = col;
            break;
        }
    }
    if (ts_col >= 0)
        table_->sortByColumn(ts_col, Qt::AscendingOrder);

    connect(model_, &BaseBufferTableModel::exportDoneSignal,
            this, &BaseBufferTableWidget::exportDoneSlot);
}

void BaseBufferTableWidget::clear()
{
    traced_assert(model_);
    model_->clearData();
}

void BaseBufferTableWidget::exportSlot()
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

void BaseBufferTableWidget::exportDoneSlot(bool cancelled) { emit exportDoneSignal(cancelled); }

void BaseBufferTableWidget::updateToSettingsChange()
{
    logdbg;

    traced_assert(model_);
    model_->rebuild();
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

void BaseBufferTableWidget::resetModel()
{
    traced_assert(model_);
    model_->reset();
}

void BaseBufferTableWidget::updateToSelection()
{
    traced_assert(model_);
    model_->rebuild();
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

TableView& BaseBufferTableWidget::view() const { return view_; }

void BaseBufferTableWidget::resizeColumns()
{
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

void BaseBufferTableWidget::keyPressEvent(QKeyEvent* event)
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
