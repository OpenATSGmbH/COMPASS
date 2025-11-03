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

#include "buffer.h"
#include "buffertablemodel.h"
#include "dbcontent/dbcontent.h"
#include "tableviewdatasource.h"
#include "logger.h"
#include "compass.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <QApplication>
#include <QClipboard>
#include <QFileDialog>
#include <QKeyEvent>
#include <QMessageBox>
#include <QTableView>
#include <QVBoxLayout>

// using namespace Utils;

BufferTableWidget::BufferTableWidget(DBContent& object, TableView& view,
                                     TableViewDataSource& data_source, QWidget* parent,
                                     Qt::WindowFlags f)
    : QWidget(parent, f), object_(object), view_(view), data_source_(data_source)
{
    //setAutoFillBackground(true);

    QVBoxLayout* layout = new QVBoxLayout();

    table_ = new QTableView(this);
    table_->setSelectionBehavior(QAbstractItemView::SelectItems);
    table_->setSelectionMode(QAbstractItemView::ContiguousSelection);
    model_ = new BufferTableModel(this, object_, view_, data_source_);
    table_->setModel(model_);

    connect(model_, &BufferTableModel::exportDoneSignal,
            this, &BufferTableWidget::exportDoneSlot);

    layout->addWidget(table_);
    table_->show();

    setLayout(layout);
}

BufferTableWidget::~BufferTableWidget() = default;

void BufferTableWidget::clear()
{
    traced_assert(model_);

    model_->clearData();
}

void BufferTableWidget::show(std::shared_ptr<Buffer> buffer) 
{
    traced_assert(buffer);

    logdbg << "object " << object_.name() << " buffer size "
           << buffer->size() << " properties " << buffer->properties().size();
    traced_assert(table_);
    traced_assert(model_);

    model_->setData(buffer);
    table_->resizeColumnsToContents();

    logdbg << " BufferTableWidget: show: end";
}

void BufferTableWidget::exportSlot()
{
    loginf << "object " << object_.name();

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

void BufferTableWidget::exportDoneSlot(bool cancelled) { emit exportDoneSignal(cancelled); }

void BufferTableWidget::updateToSettingsChange()
{
    logdbg;

    traced_assert(model_);
    model_->rebuild();
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

// void BufferTableWidget::usePresentation(bool value)
// {
//     traced_assert(model_);
//     model_->rebuild();
//     traced_assert(table_);
//     table_->resizeColumnsToContents();
// }

// void BufferTableWidget::ignoreNonTargetReports(bool value)
// {
//     traced_assert(model_);
//     model_->rebuild();
//     traced_assert(table_);
//     table_->resizeColumnsToContents();
// }

// bool BufferTableWidget::showOnlySelected() const
// {
//     traced_assert(model_);
//     return model_->showOnlySelected();
// }

// bool BufferTableWidget::usePresentation() const
// {
//     traced_assert(model_);
//     return model_->usePresentation();
// }

void BufferTableWidget::resetModel()
{
    traced_assert(model_);
    model_->reset();
}

void BufferTableWidget::updateToSelection()
{
    traced_assert(model_);
    model_->rebuild();
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

TableView& BufferTableWidget::view() const { return view_; }

bool BufferTableWidget::hasData() const
{
    traced_assert(model_);
    return model_->hasData();
}

void BufferTableWidget::resizeColumns()
{
    traced_assert(table_);
    table_->resizeColumnsToContents();
}

void BufferTableWidget::keyPressEvent(QKeyEvent* event)
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
}
