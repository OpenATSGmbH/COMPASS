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

#include "fftsconfigurationdialog.h"
#include "ffttablemodel.h"
#include "ffteditwidget.h"
#include "fftmanager.h"
#include "util/number.h"
#include "logger.h"
#include "compass.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QSortFilterProxyModel>
#include <QTableView>
#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>

using namespace std;
using namespace Utils;

FFTsConfigurationDialog::FFTsConfigurationDialog(FFTManager& ds_man)
    : QDialog(), fft_man_(ds_man)
{
    setWindowTitle("Configure FFTs");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);

    setMinimumSize(QSize(1000, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QHBoxLayout* top_layout = new QHBoxLayout();

    table_model_ = new FFTTableModel(fft_man_, *this);

    proxy_model_ = new QSortFilterProxyModel();
    proxy_model_->setSourceModel(table_model_);

    table_view_ = new QTableView();
    table_view_->setModel(proxy_model_);
    table_view_->setSortingEnabled(true);
    table_view_->sortByColumn(2, Qt::AscendingOrder);
    table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_view_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_view_->horizontalHeader()->resizeSections(QHeaderView::ResizeToContents);
    //table_view_->setIconSize(QSize(24, 24));
    table_view_->setWordWrap(true);
    table_view_->reset();

    connect(table_view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
            this, &FFTsConfigurationDialog::currentRowChanged);

    // HACKY adjust row size but works
    connect(table_view_->horizontalHeader(), SIGNAL(sectionResized(int, int, int)),
            table_view_, SLOT(resizeRowsToContents()));

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
    top_layout->addWidget(table_view_);

    edit_widget_ = new FFTEditWidget (fft_man_, *this);
    top_layout->addWidget(edit_widget_);

    main_layout->addLayout(top_layout);

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    main_layout->addWidget(line);

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* new_button = new QPushButton ("Create New");
    new_button->setToolTip("Allows creation of new FFT");
    connect(new_button, &QPushButton::clicked, this, &FFTsConfigurationDialog::newFFTClickedSlot);
    button_layout->addWidget(new_button);

    button_layout->addStretch();

    QPushButton* import_button = new QPushButton ("Import");
    import_button->setToolTip("Imports FFTs as JSON file. The FFTs defined in the JSON file"
                              " will override existing ones (with same name) in the configuration and database");
    connect(import_button, &QPushButton::clicked, this, &FFTsConfigurationDialog::importClickedSlot);
    button_layout->addWidget(import_button);

    button_layout->addSpacing(32);

    QPushButton* delete_all_button = new QPushButton("Delete All");
    delete_all_button->setToolTip("Deletes all FFTs existing in the configuration,"
                                  " but not those (also) defined in the database");
    connect(delete_all_button, &QPushButton::clicked, this, &FFTsConfigurationDialog::deleteAllClickedSlot);
    button_layout->addWidget(delete_all_button);

    button_layout->addSpacing(32);

    QPushButton* export_button = new QPushButton ("Export");
    export_button->setToolTip("Exports all FFTs as JSON file");
    connect(export_button, &QPushButton::clicked, this, &FFTsConfigurationDialog::exportClickedSlot);
    button_layout->addWidget(export_button);

    button_layout->addStretch();

    QPushButton* done_button = new QPushButton("Done");
    connect(done_button, &QPushButton::clicked, this, &FFTsConfigurationDialog::doneClickedSlot);
    button_layout->addWidget(done_button);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}

void FFTsConfigurationDialog::updateFFT(const std::string& name)
{
    table_model_->updateFFT(name);
}

void FFTsConfigurationDialog::beginResetModel()
{
    table_model_->beginModelReset();
}

void FFTsConfigurationDialog::endResetModel()
{
    table_model_->endModelReset();
}

void FFTsConfigurationDialog::currentRowChanged(const QModelIndex& current, const QModelIndex& previous)
{
    traced_assert(edit_widget_);

    if (!current.isValid())
    {
        loginf << "invalid index";

        edit_widget_->clear();

        return;
    }

    auto const source_index = proxy_model_->mapToSource(current);
    traced_assert(source_index.isValid());

    string name = table_model_->getNameOf(source_index);

    loginf << "current name " << name;

    edit_widget_->showFFT(name);
}

void FFTsConfigurationDialog::newFFTClickedSlot()
{
    loginf;

    bool ok;
    QString text =
            QInputDialog::getText(this, tr("FFT Name"),
                                  tr("Specify a (unique) FFT name:"), QLineEdit::Normal, "", &ok);

    if (ok)
    {
        std::string name = text.toStdString();

        if (!name.size())
        {
            QMessageBox m_warning(QMessageBox::Warning, "Adding FFT Failed",
                                  "FFT has to have a non-empty name.", QMessageBox::Ok);
            m_warning.exec();
            return;
        }

        if (fft_man_.hasConfigFFT(name) || fft_man_.hasDBFFT(name))
        {
            QMessageBox m_warning(QMessageBox::Warning, "Adding FFT Failed",
                                  "FFT with this name already exists.", QMessageBox::Ok);
            m_warning.exec();
            return;
        }

        beginResetModel();

        fft_man_.addNewFFT(name);

        endResetModel();

        QModelIndexList items = table_model_->match(
                    table_model_->index(0, 0), Qt::DisplayRole, name.c_str(),
                    -1, Qt::MatchExactly);
        traced_assert(items.size() == 1);

        QModelIndex tmp = items.at(0);
        traced_assert(tmp.isValid());

        auto const target_index = proxy_model_->mapFromSource(tmp);
        traced_assert(target_index.isValid());

        table_view_->selectionModel()->setCurrentIndex(target_index,
                                                       QItemSelectionModel::Select | QItemSelectionModel::Rows);


    }
}

void FFTsConfigurationDialog::importClickedSlot()
{
    loginf;

    string filename = QFileDialog::getOpenFileName(
                this, "Import FFTs",
                COMPASS::instance().lastUsedPath().c_str(), "*.json").toStdString();

    if (filename.size() > 0)
    {
        table_model_->beginModelReset();

        fft_man_.importFFTs(filename);

        table_model_->endModelReset();

        table_view_->resizeColumnsToContents();
        edit_widget_->updateContent();
    }
}

void FFTsConfigurationDialog::deleteAllClickedSlot()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(
                nullptr, "Delete All FFTs",
                "This will delete all FFTs existing in the configuration and in the database. Do you want to continue?",
                QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {
        loginf << "deletion confirmed";

        table_model_->beginModelReset();

        fft_man_.deleteAllFFTs();
        edit_widget_->clear();

        table_model_->endModelReset();
    }
}

void FFTsConfigurationDialog::exportClickedSlot()
{
    loginf;

    string filename = QFileDialog::getSaveFileName(
                this, "Export FFTs as JSON",
                COMPASS::instance().lastUsedPath().c_str(), "*.json").toStdString();

    if (filename.size() > 0)
    {
        loginf << "file '" << filename << "'";

        fft_man_.exportFFTs(filename);
    }
}

void FFTsConfigurationDialog::doneClickedSlot()
{
    emit doneSignal();
}
