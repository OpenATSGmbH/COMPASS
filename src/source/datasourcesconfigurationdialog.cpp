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

#include "datasourcesconfigurationdialog.h"
#include "datasourcetablemodel.h"
#include "datasourceeditwidget.h"
#include "datasourcemanager.h"
#include "datasourcecreatedialog.h"
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

using namespace std;
using namespace Utils;

DataSourcesConfigurationDialog::DataSourcesConfigurationDialog(DataSourceManager& ds_man)
    : QDialog(), ds_man_(ds_man)
{
    setWindowTitle("Configure Data Sources");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);

    setMinimumSize(QSize(1000, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QHBoxLayout* top_layout = new QHBoxLayout();

    table_model_ = new DataSourceTableModel(ds_man_, *this);

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
            this, &DataSourcesConfigurationDialog::currentRowChanged);

    // HACKY adjust row size but works
    connect(table_view_->horizontalHeader(), SIGNAL(sectionResized(int, int, int)),
            table_view_, SLOT(resizeRowsToContents()));

    table_view_->resizeColumnsToContents();
    table_view_->resizeRowsToContents();
    top_layout->addWidget(table_view_);

    std::function<void(unsigned int)> update_ds_func =
        [this] (unsigned int ds_id) { table_model_->updateDataSource(ds_id); };

    std::function<void(unsigned int)> delete_ds_func = [this](unsigned int ds_id)
    {
        table_model_->beginModelReset();
        ds_man_.deleteConfigDataSource(ds_id);
        table_model_->endModelReset();
    };

    edit_widget_ = new DataSourceEditWidget (ds_man_, update_ds_func, delete_ds_func);
    top_layout->addWidget(edit_widget_);

    main_layout->addLayout(top_layout);

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    main_layout->addWidget(line);

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* new_button = new QPushButton ("Create New");
    new_button->setToolTip("Allows creation of new data source");
    connect(new_button, &QPushButton::clicked, this, &DataSourcesConfigurationDialog::newDSClickedSlot);
    button_layout->addWidget(new_button);

    button_layout->addStretch();

    QPushButton* import_button = new QPushButton ("Import");
    import_button->setToolTip("Imports data sources as JSON file. The data sources defined in the JSON file"
                              " will override existing ones (with same SAC/SIC) in the configuration and database");
    connect(import_button, &QPushButton::clicked, this, &DataSourcesConfigurationDialog::importClickedSlot);
    button_layout->addWidget(import_button);

    button_layout->addSpacing(32);

    QPushButton* delete_all_button = new QPushButton("Delete All");
    delete_all_button->setToolTip("Deletes all data sources existing in the configuration,"
                                  " but not those (also) defined in the database");
    connect(delete_all_button, &QPushButton::clicked, this, &DataSourcesConfigurationDialog::deleteAllClickedSlot);
    button_layout->addWidget(delete_all_button);

    button_layout->addSpacing(32);

    QPushButton* export_button = new QPushButton ("Export");
    export_button->setToolTip("Exports all data sources as JSON file");
    connect(export_button, &QPushButton::clicked, this, &DataSourcesConfigurationDialog::exportClickedSlot);
    button_layout->addWidget(export_button);

    button_layout->addStretch();

    QPushButton* done_button = new QPushButton("Done");
    connect(done_button, &QPushButton::clicked, this, &DataSourcesConfigurationDialog::doneClickedSlot);
    button_layout->addWidget(done_button);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}

void DataSourcesConfigurationDialog::currentRowChanged(const QModelIndex& current, const QModelIndex& previous)
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

    unsigned int id = table_model_->getIdOf(source_index);

    loginf << "current id " << id;

    edit_widget_->showID(id);
}

void DataSourcesConfigurationDialog::newDSClickedSlot()
{
    loginf;

    create_dialog_.reset(new DataSourceCreateDialog(*this, ds_man_));
    connect(create_dialog_.get(), &DataSourceCreateDialog::doneSignal,
            this, &DataSourcesConfigurationDialog::newDSDoneSlot);

    create_dialog_->show();
}

void DataSourcesConfigurationDialog::newDSDoneSlot()
{
    loginf;

    traced_assert(create_dialog_);

    if (!create_dialog_->cancelled())
    {
        string ds_type = create_dialog_->dsType();

        unsigned int sac = create_dialog_->sac();
        unsigned int sic = create_dialog_->sic();

        loginf << "ds_type " << ds_type
               << " sac " << sac << " sic " << sic;

        unsigned int ds_id = Number::dsIdFrom(sac, sic);

        traced_assert(!ds_man_.hasConfigDataSource(ds_id));

        table_model_->beginModelReset();

        ds_man_.createConfigDataSource(ds_id);
        traced_assert(ds_man_.hasConfigDataSource(ds_id));
        ds_man_.configDataSource(ds_id).dsType(ds_type);

        table_model_->endModelReset();

        auto const model_index = table_model_->dataSourceIndex(ds_id);

        auto const source_index = proxy_model_->mapFromSource(model_index);
        traced_assert(source_index.isValid());

        table_view_->selectRow(source_index.row());
    }

    create_dialog_->close();
    create_dialog_ = nullptr;
}

void DataSourcesConfigurationDialog::importClickedSlot()
{
    loginf;

    string filename = QFileDialog::getOpenFileName(
                this, "Import Data Sources",
                COMPASS::instance().lastUsedPath().c_str(), "*.json").toStdString();

    if (filename.size() > 0)
    {
        table_model_->beginModelReset();

        ds_man_.importDataSources(filename);

        table_model_->endModelReset();

        edit_widget_->updateContent();
    }
}

void DataSourcesConfigurationDialog::deleteAllClickedSlot()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(
                nullptr, "Delete All Data Sources",
                "This will delete all data sources existing in the configuration,"
                         " but not those (also) defined in the database. Do you want to continue?",
                QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes)
    {
        loginf << "deletion confirmed";

        table_model_->beginModelReset();

        ds_man_.deleteAllConfigDataSources();
        edit_widget_->clear();

        table_model_->endModelReset();
    }
}

void DataSourcesConfigurationDialog::exportClickedSlot()
{
    loginf;

    string filename = QFileDialog::getSaveFileName(
                this, "Export Data Sources as JSON",
                COMPASS::instance().lastUsedPath().c_str(), "*.json").toStdString();

    if (filename.size() > 0)
    {
        loginf << "file '" << filename << "'";

        ds_man_.exportDataSources(filename);
    }
}

void DataSourcesConfigurationDialog::doneClickedSlot()
{
    emit doneSignal();
}
