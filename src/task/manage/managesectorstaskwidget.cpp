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

#include "managesectorstaskwidget.h"
#include "managesectorstask.h"
#include "logger.h"
#include "compass.h"
#include "evaluationmanager.h"
#include "sector.h"
#include "airspace.h"
#include "sectorlayer.h"
#include "files.h"
#include "importsectordialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QTableWidget>
#include <QHeaderView>
#include <QColorDialog>
#include <QApplication>
#include <QTreeWidget>
#include <QTreeWidgetItem>

using namespace std;
using namespace Utils;

ManageSectorsTaskWidget::ManageSectorsTaskWidget(ManageSectorsTask& task, QWidget* parent)
    : TaskWidget(parent), task_(task)
{
    main_layout_ = new QVBoxLayout();

    tab_widget_ = new QTabWidget();

    main_layout_->addWidget(tab_widget_);

    addImportTab();
    addManageTab();

    updateSectorTableSlot();

    QObject::connect(&COMPASS::instance(), &COMPASS::databaseOpenedSignal,
                     this, &ManageSectorsTaskWidget::updateSectorTableSlot);
    QObject::connect(&COMPASS::instance(), &COMPASS::databaseClosedSignal,
                     this, &ManageSectorsTaskWidget::updateSectorTableSlot);

    setLayout(main_layout_);
}

void ManageSectorsTaskWidget::expertModeChangedSlot() {}

void ManageSectorsTaskWidget::addImportTab()
{
    traced_assert(tab_widget_);

    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_tab_layout = new QVBoxLayout();

    // file stuff
    {
        QVBoxLayout* files_layout = new QVBoxLayout();

        QLabel* files_label = new QLabel("File Selection");
        files_label->setFont(font_bold);
        files_layout->addWidget(files_label);

        file_list_ = new QListWidget();
        file_list_->setWordWrap(true);
        file_list_->setTextElideMode(Qt::ElideNone);
        file_list_->setSelectionBehavior(QAbstractItemView::SelectItems);
        file_list_->setSelectionMode(QAbstractItemView::SingleSelection);
        connect(file_list_, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(selectedFileSlot()));

        updateFileListSlot();
        files_layout->addWidget(file_list_);

        QHBoxLayout* button_layout = new QHBoxLayout();

        add_file_button_ = new QPushButton("Add");
        connect(add_file_button_, &QPushButton::clicked, this, &ManageSectorsTaskWidget::addFileSlot);
        button_layout->addWidget(add_file_button_);

        delete_file_button_ = new QPushButton("Remove");
        connect(delete_file_button_, &QPushButton::clicked, this, &ManageSectorsTaskWidget::deleteFileSlot);
        button_layout->addWidget(delete_file_button_);

        delete_all_files_button_ = new QPushButton("Remove All");
        connect(delete_all_files_button_, &QPushButton::clicked, this, &ManageSectorsTaskWidget::deleteAllFilesSlot);
        button_layout->addWidget(delete_all_files_button_);

        files_layout->addLayout(button_layout);

        main_tab_layout->addLayout(files_layout);

        parse_msg_edit_ = new QTextEdit ();
        parse_msg_edit_->setReadOnly(true);
        main_tab_layout->addWidget(parse_msg_edit_);

        import_button_ = new QPushButton("Import");
        connect (import_button_, &QPushButton::clicked, this, &ManageSectorsTaskWidget::importSlot);
        import_button_->setDisabled(true);
        main_tab_layout->addWidget(import_button_); // is enabled in updateContext
    }

    if (task_.currentFilename().size())
        updateParseMessage();

    QWidget* main_tab_widget = new QWidget();
    main_tab_widget->setContentsMargins(0, 0, 0, 0);
    main_tab_widget->setLayout(main_tab_layout);
    tab_widget_->addTab(main_tab_widget, "Import");
}

void ManageSectorsTaskWidget::addManageTab()
{
    QVBoxLayout* manage_tab_layout = new QVBoxLayout();

    sector_table_ = new QTableWidget();
    sector_table_->setEditTriggers(QAbstractItemView::AllEditTriggers);
    sector_table_->setColumnCount(table_columns_.size());
    sector_table_->setHorizontalHeaderLabels(table_columns_);
    sector_table_->verticalHeader()->setVisible(false);
    sector_table_->sortByColumn(2, Qt::DescendingOrder);
    connect(sector_table_, &QTableWidget::itemChanged, this,
            &ManageSectorsTaskWidget::sectorItemChangedSlot);

    manage_tab_layout->addWidget(sector_table_);

    QFont font_bold;
    font_bold.setBold(true);

    // bottom buttons
    {
        QHBoxLayout* button_layout = new QHBoxLayout();

        QPushButton* export_button_ = new QPushButton("Export All");
        connect(export_button_, &QPushButton::clicked, this,
                &ManageSectorsTaskWidget::exportSectorsSlot);
        button_layout->addWidget(export_button_);

        QPushButton* clear_button_ = new QPushButton("Clear All");
        connect(clear_button_, &QPushButton::clicked, this,
                &ManageSectorsTaskWidget::clearSectorsSlot);
        button_layout->addWidget(clear_button_);

        QPushButton* import_button_ = new QPushButton("Import");
        connect(import_button_, &QPushButton::clicked, this,
                &ManageSectorsTaskWidget::importSectorsSlot);
        button_layout->addWidget(import_button_);

        QPushButton* import_airspace_button_ = new QPushButton("Import Air Space");
        connect(import_airspace_button_, &QPushButton::clicked, this,
                &ManageSectorsTaskWidget::importAirSpaceSectorsSlot);
        button_layout->addWidget(import_airspace_button_);

        import_airspace_button_->setVisible(!COMPASS::instance().isAppImage());

        manage_tab_layout->addLayout(button_layout);
    }

    QWidget* manage_tab_widget = new QWidget();
    manage_tab_widget->setContentsMargins(0, 0, 0, 0);
    manage_tab_widget->setLayout(manage_tab_layout);
    tab_widget_->addTab(manage_tab_widget, "Manage");
}

void ManageSectorsTaskWidget::updateSectorTableSlot()
{
    logdbg << "start";

    traced_assert(sector_table_);

    sector_table_->blockSignals(true);

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    sector_table_->setDisabled(true); // otherwise first element is edited after
    sector_table_->clearContents();

    if (!eval_man.sectorsLoaded())
    {
        sector_table_->blockSignals(false);
        sector_table_->setDisabled(false);

        return;
    }

    traced_assert(eval_man.sectorsLoaded());
    vector<std::shared_ptr<SectorLayer>>& sector_layers = eval_man.sectorsLayers();

    unsigned int num_layers=0;
    for (auto& sec_lay_it : sector_layers)
        num_layers += sec_lay_it->sectors().size();

    sector_table_->setRowCount(num_layers);

    int row = 0;
    int col = 0;

    for (auto& sec_lay_it : sector_layers)
    {
        string layer_name = sec_lay_it->name();

        for (auto& sec_it : sec_lay_it->sectors())
        {
            unsigned int sector_id = sec_it->id();

            col = 0;

            {  // Sector ID
                QTableWidgetItem* item = new QTableWidgetItem(QString::number(sec_it->id()));
                item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                item->setData(Qt::UserRole, QVariant(sector_id));
                sector_table_->setItem(row, col, item);
            }

            {  // Sector Name
                ++col;
                QTableWidgetItem* item = new QTableWidgetItem(sec_it->name().c_str());
                item->setFlags(item->flags() | Qt::ItemIsEditable);
                item->setData(Qt::UserRole, QVariant(sector_id));
                sector_table_->setItem(row, col, item);
            }

            {  // Layer Name
                ++col;
                QTableWidgetItem* item = new QTableWidgetItem(layer_name.c_str());
                item->setFlags(item->flags() | Qt::ItemIsEditable);
                item->setData(Qt::UserRole, QVariant(sector_id));
                sector_table_->setItem(row, col, item);
            }

            {  // Exclude
                ++col;
                QTableWidgetItem* item = new QTableWidgetItem();
                item->setFlags(item->flags() | Qt::ItemIsEditable | Qt::ItemIsUserCheckable);
                item->setCheckState(sec_it->isExclusionSector() ? Qt::Checked : Qt::Unchecked);
                //item1->setCheckState();
                item->setData(Qt::UserRole, QVariant(sector_id));
                sector_table_->setItem(row, col, item);
            }

            {  // Num Points
                ++col;
                QTableWidgetItem* item = new QTableWidgetItem(QString::number(sec_it->points().size()));
                item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                item->setData(Qt::UserRole, QVariant(sector_id));
                sector_table_->setItem(row, col, item);
            }

            {  // Altitude Minimum
                ++col;
                if (sec_it->hasMinimumAltitude())
                {
                    QTableWidgetItem* item = new QTableWidgetItem(QString::number(sec_it->minimumAltitude()));
                    item->setFlags(item->flags() | Qt::ItemIsEditable);
                    item->setData(Qt::UserRole, QVariant(sector_id));
                    sector_table_->setItem(row, col, item);
                }
                else
                {
                    QTableWidgetItem* item = new QTableWidgetItem();
                    item->setFlags(item->flags() | Qt::ItemIsEditable);
                    item->setData(Qt::UserRole, QVariant(sector_id));
                    item->setBackground(Qt::darkGray);
                    sector_table_->setItem(row, col, item);
                }
            }

            {  // Altitude Maximum
                ++col;
                if (sec_it->hasMaximumAltitude())
                {
                    QTableWidgetItem* item = new QTableWidgetItem(QString::number(sec_it->maximumAltitude()));
                    item->setFlags(item->flags() | Qt::ItemIsEditable);
                    item->setData(Qt::UserRole, QVariant(sector_id));
                    sector_table_->setItem(row, col, item);
                }
                else
                {
                    QTableWidgetItem* item = new QTableWidgetItem();
                    item->setFlags(item->flags() | Qt::ItemIsEditable);
                    item->setData(Qt::UserRole, QVariant(sector_id));
                    item->setBackground(Qt::darkGray);
                    sector_table_->setItem(row, col, item);
                }
            }

            {  // Color
                ++col;

                QPushButton* button = new QPushButton();
                QPalette pal = button->palette();
                pal.setColor(QPalette::Button, QColor(sec_it->colorStr().c_str()));
                //loginf << "UGA " << QColor(sector->colorStr().c_str()).name().toStdString();
                button->setAutoFillBackground(true);
                button->setPalette(pal);
                button->setFlat(true);
                button->update();

                connect (button, &QPushButton::clicked, this, &ManageSectorsTaskWidget::changeSectorColorSlot);

                button->setProperty("sector_id", QVariant(sec_it->id()));
                sector_table_->setCellWidget(row, col, button);
            }

            {  // Delete
                ++col;

                QPushButton* button = new QPushButton();
                button->setIcon(Files::IconProvider::getIcon("delete.png"));
                button->setFlat(true);

                connect (button, &QPushButton::clicked, this, &ManageSectorsTaskWidget::deleteSectorSlot);

                button->setProperty("sector_id", QVariant(sec_it->id()));
                sector_table_->setCellWidget(row, col, button);
            }

            ++row;
        }
    }

    sector_table_->resizeColumnsToContents();

    sector_table_->blockSignals(false);
    sector_table_->setDisabled(false);
}

void ManageSectorsTaskWidget::addFileSlot()
{
    QFileDialog dialog(this);
    dialog.setWindowTitle("Add Sector File(s)");
    dialog.setDirectory(COMPASS::instance().lastUsedPath().c_str());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    // dialog.setNameFilter(trUtf8("Splits (*.000 *.001)"));
    QStringList fileNames;
    if (dialog.exec())
    {
        for (auto& filename : dialog.selectedFiles())
            addFile(filename.toStdString());
    }
}

void ManageSectorsTaskWidget::addFile(const std::string& filename)
{
    if (!task_.hasFile(filename))
        task_.addFile(filename);
}

void ManageSectorsTaskWidget::selectFile(const std::string& filename)
{
    traced_assert(task_.hasFile(filename));
    task_.currentFilename(filename);

    QList<QListWidgetItem*> items = file_list_->findItems(filename.c_str(), Qt::MatchExactly);
    traced_assert(items.size() > 0);

    for (auto item_it : items)
    {
        traced_assert(item_it);
        file_list_->setCurrentItem(item_it);
    }

    updateParseMessage();
}

void ManageSectorsTaskWidget::updateParseMessage ()
{
    loginf;

    traced_assert(parse_msg_edit_);
    parse_msg_edit_->setText(task_.parseMessage().c_str());

    import_button_->setEnabled(task_.canImportFile());
}

void ManageSectorsTaskWidget::deleteFileSlot()
{
    loginf;

    if (!file_list_->currentItem() || !task_.currentFilename().size())
    {
        QMessageBox m_warning(QMessageBox::Warning, "Sector File Deletion Failed",
                              "Please select a file in the list.", QMessageBox::Ok);
        m_warning.exec();
        return;
    }

    traced_assert(task_.currentFilename().size());
    traced_assert(task_.hasFile(task_.currentFilename()));
    task_.removeCurrentFilename();
}

void ManageSectorsTaskWidget::deleteAllFilesSlot()
{
    loginf;
    task_.removeAllFiles();
}

void ManageSectorsTaskWidget::selectedFileSlot()
{
    traced_assert(file_list_->currentItem());

    QString filename = file_list_->currentItem()->text();

    loginf << "filename '" << filename.toStdString();

    traced_assert(task_.hasFile(filename.toStdString()));
    task_.currentFilename(filename.toStdString());
}

void ManageSectorsTaskWidget::updateFileListSlot()
{
    traced_assert(file_list_);

    file_list_->clear();

    for (auto it : task_.fileList())
    {
        if (!Files::fileExists(it))
        {
            logwrn << "file '" << it << "' does not exist";
            continue;
        }

        QListWidgetItem* item = new QListWidgetItem(tr(it.c_str()), file_list_);

        if (it == task_.currentFilename())
            file_list_->setCurrentItem(item);
    }
}

void ManageSectorsTaskWidget::importSlot()
{
    loginf;


    string filename = task_.currentFilename();

    filename = Files::getFilenameFromPath(filename);

    if (filename.find(".", 0) != string::npos)
        filename = filename.substr(0, filename.find(".", 0));

    ImportSectorDialog dialog(filename.c_str(), this);
    int ret = dialog.exec();

    if (ret == QDialog::Accepted)
    {
        loginf << "accepted, layer name '" << dialog.layerName()
               << "' exclude " << dialog.exclude();

        traced_assert(task_.canImportFile());
        task_.importFile(dialog.layerName(), dialog.exclude(), dialog.color());

        updateSectorTableSlot();
    }
    else
        loginf << "cancelled";
}

void ManageSectorsTaskWidget::sectorItemChangedSlot(QTableWidgetItem* item)
{
    loginf;

    traced_assert(item);
    traced_assert(sector_table_);

    bool ok;
    unsigned int sector_id = item->data(Qt::UserRole).toUInt(&ok);
    traced_assert(ok);
    int col = sector_table_->currentColumn();

    traced_assert(col < table_columns_.size());
    std::string col_name = table_columns_.at(col).toStdString();

    std::string text = item->text().toStdString();

    loginf << "sector_id " << sector_id
           << " col_name " << col_name << " text '" << text << "'";

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    traced_assert(eval_man.hasSector(sector_id));

    std::shared_ptr<Sector> sector = eval_man.sector(sector_id);

    if (col_name == "Sector Name")
    {
        if (text.size())
        {
            if (eval_man.hasSector(text, sector->layerName()))
            {
                QMessageBox m_warning(QMessageBox::Warning, "Sector Change Failed",
                ("Layer '"+sector->layerName()+"' Sector '"+text+"' already exists.").c_str(), QMessageBox::Ok);
                m_warning.exec();
            }
            else
                sector->name(text);
        }
    }
    else if (col_name == "Layer Name")
    {
        if (text.size())
        {
            if (eval_man.hasSector(sector->name(), text))
            {
                QMessageBox m_warning(QMessageBox::Warning, "Sector Change Failed",
                ("Layer '"+text+"' Sector '"+sector->name()+"' already exists.").c_str(), QMessageBox::Ok);
                m_warning.exec();
            }
            else
                sector->layerName(text);
        }
    }
    else if (col_name == "Exclude")
    {
        sector->exclude(item->checkState() == Qt::Checked);
    }
    else if (col_name == "Altitude Minimum")
    {
        if (!text.size())
            sector->removeMinimumAltitude();
        else
        {
            double value = item->text().toDouble(&ok);

            if (ok)
                sector->setMinimumAltitude(value);
        }
    }
    else if (col_name == "Altitude Maximum")
    {
        if (!text.size())
            sector->removeMaximumAltitude();
        else
        {
            double value = item->text().toDouble(&ok);

            if (ok)
                sector->setMaximumAltitude(value);
        }
    }
    else
        traced_assert(false);  // impossible

    updateSectorTableSlot();
}

void ManageSectorsTaskWidget::changeSectorColorSlot()
{
    loginf;

    QPushButton* button = dynamic_cast<QPushButton*>(sender());
    traced_assert(button);

    QVariant sector_id_var = button->property("sector_id");
    traced_assert(sector_id_var.isValid());

    unsigned int sector_id = sector_id_var.toUInt();

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    traced_assert(eval_man.hasSector(sector_id));

    std::shared_ptr<Sector> sector = eval_man.sector(sector_id);

    QColor current_color = QColor(sector->colorStr().c_str());

    QColor color =
            QColorDialog::getColor(current_color, QApplication::activeWindow(), "Select Sector Color");

    if (color.isValid())
    {
        loginf << "color " << color.name().toStdString();
        sector->colorStr(color.name().toStdString());
        updateSectorTableSlot();
    }
}

void ManageSectorsTaskWidget::deleteSectorSlot()
{
    loginf;

    QPushButton* button = dynamic_cast<QPushButton*>(sender());
    traced_assert(button);

    QVariant sector_id_var = button->property("sector_id");
    traced_assert(sector_id_var.isValid());

    unsigned int sector_id = sector_id_var.toUInt();

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    traced_assert(eval_man.hasSector(sector_id));

    std::shared_ptr<Sector> sector = eval_man.sector(sector_id);

    eval_man.deleteSector(sector);

    updateSectorTableSlot();
}

void ManageSectorsTaskWidget::exportSectorsSlot ()
{
    loginf;

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    QFileDialog dialog(nullptr);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setNameFilter("JSON Files (*.json)");
    dialog.setDefaultSuffix("json");
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);

    QStringList file_names;
    if (dialog.exec())
        file_names = dialog.selectedFiles();

    QString filename;

    if (file_names.size() == 1)
        filename = file_names.at(0);
    else
        loginf << "cancelled";

    if (filename.size() > 0)
        eval_man.exportSectors(filename.toStdString());
}

void ManageSectorsTaskWidget::clearSectorsSlot ()
{
    loginf;

    COMPASS::instance().evaluationManager().deleteAllSectors();

    updateSectorTableSlot();
}

void ManageSectorsTaskWidget::importSectorsSlot ()
{
    loginf;

    QString filename =
        QFileDialog::getOpenFileName(nullptr, "Import Sectors from JSON",
                                     COMPASS::instance().lastUsedPath().c_str(), "*.json");

    if (filename.size() > 0)
    {
        importSectorsJSON(filename.toStdString());
    }
    else
        loginf << "cancelled";
}

void ManageSectorsTaskWidget::importSectorsJSON (const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    traced_assert(Files::fileExists(filename));

    COMPASS::instance().evaluationManager().importSectors(filename);

    updateSectorTableSlot();
}

void ManageSectorsTaskWidget::importAirSpaceSectorsSlot()
{
    loginf;

    QString filename =
        QFileDialog::getOpenFileName(nullptr, "Import Air Space Sectors from JSON",
                                     COMPASS::instance().lastUsedPath().c_str(), "*.json");

    if (filename.isEmpty())
        return;

    importAirSpaceSectorsJSON(filename.toStdString());
}

void ManageSectorsTaskWidget::importAirSpaceSectorsJSON(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    traced_assert(Files::fileExists(filename));

    auto max_sector_id = COMPASS::instance().evaluationManager().getMaxSectorId();

    AirSpace air_space;

    if (!air_space.readJSON(filename, max_sector_id))
    {
        QMessageBox::critical(this, "Error", "Air space file could not be read.");
        return;
    }

    QDialog dlg;
    dlg.setWindowTitle("Choose sectors to import");

    QVBoxLayout* layout = new QVBoxLayout;
    dlg.setLayout(layout);

    QTreeWidget* list = new QTreeWidget(&dlg);
    list->setHeaderLabels({ "", "Sector", "Layer", "#Points", "Altitude min", "Altitude max" });
    list->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::ResizeToContents);
    list->header()->setSectionResizeMode(1, QHeaderView::ResizeMode::Stretch);
    list->header()->setSectionResizeMode(2, QHeaderView::ResizeMode::Stretch);
    list->header()->setSectionResizeMode(3, QHeaderView::ResizeMode::ResizeToContents);
    list->header()->setSectionResizeMode(4, QHeaderView::ResizeMode::ResizeToContents);
    list->header()->setSectionResizeMode(5, QHeaderView::ResizeMode::ResizeToContents);

    layout->addWidget(list);

    QHBoxLayout* button_layout = new QHBoxLayout;
    QPushButton* button_import = new QPushButton("Import");
    QPushButton* button_cancel = new QPushButton("Cancel");

    connect (button_import, &QPushButton::pressed, &dlg, &QDialog::accept);
    connect (button_cancel, &QPushButton::pressed, &dlg, &QDialog::reject);

    button_layout->addStretch(1);
    button_layout->addWidget(button_import);
    button_layout->addWidget(button_cancel);

    layout->addLayout(button_layout);

    auto layers = air_space.layers();

    for (auto l : layers)
    {
        for (auto s : l->sectors())
        {
            QTreeWidgetItem* item = new QTreeWidgetItem;
            item->setCheckState(0, Qt::CheckState::Checked);
            item->setText(1, QString::fromStdString(s->name()));
            item->setText(2, QString::fromStdString(l->name()));
            item->setText(3, QString::number(s->points().size()));

            bool has_alt_min = s->hasMinimumAltitude();
            bool has_alt_max = s->hasMaximumAltitude();

            item->setText(4, has_alt_min ? QString::number(s->minimumAltitude()) : "-");
            item->setText(5, has_alt_max ? QString::number(s->maximumAltitude()) : "-");

            list->addTopLevelItem(item);
        }
    }

    dlg.resize(500, 800);

    auto ret = dlg.exec();
    if (ret == QDialog::Rejected)
        return;

    std::set<std::string> sectors_to_import;

    for (int i = 0; i < list->topLevelItemCount(); ++i)
    {
        QTreeWidgetItem* item = list->topLevelItem(i);
        if (item->checkState(0) == Qt::Checked)
            sectors_to_import.insert(item->text(1).toStdString());
    }

    if (sectors_to_import.empty())
        return;

    if (!COMPASS::instance().evaluationManager().importAirSpace(air_space, sectors_to_import))
        QMessageBox::critical(this, "Error", "Importing air space sectors failed.");

    updateSectorTableSlot();
}
