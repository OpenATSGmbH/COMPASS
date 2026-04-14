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

#include <taskwidget.h>

class ManageSectorsTask;

class QVBoxLayout;
class QPushButton;
class QListWidget;
class QTableWidget;
class QCheckBox;
class QTabWidget;
class QTextEdit;
class QTableWidgetItem;

class ManageSectorsTaskWidget : public TaskWidget
{
    Q_OBJECT
public slots:
    void expertModeChangedSlot();

    void addFileSlot();
    void deleteFileSlot();
    void deleteAllFilesSlot();
    void selectedFileSlot();
    void updateFileListSlot();

    void importSlot();

    void sectorItemChangedSlot(QTableWidgetItem* item);

    void changeSectorColorSlot();
    void deleteSectorSlot();

    void exportSectorsSlot ();
    void clearSectorsSlot ();
    void importSectorsSlot ();

    void updateSectorTableSlot();

private slots:
    void importAirSpaceSectorsSlot ();

public:
    ManageSectorsTaskWidget(ManageSectorsTask& task, QWidget* parent = nullptr);

    void addFile(const std::string& filename);
    void selectFile(const std::string& filename);

    void updateParseMessage ();

    void importSectorsJSON (const std::string& filename);
    void importAirSpaceSectorsJSON (const std::string& filename);

protected:
    void addImportTab();
    void addManageTab();

    ManageSectorsTask& task_;

    QVBoxLayout* main_layout_{nullptr};

    QTabWidget* tab_widget_{nullptr};

    QListWidget* file_list_{nullptr};
    QPushButton* add_file_button_{nullptr};
    QPushButton* delete_file_button_{nullptr};
    QPushButton* delete_all_files_button_{nullptr};

    QTextEdit* parse_msg_edit_ {nullptr};

    QPushButton* import_button_ {nullptr};

    QTableWidget* sector_table_{nullptr};
    QStringList table_columns_{"ID", "Sector Name",  "Layer Name", "Exclude", "Num Points", "Altitude Minimum",
                               "Altitude Maximum", "Color", "Delete"};
};
