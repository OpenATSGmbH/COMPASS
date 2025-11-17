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

class JSONImportTask;
class RadarPlotPositionCalculatorTask;

class QPushButton;
class QListWidget;
class QCheckBox;
class QLineEdit;
class QComboBox;
class QHBoxLayout;
class QStackedWidget;
class QTabWidget;
class QLabel;

class JSONImportTaskWidget : public TaskWidget
{
    Q_OBJECT

  public slots:
    void fileLineIDEditSlot(const QString& text);
    void dateChangedSlot(QDate date);

    void testImportSlot();

    void addSchemaSlot();
    void removeSchemaSlot();
    void selectedSchemaChangedSlot(const QString& text);

    void addObjectParserSlot();
    void removeObjectParserSlot();
    void selectedObjectParserSlot(const QString& text);

    void expertModeChangedSlot();

  public:
    JSONImportTaskWidget(JSONImportTask& task, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~JSONImportTaskWidget();

    void updateSourceLabel();

    void selectSchema(const std::string& schema_name);

    void runStarted();
    void runDone();

  protected:
    JSONImportTask& task_;

    QHBoxLayout* main_layout_{nullptr};

    QTabWidget* tab_widget_{nullptr};

    QLabel* source_label_{nullptr};

    QListWidget* file_list_{nullptr};
    QPushButton* add_file_button_{nullptr};
    QPushButton* delete_file_button_{nullptr};
    QPushButton* delete_all_files_button_{nullptr};

    QComboBox* schema_box_{nullptr};
    QPushButton* add_schema_button_{nullptr};
    QPushButton* delete_schema_button_{nullptr};

    QLabel* parser_label_{nullptr};
    QComboBox* object_parser_box_{nullptr};
    QPushButton* add_object_parser_button_{nullptr};
    QPushButton* delete_object_parser_button_{nullptr};

    QStackedWidget* object_parser_widget_{nullptr};

    QPushButton* test_button_{nullptr};

    void addMainTab();
    void addMappingsTab();

    void updateSchemasBox();
    void updateToCurrentSchema();
    void updateParserBox();
};
