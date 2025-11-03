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

#include "jsonimporttaskwidget.h"

//#include "compass.h"
//#include "dbcontent/dbcontent.h"
//#include "dbcontent/dbcontentcombobox.h"
//#include "dbcontent/dbcontentmanager.h"
//#include "dbcontent/variable/variable.h"
//#include "dbcontent/variable/variableselectionwidget.h"
#include "jsonimporttask.h"
#include "jsonparsingschema.h"
#include "logger.h"
#include "dbcontent/selectdialog.h"
//#include "stringconv.h"
#include "util/timeconv.h"

#include <QCheckBox>
#include <QFormLayout>
#include <QFrame>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QDateEdit>

using namespace Utils;
using namespace std;

JSONImportTaskWidget::JSONImportTaskWidget(JSONImportTask& task, QWidget* parent, Qt::WindowFlags f)
    : TaskWidget(parent, f), task_(task)
{
    main_layout_ = new QHBoxLayout();

    tab_widget_ = new QTabWidget();
    main_layout_->addWidget(tab_widget_);

    addMainTab();
    addMappingsTab();

    expertModeChangedSlot();

    setLayout(main_layout_);
}

void JSONImportTaskWidget::addMainTab()
{
    traced_assert(tab_widget_);

    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_tab_layout = new QVBoxLayout();

    {
        QFormLayout* source_layout = new QFormLayout();

        source_label_ = new QLabel();
        source_layout->addWidget(source_label_);

        // line
        QComboBox* file_line_box = new QComboBox();
        file_line_box->addItems({"1", "2", "3", "4"});

        connect(file_line_box, &QComboBox::currentTextChanged,
                this, &JSONImportTaskWidget::fileLineIDEditSlot);
        source_layout->addRow("Line ID", file_line_box);

        // date
        QDateEdit* date_edit = new QDateEdit();
        date_edit->setDisplayFormat("yyyy-MM-dd");

        //loginf << "UGA " << Time::toDateString(task_.date());

        QDate date = QDate::fromString(Time::toDateString(task_.date()).c_str(), "yyyy-MM-dd");
        //loginf << "UGA2 " << date.toString().toStdString();

        date_edit->setDate(date);

        connect(date_edit, &QDateEdit::dateChanged,
                this, &JSONImportTaskWidget::dateChangedSlot);
        source_layout->addRow("UTC Day", date_edit);

        updateSourceLabel();

        main_tab_layout->addLayout(source_layout);
    }

    main_tab_layout->addStretch();

    // button stuff
    {
        test_button_ = new QPushButton("Test Import");
        connect(test_button_, &QPushButton::clicked, this, &JSONImportTaskWidget::testImportSlot);
        main_tab_layout->addWidget(test_button_);
    }

    QWidget* main_tab_widget = new QWidget();
    main_tab_widget->setContentsMargins(0, 0, 0, 0);
    main_tab_widget->setLayout(main_tab_layout);
    tab_widget_->addTab(main_tab_widget, "Main");
}

void JSONImportTaskWidget::addMappingsTab()
{
    QVBoxLayout* mappings_layout = new QVBoxLayout();

    QFont font_bold;
    font_bold.setBold(true);

    // schema selection
    {
        QHBoxLayout* schema_layout = new QHBoxLayout();

        QLabel* schemas_label = new QLabel("Schema");
        schemas_label->setFont(font_bold);
        schema_layout->addWidget(schemas_label);

        schema_box_ = new QComboBox();
        connect(schema_box_, SIGNAL(activated(const QString&)), this,
                SLOT(selectedSchemaChangedSlot(const QString&)));
        updateSchemasBox();
        schema_layout->addWidget(schema_box_);

        add_schema_button_ = new QPushButton("Add");
        connect(add_schema_button_, SIGNAL(clicked()), this, SLOT(addSchemaSlot()));
        schema_layout->addWidget(add_schema_button_);

        delete_schema_button_ = new QPushButton("Remove");
        connect(delete_schema_button_, SIGNAL(clicked()), this, SLOT(removeSchemaSlot()));
        schema_layout->addWidget(delete_schema_button_);

        mappings_layout->addLayout(schema_layout);

        QFrame* line = new QFrame();
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        mappings_layout->addWidget(line);
    }

    // json object parser stuff
    {
        QVBoxLayout* parsers_layout = new QVBoxLayout();

        parser_label_ = new QLabel("JSON Object Parsers");
        parser_label_->setFont(font_bold);
        parsers_layout->addWidget(parser_label_);

        object_parser_box_ = new QComboBox();
        connect(object_parser_box_, SIGNAL(currentIndexChanged(const QString&)), this,
                SLOT(selectedObjectParserSlot(const QString&)));

        parsers_layout->addWidget(object_parser_box_);

        QHBoxLayout* button_layout = new QHBoxLayout();

        add_object_parser_button_ = new QPushButton("Add");
        connect(add_object_parser_button_, SIGNAL(clicked()), this, SLOT(addObjectParserSlot()));
        button_layout->addWidget(add_object_parser_button_);

        delete_object_parser_button_ = new QPushButton("Remove");
        connect(delete_object_parser_button_, SIGNAL(clicked()), this,
                SLOT(removeObjectParserSlot()));
        button_layout->addWidget(delete_object_parser_button_);

        parsers_layout->addLayout(button_layout);

        QFrame* line = new QFrame();
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        parsers_layout->addWidget(line);

        object_parser_widget_ = new QStackedWidget();
        parsers_layout->addWidget(object_parser_widget_);

        updateToCurrentSchema();

        mappings_layout->addLayout(parsers_layout);
    }

    QWidget* mappings_tab_widget = new QWidget();
    mappings_tab_widget->setContentsMargins(0, 0, 0, 0);
    mappings_tab_widget->setLayout(mappings_layout);
    tab_widget_->addTab(mappings_tab_widget, "Mappings");
}

JSONImportTaskWidget::~JSONImportTaskWidget() {}

void JSONImportTaskWidget::updateSourceLabel()
{
    source_label_->setText(("Source: "+task_.importFilename()).c_str());
}

void JSONImportTaskWidget::selectSchema(const std::string& schema_name)
{
    traced_assert(task_.hasSchema(schema_name));
    task_.currentSchemaName(schema_name);

    updateToCurrentSchema();

}

void JSONImportTaskWidget::addSchemaSlot()
{
    loginf;

    bool ok;
    QString text =
        QInputDialog::getText(this, tr("Input Name of New Schema"), tr("JSON Parsing Schema Name:"),
                              QLineEdit::Normal, "New", &ok);

    if (ok)
    {
        std::string name = text.toStdString();
        loginf << "name '" << name << "'";

        if (!name.size())
        {
            QMessageBox m_warning(QMessageBox::Warning, "JSON Parsing Schema Adding Failed",
                                  "A schema name must be set.", QMessageBox::Ok);

            m_warning.exec();
            return;
        }

        if (task_.hasSchema(name))
        {
            QMessageBox m_warning(QMessageBox::Warning, "JSON Parsing Schema Adding Failed",
                                  "Schema with this name is already defined.", QMessageBox::Ok);

            m_warning.exec();
            return;
        }

        std::string instance = "JSONParsingSchema" + name + "0";

        auto config = Configuration::create("JSONParsingSchema", instance);
        config->addParameter<std::string>("name", name);

        task_.generateSubConfigurableFromConfig(std::move(config));
        updateSchemasBox();
    }
}

void JSONImportTaskWidget::removeSchemaSlot()
{
    loginf;

    if (!task_.currentSchemaName().size())
    {
        QMessageBox m_warning(QMessageBox::Warning, "JSON File Deletion Failed",
                              "Please select a file in the list.", QMessageBox::Ok);
        m_warning.exec();
        return;
    }

    task_.removeCurrentSchema();
    updateSchemasBox();
    updateToCurrentSchema();
}

void JSONImportTaskWidget::selectedSchemaChangedSlot(const QString& text)
{
    loginf << "text " << text.toStdString();

    traced_assert(task_.hasSchema(text.toStdString()));
    task_.currentSchemaName(text.toStdString());

    updateToCurrentSchema();
}

void JSONImportTaskWidget::updateSchemasBox()
{
    loginf;

    schema_box_->clear();

    schema_box_->addItem("jASTERIX");

    for (auto& schema_it : task_)
    {
        schema_box_->addItem(schema_it.first.c_str());
    }

    if (task_.currentSchemaName().size())
    {
        schema_box_->setCurrentText(task_.currentSchemaName().c_str());
    }
    else if (schema_box_->currentText().size())
    {
        task_.currentSchemaName(schema_box_->currentText().toStdString());
    }
}

void JSONImportTaskWidget::updateToCurrentSchema()
{
    updateParserBox();

    if (task_.currentSchemaName() == "jASTERIX")
    {
        parser_label_->setText("Editing Only Possible in Import ASTERIX Task");
        object_parser_box_->setDisabled(true);
        delete_schema_button_->setDisabled(true);
        add_object_parser_button_->setDisabled(true);
        delete_object_parser_button_->setDisabled(true);
        object_parser_widget_->setDisabled(true);
    }
    else
    {
        parser_label_->setText("JSON Object Parsers");
        object_parser_box_->setDisabled(false);
        delete_schema_button_->setDisabled(false);
        add_object_parser_button_->setDisabled(false);
        delete_object_parser_button_->setDisabled(false);
        object_parser_widget_->setDisabled(false);
    }
}

void JSONImportTaskWidget::addObjectParserSlot()
{
    if (!task_.hasCurrentSchema())
    {
        QMessageBox m_warning(QMessageBox::Warning, "JSON Object Parser Adding Failed",
                              "No current JSON Parsing Schema is selected.", QMessageBox::Ok);

        m_warning.exec();
        return;
    }

    dbContent::SelectDBContentDialog dialog;

    int ret = dialog.exec();

    if (ret == QDialog::Accepted)
    {
        std::string dbcontent_name = dialog.selectedObject();
        loginf << "dbcontent_name "
               << dbcontent_name;

        shared_ptr<JSONParsingSchema> current = task_.currentJSONSchema();

        if (!dbcontent_name.size() || current->hasObjectParser(dbcontent_name))
        {
            QMessageBox m_warning(QMessageBox::Warning, "JSON Object Parser Adding Failed",
                                  "Object parser name empty or already defined.", QMessageBox::Ok);

            m_warning.exec();
            return;
        }

        std::string instance = "JSONObjectParser" + dbcontent_name + "0";

        auto config = Configuration::create("JSONObjectParser", instance);
        config->addParameter<std::string>("dbcontent_name", dbcontent_name);

        current->generateSubConfigurableFromConfig(std::move(config));
        updateParserBox();
    }
}
void JSONImportTaskWidget::removeObjectParserSlot()
{
    loginf;

    if (object_parser_box_->currentIndex() >= 0)
    {
        std::string name = object_parser_box_->currentText().toStdString();

        traced_assert(task_.hasCurrentSchema());
        shared_ptr<JSONParsingSchema> current = task_.currentJSONSchema();

        traced_assert(current->hasObjectParser(name));
        current->removeParser(name);

        updateParserBox();
    }
}

void JSONImportTaskWidget::selectedObjectParserSlot(const QString& text)
{
    loginf << "text " << text.toStdString();

    if (object_parser_widget_)
        while (object_parser_widget_->count() > 0)
            object_parser_widget_->removeWidget(object_parser_widget_->widget(0));

    traced_assert(object_parser_box_);

    if (object_parser_box_->currentIndex() >= 0)
    {
        std::string name = object_parser_box_->currentText().toStdString();

        traced_assert(task_.hasCurrentSchema());
        traced_assert(task_.currentJSONSchema()->hasObjectParser(name));
        traced_assert(object_parser_widget_);

        if (object_parser_widget_->indexOf(task_.currentJSONSchema()->parser(name).widget()) < 0)
            object_parser_widget_->addWidget(task_.currentJSONSchema()->parser(name).widget());

        object_parser_widget_->setCurrentWidget(task_.currentJSONSchema()->parser(name).widget());
    }
}

void JSONImportTaskWidget::expertModeChangedSlot() {}


void JSONImportTaskWidget::fileLineIDEditSlot(const QString& text)
{
    loginf << "value '" << text.toStdString() << "'";

    bool ok;

    unsigned int line_id = text.toUInt(&ok);

    traced_assert(ok);

    traced_assert(line_id > 0 && line_id <= 4);

    task_.fileLineID(line_id-1);
}

void JSONImportTaskWidget::dateChangedSlot(QDate date)
{
    string tmp = date.toString("yyyy-MM-dd").toStdString();

    loginf << "start" << tmp;

    task_.date(Time::fromDateString(tmp));
}

void JSONImportTaskWidget::testImportSlot()
{
    loginf;

    if (!task_.canImportFile())
    {
        QMessageBox m_warning(QMessageBox::Warning, "JSON File Test Import Failed",
                              "Please select a file in the list.", QMessageBox::Ok);
        m_warning.exec();
        return;
    }

    task_.test(true);
    task_.run();
}

void JSONImportTaskWidget::runStarted()
{
    loginf;

    test_button_->setDisabled(true);
}

void JSONImportTaskWidget::runDone()
{
    loginf;

    test_button_->setDisabled(false);
}

void JSONImportTaskWidget::updateParserBox()
{
    loginf;

    traced_assert(object_parser_box_);
    object_parser_box_->clear();

    if (task_.hasCurrentSchema())
    {
        if (task_.currentSchemaName() == "jASTERIX")
            return;

        loginf << "current schema " << task_.currentJSONSchema()->name();

        for (auto& parser_it : *task_.currentJSONSchema())
        {
            object_parser_box_->addItem(parser_it.first.c_str());
        }
    }
}
