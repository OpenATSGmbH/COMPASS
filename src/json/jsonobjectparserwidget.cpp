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

#include "jsonobjectparserwidget.h"

#include "configuration.h"
#include "datatypeformatselectionwidget.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "files.h"
#include "jsonobjectparser.h"
#include "logger.h"
#include "unitselectionwidget.h"

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

using namespace Utils;

JSONObjectParserWidget::JSONObjectParserWidget(JSONObjectParser& parser, QWidget* parent)
    : QWidget(parent), parser_(&parser)
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_layout = new QVBoxLayout();

    std::string tmp = "JSON Object Parser " + parser_->name();
    QLabel* main_label = new QLabel(tmp.c_str());
    main_label->setFont(font_bold);
    main_layout->addWidget(main_label);

    {
        QGridLayout* grid = new QGridLayout();

        int row = 0;

        grid->addWidget(new QLabel("Active"), row, 0);

        active_check_ = new QCheckBox();
        active_check_->setDisabled(true); // only for show
        active_check_->setChecked(parser_->active());
        //connect(active_check_, &QCheckBox::clicked, this, &JSONObjectParserWidget::toggleActiveSlot);
        grid->addWidget(active_check_, row, 1);

        ++row;
        grid->addWidget(new QLabel("DBContent"), row, 0);
        grid->addWidget(new QLabel(parser_->dbContentName().c_str()), row, 1);

        ++row;
        grid->addWidget(new QLabel("JSON Container Key"), row, 0);

        json_container_key_edit_ = new QLineEdit();
        connect(json_container_key_edit_, SIGNAL(textEdited(const QString&)), this,
                SLOT(jsonContainerKeyChangedSlot()));
        grid->addWidget(json_container_key_edit_, row, 1);

        ++row;
        grid->addWidget(new QLabel("JSON Key"));

        json_key_edit_ = new QLineEdit();
        connect(json_key_edit_, SIGNAL(textEdited(const QString&)), this,
                SLOT(jsonKeyChangedSlot()));
        grid->addWidget(json_key_edit_, row, 1);

        ++row;
        grid->addWidget(new QLabel("JSON Value"));

        json_value_edit_ = new QLineEdit();
        connect(json_value_edit_, SIGNAL(textEdited(const QString&)), this,
                SLOT(jsonValueChangedSlot()));
        grid->addWidget(json_value_edit_, row, 1);

        ++row;
        grid->addWidget(new QLabel("Override Data Source"));

        override_data_source_check_ = new QCheckBox();
        connect(override_data_source_check_, SIGNAL(stateChanged(int)), this,
                SLOT(overrideDataSourceChangedSlot()));
        grid->addWidget(override_data_source_check_, row, 1);

        ++row;
        grid->addWidget(new QLabel("Data Source Variable"));

        data_source_variable_name_edit_ = new QLineEdit();
        connect(data_source_variable_name_edit_, SIGNAL(textEdited(const QString&)), this,
                SLOT(dataSourceVariableChangedSlot()));
        grid->addWidget(data_source_variable_name_edit_, row, 1);

        main_layout->addLayout(grid);
    }

    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);

    mappings_grid_ = new QGridLayout();
    updateMappingsGrid();

    QWidget* scroll_widget = new QWidget();
    scroll_widget->setLayout(mappings_grid_);

    scroll_area->setWidget(scroll_widget);
    main_layout->addWidget(scroll_area, 1);

    main_layout->addStretch();

    QPushButton* add_file_button = new QPushButton("Add");
    connect(add_file_button, SIGNAL(clicked()), this, SLOT(addNewMappingSlot()));
    main_layout->addWidget(add_file_button);

    setLayout(main_layout);

    update();

    show();
}

void JSONObjectParserWidget::update()
{
    traced_assert(parser_);
    traced_assert(json_container_key_edit_);
    traced_assert(json_key_edit_);
    traced_assert(json_value_edit_);
    traced_assert(override_data_source_check_);
    traced_assert(data_source_variable_name_edit_);

    json_container_key_edit_->setText(parser_->JSONContainerKey().c_str());
    json_key_edit_->setText(parser_->JSONKey().c_str());
    json_value_edit_->setText(parser_->JSONValue().c_str());

    data_source_variable_name_edit_->setText(parser_->dataSourceVariableName().c_str());
}

void JSONObjectParserWidget::updateActive()
{
    loginf << "value " << parser_->active();

    traced_assert(active_check_);
    active_check_->setChecked(parser_->active());
}

void JSONObjectParserWidget::updateMappingsGrid()
{
    loginf;
    traced_assert(parser_);
    traced_assert(mappings_grid_);

    QLayoutItem* child;
    while (!mappings_grid_->isEmpty() && (child = mappings_grid_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    // format_selections_.clear();

    QFont font_bold;
    font_bold.setBold(true);

    QIcon del_icon(Files::IconProvider::getIcon("delete.png"));

    int row = 0;

    QLabel* active_label = new QLabel("Active");
    active_label->setFont(font_bold);
    mappings_grid_->addWidget(active_label, row, 0);

    QLabel* key_label = new QLabel("JSON Key");
    key_label->setFont(font_bold);
    mappings_grid_->addWidget(key_label, row, 1);

    QLabel* comment_label = new QLabel("Comment");
    comment_label->setFont(font_bold);
    mappings_grid_->addWidget(comment_label, row, 2);

    QLabel* dbcontvar_label = new QLabel("DBContent Variable");
    dbcontvar_label->setFont(font_bold);
    mappings_grid_->addWidget(dbcontvar_label, row, 3);

    QLabel* mandatory_label = new QLabel("Mandatory");
    mandatory_label->setFont(font_bold);
    mappings_grid_->addWidget(mandatory_label, row, 4);

    QLabel* unit_label = new QLabel("Unit");
    unit_label->setFont(font_bold);
    mappings_grid_->addWidget(unit_label, row, 5);

    QLabel* format_label = new QLabel("Format");
    format_label->setFont(font_bold);
    mappings_grid_->addWidget(format_label, row, 6);

    QLabel* array_label = new QLabel("In Array");
    array_label->setFont(font_bold);
    mappings_grid_->addWidget(array_label, row, 7);

    QLabel* append_label = new QLabel("Append");
    append_label->setFont(font_bold);
    mappings_grid_->addWidget(append_label, row, 8);

    ++row;

    std::multimap<std::string, std::pair<unsigned int, JSONDataMapping*>> sorted_mappings;

    unsigned int index = 0;
    for (auto& map_it : *parser_)
    {
        map_it->initializeIfRequired();
        sorted_mappings.insert({map_it->jsonKey(), {index, map_it.get()}});
        ++index;
        // loginf << "UGA insert " << map_it.jsonKey();
    }

    for (auto& map_it : sorted_mappings)
    {
        QVariant data = QVariant::fromValue(map_it.second.second);  // JSONDataMapping* as QVariant

        QCheckBox* active_check = new QCheckBox();
        active_check->setChecked(map_it.second.second->active());
        connect(active_check, SIGNAL(stateChanged(int)), this, SLOT(mappingActiveChangedSlot()));
        active_check->setProperty("mapping", data);
        mappings_grid_->addWidget(active_check, row, 0);

        QLineEdit* key_edit = new QLineEdit(map_it.first.c_str());
        connect(key_edit, SIGNAL(textEdited(const QString&)), this, SLOT(mappingKeyChangedSlot()));
        key_edit->setProperty("mapping", data);
        mappings_grid_->addWidget(key_edit, row, 1);

        QLineEdit* comment_edit = new QLineEdit(map_it.second.second->comment().c_str());
        connect(comment_edit, SIGNAL(textEdited(const QString&)), this,
                SLOT(mappingCommentChangedSlot()));
        comment_edit->setProperty("mapping", data);
        mappings_grid_->addWidget(comment_edit, row, 2);

        dbContent::VariableSelectionWidget* var_sel = new dbContent::VariableSelectionWidget();
        var_sel->showMetaVariables(false);
        var_sel->showDBContentOnly(map_it.second.second->dbObjectName());
        var_sel->showEmptyVariable(true);
        if (map_it.second.second->hasVariable())
            var_sel->selectedVariable(map_it.second.second->variable());
        connect(var_sel, &dbContent::VariableSelectionWidget::selectionChanged,
                this, &JSONObjectParserWidget::mappingDBContentVariableChangedSlot);
        var_sel->setProperty("mapping", data);
        // var_sel->setProperty("row", row);
        mappings_grid_->addWidget(var_sel, row, 3);

        QCheckBox* mandatory_check = new QCheckBox();
        mandatory_check->setChecked(map_it.second.second->mandatory());
        connect(mandatory_check, SIGNAL(stateChanged(int)), this,
                SLOT(mappingMandatoryChangedSlot()));
        mandatory_check->setProperty("mapping", data);
        mappings_grid_->addWidget(mandatory_check, row, 4);

        UnitSelectionWidget* unit_sel = new UnitSelectionWidget(
            map_it.second.second->dimensionRef(), map_it.second.second->unitRef());
        mappings_grid_->addWidget(unit_sel, row, 5);

        DataTypeFormatSelectionWidget* data_format_widget = new DataTypeFormatSelectionWidget(
            map_it.second.second->formatDataTypeRef(), map_it.second.second->jsonValueFormatRef());

        mappings_grid_->addWidget(data_format_widget, row, 6);

        QCheckBox* in_array_check = new QCheckBox();
        in_array_check->setChecked(map_it.second.second->inArray());
        connect(in_array_check, SIGNAL(stateChanged(int)), this, SLOT(mappingInArrayChangedSlot()));
        in_array_check->setProperty("mapping", data);
        mappings_grid_->addWidget(in_array_check, row, 7);

        QCheckBox* append_check = new QCheckBox();
        append_check->setChecked(map_it.second.second->appendValue());
        connect(append_check, SIGNAL(stateChanged(int)), this, SLOT(mappingAppendChangedSlot()));
        append_check->setProperty("mapping", data);
        mappings_grid_->addWidget(append_check, row, 8);

        QPushButton* del = new QPushButton();
        del->setIcon(del_icon);
        del->setFixedSize(UI_ICON_SIZE);
        del->setFlat(UI_ICON_BUTTON_FLAT);
        connect(del, SIGNAL(clicked()), this, SLOT(mappingDeleteSlot()));
        del->setProperty("mapping", data);
        del->setProperty("index", map_it.second.first);
        mappings_grid_->addWidget(del, row, 9);

        row++;
    }
}

void JSONObjectParserWidget::setParser(JSONObjectParser& parser) { parser_ = &parser; }

//void JSONObjectParserWidget::toggleActiveSlot ()
//{
//    loginf;

//    QCheckBox* widget = static_cast<QCheckBox*>(sender());
//    traced_assert(widget);

//    parser_->active(widget->checkState() == Qt::Checked);
//}

void JSONObjectParserWidget::jsonContainerKeyChangedSlot()
{
    traced_assert(parser_);
    traced_assert(json_container_key_edit_);

    parser_->JSONContainerKey(json_container_key_edit_->text().toStdString());
}

void JSONObjectParserWidget::jsonKeyChangedSlot()
{
    traced_assert(parser_);
    traced_assert(json_key_edit_);

    parser_->JSONKey(json_key_edit_->text().toStdString());
}

void JSONObjectParserWidget::jsonValueChangedSlot()
{
    traced_assert(parser_);
    traced_assert(json_value_edit_);

    parser_->JSONValue(json_value_edit_->text().toStdString());
}

void JSONObjectParserWidget::overrideDataSourceChangedSlot()
{
    traced_assert(parser_);
    traced_assert(override_data_source_check_);

    parser_->overrideDataSource(override_data_source_check_->isChecked());
}

void JSONObjectParserWidget::dataSourceVariableChangedSlot()
{
    traced_assert(parser_);
    traced_assert(data_source_variable_name_edit_);

    parser_->dataSourceVariableName(data_source_variable_name_edit_->text().toStdString());
}

void JSONObjectParserWidget::addNewMappingSlot()
{
    traced_assert(parser_);

    auto config = Configuration::create("JSONDataMapping");
    config->addParameter<std::string>("json_key", config->getInstanceId());
    config->addParameter<std::string>("dbcontent_name", parser_->dbContentName());

    parser_->generateSubConfigurableFromConfig(std::move(config));

    updateMappingsGrid();
}

void JSONObjectParserWidget::mappingActiveChangedSlot()
{
    loginf;

    QCheckBox* widget = static_cast<QCheckBox*>(sender());
    traced_assert(widget);
    QVariant data = widget->property("mapping");

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    if (!mapping->hasVariable() && widget->checkState() == Qt::Checked)
    {
        QMessageBox m_warning(QMessageBox::Warning, "Activation failed", "DBContent Variable not defined.",
                              QMessageBox::Ok);

        m_warning.exec();

        widget->setChecked(false);
        return;
    }

    if (mapping->mandatory() && widget->checkState() != Qt::Checked)
    {
        QMessageBox m_warning(QMessageBox::Warning, "Deactivation failed",
                              "Deactivation of mandatory variables is not allowed.",
                              QMessageBox::Ok);

        m_warning.exec();

        widget->setChecked(true);

        return;
    }

    mapping->active(widget->checkState() == Qt::Checked);
}

void JSONObjectParserWidget::mappingKeyChangedSlot()
{
    loginf;

    QLineEdit* widget = static_cast<QLineEdit*>(sender());
    traced_assert(widget);
    QVariant data = widget->property("mapping");

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    mapping->jsonKey(widget->text().toStdString());
}

void JSONObjectParserWidget::mappingCommentChangedSlot()
{
    loginf;

    QLineEdit* widget = static_cast<QLineEdit*>(sender());
    traced_assert(widget);
    QVariant data = widget->property("mapping");

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    mapping->comment(widget->text().toStdString());
}

void JSONObjectParserWidget::mappingDBContentVariableChangedSlot()
{
    loginf;

    dbContent::VariableSelectionWidget* var_widget =
            static_cast<dbContent::VariableSelectionWidget*>(sender());
    traced_assert(var_widget);
    QVariant data = var_widget->property("mapping");
    // unsigned int row = var_widget->property("row").toUInt();

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    if (var_widget->hasVariable())
    {
        loginf << "variable set";

        mapping->dbcontVariableName(var_widget->selectedVariable().name());
    }
    else
    {
        loginf << "variable removed";

        mapping->dbcontVariableName("");
    }
}

void JSONObjectParserWidget::mappingMandatoryChangedSlot()
{
    loginf;

    QCheckBox* widget = static_cast<QCheckBox*>(sender());
    traced_assert(widget);
    QVariant data = widget->property("mapping");

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    mapping->mandatory(widget->checkState() == Qt::Checked);
}

void JSONObjectParserWidget::mappingInArrayChangedSlot()
{
    loginf;

    QCheckBox* widget = static_cast<QCheckBox*>(sender());
    traced_assert(widget);
    QVariant data = widget->property("mapping");

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    mapping->inArray(widget->checkState() == Qt::Checked);
}

void JSONObjectParserWidget::mappingAppendChangedSlot()
{
    loginf;

    QCheckBox* widget = static_cast<QCheckBox*>(sender());
    traced_assert(widget);
    QVariant data = widget->property("mapping");

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    mapping->appendValue(widget->checkState() == Qt::Checked);
}

void JSONObjectParserWidget::mappingDeleteSlot()
{
    loginf;

    QPushButton* widget = static_cast<QPushButton*>(sender());
    traced_assert(widget);
    QVariant data = widget->property("mapping");

    JSONDataMapping* mapping = data.value<JSONDataMapping*>();
    traced_assert(mapping);

    unsigned int index = widget->property("index").toUInt();

    traced_assert(parser_->hasMapping(index));
    parser_->removeMapping(index);

    updateMappingsGrid();
}
