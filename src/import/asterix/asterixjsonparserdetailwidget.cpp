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

#include "asterixjsonparserdetailwidget.h"
#include "datatypeformatselectionwidget.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "logger.h"
#include "unitselectionwidget.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableeditdialog.h"
#include "dbcontent/variable/variablecreatedialog.h"
#include "stringconv.h"
#include "traced_assert.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QFormLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QComboBox>

using namespace std;
using namespace Utils;

ASTERIXJSONParserDetailWidget::ASTERIXJSONParserDetailWidget(ASTERIXJSONParser& parser, QWidget* parent)
    : QWidget(parent), parser_(parser)
{
    expert_mode_ = COMPASS::instance().expertMode();

    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* form_layout = new QFormLayout;
    form_layout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);

    QFont font_bold;
    font_bold.setBold(true);

    //    QLabel* info_label_ {nullptr}; // shows type of mapping, or missing details
    info_label_ = new QLabel();
    form_layout->addRow("Info", info_label_);

    //    QCheckBox* active_check_ {nullptr};
    active_check_ = new QCheckBox();
    active_check_->setDisabled(true);
    //active_check->setChecked(map_it.second.second->active());
    connect(active_check_, SIGNAL(stateChanged(int)), this, SLOT(mappingActiveChangedSlot()));
    form_layout->addRow("Active", active_check_);

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    form_layout->addRow(line);


    //    QLineEdit* comment_edit_ {nullptr};

    //    QLabel* json_key_label_ {nullptr};
    QLabel* asterix_label = new QLabel("ASTERIX");
    asterix_label->setFont(font_bold);
    form_layout->addRow(asterix_label);

    json_key_box_ = new QComboBox();
    json_key_box_->setEditable(false);
    //json_key_box_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    connect(json_key_box_, SIGNAL(currentIndexChanged(const QString &)),
            this, SLOT(mappingJSONKeyChangedSlot(const QString &)));
    form_layout->addRow("JSON Key", json_key_box_);

    asterix_desc_label_ = new QLabel();
    asterix_desc_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    asterix_desc_label_->setTextFormat(Qt::TextFormat::PlainText);
    asterix_desc_label_->setWordWrap(true);
    form_layout->addRow("Description", asterix_desc_label_);

    // QLabel* asterix_editions_label_ {nullptr};

    asterix_editions_label_ = new QLabel();
    asterix_editions_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    asterix_editions_label_->setTextFormat(Qt::TextFormat::PlainText);
    asterix_editions_label_->setWordWrap(true);
    form_layout->addRow("Editions", asterix_editions_label_);

    //    QCheckBox* in_array_check_ {nullptr};

    in_array_check_ = new QCheckBox();
    in_array_check_->setDisabled(true);
    connect(in_array_check_, SIGNAL(stateChanged(int)), this, SLOT(mappingInArrayChangedSlot()));
    form_layout->addRow("In Array", in_array_check_);

    //    QCheckBox* append_check {nullptr};
    append_check = new QCheckBox();
    append_check->setDisabled(true);
    connect(append_check, SIGNAL(stateChanged(int)), this, SLOT(mappingAppendChangedSlot()));
    form_layout->addRow("Append", append_check);


    //    UnitSelectionWidget* unit_sel_ {nullptr};
    unit_sel_ = new UnitSelectionWidget();
    form_layout->addRow("Unit", unit_sel_);



    //    DataTypeFormatSelectionWidget* data_format_widget_ {nullptr};
    data_format_widget_ = new DataTypeFormatSelectionWidget();
    form_layout->addRow("Format", data_format_widget_);

    QFrame* line2 = new QFrame();
    line2->setFrameShape(QFrame::HLine);
    line2->setFrameShadow(QFrame::Sunken);
    form_layout->addRow(line2);

    // dbcont var

    QLabel* dbcontvar_label = new QLabel("DBContent Variable");
    dbcontvar_label->setFont(font_bold);
    form_layout->addRow(dbcontvar_label);

    dbcont_var_sel_ = new dbContent::VariableSelectionWidget();
    dbcont_var_sel_->showMetaVariables(false);
    dbcont_var_sel_->showDBContentOnly(parser_.dbContentName());
    dbcont_var_sel_->showEmptyVariable(true);
    dbcont_var_sel_->setDisabled(true);

    connect(dbcont_var_sel_, &dbContent::VariableSelectionWidget::selectionChanged,
            this, &ASTERIXJSONParserDetailWidget::mappingDBContentVariableChangedSlot);
    form_layout->addRow("Name", dbcont_var_sel_);

    dbcont_var_data_type_label_ = new QLabel();
    form_layout->addRow("Data Type", dbcont_var_data_type_label_);

    dbcont_var_comment_edit_ = new QTextEdit();
    dbcont_var_comment_edit_->setWordWrapMode(QTextOption::WrapMode::WrapAnywhere);

    connect(dbcont_var_comment_edit_, &QTextEdit::textChanged, this,
        &ASTERIXJSONParserDetailWidget::dbcontVariableCommentChangedSlot);

    form_layout->addRow("Comment", dbcont_var_comment_edit_);

    main_layout->addLayout(form_layout);

    // buttons

    new_dbcontvar_button_ = new QPushButton("New DBContent Variable");
    new_dbcontvar_button_->setHidden(true);
    connect(new_dbcontvar_button_, &QPushButton::clicked,
            this, &ASTERIXJSONParserDetailWidget::createNewDBVariableSlot);
    main_layout->addWidget(new_dbcontvar_button_);

    dbcontvar_edit_button_ = new QPushButton("Edit DBContent Variable");
    dbcontvar_edit_button_->setHidden(true);
    connect(dbcontvar_edit_button_, &QPushButton::clicked,
            this, &ASTERIXJSONParserDetailWidget::editDBVariableSlot);
    main_layout->addWidget(dbcontvar_edit_button_);

    dbcontvar_delete_button_ = new QPushButton("Delete DBContent Variable");
    dbcontvar_delete_button_->setToolTip("Deletes the DBContent Variable from the DBContent, not just from the Mapping.");
    dbcontvar_delete_button_->setHidden(true);
    connect(dbcontvar_delete_button_, &QPushButton::clicked,
            this, &ASTERIXJSONParserDetailWidget::deleteDBVariableSlot);
    main_layout->addWidget(dbcontvar_delete_button_);

    delete_mapping_button_ = new QPushButton("Delete Mapping");
    delete_mapping_button_->setHidden(true);
    connect(delete_mapping_button_, &QPushButton::clicked,
            this, &ASTERIXJSONParserDetailWidget::deleteMappingSlot);
    main_layout->addWidget(delete_mapping_button_);

    setLayout(main_layout);

    connect(&parser_, &ASTERIXJSONParser::rowContentChangedSignal,
            this, &ASTERIXJSONParserDetailWidget::rowContentChangedSlot);
}

void ASTERIXJSONParserDetailWidget::currentIndexChangedSlot (unsigned int index)
{
    loginf << "index " << index;

    setting_new_content_ = true;

    traced_assert(index < parser_.totalEntrySize());

    traced_assert(info_label_);
    traced_assert(active_check_);
    traced_assert(json_key_box_);
    traced_assert(unit_sel_);
    traced_assert(data_format_widget_);

    has_current_entry_ = true;
    entry_type_ = parser_.entryType(index);
    entry_index_ = index;

    if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping)
    {
        JSONDataMapping& mapping = parser_.mapping(entry_index_);

        loginf << "mapping " << entry_index_
               << " key '" << mapping.jsonKey() << "'";

        if (!parser_.existsJSONKeyInCATInfo(mapping.jsonKey()))
            info_label_->setText("JSON Key not found in ASTERIX Info");
        else
            info_label_->setText("Existing Mapping");

        active_check_->setEnabled(expert_mode_);
//        active_check_->setAttribute(Qt::WA_TransparentForMouseEvents, !expert_mode_); // read only
//        active_check_->setFocusPolicy(!expert_mode_ ? Qt::NoFocus : Qt::StrongFocus);
        active_check_->setChecked(mapping.active());

        showJSONKey(mapping.jsonKey(), !parser_.existsJSONKeyInCATInfo(mapping.jsonKey()));

        unit_sel_->update(mapping.dimensionRef(), mapping.unitRef());
        unit_sel_->setEnabled(expert_mode_);

        data_format_widget_->update(mapping.formatDataTypeRef(), mapping.jsonValueFormatRef());
        data_format_widget_->setEnabled(expert_mode_);

        showDBContentVariable(mapping.dbcontVariableName(), true);

        delete_mapping_button_->setHidden(false);
        delete_mapping_button_->setEnabled(expert_mode_);

        setting_new_content_ = false;

        return;
    }
    else if (entry_type_ == ASTERIXJSONParser::EntryType::UnmappedJSONKey)
    {

        info_label_->setText("Unmapped JSON Key");

        active_check_->setEnabled(false);
        active_check_->setChecked(false);

        string key = parser_.unmappedJSONKey(entry_index_);

        loginf << "not added JSON " << entry_index_
               << " key '" << key << "'";

        showJSONKey(key, false);

        unit_sel_->clear();
        unit_sel_->setEnabled(expert_mode_);

        data_format_widget_->clear();
        data_format_widget_->setEnabled(expert_mode_);

        showDBContentVariable("");

        delete_mapping_button_->setHidden(true);

        setting_new_content_ = false;

        return;
    }
    else if (entry_type_ == ASTERIXJSONParser::EntryType::UnmappedDBContentVariable)
    {
        info_label_->setText("Unmapped DBContent Variable");

        active_check_->setEnabled(false);
        active_check_->setChecked(false);

        string dbcontvar = parser_.unmappedDBContentVariable(entry_index_);

        loginf << "not added dbcontvar " << entry_index_
               << " key '" << dbcontvar << "'";

        showJSONKey("", false);

        unit_sel_->clear();
        unit_sel_->setEnabled(expert_mode_);

        data_format_widget_->clear();
        data_format_widget_->setEnabled(expert_mode_);

        showDBContentVariable(dbcontvar);

        delete_mapping_button_->setHidden(true);

        setting_new_content_ = false;
    }
    else
        throw runtime_error("ASTERIXJSONParserDetailWidget: currentIndexChangedSlot: unknown entry type");

}

void ASTERIXJSONParserDetailWidget::rowContentChangedSlot (unsigned int index)
{
    loginf << "index " << index;

    if (has_current_entry_ && entry_index_ == index)
    {
        currentIndexChangedSlot(index);
    }
}

void ASTERIXJSONParserDetailWidget::showJSONKey (const std::string& key, bool unmapped_selectable)
{
    traced_assert(json_key_box_);
    traced_assert(asterix_desc_label_);

    json_key_box_->clear();

    if (!key.size()) // shown none
    {
        traced_assert(!unmapped_selectable);

        json_key_box_->addItem("");
        json_key_box_->setCurrentText("");
        json_key_box_->setDisabled(true);

        asterix_desc_label_->setText("");
        asterix_editions_label_->setText("");
    }
    else
    {
        json_key_box_->addItem(key.c_str());
        json_key_box_->setCurrentText(key.c_str());

        if (unmapped_selectable)
        {
            for (const auto& jkey_it : parser_.notAddedJSONKeys())
            {
                if (jkey_it != key)
                    json_key_box_->addItem(jkey_it.c_str());

            }

            json_key_box_->setEnabled(expert_mode_);
        }
        else
            json_key_box_->setEnabled(false);

        const jASTERIX::CategoryItemInfo& item_info = parser_.categoryItemInfo();

        string editions;

        if (item_info.count(key))
        {
            asterix_desc_label_->setText(item_info.at(key).description_.c_str());

            for (auto& ed_it : item_info.at(key).editions_)
            {
                if (editions.size())
                    editions += ",";

                editions += ed_it;
            }

        }
        else
            asterix_desc_label_->setText("");

        asterix_editions_label_->setText(editions.c_str());
    }
}

void ASTERIXJSONParserDetailWidget::showDBContentVariable (const std::string& var_name, bool mapping_exists)
{
    traced_assert(dbcont_var_sel_);
    traced_assert(dbcont_var_comment_edit_);

    if (var_name.size())
    {
        dbcont_var_sel_->setEnabled(expert_mode_);

        traced_assert(parser_.dbContent().hasVariable(var_name));
        dbcont_var_sel_->selectedVariable(parser_.dbContent().variable(var_name));
        dbcont_var_data_type_label_->setText(dbcont_var_sel_->selectedVariable().dataTypeString().c_str());

        dbcont_var_comment_edit_->setEnabled(expert_mode_);
        dbcont_var_comment_edit_->setText(parser_.dbContent().variable(var_name).description().c_str());

        if (mapping_exists)
        {
            new_dbcontvar_button_->setText("New DBContent Variable");
            new_dbcontvar_button_->setHidden(!expert_mode_);
        }
        else
        {
            new_dbcontvar_button_->setHidden(true);
        }

        if (expert_mode_)
            dbcontvar_edit_button_->setText("Edit DBContent Variable");
        else
            dbcontvar_edit_button_->setText("Show DBContent Variable");

        dbcontvar_edit_button_->setHidden(false);
        dbcontvar_delete_button_->setHidden(!expert_mode_);
    }
    else
    {
        dbcont_var_sel_->setDisabled(false);
        dbcont_var_sel_->selectEmptyVariable();
        dbcont_var_data_type_label_->setText("");

        dbcont_var_comment_edit_->setDisabled(true);

        dbcont_var_comment_edit_->setText("");

        new_dbcontvar_button_->setHidden(!expert_mode_);

        if (mapping_exists)
            new_dbcontvar_button_->setText("New DBContent Variable");
        else
            new_dbcontvar_button_->setText("New DBContent Variable && Mapping");

        dbcontvar_edit_button_->setHidden(true);
        dbcontvar_delete_button_->setHidden(true);
    }
}


void ASTERIXJSONParserDetailWidget::mappingActiveChangedSlot()
{
    if (setting_new_content_)
        return;

    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping);
    traced_assert(active_check_);

    parser_.mapping(entry_index_).active(active_check_->checkState() == Qt::Checked);

    parser_.doMappingChecks();

    parser_.selectMapping(entry_index_);
}

void ASTERIXJSONParserDetailWidget::mappingJSONKeyChangedSlot (const QString& text)
{
    if (setting_new_content_)
        return;

    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping);
    traced_assert(in_array_check_);

    parser_.mapping(entry_index_).jsonKey(text.toStdString());

    parser_.doMappingChecks();

    parser_.selectMapping(entry_index_);
}

void ASTERIXJSONParserDetailWidget::mappingInArrayChangedSlot()
{
    if (setting_new_content_)
        return;

    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping);
    traced_assert(in_array_check_);

    parser_.mapping(entry_index_).inArray(in_array_check_->checkState() == Qt::Checked);
}

void ASTERIXJSONParserDetailWidget::mappingAppendChangedSlot()
{
    if (setting_new_content_)
        return;

    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping);
    traced_assert(append_check);

    parser_.mapping(entry_index_).appendValue(append_check->checkState() == Qt::Checked);
}

void ASTERIXJSONParserDetailWidget::mappingDBContentVariableChangedSlot()
{
    if (setting_new_content_)
        return;

    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping
            || entry_type_ == ASTERIXJSONParser::EntryType::UnmappedJSONKey);
    traced_assert(dbcont_var_sel_);

    if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping)
    {
        // setting variable in existing mapping

        if (dbcont_var_sel_->hasVariable())
        {
            parser_.mapping(entry_index_).dbcontVariableName(dbcont_var_sel_->selectedVariable().name());
            dbcont_var_data_type_label_->setText(dbcont_var_sel_->selectedVariable().dataTypeString().c_str());
        }
        else
        {
            parser_.mapping(entry_index_).dbcontVariableName("");
            dbcont_var_data_type_label_->setText("");
        }

        parser_.doMappingChecks();

        parser_.selectMapping(entry_index_);
    }
    else if (dbcont_var_sel_->hasVariable())
    {
        dbcont_var_data_type_label_->setText(dbcont_var_sel_->selectedVariable().dataTypeString().c_str());

        // create new mapping

        string json_key = parser_.unmappedJSONKey(entry_index_);

        auto new_cfg = Configuration::create("JSONDataMapping");

        new_cfg->addParameter<std::string>("json_key", json_key);
        new_cfg->addParameter<std::string>("db_content_name", parser_.dbContentName());
        new_cfg->addParameter<std::string>("db_content_variable_name", dbcont_var_sel_->selectedVariable().name());

        parser_.generateSubConfigurableFromConfig(std::move(new_cfg));

        parser_.doMappingChecks();

        traced_assert(parser_.hasJSONKeyInMapping(json_key));

        parser_.selectMapping(parser_.indexOfJSONKeyInMapping(json_key));
    }
}

void ASTERIXJSONParserDetailWidget::dbcontVariableCommentChangedSlot()
{
    if (setting_new_content_)
        return;

    traced_assert(has_current_entry_);
    traced_assert(dbcont_var_sel_);
    traced_assert(dbcont_var_sel_->hasVariable());
    traced_assert(dbcont_var_comment_edit_);

    dbcont_var_sel_->selectedVariable().description(dbcont_var_comment_edit_->document()->toPlainText().toStdString());
}


void ASTERIXJSONParserDetailWidget::createNewDBVariableSlot()
{
    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping
            || entry_type_ == ASTERIXJSONParser::EntryType::UnmappedJSONKey);
    traced_assert(dbcont_var_sel_);

    string json_key;

    if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping)
        json_key = parser_.mapping(entry_index_).jsonKey();
    else
        json_key = parser_.unmappedJSONKey(entry_index_);

    // set description
    string description {"From "+json_key};

    string name {json_key};

    if (parser_.categoryItemInfo().count(name))
    {
        if (parser_.categoryItemInfo().at(name).description_.size())
            description += "\n\n"+parser_.categoryItemInfo().at(name).description_;

//        if (parser_.categoryItemInfo().at(name).editions_.size())
//        {
//            string editions;

//            if (description.size())
//                description += "\n\n";

//            for (auto& ed_it : parser_.categoryItemInfo().at(name).editions_)
//            {
//                if (editions.size())
//                    editions += ",";

//                editions += ed_it;
//            }

//            description += "Editions: " + editions;
//        }
    }

    vector<string> parts = String::split(name, '.');
    if (parts.size())
        name = *parts.rbegin();


    dbContent::VariableCreateDialog dialog (parser_.dbContent(), name, description, this);

    int ret = dialog.exec();

    if (ret == QDialog::Accepted)
    {
        loginf << "accept";

        // create new dbcont var
        {
            traced_assert(!parser_.dbContent().hasVariable(dialog.name()));

            auto new_cfg = Configuration::create("Variable");
            new_cfg->addParameter<std::string>("name", dialog.name());
            new_cfg->addParameter<std::string>("short_name", dialog.shortName());
            new_cfg->addParameter<std::string>("description", dialog.description());
            new_cfg->addParameter<std::string>("db_column_name", dialog.dbColumnName());
            new_cfg->addParameter<std::string>("data_type_str", dialog.dataTypeStr());
            new_cfg->addParameter<std::string>("representation_str", dialog.representationStr());
            new_cfg->addParameter<std::string>("dimension", dialog.dimension());
            new_cfg->addParameter<std::string>("unit", dialog.unit());

            parser_.dbContent().generateSubConfigurableFromConfig(std::move(new_cfg));

            traced_assert(parser_.dbContent().hasVariable(dialog.name()));
        }


        if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping) // set in existing mapping
        {
            parser_.mapping(entry_index_).dbcontVariableName(dialog.name());

            parser_.doMappingChecks();

            parser_.selectMapping(entry_index_);
        }
        else // create new mapping
        {
            traced_assert(!parser_.hasJSONKeyInMapping(json_key));

            auto new_cfg = Configuration::create("JSONDataMapping");
            new_cfg->addParameter<std::string>("json_key", json_key);
            new_cfg->addParameter<std::string>("db_content_name", parser_.dbContentName());
            new_cfg->addParameter<std::string>("db_content_variable_name", dialog.name());

            parser_.generateSubConfigurableFromConfig(std::move(new_cfg));

            parser_.doMappingChecks();

            traced_assert(parser_.hasJSONKeyInMapping(json_key));

            parser_.selectMapping(parser_.indexOfJSONKeyInMapping(json_key));
        }
    }
    else
    {
        loginf << "reject";
    }

//    if (dialog.variableEdited())
//    {
//        if (dialog.variable().name() != current_var_name) // renaming done
//        {
//            if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping) // change in mapping
//                parser_.mapping(entry_index_).dbcontVariableName(dialog.variable().name());
//        }

//        parser_.doMappingChecks();

//        if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping)
//            parser_.selectMapping(entry_index_); // ok since order in mappings is the same
//        else if (entry_type_ == ASTERIXJSONParser::EntryType::UnmappedDBContVariable)
//            parser_.selectUnmappedDBContVariable(dialog.variable().name()); // search for new name
//    }

//    loginf << "done";

}

void ASTERIXJSONParserDetailWidget::deleteDBVariableSlot()
{
    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping
            || entry_type_ == ASTERIXJSONParser::EntryType::UnmappedDBContentVariable);
    traced_assert(dbcont_var_sel_);

    traced_assert(dbcont_var_sel_->hasVariable());


    string dbcontvar_name = dbcont_var_sel_->selectedVariable().name();

    loginf << "deleting var '" << dbcontvar_name << "'";

    // delete variable
    traced_assert(parser_.dbContent().hasVariable(dbcontvar_name));
    parser_.dbContent().deleteVariable(dbcontvar_name);
    
    loginf;

    if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping)
    {
        // remove from mapping
        parser_.mapping(entry_index_).dbcontVariableName("");

        parser_.doMappingChecks();

        parser_.selectMapping(entry_index_);
    }
    else // deleted unmapped dbcontvar, clear widget
    {
        parser_.doMappingChecks();

        // clear widget
        has_current_entry_ = false;

        setting_new_content_ = true;

        active_check_->setDisabled(true);
        active_check_->setChecked(false);

        showJSONKey("", false);
        unit_sel_->clear();
        data_format_widget_->clear();
        showDBContentVariable("");

        delete_mapping_button_->setHidden(true);
        new_dbcontvar_button_->setHidden(true);
        dbcontvar_edit_button_->setHidden(true);
        dbcontvar_delete_button_->setHidden(true);

        setting_new_content_ = false;
    }
}
void ASTERIXJSONParserDetailWidget::editDBVariableSlot()
{
    loginf;

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping
            || entry_type_ == ASTERIXJSONParser::EntryType::UnmappedDBContentVariable);
    traced_assert(dbcont_var_sel_);

    traced_assert(dbcont_var_sel_->hasVariable());

    dbContent::VariableEditDialog dialog (dbcont_var_sel_->selectedVariable(), this);

    string current_var_name = dbcont_var_sel_->selectedVariable().name();

    dialog.exec();

    if (dialog.variableEdited())
    {
        if (dialog.variable().name() != current_var_name) // renaming done
        {
            if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping) // change in mapping
                parser_.mapping(entry_index_).dbcontVariableName(dialog.variable().name());
        }

        parser_.doMappingChecks();

        if (entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping)
            parser_.selectMapping(entry_index_); // ok since order in mappings is the same
        else if (entry_type_ == ASTERIXJSONParser::EntryType::UnmappedDBContentVariable)
            parser_.selectUnmappedDBContentVariable(dialog.variable().name()); // search for new name
    }

    loginf << "done";
}

void ASTERIXJSONParserDetailWidget::deleteMappingSlot()
{
    traced_assert(delete_mapping_button_);
    loginf << "delete";

    traced_assert(has_current_entry_);
    traced_assert(entry_type_ == ASTERIXJSONParser::EntryType::ExistingMapping);

    traced_assert(parser_.hasMapping(entry_index_));
    parser_.removeMapping(entry_index_);

    parser_.doMappingChecks();

    // clear widget
    has_current_entry_ = false;

    setting_new_content_ = true;

    active_check_->setDisabled(true);
    active_check_->setChecked(false);

    showJSONKey("", false);
    unit_sel_->clear();
    data_format_widget_->clear();
    showDBContentVariable("");

    delete_mapping_button_->setHidden(true);
    new_dbcontvar_button_->setHidden(true);
    dbcontvar_edit_button_->setHidden(true);
    dbcontvar_delete_button_->setHidden(true);

    setting_new_content_ = false;

}
