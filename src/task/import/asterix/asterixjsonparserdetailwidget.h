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

#include "asterixjsonparser.h"

#include <QWidget>

class UnitSelectionWidget;
class DataTypeFormatSelectionWidget;

class QLabel;
class QLineEdit;
class QCheckBox;
class QPushButton;
class QTextEdit;
class QComboBox;

namespace dbContent
{
class VariableSelectionWidget;
}

class ASTERIXJSONParserDetailWidget : public QWidget
{
    Q_OBJECT

public slots:
    void rowContentChangedSlot (unsigned int index);
    void currentIndexChangedSlot (unsigned int index);

    void mappingActiveChangedSlot();

    void mappingJSONKeyChangedSlot (const QString &text);
    void mappingInArrayChangedSlot();
    void mappingAppendChangedSlot();

    void mappingDBContentVariableChangedSlot();
    void dbcontVariableCommentChangedSlot();

    void createNewDBVariableSlot(); // create new dbcontvar, and mapping if required
    void editDBVariableSlot();
    void deleteDBVariableSlot();

    void deleteMappingSlot();

signals:


public:
    explicit ASTERIXJSONParserDetailWidget(ASTERIXJSONParser& parser, QWidget* parent = nullptr);

private:
    ASTERIXJSONParser& parser_;
    bool expert_mode_ {false}; // COMPASS expert mode

    bool has_current_entry_ {false};
    ASTERIXJSONParser::EntryType entry_type_;
    unsigned int entry_index_;

    QLabel* info_label_ {nullptr}; // shows type of mapping, or missing details
    QCheckBox* active_check_ {nullptr};

    QComboBox* json_key_box_ {nullptr};
    QLabel* asterix_desc_label_ {nullptr};
    QLabel* asterix_editions_label_ {nullptr};
    QCheckBox* in_array_check_ {nullptr};
    QCheckBox* append_check {nullptr};
    UnitSelectionWidget* unit_sel_ {nullptr};
    DataTypeFormatSelectionWidget* data_format_widget_ {nullptr};

    dbContent::VariableSelectionWidget* dbcont_var_sel_ {nullptr};
    QLabel* dbcont_var_data_type_label_ {nullptr};

    QTextEdit* dbcont_var_comment_edit_ {nullptr};

    QPushButton* delete_mapping_button_ {nullptr}; // delete mapping
    QPushButton* new_dbcontvar_button_ {nullptr}; // create new dbcontvar, and mapping if required

    QPushButton* dbcontvar_edit_button_ {nullptr};
    QPushButton* dbcontvar_delete_button_ {nullptr};

    bool setting_new_content_ {false};

    void showJSONKey (const std::string& key, bool unmapped_selectable);
    void showDBContentVariable (const std::string& var_name, bool mapping_exists=false);

};
