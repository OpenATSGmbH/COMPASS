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

#include "dbcontent/variable/variableeditdialog.h"
#include "compass.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variabledatatypecombobox.h"
#include "stringrepresentationcombobox.h"
#include "unitselectionwidget.h"
//#include "stringconv.h"
#include "dbcontent/dbcontent.h"
#include "logger.h"

#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QListWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QTextEdit>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>

#include <json.hpp>

using namespace std;

namespace dbContent
{

VariableEditDialog::VariableEditDialog(Variable& variable, QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f), variable_(variable)
{
    expert_mode_ = variable_.dbContent().compass().expertMode();

    setWindowFlags(Qt::Window | Qt::WindowTitleHint); //  | Qt::CustomizeWindowHint

    if (expert_mode_)
        setWindowTitle("Edit DBContent Variable");
    else
        setWindowTitle("Show DBContent Variable");

    setModal(true);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* form_layout = new QFormLayout;
    form_layout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);

    //    QLineEdit* name_edit_ {nullptr};

    name_edit_ = new QLineEdit(variable_.name().c_str());
    name_edit_->setReadOnly(!expert_mode_);
    connect(name_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::nameChangedSlot);
    form_layout->addRow("Name", name_edit_);

    //    QLineEdit* short_name_edit_ {nullptr};
    short_name_edit_ = new QLineEdit(variable_.shortName().c_str());
    short_name_edit_->setReadOnly(!expert_mode_);
    connect(short_name_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::shortNameChangedSlot);
    form_layout->addRow("Short Name", short_name_edit_);

    //    QTextEdit* description_edit_ {nullptr};
    description_edit_ = new QTextEdit();
    description_edit_->document()->setPlainText(variable_.description().c_str());
    short_name_edit_->setReadOnly(!expert_mode_);
    description_edit_->setWordWrapMode(QTextOption::WrapMode::WrapAnywhere);

    connect(description_edit_, &QTextEdit::textChanged, this,
            &VariableEditDialog::commentChangedSlot);

    form_layout->addRow("Comment", description_edit_);

    // origin model, see readme_dbcontent.md: editable in expert mode, a change of the
    // candidates applies to the next import

    origin_combo_ = new QComboBox();
    for (const auto& origin : {Variable::OriginAsterix, Variable::OriginCalculated,
                               Variable::OriginInternal, Variable::OriginSelection})
        origin_combo_->addItem(origin.c_str());
    origin_combo_->setCurrentText(variable_.origin().c_str());
    origin_combo_->setEnabled(expert_mode_);
    origin_combo_->setToolTip(variable_.info().c_str());
    connect(origin_combo_, &QComboBox::currentTextChanged, this, &VariableEditDialog::originChangedSlot);
    form_layout->addRow("Origin", origin_combo_);

    data_item_edit_ = new QLineEdit(variable_.dataItem().c_str());
    data_item_edit_->setReadOnly(!expert_mode_);
    data_item_edit_->setToolTip("ASTERIX item in EUROCONTROL notation, I021/071.Time of Applicability for Position");
    connect(data_item_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::dataItemChangedSlot);
    form_layout->addRow("Data Item", data_item_edit_);

    inputs_edit_ = new QLineEdit(boost::algorithm::join(variable_.inputs(), ", ").c_str());
    inputs_edit_->setReadOnly(!expert_mode_);
    inputs_edit_->setToolTip("Variables of this content the value is calculated from, comma separated");
    connect(inputs_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::inputsChangedSlot);
    form_layout->addRow("Inputs", inputs_edit_);

    {
        QWidget* candidates_widget = new QWidget();
        QHBoxLayout* candidates_layout = new QHBoxLayout();
        candidates_layout->setContentsMargins(0, 0, 0, 0);

        candidates_list_ = new QListWidget();
        candidates_list_->setToolTip("Candidates in priority order, the first with a value supplies the record. "
                                     "A change applies to the next import.");
        candidates_layout->addWidget(candidates_list_);

        QVBoxLayout* buttons_layout = new QVBoxLayout();

        candidate_up_button_ = new QPushButton("Up");
        candidate_up_button_->setToolTip("Move the selected candidate up in the priority order");
        connect(candidate_up_button_, &QPushButton::clicked, this, &VariableEditDialog::moveCandidateUpSlot);
        buttons_layout->addWidget(candidate_up_button_);

        candidate_down_button_ = new QPushButton("Down");
        candidate_down_button_->setToolTip("Move the selected candidate down in the priority order");
        connect(candidate_down_button_, &QPushButton::clicked, this, &VariableEditDialog::moveCandidateDownSlot);
        buttons_layout->addWidget(candidate_down_button_);

        candidate_remove_button_ = new QPushButton("Remove");
        candidate_remove_button_->setToolTip("Remove the selected candidate");
        connect(candidate_remove_button_, &QPushButton::clicked, this, &VariableEditDialog::removeCandidateSlot);
        buttons_layout->addWidget(candidate_remove_button_);

        buttons_layout->addStretch();
        candidates_layout->addLayout(buttons_layout);
        candidates_widget->setLayout(candidates_layout);

        form_layout->addRow("Candidates", candidates_widget);
    }

    {
        QWidget* add_widget = new QWidget();
        QHBoxLayout* add_layout = new QHBoxLayout();
        add_layout->setContentsMargins(0, 0, 0, 0);

        candidate_add_combo_ = new QComboBox();
        candidate_add_combo_->setToolTip("Variable of this content to add as the last candidate");
        for (const auto& var_it : variable_.object().variables())
            if (var_it.first != variable_.name())
                candidate_add_combo_->addItem(var_it.first.c_str());
        add_layout->addWidget(candidate_add_combo_);

        candidate_add_button_ = new QPushButton("Add");
        connect(candidate_add_button_, &QPushButton::clicked, this, &VariableEditDialog::addCandidateSlot);
        add_layout->addWidget(candidate_add_button_);

        add_widget->setLayout(add_layout);
        form_layout->addRow("Add Candidate", add_widget);
    }

    source_variable_edit_ = new QLineEdit(variable_.sourceVariableName().c_str());
    source_variable_edit_->setReadOnly(!expert_mode_);
    source_variable_edit_->setToolTip("UCHAR variable of this content holding the 1-based index of the chosen candidate");
    connect(source_variable_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::sourceVariableChangedSlot);
    form_layout->addRow("Source Variable", source_variable_edit_);

    updateCandidates();
    updateOriginWidgets();

    //    VariableDataTypeComboBox* type_combo_ {nullptr};
    type_combo_ = new VariableDataTypeComboBox(variable_.dataTypeRef(), variable_.dataTypeStringRef());
    type_combo_->setEnabled(expert_mode_);
    form_layout->addRow("Data Type", type_combo_);

    //    UnitSelectionWidget* unit_sel_ {nullptr};
    unit_sel_ = new UnitSelectionWidget(variable_.dbContent().compass().unitManager(), variable_.dimension(), variable_.unit());
    unit_sel_->setEnabled(expert_mode_);
    form_layout->addRow("Unit", unit_sel_);

    //    StringRepresentationComboBox* representation_box_ {nullptr};
    representation_box_ = new StringRepresentationComboBox(variable_.representationRef(),
                                                           variable_.representationStringRef());
    representation_box_->setEnabled(expert_mode_);
    form_layout->addRow("Representation", representation_box_);

    //    QLineEdit* db_column_edit_ {nullptr};
    db_column_edit_ = new QLineEdit(variable_.dbColumnName().c_str());
    db_column_edit_->setReadOnly(!expert_mode_);
    connect(db_column_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::dbColumnChangedSlot);
    form_layout->addRow("DBColumn", db_column_edit_);

    main_layout->addLayout(form_layout);

    done_button_ = new QPushButton("Done");
    connect(done_button_, &QPushButton::clicked, this, &VariableEditDialog::doneSlot);
    main_layout->addWidget(done_button_);

    setLayout(main_layout);
}

bool VariableEditDialog::variableEdited() const
{
    return variable_edited_;
}

Variable &VariableEditDialog::variable() const
{
    return variable_;
}

void VariableEditDialog::nameChangedSlot(const QString& name)
{
    loginf << "name '" << name.trimmed().toStdString() << "'";

    traced_assert(name_edit_);
    string new_name = name.trimmed().toStdString();

    if (new_name == variable_.name())
        return;

    if (!new_name.size())
    {
        name_edit_->setStyleSheet(variable_.dbContent().compass().lineEditInvalidStyle());
        return;
    }

    if (variable_.object().hasVariable(new_name))
    {
        logwrn << "name '" << new_name << "' already in use";

        name_edit_->setStyleSheet(variable_.dbContent().compass().lineEditInvalidStyle());
        name_edit_->setToolTip(("Variable name '"+new_name+"' already in use").c_str());
        return;
    }

    // ok, rename

    name_edit_->setStyleSheet("");
    name_edit_->setToolTip("");

    loginf << "renaming '" << variable_.name() << "' to '" << new_name << "'";
    variable_.object().renameVariable(std::string(variable_.name()), new_name); // copy, otherwise wrong

    if (new_name.size())
    {
        string db_column_name = name.toStdString();

        std::replace_if(db_column_name.begin(), db_column_name.end(), [](char ch) {
                return !(isalnum(ch) || ch == '_');
            }, '_');

        boost::algorithm::to_lower(db_column_name);

        db_column_edit_->setText(db_column_name.c_str());
        dbColumnChangedSlot(db_column_name.c_str());
    }

    variable_edited_ = true;
}

void VariableEditDialog::shortNameChangedSlot(const QString& name)
{
    loginf << "name '" << name.trimmed().toStdString() << "'";

    std::string short_name = name.trimmed().toStdString();

    // at most 10 characters, see readme_dbcontent.md
    if (short_name.size() > 10)
    {
        short_name_edit_->setStyleSheet(variable_.dbContent().compass().lineEditInvalidStyle());
        short_name_edit_->setToolTip("A short name has at most 10 characters");
        return;
    }

    short_name_edit_->setStyleSheet("");
    short_name_edit_->setToolTip("");

    variable_.shortName(short_name);

    variable_edited_ = true;
}

void VariableEditDialog::originChangedSlot(const QString& origin)
{
    variable_.origin(origin.toStdString());

    updateOriginWidgets();

    variable_edited_ = true;
}

void VariableEditDialog::dataItemChangedSlot(const QString& data_item)
{
    variable_.dataItem(data_item.trimmed().toStdString());

    variable_edited_ = true;
}

void VariableEditDialog::inputsChangedSlot(const QString& inputs)
{
    std::vector<std::string> input_names;

    for (const QString& part : inputs.split(','))
    {
        std::string input_name = part.trimmed().toStdString();

        if (input_name.size())
            input_names.push_back(input_name);
    }

    variable_.inputs(input_names);

    variable_edited_ = true;
}

void VariableEditDialog::sourceVariableChangedSlot(const QString& name)
{
    variable_.sourceVariableName(name.trimmed().toStdString());

    variable_edited_ = true;
}

void VariableEditDialog::moveCandidateUpSlot()
{
    int row = candidates_list_->currentRow();

    if (row > 0)
        moveCandidate(row, row - 1);
}

void VariableEditDialog::moveCandidateDownSlot()
{
    int row = candidates_list_->currentRow();

    if (row >= 0 && row + 1 < candidates_list_->count())
        moveCandidate(row, row + 1);
}

void VariableEditDialog::moveCandidate(int from, int to)
{
    nlohmann::json selection = variable_.selection();

    traced_assert(from >= 0 && from < (int) selection.size());
    traced_assert(to >= 0 && to < (int) selection.size());

    nlohmann::json candidate = selection.at(from);
    selection.erase(from);
    selection.insert(selection.begin() + to, candidate);

    variable_.selection(selection);

    updateCandidates(to);

    variable_edited_ = true;
}

void VariableEditDialog::removeCandidateSlot()
{
    int row = candidates_list_->currentRow();

    nlohmann::json selection = variable_.selection();

    if (row < 0 || row >= (int) selection.size())
        return;

    selection.erase(row);
    variable_.selection(selection);

    updateCandidates(std::min(row, (int) selection.size() - 1));

    variable_edited_ = true;
}

void VariableEditDialog::addCandidateSlot()
{
    std::string var_name = candidate_add_combo_->currentText().toStdString();

    if (var_name.empty() || !variable_.object().hasVariable(var_name))
        return;

    nlohmann::json selection = variable_.selection();
    selection.push_back({{"variable", var_name}});
    variable_.selection(selection);

    updateCandidates(selection.size() - 1);

    variable_edited_ = true;
}

void VariableEditDialog::updateOriginWidgets()
{
    bool is_asterix = variable_.isAsterix();
    bool is_calculated = variable_.isCalculated();
    bool is_selection = variable_.isSelection();

    data_item_edit_->setEnabled(is_asterix);
    inputs_edit_->setEnabled(is_calculated);

    candidates_list_->setEnabled(is_selection);
    candidate_up_button_->setEnabled(is_selection && expert_mode_);
    candidate_down_button_->setEnabled(is_selection && expert_mode_);
    candidate_remove_button_->setEnabled(is_selection && expert_mode_);
    candidate_add_combo_->setEnabled(is_selection && expert_mode_);
    candidate_add_button_->setEnabled(is_selection && expert_mode_);
    source_variable_edit_->setEnabled(is_selection);
}

void VariableEditDialog::updateCandidates(int current_row)
{
    candidates_list_->clear();

    const nlohmann::json& selection = variable_.selection();

    for (unsigned int index = 1; index <= selection.size(); ++index)
    {
        const nlohmann::json& candidate = selection.at(index - 1);

        std::string text = std::to_string(index) + ": " + variable_.selectionCandidateLabel(index);

        if (candidate.is_object() && candidate.contains("variable") && candidate.at("variable").is_string())
        {
            std::string var_name = candidate.at("variable");

            if (var_name != variable_.selectionCandidateLabel(index))
                text += " " + var_name;

            for (const auto& key : {"require_true", "require_false"})
            {
                if (!candidate.contains(key) || !candidate.at(key).is_array())
                    continue;

                for (const auto& flag : candidate.at(key))
                    if (flag.is_string())
                        text += std::string(", requires ") + flag.get<std::string>()
                                + (std::string(key) == "require_true" ? " true" : " false");
            }
        }
        else if (candidate.is_object() && candidate.contains("calculation") && candidate.at("calculation").is_string())
            text += ", calculation " + candidate.at("calculation").get<std::string>();

        candidates_list_->addItem(text.c_str());
    }

    if (current_row >= 0 && current_row < candidates_list_->count())
        candidates_list_->setCurrentRow(current_row);
}

void VariableEditDialog::commentChangedSlot()
{
    traced_assert(description_edit_);

    variable_.description(description_edit_->document()->toPlainText().trimmed().toStdString());

    variable_edited_ = true;
}

void VariableEditDialog::dbColumnChangedSlot(const QString& name)
{
    traced_assert(db_column_edit_);
    string new_name = name.trimmed().toStdString();

    if (new_name == variable_.dbColumnName())
        return;

    if (!new_name.size())
    {
        db_column_edit_->setStyleSheet(variable_.dbContent().compass().lineEditInvalidStyle());
        return;
    }

    if (variable_.object().hasVariableDBColumnName(new_name))
    {
        logwrn << "name '" << new_name << "' already in use";

        db_column_edit_->setStyleSheet(variable_.dbContent().compass().lineEditInvalidStyle());
        db_column_edit_->setToolTip(("Variable DB Column name '"+new_name+"' already in use").c_str());
        return;
    }

    db_column_edit_->setStyleSheet("");
    db_column_edit_->setToolTip("");

    loginf << "changing '" << variable_.dbColumnName()
           << "' to '" << new_name << "'";
    variable_.dbColumnName(new_name);

    variable_edited_ = true;
}

void VariableEditDialog::doneSlot()
{
    loginf;

    accept();
}

}

