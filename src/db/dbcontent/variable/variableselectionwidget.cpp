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

#include "dbcontent/variable/variableselectionwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableselectiondialog.h"
#include "files.h"
#include "global.h"
#include "dbcontent/variable/metavariable.h"
#include "logger.h"
#include "test/ui_test_conversions.h"

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>

#include <algorithm>

using namespace Utils;
using namespace std;

namespace dbContent
{

VariableSelectionWidget::VariableSelectionWidget(DBContentManager& dbcont_man, bool h_box, QWidget* parent)
    : QFrame(parent), dbcont_man_(dbcont_man)
{
    setFrameStyle(QFrame::Panel | QFrame::Sunken);
    setLineWidth(1);

    QBoxLayout* layout;

    object_label_ = new QLabel(this);
    variable_label_ = new QLabel(this);
    variable_label_->setAlignment(Qt::AlignRight);

    sel_button_ = new QPushButton(this);
    sel_button_->setIcon(Files::IconProvider::getIcon("expand.png"));
    sel_button_->setFixedSize(UI_ICON_SIZE);
    sel_button_->setFlat(UI_ICON_BUTTON_FLAT);

    QSizePolicy sp_retain = sel_button_->sizePolicy();
    sp_retain.setRetainSizeWhenHidden(true);
    sel_button_->setSizePolicy(sp_retain);

    if (h_box)
    {
        layout = new QHBoxLayout;
        layout->setContentsMargins(1, 1, 1, 1);
        layout->setSpacing(1);

        layout->addWidget(object_label_);
        layout->addWidget(variable_label_);

        layout->addWidget(sel_button_);
    }
    else
    {
        layout = new QVBoxLayout;
        layout->setContentsMargins(1, 1, 1, 1);
        layout->setSpacing(1);

        QHBoxLayout* select_layout = new QHBoxLayout();
        select_layout->setContentsMargins(1, 1, 1, 1);
        select_layout->setSpacing(1);

        select_layout->addWidget(object_label_);
        select_layout->addWidget(sel_button_);
        layout->addLayout(select_layout);

        layout->addWidget(variable_label_);
    }
    setLayout(layout);

    connect(sel_button_, SIGNAL(clicked()), this, SLOT(showDialogSlot()));
}

VariableSelectionWidget::~VariableSelectionWidget() {}

void VariableSelectionWidget::setReadOnly(bool read_only)
{
    traced_assert(sel_button_);

    sel_button_->setDisabled(read_only);
}

void VariableSelectionWidget::updateToolTip()
{
    if (hasVariable())
        setToolTip(selectedVariable().info().c_str());
    else if (hasMetaVariable())
        setToolTip(selectedMetaVariable().info().c_str());
    else
        setToolTip("");
}

void VariableSelectionWidget::showDialogSlot()
{
    VariableSelectionDialog::Settings dialog_settings;

    dialog_settings.show_meta_variables = show_meta_variables_;
    dialog_settings.show_meta_variables_only = show_meta_variables_only_;
    dialog_settings.show_dbcont_only = show_dbcont_only_;
    dialog_settings.only_dbcontent_name = only_dbcontent_name_;
    dialog_settings.show_data_types_only = show_data_types_only_;
    dialog_settings.only_data_types = only_data_types_;
    dialog_settings.show_existing_in_db_only = show_existing_in_db_only_;
    dialog_settings.show_empty_variable = show_empty_variable_;
    dialog_settings.multi_select = false;

    VariableSelectionDialog dialog(dbcont_man_, dialog_settings, this);

    if (dialog.exec() != QDialog::Accepted)
        return;

    if (dialog.emptySelected())
    {
        applySelection("", "");
        return;
    }

    traced_assert(dialog.selection().size() == 1);

    applySelection(dialog.selection().front().first, dialog.selection().front().second);
}

void VariableSelectionWidget::applySelection(const std::string& obj_name,
                                             const std::string& var_name)
{
    traced_assert(object_label_);
    traced_assert(variable_label_);

    if (var_name.size() == 0 && obj_name.size() == 0)
    {
        meta_variable_selected_ = false;
        variable_selected_ = false;
    }
    else
    {
        if (obj_name == META_OBJECT_NAME)
        {
            traced_assert(dbcont_man_.existsMetaVariable(var_name));

            meta_variable_selected_ = true;
            variable_selected_ = false;
        }
        else
        {
            traced_assert(dbcont_man_.dbContent(obj_name).hasVariable(var_name));

            meta_variable_selected_ = false;
            variable_selected_ = true;
        }
    }

    object_label_->setText(obj_name.c_str());
    variable_label_->setText(var_name.c_str());

    loginf << "obj " << obj_name.c_str() << " var "
           << var_name.c_str();

    updateToolTip();

    emit selectionChanged();
}

void VariableSelectionWidget::selectedVariable(Variable& variable)
{
    traced_assert(object_label_);
    traced_assert(variable_label_);

    object_label_->setText(QString::fromStdString(variable.dbContent().name()));
    variable_label_->setText(variable.name().c_str());

    variable_selected_ = true;
    meta_variable_selected_ = false;

    updateToolTip();
}

void VariableSelectionWidget::selectEmptyVariable()
{
    traced_assert(show_empty_variable_);

    traced_assert(object_label_);
    traced_assert(variable_label_);

    object_label_->setText("");
    variable_label_->setText("");

    variable_selected_ = false;
    meta_variable_selected_ = false;

    updateToolTip();
}

Variable& VariableSelectionWidget::selectedVariable() const
{
    traced_assert(object_label_);
    traced_assert(variable_label_);
    traced_assert(variable_selected_);

    std::string obj_name = object_label_->text().toStdString();
    std::string var_name = variable_label_->text().toStdString();

    traced_assert(dbcont_man_.dbContent(obj_name).hasVariable(var_name));

    return dbcont_man_.dbContent(obj_name).variable(var_name);
}

void VariableSelectionWidget::selectedMetaVariable(MetaVariable& variable)
{
    traced_assert(object_label_);
    traced_assert(variable_label_);

    object_label_->setText(QString::fromStdString(META_OBJECT_NAME));
    variable_label_->setText(variable.name().c_str());

    variable_selected_ = false;
    meta_variable_selected_ = true;

    updateToolTip();
}

MetaVariable& VariableSelectionWidget::selectedMetaVariable() const
{
    traced_assert(object_label_);
    traced_assert(variable_label_);
    traced_assert(meta_variable_selected_);

    std::string obj_name = object_label_->text().toStdString();
    std::string var_name = variable_label_->text().toStdString();

    traced_assert(obj_name == META_OBJECT_NAME);
    traced_assert(dbcont_man_.existsMetaVariable(var_name));

    return dbcont_man_.metaVariable(var_name);
}

std::pair<std::string, std::string> VariableSelectionWidget::selectionAsString() const
{
    traced_assert(object_label_);
    traced_assert(variable_label_);

    std::string obj_name = object_label_->text().toStdString();
    std::string var_name = variable_label_->text().toStdString();

    return std::make_pair(obj_name, var_name);
}

void VariableSelectionWidget::showDBContentOnly(const std::string& only_dbcontent_name)
{
    show_dbcont_only_ = true;
    only_dbcontent_name_ = only_dbcontent_name;

    traced_assert(object_label_);
    object_label_->hide();
}

void VariableSelectionWidget::disableShowDBContentOnly()
{
    show_dbcont_only_ = false;
    only_dbcontent_name_ = "";

    traced_assert(object_label_);
    object_label_->show();
}

std::string VariableSelectionWidget::onlyDBContentName() const { return only_dbcontent_name_; }

bool VariableSelectionWidget::showEmptyVariable() const { return show_empty_variable_; }

void VariableSelectionWidget::showEmptyVariable(bool show_empty_variable)
{
    show_empty_variable_ = show_empty_variable;
}

void VariableSelectionWidget::showDataTypesOnly(const std::vector<PropertyDataType>& only_data_types)
{
    only_data_types_ = only_data_types;
    show_data_types_only_ = true;
}

void VariableSelectionWidget::disableShowDataTypesOnly()
{
    show_data_types_only_ = false;
}

bool VariableSelectionWidget::showMetaVariablesOnly() const { return show_meta_variables_only_; }

void VariableSelectionWidget::showMetaVariablesOnly(bool show_meta_variables_only)
{
    show_meta_variables_only_ = show_meta_variables_only;

    if (show_meta_variables_only_)
        show_meta_variables_ = true;
}

bool VariableSelectionWidget::showMetaVariables() const { return show_meta_variables_; }

void VariableSelectionWidget::showMetaVariables(bool show_meta_variables)
{
    show_meta_variables_ = show_meta_variables;
}

bool VariableSelectionWidget::showExistingInDBOnly() const
{
    return show_existing_in_db_only_;
}

void VariableSelectionWidget::showExistingInDBOnly(bool show_existing_only)
{
    show_existing_in_db_only_ = show_existing_only;
}

boost::optional<QString> VariableSelectionWidget::uiGet(const QString& what) const
{
    QString obj_str = object_label_->text();
    QString var_str = variable_label_->text();

    QStringList strings;
    strings.push_back(obj_str);
    strings.push_back(var_str);

    return ui_test::conversions::stringFromValue<QStringList>(strings);
}

bool VariableSelectionWidget::uiSet(const QString& str)
{
    // value as used with the former menu-based selection, e.g. "Meta|Latitude",
    // "CAT048|Time of Day", a plain variable name in single-content mode, or
    // an empty string to clear the selection (if enabled)

    auto strings = ui_test::conversions::valueFromString<QStringList>(str);

    if (!strings.has_value() || strings.value().empty())
    {
        if (!show_empty_variable_)
            return false;

        applySelection("", "");
        return true;
    }

    const QStringList& parts = strings.value();

    std::string obj_name;
    std::string var_name;

    if (parts.size() == 1 && show_dbcont_only_)
    {
        obj_name = only_dbcontent_name_;
        var_name = parts.at(0).toStdString();
    }
    else if (parts.size() == 2)
    {
        obj_name = parts.at(0).toStdString();
        var_name = parts.at(1).toStdString();
    }
    else
        return false;

    if (obj_name == META_OBJECT_NAME)
    {
        if (!dbcont_man_.existsMetaVariable(var_name))
            return false;
    }
    else
    {
        if (!dbcont_man_.existsDBContent(obj_name) ||
            !dbcont_man_.dbContent(obj_name).hasVariable(var_name))
            return false;
    }

    applySelection(obj_name, var_name);

    return true;
}

QWidget* VariableSelectionWidget::uiRerouteToNative() const
{
    //selection button triggers the selection dialog and can be handled by native qt ui injections.
    return sel_button_;
}

}

