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

#include "dbcontent/variable/variableselectiondialog.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "files.h"
#include "global.h"
#include "logger.h"
#include "popupmenu.h"
#include "traced_assert.h"

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSortFilterProxyModel>
#include <QStandardItemModel>
#include <QTreeView>
#include <QVBoxLayout>

#include <algorithm>

using namespace Utils;
using namespace std;

namespace dbContent
{

namespace
{
    // fixed display order of variable groups; unknown groups sort after these, "Other" last
    const std::vector<std::string> GroupOrder = {
        "Origin", "Type", "Time", "Position", "Accuracy", "Movement",
        "Secondary Identification", "Altitude", "Track", "Ages", "Warning/Alert"};

    const std::string OtherGroupName = "Other";
    const std::string InternalItemName = "Internal/Calculated";

    const int ContentNameRole  = Qt::UserRole + 1;
    const int VariableNameRole = Qt::UserRole + 2;

    const int ContentStripColumns = 6;

    int groupOrderIndex(const std::string& group)
    {
        auto it = std::find(GroupOrder.begin(), GroupOrder.end(), group);
        if (it != GroupOrder.end())
            return (int)std::distance(GroupOrder.begin(), it);
        if (group == OtherGroupName)
            return (int)GroupOrder.size() + 1;
        return (int)GroupOrder.size(); // unknown groups between known ones and Other
    }
}

/**
 */
VariableSelectionDialog::VariableSelectionDialog(DBContentManager& dbcont_man,
                                                 const Settings& settings, QWidget* parent)
    : QDialog(parent), dbcont_man_(dbcont_man), settings_(settings)
{
    setObjectName("variable_selection_dialog");
    setWindowTitle(settings_.multi_select ? "Select Variables" : "Select Variable");
    setMinimumSize(700, 600);

    createUI();
    updateModel();
    updateSelectButton();
}

VariableSelectionDialog::~VariableSelectionDialog() = default;

/**
 */
void VariableSelectionDialog::createUI()
{
    QVBoxLayout* main_layout = new QVBoxLayout();
    setLayout(main_layout);

    // search + grouping row
    {
        QHBoxLayout* search_layout = new QHBoxLayout();

        search_layout->addWidget(new QLabel("Search"));

        search_edit_ = new QLineEdit();
        search_edit_->setObjectName("search_edit");
        search_edit_->setClearButtonEnabled(true);
        search_edit_->setToolTip("Filter variables by name, ASTERIX data item or description");
        connect(search_edit_, &QLineEdit::textChanged,
                this, &VariableSelectionDialog::searchChangedSlot);
        search_layout->addWidget(search_edit_, 1);

        search_layout->addWidget(new QLabel("Grouping"));

        grouping_combo_ = new QComboBox();
        grouping_combo_->setObjectName("grouping_combo");
        grouping_combo_->addItem("Data Item");
        grouping_combo_->addItem("Group");
        grouping_combo_->setToolTip("Group variables by ASTERIX data item or by variable group");
        grouping_combo_->setCurrentIndex(dbcont_man_.variableSelectionByGroup() ? 1 : 0);
        connect(grouping_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &VariableSelectionDialog::groupingChangedSlot);
        search_layout->addWidget(grouping_combo_);

        main_layout->addLayout(search_layout);
    }

    // content selection strip with gear menu button on the right
    if (!settings_.show_dbcont_only)
        createContentStrip();

    // variable tree
    {
        model_ = new QStandardItemModel(this);

        proxy_model_ = new QSortFilterProxyModel(this);
        proxy_model_->setSourceModel(model_);
        proxy_model_->setFilterCaseSensitivity(Qt::CaseInsensitive);
        proxy_model_->setFilterKeyColumn(-1); // match any column
        proxy_model_->setRecursiveFilteringEnabled(true);

        tree_view_ = new QTreeView();
        tree_view_->setObjectName("variable_tree");
        tree_view_->setModel(proxy_model_);
        tree_view_->setSelectionMode(settings_.multi_select ? QAbstractItemView::ExtendedSelection
                                                            : QAbstractItemView::SingleSelection);
        tree_view_->setSelectionBehavior(QAbstractItemView::SelectRows);
        tree_view_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        tree_view_->setUniformRowHeights(true);

        connect(tree_view_->selectionModel(), &QItemSelectionModel::selectionChanged,
                this, &VariableSelectionDialog::selectionChangedSlot);
        connect(tree_view_, &QTreeView::doubleClicked,
                this, &VariableSelectionDialog::itemDoubleClickedSlot);

        main_layout->addWidget(tree_view_, 1);
    }

    // button row
    {
        QHBoxLayout* button_layout = new QHBoxLayout();

        cancel_button_ = new QPushButton("Cancel");
        cancel_button_->setObjectName("cancel_button");
        cancel_button_->setIcon(QIcon());
        cancel_button_->setToolTip("Close without changing the selection");
        connect(cancel_button_, &QPushButton::clicked, this, &QDialog::reject);
        button_layout->addWidget(cancel_button_);

        if (settings_.show_empty_variable)
        {
            select_none_button_ = new QPushButton("Select None");
            select_none_button_->setObjectName("select_none_button");
            select_none_button_->setIcon(QIcon());
            select_none_button_->setToolTip("Clear the variable selection");
            connect(select_none_button_, &QPushButton::clicked,
                    this, &VariableSelectionDialog::selectNoneSlot);
            button_layout->addWidget(select_none_button_);
        }

        button_layout->addStretch(1);

        select_button_ = new QPushButton("Select");
        select_button_->setObjectName("select_button");
        select_button_->setIcon(QIcon());
        select_button_->setToolTip(settings_.multi_select ? "Use the selected variables"
                                                          : "Use the selected variable");
        connect(select_button_, &QPushButton::clicked,
                this, &VariableSelectionDialog::selectSlot);
        button_layout->addWidget(select_button_);

        main_layout->addLayout(button_layout);
    }
}

/**
 */
void VariableSelectionDialog::createContentStrip()
{
    QVBoxLayout* main_layout = static_cast<QVBoxLayout*>(layout());

    QHBoxLayout* strip_layout = new QHBoxLayout();

    content_strip_layout_ = new QGridLayout();
    content_strip_layout_->setContentsMargins(0, 0, 0, 0);

    std::set<std::string> hidden = dbcont_man_.variableSelectionHiddenContents();

    std::vector<std::string> contents;

    if (settings_.show_meta_variables)
        contents.push_back(META_OBJECT_NAME);

    if (!settings_.show_meta_variables_only)
    {
        for (auto& object_it : dbcont_man_)
        {
            if (settings_.show_existing_in_db_only && !object_it.second->hasData())
                continue;

            contents.push_back(object_it.first);
        }
    }

    unsigned int cnt = 0;

    for (const auto& content_name : contents)
    {
        QCheckBox* checkbox = new QCheckBox(QString::fromStdString(content_name));
        checkbox->setObjectName(QString::fromStdString("content_" + content_name));
        checkbox->setChecked(!hidden.count(content_name));
        connect(checkbox, &QCheckBox::toggled,
                this, &VariableSelectionDialog::contentToggledSlot);

        content_checkboxes_[content_name] = checkbox;

        content_strip_layout_->addWidget(checkbox, cnt / ContentStripColumns,
                                         cnt % ContentStripColumns);
        ++cnt;
    }

    strip_layout->addLayout(content_strip_layout_, 1);

    // gear button with content selection menu, right side
    content_menu_button_ = new QPushButton();
    content_menu_button_->setObjectName("content_menu_button");
    content_menu_button_->setStyleSheet("QPushButton::menu-indicator { image: none; }");
    content_menu_button_->setIcon(Files::IconProvider::getIcon("edit.png"));
    content_menu_button_->setFixedSize(UI_ICON_SIZE);
    content_menu_button_->setFlat(UI_ICON_BUTTON_FLAT);
    content_menu_button_->setToolTip("Content selection");

    content_menu_ = new PopupMenu(content_menu_button_);
    content_menu_->setToolTipsVisible(true);

    QAction* all_action = content_menu_->addAction("Select All");
    all_action->setToolTip("Show variables of all contents");
    connect(all_action, &QAction::triggered, this, [this]() { setAllContentsChecked(true); });

    QAction* none_action = content_menu_->addAction("Select Nothing");
    none_action->setToolTip("Hide variables of all contents");
    connect(none_action, &QAction::triggered, this, [this]() { setAllContentsChecked(false); });

    content_menu_->addSeparator();

    QAction* target_action = content_menu_->addAction("Target Reports Only");
    target_action->setToolTip("Show only contents containing target reports");
    connect(target_action, &QAction::triggered, this, [this]() { checkContentsByRole(true); });

    QAction* status_action = content_menu_->addAction("Status Messages Only");
    status_action->setToolTip("Show only contents containing status messages");
    connect(status_action, &QAction::triggered, this, [this]() { checkContentsByRole(false); });

    strip_layout->addWidget(content_menu_button_, 0, Qt::AlignTop | Qt::AlignRight);

    static_cast<QVBoxLayout*>(main_layout)->addLayout(strip_layout);
}

/**
 */
bool VariableSelectionDialog::showDataType(PropertyDataType type) const
{
    if (!settings_.show_data_types_only)
        return true;

    return std::find(settings_.only_data_types.begin(), settings_.only_data_types.end(), type) !=
           settings_.only_data_types.end();
}

/**
 */
bool VariableSelectionDialog::contentChecked(const std::string& content_name) const
{
    auto it = content_checkboxes_.find(content_name);
    if (it == content_checkboxes_.end())
        return true; // no strip (single content mode) => always shown

    return it->second->isChecked();
}

/**
 */
void VariableSelectionDialog::setAllContentsChecked(bool checked)
{
    for (auto& check_it : content_checkboxes_)
    {
        check_it.second->blockSignals(true);
        check_it.second->setChecked(checked);
        check_it.second->blockSignals(false);
    }

    contentToggledSlot();
}

/**
 */
void VariableSelectionDialog::checkContentsByRole(bool target_reports)
{
    for (auto& check_it : content_checkboxes_)
    {
        bool check;

        if (check_it.first == META_OBJECT_NAME)
            check = target_reports; // meta variables mainly resolve to target report contents
        else
        {
            const DBContent& content = dbcont_man_.dbContent(check_it.first);
            check = target_reports ? content.containsTargetReports()
                                   : content.containsStatusContent();
        }

        check_it.second->blockSignals(true);
        check_it.second->setChecked(check);
        check_it.second->blockSignals(false);
    }

    contentToggledSlot();
}

/**
 */
std::string VariableSelectionDialog::dataItemOf(const std::string& source) const
{
    if (source.empty())
        return "";

    QString first_line = QString::fromStdString(source).section('\n', 0, 0);

    static const QRegularExpression item_regex("^(I\\d{3}/[A-Za-z0-9]+)");

    auto match = item_regex.match(first_line);
    if (match.hasMatch())
        return match.captured(1).toStdString();

    return "";
}

/**
 */
QStandardItem* VariableSelectionDialog::makeVariableRow(const std::string& content_name,
                                                        const std::string& var_name,
                                                        const Variable* variable,
                                                        const MetaVariable* meta_variable,
                                                        QList<QStandardItem*>& row) const
{
    traced_assert(variable || meta_variable);

    QStandardItem* name_item = new QStandardItem(QString::fromStdString(var_name));
    name_item->setData(QString::fromStdString(content_name), ContentNameRole);
    name_item->setData(QString::fromStdString(var_name), VariableNameRole);

    std::string source = variable ? variable->source() : "";
    std::string type_str = variable ? variable->dataTypeString()
                                    : (meta_variable->hasVariables() ? meta_variable->dataTypeString() : "");
    std::string description = variable ? variable->description() : meta_variable->description();
    std::string info = variable ? variable->info() : meta_variable->info();

    QStandardItem* item_item = new QStandardItem(
        QString::fromStdString(source).section('\n', 0, 0).simplified());
    QStandardItem* type_item = new QStandardItem(QString::fromStdString(type_str));
    QStandardItem* descr_item = new QStandardItem(QString::fromStdString(description).simplified());

    row << name_item << item_item << type_item << descr_item;

    bool has_dbc = variable ? variable->hasDBContent() : meta_variable->hasDBContent();

    QFont font_italic;
    font_italic.setItalic(true);
    font_italic.setWeight(QFont::Light);

    for (QStandardItem* item : row)
    {
        item->setToolTip(QString::fromStdString(info));
        item->setEditable(false);

        if (!has_dbc)
            item->setFont(font_italic);
    }

    if (!has_dbc)
        name_item->setIcon(Files::IconProvider::getIcon("db_empty.png"));

    return name_item;
}

/**
 */
void VariableSelectionDialog::addVariableRows(QStandardItem* content_item,
                                              const std::string& content_name,
                                              bool group_content_by_group)
{
    // collect group node name -> variable rows, then append in display order
    std::map<std::string, std::vector<QList<QStandardItem*>>> groups;

    auto add_variable = [&](const std::string& var_name, const Variable* variable,
                            const MetaVariable* meta_variable)
    {
        std::string group_name;

        if (group_content_by_group)
        {
            group_name = variable ? variable->group() : meta_variable->group();
            if (group_name.empty())
                group_name = OtherGroupName;
        }
        else
        {
            group_name = dataItemOf(variable ? variable->source() : "");
            if (group_name.empty())
                group_name = InternalItemName;
        }

        QList<QStandardItem*> row;
        makeVariableRow(content_name, var_name, variable, meta_variable, row);
        groups[group_name].push_back(row);
    };

    if (content_name == META_OBJECT_NAME)
    {
        for (auto& meta_it : dbcont_man_.metaVariables())
        {
            if (meta_it.second->hasVariables() && !showDataType(meta_it.second->dataType()))
                continue;

            if (settings_.show_existing_in_db_only && !meta_it.second->hasDBContent())
                continue;

            add_variable(meta_it.first, nullptr, meta_it.second.get());
        }
    }
    else
    {
        traced_assert(dbcont_man_.existsDBContent(content_name));

        for (auto& var_it : dbcont_man_.dbContent(content_name).variables())
        {
            if (!showDataType(var_it.second->dataType()))
                continue;

            if (settings_.show_existing_in_db_only && !var_it.second->hasDBContent())
                continue;

            add_variable(var_it.first, var_it.second.get(), nullptr);
        }
    }

    // display order: group mode uses the fixed taxonomy order, data item mode is
    // alphabetical with the internal/calculated bucket last
    std::vector<std::string> group_names;
    for (const auto& group_it : groups)
        group_names.push_back(group_it.first);

    if (group_content_by_group)
        std::stable_sort(group_names.begin(), group_names.end(),
                         [](const std::string& a, const std::string& b)
                         {
                             int ia = groupOrderIndex(a), ib = groupOrderIndex(b);
                             if (ia != ib)
                                 return ia < ib;
                             return a < b;
                         });
    else
        std::stable_sort(group_names.begin(), group_names.end(),
                         [](const std::string& a, const std::string& b)
                         {
                             if ((a == InternalItemName) != (b == InternalItemName))
                                 return b == InternalItemName;
                             return a < b;
                         });

    QFont font_bold;
    font_bold.setBold(true);

    for (const auto& group_name : group_names)
    {
        std::string label = group_name;

        if (!group_content_by_group)
        {
            std::string title = dataItemTitle(group_name);
            if (!title.empty())
                label += " " + title;
        }

        QStandardItem* group_item = new QStandardItem(QString::fromStdString(label));
        group_item->setFont(font_bold);
        group_item->setEditable(false);
        group_item->setSelectable(false);

        QList<QStandardItem*> group_row;
        group_row << group_item << new QStandardItem() << new QStandardItem()
                  << new QStandardItem();

        for (int i = 1; i < group_row.size(); ++i)
        {
            group_row[i]->setEditable(false);
            group_row[i]->setSelectable(false);
        }

        for (auto& row : groups.at(group_name))
            group_item->appendRow(row);

        if (content_item)
            content_item->appendRow(group_row);
        else
            model_->appendRow(group_row);
    }
}

/**
 */
void VariableSelectionDialog::updateModel()
{
    model_->clear();
    model_->setHorizontalHeaderLabels({"Name", "Data Item", "Type", "Description"});

    bool by_group = grouping_combo_->currentIndex() == 1;

    if (settings_.show_dbcont_only)
    {
        addVariableRows(nullptr, settings_.only_dbcontent_name, by_group);
    }
    else
    {
        QFont font_italic;
        font_italic.setItalic(true);
        font_italic.setWeight(QFont::Light);

        auto add_content = [&](const std::string& content_name, bool has_data,
                               bool group_by_group)
        {
            if (!contentChecked(content_name))
                return;

            QStandardItem* content_item =
                new QStandardItem(QString::fromStdString(content_name));
            content_item->setEditable(false);
            content_item->setSelectable(false);

            QFont font_content = content_item->font();
            font_content.setBold(true);
            content_item->setFont(font_content);

            if (!has_data)
            {
                content_item->setFont(font_italic);
                content_item->setIcon(Files::IconProvider::getIcon("db_empty.png"));
            }

            QList<QStandardItem*> content_row;
            content_row << content_item << new QStandardItem() << new QStandardItem()
                        << new QStandardItem();

            for (int i = 1; i < content_row.size(); ++i)
            {
                content_row[i]->setEditable(false);
                content_row[i]->setSelectable(false);
            }

            addVariableRows(content_item, content_name, group_by_group);

            model_->appendRow(content_row);
        };

        // meta variables first, always grouped by group
        if (settings_.show_meta_variables && content_checkboxes_.count(META_OBJECT_NAME))
        {
            bool meta_has_data = false;
            for (auto& meta_it : dbcont_man_.metaVariables())
                if (meta_it.second->hasDBContent())
                {
                    meta_has_data = true;
                    break;
                }

            add_content(META_OBJECT_NAME, meta_has_data, true);
        }

        if (!settings_.show_meta_variables_only)
        {
            for (auto& object_it : dbcont_man_)
            {
                if (settings_.show_existing_in_db_only && !object_it.second->hasData())
                    continue;

                add_content(object_it.first, object_it.second->hasData(), by_group);
            }
        }
    }

    // expand to group level, or everything while a search filter is active
    if (search_edit_->text().isEmpty())
        tree_view_->expandToDepth(settings_.show_dbcont_only ? 0 : 1);
    else
        tree_view_->expandAll();

    tree_view_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    tree_view_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    tree_view_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    tree_view_->header()->setSectionResizeMode(3, QHeaderView::Stretch);
}

/**
 */
void VariableSelectionDialog::updateSelectButton()
{
    traced_assert(select_button_);

    bool has_selection = false;

    for (const auto& index : tree_view_->selectionModel()->selectedRows(0))
        if (!index.data(VariableNameRole).toString().isEmpty())
        {
            has_selection = true;
            break;
        }

    select_button_->setEnabled(has_selection);
}

/**
 */
void VariableSelectionDialog::storeSettings()
{
    dbcont_man_.variableSelectionByGroup(grouping_combo_->currentIndex() == 1);

    if (!content_checkboxes_.empty())
    {
        std::set<std::string> hidden;

        for (const auto& check_it : content_checkboxes_)
            if (!check_it.second->isChecked())
                hidden.insert(check_it.first);

        dbcont_man_.variableSelectionHiddenContents(hidden);
    }
}

/**
 */
void VariableSelectionDialog::searchChangedSlot(const QString& text)
{
    proxy_model_->setFilterFixedString(text);

    if (text.isEmpty())
        tree_view_->expandToDepth(settings_.show_dbcont_only ? 0 : 1);
    else
        tree_view_->expandAll();

    updateSelectButton();
}

/**
 */
void VariableSelectionDialog::groupingChangedSlot(int index)
{
    storeSettings();
    updateModel();
    updateSelectButton();
}

/**
 */
void VariableSelectionDialog::contentToggledSlot()
{
    storeSettings();
    updateModel();
    updateSelectButton();
}

/**
 */
void VariableSelectionDialog::selectionChangedSlot()
{
    updateSelectButton();
}

/**
 */
void VariableSelectionDialog::itemDoubleClickedSlot(const QModelIndex& index)
{
    QModelIndex name_index = index.sibling(index.row(), 0);

    if (name_index.data(VariableNameRole).toString().isEmpty())
        return; // content or group node

    selection_.clear();
    selection_.emplace_back(name_index.data(ContentNameRole).toString().toStdString(),
                            name_index.data(VariableNameRole).toString().toStdString());
    empty_selected_ = false;

    storeSettings();
    accept();
}

/**
 */
void VariableSelectionDialog::selectSlot()
{
    selection_.clear();

    for (const auto& index : tree_view_->selectionModel()->selectedRows(0))
    {
        QString var_name = index.data(VariableNameRole).toString();
        if (var_name.isEmpty())
            continue;

        selection_.emplace_back(index.data(ContentNameRole).toString().toStdString(),
                                var_name.toStdString());
    }

    if (selection_.empty())
        return;

    empty_selected_ = false;

    storeSettings();
    accept();
}

/**
 */
void VariableSelectionDialog::selectNoneSlot()
{
    traced_assert(settings_.show_empty_variable);

    selection_.clear();
    empty_selected_ = true;

    storeSettings();
    accept();
}

/**
 */
std::string VariableSelectionDialog::dataItemTitle(const std::string& item)
{
    // generated from data/jasterix_definitions (newest edition per category)
    // by scripts/assign_variable_groups.py companion extraction
    static const std::map<std::string, std::string> titles = {
        {"I001/010", "Data Source Identifier"},
        {"I001/020", "Target Report Descriptor"},
        {"I001/030", "Warning/Error Conditions"},
        {"I001/040", "Measured Position in Polar Coordinates"},
        {"I001/042", "Calculated Position in Cartesian Coordinates"},
        {"I001/050", "Mode-2 Code in Octal Representation"},
        {"I001/060", "Mode-2 Code Confidence Indicator"},
        {"I001/070", "Mode-3/A Code in Octal Representation"},
        {"I001/080", "Mode-3/A Code Confidence Indicator"},
        {"I001/090", "Mode-C Code in Binary Representation"},
        {"I001/100", "Mode-C Code and Code Confidence Indicator"},
        {"I001/120", "Measured Radial Doppler Speed"},
        {"I001/130", "Radar Plot Characteristics"},
        {"I001/131", "Received Power"},
        {"I001/141", "Truncated Time of Day"},
        {"I001/150", "Presence of X-Pulse"},
        {"I001/161", "Track Plot Number"},
        {"I001/170", "Track Status"},
        {"I001/200", "Calculated Track Velocity in Polar Coordinates"},
        {"I001/210", "Track Quality"},
        {"I001/REF", "Reserved Expansion Field"},
        {"I001/SPF", "Special Purpose Field"},
        {"I002/000", "Message Type"},
        {"I002/010", "Data Source Identifier"},
        {"I002/020", "Sector Number"},
        {"I002/030", "Time of Day"},
        {"I002/041", "Antenna Rotation Speed"},
        {"I002/050", "Station Configuration Status"},
        {"I002/060", "Station Processing Mode"},
        {"I002/070", "Plot Count Values"},
        {"I002/080", "Warning/Error Conditions"},
        {"I002/090", "Collimation Error"},
        {"I002/100", "Dynamic Window - Type 1"},
        {"I002/REF", "Reserved Expansion Field"},
        {"I002/SPF", "Special Purpose Field"},
        {"I004/000", "Message Type"},
        {"I004/010", "Data Source Identifier"},
        {"I004/015", "SDPS Identifier"},
        {"I004/020", "Time of Message"},
        {"I004/030", "Track Number 1"},
        {"I004/035", "Track Number 2"},
        {"I004/040", "Alert Identifier"},
        {"I004/045", "Alert Status"},
        {"I004/060", "Safety Net Function & System Status"},
        {"I004/070", "Conflict Timing and Separation"},
        {"I004/074", "Longitudinal Deviation"},
        {"I004/075", "Transversal Distance Deviation"},
        {"I004/076", "Vertical Deviation"},
        {"I004/100", "Area Definition"},
        {"I004/110", "FDPS Sector Control Identification"},
        {"I004/120", "Conflict Characteristics"},
        {"I004/170", "Aircraft Identification & Characteristics 1"},
        {"I004/171", "Aircraft Identification & Characteristics 2"},
        {"I004/REF", "Reserved Expansion Field"},
        {"I004/SPF", "Special Purpose Field"},
        {"I010/000", "Message Type"},
        {"I010/010", "Data Source Identifier"},
        {"I010/020", "Target Report Descriptor"},
        {"I010/040", "Measured Position in Polar Coordinates"},
        {"I010/041", "Position in WGS-84 Coordinates"},
        {"I010/042", "Position in Cartesian Coordinates"},
        {"I010/060", "Mode-3/A Code in Octal Representation"},
        {"I010/090", "Flight Level in Binary Representation"},
        {"I010/091", "Measured Height"},
        {"I010/131", "Amplitude of Primary Plot"},
        {"I010/140", "Time of Day"},
        {"I010/161", "Track Number"},
        {"I010/170", "Track Status"},
        {"I010/200", "Calculated Track Velocity in Polar Co-ordinates"},
        {"I010/202", "Calculated Track Velocity in Cartesian Coordinates"},
        {"I010/210", "Calculated Acceleration"},
        {"I010/220", "Target Address"},
        {"I010/245", "Target Identification"},
        {"I010/250", "Mode S MB Data"},
        {"I010/270", "Target Size & Orientation"},
        {"I010/280", "Presence"},
        {"I010/300", "Vehicle Fleet Identification"},
        {"I010/310", "Pre-programmed Message"},
        {"I010/500", "Position Accuracy"},
        {"I010/550", "System Status"},
        {"I010/REF", "Reserved Expansion Field"},
        {"I010/SPF", "Special Purpose Field"},
        {"I019/000", "Message Type"},
        {"I019/010", "Data Source Identifier"},
        {"I019/140", "Time of Day"},
        {"I019/550", "System Status"},
        {"I019/551", "Tracking Processor Detailed Status"},
        {"I019/552", "Remote Sensor Detailed Status"},
        {"I019/553", "Reference Transponder Detailed Status"},
        {"I019/600", "Position of the MLT System Reference Point"},
        {"I019/610", "Height of the MLT System Reference Point"},
        {"I019/620", "WGS-84 Undulation"},
        {"I019/REF", "Reserved Expansion Field"},
        {"I019/SPF", "Special Purpose Field"},
        {"I020/010", "Data Source Identifier"},
        {"I020/020", "Target Report Descriptor"},
        {"I020/030", "Warning/Error Conditions"},
        {"I020/041", "Position in WGS-84 Coordinates"},
        {"I020/042", "Position in Cartesian Coordinates"},
        {"I020/050", "Mode-2 Code in Octal Representation"},
        {"I020/055", "Mode-1 Code in Octal Representation"},
        {"I020/070", "Mode-3/A Code in Octal Representation"},
        {"I020/090", "Flight Level in Binary Representation"},
        {"I020/100", "Mode-C Code"},
        {"I020/105", "Geometric Height (WGS-84)"},
        {"I020/110", "Measured Height (Local Cartesian Coordinates)"},
        {"I020/140", "Time of Day"},
        {"I020/161", "Track Number"},
        {"I020/170", "Track Status"},
        {"I020/202", "Calculated Track Velocity in Cartesian Coordinates"},
        {"I020/210", "Calculated Acceleration"},
        {"I020/220", "Target Address"},
        {"I020/230", "Communications/ACAS Capability and Flight Status"},
        {"I020/245", "Target Identification"},
        {"I020/250", "Mode S MB Data"},
        {"I020/260", "ACAS Resolution Advisory Report"},
        {"I020/300", "Vehicle Fleet Identification"},
        {"I020/310", "Pre-programmed Message"},
        {"I020/400", "Contributing Receivers"},
        {"I020/500", "Position Accuracy"},
        {"I020/REF", "Reserved Expansion Field"},
        {"I020/SPF", "Special Purpose Field"},
        {"I021/008", "Aircraft Operational Status"},
        {"I021/010", "Data Source Identification"},
        {"I021/015", "Service Identification"},
        {"I021/016", "Service Management"},
        {"I021/020", "Emitter Category"},
        {"I021/040", "Target Report Descriptor"},
        {"I021/070", "Mode 3/A Code in Octal Representation"},
        {"I021/071", "Time of Applicability for Position"},
        {"I021/072", "Time of Applicability for Velocity"},
        {"I021/073", "Time of Message Reception for Position"},
        {"I021/074", "Time of Message Reception of Position-High Precision"},
        {"I021/075", "Time of Message Reception for Velocity"},
        {"I021/076", "Time of Message Reception of Velocity-High Precision"},
        {"I021/077", "Time of ASTERIX Report Transmission"},
        {"I021/080", "Target Address"},
        {"I021/090", "Quality Indicators"},
        {"I021/110", "Trajectory Intent"},
        {"I021/130", "Position in WGS-84 Co-ordinates"},
        {"I021/131", "High-Resolution Position in WGS-84 Co-ordinates"},
        {"I021/132", "Message Amplitude"},
        {"I021/140", "Geometric Height"},
        {"I021/145", "Flight Level"},
        {"I021/146", "Selected Altitude"},
        {"I021/148", "Final State Selected Altitude"},
        {"I021/150", "Air Speed"},
        {"I021/151", "True Airspeed"},
        {"I021/152", "Magnetic Heading"},
        {"I021/155", "Barometric Vertical Rate"},
        {"I021/157", "Geometric Vertical Rate"},
        {"I021/160", "Airborne Ground Vector"},
        {"I021/161", "Track Number"},
        {"I021/165", "Track Angle Rate"},
        {"I021/170", "Target Identification"},
        {"I021/200", "Target Status"},
        {"I021/210", "MOPS Version"},
        {"I021/220", "Met Information"},
        {"I021/230", "Roll Angle"},
        {"I021/250", "Mode S MB Data"},
        {"I021/260", "ACAS Resolution Advisory Report"},
        {"I021/271", "Surface Capabilities and Characteristics"},
        {"I021/295", "Data Ages"},
        {"I021/400", "Receiver ID"},
        {"I021/REF", "Reserved Expansion Field"},
        {"I021/SPF", "Special Purpose Field"},
        {"I023/000", "Report Type"},
        {"I023/010", "Data Source Identifier"},
        {"I023/015", "Service Type and Identification"},
        {"I023/070", "Time of Day"},
        {"I023/100", "Ground Station Status"},
        {"I023/101", "Service Configuration"},
        {"I023/110", "Service Status"},
        {"I023/120", "Service Statistics"},
        {"I023/200", "Operational Range"},
        {"I023/REF", "Reserved Expansion Field"},
        {"I023/SPF", "Special Purpose Field"},
        {"I025/000", "Report Type"},
        {"I025/010", "Data Source Identifier"},
        {"I025/015", "Service Identification"},
        {"I025/020", "Service Designator"},
        {"I025/070", "Time of Day"},
        {"I025/100", "System and Service Status"},
        {"I025/105", "System and Service Error Codes"},
        {"I025/120", "Component Status"},
        {"I025/140", "Service Statistics"},
        {"I025/200", "Message Identification"},
        {"I025/600", "Position of the System Reference Point"},
        {"I025/610", "Height of the System Reference Point"},
        {"I025/REF", "Reserved Expansion Field"},
        {"I025/SPF", "Special Purpose Field"},
        {"I034/000", "Message Type"},
        {"I034/010", "Data Source Identifier"},
        {"I034/020", "Sector Number"},
        {"I034/030", "Time of Day"},
        {"I034/041", "Antenna Rotation Speed"},
        {"I034/050", "System Configuration and Status"},
        {"I034/060", "System Processing Mode"},
        {"I034/070", "Message Count Values"},
        {"I034/090", "Collimation Error"},
        {"I034/100", "Generic Polar Window"},
        {"I034/110", "Data Filter"},
        {"I034/120", "3D-Position Of Data Source"},
        {"I034/REF", "Reserved Expansion Field"},
        {"I034/SPF", "Special Purpose Field"},
        {"I048/010", "Data Source Identifier"},
        {"I048/020", "Target Report Descriptor"},
        {"I048/030", "Warning/Error Conditions"},
        {"I048/040", "Measured Position in Polar Co-ordinates"},
        {"I048/042", "Calculated Position in Cartesian Co-ordinates"},
        {"I048/050", "Mode-2 Code in Octal Representation"},
        {"I048/055", "Mode-1 Code in Octal Representation"},
        {"I048/060", "Mode-2 Code Confidence Indicator"},
        {"I048/065", "Mode-1 Code Confidence Indicator"},
        {"I048/070", "Mode-3/A Code in Octal Representation"},
        {"I048/080", "Mode-3/A Code Confidence Indicator"},
        {"I048/090", "Flight Level in Binary Representation"},
        {"I048/100", "Mode-C Code and Code Confidence Indicator"},
        {"I048/110", "Height Measured by a 3D Radar"},
        {"I048/120", "Radial Doppler Speed"},
        {"I048/130", "Radar Plot Characteristics"},
        {"I048/140", "Time of Day"},
        {"I048/161", "Track Number"},
        {"I048/170", "Track Status"},
        {"I048/200", "Calculated Track Velocity in Polar Co-ordinates"},
        {"I048/210", "Track Quality"},
        {"I048/220", "Aircraft Address"},
        {"I048/230", "Communications/ACAS Capability and Flight Status"},
        {"I048/240", "Aircraft Identification"},
        {"I048/250", "Mode S MB Data"},
        {"I048/260", "ACAS Resolution Advisory Report"},
        {"I048/REF", "Reserved Expansion Field"},
        {"I048/SPF", "Special Purpose Field"},
        {"I062/010", "Data Source Identifier"},
        {"I062/015", "Service Identification"},
        {"I062/040", "Track Number"},
        {"I062/060", "Track Mode 3/A Code"},
        {"I062/070", "Time Of Track Information"},
        {"I062/080", "Track Status"},
        {"I062/100", "Calculated Track Position. (Cartesian)"},
        {"I062/105", "Calculated Position In WGS-84 Co-ordinates"},
        {"I062/110", "Mode 5 Data reports & Extended Mode 1 Code"},
        {"I062/120", "Track Mode 2 Code"},
        {"I062/130", "Calculated Track Geometric Altitude"},
        {"I062/135", "Calculated Track Barometric Altitude"},
        {"I062/136", "Measured Flight Level"},
        {"I062/185", "Calculated Track Velocity (Cartesian)"},
        {"I062/200", "Mode of Movement"},
        {"I062/210", "Calculated Acceleration (Cartesian)"},
        {"I062/220", "Calculated Rate Of Climb/Descent"},
        {"I062/245", "Target Identification"},
        {"I062/270", "Target Size & Orientation"},
        {"I062/290", "System Track Update Ages"},
        {"I062/295", "Track Data Ages"},
        {"I062/300", "Vehicle Fleet Identification"},
        {"I062/340", "Measured Information"},
        {"I062/380", "Aircraft Derived Data"},
        {"I062/390", "Flight Plan Related Data"},
        {"I062/500", "Estimated Accuracies"},
        {"I062/510", "Composed Track Number"},
        {"I062/REF", "Reserved Expansion Field"},
        {"I062/SPF", "Special Purpose Field"},
        {"I063/010", "Data Source Identifier"},
        {"I063/015", "Service Identification"},
        {"I063/030", "Time of Message"},
        {"I063/050", "Sensor Identifier"},
        {"I063/060", "Sensor Configuration and Status"},
        {"I063/070", "Time Stamping Bias"},
        {"I063/080", "SSR / Mode S Range Gain and Bias"},
        {"I063/081", "SSR / Mode S Azimuth Bias"},
        {"I063/090", "PSR Range Gain and Bias"},
        {"I063/091", "PSR Azimuth Bias"},
        {"I063/092", "PSR Elevation Bias"},
        {"I063/REF", "Reserved Expansion Field"},
        {"I063/SPF", "Special Purpose Field"},
        {"I065/000", "Message Type"},
        {"I065/010", "Data Source Identifier"},
        {"I065/015", "Service Identification"},
        {"I065/020", "Batch Number"},
        {"I065/030", "Time of Message"},
        {"I065/040", "SDPS Configuration and Status"},
        {"I065/050", "Service Status Report"},
        {"I065/REF", "Reserved Expansion Field"},
        {"I065/SPF", "Special Purpose Field"},
    };

    auto it = titles.find(item);
    if (it != titles.end())
        return it->second;

    return "";
}

}
