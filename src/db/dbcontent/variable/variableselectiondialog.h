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

#include "property.h"

#include <QDialog>

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

class QAction;
class QButtonGroup;
class QComboBox;
class QGridLayout;
class QLineEdit;
class QPushButton;
class QRadioButton;
class QSortFilterProxyModel;
class QStandardItem;
class QStandardItemModel;
class QToolButton;
class QTreeView;
class QWidget;

class DBContentManager;
class PopupMenu;

namespace dbContent
{

class Variable;
class MetaVariable;
class ReportVariable;

/**
 * Modal variable picker replacing the former QMenu-based selection.
 * Shows the variables of one content in a tree (content -> data item or group -> variable),
 * with search, an exclusive content selection strip, and switchable grouping.
 */
class VariableSelectionDialog : public QDialog
{
    Q_OBJECT

  public:
    struct Settings
    {
        bool show_meta_variables{false};
        bool show_meta_variables_only{false};

        bool show_dbcont_only{false};
        std::string only_dbcontent_name;

        bool show_data_types_only{false};
        std::vector<PropertyDataType> only_data_types;

        bool show_existing_in_db_only{false};

        bool show_empty_variable{false}; // offer a Select None button
        bool multi_select{false};        // allow selection of multiple variables

        bool show_reports{true};         // offer the Report Variables of the stored reports
    };

    VariableSelectionDialog(DBContentManager& dbcont_man, const Settings& settings,
                            QWidget* parent = nullptr);
    virtual ~VariableSelectionDialog();

    /// selected (content name or META_OBJECT_NAME, variable name) pairs, empty if Select None was used
    const std::vector<std::pair<std::string, std::string>>& selection() const { return selection_; }
    bool emptySelected() const { return empty_selected_; }

    /// title of an ASTERIX data item, e.g. "I048/140" -> "Time of Day"; empty if unknown
    static std::string dataItemTitle(const std::string& item);

  protected slots:
    void searchChangedSlot(const QString& text);
    void groupingChangedSlot(int index);
    void contentSelectedSlot();
    void hideStatusContentsToggledSlot(bool hide);
    void selectionChangedSlot();
    void itemDoubleClickedSlot(const QModelIndex& index);
    void selectSlot();
    void selectNoneSlot();

  protected:
    void createUI();
    void createContentStrip();
    void updateContentStrip();
    void updateModel();
    void updateSelectButton();
    void storeSettings();

    bool showDataType(PropertyDataType type) const;
    std::vector<std::string> selectableContents() const;
    /// names of the stored Reports offering Report Variables, decision 18
    std::vector<std::string> selectableReports() const;
    bool contentShown(const std::string& content_name) const;
    bool isReport(const std::string& content_name) const;

    QStandardItem* makeVariableRow(const std::string& content_name, const std::string& var_name,
                                   const Variable* variable, const MetaVariable* meta_variable,
                                   const ReportVariable* report_variable,
                                   QList<QStandardItem*>& row) const;
    /// rows of a Report entry, restricted to one host data content if given
    void addReportVariableRows(QStandardItem* content_item, const std::string& report_name,
                               const std::string& host_dbcontent_name);
    void addVariableRows(QStandardItem* content_item, const std::string& content_name,
                         bool group_content_by_group);
    std::string dataItemOf(const std::string& source) const;

    DBContentManager& dbcont_man_;
    Settings settings_;

    QLineEdit* search_edit_{nullptr};
    QComboBox* grouping_combo_{nullptr};

    QGridLayout* content_strip_layout_{nullptr};
    QButtonGroup* content_group_{nullptr};
    std::map<std::string, QRadioButton*> content_buttons_; // content name -> radio button

    // the Reports form a second, collapsible row group under the data contents
    QWidget*     report_strip_widget_{nullptr};
    QGridLayout* report_strip_layout_{nullptr};
    QToolButton* report_expand_button_{nullptr};

    QPushButton* content_menu_button_{nullptr};
    PopupMenu* content_menu_{nullptr};
    QAction* hide_status_action_{nullptr};

    std::string selected_content_; // content whose variables are shown, empty if none

    QTreeView* tree_view_{nullptr};
    QStandardItemModel* model_{nullptr};
    QSortFilterProxyModel* proxy_model_{nullptr};

    QPushButton* cancel_button_{nullptr};
    QPushButton* select_none_button_{nullptr};
    QPushButton* select_button_{nullptr};

    std::vector<std::pair<std::string, std::string>> selection_;
    bool empty_selected_{false};
};

}
