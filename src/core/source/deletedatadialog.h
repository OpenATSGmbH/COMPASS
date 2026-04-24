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

#include "json_fwd.hpp"

#include <QDialog>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace context { class DBContextManager; }
class QCheckBox;
class QTreeWidget;
class QTreeWidgetItem;

class DeleteDataDialog : public QDialog
{
    Q_OBJECT

public:
    DeleteDataDialog(context::DBContextManager& ctx_man, QWidget* parent = nullptr);
    virtual ~DeleteDataDialog() = default;

    nlohmann::json selectedDeleteInfo() const;
    QString deleteDescription() const;

    /// Pre-select tree DSType rows by name (ticking them). Call before exec().
    void preselectDSTypes(const std::set<std::string>& names);
    /// Pre-select tree DataSource rows (and their parent DSType) by id.
    void preselectDataSources(const std::set<unsigned int>& ds_ids);
    /// Pre-check the top DBContent checkboxes by name.
    void preselectDBContents(const std::set<std::string>& names);

    /// If non-empty, restrict the dialog's output to the listed DBContents
    /// only — both the top-check "delete everywhere" and the per-DS step.
    /// Intended for scoped deletes where the caller wants to confine the
    /// dialog to a single DBContent (e.g. from a DBContent row click).
    void setRestrictedDBContents(const std::set<std::string>& names);

private:
    void createUI();
    void populateDataSourcesTree();
    void itemChangedSlot(QTreeWidgetItem* item, int column);
    void showTreeContextMenu(const QPoint& pos);

    static void setCheckRecursive(QTreeWidgetItem* item, Qt::CheckState state);

    struct SelectedDS
    {
        unsigned int ds_id;
        std::string name;
        bool all_lines;
        std::set<unsigned int> line_ids;
    };

    void collectSelections(std::set<std::string>& dbcontents,
                           std::vector<SelectedDS>& data_sources) const;

    context::DBContextManager& ctx_man_;

    std::map<std::string, QCheckBox*> dbcontent_checks_;
    QTreeWidget* ds_tree_{nullptr};

    std::set<std::string> restricted_dbcontents_; // empty = no restriction
};
