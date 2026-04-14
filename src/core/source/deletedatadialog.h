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

class DataSourceManager;
class QCheckBox;
class QTreeWidget;
class QTreeWidgetItem;

class DeleteDataDialog : public QDialog
{
    Q_OBJECT

public:
    DeleteDataDialog(DataSourceManager& ds_man, QWidget* parent = nullptr);
    virtual ~DeleteDataDialog() = default;

    nlohmann::json selectedDeleteInfo() const;
    QString deleteDescription() const;

private:
    void createUI();
    void populateDataSourcesTree();

    struct SelectedDS
    {
        unsigned int ds_id;
        std::string name;
        bool all_lines;
        std::set<unsigned int> line_ids;
    };

    void collectSelections(std::set<std::string>& dbcontents,
                           std::vector<SelectedDS>& data_sources) const;

    DataSourceManager& ds_man_;

    std::map<std::string, QCheckBox*> dbcontent_checks_;
    QTreeWidget* ds_tree_{nullptr};
};
