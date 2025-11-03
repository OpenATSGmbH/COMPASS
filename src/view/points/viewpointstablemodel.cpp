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

#include "viewpointstablemodel.h"
#include "viewmanager.h"
#include "viewpoint.h"
#include "json.hpp"
#include "json_tools.h"
//#include "stringconv.h"
#include "files.h"
#include "compass.h"
#include "dbinterface.h"
#include "viewpointswidget.h"
#include "logger.h"

#include <QMessageBox>

#include <fstream>


using namespace nlohmann;
using namespace Utils;
using namespace std;

ViewPointsTableModel::ViewPointsTableModel(ViewManager& view_manager)
    : view_manager_(view_manager)
{
    table_columns_ = default_table_columns_;

    open_icon_ = Files::IconProvider::getIcon("not_recommended.png");
    closed_icon_ = Files::IconProvider::getIcon("not_todo.png");
    todo_icon_ = Files::IconProvider::getIcon("todo.png");
    unknown_icon_ = Files::IconProvider::getIcon("todo_maybe.png");
}

//ViewPointsTableModel::~ViewPointsTableModel()
//{

//}


void ViewPointsTableModel::loadViewPoints()
{
    loginf;

    beginResetModel();

    // load view points
    if (COMPASS::instance().dbInterface().existsViewPointsTable())
    {
        for (const auto& vp_it : COMPASS::instance().dbInterface().viewPoints())
        {
            //assert (!view_points_.count(vp_it.first));
            traced_assert(!hasViewPoint(vp_it.first));

            //view_points_.emplace_back(vp_it.first, vp_it.second, view_manager_, false);

            view_points_.push_back({vp_it.first, vp_it.second, view_manager_, false});  // args for mapped value
            // args for mapped value

            if (vp_it.first > max_id_)
                max_id_ = vp_it.first;
        }
    }

    updateTableColumns();
    updateTypes();
    updateStatuses();

    endResetModel();
}
void ViewPointsTableModel::addViewPoints(const std::vector <nlohmann::json>& viewpoints)
{
    loginf;

    beginResetModel();

    for (auto vp_it = viewpoints.begin(); vp_it != viewpoints.end(); ++vp_it)
        saveNewViewPoint(*vp_it, std::next(vp_it) == viewpoints.end());

    updateTableColumns();
    updateTypes();
    updateStatuses();

    endResetModel();
}

void ViewPointsTableModel::clearViewPoints()
{
    loginf;

    beginResetModel();

    view_points_.clear();

    max_id_ = 0;

    table_columns_ = default_table_columns_;
    types_.clear();
    statuses_.clear();

    endResetModel();

    loginf << "done";
}

int ViewPointsTableModel::rowCount(const QModelIndex& parent) const
{
    return view_points_.size();
}

int ViewPointsTableModel::columnCount(const QModelIndex& parent) const
{
    return table_columns_.size();
}

QVariant ViewPointsTableModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    switch (role)
    {
    case Qt::DisplayRole:
    case Qt::EditRole:
    {
        logdbg << "display role: row " << index.row() << " col " << index.column();

        traced_assert(index.row() >= 0);
        traced_assert((unsigned int)index.row() < view_points_.size());

        const ViewPoint& vp = view_points_.at(index.row());

        logdbg << "got key " << view_points_.at(index.row()).id();

        traced_assert(index.column() < table_columns_.size());
        std::string col_name = table_columns_.at(index.column()).toStdString();

        if (!vp.data().contains(col_name))
            return QVariant();

        const json& data = vp.data().at(col_name);

        //            if (col_name == "status" && (data == "open" || data == "closed" || data == "todo"))
        //                return QVariant();

        // s1.find(s2) != std::string::npos
//        if (data.is_number() && col_name.find(ViewPoint::VP_TIMESTAMP_KEY) != std::string::npos)
//            return Time::toString(data).c_str();

        if (data.is_boolean())
            return data.get<bool>();

        if (data.is_number())
            return data.get<float>();

        return JSON::toString(data).c_str();
    }
    case Qt::DecorationRole:
    {
        traced_assert(index.column() < table_columns_.size());

        if (table_columns_.at(index.column()).toStdString() == ViewPoint::VP_STATUS_KEY)
        {
            traced_assert(index.row() >= 0);
            traced_assert((unsigned int)index.row() < view_points_.size());

            const ViewPoint& vp = view_points_.at(index.row());

            const json& data = vp.data().at(ViewPoint::VP_STATUS_KEY);
            traced_assert(data.is_string());

            std::string status = data;

            if (status == "open")
                return open_icon_;
            else if (status == "closed")
                return closed_icon_;
            else if (status == "todo")
                return todo_icon_;
            else
                return unknown_icon_;
        }
        else
            return QVariant();
    }
    default:
    {
        return QVariant();
    }
    }
}

QVariant ViewPointsTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < table_columns_.size());
        return table_columns_.at(section);
    }

    return QVariant();
}

QModelIndex ViewPointsTableModel::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

QModelIndex ViewPointsTableModel::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

Qt::ItemFlags ViewPointsTableModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    traced_assert(index.column() < table_columns_.size());

    if (table_columns_.at(index.column()).toStdString() == ViewPoint::VP_COMMENT_KEY)
        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
    else
        return QAbstractItemModel::flags(index);
}

bool ViewPointsTableModel::setData(const QModelIndex& index, const QVariant &value, int role)
{
    if (index.isValid() && role == Qt::EditRole)
    {
        unsigned int id = getIdOf(index);

        loginf << "row " << index.row() << " col " << index.column()
               << " id " << id << " '" << value.toString().toStdString() << "'";

        //assert (id < view_points_.size());

        auto it = view_points_.begin()+index.row();

        traced_assert(index.column() == statusColumn() || index.column() == commentColumn());

        //        if (index.column() == statusColumn())
        //            view_points_.at(id).setStatus(value.toString().toStdString());
        //        else
        //            view_points_.at(id).setComment(value.toString().toStdString());

        if (index.column() == statusColumn())
        {
            view_points_.modify(it, [value](ViewPoint& p) { p.setStatus(value.toString().toStdString()); });
            updateStatuses();
        }
        else
            view_points_.modify(it, [value](ViewPoint& p) { p.setComment(value.toString().toStdString()); });

        emit dataChanged(index, index);

        return true;
    }
    return false;
}

//int ViewPointsTableModel::columnIndex (QString name)
//{
//    traced_assert(table_columns_.contains(name));
//    return table_columns_.indexOf(name);
//}

bool ViewPointsTableModel::updateTableColumns()
{
    loginf;

    bool changed = false;

    for (auto& vp_it : view_points_)
    {
        const nlohmann::json& data = vp_it.data();

        traced_assert(data.is_object());
        for (auto& j_it : data.get<json::object_t>())
        {
            logdbg << "'" << j_it.first << "'";

            if (j_it.second.is_object() || j_it.second.is_array()) // skip complex items
                continue;

            if (!table_columns_.contains(j_it.first.c_str()))
            {
                beginInsertColumns(QModelIndex(), table_columns_.size(), table_columns_.size());
                table_columns_.append(j_it.first.c_str());
                endInsertColumns();

                changed = true;
            }
        }
    }

    return changed;
}

void ViewPointsTableModel::updateTypes()
{
    loginf;

    QStringList old_types = types_;
    types_.clear();

    for (auto& vp_it : view_points_)
    {
        const nlohmann::json& data = vp_it.data();

        traced_assert(data.contains(ViewPoint::VP_TYPE_KEY));

        const string& type = data.at(ViewPoint::VP_TYPE_KEY);

        if (!types_.contains(type.c_str()))
            types_.append(type.c_str());
    }

    if (types_ != old_types)
    {
        loginf << "changed";

        emit typesChangedSignal(types_);
    }
}


void ViewPointsTableModel::updateStatuses()
{
    loginf;

    QStringList old_statuses = statuses_;
    statuses_.clear();

    for (auto& vp_it : view_points_)
    {
        const nlohmann::json& data = vp_it.data();

        traced_assert(data.contains(ViewPoint::VP_STATUS_KEY));

        const string& status = data.at(ViewPoint::VP_STATUS_KEY);

        if (!statuses_.contains(status.c_str()))
            statuses_.append(status.c_str());
    }

    if (statuses_ != old_statuses)
    {
        loginf << "changed";

        emit statusesChangedSignal(statuses_);
    }
}
QStringList ViewPointsTableModel::types() const
{
    return types_;
}

QStringList ViewPointsTableModel::statuses() const
{
    return statuses_;
}

QStringList ViewPointsTableModel::tableColumns() const
{
    return table_columns_;
}

QStringList ViewPointsTableModel::defaultTableColumns() const
{
    return default_table_columns_;
}

const ViewPointCache& ViewPointsTableModel::viewPoints() const
{
    return view_points_;
}

bool ViewPointsTableModel::hasViewPoint (unsigned int id)
{
    return view_points_.get<vp_tag>().find(id) != view_points_.get<vp_tag>().end();
}

unsigned int ViewPointsTableModel::saveNewViewPoint(const nlohmann::json& data, bool update)
{
    unsigned int new_id = max_id_+1;

    traced_assert(!hasViewPoint(new_id));

    nlohmann::json new_data = data;
    new_data[ViewPoint::VP_ID_KEY] = new_id;

    saveNewViewPoint(new_id, new_data, update); // auto increments max_id

    return new_id;
}

const ViewPoint& ViewPointsTableModel::saveNewViewPoint(unsigned int id, const nlohmann::json& data, bool update)
{
    if (hasViewPoint(id))
        throw std::runtime_error ("ViewPointsTableModel: addNewViewPoint: id "+std::to_string(id)+" already exists");

    unsigned int row = view_points_.size();

    if (update)
        beginInsertRows(QModelIndex(), row, row);

    nlohmann::json new_data = data;

    traced_assert(new_data.is_object());
    json::object_t& new_data_ref = new_data.get_ref<json::object_t&>();

    view_points_.push_back({id, new_data_ref, view_manager_, true});

    if (id > max_id_)
        max_id_ = id;

    //    view_points_.emplace(std::piecewise_construct,
    //                         std::forward_as_tuple(id),   // args for key
    //                         std::forward_as_tuple(id, new_data_ref, view_manager_, true));  // args for mapped value

    traced_assert(hasViewPoint(id));

    if (update)
    {
        emit endInsertRows();

        if (updateTableColumns()) // true if changed
            view_manager_.viewPointsWidget()->resizeColumnsToContents();

        updateTypes();
        updateStatuses();
    }

    return viewPoint(id);
}

//bool ViewPointsTableModel::existsViewPoint(unsigned int id)
//{
//    return view_points_.count(id) == 1;
//}

const ViewPoint& ViewPointsTableModel::viewPoint(unsigned int id)
{
    traced_assert(hasViewPoint(id));
    return *view_points_.get<vp_tag>().find(id);
}

//void ViewPointsTableModel::removeViewPoint(unsigned int id)
//{
//    traced_assert(existsViewPoint(id));

//    view_points_.erase(id);
//    ATSDB::instance().interface().deleteViewPoint(id);
//}

void ViewPointsTableModel::deleteAllViewPoints ()
{
    if (!view_points_.size())
        return;

    if (table_columns_.size() > default_table_columns_.size())
    {
        beginRemoveColumns(QModelIndex(), default_table_columns_.size(), table_columns_.size()-1);
        table_columns_ = default_table_columns_;
        endRemoveColumns();
    }

    view_manager_.unsetCurrentViewPoint();

    beginRemoveRows(QModelIndex(), 0, view_points_.size()-1); // TODO

    view_points_.clear();
    COMPASS::instance().dbInterface().deleteAllViewPoints();

    endRemoveRows();
}

void ViewPointsTableModel::printViewPoints()
{
    for (auto& vp_it : view_points_)
        vp_it.print();
}

void ViewPointsTableModel::exportViewPoints (const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    json data;

    data[ViewPoint::VP_COLLECTION_CONTENT_TYPE_KEY   ] = ViewPoint::VP_COLLECTION_CONTENT_TYPE;
    data[ViewPoint::VP_COLLECTION_CONTENT_VERSION_KEY] = ViewPoint::VP_COLLECTION_CONTENT_VERSION;

    //data["view_point_context"] = json::object();
    //json& context = data.at("view_point_context");

    data[ViewPoint::VP_COLLECTION_ARRAY_KEY] = json::array();
    json& view_points = data.at(ViewPoint::VP_COLLECTION_ARRAY_KEY);

    unsigned int cnt = 0;
    for (auto& vp_it : view_points_)
    {
        view_points[cnt] = vp_it.data();
        ++cnt;
    }

    std::ofstream file(filename);
    file << data.dump(4);

    QMessageBox m_info(QMessageBox::Information, "View Points Export File",
                       "File export: '"+QString(filename.c_str())+"' done.\n"
                       +QString::number(view_points.size())+" View Points saved.", QMessageBox::Ok);
    m_info.exec();
}

//void ViewPointsTableModel::update()
//{
//    loginf;
//    beginResetModel();
//    updateTableColumns();
//    endResetModel();
//}

unsigned int ViewPointsTableModel::getIdOf (const QModelIndex& index)
{
    traced_assert(index.isValid());
    //    auto map_it = view_points_.begin();
    //    std::advance(map_it, index.row());
    //    traced_assert(map_it != view_points_.end());

    traced_assert(index.row() >= 0);
    traced_assert((unsigned int)index.row() < view_points_.size());

    return view_points_.at(index.row()).id();

    //return map_it->first;
}

void ViewPointsTableModel::setStatus (const QModelIndex& row_index, const std::string& value)
{
    traced_assert(row_index.isValid());

    QModelIndex status_index = index(row_index.row(), statusColumn(), QModelIndex());
    traced_assert(status_index.isValid());

    setData(status_index, value.c_str(), Qt::EditRole);

    //    unsigned int id = getIdOf(index);
    //    traced_assert(id < view_points_.size());
    //    loginf << "id " << id << " status " << value;
    //    view_points_.at(id).data()["status"] = value;
    //    view_points_.at(id).dirty(true);

    //    emit dataChanged(index, index, {Qt::UserRole});
}

