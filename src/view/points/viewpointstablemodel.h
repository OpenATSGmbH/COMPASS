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

#include "viewpoint.h"

#include <QAbstractItemModel>
#include <QIcon>

#include "json_fwd.hpp"

#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/random_access_index.hpp>
#include <boost/multi_index_container.hpp>

class ViewManager;

struct vp_tag
{
};

typedef boost::multi_index_container<
    ViewPoint,
    boost::multi_index::indexed_by<
        boost::multi_index::random_access<>,  // this index represents insertion order
        boost::multi_index::hashed_unique<boost::multi_index::tag<vp_tag>,
            boost::multi_index::member<
        ViewPoint, const unsigned int, &ViewPoint::id_> >
        > >
    ViewPointCache;

class ViewPointsTableModel : public QAbstractItemModel
{
    Q_OBJECT

signals:
    void typesChangedSignal(QStringList types);
    void statusesChangedSignal(QStringList types);

public:
    ViewPointsTableModel(ViewManager& view_manager);
    //virtual ~ViewPointsTableModel();

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    bool setData(const QModelIndex &index, const QVariant &value, int role) override;

    void loadViewPoints();
    void addViewPoints(const std::vector <nlohmann::json>& viewpoints);
    void clearViewPoints();

    bool hasViewPoint (unsigned int id);
    unsigned int saveNewViewPoint(const nlohmann::json& data, bool update=true);
    const ViewPoint& saveNewViewPoint(unsigned int id, const nlohmann::json& data, bool update=true);
    //bool existsViewPoint(unsigned int id);
    const ViewPoint& viewPoint(unsigned int id);
    //void removeViewPoint(unsigned int id);
    void deleteAllViewPoints ();

    //std::map<unsigned int, ViewPoint>& viewPoints() { return view_points_; }
    void printViewPoints();
    //void saveViewPoints();

    void exportViewPoints (const std::string& filename);

    //void update();
    unsigned int getIdOf (const QModelIndex& index);

    void setStatus (const QModelIndex &row_index, const std::string& value);

    int typeColumn () { return table_columns_.indexOf(ViewPoint::VP_TYPE_KEY.c_str()); }
    int statusColumn () { return table_columns_.indexOf(ViewPoint::VP_STATUS_KEY.c_str()); }
    int commentColumn () { return table_columns_.indexOf(ViewPoint::VP_COMMENT_KEY.c_str()); }
    //int columnIndex (QString name);

    bool updateTableColumns(); // true if changed
    void updateTypes(); // emits signal if changed
    void updateStatuses(); // emits signal if changed

    QStringList types() const;
    QStringList statuses() const;
    QStringList tableColumns() const;

    QStringList defaultTableColumns() const;

    const ViewPointCache& viewPoints() const;

private:
    ViewManager& view_manager_;

    QStringList default_table_columns_ {ViewPoint::VP_ID_KEY.c_str(), ViewPoint::VP_NAME_KEY.c_str(),
                ViewPoint::VP_TYPE_KEY.c_str(), ViewPoint::VP_STATUS_KEY.c_str(), ViewPoint::VP_COMMENT_KEY.c_str()};
    QStringList table_columns_;

    QStringList types_;
    QStringList statuses_;

    QIcon open_icon_;
    QIcon closed_icon_;
    QIcon todo_icon_;
    QIcon unknown_icon_;

    //std::map<unsigned int, ViewPoint> view_points_;
    ViewPointCache view_points_;
    unsigned int max_id_ {0};
};
