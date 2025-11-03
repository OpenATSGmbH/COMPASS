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

#include "configurable.h"
#include "appmode.h"
#include "dbfilter.h"

#include <QObject>

#include <map>
#include <string>
#include <vector>

class DataSourcesFilter;
class COMPASS;
class FilterManagerWidget;
class ViewableDataConfig;
class Buffer;

namespace dbContent {

class Variable;

}

class FilterManager : public QObject, public Configurable
{
    Q_OBJECT
signals:
    void changedFiltersSignal();

public slots:

    void databaseOpenedSlot();
    void databaseClosedSlot();
    void dataSourcesChangedSlot();

    void appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current);

    //void deleteFilterSlot(DBFilter* filter);

    void unshowViewPointSlot (const ViewableDataConfig* vp);
    void showViewPointSlot (const ViewableDataConfig* vp);

public:
    FilterManager(const std::string& class_id, const std::string& instance_id, COMPASS* compass);
    virtual ~FilterManager();

    bool useFilters() const;
    void useFilters(bool useFilters);

    std::string getSQLCondition(const std::string& dbcontent_name);

    unsigned int getNumFilters();
    DBFilter* getFilter(unsigned int index);
    const std::vector<std::unique_ptr<DBFilter>>& filters() { return filters_; }

    bool hasFilter (const std::string& name);
    DBFilter* getFilter (const std::string& name);

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    // resets all filters
    void reset();

    FilterManagerWidget* widget();

    void setConfigInViewPoint (nlohmann::json& data);
    void disableAllFilters ();

    void filterBuffers(std::map<std::string, std::shared_ptr<Buffer>>& data);

    void resetToStartupConfiguration();

protected:
    // database id, resets if changed
    std::string db_id_;
    bool use_filters_{false};

    FilterManagerWidget* widget_{nullptr};

    std::vector<std::unique_ptr<DBFilter>> filters_;

    virtual void checkSubConfigurables();

    bool checkDBContent (const std::string& dbcontent_name); // returns true if ok
};
