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

#include "filtermanager.h"

#include "compass.h"
#include "dbfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "datasourcemanager.h"
#include "filtermanagerwidget.h"
#include "logger.h"
#include "viewpoint.h"
#include "utnfilter.h"
#include "adsbqualityfilter.h"
#include "acadfilter.h"
#include "acidfilter.h"
#include "mode3afilter.h"
#include "modecfilter.h"
#include "primaryonlyfilter.h"
#include "timestampfilter.h"
#include "trackertracknumberfilter.h"
#include "reftrajaccuracyfilter.h"
#include "mlatrufilter.h"
#include "excludedtimewindowsfilter.h"

#include "json.hpp"

using namespace std;
using namespace nlohmann;

FilterManager::FilterManager(const std::string& class_id, const std::string& instance_id,
                             COMPASS* compass)
    : Configurable(class_id, instance_id, compass, "filter.json")
{
    logdbg << "start";

    registerParameter("use_filters", &use_filters_, false);
    registerParameter("db_id", &db_id_, std::string());

    createSubConfigurables();
}

FilterManager::~FilterManager()
{
    filters_.clear();

    if (widget_)
    {
        delete widget_;
        widget_ = nullptr;
    }
}

bool FilterManager::useFilters() const
{
    return use_filters_;
}

void FilterManager::useFilters(bool use_filters)
{
    use_filters_ = use_filters;
    loginf << "start" << use_filters_;

    if (widget_)
        widget_->updateUseFilters();
}

void FilterManager::generateSubConfigurable(const std::string& class_id,
                                            const std::string& instance_id)
{
    if (hasSubConfigurable(class_id, instance_id))
    {
        logerr << "filter " << instance_id
               << " already present";
        return;
    }

    logdbg << "filter class_id " << class_id << " instance_id " << instance_id;

    if (class_id == "DBFilter")
    {
        DBFilter* filter = new DBFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "ADSBQualityFilter")
    {
        ADSBQualityFilter* filter = new ADSBQualityFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "ACADFilter")
    {
        ACADFilter* filter = new ACADFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "ACIDFilter")
    {
        ACIDFilter* filter = new ACIDFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "Mode3AFilter")
    {
        Mode3AFilter* filter = new Mode3AFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "ModeCFilter")
    {
        ModeCFilter* filter = new ModeCFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "TimestampFilter")
    {
        TimestampFilter* filter = new TimestampFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "TrackerTrackNumberFilter")
    {
        TrackerTrackNumberFilter* filter = new TrackerTrackNumberFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "UTNFilter")
    {
        if (hasSubConfigurable(class_id, instance_id))
        {
            logerr << "utn filter "
                   << instance_id << " already present";
            return;
        }

        UTNFilter* filter = new UTNFilter(class_id, instance_id, this);

        filters_.emplace_back(filter);
    }
    else if (class_id == "ADSBQualityFilter")
    {
        if (hasSubConfigurable(class_id, instance_id))
        {
            logerr << "adsb quality filter "
                   << instance_id << " already present";
            return;
        }

        ADSBQualityFilter* filter = new ADSBQualityFilter(class_id, instance_id, this);

        filters_.emplace_back(filter);
    }
    else if (class_id == "PrimaryOnlyFilter")
    {
        PrimaryOnlyFilter* filter = new PrimaryOnlyFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "RefTrajAccuracyFilter")
    {
        RefTrajAccuracyFilter* filter = new RefTrajAccuracyFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "MLATRUFilter")
    {
        MLATRUFilter* filter = new MLATRUFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else if (class_id == "ExcludedTimeWindowsFilter")
    {
        ExcludedTimeWindowsFilter* filter = new ExcludedTimeWindowsFilter(class_id, instance_id, this);
        filters_.emplace_back(filter);
    }
    else
        throw std::runtime_error("FilterManager: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

bool FilterManager::checkDBContent (const std::string& dbcontent_name)
{
    if (!COMPASS::instance().dbContentManager().existsDBContent(dbcontent_name))
    {
        loginf << "failed because of non-existing dbcontbject '" << dbcontent_name << "'";
        return false;
    }

    DBContent& object = COMPASS::instance().dbContentManager().dbContent(dbcontent_name);

    if (!object.existsInDB())
    {
        loginf << "failed because of empty dbcontbject '" << dbcontent_name << "'";
        return false;
    }

    return true;
}

void FilterManager::checkSubConfigurables()
{
    // check for UTN filter

    string classid = "UTNFilter";

    if (std::find_if(filters_.begin(), filters_.end(),
                     [&classid](const unique_ptr<DBFilter>& x) { return x->classId() == classid;}) == filters_.end())
    { // no UTN filter
        Configurable::generateSubConfigurableFromConfig(Configuration::create(classid, classid+"0"));
    }

    classid = "TimestampFilter";

    if (std::find_if(filters_.begin(), filters_.end(),
                     [&classid](const unique_ptr<DBFilter>& x) { return x->classId() == classid;}) == filters_.end())
    {
        Configurable::generateSubConfigurableFromConfig(Configuration::create(classid, classid+"0"));
    }

    classid = "TrackerTrackNumberFilter";

    if (std::find_if(filters_.begin(), filters_.end(),
                     [&classid](const unique_ptr<DBFilter>& x) { return x->classId() == classid;}) == filters_.end())
    {
        Configurable::generateSubConfigurableFromConfig(Configuration::create(classid, classid+"0"));
    }

    classid = "RefTrajAccuracyFilter";
    if (std::find_if(filters_.begin(), filters_.end(),
                     [&classid](const unique_ptr<DBFilter>& x) { return x->classId() == classid;}) == filters_.end())
    {
        Configurable::generateSubConfigurableFromConfig(Configuration::create(classid, classid+"0"));
    }

    classid = "MLATRUFilter";

    if (std::find_if(filters_.begin(), filters_.end(),
                     [&classid](const unique_ptr<DBFilter>& x) { return x->classId() == classid;}) == filters_.end())
    {
        Configurable::generateSubConfigurableFromConfig(Configuration::create(classid, classid+"0"));
    }

   classid = "ExcludedTimeWindowsFilter";

    if (std::find_if(filters_.begin(), filters_.end(),
                     [&classid](const unique_ptr<DBFilter>& x) { return x->classId() == classid;}) == filters_.end())
    {
        Configurable::generateSubConfigurableFromConfig(Configuration::create(classid, classid+"0"));
    }
}

std::string FilterManager::getSQLCondition(const std::string& dbcontent_name)
{
    traced_assert(COMPASS::instance().dbContentManager().dbContent(dbcontent_name).loadable());

    std::stringstream ss;

    bool first = true;
    std::string condition_str;

    for (auto& filter : filters_)
    {
        logdbg << "filter " << filter->instanceId() << " active "
               << filter->getActive() << " filters " << dbcontent_name << " "
               << filter->filters(dbcontent_name);

        if (filter->getActive() && filter->filters(dbcontent_name))
        {
            condition_str = filter->getConditionString(dbcontent_name, first);

            logdbg << "filter " << filter->instanceId()
                   << " condition '" << condition_str << "'";

            ss << condition_str;
        }
    }

    loginf << "name " << dbcontent_name << " '" << ss.str() << "'";
    return ss.str();
}

unsigned int FilterManager::getNumFilters() { return filters_.size(); }

DBFilter* FilterManager::getFilter(unsigned int index)
{
    traced_assert(index < filters_.size());

    return filters_.at(index).get();
}

bool FilterManager::hasFilter (const std::string& name)
{
    auto it = find_if(filters_.begin(), filters_.end(), [name] (const unique_ptr<DBFilter>& f)
    { return f->getName() == name; } );

    return it != filters_.end();
}

DBFilter* FilterManager::getFilter (const std::string& name)
{
    auto it = find_if(filters_.begin(), filters_.end(), [name] (const unique_ptr<DBFilter>& f)
    { return f->getName() == name; } );

    traced_assert(it != filters_.end());

    return it->get();
}


void FilterManager::reset()
{
    for (unsigned int cnt = 0; cnt < filters_.size(); cnt++)
    {
        if (!filters_.at(cnt)->unusable())
            filters_.at(cnt)->reset();
    }
}

//void FilterManager::deleteFilterSlot(DBFilter* filter)
//{
//    auto it = find(filters_.begin(), filters_.end(), filter);
//    if (it == filters_.end())
//        throw std::runtime_error("FilterManager: deleteFilter: called with unknown filter");
//    else
//    {
//        filters_.erase(it);
//    }

//    emit changedFiltersSignal();
//}

void FilterManager::unshowViewPointSlot (const ViewableDataConfig* vp)
{
    loginf;
    traced_assert(vp);
}

void FilterManager::showViewPointSlot (const ViewableDataConfig* vp)
{
    loginf;
    traced_assert(vp);

    const json& data = vp->data();

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    // add all data source types that need loading
    if (data.contains(ViewPoint::VP_DS_TYPES_KEY)) // the listed ones should be loaded
    {
        const json& data_source_types  = data.at(ViewPoint::VP_DS_TYPES_KEY);

        std::set<std::string> ds_types = data_source_types.get<std::set<std::string>>();

        logdbg << "load " << ds_types.size() << " ds_types";

        ds_man.setLoadOnlyDSTypes(ds_types);
    }
    else // all should be loaded
    {
        logdbg << "load all ds_types";

        ds_man.setLoadDSTypes(true);
    }

    if (data.contains(ViewPoint::VP_SELECTED_RECNUMS_KEY))
    {
        auto& selected = data.at(ViewPoint::VP_SELECTED_RECNUMS_KEY);
        traced_assert(selected.is_array());
        std::vector<unsigned long> vec = selected.get<std::vector<unsigned long>>();

        COMPASS::instance().dbContentManager().storeSelectedRecNums(vec);
    }

    // add all data sources that need loading
    if (data.contains(ViewPoint::VP_DS_KEY)) // the listed ones should be loaded
    {
        const json& data_sources  = data.at(ViewPoint::VP_DS_KEY);

        std::map<unsigned int, std::set<unsigned int>> ds_ids
                = data_sources.get<std::map<unsigned int, std::set<unsigned int>>>(); // ds_id + line strs

        logdbg << "load " << ds_ids.size() << " ds_ids";

        ds_man.setLoadOnlyDataSources(ds_ids);
    }
    else // all should be loaded
    {
        logdbg << "load all ds_ids";

        ds_man.setLoadDataSources(true);
        ds_man.setLoadAllDataSourceLines();
    }

    // add filters
    use_filters_ = data.contains(ViewPoint::VP_FILTERS_KEY);

    disableAllFilters();

    if (data.contains(ViewPoint::VP_FILTERS_KEY))
    {
        const json& filters = data.at(ViewPoint::VP_FILTERS_KEY);

        logdbg << "filter data '" << filters.dump(4) << "'";

        traced_assert(filters.is_object());

        for (auto& fil_it : filters.get<json::object_t>())
        {
            std::string filter_name = fil_it.first;

            auto it = find_if(filters_.begin(), filters_.end(),
                              [filter_name] (const unique_ptr<DBFilter>& f) { return f->getName() == filter_name; } );

            if (it == filters_.end())
            {
                logerr << "filter '" << filter_name << "' not found";
                continue;
            }

            (*it)->setActive(true);
            (*it)->loadViewPointConditions(filters);
        }
    }

    if (widget_)
        widget_->updateUseFilters();
}

void FilterManager::setConfigInViewPoint (nlohmann::json& data)
{
    loginf;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    if (ds_man.dsTypeFiltered()) // ds types filters active
        data[ViewPoint::VP_DS_TYPES_KEY] = ds_man.wantedDSTypes(); // add all data sources that need loading

    if (ds_man.loadDataSourcesFiltered()) // ds filters active
        data[ViewPoint::VP_DS_KEY] = ds_man.getLoadDataSources(); // add all data sources that need loading

    // add filters
    if (use_filters_)
    {
        data[ViewPoint::VP_FILTERS_KEY] = json::object();
        json& filters = data.at(ViewPoint::VP_FILTERS_KEY);

        for (auto& fil_it : filters_)
        {
            if (fil_it->getActive())
                fil_it->saveViewPointConditions(filters);
        }

        loginf << "filters: '" << filters.dump(4) << "'";
    }
}

FilterManagerWidget* FilterManager::widget()
{
    if (!widget_)
    {
        widget_ = new FilterManagerWidget(*this);
        connect(this, &FilterManager::changedFiltersSignal, widget_, &FilterManagerWidget::updateFilters);
    }

    traced_assert(widget_);
    return widget_;
}

void FilterManager::databaseOpenedSlot()
{
    loginf;

    if (widget_)
        widget_->setDisabled(false);

    traced_assert(hasFilter("Timestamp"));
    getFilter("Timestamp")->reset();
}

void FilterManager::databaseClosedSlot()
{
    loginf;

    if (widget_)
        widget_->setDisabled(true);
}

void FilterManager::dataSourcesChangedSlot()
{
    loginf;

    if (hasFilter("Tracker Track Number"))
    {
        TrackerTrackNumberFilter* filter = dynamic_cast<TrackerTrackNumberFilter*>(getFilter("Tracker Track Number"));
        traced_assert(filter);
        filter->updateDataSourcesSlot();
    }
}

void FilterManager::appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current)
{
    loginf;

    for (auto& fil_it : filters_)
        fil_it->updateToAppMode(app_mode_current);
}

//void FilterManager::startedSlot()
//{
//    loginf;
//    createSubConfigurables();

//    std::string tmpstr = COMPASS::instance().interface().connection().identifier();
//    replace(tmpstr.begin(), tmpstr.end(), ' ', '_');

//    if (db_id_.compare(tmpstr) != 0)
//    {
//        loginf << "different db id, resetting filters";
//        reset();
//        db_id_ = tmpstr;
//    }

//    emit changedFiltersSignal();

//    if (widget_)
//        widget_->databaseOpenedSlot();
//}

void FilterManager::disableAllFilters ()
{
    for (auto& fil_it : filters_)
        if (!fil_it->unusable())
            fil_it->setActive(false);
}

void FilterManager::filterBuffers(std::map<std::string, std::shared_ptr<Buffer>>& data)
{
    loginf;

    vector<unsigned int> indexes_to_remove;

    for (auto& buf_it : data)
    {
        for (auto& fil_it : filters_)
        {
            if (fil_it->getActive())
            {
                indexes_to_remove = fil_it->filterBuffer(buf_it.first, buf_it.second);
                buf_it.second->removeIndexes(indexes_to_remove);
            }
        }
    }
}

void FilterManager::resetToStartupConfiguration()
{
    loginf;

    disableAllFilters();

    useFilters(false);
}

