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

#include "viewmanager.h"
#include "loadcontroller.h"
#include "livecontroller.h"
#include "compass.h"
#include "buffer.h"
#include "logger.h"
#include "stringconv.h"
#include "view.h"
#include "viewcontainer.h"
#include "viewcontainerwidget.h"
#include "viewpoint.h"
#include "viewpointgenerator.h"
#include "viewabledataconfig.h"
#include "dbinterface.h"
#include "viewpointswidget.h"
#include "filtermanager.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentdataset.h"
#include "dbcontent/loadoperation.h"
#include "dbcontent/livedatafeed.h"
#include "dbcontent/dbcontentdataengine.h"
#include "db_context_manager.h"
#include "dbcontent/loadrequest.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "viewpointsreportgenerator.h"
#include "viewpointsreportgeneratordialog.h"
#include "util/timeconv.h"
#include "util/number.h"
#include "viewpoint_commands.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "geographicview_commands.h"
#endif
#include "global.h"

#include "json.hpp"

#include <QMessageBox>
#include <QWidget>
#include <QMetaType>
#include <QApplication>
#include <QCoreApplication>
#include <QEventLoop>
#include <QScopedValueRollback>
#include <QTabWidget>
#include <QTimer>

#include "traced_assert.h"

#define SCAN_PRESETS

using namespace Utils;
using namespace nlohmann;
using namespace std;

namespace
{
    // Pump only the events that are safe during loading dispatch:
    //   paint / timer / WM-ping replies (so the WM does not flag us as unresponsive).
    // Excluded:
    //   user input  - modal dialog already blocks it; double-belt against any non-modal load path.
    //   sockets     - keeps RT command TCP from firing a second load mid-dispatch.
    inline void pumpLoadingEvents()
    {
        QCoreApplication::processEvents(
            QEventLoop::ExcludeUserInputEvents | QEventLoop::ExcludeSocketNotifiers);
    }
}

/**
*/
ViewManager::ViewManager(nlohmann::json& config, COMPASS& compass)
    : Configurable(config, &compass), compass_(compass)
{
    logdbg;

    qRegisterMetaType<ViewPoint*>("ViewPoint*");

    init_view_point_commands(compass_);

#if USE_EXPERIMENTAL_SOURCE == true
    init_geographic_view_commands();
#endif

    registerParameter("automatic_reload", &config_.automatic_reload, Config().automatic_reload);
    registerParameter("automatic_redraw", &config_.automatic_redraw, Config().automatic_redraw);

    load_controller_.reset(new LoadController(compass_));
    live_controller_.reset(new LiveController(compass_, *this));
}

/**
*/
void ViewManager::init(QTabWidget* main_tab_widget)
{
    logdbg;

    traced_assert(main_tab_widget);
    traced_assert(!main_tab_widget_);
    traced_assert(!initialized_);

    main_tab_widget_ = main_tab_widget;

    connect (&compass_, &COMPASS::appModeSwitchSignal, this, &ViewManager::appModeSwitchSlot);

    // view point stuff

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    view_points_widget_ = new ViewPointsWidget(*this);

    QApplication::restoreOverrideCursor();

    traced_assert(view_points_widget_);

    FilterManager& filter_man = compass_.filterManager();

    connect (this, &ViewManager::showViewPointSignal, &filter_man, &FilterManager::showViewPointSlot);
    connect (this, &ViewManager::unshowViewPointSignal, &filter_man, &FilterManager::unshowViewPointSlot);

    view_class_list_.insert({"HistogramView", "Histogram View"});
    view_class_list_.insert({"TableView", "Table View"});

#if USE_EXPERIMENTAL_SOURCE == true
    view_class_list_.insert({"GeographicView", "Geographic View"});
#endif

    view_class_list_.insert({"ScatterPlotView", "Scatterplot View"});
    view_class_list_.insert({"GridView", "Grid View"});

#ifdef SCAN_PRESETS
    //scan view presets
    if (!presets_.scanForPresets())
        logwrn << "view presets could not be loaded";
#endif

    connect(&presets_, &ViewPresets::presetEdited, this, &ViewManager::presetEdited);

    initialized_ = true;

    createSubConfigurables();
    updateFeatures();
}

/**
*/
void ViewManager::loadViewPoints()
{
    traced_assert(view_points_widget_);
    view_points_widget_->loadViewPoints();
}

/**
*/
void ViewManager::close()
{
    logdbg;
    initialized_ = false;

    logdbg << "deleting container widgets";
    while (container_widgets_.size())
    {
        auto first_it = container_widgets_.begin();
        logdbg << "deleting container widget " << first_it->first;
        delete first_it->second; // deletes the respective view container, which removes itself from this
        container_widgets_.erase(first_it);
    }

    logdbg << "deleting containers size " << containers_.size();
    while (containers_.size())
    {
        auto first_it = containers_.begin();
        logdbg << "deleting container " << first_it->first;
        delete first_it->second;
        //containers_.erase(first_it);  // TODO CAUSES SEGFAULT, FIX THIS
    }

    // if (view_points_widget_)
    // {
        //view_points_widget_->tableModel()->saveViewPoints();
        //delete view_points_widget_;
        view_points_widget_ = nullptr;
    //}

    view_points_report_gen_ = nullptr;

    logdbg << "done";
}

/**
*/
ViewManager::~ViewManager()
{
    logdbg;

    traced_assert(!container_widgets_.size());
    traced_assert(!containers_.size());
    traced_assert(!initialized_);

    view_points_widget_ = nullptr;
}

/**
*/
void ViewManager::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);
    const auto& instance_name = Configuration::getInstanceName(child_json);

    logdbg << "class_name " << class_name << " instance_name "
           << instance_name;

    traced_assert(initialized_);

    if (class_name == "ViewContainer")
    {
        ViewContainer* container =
                new ViewContainer(child_json, *this, this, main_tab_widget_, 0);
        traced_assert(containers_.count(instance_name) == 0);
        containers_.insert(std::pair<std::string, ViewContainer*>(instance_name, container));

        unsigned int number = String::getAppendedInt(instance_name);
        if (number >= container_count_)
            container_count_ = number;
    }
    else if (class_name == "ViewContainerWidget")
    {
        ViewContainerWidget* container_widget =
                new ViewContainerWidget(child_json, *this);
        traced_assert(containers_.count(container_widget->viewContainer().instanceName()) == 0);
        containers_.insert(std::pair<std::string, ViewContainer*>(
                               container_widget->viewContainer().instanceName(), &container_widget->viewContainer()));
        traced_assert(container_widgets_.count(instance_name) == 0);
        container_widgets_.insert(
                    std::pair<std::string, ViewContainerWidget*>(instance_name, container_widget));

        unsigned int number = String::getAppendedInt(instance_name);
        if (number >= container_count_)
            container_count_ = number;
    }
    else if (class_name == "ViewPointsReportGenerator")
    {
        traced_assert(!view_points_report_gen_);

        view_points_report_gen_.reset(new ViewPointsReportGenerator(child_json, this));
        traced_assert(view_points_report_gen_);
    }
    else
        throw std::runtime_error("ViewManager: generateSubConfigurable: unknown class_name " +
                                 class_name);
//    if (widget_)
//        widget_->update();
}

/**
*/
void ViewManager::checkSubConfigurables()
{
    if (containers_.size() == 0)
    {
        generateSubConfigurableFromConfig("ViewContainer", "ViewContainer0");
    }

    if (!view_points_report_gen_)
    {
        generateSubConfigurableFromConfig("ViewPointsReportGenerator", "ViewPointsReportGenerator0");
    }
}

/**
*/
void ViewManager::enableStoredReadSets()
{
    loginf;

    for (const auto& cont_it : compass_.dbContentManager())
    {
        logdbg << "stored readset for '" << cont_it.first << "'";
        tmp_stored_readset_[cont_it.first] = getReadSet(cont_it.first);
    }

    use_tmp_stored_readset_ = true;
}

/**
*/
void ViewManager::disableStoredReadSets()
{
    loginf;

    use_tmp_stored_readset_ = false;
    tmp_stored_readset_.clear();
}

/**
*/
dbContent::VariableSet ViewManager::getReadSet(const std::string& dbcontent_name)
{
    if (use_tmp_stored_readset_)
    {
        logdbg << "stored readset for '" << dbcontent_name << "'";
        traced_assert(tmp_stored_readset_.count(dbcontent_name));
        return tmp_stored_readset_.at(dbcontent_name);
    }

    dbContent::VariableSet read_set;
    dbContent::VariableSet read_set_tmp;

    for (auto view_it : views_)
    {
        read_set_tmp = view_it.second->getSet(dbcontent_name);
        read_set.add(read_set_tmp);
    }
    return read_set;
}

/**
*/
ViewPointsWidget* ViewManager::viewPointsWidget() const
{
    traced_assert(view_points_widget_);
    return view_points_widget_;
}

/**
*/
ViewPointsReportGenerator& ViewManager::viewPointsGenerator()
{
    traced_assert(view_points_report_gen_);
    return *view_points_report_gen_;
}

/**
*/
std::pair<bool, std::string> ViewManager::loadViewPoints(nlohmann::json json_obj)
{
    try
    {
        //check if valid JSON
        std::string err;
        bool json_ok = ViewPoint::isValidJSON(json_obj, "", &err, true);
        if (!json_ok)
            return std::make_pair(false, err);

        DBInterface& db_interface = compass_.dbInterface();

        //delete existing viewpoints
        if (db_interface.existsViewPointsTable() && db_interface.viewPoints().size())
            db_interface.deleteAllViewPoints();

        traced_assert(json_obj.contains(ViewPoint::VP_COLLECTION_ARRAY_KEY));
        
        //add new ones
        json& view_points = json_obj.at(ViewPoint::VP_COLLECTION_ARRAY_KEY);
        traced_assert(view_points.size());

        unsigned int id;
        for (auto& vp_it : view_points.get<json::array_t>())
        {
            traced_assert(vp_it.contains(ViewPoint::VP_ID_KEY));

            id = vp_it.at(ViewPoint::VP_ID_KEY);

            if (!vp_it.contains(ViewPoint::VP_STATUS_KEY))
                vp_it[ViewPoint::VP_STATUS_KEY] = "open";

            db_interface.setViewPoint(id, vp_it.dump());
        }

        //reload viewpoints
        loadViewPoints();

        loginf << "imported " << std::to_string(view_points.size()) << " view points";
    }
    catch (const std::exception& ex)
    {
        return std::make_pair(false, ex.what());
    }
    catch (...)
    {
        return std::make_pair(false, "unknown error");
    }
    
    return std::make_pair(true, "");  
}

/**
*/
void ViewManager::clearViewPoints()
{
    DBInterface& db_interface = compass_.dbInterface();

            //delete existing viewpoints
    if (db_interface.existsViewPointsTable() && db_interface.viewPoints().size())
        db_interface.deleteAllViewPoints();

    viewPointsWidget()->clearViewPoints();

}

/**
*/
void ViewManager::addViewPoints(const std::vector <nlohmann::json>& viewpoints)
{
    viewPointsWidget()->addViewPoints(viewpoints);
}

/**
*/
void ViewManager::setCurrentViewPoint (ViewableDataConfig* viewable,
                                       bool load_blocking)
{
    if (current_viewable_)
        unsetCurrentViewPoint();

    current_viewable_ = viewable;

    view_point_data_selected_ = false;

    logdbg << "setting current view point data: '"
    << viewable->data().dump(4) << "'";

    emit showViewPointSignal(current_viewable_);

    // After every view has consumed the viewpoint (and so registered any
    // annotation), bring a compatible view to the front. Switching before
    // the signal would make a not-yet-shown widget try to render mid-setup
    // (the GridView duplicates its legend / skips the chart on that path).
    activateCompatibleViewTabs(current_viewable_);

    reload(load_blocking);
}

/**
*/
void ViewManager::setCurrentViewPoint (std::shared_ptr<ViewableDataConfig> viewable,
                                       bool load_blocking)
{
    traced_assert(viewable);

    // Assigned only after the raw overload ran: that one starts with
    // unsetCurrentViewPoint(), which would immediately release the reference
    // again. The by-value parameter keeps the viewable alive meanwhile, which
    // also covers a blocking load completing inside the call.
    setCurrentViewPoint(viewable.get(), load_blocking);

    current_viewable_owned_ = std::move(viewable);
}

/**
*/
void ViewManager::activateCompatibleViewTabs(const ViewableDataConfig* viewable)
{
    if (!viewable)
        return;

    auto features = ViewPointGenVP::scanForFeatures(viewable->data());
    if (features.empty())
        return;

    std::set<std::string> feature_types;
    for (const auto& f : features)
    {
        if (f.feature_json.is_object()
            && f.feature_json.contains(ViewPointGenFeature::FeatureTypeFieldType))
        {
            feature_types.insert(
                f.feature_json[ViewPointGenFeature::FeatureTypeFieldType].get<std::string>());
        }
    }
    if (feature_types.empty())
        return;

    auto supports = [&feature_types](View* v) {
        if (!v)
            return false;
        auto accepted = v->acceptedAnnotationFeatureTypes();
        for (const auto& t : feature_types)
            if (accepted.count(t))
                return true;
        return false;
    };

    for (const auto& entry : containers_)
    {
        ViewContainer* container = entry.second;
        if (!container)
            continue;

        View* current = container->currentView();
        if (current && supports(current))
            continue;

        for (const auto& v : container->getViews())
        {
            if (supports(v.get()))
            {
                View* view_to_show = v.get();
                view_to_show->showInTabWidget();

                // QtCharts occasionally leaves the chart unrendered on a
                // tab's first show this session - the user then sees the
                // legend stretched across the chart area and only a manual
                // resize triggers a redraw. Forcing a geometry refresh after
                // the show event has run does the same thing programmatically.
                QWidget* central = view_to_show->getCentralWidget();
                if (central)
                {
                    QTimer::singleShot(0, central, [central]()
                    {
                        central->updateGeometry();
                        central->update();
                    });
                }
                break;
            }
        }
    }
}

/**
*/
void ViewManager::unsetCurrentViewPoint ()
{
    if (current_viewable_)
    {
        emit unshowViewPointSignal(current_viewable_);

        current_viewable_ = nullptr;

        // released last: the views drop their pointer in the signal above
        current_viewable_owned_.reset();

        view_point_data_selected_ = false;
    }
}

/**
*/
void ViewManager::doViewPointAfterLoad ()
{
    logdbg;

    if (!current_viewable_)
    {
        logdbg << "no viewable";
        return; // nothing to do
    }

    if (view_point_data_selected_)
    {
        logdbg << "data already selected";
        return; // already done, this is a re-load
    }

    traced_assert(view_points_widget_);

    const json& data = current_viewable_->data();

    logdbg << "data '" << data.dump(4) << "'";

    bool vp_contains_timestamp = data.contains(ViewPoint::VP_TIMESTAMP_KEY);
    boost::posix_time::ptime vp_timestamp;
    bool vp_contains_time_window = data.contains(ViewPoint::VP_TIME_WIN_KEY);
    float vp_time_window;
    boost::posix_time::ptime vp_ts_min, vp_ts_max;

    if (!vp_contains_timestamp)
    {
        loginf << "no time given";
        return; // nothing to do
    }
    else
    {
        traced_assert(data.at(ViewPoint::VP_TIMESTAMP_KEY).is_string());
        vp_timestamp = Time::fromString(data.at(ViewPoint::VP_TIMESTAMP_KEY));

        loginf << "time " << Time::toString(vp_timestamp);
    }

    if (vp_contains_time_window)
    {
        traced_assert(data.at(ViewPoint::VP_TIME_WIN_KEY).is_number());
        vp_time_window = data.at(ViewPoint::VP_TIME_WIN_KEY);
        vp_ts_min = vp_timestamp - Time::partialSeconds(vp_time_window / 2.0);
        vp_ts_max = vp_timestamp + Time::partialSeconds(vp_time_window / 2.0);

        loginf << "time window min " << Time::toString(vp_ts_min)
               << " max " << Time::toString(vp_ts_max);
    }

    DBContentManager& dbcont_man = compass_.dbContentManager();

    bool selection_changed = false;
    for (auto& dbcont_it : dbcont_man)
    {
        std::string dbcontent_name = dbcont_it.first;

        if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_timestamp_))
        {
            logerr << "required variables missing in " << dbcontent_name;
            continue;
        }

        const dbContent::Variable& ts_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_timestamp_);

        if (currentBuffers().count(dbcont_it.first))
        {
            std::shared_ptr<Buffer> buffer = currentBuffers().at(dbcont_it.first);

            traced_assert(buffer->has<bool>(dbcontent_vars::selected_var_.name()));
            NullableVector<bool>& selected_vec = buffer->get<bool>(dbcontent_vars::selected_var_.name());

            traced_assert(buffer->has<boost::posix_time::ptime>(ts_var.name()));
            NullableVector<boost::posix_time::ptime>& tods = buffer->get<boost::posix_time::ptime>(ts_var.name());

            unsigned int buffer_size = buffer->size();

            bool ts_null;
            boost::posix_time::ptime timestamp;

            for (unsigned int cnt =0; cnt < buffer_size; ++cnt)
            {
                ts_null = tods.isNull(cnt);

                if (ts_null)
                    continue; // nothing to do

                timestamp = tods.get(cnt);

                if (vp_contains_time_window)
                {
                    if (timestamp >= vp_ts_min && timestamp <= vp_ts_max)
                    {
                        selected_vec.set(cnt, true);
                        selection_changed = true;

                        logdbg << "time " << timestamp << " selected ";
                    }
                }
                else if (vp_contains_timestamp && timestamp == vp_timestamp)
                {
                    selected_vec.set(cnt, true);
                    selection_changed = true;
                }
            }
        }
    }

    view_point_data_selected_ = true;

    if (selection_changed)
    {
        loginf << "selection changed";
        emit selectionChangedSignal();
    }
}

/**
*/
void ViewManager::selectTimeWindow(boost::posix_time::ptime ts_min, boost::posix_time::ptime ts_max)
{
    loginf << "ts_min " << ts_min << " ts_max " << ts_max;

    DBContentManager& dbcont_man = compass_.dbContentManager();

    bool selection_changed = false;
    for (auto& dbcont_it : dbcont_man)
    {
        std::string dbcontent_name = dbcont_it.first;

        if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_timestamp_))
        {
            logerr << "required variables missing, quitting";
            continue;
        }

        const dbContent::Variable& ts_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_timestamp_);

        if (currentBuffers().count(dbcont_it.first))
        {
            std::shared_ptr<Buffer> buffer = currentBuffers().at(dbcont_it.first);

            traced_assert(buffer->has<bool>(dbcontent_vars::selected_var_.name()));
            NullableVector<bool>& selected_vec = buffer->get<bool>(dbcontent_vars::selected_var_.name());

            traced_assert(buffer->has<boost::posix_time::ptime>(ts_var.name()));
            NullableVector<boost::posix_time::ptime>& ts_vec = buffer->get<boost::posix_time::ptime>(ts_var.name());

            unsigned int buffer_size = buffer->size();

            bool tod_null;
            boost::posix_time::ptime timestamp;

            for (unsigned int cnt =0; cnt < buffer_size; ++cnt)
            {
                tod_null = ts_vec.isNull(cnt);

                if (tod_null)
                    continue; // nothing to do

                timestamp = ts_vec.get(cnt);

                if (timestamp >= ts_min && timestamp <= ts_max)
                {
                    selected_vec.set(cnt, true);
                    selection_changed = true;

                    logdbg << "time " << timestamp << " selected ";

                }
            }
        }
    }

    view_point_data_selected_ = true;

    if (selection_changed)
    {
        loginf << "selection changed";
        emit selectionChangedSignal();
    }
}

/**
*/
void ViewManager::showMainViewContainerAddView()
{
    traced_assert(containers_.count("ViewContainer0"));
    containers_.at("ViewContainer0")->showAddViewMenuSlot();
}

/**
*/
std::map<std::string, std::string> ViewManager::viewClassList() const
{
    return view_class_list_;
}

/**
*/
unsigned int ViewManager::newViewNumber(const std::string& class_name)
{
    int max_number = -1;
    int tmp;

    for (auto& view_it : views_)
    {
        if (view_it.second->className() != class_name)
            continue;

        tmp = String::getAppendedInt(view_it.second->instanceName());

        if (tmp > max_number)
            max_number = tmp;
    }

    return max_number + 1;
}

std::string ViewManager::newViewInstanceId(const std::string& class_name)
{
    return class_name + to_string(newViewNumber(class_name));
}

std::string ViewManager::newViewName(const std::string& class_name)
{
    traced_assert(view_class_list_.count(class_name));
    return view_class_list_.at(class_name) + " " + to_string(newViewNumber(class_name));
}

bool ViewManager::isProcessingData() const
{
    return processing_data_;
}

void ViewManager::resetToStartupConfiguration()
{
    loginf;

    enableStoredReadSets();

    // logdbg << "deleting container widgets";
    // while (container_widgets_.size())
    // {
    //     auto first_it = container_widgets_.begin();
    //     logdbg << "deleting container widget " << first_it->first;

    //     first_it->second->setTmpDisableRemoveConfigOnDelete(true);
    //     delete first_it->second; // deletes the respective view container, which removes itself from this

    //     container_widgets_.erase(first_it);
    // }

    // logdbg << "deleting containers size " << containers_.size();
    // while (containers_.size())
    // {
    //     auto first_it = containers_.begin();
    //     logdbg << "deleting container " << first_it->first;

    //     first_it->second->setTmpDisableRemoveConfigOnDelete(true);
    //     delete first_it->second;
    //     //containers_.erase(first_it);  // TODO CAUSES SEGFAULT, FIX THIS
    // }

    logdbg << "resettings containers";

    for (auto& cw : container_widgets_)
        cw.second->setVisible(false);

    for (auto& c : containers_)
        c.second->resetToStartupConfiguration();

    for (auto& cw : container_widgets_)
        cw.second->setVisible(true);

    //logdbg << "view points generator";
    //view_points_report_gen_->setTmpDisableRemoveConfigOnDelete(true);
    //view_points_report_gen_ = nullptr;

    //createSubConfigurables();

    disableStoredReadSets();
}

/**
*/
bool ViewManager::isInitialized() const
{
    return initialized_;
}

/**
*/
ViewContainerWidget* ViewManager::addNewContainerWidget()
{
    logdbg;
    
    container_count_++;
    std::string container_widget_name = "ViewWindow" + std::to_string(container_count_);

    generateSubConfigurableFromConfig("ViewContainerWidget", container_widget_name);

    traced_assert(container_widgets_.count(container_widget_name) == 1);

    return container_widgets_.at(container_widget_name);
}

/**
*/
void ViewManager::clearDataInViews()
{
    for (auto& view_it : views_)
    {
        view_it.second->clearData();
    }
}

/**
*/
void ViewManager::registerView(View* view)
{
    logdbg;
    traced_assert(view);
    traced_assert(!isRegistered(view));
    views_[view->instanceName()] = view;
}

/**
*/
void ViewManager::unregisterView(View* view)
{
    logdbg << view->getName().c_str();
    traced_assert(view);
    traced_assert(isRegistered(view));

    std::map<std::string, View*>::iterator it;

    it = views_.find(view->instanceName());
    views_.erase(it);
}

/**
*/
bool ViewManager::isRegistered(View* view)
{
    logdbg;
    traced_assert(view);

    std::map<std::string, View*>::iterator it;

    it = views_.find(view->instanceName());

    return !(it == views_.end());
}

/**
*/
void ViewManager::removeContainer(std::string instance_name)
{
    std::map<std::string, ViewContainer*>::iterator it;

    logdbg << "instance " << instance_name;

    it = containers_.find(instance_name);

    if (it != containers_.end())
    {
        containers_.erase(it);

        return;
    }

    throw std::runtime_error("ViewManager: removeContainer:  key not found");
}

/**
*/
void ViewManager::removeContainerWidget(std::string instance_name)
{
    std::map<std::string, ViewContainerWidget*>::iterator it;

    logdbg << "instance " << instance_name;

    it = container_widgets_.find(instance_name);

    if (it != container_widgets_.end())
    {
        container_widgets_.erase(it);

        return;
    }

    throw std::runtime_error("ViewManager: removeContainer: key not found");
}

/**
*/
void ViewManager::deleteContainerWidget(std::string instance_name)
{
    std::map<std::string, ViewContainerWidget*>::iterator it;

    logdbg << "instance " << instance_name;

    it = container_widgets_.find(instance_name);

    if (it != container_widgets_.end())
    {
        it->second->deleteLater();
        container_widgets_.erase(it);

        return;
    }

    throw std::runtime_error("ViewManager: deleteContainerWidget: key not found");
}

/**
*/
void ViewManager::viewShutdown(View* view, const std::string& err)
{
    delete view;

    if (err.size())
        QMessageBox::critical(QApplication::activeWindow(), "View Shutdown", QString::fromStdString(err));
}

/**
*/
void ViewManager::selectionChangedSlot()
{
    loginf;
    emit selectionChangedSignal();
}

/**
*/
void ViewManager::databaseOpenedSlot()
{
    loginf;

    for (auto& view_it : views_)
        view_it.second->databaseOpened();
}

/**
*/
void ViewManager::databaseClosedSlot()
{
    loginf;

    unsetCurrentViewPoint();

    // drop the displayed data set: it belongs to the DB being closed, and currentBuffers()
    // would keep handing it out. Cancels a still-running load (see setCurrentSource).
    setCurrentSource(nullptr);
    clearSelectedRecNums(); // rec nums of the closed DB must not carry into the next one

    clearDataInViews();

    for (auto& view_it : views_)
        view_it.second->databaseClosed();
}

/**
*/
void ViewManager::loadingStartedSlot()
{
    if (processing_data_)
    {
        loginf << "re-entry detected (currently in '" << current_dispatch_
               << "'), deferring via queued connection";
        QMetaObject::invokeMethod(this, &ViewManager::loadingStartedSlot, Qt::QueuedConnection);
        return;
    }
    QScopedValueRollback<bool> guard(processing_data_, true);
    QScopedValueRollback<std::string> name_guard(current_dispatch_, "loadingStartedSlot");

    //reset reload flag
    reload_needed_ = false;
    loading_done_dispatched_ = false;

    // start the load dialog/cursor here (fired off the op's startedSignal, so after the
    // engine's single-op wait - the outgoing load is already cleaned up). Only for an
    // offline/priming LoadOperation; the live feed (fresh entry) raises no dialog.
    if (auto* op = dynamic_cast<LoadOperation*>(current_source_.get()))
        load_controller_->begin(*op);

    loginf << "begin, " << views_.size() << " views";

    for (auto& view_it : views_)
        view_it.second->loadingStarted();

    // bookend for external UI/view chrome consumers (owned here, not the manager)
    emit loadingStartedSignal();

    loginf << "end";
}

// driven by the current source's dataChangedSignal. names empty = synthetic
// finalize event (completion handled by loadingDoneSlot). reset true indicates
// all shown info should be re-created.
void ViewManager::sourceDataChangedSlot(const std::vector<std::string>& names, bool reset, bool last)
{
    if (!current_source_)
        return;

    // an empty-names event only matters as the last=true finalize (drive providers to
    // finalize); a non-final empty event has nothing to do
    if (names.empty() && !last)
        return;

    if (processing_data_)
    {
        loginf << "re-entry detected (currently in '" << current_dispatch_
               << "'), deferring via queued connection";
        // @TODO: the deferred payload carries no source identity - if current_source_ were
        // swapped before it runs, it would be applied to the new source. Safe today only by
        // FIFO ordering (live) and load modality (offline).
        QMetaObject::invokeMethod(this,
            [this, names, reset, last]() {
                sourceDataChangedSlot(names, reset, last);
            }, Qt::QueuedConnection);
        return;
    }
    QScopedValueRollback<bool> guard(processing_data_, true);
    QScopedValueRollback<std::string> name_guard(current_dispatch_, "sourceDataChangedSlot");

    if (loading_done_dispatched_)
        logwrn << "called AFTER loadingDoneSlot completed - lifecycle contract violated";

    // restore the carried-over selection onto the freshly arrived buffers before the
    // views read them (selection carry-over across reloads)
    if (!names.empty())
        applyCarriedSelection(names);

    loginf << "begin, " << views_.size() << " views, requires_reset " << reset;

    using namespace boost::posix_time;
    ptime tmp_time;

    for (auto& view_it : views_)
    {
        tmp_time = microsec_clock::local_time();

        // the single data-delivery callback: the view pulls buffers/index from the source
        // in its updateFromSource_impl (finalizing on last=true); an item provider (geo,
        // owned by the derived widget) is fed here too. names empty on the last=true finalize.
        view_it.second->updateFromSource(*current_source_, names, reset, last);

        loginf << "view " << view_it.first << " took "
               << String::timeStringFromDouble((microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true);
    }

    // distribution side-effects: data-source loaded counts (per arrival) + status-widget
    // trigger (once, on the finalize event)
    if (current_source_)
        compass_.dbContextManager().setLoadedCounts(current_source_->loadedCounts());
    if (last)
        emit dataDistributedSignal(reset);

    loginf << "end";
}

/**
 */
void ViewManager::setCurrentSource(std::shared_ptr<DBContentDataSet> source)
{
    if (current_source_ == source)
        return;

    // an outgoing operation that is still running is abandoned here: cancel it, since its
    // buffers will never be displayed. The load UX ends itself off the op's finishedSignal
    // (LoadController::opFinishedSlot) - it must, because we disconnect the op below and
    // loadingDoneSlot would never run for it.
    if (auto* op = dynamic_cast<LoadOperation*>(current_source_.get()))
    {
        if (op->isRunning())
        {
            logwrn << "abandoning a still-running load";
            op->cancel();

            // loadingDoneSlot consumes the carry-over, and this op will never reach it -
            // leaving it staged would restore this load's rec nums onto a later one
            carried_selection_.clear();
        }
    }

    if (current_source_)
        disconnect(current_source_.get(), nullptr, this, nullptr);

    current_source_ = std::move(source);

    if (current_source_)
    {
        connect(current_source_.get(), &DBContentDataSet::dataChangedSignal,
                this, &ViewManager::sourceDataChangedSlot);

        // an offline/priming LoadOperation drives the load bookends off its own state
        // machine; the live feed (not a LoadOperation) has none - its bookends come from
        // appModeSwitchSlot instead
        if (auto* op = dynamic_cast<LoadOperation*>(current_source_.get()))
        {
            connect(op, &LoadOperation::startedSignal, this, &ViewManager::loadingStartedSlot);
            connect(op, &LoadOperation::finishedSignal, this, &ViewManager::loadingDoneSlot);
        }
    }
}

/**
 */
void ViewManager::reload(bool blocking, bool measure_performance)
{
    // while live is running the feed owns the data and is the current source; a full reload
    // would swap the source away from it and lose the accumulated live data (live view
    // refreshes are handled in-widget, View::notifyViewUpdateNeeded -> liveReload). Paused
    // displays a LoadOperation like offline, so a reload there is a normal load.
    if (compass_.appMode() == AppMode::LiveRunning)
    {
        logwrn << "reload ignored while live is running";
        return;
    }

    LoadRequest req = LoadRequest::standard();
    req.measure_db_performance_ = measure_performance;

    issueLoad(req, blocking);
}

/**
 * Paused live = an offline display over the current database: the live window prime no longer
 * bounds what is shown, and the user's filters apply as offline. From here the load button and
 * the filters (incl. the time window) work exactly as offline - reload() only refuses while
 * live is actually running. Posted from appModeSwitchSlot, so the mode may have moved on again
 * (a quick pause/resume) by the time this runs.
 */
void ViewManager::loadPausedDisplay()
{
    if (compass_.appMode() != AppMode::LivePaused)
    {
        loginf << "no longer paused, skipping the paused display load";
        return;
    }

    issueLoad(LoadRequest::standard());
}

/**
 * Runs a request as the new display source: carries the selection over, clears the views and
 * makes the operation current_source_. The dialog/cursor are raised by loadingStartedSlot off
 * the op's startedSignal (after the engine's single-op wait), not here - so a superseded load
 * can't leak them.
 */
void ViewManager::issueLoad(const LoadRequest& req, bool blocking)
{
    // capture the outgoing selection BEFORE swapping the source away, so it can be
    // restored onto the freshly loaded buffers as they arrive (selection carry-over)
    captureSelection();

    clearDataInViews();

    auto op = std::make_shared<LoadOperation>(compass_.dbContentManager(), req);
    setCurrentSource(op);

    compass_.dbContentManager().dataEngine().load(op);
    if (blocking)
        op->wait();
}

/**
 */
const std::map<std::string, std::shared_ptr<Buffer>>& ViewManager::currentBuffers() const
{
    static const std::map<std::string, std::shared_ptr<Buffer>> empty;
    return current_source_ ? current_source_->buffers() : empty;
}

/**
 */
bool ViewManager::hasMaxLatency() const
{
    return live_controller_->hasMaxLatency();
}

/**
 */
boost::posix_time::time_duration ViewManager::maxLatency() const
{
    return live_controller_->maxLatency();
}

/**
 * ASTERIX live watchdog: force one live tick (no new inserts). Delegates to the live session.
 */
void ViewManager::forceLiveUpdate()
{
    live_controller_->processLiveModeSlot();
}

/**
 * Reads the current source's selected_ flags into carried_selection_ so the selection
 * survives the upcoming source swap. A selection already staged (e.g. from a view point)
 * is kept as-is.
 */
void ViewManager::captureSelection()
{
    if (!carried_selection_.empty())
        return; // already staged (e.g. from a view point)

    if (!current_source_)
        return;

    for (const auto& buf_it : current_source_->buffers())
    {
        auto& buffer = buf_it.second;

        if (!buffer->has<bool>(dbcontent_vars::selected_var_.name())
            || !buffer->has<unsigned long>(dbcontent_vars::meta_var_rec_num_.name()))
            continue;

        NullableVector<bool>& selected_vec = buffer->get<bool>(dbcontent_vars::selected_var_.name());
        NullableVector<unsigned long>& rec_num_vec =
            buffer->get<unsigned long>(dbcontent_vars::meta_var_rec_num_.name());

        size_t data_size = selected_vec.contentSize();
        for (unsigned int cnt = 0; cnt < data_size; ++cnt)
        {
            if (!selected_vec.isNull(cnt) && selected_vec.get(cnt))
                carried_selection_[buf_it.first].insert(rec_num_vec.get(cnt));
        }
    }
}

/**
 * Restores the carried selection onto a single freshly arrived buffer (matched rec nums
 * are set and removed from the carry set).
 */
void ViewManager::restoreSelectionInto(const std::string& dbcontent_name, Buffer& buffer)
{
    if (!carried_selection_.count(dbcontent_name))
        return;

    auto& sel_recnums = carried_selection_.at(dbcontent_name);
    if (sel_recnums.empty())
        return;

    if (!buffer.has<bool>(dbcontent_vars::selected_var_.name())
        || !buffer.has<unsigned long>(dbcontent_vars::meta_var_rec_num_.name()))
        return;

    NullableVector<bool>& selected_vec = buffer.get<bool>(dbcontent_vars::selected_var_.name());
    NullableVector<unsigned long>& rec_num_vec =
        buffer.get<unsigned long>(dbcontent_vars::meta_var_rec_num_.name());

    std::map<unsigned long, unsigned int> unique_rec_nums =
        rec_num_vec.uniqueValuesWithIndexes(sel_recnums); // value -> index

    for (auto& rec_num_it : unique_rec_nums)
    {
        selected_vec.set(rec_num_it.second, true);
        sel_recnums.erase(rec_num_it.first);
    }
}

/**
 * Restores the carried selection onto the given contents of the current source (called
 * from sourceDataChangedSlot as buffers arrive, before the views are driven).
 */
void ViewManager::applyCarriedSelection(const std::vector<std::string>& names)
{
    if (carried_selection_.empty() || !current_source_)
        return;

    for (const auto& name : names)
    {
        auto it = current_source_->buffers().find(name);
        if (it != current_source_->buffers().end())
            restoreSelectionInto(name, *it->second);
    }
}

/**
 * Stages an explicit selection (by rec num) for the next load, replacing any current
 * selection. Used by FilterManager / view-point selection.
 */
void ViewManager::storeSelectedRecNums(const std::vector<unsigned long>& selected)
{
    clearSelectedRecNums(); // replace any current selection

    DBContentManager& dbcont_man = compass_.dbContentManager();
    for (auto rec_num : selected)
        carried_selection_[dbcont_man.dbContentWithId(Utils::Number::recNumGetDBContId(rec_num))].insert(rec_num);
}

/**
 * Clears the staged carry selection and the selected_ flags on the current source.
 */
void ViewManager::clearSelectedRecNums()
{
    carried_selection_.clear();

    if (!current_source_)
        return;

    for (const auto& buf_it : current_source_->buffers())
    {
        auto& buffer = buf_it.second;
        if (buffer->has<bool>(dbcontent_vars::selected_var_.name()))
            buffer->get<bool>(dbcontent_vars::selected_var_.name()).setAll(false);
    }
}

/**
 * @TODO: the op's terminal state is ignored - Failed (empty buffers, error in result()) and
 * Cancelled (partial buffers) are finalized like a successful load, so the user sees an empty
 * or truncated result with no indication.
 */
void ViewManager::loadingDoneSlot() // emitted when all dbconts have finished loading
{
    if (processing_data_)
    {
        loginf << "re-entry detected (currently in '" << current_dispatch_
               << "'), deferring via queued connection";
        QMetaObject::invokeMethod(this, &ViewManager::loadingDoneSlot, Qt::QueuedConnection);
        return;
    }
    {
        QScopedValueRollback<bool> guard(processing_data_, true);
        QScopedValueRollback<std::string> name_guard(current_dispatch_, "loadingDoneSlot");

        // apply the pending view point onto the freshly loaded data BEFORE the views redraw
        // (preserves the former finishLoading ordering: set selection -> loadingDone redraw)
        doViewPointAfterLoad();

        loginf << "begin, " << views_.size() << " views";

        load_controller_->beginViewPhase(static_cast<unsigned int>(views_.size()));

        using namespace boost::posix_time;
        ptime tmp_time;
        ptime loop_start = microsec_clock::local_time();

        // Threshold: only pump events between views once the loop has been running
        // for a while. Short loops (typical UI test loads, ~1-2 s total) finish
        // before the threshold and never pump - this keeps queued RT commands from
        // interleaving with view dispatch and breaking UI test injection. Long
        // loads (the original "WM unresponsive" case, 12+ s) cross the threshold
        // and start pumping, restoring responsiveness for the user.
        constexpr int pump_threshold_ms = 3000;

        for (auto& view_it : views_)
        {
            tmp_time = microsec_clock::local_time();
            view_it.second->loadingDone();
            loginf << "view " << view_it.first << " took "
                   << String::timeStringFromDouble((microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true);
            load_controller_->advanceViewPhase();

            auto elapsed_ms = (microsec_clock::local_time() - loop_start).total_milliseconds();
            if (elapsed_ms > pump_threshold_ms)
                pumpLoadingEvents();
        }

        // selection carry-over consumed for this load; drop any leftover (unmatched) rec nums
        carried_selection_.clear();
    } // processing_data_ released here

    // Close the dialog OUTSIDE the re-entrancy guard (processing_data_ == false) so its
    // processEvents flushes any deferred view redraw (e.g. the geo view's) now, while the
    // dialog is still up - not after the load is marked done. Matches the pre-refactor
    // finishLoading ordering. loading_done_dispatched_ is set only after this drain, so the
    // flushed finalize is not misflagged as a late event.
    load_controller_->end();

    loading_done_dispatched_ = true;

    // bookend for external UI/view chrome consumers (owned here, not the manager)
    emit loadingDoneSignal();

    loginf << "end";
}

/**
 * Live-session entry bookend. The lean live counterpart of loadingStartedSlot: view chrome +
 * the external bookend, but none of the offline-op work (no load dialog/cursor - the live
 * feed is not a LoadOperation). Data has already been distributed via refreshDisplay by now.
 */
void ViewManager::beginLiveSession()
{
    if (processing_data_)
    {
        loginf << "re-entry detected (currently in '" << current_dispatch_
               << "'), deferring via queued connection";
        QMetaObject::invokeMethod(this, &ViewManager::beginLiveSession, Qt::QueuedConnection);
        return;
    }
    QScopedValueRollback<bool> guard(processing_data_, true);
    QScopedValueRollback<std::string> name_guard(current_dispatch_, "beginLiveSession");

    reload_needed_ = false;
    loading_done_dispatched_ = false;

    for (auto& view_it : views_)
        view_it.second->loadingStarted();

    emit loadingStartedSignal();
}

/**
 * Live-session exit bookend. The lean live counterpart of loadingDoneSlot: view chrome + the
 * external bookend, but none of the offline-op finalize work (no view point, progress phase,
 * pumping, dialog, or selection carry-over). current_source_ is already null here (cleared in
 * appModeSwitchSlot before this runs), so no source-driven distribution can interleave.
 */
void ViewManager::endLiveSession()
{
    if (processing_data_)
    {
        loginf << "re-entry detected (currently in '" << current_dispatch_
               << "'), deferring via queued connection";
        QMetaObject::invokeMethod(this, &ViewManager::endLiveSession, Qt::QueuedConnection);
        return;
    }
    {
        QScopedValueRollback<bool> guard(processing_data_, true);
        QScopedValueRollback<std::string> name_guard(current_dispatch_, "endLiveSession");

        for (auto& view_it : views_)
            view_it.second->loadingDone();
    }

    loading_done_dispatched_ = true;

    emit loadingDoneSignal();
}

/**
*/
void ViewManager::appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current)
{
    loginf << "app_mode " << compass_.appModeStr();

    const bool was_live = isLiveSession(app_mode_previous);
    const bool now_live = isLiveSession(app_mode_current);

    // drive the live-session state machine + swap current_source_. The feed is the source only
    // while running: a pause hands the display to its own offline load, a resume catches up on
    // the pause window and points back at the feed, an exit drops it.
    if (app_mode_current == AppMode::LiveRunning)
    {
        // leave the paused display FIRST: this cancels and disconnects a still-running paused
        // load, so it cannot drive bookends or the progress dialog while the catch-up below
        // pumps events. The feed emits nothing until refreshDisplay, so pointing at it early
        // distributes nothing.
        setCurrentSource(live_controller_->feedPtr());

        // both entries load the live window from the DB first - a fresh entry to open on the
        // recent history, a resume to catch up on what accumulated while paused - and the
        // ticks then accumulate on top
        if (app_mode_previous == AppMode::LivePaused)
            live_controller_->resumeSession();
        else
            live_controller_->startSession();

        // show that window + distribute, without a DB delete here (the every-tick bound
        // resumes on the next real tick)
        live_controller_->refreshDisplay();
    }
    else if (app_mode_current == AppMode::LivePaused)
    {
        // the DB keeps ingesting silently; only the display leaves live. The paused display
        // is loaded from the DB below - queued, so it runs after this app-mode switch has
        // been delivered to every receiver (filters, data sources, main window) and not
        // nested inside the signal chain. Mirrors the pre-rewrite ordering, where COMPASS
        // emitted appModeSwitchSignal first and only then ran its pause load.
        live_controller_->pauseSession();

        QMetaObject::invokeMethod(this, &ViewManager::loadPausedDisplay, Qt::QueuedConnection);
    }
    else if (was_live) // -> Offline
    {
        setCurrentSource(nullptr);
        live_controller_->stopSession(); // disarm ticks + drop the live cache
    }

    for (auto& view_it : views_)
    {
        view_it.second->appModeSwitch(app_mode_previous, app_mode_current);

        if (app_mode_current == AppMode::LiveRunning)
            view_it.second->enableInTabWidget(view_it.second->className() == "GeographicView");
        else
            view_it.second->enableInTabWidget(true);
    }

    // live-session bookends: entering the session opens the "one long load cycle", leaving it
    // closes it. These are distinct from the offline load bookends (loadingStarted/DoneSlot,
    // driven off the LoadOperation's state machine) - a live session raises no offline dialog/
    // view-point/progress-phase work, so it gets its own lean bookend pair.
    if (!was_live && now_live)
        beginLiveSession();
    else if (was_live && !now_live)
        endLiveSession();
}

/**
*/
View* ViewManager::latestView()
{
    time_t latest = std::numeric_limits<time_t>::min();
    View* latest_view = nullptr;

    for (const auto& elem : views_)
    {
        if (elem.second->created() > latest)
        {
            latest      = elem.second->created();
            latest_view = elem.second;
        }
    }

    return latest_view;
}

/**
*/
ViewContainerWidget* ViewManager::latestViewContainer()
{
    time_t latest = std::numeric_limits<time_t>::min();
    ViewContainerWidget* latest_container = nullptr;

    for (const auto& elem : container_widgets_)
    {
        if (elem.second->viewContainer().created() > latest)
        {
            latest           = elem.second->viewContainer().created();
            latest_container = elem.second;
        }
    }

    return latest_container;
}

/**
*/
bool ViewManager::viewPresetsEnabled() const
{
#ifdef SCAN_PRESETS
    return true;
#else
    return false;
#endif
}

/**
 * Notifies the view manager that the reload state in a view has changed, 
 * determines the new global reload state, and informs all views about it.
 */
void ViewManager::notifyReloadStateChanged()
{
    //query views if one of them needs to reload
    bool reload_needed = false;
    for (const auto& elem : views_)
    {
        if (!elem.second->reloadNeeded())
            continue;

        logdbg << "view '" << elem.first << "' needs to reload";

        reload_needed = true;
        break;
    }

    logdbg << "reload needed before: " << reload_needed_ << ", now: " << reload_needed;

    //reload state has not changed? => just return
    if (reload_needed_ == reload_needed)
        return;

    //update global reload flag
    reload_needed_ = reload_needed;

    logdbg << "emitting new reload state " << reload_needed_;

    //inform views about changed reload state
    emit reloadStateChanged();
}

/**
 * Checks if a reload is needed (has been notified by a view).
 */
bool ViewManager::reloadNeeded() const
{
    return reload_needed_;
}

/**
 * Enables/disables automatic reloading in the view manager and informs all views about it.
 */
void ViewManager::enableAutomaticReload(bool enable)
{
    if (config_.automatic_reload == enable)
        return;

    config_.automatic_reload = enable;

    //inform views about changed auto-update state
    emit automaticUpdatesChanged();
}

/**
 * Enables/disables automatic redrawing in the view manager and informs all views about it.
 */
void ViewManager::enableAutomaticRedraw(bool enable)
{
    if (config_.automatic_redraw == enable)
        return;

    config_.automatic_redraw = enable;

    //inform about changed auto-update state
    emit automaticUpdatesChanged();
}

/**
 */
bool ViewManager::automaticReloadEnabled() const
{
    return config_.automatic_reload;
}

/**
 */
bool ViewManager::automaticRedrawEnabled() const
{
    return config_.automatic_redraw;
}

/**
*/
void ViewManager::updateFeatures()
{
    for (const auto& cw : container_widgets_)
        if (cw.second)
            cw.second->updateFeatures();
    
    for (const auto& v : views_)
        if (v.second)
            v.second->updateFeatures();
}

// void ViewManager::saveViewAsTemplate (View *view, std::string template_name)
//{
//    //view->saveConfigurationAsTemplate("Template"+view->getInstanceId());
//    saveTemplateConfiguration (view, template_name);
//}

