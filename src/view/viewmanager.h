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
#include "dbcontent/variable/variableset.h"
#include "appmode.h"
#include "viewpresets.h"

#include <QObject>

#include "boost/date_time/posix_time/ptime.hpp"

class COMPASS;
class Buffer;
class ViewContainer;
class ViewContainerWidget;
//class ViewManagerWidget;
class View;
class ViewableDataConfig;
class ViewPointsWidget;
class ViewPointsReportGenerator;

class QWidget;
class QTabWidget;

/**
*/
class ViewManager : public QObject, public Configurable
{
    Q_OBJECT

  signals:
    void selectionChangedSignal();
    void unshowViewPointSignal (const ViewableDataConfig* vp);
    void showViewPointSignal (const ViewableDataConfig* vp);
    void reloadStateChanged();
    void automaticUpdatesChanged();
    void presetEdited(ViewPresets::EditAction ea);

  public slots:
    void selectionChangedSlot();

    void databaseOpenedSlot();
    void databaseClosedSlot();

    void loadingStartedSlot();
    // all data contained, also new one. requires_reset true indicates that all shown info should be re-created,
    // e.g. when data in the beginning was removed, or order of previously emitted data was changed, etc.
    void loadedDataSlot (const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset);
    void loadingDoneSlot(); // emitted when all dbconts have finished loading

    void appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current);

  public:
    struct Config
    {
        bool automatic_reload = true;
        bool automatic_redraw = true;
    };

    ViewManager(const std::string& class_id, const std::string& instance_id, COMPASS* compass);
    virtual ~ViewManager();

    void init(QTabWidget* main_tab_widget);
    void close();

    void clearDataInViews();

    void registerView(View* view);
    void unregisterView(View* view);
    bool isRegistered(View* view);

    ViewContainerWidget* addNewContainerWidget();

    // void deleteContainer (std::string instance_id);
    void removeContainer(std::string instance_id);
    void deleteContainerWidget(std::string instance_id);
    void removeContainerWidget(std::string instance_id);

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    void viewShutdown(View* view, const std::string& err = "");

    std::map<std::string, ViewContainer*> getContainers() { return containers_; }
    std::map<std::string, ViewContainerWidget*> getContainerWidgets() { return container_widgets_; }
    std::map<std::string, View*> getViews() { return views_; }
    View* latestView();
    ViewContainerWidget* latestViewContainer();
    
    dbContent::VariableSet getReadSet(const std::string& dbcontent_name);

    //@TODO: needed because of view container widget hack in ui_test_find.h
    //remove if no longer needed!
    ViewContainerWidget* containerWidget(const std::string& container_widget_name)
    {
        auto it = container_widgets_.find(container_widget_name);
        if (it == container_widgets_.end())
            return nullptr;

        return it->second;
    }

    ViewPointsWidget* viewPointsWidget() const;
    ViewPointsReportGenerator& viewPointsGenerator();

    void loadViewPoints();
    std::pair<bool, std::string> loadViewPoints(nlohmann::json json_obj);
    void clearViewPoints();
    void addViewPoints(const std::vector <nlohmann::json>& viewpoints);

    void setCurrentViewPoint (const ViewableDataConfig* viewable, 
                              bool load_blocking = false);
    void unsetCurrentViewPoint ();
    void doViewPointAfterLoad ();

    void selectTimeWindow(boost::posix_time::ptime ts_min, boost::posix_time::ptime ts_max);

    void showMainViewContainerAddView();

    std::map<std::string, std::string> viewClassList() const;

    unsigned int newViewNumber(const std::string& class_id);
    std::string newViewInstanceId(const std::string& class_id);
    std::string newViewName(const std::string& class_id);

    void disableDataDistribution(bool value);
    // disables propagation of data to the views. used when loading is performed for processing purposes

    bool isProcessingData() const;

    void resetToStartupConfiguration();

    bool isInitialized() const;

    bool viewPresetsEnabled() const;
    ViewPresets& viewPresets() { return presets_; }
    const ViewPresets& viewPresets() const { return presets_; }

    void notifyReloadStateChanged();
    bool reloadNeeded() const;
    void enableAutomaticReload(bool enable);
    void enableAutomaticRedraw(bool enable);
    bool automaticReloadEnabled() const;
    bool automaticRedrawEnabled() const;

    void updateFeatures();

    template<class T>
    std::vector<T*> viewsOfType()
    {
        std::vector<T*> views;
        for (const auto& v : views_)
        {
            T* vt = dynamic_cast<T*>(v.second);
            if (vt != nullptr)
                views.push_back(vt);
        }

        return views;
    }

protected:
    virtual void checkSubConfigurables();

    void enableStoredReadSets();
    void disableStoredReadSets();

    COMPASS& compass_;

    ViewPointsWidget* view_points_widget_{nullptr};

    Config config_;

    bool initialized_     = false;
    bool processing_data_ = false;
    bool reload_needed_   = false;

    QTabWidget* main_tab_widget_{nullptr};

    std::map<std::string, ViewContainer*> containers_;
    std::map<std::string, ViewContainerWidget*> container_widgets_;
    std::map<std::string, View*> views_;

    std::unique_ptr<ViewPointsReportGenerator> view_points_report_gen_;

    const ViewableDataConfig* current_viewable_ {nullptr};
    bool view_point_data_selected_ {false};

    unsigned int container_count_{0};

    std::map<std::string, std::string> view_class_list_; // class name -> name (without appended number)

    bool disable_data_distribution_ {false};

    bool use_tmp_stored_readset_ {false};
    std::map<std::string, dbContent::VariableSet> tmp_stored_readset_;

    ViewPresets presets_;
};
