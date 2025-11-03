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
#include "viewcontainerwidget.h"
#include "buffer.h"
#include "appmode.h"
#include "viewpresets.h"
#include "viewinfo.h"

#include <QObject>

#include <memory>
#include <map>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional/optional.hpp>

class ViewContainer;
class ViewWidget;
class QQWidget;
class Workflow;

class ViewableDataConfig;
class LatexVisitor;

/**
@brief Serves as base class for all views. Subclasses can be embedded in a ViewContainerWidget.

A view consists of two main components, a widget to display its content (not specifically a qt
widget) and a model manage its data. To add a new view to palantir, one will typically have to
introduce three new classes: a view class, a view widget class and a view model class. These classes
should be named similarly, e.g. *View, *ViewModel, *ViewWidget. Often these classes have to be
derived from a similar group of coherent classes. For example a new view based on a DBView will need
a model based on a DBViewModel and a widget based on a DBViewWidget.

A view always has a center widget, in which the ViewWidget is embedded. The central widget will then
again always be embedded inside a ViewContainerWidget, which shows the specific views in tabs.
 */
class View : public QObject, public Configurable
{
    Q_OBJECT
public:
    enum ViewUpdateFlags
    {
        VU_UpdateComponents = 1 << 0, // update view components (load state, toolbar activity, etc)
        VU_Redraw           = 1 << 1, // redraw view
        VU_Recompute        = 1 << 2, // recompute intermediate view data from buffers
        VU_Reload           = 1 << 3  // reload view (will overrule all other updates)
    };

    enum PresetError
    {
        NoError = 0,
        IncompatibleVersion,
        IncompatibleContent,
        ApplyFailed,
        GeneralError,
        UnknownError
    };

    View(const std::string& class_id, 
         const std::string& instance_id,
         ViewContainer* container,
         ViewManager& view_manager);
    virtual ~View();

    bool init();
    bool isInit() const { return init_; }

    virtual void databaseOpened();
    virtual void databaseClosed();

    virtual void loadingStarted();
    virtual void loadedData(const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset);
    virtual void loadingDone();
    virtual void clearData();
    virtual void appModeSwitch(AppMode app_mode_previous, AppMode app_mode_current);

    const std::string& getName() const;

    /// @brief Returns the view's central widget
    QWidget* getCentralWidget() { return central_widget_; }

    void enableInTabWidget(bool value);
    void showInTabWidget();

    virtual dbContent::VariableSet getSet(const std::string& dbcontent_name) = 0;

    void viewShutdown(const std::string& err);
    void emitSelectionChange();

    AppMode appMode() const { return app_mode_; }
    time_t created() const { return creation_time_; }

    const ViewWidget* getViewWidget() const { traced_assert(widget_); return widget_; }

    virtual void accept(LatexVisitor& v) = 0;

    virtual std::set<std::string> acceptedAnnotationFeatureTypes() const { return {}; }

    QImage renderData() const;
    QImage renderView() const;

    void setExporting(bool ok);
    bool hasScreenshotContent() const;

    bool reloadNeeded() const;
    bool redrawNeeded() const;
    bool updateNeeded() const;

    void updateView();
    void updateComponents();
    virtual void updateFeatures() {}
    
    PresetError applyPreset(const ViewPresets::Preset& preset, 
                            std::vector<MissingKey>* missing_subconfig_keys = nullptr,
                            std::vector<MissingKey>* missing_param_keys = nullptr,
                            std::string* error_msg = nullptr);
    const ViewPresets::Preset* activePreset() const;
    bool presetChanged() const;

    ViewInfos viewInfos() const;
    nlohmann::json viewInfoJSON() const;

    //shortcut update flags
    static const int VU_PureRedraw       = VU_Redraw;                                      // just redraw
    static const int VU_RecomputedRedraw = VU_Redraw | VU_Recompute;                       // recompute + redraw
    static const int VU_Complete         = VU_Redraw | VU_Recompute | VU_UpdateComponents; // complete view update (no reload though)

signals:
    void selectionChangedSignal();  // do not emit manually, call emitSelectionChange()
    void presetChangedSignal();

public slots:
    void selectionChangedSlot();
    virtual void unshowViewPointSlot (const ViewableDataConfig* vp)=0;
    virtual void showViewPointSlot (const ViewableDataConfig* vp)=0;

protected:
    virtual void updateSelection() = 0;
    virtual bool init_impl() { return true; }

    virtual void onConfigurationChanged(const std::vector<std::string>& changed_params) override final;
    virtual void onConfigurationChanged_impl(const std::vector<std::string>& changed_params) {};

    virtual void onModified() override final;

    virtual void viewManagerReloadStateChanged();
    virtual void viewManagerAutoUpdatesChanged();

    virtual bool refreshScreenOnNeededReload() const { return false; }

    virtual ViewInfos viewInfos_impl() const { return ViewInfos(); }
    virtual void viewInfoJSON_impl(nlohmann::json& info) const {}

    void constructWidget();
    void setWidget(ViewWidget* widget);

    //can be used by derived views to signal certain changes in the view
    void notifyViewUpdateNeeded(int flags, bool add = true);
    void notifyRedrawNeeded(bool add = true);
    void notifyReloadNeeded(bool add = true);
    void notifyRefreshNeeded();

    void updateView(int flags);

    /// @brief Returns the view's widget, override this method in derived classes.
    ViewWidget* getWidget() { traced_assert(widget_); return widget_; }

    ViewManager& view_manager_;

    /// The view's widget
    ViewWidget* widget_ {nullptr};
    /// The ViewContainerWidget the view is currently embedded in
    ViewContainer* container_ {nullptr};
    /// The widget containing the view's widget
    QWidget* central_widget_ {nullptr};

private:
    friend class ViewVariable;

    unsigned int getInstanceKey();

    void runAutomaticUpdates();

    void presetEdited(ViewPresets::EditAction ea);

    std::string name_;

    bool    init_ = false;
    AppMode app_mode_;
    time_t  creation_time_;

    boost::optional<int> issued_update_;

    bool preset_changed_ = false;
    boost::optional<ViewPresets::Preset> active_preset_;

    /// Static member counter
    static unsigned int cnt_;
};
