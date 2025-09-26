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

#include <memory>

#include <QWidget>

#include "configurable.h"
#include "appmode.h"
#include "ui_test_testable.h"
#include "json_fwd.hpp"

#include <boost/optional.hpp>

//class EventProcessor;
class View;
class ViewToolWidget;
class ViewDataWidget;
class ViewConfigWidget;
class ViewToolSwitcher;
class ViewLoadStateWidget;
class ViewPresetWidget;
class ViewInfoWidget;

class QSplitter;
class QLayout;

/**
@brief Base class for a views "display" component.

A view widget bases on a QWidget and provides a way to display
the views data, which is held in a model.

A ViewWidget consists of a standard layout with a set of typical components located at predefined regions of the widget.
Some of them need to be derived and set manually in the derived ViewWidget's constructor, some are pregenerated and can
be used directly.
_____________________________________________________________________________
|_ViewPresetWidget____|________ViewToolWidget________________________________|           
|                                              ||                            |
| ViewDataWidget                               || ViewConfigWidget           |
|                                              ||                            |
|                                              ||                            |
|                                              ||                            |
|                                              ||                            |
|                                              ||                            |
|                                              ||                            |
|                DATA AREA                     <>       CONFIG AREA          |
|                                              ||                            |
|                                              ||                            |
|                                              ||                            |
|                                              ||                            |
|                                              ||____________________________|
|                                              ||ViewInfoWidget              |
|                                              ||                            |
|                                              ||____________________________|
|                                              ||ViewLoadStateWidget         |
|______________________________________________||____________________________|
|'Lower widget'                                                              |
|____________________________________________________________________________|

ViewDataWidget [derive]: Visualizes the data of the view (e.g. a graph). Located on the left side of the widget.
Needs to be derived and then set via setDataWidget() in the constructor of the derived class.

ViewConfigWidget [derive]: Implements a configuration area for the view located on the right side of the widget.
Needs to be derived and then set via setConfigWidget() in the constructor of the derived class.

ViewPresetWidget: Widget for selecting and editing view presets. No need to derive.

ViewToolWidget: A toolbar located above the ViewDataWidget, holding the view's needed tool buttons and actions.
This is a generic class which doesn't need to be derived, but is rather filled in the derived ViewWidget's constructor and
provided with all needed callbacks. Interacts with the ViewDataWidget to switch the view's active tool and handles tool
interaction like activating, deactivating and cancelling tools.

ViewLoadStateWidget: A widget located below the configuration area. Provides state information for the view
and means to update the view manually. Interacts with the ViewWidget and the ViewDataWidget to e.g.issue reloads and redraws,
ot to obtain state information from them.

'Lower widget': This is a widget residing on the bottom along the whole ViewWidget's width.
Any QWidget derived class can be set as the 'lower widget' via setLowerWidget().
The widget's container is only visible if the widget is set.

The ViewWidget acts both to generate the basic layout and to handle interaction between all these components.
It also serves as the View's main interface to all ui and display functionality.
*/
class ViewWidget : public QWidget, public Configurable, public ui_test::UITestable
{
    Q_OBJECT

signals:
    void viewRefreshed();

public:
    ViewWidget(const std::string& class_id, const std::string& instance_id,
               Configurable* config_parent, View* view, QWidget* parent = nullptr);
    virtual ~ViewWidget();

    void toggleConfigWidget();

    void updateToolWidget();
    void updateLoadState();
    void updateInfoWidget();
    void updateComponents();

    bool refreshView();
    void clearData();

    void loadingStarted();
    void loadingDone();
    void redrawStarted();
    void redrawDone();
    void appModeSwitch(AppMode app_mode);
    void configChanged();

    ViewDataWidget* getViewDataWidget() { traced_assert(data_widget_); return data_widget_; }
    const ViewDataWidget* getViewDataWidget() const { traced_assert(data_widget_); return data_widget_; }
    ViewConfigWidget* getViewConfigWidget() { traced_assert(config_widget_); return config_widget_; }
    const ViewConfigWidget* getViewConfigWidget() const { traced_assert(config_widget_); return config_widget_; }

    std::string loadedMessage() const;

    void init();
    bool isInit() const { return init_; }

    bool isVariableSetLoaded() const;

    View* getView() { return view_; }

    nlohmann::json viewInfoJSON() const;

    boost::optional<QString> uiGet(const QString& what = QString()) const override final;
    nlohmann::json uiGetJSON(const QString& what = QString()) const override final;
    void uiRefresh() override final;

    QImage renderContents();

    static QIcon getIcon(const std::string& fn);

    static const int DataWidgetStretch;
    static const int ConfigWidgetStretch;

protected:
    ViewToolWidget* getViewToolWidget() { traced_assert(tool_widget_); return tool_widget_; }
    const ViewToolWidget* getViewToolWidget() const { traced_assert(tool_widget_); return tool_widget_; }
    ViewToolSwitcher* getViewToolSwitcher() { traced_assert(tool_switcher_); return tool_switcher_.get(); }
    const ViewToolSwitcher* getViewToolSwitcher() const { traced_assert(tool_switcher_); return tool_switcher_.get(); }
    ViewLoadStateWidget* getViewLoadStateWidget() { traced_assert(state_widget_); return state_widget_; }
    const ViewLoadStateWidget* getViewLoadStateWidget() const { traced_assert(state_widget_); return state_widget_; }
    ViewPresetWidget* getViewPresetWidget() { return preset_widget_; }
    const ViewPresetWidget* getViewPresetWidget() const { return preset_widget_; }
    ViewInfoWidget* getViewInfoWidget() { return info_widget_; }
    const ViewInfoWidget* getViewInfoWidget() const { return info_widget_; }

    /**
     * Reimplement to provide the ViewLoadStateWidget with view specific load information.
     */
    virtual std::string loadedMessage_impl() const { return ""; }

    /**
     * Reimplement for specific initialization behavior.
     */
    virtual void init_impl() const {}

    /**
     * Reimplement to add additional information to the view's view info.
     */
    virtual void viewInfoJSON_impl(nlohmann::json& info) const {}

    /**
     * Reimplement to show the info widget or not.
     */
    virtual bool showInfoWidget() { return false; }

    void setDataWidget(ViewDataWidget* w);
    void setConfigWidget(ViewConfigWidget* w);
    void setLowerWidget(QWidget* w);

private:
    void createStandardLayout();
    void connectWidgets();

    static const int PresetSelectionWidth  = 200;
    static const int PresetSelectionSpacer = 20;

    /// The view the widget is part of
    View* view_ = nullptr;

    //containers
    QSplitter*      main_splitter_           = nullptr;
    QWidget*        data_widget_container_   = nullptr;
    QWidget*        config_widget_container_ = nullptr;
    QWidget*        lower_widget_container_  = nullptr;
    QWidget*        right_widget_            = nullptr;

    //view widget components
    ViewToolWidget*      tool_widget_   = nullptr;
    ViewDataWidget*      data_widget_   = nullptr;
    ViewConfigWidget*    config_widget_ = nullptr;
    ViewInfoWidget*      info_widget_   = nullptr;
    ViewLoadStateWidget* state_widget_  = nullptr;
    ViewPresetWidget*    preset_widget_ = nullptr;
    
    QWidget*             lower_widget_  = nullptr;

    std::unique_ptr<ViewToolSwitcher> tool_switcher_;

    bool init_ = false;
};
