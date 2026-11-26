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

#include "viewwidget.h"
#include "view.h"
#include "viewtoolwidget.h"
#include "viewmodel.h"
#include "viewrenderwidget.h"
#include "viewconfigwidget.h"
#include "viewtoolswitcher.h"
#include "viewtool.h"
#include "viewscreenshotdialog.h"
#include "viewloadstatewidget.h"
#include "viewpresetwidget.h"
#include "viewinfowidget.h"
#include "files.h"
#include "ui_test_common.h"
#include "dbcontentmanager.h"
#include "dbcontent.h"
#include "viewmanager.h"
#include "buffer.h"
#include "dbcontent/variable/variable.h"
#include "logger.h"

#include "json.hpp"

#include <QApplication>
#include <QVBoxLayout>
#include <QSplitter>
#include <QAction>
#include <QSignalBlocker>
#include <QSettings>
#include <QPainter>
#include <QScopedValueRollback>
#include <QResizeEvent>

const int ViewWidget::DataWidgetStretch   = 3;
const int ViewWidget::ConfigWidgetStretch = 1;

const int ViewWidget::ToolScreenshot  = ViewTool::HostIdBase;
const int ViewWidget::ToolConfigPanel = ViewTool::HostIdBase + 1;

ViewWidget::ViewWidget(View* view, QWidget* parent)
    : QWidget(parent),
      view_  (view)
{
    //generate and set a nice object name which can be used to identify the view widget in the object hierarchy
    UI_TEST_OBJ_NAME(this, QString::fromStdString(view->instanceName()))

    setContentsMargins(0, 0, 0, 0);

    createStandardLayout();
}

/**
@brief Destructor.
*/
ViewWidget::~ViewWidget()
{
    //the model outlives the view widget, the render widget can still trigger it while it is destroyed
    if (model_)
        model_->disconnect(this);

    //a hidden panel has the size 0 in the splitter state, the last state with the panel is kept
    if (main_splitter_ && !right_widget_->isHidden())
    {
        traced_assert(view_);

        QSettings settings("COMPASS", view_->instanceName().c_str());
        settings.setValue("mainSplitterSizes", main_splitter_->saveState());
    }
}

/**
 * Creates the standard view layout.
 */
void ViewWidget::createStandardLayout()
{
    traced_assert(view_);

    //create main layout
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(0);
    main_layout->setMargin(0);

    setLayout(main_layout);

    //create top layout
    top_layout_ = new QHBoxLayout;
    top_layout_->setContentsMargins(0, 0, 0, 0);
    top_layout_->setSpacing(0);
    top_layout_->setMargin(0);

    main_layout->addLayout(top_layout_);

    //create preset selection
    if (view_->viewManager().viewPresetsEnabled())
    {
        preset_widget_ = new ViewPresetWidget(view_, this);
        preset_widget_->setFixedWidth(PresetSelectionWidth);

        top_layout_->addWidget(preset_widget_);
        top_layout_->addSpacerItem(new QSpacerItem(PresetSelectionSpacer, 1, QSizePolicy::Fixed, QSizePolicy::Preferred));
    }

    //the tool widget follows in setRenderWidget(), it needs the tool switcher

    //central layout
    QHBoxLayout* central_layout = new QHBoxLayout;
    central_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->addLayout(central_layout);
    
    //create lower widget container
    {
        lower_widget_container_ = new QWidget;
        lower_widget_container_->setVisible(false);
        lower_widget_container_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        main_layout->addWidget(lower_widget_container_);
    }

    //main splitter (containing a left and a right widget)
    main_splitter_ = new QSplitter;
    main_splitter_->setObjectName("main_splitter");
    main_splitter_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    main_splitter_->setOrientation(Qt::Horizontal);
    main_splitter_->setContentsMargins(0, 0, 0, 0);

    QWidget* left_widget = new QWidget;
    left_widget->setContentsMargins(0, 0, 0, 0);

    right_widget_ = new QWidget;
    right_widget_->setObjectName("config_panel");
    right_widget_->setContentsMargins(0, 0, 0, 0);

    QVBoxLayout* left_layout = new QVBoxLayout;
    left_layout->setContentsMargins(0, 0, 0, 0);
    left_layout->setMargin(0);

    QVBoxLayout* right_layout = new QVBoxLayout;
    right_layout->setContentsMargins(0, 0, 0, 0);
    right_layout->setMargin(0);
    right_layout->setSpacing(0);

    left_widget->setLayout(left_layout);
    right_widget_->setLayout(right_layout);

    main_splitter_->addWidget(left_widget);
    main_splitter_->addWidget(right_widget_);
    main_splitter_->handle(1)->setObjectName("main_splitter_handle");

    //create data widget container in left widget
    {
        QSizePolicy size_policy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        //size_policy.setHorizontalStretch(DataWidgetStretch);

        data_widget_container_ = new QWidget;
        data_widget_container_->setSizePolicy(size_policy);
        data_widget_container_->setContentsMargins(0, 0, 0, 0);

        //an explicit minimum replaces the minimum size hints of the render widgets in the layout
        data_widget_container_->setMinimumWidth(DataMinWidth);

        left_layout->addWidget(data_widget_container_);
    }

    //create config widget container in right widget
    {
        QSizePolicy size_policy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        //size_policy.setHorizontalStretch(ConfigWidgetStretch);

        config_widget_container_ = new QWidget;
        config_widget_container_->setSizePolicy(size_policy);
        config_widget_container_->setContentsMargins(0, 0, 0, 0);

        right_layout->addWidget(config_widget_container_);
    }

    //add main splitter to central layout and restore state from config
    QSettings settings("COMPASS", view_->instanceName().c_str());

#if 0
    main_splitter_->restoreState(settings.value("mainSplitterSizes").toByteArray());
#else
    if (settings.value("mainSplitterSizes").isValid())
    {
        main_splitter_->restoreState(settings.value("mainSplitterSizes").toByteArray());
    }
    else
    {
        //wtf qt...
        //https://stackoverflow.com/questions/43831474/how-to-equally-distribute-the-width-of-qsplitter
        main_splitter_->setSizes({ 100000 * DataWidgetStretch, 100000 * ConfigWidgetStretch });
    }
#endif

    //create info widget
    {
        info_widget_ = new ViewInfoWidget(view_, right_widget_);
        info_widget_->setVisible(false);
        info_widget_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

        right_layout->addWidget(info_widget_);
    }

    //create load state widget in right widget
    {
        state_widget_ = new ViewLoadStateWidget(this, right_widget_);

        right_layout->addWidget(state_widget_);
    }

    central_layout->addWidget(main_splitter_);

    setFocusPolicy(Qt::StrongFocus);

    //deactivate collapsing of children
    main_splitter_->setChildrenCollapsible(false); 
    for (int i = 0; i < main_splitter_->count(); ++i)
        main_splitter_->setCollapsible(i, false);
}


/**
 * View initialization. Called by View on view creation.
*/
void ViewWidget::init()
{
    //init should only be called once
    if (isInit())
        throw std::runtime_error("ViewWidget: init: Called twice");

    //check if all relevant widgets have been constructed
    traced_assert(model_);
    traced_assert(render_widget_);
    traced_assert(config_widget_);
    traced_assert(tool_widget_);
    traced_assert(state_widget_);
    traced_assert(info_widget_);

    //add screenshot button and toggle button for config widget
    registerHostTools();

    //set info widget visibility
    info_widget_->setVisible(showInfoWidget());

    //call derived
    init_impl();

    init_ = true;

    //update view components after init is done
    updateComponents();
}

/**
 */
void ViewWidget::runPostInit()
{
    config_widget_->onInit();
}

/**
 */
void ViewWidget::setRenderWidget(ViewModel* model,
                                 ViewRenderWidget* render_widget,
                                 const UpdateDisplayFunc& update_display,
                                 bool initial_display)
{
    if (!model || !render_widget || !update_display)
        throw std::runtime_error("ViewWidget: setRenderWidget: Null pointer passed");
    if (!data_widget_container_)
        throw std::runtime_error("ViewWidget: setRenderWidget: No container to add to");
    if (render_widget_)
        throw std::runtime_error("ViewWidget: setRenderWidget: Already set");

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);

    data_widget_container_->setLayout(layout);

    layout->addWidget(render_widget);

    model_          = model;
    render_widget_  = render_widget;
    update_display_ = update_display;

    //the UI tests address the data area by this name
    render_widget_->setObjectName("datawidget");

    //the render widget owns its tools
    tool_switcher_ = &render_widget_->toolSwitcher();

    //create tool widget
    {
        tool_widget_ = new ViewToolWidget(tool_switcher_, this);
        tool_widget_->setContentsMargins(0, 0, 0, 0);

        top_layout_->addWidget(tool_widget_);
    }

    connect(model_, &ViewModel::displayDataChangedSignal, this, &ViewWidget::displayDataChanged);
    connect(model_, &ViewModel::processingFinishedSignal, this, &ViewWidget::modelProcessingFinished);

    // the display before the first load does not count as drawn
    if (initial_display)
    {
        update_display_();
        render_widget_->resetDrawState();
    }

    //try to connect
    connectWidgets();
}

/**
 * Registers the tools of the view widget itself: the screenshot button and the toggle button of the
 * config widget.
 */
void ViewWidget::registerHostTools()
{
    ViewTool screenshot;
    screenshot.id       = ToolScreenshot;
    screenshot.name     = "Save Screenshot";
    screenshot.icon     = getIcon("camera.png");
    screenshot.kind     = ViewTool::Kind::Trigger;
    screenshot.callback = [ this ] (bool)
    {
        ViewScreenshotDialog dlg(view_, tool_widget_);
        dlg.exec();
    };

    tool_widget_->addSpacerIfNone();
    tool_widget_->addSeparatorIfValid();
    tool_switcher_->registerTool(screenshot);

    ViewTool config_panel;
    config_panel.id       = ToolConfigPanel;
    config_panel.name     = "Toggle Configuration Panel";
    config_panel.key      = Qt::Key_C;
    config_panel.icon     = getIcon("configuration.png");
    config_panel.kind     = ViewTool::Kind::Toggle;
    config_panel.callback = [ this ] (bool on)
    {
        updateConfigPanel(on ? config_panel::UserAction::Open : config_panel::UserAction::Close);
    };
    config_panel.update   = [ this ] (QAction* action)
    {
        //the state without the callback, it would toggle the panel again
        QSignalBlocker blocker(action);
        action->setChecked(config_panel::isShown(config_panel_state_));
    };

    tool_widget_->addSpacerIfNone();
    tool_widget_->addSeparatorIfValid();
    tool_switcher_->registerTool(config_panel);
}

/**
 */
void ViewWidget::setConfigWidget(ViewConfigWidget* w)
{
    if (!w)
        throw std::runtime_error("ViewWidget::setConfigWidget: Null pointer passed");
    if (!config_widget_container_)
        throw std::runtime_error("ViewWidget::setConfigWidget: No container to add to");

    //remove old config widget?
    if (config_widget_)
    {
        config_widget_container_->layout()->removeWidget(config_widget_);

        //setLayout() can only be called on widgets without layout
        //the old layout can be removed by just deleting it
        delete config_widget_container_->layout();

        delete config_widget_;
        config_widget_ = nullptr;
    }

    //add new config widget
    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);

    config_widget_container_->setLayout(layout);

    layout->addWidget(w);

    config_widget_ = w;

    //try to connect
    connectWidgets();
}

/**
 * Tries to connect data and config widget.
 * (Note: Could also be done in init() actually)
 */
void ViewWidget::connectWidgets()
{
    if (config_widget_ && render_widget_)
    {
        connect(this, &ViewWidget::displayChangedSignal, config_widget_, &ViewConfigWidget::onDisplayChange);

        if (info_widget_)
            connect(this, &ViewWidget::displayChangedSignal, info_widget_, &ViewInfoWidget::onDisplayChange);
    }
}

/**
 */
void ViewWidget::setLowerWidget(QWidget* w)
{
    if (!w)
        throw std::runtime_error("ViewWidget::setLowerWidget: Null pointer passed");
    if (!lower_widget_container_)
        throw std::runtime_error("ViewWidget::setLowerWidget: No container to add to");
    if (lower_widget_)
        throw std::runtime_error("ViewWidget::setLowerWidget: Already set");

    lower_widget_ = w;
    lower_widget_->setParent(lower_widget_container_);
    
    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    lower_widget_container_->setLayout(layout);

    layout->addWidget(lower_widget_);

    //the lower widget must not set the minimum width of the view, in a narrow view its right end is cut off
    lower_widget_container_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    lower_widget_container_->setVisible(true);
}

/**
 */
void ViewWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    updateConfigPanel(config_panel::UserAction::None);
}

/**
 * Applies the config panel state for the current view width and the given user action.
 */
void ViewWidget::updateConfigPanel(config_panel::UserAction action)
{
    traced_assert(right_widget_);

    const auto thresholds = config_panel::thresholdsFor(right_widget_->minimumSizeHint().width());
    const auto state      = config_panel::nextState(config_panel_state_, config_panel_view_width_, width(),
                                                    thresholds, action);

    config_panel_view_width_ = width();

    if (state == config_panel_state_)
        return;

    logdbg << "config panel " << config_panel::toString(state) << " view width " << width();

    config_panel_state_ = state;

    right_widget_->setVisible(config_panel::isShown(config_panel_state_));

    updateToolWidget();
}

/**
 */
QIcon ViewWidget::getIcon(const std::string& fn)
{
    return Utils::Files::IconProvider::getIcon(fn);
}

/**
 * Refresh the view depending on what is to do (e.g. reload, redraw or do nothing).
 * Will in all cases result in emitting the viewRefreshed() signal.
 * Returns false if the view is already busy with a running refresh.
 */
bool ViewWidget::refreshView()
{
    // view already busy => return
    // viewRefreshed() will be emitted after view has finished its current refresh
    if (getViewLoadStateWidget()->viewBusy())
        return false;

    if (getViewLoadStateWidget()->viewUpdateRequired())
    {
        // update required => run view update
        // viewRefreshed() either emitted from triggered redraw or reload
        getView()->updateView(); 
    }
    else if (getViewLoadStateWidget()->viewReloadRequired())
    {
        // reload required (most likely due to no data loaded yet) => reload view
        // viewRefreshed() emitted from triggered reload
        view_->viewManager().reload();
    }
#if 0
    else 
    {
        // fallback 1: be sceptical and reload in all other cases (will completely update the view)
        // viewRefreshed() emitted from triggered reload
        view_->viewManager().reload(); // fallback: just reload
    }
#else
    else
    {
        // fallback 2: trust the system and do nothing, but send the viewRefreshed() signal 
        // to inform listeners that view is fresh
        emit viewRefreshed();
    }
#endif

    return true;
}

/**
*/
void ViewWidget::clearData()
{
    //the render widget first: the Table View and the Geographic View draw again in the clear of the model
    render_widget_->clear();
    model_->clearData();

    updateComponents();
}

/**
*/
void ViewWidget::loadingStarted()
{
    loginf;

    //clear and update display, see clearData() for the order
    render_widget_->clear();
    model_->loadingStarted();

    redrawData(false, false);

    //propagate to subwidgets (note: order might be important)
    getViewConfigWidget()->loadingStarted();
    getViewLoadStateWidget()->loadingStarted();
    getViewToolWidget()->loadingStarted();
    getViewInfoWidget()->loadingStarted();
}

/**
*/
void ViewWidget::loadingDone()
{
    loginf;

    //propagate to subwidgets (note: order might be important)
    model_->loadingDone();
    getViewConfigWidget()->loadingDone();
    getViewLoadStateWidget()->loadingDone();
    getViewToolWidget()->loadingDone();
    getViewInfoWidget()->loadingDone();

    // The model may process the load asynchronously, in which case its
    // loadingDone() only launched the work. viewRefreshed() means "the view shows the
    // new data" to everything waiting on it - runtime commands and the UI tests read
    // the view right after it - so it must wait for the commit, not for the launch.
    if (model_->hasPendingProcessing())
    {
        refresh_pending_ = true;
        return;
    }

    emit viewRefreshed();
}

/**
 * The model finished its asynchronous processing: emit the refresh edge that
 * loadingDone() held back.
 */
void ViewWidget::modelProcessingFinished()
{
    if (!refresh_pending_)
        return;

    refresh_pending_ = false;

    emit viewRefreshed();
}

/**
*/
void ViewWidget::redrawStarted()
{
    //propagate to subwidgets (note: order might be important)
    getViewConfigWidget()->redrawStarted();
    getViewLoadStateWidget()->redrawStarted();
    getViewToolWidget()->redrawStarted();
    getViewInfoWidget()->redrawStarted();
}

/**
*/
void ViewWidget::redrawDone()
{
    //propagate to subwidgets (note: order might be important)
    getViewConfigWidget()->redrawDone();
    getViewLoadStateWidget()->redrawDone();
    getViewToolWidget()->redrawDone();
    getViewInfoWidget()->redrawDone();

    // The model may run the redraw's compute phase asynchronously, in which
    // case redrawData() only launched the work - the same edge loadingDone() holds
    // back: viewRefreshed() means "the view shows the result", so it must wait for
    // the commit (modelProcessingFinished), not for the launch.
    if (model_->hasPendingProcessing())
    {
        refresh_pending_ = true;
        return;
    }

    emit viewRefreshed();
}

/**
*/
void ViewWidget::appModeSwitch(AppMode app_mode)
{
    //propagate to subwidgets (note: order might be important)
    getViewConfigWidget()->appModeSwitch(app_mode);
    getViewLoadStateWidget()->appModeSwitch(app_mode);
    getViewToolWidget()->appModeSwitch(app_mode);
    getViewInfoWidget()->appModeSwitch(app_mode);
}

/**
*/
void ViewWidget::configChanged()
{
    //propagate to subwidgets (note: order might be important)
    getViewConfigWidget()->configChanged();
    getViewToolWidget()->configChanged();
    getViewInfoWidget()->configChanged();
}

/**
 * Updates the tool widget's items.
 */
void ViewWidget::updateToolWidget()
{
    if (!tool_widget_)
        return;

    getViewToolWidget()->updateItems();
}

/**
 * Updates the widget load state.
 */
void ViewWidget::updateLoadState()
{
    if (!state_widget_)
        return;

    getViewLoadStateWidget()->updateState();
}

/**
 * Updates the info widget.
 */
void ViewWidget::updateInfoWidget()
{
    if (!info_widget_)
        return;

    getViewInfoWidget()->updateInfos();
}

/**
 * Updates the view widget's individual components.
*/
void ViewWidget::updateComponents()
{
    updateToolWidget();
    updateLoadState();
    updateInfoWidget();
}

/**
 * Returns a view-specific loaded state message.
*/
std::string ViewWidget::loadedMessage() const
{
    traced_assert(isInit());

    return loadedMessage_impl();
}

/**
 * Returns view-specific information as json struct.
 */
nlohmann::json ViewWidget::viewInfoJSON() const
{
    nlohmann::json info;

    //add component information
    info[ "data"       ] = dataInfoJSON();
    info[ "config"     ] = getViewConfigWidget()->viewInfoJSON();
    info[ "load_state" ] = getViewLoadStateWidget()->viewInfoJSON();
    info[ "toolbar"    ] = getViewToolWidget()->viewInfoJSON();
    info[ "presets"    ] = getViewPresetWidget() ? getViewPresetWidget()->viewInfoJSON() : nlohmann::json();
    info[ "info"       ] = getViewInfoWidget()->viewInfoJSON();

    info[ "config_panel" ] = config_panel::toString(config_panel_state_);

    //add view-specific widget information
    viewInfoJSON_impl(info);

    return info;
}

/**
 * Information on the data area: the data state, the loaded buffers, then the view specific information
 * of the model and the render widget.
 */
nlohmann::json ViewWidget::dataInfoJSON() const
{
    nlohmann::json info;

    //add general information
    info[ "has_data"            ] = hasData();
    info[ "has_annotations"     ] = hasAnnotations();
    info[ "drawn"               ] = isDrawn();
    info[ "drawn_content"       ] = isContentDrawn();
    info[ "has_visible_content" ] = hasVisibleContent();

    const auto& buffers = model_->viewData();

    info[ "num_buffers" ] = buffers.size();

    nlohmann::json buffer_infos = nlohmann::json::array();
    for (const auto& dbc_data : buffers)
    {
        nlohmann::json buffer_info;
        buffer_info[ "dbcontent" ] = dbc_data.first;
        buffer_info[ "size"      ] = dbc_data.second->size();

        buffer_infos.push_back(buffer_info);
    }

    info[ "buffers" ] = buffer_infos;

    //add view-specific information
    if (data_info_)
    {
        data_info_(info);
    }
    else
    {
        model_->viewInfoJSON(info);
        render_widget_->viewInfoJSON(info);
    }

    return info;
}

/**
 * Checks if the required read set of the view is available in the loaded data.
 */
bool ViewWidget::isVariableSetLoaded() const
{
    const auto& view_data = model().viewData();
    if (view_data.empty())
        return false;

    for (const auto& dbcontent_data : view_data)
    {
        auto var_set = view_->getSet(dbcontent_data.first);
        for (auto var : var_set.getSet())
        {
            if (!dbcontent_data.second->hasAnyPropertyNamed(var->name()))
                return false;
        }
    }

    return true;
}

/**
 */
void ViewWidget::updateFromSource(const DBContentDataSet& source,
                                  const std::vector<std::string>& names, bool reset, bool last)
{
    model_->updateFromSource(source, names, reset, last);
}

/**
 * A recompute runs in the model, which reports its new display data with a signal, at once or after an
 * asynchronous computation.
 */
void ViewWidget::redrawData(bool recompute, bool notify)
{
    loginf << "recompute " << recompute << " notify " << notify;

    //the model runs one recompute when its outstanding work has finished
    if (recompute && model_->hasPendingProcessing())
    {
        model_->recompute();
        return;
    }

    if (notify)
    {
        redrawStarted();
        QApplication::processEvents(); //process any ui reactions on this signal before ui is blocked by redraw
    }

    {
        QScopedValueRollback<bool> redrawing(redrawing_, true);

        if (recompute)
            model_->recompute();
        else
            update_display_();
    }

    if (notify)
    {
        redrawDone();
    }

    //signal display changed to whom it may concern
    emit displayChangedSignal();
}

/**
 */
void ViewWidget::displayDataChanged()
{
    update_display_();

    renderDisplayChanged();
}

/**
 */
void ViewWidget::renderDisplayChanged()
{
    if (!redrawing_)
        emit displayChangedSignal();
}

/**
 */
void ViewWidget::liveReload()
{
    model_->liveReload();
}

/**
 */
void ViewWidget::beginDataReload()
{
    model_->beginDataReload();
}

/**
 */
void ViewWidget::attachLayerPanel(ViewLayerTreeModel* layer_model)
{
    model_->attachLayerPanel(layer_model);
}

/**
 * Checks if the view stores any data (mirrored from the current source).
 */
bool ViewWidget::hasData() const
{
    return model().hasData();
}

/**
 */
bool ViewWidget::hasAnnotations() const
{
    return model().hasAnnotation();
}

/**
 * Checks if the view has any content (data or annotations).
 */
bool ViewWidget::hasContent() const
{
    return hasData() || hasAnnotations();
}

/**
 * Checks if the view has any content (data or annotations) which is currently drawn.
 */
bool ViewWidget::hasVisibleContent() const
{
    return hasContent() && isContentDrawn();
}

/**
 * Checks if the display has been drawn.
 */
bool ViewWidget::isDrawn() const
{
    return renderWidget().isDrawn();
}

/**
 * Checks if any content has been drawn.
 */
bool ViewWidget::isContentDrawn() const
{
    return renderWidget().hasContent();
}

/**
 */
bool ViewWidget::hasPendingProcessing() const
{
    return model().hasPendingProcessing();
}

/**
 */
bool ViewWidget::hasScreenshotContent() const
{
    return hasContent() && isDrawn();
}

/**
 * Renders the contents of the render widget.
 * Notice: does not work with OpenGL widgets, e.g. the OSG view => override as needed.
 */
QImage ViewWidget::renderData()
{
    QImage img(render_widget_->size(), QImage::Format_ARGB32);
    QPainter painter(&img);

    //the widget render, ViewRenderWidget::render() renders the display data
    static_cast<QWidget*>(render_widget_)->render(&painter);

    return img;
}

/**
*/
QImage ViewWidget::renderContents()
{
    traced_assert(render_widget_);
    QImage data_img = renderData();

    QImage img(this->size(), QImage::Format_ARGB32);
    QPainter painter(&img);

    render(&painter);

    auto p0 = this->mapToGlobal(QPoint(0,0));
    auto p1 = render_widget_->mapToGlobal(QPoint(0,0));

    painter.drawImage(p1 - p0, data_img);

    return img;
}

/**
 * Running the 'uiget' rtcommand on a view widget will yield view specific json information.
 */
boost::optional<QString> ViewWidget::uiGet(const QString& what) const
{
    traced_assert(view_);

    std::string view_info = view_->viewInfoJSON().dump();
    return QString::fromStdString(view_info);
}

/**
 * Running the 'uiget' rtcommand on a view widget will yield view specific json information.
 */
nlohmann::json ViewWidget::uiGetJSON(const QString& what) const
{
    traced_assert(view_);
    return view_->viewInfoJSON();
}

/**
 * Running the 'uirefresh' rtcommand will refresh the view depending on its load state.
 */
void ViewWidget::uiRefresh()
{
    //just refresh the view
    refreshView();
}

/**
 */
void ViewWidget::databaseOpened()
{
    if (model_)
        model_->databaseOpened();

    //the config widget shows the stored config, with the availability in this database (e.g. Report Variables)
    if (config_widget_)
        config_widget_->configChanged();
}

/**
 */
void ViewWidget::databaseClosed()
{
    if (model_)
        model_->databaseClosed();

    if (config_widget_)
        config_widget_->configChanged();
}
