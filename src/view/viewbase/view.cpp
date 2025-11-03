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

#include "view.h"
#include "logger.h"
#include "viewcontainer.h"
#include "viewmanager.h"
#include "viewwidget.h"
#include "viewabledataconfig.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "viewdatawidget.h"
#include "config.h"

#include <QVBoxLayout>
#include <QWidget>

#include "traced_assert.h"

#include <fstream>

#include <boost/date_time/posix_time/conversion.hpp>

unsigned int View::cnt_ = 0;

/**
@brief Constructor.
@param class_id Configurable class id.
@param instance_id Configurable instance id.
@param w ViewContainerWidget the view is embedded in, configurable parent.
 */
View::View(const std::string& class_id, 
           const std::string& instance_id, 
           ViewContainer* container,
           ViewManager& view_manager)
    : Configurable(class_id, instance_id, container),
      view_manager_(view_manager),
      widget_(nullptr),
      container_(container)
{
    logdbg;

    registerParameter("name", &name_, std::string());

    loginf << "name '" << name_ << "'";
    traced_assert(name_.size());

    creation_time_ = boost::posix_time::to_time_t(boost::posix_time::microsec_clock::local_time());

    central_widget_ = new QWidget();
    //central_widget_->setAutoFillBackground(true);

    connect(this, &View::selectionChangedSignal, &view_manager_, &ViewManager::selectionChangedSlot);

    connect(&view_manager_, &ViewManager::selectionChangedSignal, this, &View::selectionChangedSlot);
    connect(&view_manager_, &ViewManager::unshowViewPointSignal, this, &View::unshowViewPointSlot);
    connect(&view_manager_, &ViewManager::showViewPointSignal, this, &View::showViewPointSlot);
    connect(&view_manager_, &ViewManager::reloadStateChanged, this, &View::viewManagerReloadStateChanged);
    connect(&view_manager_, &ViewManager::automaticUpdatesChanged, this, &View::viewManagerAutoUpdatesChanged);
    connect(&view_manager_, &ViewManager::presetEdited, this, &View::presetEdited);

    //do not write view name to presets
    addJSONExportFilter(Configurable::JSONExportType::Preset, Configurable::JSONExportFilterType::ParamID, "name");
}

/**
@brief Destructor.

Just deleting a view is totally feasible and will remove the view from its ViewContainerWidget.
 */
View::~View()
{
    // unregister from manager
    if (view_manager_.isRegistered(this))
        view_manager_.unregisterView(this);

    // remove view from container widget first, then delete it
    //container_->removeView(this);
    delete central_widget_;
}

/**
@brief Inits the view.

Construct the widget and model here in derived views. Construct the widget first,
then the model, which will need the widget in it's constructor.
 */
bool View::init()
{
    logdbg;

    traced_assert(!init_);

    // register in manager
    view_manager_.registerView(this);

    // add view to container widget
    //container_->addView(this);

    app_mode_ = COMPASS::instance().appMode();

    // invoke derive class (will create subconfigurables, such as view widget)
    if (!init_impl())
        return false;
    
    auto w = getWidget();
    traced_assert(w);

    //init view widget
    w->init();

    init_ = true;

    return true;
}

/**
*/
void View::databaseOpened()
{
}

/**
*/
void View::databaseClosed()
{
}

/**
@brief Returns the views name.
@return The views name.
 */
const std::string& View::getName() const { return name_; }

/**
*/
void View::enableInTabWidget(bool value)
{
    traced_assert(container_);
    container_->enableViewTab(getCentralWidget(), value);
}

/**
*/
void View::showInTabWidget()
{
    traced_assert(container_);
    container_->showView(getCentralWidget());
}

/**
@brief Returns a unique instance key.
@return Unique instance key.
 */
unsigned int View::getInstanceKey()
{
    ++cnt_;
    return cnt_;
}

/**
@brief Sets the view's widget.

Should only be called once to set the widget.
@param model The view's widget.
 */
void View::setWidget(ViewWidget* widget)
{
    if (widget_)
        throw std::runtime_error("View: setWidget: widget already set");

    widget_ = widget;
    if (widget_)
        constructWidget();
}

/**
@brief Embeds the view's widget into the central widget.
 */
void View::constructWidget()
{
    traced_assert(widget_);
    traced_assert(central_widget_);

    QVBoxLayout* central_vlayout = new QVBoxLayout();
    central_vlayout->setMargin(0);

    QHBoxLayout* central_hlayout = new QHBoxLayout();
    central_hlayout->setMargin(0);

    central_widget_->setLayout(central_vlayout);
    widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    central_hlayout->addWidget(widget_);
    central_vlayout->addLayout(central_hlayout);
}

/**
@brief Will delete the view and show the given error message.
@param err An error message.
 */
void View::viewShutdown(const std::string& err) { view_manager_.viewShutdown(this, err); }

/**
*/
void View::emitSelectionChange()
{
    //    traced_assert(!selection_change_emitted_);
    //    selection_change_emitted_ = true;

    emit selectionChangedSignal();
}

/**
*/
void View::selectionChangedSlot()
{
    //    if (selection_change_emitted_)
    //        selection_change_emitted_ = false;
    //    else // only update if not self-emitted
    updateSelection();
}

/**
*/
void View::loadingStarted()
{
    logdbg;

    //reload reverts any pending view updates
    issued_update_.reset();

    if (widget_)
        widget_->loadingStarted();
}

/**
*/
void View::loadedData(const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset)
{
    logdbg;

    if (widget_ && widget_->getViewDataWidget())
        widget_->getViewDataWidget()->updateData(data, requires_reset);
}

/**
*/
void View::loadingDone()
{
    logdbg;

    if (widget_)
        widget_->loadingDone();
}

/**
*/
void View::clearData()
{
    logdbg;

    if (widget_)
        widget_->clearData();
}

/**
*/
void View::appModeSwitch(AppMode app_mode_previous, AppMode app_mode_current)
{
    logdbg << "app_mode " << toString(app_mode_current)
           << " prev " << toString(app_mode_previous);

    app_mode_ = app_mode_current;

    if (widget_)
        widget_->appModeSwitch(app_mode_current);
}

/**
 * Reacts on configuration changes in the view triggered by at runtime reconfiguration of the configurable.
 */
void View::onConfigurationChanged(const std::vector<std::string>& changed_params)
{
    logdbg;

    //invoke derived for view-specific updates
    onConfigurationChanged_impl(changed_params);

    //inform the view widget
    traced_assert(widget_);
    widget_->configChanged();

    //signal view + config changes
    notifyRefreshNeeded();

    //every external configuration change potentially modifies the configuration
    notifyModifications();
}

/**
 * React on modifications in the configurable hierarchy.
 */
void View::onModified()
{
    //no preset? => no need to do anything
    if (!active_preset_.has_value())
        return;

#if 1
    //preset change already registered
    if (preset_changed_)
        return;

    preset_changed_ = true;
#else
    //get current config
    nlohmann::json cfg;
    generateJSON(cfg, Configurable::JSONExportType::Preset);

    //check if preset config has changed
    preset_changed_ = (cfg != active_preset_->view_config);

    if (preset_changed_)
    {
        auto d = nlohmann::json::diff(active_preset_->view_config, cfg);
        std::cout << d.dump(4) << std::endl;

        std::ofstream out("/home/mcphatty/test.json");
        out << cfg.dump(4);
    }
#endif

    //notify preset change
    emit presetChangedSignal();
}

/**
 * Renders the data widget into an image.
 */
QImage View::renderData() const
{
    traced_assert(widget_ && widget_->getViewDataWidget());  
    return widget_->getViewDataWidget()->renderData();
}

/**
 * Renders the complete view widget into an image.
 */
QImage View::renderView() const
{
    traced_assert(widget_);  
    return widget_->renderContents();
}

/**
 */
void View::setExporting(bool ok)
{
    traced_assert(widget_ && widget_->getViewDataWidget());  
    return widget_->getViewDataWidget()->isExporting(ok);
}

/**
 * Asks the data widget if it shows any data.
 */
bool View::hasScreenshotContent() const
{
    traced_assert(widget_ && widget_->getViewDataWidget());  
    return widget_->getViewDataWidget()->hasScreenshotContent();
}

/**
 * Reacts on a changed global load state in the view manager.
 */
void View::viewManagerReloadStateChanged()
{
    traced_assert(widget_);

    //inform dependent components of changed reload state
    widget_->updateComponents();
}

/**
 * Reacts on changed auto-update flags in the view manager.
 */
void View::viewManagerAutoUpdatesChanged()
{
    traced_assert(widget_);

    //run any needed automatic updates
    //runAutomaticUpdates();

    //inform dependent components of changed reload state
    widget_->updateComponents();
}

/**
 * Checks if THIS view needs a reload (this is different from the global reload flag in the view manager).
*/
bool View::reloadNeeded() const
{
    return updateNeeded() && (issued_update_.value() & VU_Reload);
}

/**
 * Check if this view needs a redraw.
 */
bool View::redrawNeeded() const
{
    return updateNeeded() && (issued_update_.value() & VU_Redraw);
}

/**
 * Check if this view needs an update (redraw, reload, etc).
 */
bool View::updateNeeded() const
{
    return issued_update_.has_value();
}

/**
 * Run any needed automatic updates.
 */
void View::runAutomaticUpdates()
{
    if (!issued_update_.has_value())
        return;

    if (issued_update_.value() & VU_Reload)
    {
        //reload issued => run automatic reload?
        if (view_manager_.automaticReloadEnabled())
            updateView();
        else if (refreshScreenOnNeededReload()) //view might want to redraw after an issued reload (e.g. to clear itself)
            updateView(VU_RecomputedRedraw);
    }
    else if (issued_update_.value() != 0)
    {
        //redraw issued => run automatic redraw?
        if (view_manager_.automaticRedrawEnabled())
            updateView();
    }
}

/**
 * Informs the view that the given view update is needed.
 * (Note: the update is cached until the next call to updateView())
 * @param add If true the update is added incrementally to any existing update, if false it will overwrite any existing update.
 */
void View::notifyViewUpdateNeeded(int flags, bool add)
{
    traced_assert(widget_);

    //live mode updates are handled immediately in their own way
    if (COMPASS::instance().appMode() == AppMode::LiveRunning)
    {
        if (flags & VU_Reload)
        {
            //in live mode a view handles its reload internally in its data widget
            widget_->getViewDataWidget()->liveReload();
        }
        else if (flags & VU_Redraw)
        {
            //just redraw
            updateView(VU_PureRedraw);
        }
        return;
    }

    //check if this is the first update issued
    bool update_components  = !updateNeeded();

    //add update to issued updates
    if (add && issued_update_.has_value())
        issued_update_.value() |= flags; //add incrementally to existing update
    else
        issued_update_ = flags;

    //check reload state after adding update
    bool reload_view_needed = reloadNeeded();

    //notify view if reload state has changed
    if (view_manager_.reloadNeeded() != reload_view_needed)
        view_manager_.notifyReloadStateChanged();

    //run any needed automatic updates
    runAutomaticUpdates();

    //inform dependent components if this is the first view update added
    if (update_components)
        widget_->updateComponents();
}

/**
 * Notifies the view that a view redraw is needed.
 * @param add If true the update is added incrementally to any existing update, if false it will overwrite any existing update.
 */
void View::notifyRedrawNeeded(bool add)
{
    notifyViewUpdateNeeded(VU_RecomputedRedraw, add);
}

/**
 * Notifies the view that a view reload is needed.
 * @param add If true the update is added incrementally to any existing update, if false it will overwrite any existing update.
 */
void View::notifyReloadNeeded(bool add)
{
    notifyViewUpdateNeeded(VU_Reload, add);
}

/**
 * Notifies the view that a "refresh" is needed and determines what kind of update is needed.
 * It is suggested to use this method in derived classes in order to notify the view about needed updates,
 * since it can automatically determine what is needed.
 * 
 * (Note: this function will always reset any pending updates, as it determines the new needed update state on-the-fly)
 */
void View::notifyRefreshNeeded()
{
    traced_assert(widget_);

    //reset already issued updates, since we can determine which updates are needed on-the-fly (either redraw or reload),
    //no matter what the last config state has looked like
    issued_update_.reset();

    //run a check if all needed variables are already loaded
    bool has_varset = widget_->isVariableSetLoaded();

    //std::cout << "has varset: " << has_varset << std::endl;

    //update depending on that information
    bool reload_needed = !has_varset;

    if (reload_needed)
        notifyReloadNeeded(false);
    else
        notifyRedrawNeeded(false);
}

/**
 * Updates the view by applying all needed updates.
 * This method can be used to keep the view up-to-date from the outside.
 */
void View::updateView()
{
    traced_assert(widget_);

    int update_flags = 0;

    //add my own issued updates
    if (issued_update_.has_value())
    {
        update_flags  |= issued_update_.value();
        issued_update_.reset();
    }

    //add view manager's needed updates
    if (view_manager_.reloadNeeded())
    {
        //viewmanager desires global reload
        update_flags |= VU_Reload;
    }

    //nothing to do?
    if (update_flags == 0)
        return; 

    bool components_updated = update_flags & VU_UpdateComponents;
    bool view_reloaded      = update_flags & VU_Reload;

    //update view
    updateView(update_flags);

    //inform dependent components now if not previously done by view update
    if (!components_updated && !view_reloaded)
        widget_->updateComponents();
}

/**
 * Updates the view according to the given update flags.
*/
void View::updateView(int flags)
{
    traced_assert(widget_);

    if (flags & VU_Reload) //reload = complete update
    {
        //start reload (will reset all cached updates)
        if (COMPASS::instance().dbOpened())
            COMPASS::instance().dbContentManager().load();
    }
    else //handle all other updates
    {
        if (flags & VU_Redraw)
            widget_->getViewDataWidget()->redrawData(flags & VU_Recompute, true);
        if (flags & VU_UpdateComponents)
            widget_->updateComponents();
    }
}

/**
*/
void View::updateComponents()
{
    traced_assert(widget_);

    widget_->updateComponents();
}

/**
 * Applies the given preset to the view.
 */
View::PresetError View::applyPreset(const ViewPresets::Preset& preset,
                                    std::vector<MissingKey>* missing_subconfig_keys,
                                    std::vector<MissingKey>* missing_param_keys,
                                    std::string* error_msg)
{
    auto version = COMPASS::instance().config().getString("version");

    if (!preset.app_version.empty() && preset.app_version != version)
    {
        loginf << "preset version mismatch, "
               << preset.app_version << ", app version " << version;
        //return PresetError::IncompatibleVersion;
    }

    bool assert_on_errors = !COMPASS::instance().isAppImage();

    auto result = reconfigure(preset.view_config, missing_subconfig_keys, missing_param_keys, assert_on_errors);

    if (error_msg)
        *error_msg = result.second;

    if (result.first == View::ReconfigureError::PreCheckFailed)
        return PresetError::IncompatibleContent;
    else if (result.first == View::ReconfigureError::ApplyFailed)
        return PresetError::ApplyFailed;
    else if (result.first == View::ReconfigureError::GeneralError)
        return PresetError::GeneralError;
    else if (result.first == View::ReconfigureError::UnknownError)
        return PresetError::UnknownError;

    //store preset and revert changes
    active_preset_  = preset;
    preset_changed_ = false;

    //notify about preset changes
    emit presetChangedSignal();

    return PresetError::NoError;
}

/**
 * Reacts on edited presets. Depending on the type of edit this might concern a view or not.
 */
void View::presetEdited(ViewPresets::EditAction ea)
{
    traced_assert(ea.valid());

    const auto& presets = view_manager_.viewPresets().presets();

    if (ea.mode == ViewPresets::EditMode::Add)
    {
        //if my view has added the preset => set active preset to new one
        if (ea.view == this)
        {
            const auto& preset = presets.at(ea.key);

            active_preset_  = preset;
            preset_changed_ = false;
        }
    }
    else if (ea.mode == ViewPresets::EditMode::Remove)
    {
        if (!active_preset_.has_value())
            return;

        //if my active preset has been removed => reset active preset
        if (active_preset_->key() == ea.key)
        {
            active_preset_.reset();
            preset_changed_ = false;
        }
    }
    else if (ea.mode == ViewPresets::EditMode::Update)
    {
        if (!active_preset_.has_value())
            return;
        
        //if my active preset is the updated preset => update local preset
        if (active_preset_->key() == ea.key)
        {
            const auto& preset = presets.at(ea.key);

            active_preset_  = preset;

            //if the config has been updated the preset is synced to the current view config => reset changes
            if (ea.config_changed)
                preset_changed_ = false;
        }
    }
    else if (ea.mode == ViewPresets::EditMode::Copy)
    {
        if (!active_preset_.has_value())
            return;

        //@TODO: actions needed?
    }
    else if (ea.mode == ViewPresets::EditMode::Rename)
    {
        if (!active_preset_.has_value())
            return;

        //@TODO: actions needed?
    }

    //notify about preset changes
    emit presetChangedSignal();
}

/**
 * Return the active preset.
 */
const ViewPresets::Preset* View::activePreset() const
{
    if (!active_preset_.has_value())
        return nullptr;

    return &active_preset_.value();
}

/**
 * Check if the currently stored preset has any modifications.
 */
bool View::presetChanged() const
{
    return (active_preset_.has_value() && preset_changed_);
}

/**
 * Returns view information to be displayed by the view info widget.
 */
ViewInfos View::viewInfos() const
{
    if (!widget_ || !widget_->isInit())
        return ViewInfos();

    ViewInfos vinfos;
    
    //get custom infos
    ViewInfos vinfos_custom = viewInfos_impl();

    //create standard view infos
    ViewInfos vinfos_standard;
    vinfos_standard.addSection("Loaded Data");

    const auto& null_cnt = widget_->getViewDataWidget()->nullCount();

    if (null_cnt.has_value())
        vinfos_standard.addInfo("info_null_values", "NULL values:", std::to_string(null_cnt.value()));

    //add custom infos, then standard ones
    if (vinfos_custom.numInfos() > 0)
        vinfos.addInfos(vinfos_custom);
    if (vinfos_standard.numInfos() > 0)
        vinfos.addInfos(vinfos_standard);

    return vinfos;
}

/**
 * Returns view-specific json information.
 */
nlohmann::json View::viewInfoJSON() const
{
    traced_assert(widget_);

    nlohmann::json info;

    //add basic information
    info[ "name" ] = getName();

    //add widget information
    info[ "ui"   ] = widget_->viewInfoJSON();

    //add derived view information
    nlohmann::json view_info;
    viewInfoJSON_impl(view_info);
    info[ "view" ] = view_info;

    return info;
}
