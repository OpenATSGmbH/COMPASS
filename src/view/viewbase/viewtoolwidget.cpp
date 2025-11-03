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

#include "viewtoolwidget.h"
#include "viewtoolswitcher.h"
#include "viewwidget.h"
#include "view.h"
#include "viewscreenshotdialog.h"
#include "compass.h"
#include "timeconv.h"

#include "ui_test_common.h"

#include <QShortcut>
#include <QToolButton>
#include <QApplication>
#include <QFileDialog>

#include "traced_assert.h"

/**
 */
ViewToolWidget::ViewToolWidget(ViewWidget* view_widget, 
                               ViewToolSwitcher* tool_switcher, 
                               QWidget* parent)
:   QToolBar      (parent)
,   view_widget_  (view_widget)
,   tool_switcher_(tool_switcher)
{
    traced_assert(view_widget_);
    traced_assert(tool_switcher_);

    connect(tool_switcher_, &ViewToolSwitcher::toolChanged, this, &ViewToolWidget::toolSwitched);

    setObjectName("toolbar");
}

/**
*/
void ViewToolWidget::addAction(int id, QAction* action, bool is_tool)
{
    if (actions_.find(id) != actions_.end())
        throw std::runtime_error("ViewToolWidget::addAction: duplicate id");

    Action a;
    a.action  = action;
    a.is_tool = is_tool;

    actions_[ id ] = a;

    QToolBar::addAction(action);
}

/**
 * Adds the tool of the given id to the toolbar.
 * Note: The tool needs to be added to the ViewToolSwitcher beforehand.
 */
void ViewToolWidget::addTool(int id, const UpdateCallback& cb_update)
{
    if (actions_.find(id) != actions_.end())
        throw std::runtime_error("ViewToolWidget::addTool: duplicate id");

    if (!tool_switcher_->hasTool(id))
        throw std::runtime_error("ViewToolWidget::addTool: unknown tool id");

    auto tool = tool_switcher_->getTool(id);

    bool isActiveTool = tool_switcher_->currentTool() == id;
        
    QAction* action = new QAction;
    action->setCheckable(true);
    action->setChecked(isActiveTool);
    action->setText(tool->name);
    action->setIcon(tool->icon);

    auto cb = [ = ] (bool on) 
    { 
        if (on)
            tool_switcher_->setCurrentTool(id);
        else
            tool_switcher_->endCurrentTool(); 
    };

    connect(action, &QAction::toggled, cb);

    if (cb_update)
        update_callbacks_.push_back([=] () { cb_update(action); });

    addAction(id, action, true);
}

namespace 
{
    /**
     */
    QAction* generateCallbackAction(const QString& name,
                                    const QIcon& icon,
                                    const QKeySequence& key_combination)
    {
        QAction* action = new QAction;
    
        QString full_name = name;

        if (!key_combination.isEmpty())
        {
            full_name += " [" + key_combination.toString() + "]";

            action->setShortcut(key_combination);
            action->setShortcutContext(Qt::ShortcutContext::WindowShortcut); //@TODO: optimal context?
        }

        action->setText(full_name);
        action->setIcon(icon);

        UI_TEST_OBJ_NAME(action, name)

        return action;
    }
}

/**
 * Adds an action with callback to the toolbar.
 * Internally called version.
 */
QAction* ViewToolWidget::addActionCallback_internal(const QString& name,
                                                    const Callback& cb,
                                                    const UpdateCallback& cb_update,
                                                    const QIcon& icon,
                                                    const QKeySequence& key_combination)
{
    QAction* action = generateCallbackAction(name, icon, key_combination);
    
    connect(action, &QAction::triggered, cb);

    if (cb_update)
        update_callbacks_.push_back([=] () { cb_update(action); });

    return action;
}

/**
 * Adds an action with callback to the toolbar.
 */
void ViewToolWidget::addActionCallback(const QString& name,
                                       const Callback& cb,
                                       const UpdateCallback& cb_update,
                                       const QIcon& icon,
                                       const QKeySequence& key_combination)
{
    auto action = addActionCallback_internal(name, cb, cb_update, icon, key_combination);

    QToolBar::addAction(action);
}

/**
 * Adds an action with callback to the toolbar.
 * This version stores the action under an id which can be used to retrieve the action later on.
 */
void ViewToolWidget::addActionCallback(int id,
                                       const QString& name,
                                       const Callback& cb,
                                       const UpdateCallback& cb_update,
                                       const QIcon& icon,
                                       const QKeySequence& key_combination)
{
    if (actions_.find(id) != actions_.end())
        throw std::runtime_error("ViewToolWidget::addActionCallback: duplicate id");

    auto action = addActionCallback_internal(name, cb, cb_update, icon, key_combination);

    addAction(id, action, false);
}

/**
 * Adds a checkable action with callback to the toolbar.
 * Internally called version.
 */
QAction* ViewToolWidget::addActionCallback_internal(const QString& name,
                                                    const ToggleCallback& cb,
                                                    const UpdateCallback& cb_update,
                                                    const QIcon& icon,
                                                    const QKeySequence& key_combination,
                                                    bool checked)
{
    QAction* action = generateCallbackAction(name, icon, key_combination);
    action->setCheckable(true);
    action->setChecked(checked);
    
    connect(action, &QAction::toggled, cb);

    if (cb_update)
        update_callbacks_.push_back([=] () { cb_update(action); });

    return action;
}

/**
 * Adds a checkable action with callback to the toolbar.
 */
void ViewToolWidget::addActionCallback(const QString& name,
                                       const ToggleCallback& cb,
                                       const UpdateCallback& cb_update,
                                       const QIcon& icon,
                                       const QKeySequence& key_combination,
                                       bool checked)
{
    auto action = addActionCallback_internal(name, cb, cb_update, icon, key_combination, checked);

    QToolBar::addAction(action);
}

/**
 * Adds a spacer item to the toolbar.
 */
void ViewToolWidget::addSpacer()
{
    //add spacer widget.
    QWidget* w = new QWidget;
    w->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    auto action = addWidget(w);

    spacers_.insert(action);
}

/**
 * Adds a spacer item of a fixed size to the toolbar.
 */
void ViewToolWidget::addSpacer(int width)
{
    QWidget* w = new QWidget;
    w->setFixedSize(width, 1);

    addWidget(w);
}   

/**
 * Adds a checkable action with callback to the toolbar.
 * This version stores the action under an id which can be used to retrieve the action later on.
 */
void ViewToolWidget::addActionCallback(int id,
                                       const QString& name,
                                       const ToggleCallback& cb,
                                       const UpdateCallback& cb_update,
                                       const QIcon& icon,
                                       const QKeySequence& key_combination,
                                       bool checked)
{
    if (actions_.find(id) != actions_.end())
        throw std::runtime_error("ViewToolWidget::addActionCallback: duplicate id");

    auto action = addActionCallback_internal(name, cb, cb_update, icon, key_combination, checked);

    addAction(id, action, false);
}

/**
 * Adds a separator only if it makes sense (e.g. not adding a separator twice etc.).
*/
void ViewToolWidget::addSeparatorIfValid()
{
    if (!separatorValid())
        return;

    addSeparator();
}

/**
 * Adds a button for toggling the config widget, that is part of the ViewWidget.
*/
void ViewToolWidget::addScreenshotButton()
{
    //add spacer if no spacer yet
    if (spacers_.empty())
        addSpacer();

    //add separator if needed
    addSeparatorIfValid();

    auto screener_cb = [ = ] ()
    {
        ViewScreenshotDialog dlg(view_widget_->getView(), this);
        dlg.exec();
    };

    //add callback
    addActionCallback("Save Screenshot", screener_cb, {}, ViewWidget::getIcon("camera.png"), QKeySequence());
}

/**
 * Adds a button for toggling the config widget, that is part of the ViewWidget.
*/
void ViewToolWidget::addConfigWidgetToggle()
{
    //add spacer if no spacer yet
    if (spacers_.empty())
        addSpacer();

    //add separator if needed
    addSeparatorIfValid();

    //add toggle callback
    addActionCallback("Toggle Configuration Panel", [=] (bool on) { view_widget_->toggleConfigWidget(); }, {}, ViewWidget::getIcon("configuration.png"), Qt::Key_C, true);
}

/**
 * Checks if the given action is a spacer item.
 */
bool ViewToolWidget::actionIsSpacer(QAction* action) const
{
    if (spacers_.empty())
        return false;

    return (spacers_.find(action) != spacers_.end());
}

/**
 * Checks if adding a separator makes sense.
*/
bool ViewToolWidget::separatorValid() const
{
    //no items yet?
    if (actions().count() < 1)
        return false;

    auto last_action = actions().back();

    //last action already is separator?
    if (last_action->isSeparator())
        return false;

    //last action is a spacer item?
    if (actionIsSpacer(last_action))
        return false;

    //last item should be a normal item
    return true;
}

/**
 */
void ViewToolWidget::toolSwitched(int id, const QCursor& cursor)
{
    for (auto& t : actions_)
    {
        if (!t.second.is_tool)
            continue;

        t.second.action->blockSignals(true);
        t.second.action->setChecked(t.first == id);
        t.second.action->blockSignals(false);
    }
}

/**
 * Enable/disable the action with the given id.
*/
void ViewToolWidget::enableAction(int id, bool enable)
{
    auto it = actions_.find(id);
    if (it == actions_.end())
        return;

    it->second.action->setEnabled(enable);
}

/**
 * Invoke all added item update callbacks.
 */
void ViewToolWidget::updateItems()
{
    for (auto& cb : update_callbacks_)
        cb();
}

/**
 * React on loading start.
 */
void ViewToolWidget::loadingStarted()
{
    updateItems();
    setEnabled(false);
}

/**
 * React on loading end.
 */
void ViewToolWidget::loadingDone()
{
    updateItems();
    setEnabled(true);
}

/**
 * React on manual redraw start.
*/
void ViewToolWidget::redrawStarted()
{
    setEnabled(false);
}

/**
 * React on manual redraw end.
*/
void ViewToolWidget::redrawDone()
{
    setEnabled(true);
}

/**
 * React on app switch.
*/
void ViewToolWidget::appModeSwitch(AppMode app_mode)
{
    //update items as their state might depend on app mode
    updateItems();
}

/**
 * React on config changes.
*/
void ViewToolWidget::configChanged()
{
    //update items as their state might depend on config
    updateItems();
}

/**
 * Generates json view information.
 */
nlohmann::json ViewToolWidget::viewInfoJSON() const
{
    nlohmann::json info;

    //add general information
    nlohmann::json action_infos = nlohmann::json::array();

    for (auto a : actions())
    {
        if (a->isSeparator())
            continue;

        nlohmann::json action_info;
        action_info[ "name"      ] = a->text().toStdString();
        action_info[ "checkable "] = a->isCheckable();
        action_info[ "checked"   ] = a->isChecked();

        action_infos.push_back(action_info);
    }

    info[ "actions" ] = action_infos;

    //add view-specific information
    viewInfoJSON_impl(info);

    return info;
}
