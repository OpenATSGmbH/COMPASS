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

#include "viewloadstatewidget.h"
#include "viewwidget.h"
#include "viewdatawidget.h"
#include "view.h"
#include "compass.h"
#include "viewmanager.h"
#include "dbcontentmanager.h"
#include "ui_test_common.h"

#include <QPushButton>
#include <QToolButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCheckBox>

/**
*/
ViewLoadStateWidget::ViewLoadStateWidget(ViewWidget* view_widget, QWidget* parent)
:   QWidget     (parent     )
,   view_widget_(view_widget)
{
    traced_assert(view_widget_);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setMargin(0);
    layout->setSpacing(0);
    layout->setContentsMargins(0, 0, 0, 0);
    setLayout(layout);

    content_widget_ = new QWidget;
    layout->addWidget(content_widget_);

    QVBoxLayout* content_layout = new QVBoxLayout;
    content_layout->setMargin(DefaultMargin);
    content_widget_->setLayout(content_layout);

    QHBoxLayout* layout_h = new QHBoxLayout;
    layout_h->setMargin(0);

    QHBoxLayout* layout_buttons = new QHBoxLayout;
    layout_buttons->setMargin(0);
    layout_buttons->setSpacing(0);

    content_layout->addLayout(layout_h);
    layout_h->addLayout(layout_buttons);
    
    QFont font_status;
    font_status.setItalic(true);

    refresh_button_ = new QPushButton;
    refresh_button_->setToolTip("Refresh view");
    refresh_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    refresh_button_->setIcon(Utils::Files::IconProvider::getIcon("refresh.png"));

    UI_TEST_OBJ_NAME(refresh_button_, "refresh")
    
    layout_buttons->addWidget(refresh_button_);

    status_label_ = new QLabel("");
    status_label_->setFont(font_status);
    layout_h->addWidget(status_label_);

    setState(State::NoData);

    connect(refresh_button_, &QPushButton::pressed, this, [=] () { this->updateData(); });
}

/**
*/
bool ViewLoadStateWidget::viewUpdateRequired() const
{
    traced_assert(view_widget_);

    //@TODO: if the view has no data the state should actually be State::NoData, but for safety query data widget nontheless...
    return (view_widget_->getViewDataWidget()->hasData() && (state_ == State::RedrawRequired || state_ == State::ReloadRequired));
}

/**
*/
bool ViewLoadStateWidget::viewReloadRequired() const
{
    traced_assert(view_widget_);

    return (!view_widget_->getViewDataWidget()->hasData() || state_ == State::NoData);
}

/**
*/
bool ViewLoadStateWidget::viewBusy() const
{
    return (state_ == State::Drawing || state_ == State::Loading);
}

/**
 * Triggered by the refresh button. Updates the data depending on the action needed by the current state.
*/
void ViewLoadStateWidget::updateData()
{
    traced_assert(view_widget_);

    //actual update mechanic implemented in the view widget for broader access to this functionality.
    //the view widget will then query the ViewLoadStateWidget for what to do.
    view_widget_->refreshView();
}

/**
 * Sets the widget to a certain state.
*/
void ViewLoadStateWidget::setState(State state)
{
    state_ = state;

    //get message and color from state
    auto   msg   = messageFromState(state);
    QColor color = colorFromState(state);

    //the view widget may provide us with a special load message
    if (state == State::Loaded && view_widget_)
    {
        auto load_message = view_widget_->loadedMessage();
        if (!load_message.empty())
            msg += " - " + load_message;
    }

    status_label_->setText(QString::fromStdString(msg));

    QPalette palette = status_label_->palette();
    palette.setColor(status_label_->foregroundRole(), color);
    status_label_->setPalette(palette);

    //update refresh button activity
    if (inactive_mode_ == InactivityMode::Show)
    {
        //only deactivate during loading
        refresh_button_->setEnabled(state != State::Loading);
    }
    else if (inactive_mode_ == InactivityMode::Disable)
    {
        //only set enabled if something is needed
        refresh_button_->setEnabled(state == State::NoData         ||
                                    state == State::RedrawRequired ||
                                    state == State::ReloadRequired);
    }
    else // InactivityMode::Hide
    {
        //only set enabled if something is needed
        refresh_button_->setEnabled(state == State::NoData         ||
                                    state == State::RedrawRequired ||
                                    state == State::ReloadRequired);

        //do not show contents if properly loaded
        content_widget_->setVisible(state != State::Loaded);
    }
}

/**
 * Updates the current state by querying the view widget for information.
*/
void ViewLoadStateWidget::updateState()
{
    //not needed in live running
    if (COMPASS::instance().appMode() == AppMode::LiveRunning)
        return;

    //return if we are in loading/drawing state (will be set back after those operations have finished)
    if (state_ == State::Loading ||
        state_ == State::Drawing)
        return;

    //ask widget for special states
    bool needs_reload =  COMPASS::instance().viewManager().reloadNeeded();
    bool needs_redraw =  view_widget_->getView()->updateNeeded();
    bool needs_data   = !view_widget_->getViewDataWidget()->hasData();

    //std::cout << "needs reload: " << needs_reload << std::endl;
    //std::cout << "needs redraw: " << needs_redraw << std::endl;
    //std::cout << "needs data:   " << needs_data   << std::endl;

    if (needs_data)
        setState(State::NoData);
    else if (needs_reload)
        setState(State::ReloadRequired);
    else if (needs_redraw)
        setState(State::RedrawRequired);
    else
        setState(State::Loaded); //everything up-to-date
}

/**
 * Reacts on starting to load from database.
*/
void ViewLoadStateWidget::loadingStarted()
{
    //not needed in live running
    if (COMPASS::instance().appMode() == AppMode::LiveRunning)
        return;

    setState(State::Loading);
}

/**
 * Reacts on finishing to load from database.
*/
void ViewLoadStateWidget::loadingDone()
{
    //not needed in live running
    if (COMPASS::instance().appMode() == AppMode::LiveRunning)
        return;

    setState(State::None); //reset state
    updateState();         //determine new state
}

/**
 * Reacts on starting to redraw data.
*/
void ViewLoadStateWidget::redrawStarted()
{
    //not needed in live running
    if (COMPASS::instance().appMode() == AppMode::LiveRunning)
        return;

    setState(State::Drawing);
}

/**
 * Reacts on finishing to redraw data.
*/
void ViewLoadStateWidget::redrawDone()
{
    //not needed in live running
    if (COMPASS::instance().appMode() == AppMode::LiveRunning)
        return;

    setState(State::None); //reset state
    updateState();         //determine new state
}

/**
 * Reacts on switching the application mode.
*/
void ViewLoadStateWidget::appModeSwitch(AppMode app_mode)
{
    //hide ui in live running mode (no manual updates)
    refresh_button_->setHidden(app_mode == AppMode::LiveRunning);
    status_label_->setHidden(app_mode == AppMode::LiveRunning);
}

/**
 * Returns a displayable message given a state.
*/
std::string ViewLoadStateWidget::messageFromState(State state)
{
    switch(state)
    {
        case State::NoData:
            return "No Data Loaded";
        case State::Loading:
            return "Refreshing...";
        case State::Drawing:
            return "Refreshing...";
        case State::None:
        case State::Loaded:
            return "Up-to-date";
        case State::ReloadRequired:
            return "Refresh Required";
        case State::RedrawRequired:
            return "Refresh Required";
    }
    return "";
}

/**
 * Returns a name string given a state.
*/
std::string ViewLoadStateWidget::stringFromState(State state)
{
    switch(state)
    {
        case State::NoData:
            return "NoData";
        case State::Loading:
            return "Loading";
        case State::Drawing:
            return "Drawing";
        case State::None:
            return "None";
        case State::Loaded:
            return "Loaded";
        case State::ReloadRequired:
            return "ReloadRequired";
        case State::RedrawRequired:
            return "RedrawRequired";
    }
    return "";
}

/**
 * Returns a button label given a state.
*/
std::string ViewLoadStateWidget::buttonTextFromState(State state)
{
    //label for redraw states
    if (state == State::RedrawRequired || 
        state == State::Drawing)
        return "Redraw";
    
    //reload label in all other cases
    return "Reload";
}

/**
 * Returns a label color given a state.
*/
QColor ViewLoadStateWidget::colorFromState(State state)
{
    switch (state)
    {
        case State::None:
        case State::Loading:
        case State::Drawing:
        case State::Loaded:
        case State::NoData:
            return Qt::black;
        case State::ReloadRequired:
        case State::RedrawRequired:
            return Qt::red;
    }
    return Qt::black;
}

/**
 * Generates json view information.
 */
nlohmann::json ViewLoadStateWidget::viewInfoJSON() const
{
    nlohmann::json info;

    //add general information
    info[ "state"                  ] = stringFromState(state_);
    info[ "refresh_button_visible" ] = refresh_button_->isVisible();

    //add view-specific information
    viewInfoJSON_impl(info);

    return info;
}
